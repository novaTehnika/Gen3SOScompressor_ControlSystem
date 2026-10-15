"""Sampling, requirement criteria and the parallel point runner for the
robustness study (study_robust.py).

UNCERTAIN lists the plant and sensor parameters whose values are guesses,
with the range each is varied over. THRESH holds every pass/fail limit;
`evaluate` applies them to the metrics of one run.

A point is one (plant parameter set, scenario, slave configuration, noise
seed). `run_points` runs points in a spawn-started pool and returns small
dicts, so thousands of runs stay cheap; `point_metrics` is the single-run
function (also used serially). Metrics are those of metrics.compute plus
`dip` and `maxErrLate`, computed from the traces inside the worker because
run_cases(traces=False) drops the traces before they can be read.
"""

import math
import multiprocessing as mp
import os
from dataclasses import dataclass, field

import numpy as np

import controllers as ctl
import scenarios
import sim
from params import Params
from slave import slave_config

# ---------------------------------------------------------------------------
# Uncertain parameters: name, low, high, log-uniform?, unit shown, reason.
# Fields not listed (mr, b, Fc, veps) act only in torque mode, which the slave
# does not use; A, R, T, Patm, kH are geometry or physical constants.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Uncertain:
    name: str          # Params field
    lo: float
    hi: float
    log: bool
    scale: float       # display value = SI value * scale
    unit: str
    why: str


UNCERTAIN = [
    Uncertain('K', 0.5e9, 2.5e9, True, 1e-9, 'GPa', 'water 2.2 GPa; hoses, seals and residual gas lower it'),
    Uncertain('Vl0', 250e-6, 350e-6, False, 1e6, 'mL', 'liquid in the chamber, hoses and fittings'),
    Uncertain('Vg0', 100e-6, 300e-6, False, 1e6, 'mL', 'gas at atmosphere in the chamber at the start'),
    Uncertain('tauDiss', 10.0, 100.0, True, 1.0, 's', 'gas dissolution time constant'),
    Uncertain('tauV', 0.005, 0.05, True, 1e3, 'ms', 'drive velocity loop lag'),
    Uncertain('noise', 1e3, 20e3, True, 1e-3, 'kPa', 'transducer plus ADC noise, rms per sample'),
    Uncertain('Qf', 0.5e-6, 2e-6, False, 1e6, 'mL/s', 'discharge flow, final value; used by discharge and valve'),
]
NAMES = [u.name for u in UNCERTAIN]
BY_NAME = {u.name: u for u in UNCERTAIN}

SCENARIOS = ('compress', 'stiffstep', 'discharge', 'valve')
VALVE_TOPEN = 0.3          # s, the valve case of the requirements
VALVE_T = 12.0             # s simulated for the valve case (Params default 8 s cuts recovery short at the slow corners)
DISCHARGE_T = 30.0

_CFG = slave_config()
THRESH = {
    'inband': _CFG['G_cfgPressureInBand'],            # MPa, status band of the slave
    'compress_overshoot': 0.5,                        # MPa; slavePressureMode: pMax <= 10.5
    'stiffstep_overshoot': 0.6,                       # MPa; slavePressureMode: pMax <= 12.6
    'ripple': _CFG['G_cfgPressureInBand'] / 5,        # MPa std over the last 2 s: "well under" the band
    'dip': 1.0,                                       # MPa; slavePressureMode: pMin > 11 for target 12 (open question)
    'stroke_mm': _CFG['G_cfgPosSoftLimitMax'] - 200.0,  # GVL soft limit minus slave-sim chamberContactPos (assumed)
}
DIP_LEVELS = (0.25, 0.5, 1.0)     # MPa, reported alongside THRESH['dip']

# requirement name -> (scenario, description)
REQUIREMENTS = {
    'compress.band': ('compress', 'in band at the end'),
    'compress.overshoot': ('compress', 'overshoot <= %(compress_overshoot)g MPa'),
    'compress.ripple': ('compress', 'late ripple <= %(ripple)g MPa'),
    'stiffstep.overshoot': ('stiffstep', 'overshoot <= %(stiffstep_overshoot)g MPa'),
    'stiffstep.band': ('stiffstep', 'in band at the end'),
    'discharge.dip': ('discharge', 'dip <= %(dip)g MPa'),
    'discharge.band': ('discharge', 'in band at the end'),
    'valve.dip': ('valve', 'dip <= %(dip)g MPa'),
    'valve.band': ('valve', 'in band at the end'),
    'stroke': (None, 'stroke <= %(stroke_mm)g mm, all scenarios'),
}


def describe(req, th=THRESH):
    return REQUIREMENTS[req][1] % th


# (scenario, metric) -> limit, the metrics ranked in the tornado and scatter plots
KEY_METRICS = {
    ('compress', 'overshoot'): THRESH['compress_overshoot'],
    ('compress', 'maxErrLate'): THRESH['inband'],
    ('compress', 'ripple'): THRESH['ripple'],
    ('compress', 'stroke_mm'): THRESH['stroke_mm'],
    ('stiffstep', 'overshoot'): THRESH['stiffstep_overshoot'],
    ('stiffstep', 'maxErrLate'): THRESH['inband'],
    ('discharge', 'dip'): THRESH['dip'],
    ('discharge', 'maxErrLate'): THRESH['inband'],
    ('valve', 'dip'): THRESH['dip'],
    ('valve', 'maxErrLate'): THRESH['inband'],
}


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------

def to_value(u, x):
    """Parameter value for unit-interval coordinate x."""
    if u.log:
        return float(math.exp(math.log(u.lo) + x * (math.log(u.hi) - math.log(u.lo))))
    return float(u.lo + x * (u.hi - u.lo))


def lhs_unit(n, d, seed):
    """n x d Latin hypercube in [0,1): one point in each of n strata per column,
    columns permuted independently."""
    rng = np.random.default_rng(seed)
    cut = (np.arange(n)[:, None] + rng.random((n, d))) / n
    for j in range(d):
        cut[:, j] = cut[rng.permutation(n), j]
    return cut


def lhs(n, seed=1, uncertain=UNCERTAIN):
    """List of n dicts {param: value}, Latin hypercube over `uncertain` (log
    parameters are stratified in log space)."""
    x = lhs_unit(n, len(uncertain), seed)
    return [{u.name: to_value(u, x[i, j]) for j, u in enumerate(uncertain)} for i in range(n)]


def nominal():
    """Each parameter at its Params default."""
    base = Params()
    return {u.name: getattr(base, u.name) for u in UNCERTAIN}


def base_params():
    """Params with the settling band tied to the slave's InBand and the valve case."""
    return Params(tol=THRESH['inband'] * 1e6, Topen=VALVE_TOPEN, valveT=VALVE_T)


def make_params(values):
    return base_params().with_(**values)


def make_scenario(name, p, ff_scale=1.0):
    """Scenario `name` carrying the sampled flow Qf (the discharge case takes it
    too, so one parameter sets both) and the feedforward scale."""
    if name == 'discharge':
        return scenarios.make(name, p, T=DISCHARGE_T, Qf=p.Qf, ff_scale=ff_scale)
    if name == 'valve':
        return scenarios.make(name, p, ff_scale=ff_scale)
    return scenarios.make(name, p)


# ---------------------------------------------------------------------------
# Metrics and requirements
# ---------------------------------------------------------------------------

def extras(r, scenario):
    """Metrics that need traces: `dip` (target before the event minus the lowest
    pressure after it, MPa) and `maxErrLate` (max |error| over the last 2 s)."""
    ie = r.t >= scenario.tEvent
    last = r.t > scenario.T - 2
    return {
        'dip': float(r.target[ie][0] - np.min(r.P[ie])) / 1e6,
        'maxErrLate': float(np.max(np.abs(r.target[last] - r.P[last]))) / 1e6,
    }


def finite_or(x, default):
    return default if x is None or not math.isfinite(x) else x


def point_metrics(point):
    """Run one point: (values, scenario name, ControllerSpec, seed) -> metrics
    dict. A run that blows up (non-finite state) returns {'diverged': True}
    with every metric NaN, which fails every criterion."""
    values, scen, spec, seed = point
    p = make_params(values)
    s = make_scenario(scen, p, spec.ff_scale)
    c = ctl.SlaveController(flow_ff=spec.flow_ff, **spec.overrides)
    try:
        with np.errstate(invalid='raise', over='raise', divide='raise'):
            r = sim.run(p, s, c, seed=seed)
        m = dict(r.metrics)
        m.update(extras(r, s))
        m['diverged'] = not all(math.isfinite(v) for k, v in m.items() if k != 'tSettle')
    except (FloatingPointError, OverflowError, ValueError, ZeroDivisionError):
        m = {k: math.nan for k in ('overshoot', 'tSettle', 'peakErr', 'rmsErr', 'ripple', 'uAct',
                                   'stroke_mm', 'dip', 'maxErrLate')}
        m['diverged'] = True
    return m


@dataclass(frozen=True)
class ControllerSpec:
    """Slave configuration of one study arm. overrides use SlaveController names."""
    name: str
    overrides: dict = field(default_factory=dict)
    flow_ff: bool = False
    ff_scale: float = 1.0


def _le(x, limit):
    """x <= limit, false for NaN."""
    return bool(x <= limit)


def evaluate(scen, m, th=THRESH):
    """{requirement: bool} for the requirements of scenario `scen` plus the
    stroke check, from the metrics dict m."""
    out = {}
    band = _le(m['maxErrLate'], th['inband']) and not math.isnan(m['tSettle'])
    if scen == 'compress':
        out['compress.band'] = band
        out['compress.overshoot'] = _le(m['overshoot'], th['compress_overshoot'])
        out['compress.ripple'] = _le(m['ripple'], th['ripple'])
    elif scen == 'stiffstep':
        out['stiffstep.overshoot'] = _le(m['overshoot'], th['stiffstep_overshoot'])
        out['stiffstep.band'] = band
    elif scen in ('discharge', 'valve'):
        out[f'{scen}.dip'] = _le(m['dip'], th['dip'])
        out[f'{scen}.band'] = band
    else:
        raise KeyError(scen)
    out['stroke@' + scen] = _le(m['stroke_mm'], th['stroke_mm'])
    return out


def evaluate_all(by_scen, th=THRESH):
    """Merge `evaluate` over {scenario: metrics}; the stroke requirement holds
    only if it holds in every scenario. Returns {requirement: bool, 'all': bool}."""
    out = {}
    stroke = True
    for scen, m in by_scen.items():
        for k, v in evaluate(scen, m, th).items():
            if k.startswith('stroke@'):
                stroke = stroke and v
            else:
                out[k] = v
    out['stroke'] = stroke
    out['all'] = all(out.values())
    return out


# ---------------------------------------------------------------------------
# Parallel runner
# ---------------------------------------------------------------------------

def run_points(points, workers=None, pool=None):
    """Metrics dicts of `points` in order. workers=1 runs in this process;
    otherwise a spawn pool of `workers` (default one per CPU), or `pool` if
    given (so several calls share worker start-up)."""
    points = list(points)
    if pool is not None:
        return pool.map(point_metrics, points, chunksize=_chunk(len(points), pool))
    workers = min(workers or os.cpu_count() or 1, len(points))
    if workers <= 1:
        return [point_metrics(pt) for pt in points]
    with make_pool(workers) as pl:
        return pl.map(point_metrics, points, chunksize=_chunk(len(points), pl))


def make_pool(workers):
    pl = mp.get_context('spawn').Pool(workers)
    pl._robust_workers = workers
    return pl


def _chunk(n, pool):
    return max(1, n // (4 * getattr(pool, '_robust_workers', 4)))
