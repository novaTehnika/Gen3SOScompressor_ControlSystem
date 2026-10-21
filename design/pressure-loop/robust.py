"""
robust.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module holds the definitions and helper functions for the
robustness study, study_robust.py: the uncertain parameters and their
ranges, the requirement limits, the sampler, the evaluation of the
requirements, and a parallel runner.

UNCERTAIN lists the plant and sensor parameters whose values are not yet
measured, with the range each is varied over and the reason for it.
THRESH holds every pass/fail limit, and evaluate() applies the limits to
the metrics of one run.

A point is one combination of plant parameter values, scenario, slave
configuration and noise seed. point_metrics() runs one point and returns
its metrics. run_points() runs many points in a pool of worker processes
and returns only the metrics dicts, so thousands of runs stay cheap. The
metrics are those of metrics.compute() plus two that need the traces,
dip and maxErrLate, which are computed inside the worker before the
traces are discarded.

FILE DEPENDENCY:
./
    controllers.py
    params.py
    scenarios.py
    sim.py
    slave.py
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


# %% ===== UNCERTAIN PARAMETERS ==========================================
@dataclass(frozen=True)
class Uncertain:
    """One uncertain parameter and the range it is varied over."""

    name: str       # Params field
    lo: float       # lowest value, SI units
    hi: float       # highest value, SI units
    log: bool       # True to sample uniformly in log(value)
    scale: float    # displayed value = SI value * scale
    unit: str       # unit of the displayed value
    why: str        # reason for the range


# The parameters not listed here are fixed. mr, b, Fc and veps act only
# in torque mode, which the slave does not use. A, R, T, Patm and kH are
# geometry or physical constants.
UNCERTAIN = [
    Uncertain('K', 0.5e9, 2.5e9, True, 1e-9, 'GPa',
              'water 2.2 GPa; hoses, seals and residual gas lower it'),
    Uncertain('Vl0', 250e-6, 350e-6, False, 1e6, 'mL',
              'liquid in the chamber, hoses and fittings'),
    Uncertain('Vg0', 100e-6, 300e-6, False, 1e6, 'mL',
              'gas at atmosphere in the chamber at the start'),
    Uncertain('tauDiss', 10.0, 100.0, True, 1.0, 's',
              'gas dissolution time constant'),
    Uncertain('tauV', 0.005, 0.05, True, 1e3, 'ms',
              'drive velocity loop lag'),
    Uncertain('noise', 1e3, 20e3, True, 1e-3, 'kPa',
              'transducer plus ADC noise, rms per sample'),
    Uncertain('Qf', 0.5e-6, 2e-6, False, 1e6, 'mL/s',
              'discharge flow, final value; used by discharge and valve'),
]
NAMES = [u.name for u in UNCERTAIN]
BY_NAME = {u.name: u for u in UNCERTAIN}


# %% ===== SCENARIOS AND REQUIREMENT LIMITS ==============================
SCENARIOS = ('compress', 'stiffstep', 'discharge', 'valve')
VALVE_TOPEN = 0.3       # [s] valve opening time in the requirements
# [s] Simulated duration of the valve case. The Params default of 8 s
# cuts the recovery short at the slowest parameter combinations.
VALVE_T = 12.0
DISCHARGE_T = 30.0      # [s] simulated duration of the discharge case

_CFG = slave_config()
THRESH = {
    # [MPa] Status band of the slave (G_cfgPressureInBand)
    'inband': _CFG['G_cfgPressureInBand'],
    # [MPa] Overshoot limits from tests/slavePressureMode.m, which
    # requires pMax <= 10.5 MPa (compress) and pMax <= 12.6 MPa
    # (stiffstep)
    'compress_overshoot': 0.5,
    'stiffstep_overshoot': 0.6,
    # [MPa] Standard deviation of P over the last 2 s, set well under
    # the status band
    'ripple': _CFG['G_cfgPressureInBand'] / 5,
    # [MPa] Dip limit, a placeholder; tests/slavePressureMode.m requires
    # pMin > 11 MPa for a 12 MPa target
    'dip': 1.0,
    # [mm] Stroke limit: the GVL soft limit minus the chamber contact
    # position of 200 mm assumed in the slave simulator
    'stroke_mm': _CFG['G_cfgPosSoftLimitMax'] - 200.0,
}
DIP_LEVELS = (0.25, 0.5, 1.0)   # [MPa] other dip limits reported

# Requirement name -> (scenario, description)
REQUIREMENTS = {
    'compress.band': ('compress', 'in band at the end'),
    'compress.overshoot': ('compress',
                           'overshoot <= %(compress_overshoot)g MPa'),
    'compress.ripple': ('compress', 'late ripple <= %(ripple)g MPa'),
    'stiffstep.overshoot': ('stiffstep',
                            'overshoot <= %(stiffstep_overshoot)g MPa'),
    'stiffstep.band': ('stiffstep', 'in band at the end'),
    'discharge.dip': ('discharge', 'dip <= %(dip)g MPa'),
    'discharge.band': ('discharge', 'in band at the end'),
    'valve.dip': ('valve', 'dip <= %(dip)g MPa'),
    'valve.band': ('valve', 'in band at the end'),
    'stroke': (None, 'stroke <= %(stroke_mm)g mm, all scenarios'),
}


def describe(req, th=THRESH):
    """Return the description of requirement req with its limit."""
    return REQUIREMENTS[req][1] % th


# (scenario, metric) -> limit, for the metrics ranked in the tornado
# and scatter plots
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


# %% ===== SAMPLING ======================================================
def to_value(u, x):
    """Map a coordinate x in [0, 1) to a value of parameter u, linearly
    or, for a log parameter, linearly in log(value)."""
    if u.log:
        log_value = math.log(u.lo) + x * (math.log(u.hi) - math.log(u.lo))
        return float(math.exp(log_value))
    return float(u.lo + x * (u.hi - u.lo))


def lhs_unit(n, d, seed):
    """Return an n x d Latin hypercube sample in [0, 1).

    Each column holds one point in each of the n equal strata, at a
    random position within the stratum. The columns are permuted
    independently, so the strata are paired at random across
    parameters.
    """
    rng = np.random.default_rng(seed)
    cut = (np.arange(n)[:, None] + rng.random((n, d))) / n
    for j in range(d):
        cut[:, j] = cut[rng.permutation(n), j]
    return cut


def lhs(n, seed=1, uncertain=UNCERTAIN):
    """Return n dicts {parameter: value} from a Latin hypercube over
    uncertain. Log parameters are stratified in log space."""
    x = lhs_unit(n, len(uncertain), seed)
    return [{u.name: to_value(u, x[i, j]) for j, u in enumerate(uncertain)}
            for i in range(n)]


def nominal():
    """Return each uncertain parameter at its Params default."""
    base = Params()
    return {u.name: getattr(base, u.name) for u in UNCERTAIN}


def base_params():
    """Return Params with the settling band set to the slave's status
    band and the valve case of the requirements."""
    return Params(tol=THRESH['inband'] * 1e6,           # [MPa -> Pa]
                  Topen=VALVE_TOPEN, valveT=VALVE_T)


def make_params(values):
    """Return base_params() with the sampled values applied."""
    return base_params().with_(**values)


def make_scenario(name, par, ff_scale=1.0):
    """Return scenario name for Params par.

    The discharge case takes the sampled flow Qf too, so one parameter
    sets the flow of both outflow cases. ff_scale sets the error in the
    feedforward flow.
    """
    if name == 'discharge':
        return scenarios.make(name, par, T=DISCHARGE_T, Qf=par.Qf,
                              ff_scale=ff_scale)
    if name == 'valve':
        return scenarios.make(name, par, ff_scale=ff_scale)
    return scenarios.make(name, par)


# %% ===== METRICS AND REQUIREMENTS ======================================
def extras(r, scenario):
    """Return the metrics that need the traces.

      dip         [MPa] Target before the event minus the lowest
                  pressure after it.
      maxErrLate  [MPa] Largest |target - P| over the last 2 s.
    """
    after_event = r.t >= scenario.tEvent
    last_2s = r.t > scenario.T - 2
    dip = r.target[after_event][0] - np.min(r.P[after_event])   # [Pa]
    err_late = np.abs(r.target[last_2s] - r.P[last_2s])         # [Pa]
    return {
        'dip': float(dip) / 1e6,                        # [Pa -> MPa]
        'maxErrLate': float(np.max(err_late)) / 1e6,
    }


def finite_or(x, default):
    """Return x, or default if x is None or not finite."""
    return default if x is None or not math.isfinite(x) else x


@dataclass(frozen=True)
class ControllerSpec:
    """Slave configuration of one study arm.

    overrides use the SlaveController short names, in GVL units.
    """

    name: str
    overrides: dict = field(default_factory=dict)
    flow_ff: bool = False
    ff_scale: float = 1.0


def point_metrics(point):
    """Run one point and return its metrics dict.

    point is (values, scenario name, ControllerSpec, seed). A run whose
    state becomes non-finite returns {'diverged': True} with every
    metric NaN, which fails every requirement.
    """
    values, scen, spec, seed = point
    par = make_params(values)
    s = make_scenario(scen, par, spec.ff_scale)
    c = ctl.SlaveController(flow_ff=spec.flow_ff, **spec.overrides)
    try:
        with np.errstate(invalid='raise', over='raise', divide='raise'):
            r = sim.run(par, s, c, seed=seed)
        m = dict(r.metrics)
        m.update(extras(r, s))
        m['diverged'] = not all(math.isfinite(v) for k, v in m.items()
                                if k != 'tSettle')
    except (FloatingPointError, OverflowError, ValueError,
            ZeroDivisionError):
        m = {k: math.nan for k in ('overshoot', 'tSettle', 'peakErr',
                                   'rmsErr', 'ripple', 'uAct',
                                   'stroke_mm', 'dip', 'maxErrLate')}
        m['diverged'] = True
    return m


def _le(x, limit):
    """Return x <= limit; False when x is NaN."""
    return bool(x <= limit)


def evaluate(scen, m, th=THRESH):
    """Return {requirement: passed} for the requirements of scenario scen
    and its stroke check, from metrics dict m.

    "In band at the end" requires the error to stay within the status
    band over the last 2 s and the run to have settled.
    """
    out = {}
    band = _le(m['maxErrLate'], th['inband']) and not math.isnan(
        m['tSettle'])
    if scen == 'compress':
        out['compress.band'] = band
        out['compress.overshoot'] = _le(m['overshoot'],
                                        th['compress_overshoot'])
        out['compress.ripple'] = _le(m['ripple'], th['ripple'])
    elif scen == 'stiffstep':
        out['stiffstep.overshoot'] = _le(m['overshoot'],
                                         th['stiffstep_overshoot'])
        out['stiffstep.band'] = band
    elif scen in ('discharge', 'valve'):
        out[f'{scen}.dip'] = _le(m['dip'], th['dip'])
        out[f'{scen}.band'] = band
    else:
        raise KeyError(scen)
    out['stroke@' + scen] = _le(m['stroke_mm'], th['stroke_mm'])
    return out


def evaluate_all(by_scen, th=THRESH):
    """Combine evaluate() over {scenario: metrics}.

    The stroke requirement passes only if it passes in every scenario.
    Returns {requirement: passed, 'all': passed}.
    """
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


# %% ===== PARALLEL RUNNER ===============================================
def run_points(points, workers=None, pool=None):
    """Return the metrics dicts of points, in order.

    workers=1 runs the points in this process. Otherwise the points run
    in pool, if given, so that several calls share the worker start-up,
    or in a new pool of workers processes (default one per CPU).
    """
    points = list(points)
    if pool is not None:
        return pool.map(point_metrics, points,
                        chunksize=_chunk(len(points), pool))
    workers = min(workers or os.cpu_count() or 1, len(points))
    if workers <= 1:
        return [point_metrics(pt) for pt in points]
    with make_pool(workers) as pl:
        return pl.map(point_metrics, points,
                      chunksize=_chunk(len(points), pl))


def make_pool(workers):
    """Return a spawn-started pool of workers processes."""
    pl = mp.get_context('spawn').Pool(workers)
    pl._robust_workers = workers
    return pl


def _chunk(n, pool):
    """Return the number of points per task: about four tasks per
    worker."""
    return max(1, n // (4 * getattr(pool, '_robust_workers', 4)))
