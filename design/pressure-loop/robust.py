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
ranges, the requirements, the sampler, the evaluation of the
requirements, and a parallel runner.

UNCERTAIN lists the plant, outlet and sensor parameters whose values are
not yet measured, with the range each is varied over and the reason for
it. REQUIREMENTS lists the pass/fail checks and evaluate() applies them
to the metrics of one run. The requirements are the machine's: the
pressure stays below the mechanical ceiling, the solution never degasses
once the target is reached, the loop keeps control to the end of the
delivery, the compression ends in band, and the stroke fits.

A point is one combination of plant parameter values, scenario, slave
configuration and noise seed. point_metrics() runs one point and returns
its metrics. run_points() runs many points in a pool of worker processes
and returns only the metrics dicts, so thousands of runs stay cheap.

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
# geometry or physical constants. The oxygen ratio is a study variable,
# not an uncertainty.
# The ranges are the measured values of 2026-10-26 with their spread
# (bench/characterisation, bench/friction, bench/pressure), except the
# dissolution time constant and the supply pressure, which are not
# measured.
UNCERTAIN = [
    Uncertain('K', 0.6e9, 1.0e9, True, 1e-9, 'GPa',
              'measured 0.72, segments 0.67 to 0.78, gas-corrected 0.81'),
    Uncertain('Vl0', 320e-6, 360e-6, False, 1e6, 'mL',
              'fill at the 280 mm start, 343 mL from the geometry'),
    Uncertain('V_passages', 52e-6, 62e-6, False, 1e6, 'mL',
              'cylinder-head passages, measured 57 mL with the hose shut'),
    Uncertain('C_hose', 1.2e-13, 3.6e-13, True, 1e12, 'mL/MPa',
              'hose compliance, 0.24 mL/MPa by difference of two K runs'),
    Uncertain('tauDiss', 10.0, 100.0, True, 1.0, 's',
              'gas dissolution time constant'),
    Uncertain('tauV', 0.010, 0.050, True, 1e3, 'ms',
              'drive lag: 18.2 ms at the motor, 10 to 50 ms for the '
              'piston side, which is not resolved'),
    Uncertain('noise', 1.5e3, 5e3, True, 1e-3, 'kPa',
              'transducer plus ADC noise, 1.8 to 4.6 kPa rms per sample'),
    Uncertain('Cd', 0.8, 0.9, False, 1.0, '',
              'orifice discharge coefficient, measured 0.84 to 0.87'),
    Uncertain('P_supply', 0.52e6, 1.38e6, False, 1e-6, 'MPa',
              'oxygen supply, 75 to 200 psi with the cylinder charge'),
]
NAMES = [u.name for u in UNCERTAIN]
BY_NAME = {u.name: u for u in UNCERTAIN}


# %% ===== SCENARIOS AND REQUIREMENTS ====================================
SCENARIOS = ('compress', 'deliver')

_CFG = slave_config()
_PAR = Params()
THRESH = {
    # [MPa] Status band of the slave (G_cfgPressureInBand)
    'inband': _CFG['G_cfgPressureInBand'],
    # [MPa] Mechanical pressure ceiling (Params.P_ceiling)
    'ceiling': _PAR.P_ceiling / 1e6,
    # [MPa] Required margin above the saturation pressure
    # (Params.degas_margin)
    'degas': _PAR.degas_margin / 1e6,
    # [mm] Stroke limit: the GVL soft limit minus the chamber contact
    # position of 200 mm assumed in the slave simulator
    'stroke_mm': _CFG['G_cfgPosSoftLimitMax'] - 200.0,
    # [mL] Saline delivered after control is lost, for ranking only; the
    # requirement is that none is
    'v_lost': 10.0,
}

# Requirement name -> (scenario, description)
REQUIREMENTS = {
    'compress.band': ('compress', 'in band at the end of compression'),
    'compress.ceiling': ('compress', 'P_max <= %(ceiling)g MPa'),
    'compress.degas': ('compress',
                       'P - P_sat >= %(degas)g MPa after arrival'),
    'deliver.ceiling': ('deliver', 'P_max <= %(ceiling)g MPa'),
    'deliver.degas': ('deliver',
                      'P - P_sat >= %(degas)g MPa through the delivery'),
    'deliver.control': ('deliver',
                        'control kept to the end (V_ok = V_delivered)'),
    'stroke': (None, 'stroke <= %(stroke_mm)g mm, all scenarios'),
}


def describe(req, th=THRESH):
    """Return the description of requirement req with its limit."""
    return REQUIREMENTS[req][1] % th


@dataclass(frozen=True)
class KeyMetric:
    """A metric ranked in the tornado and scatter plots, with the limit
    it is judged against and the sense of the limit."""

    scenario: str
    metric: str
    limit: float
    upper: bool = True      # True: metric <= limit passes

    @property
    def key(self):
        return (self.scenario, self.metric)

    def excess(self, value):
        """Return a ratio that is 1 at the limit and above 1 when the
        value fails: value/limit for an upper limit, and 1 plus the
        shortfall divided by the limit for a lower one. NaN counts as
        infinite."""
        if not math.isfinite(value):
            return math.inf
        if self.upper:
            return value / self.limit
        return 1.0 + (self.limit - value) / abs(self.limit)


KEY_METRICS = [
    KeyMetric('compress', 'P_max', THRESH['ceiling']),
    KeyMetric('compress', 'maxErrLate', THRESH['inband']),
    KeyMetric('compress', 'degasMargin', THRESH['degas'], upper=False),
    KeyMetric('compress', 'stroke_mm', THRESH['stroke_mm']),
    KeyMetric('deliver', 'P_max', THRESH['ceiling']),
    KeyMetric('deliver', 'degasMargin', THRESH['degas'], upper=False),
    KeyMetric('deliver', 'V_lost', THRESH['v_lost']),
]
KEY_BY_KEY = {km.key: km for km in KEY_METRICS}


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


def base_params(ratio=1.0, P_target=None, hose_before=False):
    """Return Params for the study: the settling band set to the slave's
    status band, the oxygen charge set to ratio, the hose procedure
    and, if given, the target pressure [Pa]."""
    par = Params(tol=THRESH['inband'] * 1e6,            # [MPa -> Pa]
                 ratio=ratio, hose_before=hose_before)
    if P_target is not None:
        par = par.with_(P_target=P_target)
    return par


def make_params(values, ratio=1.0, P_target=None, hose_before=False):
    """Return base_params(ratio, P_target, hose_before) with the sampled
    values applied. A 'ratio' entry in values overrides the argument."""
    values = dict(values)
    ratio = values.pop('ratio', ratio)
    return base_params(ratio, P_target, hose_before).with_(**values)


def make_scenario(name, par, ff_scale=1.0):
    """Return scenario name for Params par, with the feedforward scale
    applied to the delivery."""
    if name == 'deliver':
        return scenarios.make(name, par, ff_scale=ff_scale)
    return scenarios.make(name, par)


# %% ===== METRICS AND REQUIREMENTS ======================================
def extras(r, scenario):
    """Return the metrics derived from the traces beyond
    metrics.compute(): maxErrLate [MPa], the largest error of the
    measured pressure over the last 2 s (the slave's in-band status is
    judged on the measurement), and V_lost [mL], the saline delivered
    after control was lost (V_delivered - V_ok)."""
    last_2s = r.t > r.t[-1] - 2
    err_late = np.abs(r.target[last_2s] - r.P_meas[last_2s])    # [Pa]
    out = {'maxErrLate': float(np.max(err_late)) / 1e6}
    m = r.metrics
    if 'V_delivered' in m:
        out['V_lost'] = m['V_delivered'] - m['V_ok']
    return out


METRIC_NAMES = ('overshoot', 'tSettle', 'peakErr', 'rmsErr', 'ripple',
                'uAct', 'stroke_mm', 'P_max', 'P_min', 'degasMargin',
                'dip', 'dipMax', 'V_delivered', 'V_ok', 'V_lost',
                't_arrive',
                'maxErrLate')


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
    ratio: float = 1.0      # oxygen charge the arm is run with
    sched: dict = None      # ScheduledSlaveController arguments, or None
    P_target: float = None  # [Pa] target pressure; None for the default
    hose_before: bool = False   # hose opened before the compression


def make_controller(spec):
    """Return the controller of a study arm."""
    if spec.sched is not None:
        return ctl.ScheduledSlaveController(
            spec.name, flow_ff=spec.flow_ff, **spec.sched,
            **spec.overrides)
    return ctl.SlaveController(spec.name, flow_ff=spec.flow_ff,
                               **spec.overrides)


def point_metrics(point):
    """Run one point and return its metrics dict.

    point is (values, scenario name, ControllerSpec, seed). A run whose
    state becomes non-finite returns {'diverged': True} with every
    metric NaN, which fails every requirement.
    """
    values, scen, spec, seed = point
    par = make_params(values, spec.ratio, spec.P_target, spec.hose_before)
    s = make_scenario(scen, par, spec.ff_scale)
    c = make_controller(spec)
    try:
        with np.errstate(invalid='raise', over='raise', divide='raise'):
            r = sim.run(par, s, c, seed=seed)
        m = {k: math.nan for k in METRIC_NAMES}
        m.update(r.metrics)
        m.update(extras(r, s))
        m['diverged'] = not all(math.isfinite(v) for k, v in m.items()
                                if k not in ('tSettle', 't_arrive'))
    except (FloatingPointError, OverflowError, ValueError,
            ZeroDivisionError):
        m = {k: math.nan for k in METRIC_NAMES}
        m['diverged'] = True
    return m


def _le(x, limit):
    """Return x <= limit; False when x is NaN."""
    return bool(x <= limit)


def _ge(x, limit):
    """Return x >= limit; False when x is NaN."""
    return bool(x >= limit)


def evaluate(scen, m, th=THRESH):
    """Return {requirement: passed} for the requirements of scenario scen
    and its stroke check, from metrics dict m."""
    out = {}
    if scen == 'compress':
        out['compress.band'] = (_le(m['maxErrLate'], th['inband'])
                                and not math.isnan(m['tSettle']))
        out['compress.ceiling'] = _le(m['P_max'], th['ceiling'])
        out['compress.degas'] = _ge(m['degasMargin'], th['degas'])
    elif scen == 'deliver':
        out['deliver.ceiling'] = _le(m['P_max'], th['ceiling'])
        out['deliver.degas'] = _ge(m['degasMargin'], th['degas'])
        out['deliver.control'] = (_ge(m['V_ok'], m['V_delivered'] - 1e-6)
                                  and m['V_delivered'] > 0)
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
