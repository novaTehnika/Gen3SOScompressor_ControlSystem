"""Run metrics, measured on the true chamber pressure after the scenario's event.

overshoot   max(P - target) after the event, MPa (negative if never reached)
peakErr     max |target - P| after the event, MPa
tSettle     s after the event from which |e| stays within tol; 0 if never
            outside it, NaN if still outside at the end
rmsErr      rms error over the last 2 s, MPa
ripple      std of P over the last 2 s, MPa
uAct        command activity over the last 2 s: std of the per-scan command
            change, mm/s ('V') or MPa of force/A ('T')
stroke_mm   piston travel used: max(x) - min(x) over the run, mm
"""

import math

import numpy as np


def compute(r, p, scenario, mode):
    """Metrics dict from a Result-like object with traces t, P, target, u, x."""
    t, P, tgt = r.t, r.P, r.target
    N = len(t)
    ie = t >= scenario.tEvent
    e = tgt - P
    out = {'overshoot': float(np.max(P[ie] - tgt[ie])) / 1e6}
    bad = np.flatnonzero((np.abs(e) >= p.tol) & ie)
    if bad.size == 0:
        out['tSettle'] = 0.0
    elif bad[-1] == N - 1:
        out['tSettle'] = math.nan
    else:
        out['tSettle'] = float(t[bad[-1] + 1] - scenario.tEvent)
    out['peakErr'] = float(np.max(np.abs(e[ie]))) / 1e6
    last = t > scenario.T - 2
    out['rmsErr'] = float(np.sqrt(np.mean(e[last] ** 2))) / 1e6
    out['ripple'] = float(np.std(P[last], ddof=1)) / 1e6
    du = float(np.std(np.diff(r.u[last]), ddof=1))
    out['uAct'] = du * 1e3 if mode == 'V' else du / p.A / 1e6
    out['stroke_mm'] = float(np.max(r.x) - np.min(r.x)) * 1e3
    return out


def loop_cost(m):
    """Scalar ranking for gain sweeps: peak error, overshoot and late ripple
    all count, and never settling is penalised."""
    cost = m['peakErr'] + 2 * m['overshoot'] + 20 * m['ripple']
    if math.isnan(m['tSettle']):
        cost += 10
    return cost
