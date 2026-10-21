"""
metrics.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module computes the performance metrics of one simulation run.
The metrics use the true chamber pressure, not the measurement, and
those that concern the response to the scenario's event are measured
from the event time tEvent. With error e = target - P:

  overshoot   [MPa] Largest P - target after the event. It is negative
              if the pressure never reaches the target.
  peakErr     [MPa] Largest |e| after the event.
  tSettle     [s] Time after the event from which |e| stays below
              Params.tol. It is 0 if |e| never leaves the band, and NaN
              if |e| is still outside the band at the end of the run.
  rmsErr      [MPa] RMS of e over the last 2 s.
  ripple      [MPa] Standard deviation of P over the last 2 s.
  uAct        Command activity: the standard deviation of the change in
              the command from one scan to the next over the last 2 s,
              in mm/s ('V' mode) or in MPa of force divided by the piston
              area ('T' mode).
  stroke_mm   [mm] Piston travel used, max(x) - min(x) over the run.

loop_cost() combines the metrics into one number for ranking the gains
in a sweep.

FILE DEPENDENCY:
None. The argument r is a sim.Result or any object with the same
traces.
"""

import math

import numpy as np


def compute(r, par, scenario, mode):
    """Return the metrics dict for the traces t, P, target, u and x of
    run r."""
    t, P, target = r.t, r.P, r.target
    N = len(t)
    after_event = t >= scenario.tEvent
    last_2s = t > scenario.T - 2
    err = target - P                                        # [Pa]
    out = {}

    # Overshoot and peak error after the event, [Pa -> MPa]
    out['overshoot'] = float(np.max(P[after_event]
                                    - target[after_event])) / 1e6
    out['peakErr'] = float(np.max(np.abs(err[after_event]))) / 1e6

    # Settling time: the first scan after the last scan outside the band
    outside = np.flatnonzero((np.abs(err) >= par.tol) & after_event)
    if outside.size == 0:
        out['tSettle'] = 0.0
    elif outside[-1] == N - 1:
        out['tSettle'] = math.nan
    else:
        out['tSettle'] = float(t[outside[-1] + 1] - scenario.tEvent)

    # Error and ripple over the last 2 s, [Pa -> MPa]
    out['rmsErr'] = float(np.sqrt(np.mean(err[last_2s]**2))) / 1e6
    out['ripple'] = float(np.std(P[last_2s], ddof=1)) / 1e6

    # Command activity over the last 2 s
    du = float(np.std(np.diff(r.u[last_2s]), ddof=1))
    if mode == 'V':
        out['uAct'] = du * 1e3                      # [m/s -> mm/s]
    else:
        out['uAct'] = du / par.A / 1e6              # [N -> Pa -> MPa]

    # Stroke used, [m -> mm]
    out['stroke_mm'] = float(np.max(r.x) - np.min(r.x)) * 1e3
    return out


def loop_cost(m):
    """Return a single cost for ranking gains in a sweep.

    The cost adds the peak error, twice the overshoot and twenty times
    the late ripple (all in MPa), plus 10 if the run never settles:

      cost = peakErr + 2 overshoot + 20 ripple (+ 10 if not settled)
    """
    cost = m['peakErr'] + 2 * m['overshoot'] + 20 * m['ripple']
    if math.isnan(m['tSettle']):
        cost += 10
    return cost
