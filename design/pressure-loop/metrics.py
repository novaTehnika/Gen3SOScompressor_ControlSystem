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

  overshoot    [MPa] Largest P - target after the event. It is negative
               if the pressure never reaches the target.
  peakErr      [MPa] Largest |e| after the event.
  tSettle      [s] Time after the event from which |e| stays below
               Params.tol. It is 0 if |e| never leaves the band, and NaN
               if |e| is still outside the band at the end of the run.
  rmsErr       [MPa] RMS of e over the last 2 s.
  ripple       [MPa] Standard deviation of P over the last 2 s.
  uAct         Command activity: the standard deviation of the change in
               the command from one scan to the next over the last 2 s,
               in mm/s ('V' mode) or in MPa of force divided by the
               piston area ('T' mode).
  stroke_mm    [mm] Piston travel used, max(x) - min(x) over the run.

The process metrics judge a run against the machine's limits: the
pressure must stay below the ceiling, must not fall below the saturation
pressure of the dissolved oxygen once the target has been reached (or
the solution degasses), and the loop must keep control to the end of
the delivery.

  P_max        [MPa] Largest pressure in the run.
  P_min        [MPa] Lowest pressure after the target is first reached.
  degasMargin  [MPa] Smallest P - P_sat after the target is first
               reached; negative means the solution degassed.
  dip          [MPa] Target minus the lowest pressure within t_recover
               of the event (the opening dip of a delivery).
  V_delivered  [mL] Saline delivered through the outlet by the end.
  V_ok         [mL] Saline delivered before the first violation: P below
               P_sat + degas_margin, P above the ceiling, or |e| above
               e_crash more than t_recover after the valve opens. It
               equals V_delivered when nothing is violated and 0 when
               the opening dip itself degasses the solution.
  t_arrive     [s] Time the pressure first comes within tol of the
               target; NaN if it never does.

loop_cost() combines the metrics into one number for ranking the gains
in a sweep.

FILE DEPENDENCY:
None. The argument r is a sim.Result or any object with the same
traces.
"""

import math

import numpy as np


def compute(r, par, scenario, mode):
    """Return the metrics dict for the traces of run r."""
    t, P, target = r.t, r.P, r.target
    N = len(t)
    after_event = t >= scenario.tEvent
    last_2s = t > t[-1] - 2
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

    # Process metrics
    out.update(process(r, par, scenario))
    return out


def process(r, par, scenario):
    """Return the process metrics: pressure ceiling, saturation margin,
    dip and delivered volume."""
    t, P, target = r.t, r.P, r.target
    err = target - P                                        # [Pa]
    out = {'P_max': float(np.max(P)) / 1e6}

    # Arrival: the first scan within tol of the target
    arrived = np.flatnonzero(np.abs(err) < par.tol)
    if arrived.size == 0:
        out['t_arrive'] = math.nan
        out['P_min'] = math.nan
        out['degasMargin'] = math.nan
    else:
        i0 = arrived[0]
        out['t_arrive'] = float(t[i0])
        out['P_min'] = float(np.min(P[i0:])) / 1e6
        if r.P_sat is not None:
            margin = P[i0:] - r.P_sat[i0:]                  # [Pa]
            out['degasMargin'] = float(np.min(margin)) / 1e6

    # Dip below the target within t_recover of the event
    after_event = t >= scenario.tEvent
    window = after_event & (t < scenario.tEvent + par.t_recover)
    out['dip'] = float(target[after_event][0] - np.min(P[window])) / 1e6

    # Delivered volume, [m^3 -> mL]
    if r.V_out is None:
        return out
    out['V_delivered'] = float(r.V_out[-1]) * 1e6

    # Volume delivered before the first violation. The error check
    # starts t_recover after the valve opens.
    if arrived.size == 0 or r.P_sat is None:
        out['V_ok'] = 0.0
        return out
    i0 = arrived[0]
    bad = ((P < r.P_sat + par.degas_margin) | (P > par.P_ceiling))
    bad[:i0] = False
    settled = t >= scenario.t_open + par.t_recover
    bad |= settled & (np.abs(err) > par.e_crash)
    i_bad = np.flatnonzero(bad)
    if i_bad.size == 0:
        out['V_ok'] = out['V_delivered']
    else:
        out['V_ok'] = float(r.V_out[i_bad[0]]) * 1e6
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
