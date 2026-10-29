"""
sim.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module runs one closed-loop simulation of the pressure loop and
returns its traces and metrics.

The simulation steps in slave scans of length Ts. In each scan:

  1. The sensor samples the chamber pressure and returns the reading the
     controller uses (sensor.py).
  2. The controller computes a new command from the reading
     (controllers.py).
  3. The plant advances one scan with the command computed in the
     previous scan, held constant, and with the outlet valve in the
     state the scenario gives for this scan (plant.py and solver.py).

The one-scan delay between steps 2 and 3 reproduces the order in which
PRG_Main calls its function blocks. The simulation sequences the valves:
the proximal valve opens at the scenario's t_prox (the plant equalises
the pressure with the hose, eq. (43)); the distal valve opens at t_open
and, in that scan, the plant also loses the tube's air volume, eq. (37);
with a purge stage the distal valve closes once the purge volume has
left and opens again after the pause. The event times are recorded for
the metrics. A run ends early when the piston reaches the limit switch
(the cylinder is down to its passages). The traces are recorded after
the plant step, except the measured pressure, which is the reading the
controller used in that scan.

FILE DEPENDENCY:
./
    metrics.py
    plant.py
    sensor.py
    solver.py (through plant.py)
    params.py, scenarios.py, controllers.py (supply the arguments)
"""

from dataclasses import dataclass

import numpy as np

import metrics
from plant import Plant
from sensor import LegacySensor, SlaveSensor


# %% ===== RESULT OF ONE RUN =============================================
@dataclass
class Result:
    """Traces and metrics of one run. Each trace has one entry per scan.

    Indexing a Result by a metric name returns that metric, for example
    r['overshoot'].
    """

    name: str              # controller label
    scenario: str          # scenario name
    mode: str              # 'V' (velocity command) or 'T' (force)
    t: np.ndarray          # [s] time
    P: np.ndarray          # [Pa] chamber gauge pressure
    P_meas: np.ndarray     # [Pa] measured pressure the controller used
    u: np.ndarray          # command: [m/s] in 'V' mode, [N] in 'T' mode
    v: np.ndarray          # [m/s] piston velocity
    x: np.ndarray          # [m] piston position
    target: np.ndarray     # [Pa] pressure target
    stiffness: np.ndarray  # [Pa/m] chamber stiffness dP/dx
    Qout: np.ndarray       # [m^3/s] outflow through the outlet
    metrics: dict          # see metrics.py
    cost: float            # see metrics.loop_cost
    A: float = 0.0         # [m^2] piston area, for u_plot()
    P_sat: np.ndarray = None   # [Pa] saturation pressure of the saline
    V_out: np.ndarray = None   # [m^3] saline delivered so far
    V_l: np.ndarray = None     # [m^3] solution remaining
    events: list = None    # [s] times of the valve events, in order
    V_purge: float = 0.0   # [m^3] virgin saline purged before delivery

    def __getitem__(self, key):
        return self.metrics[key]

    def u_plot(self):
        """Return the command in plotting units: mm/s in 'V' mode, or the
        force divided by the piston area in MPa in 'T' mode."""
        if self.mode == 'V':
            return self.u * 1e3                     # [m/s -> mm/s]
        return self.u / self.A / 1e6                # [N -> Pa -> MPa]


# %% ===== SENSOR SELECTION ==============================================
def make_sensor(par, controller, P0, legacy_sensor):
    """Return the measurement chain for a run.

    The slave configuration includes any sensor settings the controller
    requires (controller.cfg).
    """
    cfg = par.cfg(**controller.cfg)
    if legacy_sensor:
        return LegacySensor(par, cfg, P0)
    return SlaveSensor(par, cfg, P0)


# %% ===== SIMULATION ====================================================
def run(par, scenario, controller, legacy_sensor=False, seed=1):
    """Simulate scenario under controller with Params par.

    legacy_sensor selects the legacy measurement chain instead of the
    slave's. The sensor noise draws come from numpy's default generator
    seeded with seed; par.noise sets their RMS value.
    """
    # Time base, inputs and noise, one entry per scan
    Ts = par.Ts                                             # [s]
    N = round(scenario.T / Ts)                              # [-] scans
    t = np.arange(N) * Ts                                   # [s]
    target = np.array([scenario.target(tk) for tk in t])    # [Pa]
    if par.noise:
        z = np.random.default_rng(seed).standard_normal(N)
    else:
        z = np.zeros(N)

    # Initial conditions
    u_applied = controller.reset(par, scenario)
    plant = Plant(par, scenario, controller.mode, controller.accel,
                  controller.decel)
    sensor = make_sensor(par, controller, scenario.P0, legacy_sensor)

    # The tube's air volume leaves the chamber in the opening scan as a
    # flow of V_tube / Ts, eq. (37)
    Q_tube_open = par.V_tube / Ts if scenario.tube else 0.0  # [m^3/s]
    vented = False
    # Valve sequence: proximal opening, then the distal valve's first
    # opening (a purge if V_purge > 0), its closing, and its reopening
    events = []
    hose_opened = scenario.hose_open0
    t_from = scenario.t_open                    # [s] distal opening
    V_out_open = 0.0                            # [m^3] delivered at it
    purging = scenario.V_purge > 0.0
    t_closing = float('inf')                    # [s] closing ramp start
    closed_until = -1.0                         # [s] reopen after this
    V_purged = 0.0

    # Trace storage. Python lists of floats are faster than numpy arrays
    # for element-by-element writes in the loop.
    (P, P_meas, u_trace, v, x, stiffness, Q, P_sat, V_out,
     V_l) = ([0.0] * N for _ in range(10))
    t_list, target_list = t.tolist(), target.tolist()
    z_list = z.tolist()

    # Scan loop
    n_run = N
    for k in range(N):
        # 1. Sensor reading
        reading = sensor.read(plant.P, z_list[k])
        # 2. Controller command for the next scan
        u_next = controller.step(reading, target_list[k], t_list[k],
                                 scenario, plant.x)
        # 3. Plant advance with the previous scan's command and this
        #    scan's valve states
        tk = t_list[k]
        if not hose_opened and tk >= scenario.t_prox:
            plant.open_hose()
            hose_opened = True
            events.append(tk)
        if tk < closed_until:
            # Closing ramp over Tclose, then shut for the pause
            a = max(0.0, 1.0 - (tk - t_closing) / scenario.Tclose) \
                if scenario.Tclose > 0.0 else 0.0
        else:
            a = scenario.valve(tk, t_from)
        Q_tube = 0.0
        if a > 0.0 and not vented:
            Q_tube = Q_tube_open
            vented = True
            events.append(tk)
            V_out_open = plant.V_out
        plant.scan(u_applied, a, Q_tube)
        if (purging and a > 0.0
                and plant.V_out - V_out_open >= scenario.V_purge):
            # The purge is done: close, and reopen after the pause
            purging = False
            V_purged = plant.V_out
            t_closing = tk
            closed_until = tk + scenario.Tclose + scenario.t_pause
            t_from = closed_until
            events.append(tk)
            events.append(closed_until)
            if scenario.tube_refills:
                vented = False
        u_applied = u_next

        P[k] = plant.P
        P_meas[k] = reading
        u_trace[k] = u_next
        v[k] = plant.v
        x[k] = plant.x
        stiffness[k] = plant.stiffness
        Q[k] = plant.Q
        V_out[k] = plant.V_out
        V_l[k] = plant.V_l
        P_sat[k] = par.P_sat(plant.n_dis, max(plant.V_l, 1e-6))
        if plant.empty:
            n_run = k + 1
            break

    # Post-processing. A run that ended early is truncated.
    arr = lambda a: np.array(a[:n_run])  # noqa: E731
    r = Result(controller.name, scenario.name, controller.mode, t[:n_run],
               arr(P), arr(P_meas), arr(u_trace), arr(v), arr(x),
               target[:n_run], arr(stiffness), arr(Q), {}, 0.0, par.A,
               arr(P_sat), arr(V_out), arr(V_l), events, V_purged)
    r.metrics = metrics.compute(r, par, scenario, controller.mode)
    r.cost = metrics.loop_cost(r.metrics)
    return r
