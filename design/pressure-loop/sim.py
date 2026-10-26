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
PRG_Main calls its function blocks. In the scan the valve opens, the
plant also loses the tube's air volume, eq. (37). A run ends early when
the piston reaches the limit switch (the chamber is down to its dead
volume). The traces are recorded after the plant step,
except the measured pressure, which is the reading the controller used
in that scan.

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
    V_l: np.ndarray = None     # [m^3] saline remaining

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
    valve = np.array([scenario.valve(tk) for tk in t])      # [-]
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

    # Trace storage. Python lists of floats are faster than numpy arrays
    # for element-by-element writes in the loop.
    (P, P_meas, u_trace, v, x, stiffness, Q, P_sat, V_out,
     V_l) = ([0.0] * N for _ in range(10))
    t_list, target_list = t.tolist(), target.tolist()
    valve_list, z_list = valve.tolist(), z.tolist()

    # Scan loop
    n_run = N
    for k in range(N):
        # 1. Sensor reading
        reading = sensor.read(plant.P, z_list[k])
        # 2. Controller command for the next scan
        u_next = controller.step(reading, target_list[k], t_list[k],
                                 scenario, plant.x)
        # 3. Plant advance with the previous scan's command and this
        #    scan's valve state
        Q_tube = 0.0
        if valve_list[k] > 0.0 and not vented:
            Q_tube = Q_tube_open
            vented = True
        plant.scan(u_applied, valve_list[k], Q_tube)
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
               arr(P_sat), arr(V_out), arr(V_l))
    r.metrics = metrics.compute(r, par, scenario, controller.mode)
    r.cost = metrics.loop_cost(r.metrics)
    return r
