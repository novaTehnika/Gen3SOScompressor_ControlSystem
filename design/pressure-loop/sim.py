"""One closed-loop run: sensor, controller, plant, metrics.

Each scan the sensor reads the chamber pressure, the controller computes a
command from the reading, and the plant advances nsub substeps with the
command computed in the previous scan (the controller's output is applied one
scan later, as PRG_Main's FB call order implies). Traces are recorded after
the plant step, except P_meas, which is the reading the controller used.
"""

from dataclasses import dataclass

import numpy as np

import metrics
from plant import Plant
from sensor import LegacySensor, SlaveSensor


@dataclass
class Result:
    name: str
    scenario: str
    mode: str
    t: np.ndarray
    P: np.ndarray          # Pa chamber gauge pressure
    P_meas: np.ndarray     # Pa measurement the controller used
    u: np.ndarray          # command: m/s ('V') or N ('T')
    v: np.ndarray          # m/s piston velocity
    x: np.ndarray          # m piston position
    target: np.ndarray     # Pa
    stiffness: np.ndarray  # Pa/m dP/dx
    Qout: np.ndarray       # m^3/s
    metrics: dict
    cost: float
    A: float = 0.0

    def __getitem__(self, key):
        return self.metrics[key]

    def u_plot(self):
        """command in mm/s ('V') or MPa of force/A ('T')"""
        return self.u * 1e3 if self.mode == 'V' else self.u / self.A / 1e6


def make_sensor(p, controller, P0, legacy_sensor):
    cfg = p.cfg(**controller.cfg)
    if legacy_sensor:
        return LegacySensor(p, cfg, P0)
    return SlaveSensor(p, cfg, P0)


def run(p, scenario, controller, legacy_sensor=False, seed=1):
    """Simulate `scenario` under `controller` with plant parameters `p`.

    legacy_sensor selects the median plus first-order chain in pressure units
    instead of the slave's blocks. p.noise is the sensor noise in Pa rms; the
    draws come from numpy's Generator seeded with `seed`.
    """
    Ts = p.Ts
    N = round(scenario.T / Ts)
    t = np.arange(N) * Ts
    tgt = np.array([scenario.target(tk) for tk in t])
    Q = np.array([scenario.Qout(tk) for tk in t])
    z = np.random.default_rng(seed).standard_normal(N) if p.noise else np.zeros(N)

    u_applied = controller.reset(p, scenario)
    plant = Plant(p, scenario, controller.mode, controller.accel, controller.decel)
    sensor = make_sensor(p, controller, scenario.P0, legacy_sensor)
    s = plant.s
    read, step, scan = sensor.read, controller.step, plant.scan

    P = [0.0] * N
    Pm = [0.0] * N
    U = [0.0] * N
    V = [0.0] * N
    X = [0.0] * N
    S = [0.0] * N
    tl, tgl, Ql, zl = t.tolist(), tgt.tolist(), Q.tolist(), z.tolist()
    A = p.A
    for k in range(N):
        pm = read(s.P, zl[k])
        u = step(pm, tgl[k], tl[k], scenario, s.x)
        scan(u_applied, Ql[k])
        u_applied = u
        P[k], Pm[k], U[k], V[k], X[k], S[k] = s.P, pm, u, s.v, s.x, A / s.C

    r = Result(controller.name, scenario.name, controller.mode, t, np.array(P), np.array(Pm),
               np.array(U), np.array(V), np.array(X), tgt, np.array(S), Q, {}, 0.0, A)
    r.metrics = metrics.compute(r, p, scenario, controller.mode)
    r.cost = metrics.loop_cost(r.metrics)
    return r
