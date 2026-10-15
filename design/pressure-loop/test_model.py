#!/usr/bin/env python3
"""Self-checks of the pressure loop model.

    python3 design/pressure-loop/test_model.py

Prints `ok  ...` per check and stops at the first failure.
"""

import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import controllers as ctl  # noqa: E402
import scenarios  # noqa: E402
import sim  # noqa: E402
from FB_PressureController import FB_PressureController, new_FB_PressureController  # noqa: E402
from params import Params  # noqa: E402
from plant import Plant  # noqa: E402
from runner import Case, run_cases  # noqa: E402
from sensor import LegacySensor, SlaveSensor, filter_alpha  # noqa: E402
from slave import slave_config  # noqa: E402


def ok(msg):
    print(f'ok  {msg}')


def check_config():
    c = slave_config()
    assert c['G_cfgPressureSensorFaultTime'] == 0.1, c['G_cfgPressureSensorFaultTime']
    assert c['G_cfgScanTime'] == 0.002 and c['G_cfgAnalogMedianSize'] == 5
    p = Params()
    assert p.Ts == 0.002 and p.vmax == 5e-3 and p.medN == 5 and p.tauF == 0.05
    q = p.with_(G_cfgPressureKp=2.0, K=2e9)
    assert q.cfg()['G_cfgPressureKp'] == 2.0 and q.K == 2e9 and p.cfg()['G_cfgPressureKp'] == 0.5
    q = Params.from_overrides(['K=2e9', 'nsub=10', 'liquid_depletes=false', 'G_cfgScanTime=0.001'])
    assert q.K == 2e9 and q.nsub == 10 and q.liquid_depletes is False and q.Ts == 0.001
    try:
        Params.from_overrides(['nonsense=1'])
    except ValueError:
        pass
    else:
        raise AssertionError('unknown key accepted')
    ok('slave_config reads the GVL (TIME in s); Params overrides route to fields and G_cfg')


def liquid_plant(p, P0=10e6):
    sc = scenarios.stiffstep(p, P0=P0)
    return Plant(p, sc, 'V', accel=1.0, decel=1.0), sc


def check_stiffness():
    p = Params()
    plant, _ = liquid_plant(p)
    for _ in range(100):                       # reach 0.1 mm/s
        plant.scan(1e-4, 0.0)
    P1, x1 = plant.s.P, plant.s.x
    for _ in range(500):
        plant.scan(1e-4, 0.0)
    measured = (plant.s.P - P1) / (plant.s.x - x1)
    expected = p.A * p.K / p.Vl0
    assert abs(measured / expected - 1) < 0.02, (measured, expected)
    assert abs(plant.stiffness / expected - 1) < 0.02
    ok(f'sealed liquid chamber stiffness {measured / 1e9:.2f} MPa/mm = A K / Vl ({expected / 1e9:.2f})')


def check_conservation():
    p = Params()
    sc = scenarios.make('compress', p)
    plant = Plant(p, sc, 'V', accel=1.0, decel=1.0)
    n0 = plant.s.ntot
    worst = 0.0
    for k in range(3000):                      # 6 s at 3 mm/s into the gas-rich chamber
        plant.scan(3e-3, 0.0)
        worst = max(worst, abs(plant.volume_closure()))
    assert worst < 1e-8, worst
    assert abs(plant.s.ntot - n0) < 1e-15
    ok(f'volume closes against piston travel in the gas-rich chamber (max {worst:.1e} m^3 of {p.A * plant.s.x:.1e})')

    plant, sc = liquid_plant(p)
    n0, Vl0 = plant.s.ntot, plant.s.Vl
    nd0 = plant.s.ndis
    Qint = 0.0
    worst = 0.0
    for k in range(2000):                      # 4 s of 1 mL/s outflow with the piston following it
        plant.scan(1e-6 / p.A, 1e-6)
        Qint += 1e-6 * p.Ts
        worst = max(worst, abs(plant.volume_closure()))
    s = plant.s
    assert abs((s.ntot + s.nout) - n0) < 1e-15, 'gas moles'
    assert abs((Vl0 - s.Vl) / (Qint / (1 - s.P / p.K)) - 1) < 0.002, (Vl0 - s.Vl, Qint)
    assert abs(s.ndis / s.Vl / (nd0 / Vl0) - 1) < 2e-3, 'dissolved gas leaves with the liquid'
    assert worst < 1e-9, worst
    assert abs(s.nout / (nd0 * (Vl0 - s.Vl) / Vl0) - 1) < 0.02
    ok(f'outflow removes liquid ({(Vl0 - s.Vl) * 1e6:.2f} mL) and its dissolved gas; moles and volume close')

    q = p.legacy()
    plant = Plant(q, scenarios.stiffstep(q), 'V', accel=1.0, decel=1.0)
    for k in range(2000):
        plant.scan(1e-3, 1e-6)
    assert plant.s.Vl == q.Vl0 and abs(plant.volume_closure()) > 1e-6 * 0.5
    ok('legacy plant keeps the liquid volume fixed (volume does not close under outflow)')


def check_actuators():
    p = Params()
    pf = p.with_(Fcap=1e9)                     # no stall
    sc = scenarios.stiffstep(p, P0=0.0)
    plant = Plant(pf, sc, 'V', accel=0.010, decel=0.100)    # 10 and 100 mm/s^2
    seen = []
    for k in range(1000):
        plant.scan(0.005 if k < 500 else 0.0, 0.0)
        seen.append(plant.s.vref)
    seen = np.array(seen)
    up = (seen[100] - seen[50]) / (50 * p.Ts)
    dn = (seen[520] - seen[510]) / (10 * p.Ts)
    assert abs(up - 0.010) < 1e-4 and abs(dn + 0.100) < 1e-3, (up, dn)
    ok(f'velocity command slews at accel {up * 1e3:.1f} mm/s^2 up and decel {-dn * 1e3:.1f} mm/s^2 down')

    plant = Plant(p, sc, 'V', accel=1.0, decel=1.0)
    for k in range(600):
        plant.scan(0.005, 0.0)
    assert abs(plant.s.P - (p.Fcap - p.Fc) / p.A) / plant.s.P < 0.02 and plant.s.v == 0.0
    ok(f'velocity actuator stalls at the torque cap ({plant.s.P / 1e6:.1f} MPa)')

    plant = Plant(p, sc, 'T', accel=None, decel=None)
    for k in range(3000):
        plant.scan(p.A * 10e6 + 2000.0, 0.0)
    assert abs(plant.s.P - 10e6 - 2000.0 / p.A) < 5e3 and abs(plant.s.v) < 1e-6
    ok('torque actuator settles at force / A')


def check_sensor():
    p = Params().with_(noise=0.0)
    cfg = p.cfg()
    assert abs(filter_alpha(cfg) - 0.002 / 0.052) < 1e-15
    assert filter_alpha(p.cfg(G_cfgAnalogFilterTimeConst=0.0)) == 1.0
    rng = random.Random(3)
    seq = [rng.choice([2e6, 5e6, 9e6]) + rng.uniform(-1e5, 1e5) for _ in range(400)]
    slave, leg = SlaveSensor(p, cfg, seq[0]), LegacySensor(p, cfg, seq[0])
    a = [slave.read(P, 0.0) for P in seq]
    b = [leg.read(P, 0.0) for P in seq]
    err = max(abs(a[k] - b[k - 2]) for k in range(2, len(seq)))
    assert err < 1.0, err
    pn = Params()
    s2 = SlaveSensor(pn, cfg, 8e6)
    z = np.random.default_rng(0).standard_normal(4000)
    out = np.array([s2.read(8e6, zk) for zk in z])
    assert abs(out[10:].mean() - 8e6) < 2e3 and 0.02 * pn.noise < out[10:].std() < 0.4 * pn.noise, out[10:].std()
    ok(f'slave sensor chain = legacy chain two scans later (max diff {err:.1e} Pa); noise rms {out[10:].std():.0f} Pa after filtering')


def check_wiring():
    p = Params()
    sc = scenarios.stiffstep(p)
    c = ctl.SlaveController(Kp=0.8, StopStiffness=50)
    c.reset(p, sc)
    cfg = p.cfg(G_cfgPressureKp=0.8, G_cfgPressureStopStiffness=50)
    f = new_FB_PressureController()                 # PRG_Main's call, written out
    rng = random.Random(1)
    worst = 0.0
    for k in range(3000):
        ref = rng.choice([-1e6, 5e6, 10e6, 12e6, 20e6]) if k % 300 == 0 or k == 0 else ref
        meas = 10e6 + 3e6 * math.sin(k / 150.0) + rng.uniform(-1e5, 1e5)
        got = c.step(meas, ref, k * p.Ts, sc)
        f.Enable, f.Reset = True, (k == 0)
        f.Target = min(max(ref * 1e-6, 0.0), cfg['G_cfgPressureTargetMax'])
        f.Actual = meas * 1e-6
        f.Kp, f.Ki = cfg['G_cfgPressureKp'], cfg['G_cfgPressureKi']
        f.VelMax, f.DecelMax = cfg['G_cfgPressureVelMax'], cfg['G_cfgPressureDecel']
        f.Stiffness, f.StopVelMin = cfg['G_cfgPressureStopStiffness'], cfg['G_cfgPressureStopVelMin']
        f.CycleTime = cfg['G_cfgScanTime']
        FB_PressureController(f, cfg['G_cfgScanTime'])
        want = min(max(f.Velocity, -cfg['G_cfgPressureVelMax']), cfg['G_cfgPressureVelMax']) * 1e-3
        worst = max(worst, abs(got - want))
    assert worst < 1e-12, worst
    assert c.accel == cfg['G_cfgPressureAccel'] * 1e-3 and c.decel == cfg['G_cfgPressureDecel'] * 1e-3
    ok('SlaveController output is exactly the FB called as PRG_Main calls it (target clamp, VelMax clamp, accel/decel)')

    c = ctl.SlaveController(flow_ff=True)
    c.reset(p, sc)
    sc2 = scenarios.valve(p, Topen=0.0)
    c.step(10e6, 10e6, 0.0, sc2)
    v = c.step(10e6, 10e6, 2.0, sc2)
    assert abs(v - p.Qf / p.A) < 1e-9, v
    ok(f'flow feedforward adds Q/A ({v * 1e3:.3f} mm/s) after the block')


def check_legacy_reference():
    p = Params().legacy().with_(noise=0.0)
    c = ctl.LegacyController('ref', 'V', 0.5, 0.2, 0, True, 50e-3)
    r = sim.run(p, scenarios.stiffstep(p), c, legacy_sensor=True)
    # reference values for this case with noise 0
    assert abs(r['overshoot'] - 0.19860) < 1e-4 and abs(r['tSettle'] - 1.3120) < 1e-3 and abs(r['peakErr'] - 2.0) < 1e-4, r.metrics
    ok('legacy plant, sensor and controller match the reference values (stiffstep overshoot 0.1986 MPa, settle 1.312 s)')


def check_closed_loop():
    p = Params()
    t0 = time.time()
    r = sim.run(p, scenarios.compress(p), ctl.SlaveController())
    dt = time.time() - t0
    inband = slave_config()['G_cfgPressureInBand'] * 1e6
    late = r.t > r.t[-1] - 5
    assert np.all(np.abs(r.P[late] - r.target[late]) <= inband), np.abs(r.P[late] - r.target[late]).max()
    assert r['overshoot'] < 1.0 and r['stroke_mm'] < slave_config()['G_cfgPosSoftLimitMax']
    ok(f'compress with the slave controller reaches the in-band status (overshoot {r["overshoot"]:.2f} MPa, '
       f'stroke {r["stroke_mm"]:.1f} mm, 60 s in {dt:.1f} s)')

    r = sim.run(p, scenarios.discharge(p), ctl.SlaveController())
    assert r['peakErr'] < 1.0 and r['rmsErr'] < 0.05
    ok(f'discharge holds the target (peak error {r["peakErr"]:.2f} MPa)')


def check_runner():
    p = Params()
    cases = [Case(p, scenarios.stiffstep(p), ctl.SlaveController(Kp=k)) for k in (0.5, 1.0)]
    serial = run_cases(cases, workers=1)
    par = run_cases(cases, workers=2)
    for a, b in zip(serial, par):
        assert a.metrics == b.metrics
    light = run_cases(cases, workers=2, traces=False)
    assert light[0].P is None and light[0].metrics == serial[0].metrics
    ok('parallel runner (spawn) matches serial runs')


if __name__ == '__main__':
    check_config()
    check_stiffness()
    check_conservation()
    check_actuators()
    check_sensor()
    check_wiring()
    check_legacy_reference()
    check_closed_loop()
    check_runner()
