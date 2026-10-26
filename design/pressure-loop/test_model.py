#!/usr/bin/env python3
"""
test_model.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script checks the pressure loop model against physical laws,
against the slave's own function blocks, and against reference results.
It prints "ok" and a short result for each check and stops at the first
failure. tests/run_all.sh runs it.

The checks cover:
  - reading the slave configuration and overriding Params;
  - the sealed liquid chamber stiffness against A K / Vl;
  - conservation of volume and gas moles, with and without outflow;
  - the agreement of the two integration methods, which bounds the
    integration error at the chosen step;
  - the slew limits, stall and force balance of the actuators;
  - the slave sensor chain against the legacy chain;
  - the wiring of the slave controller against its block called
    directly;
  - reference values of the legacy model, and closed-loop runs of the
    slave controller;
  - the parallel runner against serial runs.

Usage:
    python3 design/pressure-loop/test_model.py

FILE DEPENDENCY:
./
    controllers.py, params.py, plant.py, runner.py, scenarios.py,
    sensor.py, sim.py, slave.py, solver.py
../generated/
    FB_PressureController.py
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
from FB_PressureController import (FB_PressureController,  # noqa: E402
                                   new_FB_PressureController)
from params import Params  # noqa: E402
from plant import Plant, iy_ndis, iy_nout, iy_ntot, iy_Vl  # noqa: E402
from runner import Case, run_cases  # noqa: E402
from sensor import LegacySensor, SlaveSensor, filter_alpha  # noqa: E402
from slave import slave_config  # noqa: E402


def ok(msg):
    print(f'ok  {msg}')


# %% ===== CONFIGURATION =================================================
def check_config():
    c = slave_config()
    assert c['G_cfgPressureSensorFaultTime'] == 0.1, \
        c['G_cfgPressureSensorFaultTime']
    assert c['G_cfgScanTime'] == 0.002 and c['G_cfgAnalogMedianSize'] == 5
    par = Params()
    assert (par.Ts == 0.002 and par.vmax == 5e-3 and par.medN == 5
            and par.tauF == 0.05)
    q = par.with_(G_cfgPressureKp=2.0, K=2e9)
    assert (q.cfg()['G_cfgPressureKp'] == 2.0 and q.K == 2e9
            and par.cfg()['G_cfgPressureKp'] == 0.5)
    q = Params.from_overrides(['K=2e9', 'nsub=10', 'liquid_depletes=false',
                               'G_cfgScanTime=0.001', 'solver=euler'])
    assert (q.K == 2e9 and q.nsub == 10 and q.liquid_depletes is False
            and q.Ts == 0.001 and q.solver == 'euler')
    try:
        Params.from_overrides(['nonsense=1'])
    except ValueError:
        pass
    else:
        raise AssertionError('unknown key accepted')
    ok('slave_config reads the GVL (TIME in s); Params overrides route to '
       'fields and G_cfg')


# %% ===== CHAMBER ========================================================
def liquid_plant(par, P0=10e6):
    """Return a velocity-mode plant with a liquid-only chamber at P0
    [Pa] and fast slew limits, and its scenario."""
    sc = scenarios.stiffstep(par, P0=P0)
    return Plant(par, sc, 'V', accel=1.0, decel=1.0), sc


def check_stiffness():
    par = Params()
    plant, _ = liquid_plant(par)
    for _ in range(100):                    # reach 0.1 mm/s
        plant.scan(1e-4, 0.0)
    P1, x1 = plant.P, plant.x
    for _ in range(500):
        plant.scan(1e-4, 0.0)
    measured = (plant.P - P1) / (plant.x - x1)              # [Pa/m]
    expected = par.A * par.K / par.Vl0                      # [Pa/m]
    assert abs(measured / expected - 1) < 0.02, (measured, expected)
    assert abs(plant.stiffness / expected - 1) < 0.02
    ok(f'sealed liquid chamber stiffness {measured / 1e9:.2f} MPa/mm = '
       f'A K / Vl ({expected / 1e9:.2f})')


def check_conservation():
    par = Params()

    # Gas-rich chamber, sealed: the volume closes and no gas is lost.
    sc = scenarios.make('compress', par)
    plant = Plant(par, sc, 'V', accel=1.0, decel=1.0)
    n0 = plant.y[iy_ntot]
    worst = 0.0
    for k in range(3000):                   # 6 s at 3 mm/s
        plant.scan(3e-3, 0.0)
        worst = max(worst, abs(plant.volume_closure()))
    assert worst < 1e-8, worst
    assert abs(plant.y[iy_ntot] - n0) < 1e-15
    ok(f'volume closes against piston travel in the gas-rich chamber '
       f'(max {worst:.1e} m^3 of {par.A * plant.x:.1e})')

    # Liquid-only chamber with outflow: the liquid and its dissolved gas
    # leave, and the moles and volume still balance.
    plant, sc = liquid_plant(par)
    n0, Vl0 = plant.y[iy_ntot], plant.y[iy_Vl]
    nd0 = plant.y[iy_ndis]
    Q_int = 0.0                             # [m^3] outflow volume
    worst = 0.0
    for k in range(2000):                   # 4 s of 1 mL/s outflow
        plant.scan(1e-6 / par.A, 1e-6)      # piston follows the outflow
        Q_int += 1e-6 * par.Ts
        worst = max(worst, abs(plant.volume_closure()))
    y = plant.y
    removed = Vl0 - y[iy_Vl]                # [m^3] unstressed liquid
    assert abs((y[iy_ntot] + y[iy_nout]) - n0) < 1e-15, 'gas moles'
    assert abs(removed / (Q_int / (1 - plant.P / par.K)) - 1) < 0.002, \
        (removed, Q_int)
    assert abs(y[iy_ndis] / y[iy_Vl] / (nd0 / Vl0) - 1) < 2e-3, \
        'dissolved gas leaves with the liquid'
    assert worst < 1e-9, worst
    assert abs(y[iy_nout] / (nd0 * removed / Vl0) - 1) < 0.02
    ok(f'outflow removes liquid ({removed * 1e6:.2f} mL) and its '
       f'dissolved gas; moles and volume close')

    # Earlier model: the liquid volume stays fixed, so the volume does
    # not close under outflow.
    q = par.legacy()
    plant = Plant(q, scenarios.stiffstep(q), 'V', accel=1.0, decel=1.0)
    for k in range(2000):
        plant.scan(1e-3, 1e-6)
    assert (plant.y[iy_Vl] == q.Vl0
            and abs(plant.volume_closure()) > 1e-6 * 0.5)
    ok('legacy plant keeps the liquid volume fixed (volume does not close '
       'under outflow)')


def check_solver():
    # Heun and forward Euler differ by about the local error of the
    # first-order method, so close agreement bounds the integration error
    # at the chosen step. study_convergence.py gives the full picture.
    worst = {}
    for name in ('compress', 'stiffstep', 'discharge'):
        r = {}
        for method in ('euler', 'heun'):
            par = Params(solver=method)
            r[method] = sim.run(par, scenarios.make(name, par),
                                ctl.SlaveController())
        dP = np.abs(r['euler'].P - r['heun'].P).max()       # [Pa]
        worst[name] = dP
        assert dP < 0.02e6, (name, dP)
    ok('forward Euler and Heun agree: max pressure difference '
       + ', '.join(f'{k} {v / 1e3:.1f} kPa' for k, v in worst.items()))


# %% ===== ACTUATORS =====================================================
def check_actuators():
    par = Params()

    # Slew limits: 10 mm/s^2 up and 100 mm/s^2 down, with no stall
    no_stall = par.with_(Fcap=1e9)
    sc = scenarios.stiffstep(par, P0=0.0)
    plant = Plant(no_stall, sc, 'V', accel=0.010, decel=0.100)
    seen = []
    for k in range(1000):
        plant.scan(0.005 if k < 500 else 0.0, 0.0)
        seen.append(plant.v_ref)
    seen = np.array(seen)
    up = (seen[100] - seen[50]) / (50 * par.Ts)             # [m/s^2]
    dn = (seen[520] - seen[510]) / (10 * par.Ts)            # [m/s^2]
    assert abs(up - 0.010) < 1e-4 and abs(dn + 0.100) < 1e-3, (up, dn)
    ok(f'velocity command slews at accel {up * 1e3:.1f} mm/s^2 up and '
       f'decel {-dn * 1e3:.1f} mm/s^2 down')

    # Stall at the torque limit
    plant = Plant(par, sc, 'V', accel=1.0, decel=1.0)
    for k in range(600):
        plant.scan(0.005, 0.0)
    P_stall = (par.Fcap - par.Fc) / par.A
    assert abs(plant.P - P_stall) / plant.P < 0.02 and plant.v == 0.0
    ok(f'velocity actuator stalls at the torque cap '
       f'({plant.P / 1e6:.1f} MPa)')

    # Torque mode settles where the force balances the pressure
    plant = Plant(par, sc, 'T', accel=None, decel=None)
    for k in range(3000):
        plant.scan(par.A * 10e6 + 2000.0, 0.0)
    assert (abs(plant.P - 10e6 - 2000.0 / par.A) < 5e3
            and abs(plant.v) < 1e-6)
    ok('torque actuator settles at force / A')


# %% ===== SENSOR ========================================================
def check_sensor():
    par = Params().with_(noise=0.0)
    cfg = par.cfg()
    assert abs(filter_alpha(cfg) - 0.002 / 0.052) < 1e-15
    assert filter_alpha(par.cfg(G_cfgAnalogFilterTimeConst=0.0)) == 1.0

    # Without noise, the slave chain equals the legacy chain two scans
    # later.
    rng = random.Random(3)
    seq = [rng.choice([2e6, 5e6, 9e6]) + rng.uniform(-1e5, 1e5)
           for _ in range(400)]
    slave = SlaveSensor(par, cfg, seq[0])
    leg = LegacySensor(par, cfg, seq[0])
    a = [slave.read(P, 0.0) for P in seq]
    b = [leg.read(P, 0.0) for P in seq]
    err = max(abs(a[k] - b[k - 2]) for k in range(2, len(seq)))
    assert err < 1.0, err

    # With noise, the filtered reading is unbiased and much quieter than
    # the raw samples.
    par_n = Params()
    s2 = SlaveSensor(par_n, cfg, 8e6)
    z = np.random.default_rng(0).standard_normal(4000)
    out = np.array([s2.read(8e6, zk) for zk in z])
    sd = out[10:].std()
    assert (abs(out[10:].mean() - 8e6) < 2e3
            and 0.02 * par_n.noise < sd < 0.4 * par_n.noise), sd
    ok(f'slave sensor chain = legacy chain two scans later (max diff '
       f'{err:.1e} Pa); noise rms {sd:.0f} Pa after filtering')


# %% ===== CONTROLLERS ===================================================
def check_wiring():
    par = Params()
    sc = scenarios.stiffstep(par)
    c = ctl.SlaveController(Kp=0.8, StopStiffness=50)
    c.reset(par, sc)
    cfg = par.cfg(G_cfgPressureKp=0.8, G_cfgPressureStopStiffness=50)

    # PRG_Main's call of the block, written out
    f = new_FB_PressureController()
    rng = random.Random(1)
    worst = 0.0
    for k in range(3000):
        if k % 300 == 0:
            ref = rng.choice([-1e6, 5e6, 10e6, 12e6, 20e6])
        meas = 10e6 + 3e6 * math.sin(k / 150.0) + rng.uniform(-1e5, 1e5)
        got = c.step(meas, ref, k * par.Ts, sc)
        f.Enable, f.Reset = True, (k == 0)
        f.Target = min(max(ref * 1e-6, 0.0), cfg['G_cfgPressureTargetMax'])
        f.Actual = meas * 1e-6
        f.Kp, f.Ki = cfg['G_cfgPressureKp'], cfg['G_cfgPressureKi']
        f.VelMax = cfg['G_cfgPressureVelMax']
        f.DecelMax = cfg['G_cfgPressureDecel']
        f.Stiffness = cfg['G_cfgPressureStopStiffness']
        f.StopVelMin = cfg['G_cfgPressureStopVelMin']
        f.CycleTime = cfg['G_cfgScanTime']
        FB_PressureController(f, cfg['G_cfgScanTime'])
        vmax = cfg['G_cfgPressureVelMax']
        want = min(max(f.Velocity, -vmax), vmax) * 1e-3
        worst = max(worst, abs(got - want))
    assert worst < 1e-12, worst
    assert (c.accel == cfg['G_cfgPressureAccel'] * 1e-3
            and c.decel == cfg['G_cfgPressureDecel'] * 1e-3)
    ok('SlaveController output is exactly the FB called as PRG_Main calls '
       'it (target clamp, VelMax clamp, accel/decel)')

    # Flow feedforward adds Q/A after the block.
    c = ctl.SlaveController(flow_ff=True)
    c.reset(par, sc)
    sc2 = scenarios.valve(par, Topen=0.0)
    c.step(10e6, 10e6, 0.0, sc2)
    v = c.step(10e6, 10e6, 2.0, sc2)
    assert abs(v - par.Qf / par.A) < 1e-9, v
    ok(f'flow feedforward adds Q/A ({v * 1e3:.3f} mm/s) after the block')


# %% ===== CLOSED LOOP ===================================================
def check_legacy_reference():
    # Reference values for this case without noise, computed with Heun
    # at nsub = 5
    par = Params().legacy().with_(noise=0.0)
    c = ctl.LegacyController('ref', 'V', 0.5, 0.2, 0, True, 50e-3)
    r = sim.run(par, scenarios.stiffstep(par), c, legacy_sensor=True)
    assert (abs(r['overshoot'] - 0.2005) < 1e-4
            and abs(r['tSettle'] - 1.3120) < 1e-3
            and abs(r['peakErr'] - 2.0) < 1e-4), r.metrics
    ok('legacy plant, sensor and controller match the reference values '
       '(stiffstep overshoot 0.2005 MPa, settle 1.312 s)')


def check_closed_loop():
    par = Params()
    t0 = time.time()
    r = sim.run(par, scenarios.compress(par), ctl.SlaveController())
    dt = time.time() - t0
    inband = slave_config()['G_cfgPressureInBand'] * 1e6    # [Pa]
    late = r.t > r.t[-1] - 5
    err_late = np.abs(r.P[late] - r.target[late])
    assert np.all(err_late <= inband), err_late.max()
    assert (r['overshoot'] < 1.0
            and r['stroke_mm'] < slave_config()['G_cfgPosSoftLimitMax'])
    ok(f'compress with the slave controller reaches the in-band status '
       f'(overshoot {r["overshoot"]:.2f} MPa, stroke '
       f'{r["stroke_mm"]:.1f} mm, 60 s in {dt:.1f} s)')

    r = sim.run(par, scenarios.discharge(par), ctl.SlaveController())
    assert r['peakErr'] < 1.0 and r['rmsErr'] < 0.05
    ok(f'discharge holds the target (peak error {r["peakErr"]:.2f} MPa)')


def check_runner():
    par = Params()
    cases = [Case(par, scenarios.stiffstep(par), ctl.SlaveController(Kp=k))
             for k in (0.5, 1.0)]
    serial = run_cases(cases, workers=1)
    parallel = run_cases(cases, workers=2)
    for a, b in zip(serial, parallel):
        assert a.metrics == b.metrics
    light = run_cases(cases, workers=2, traces=False)
    assert light[0].P is None and light[0].metrics == serial[0].metrics
    ok('parallel runner (spawn) matches serial runs')


if __name__ == '__main__':
    check_config()
    check_stiffness()
    check_conservation()
    check_solver()
    check_actuators()
    check_sensor()
    check_wiring()
    check_legacy_reference()
    check_closed_loop()
    check_runner()
