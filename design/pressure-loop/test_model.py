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
  - reading the slave configuration and overriding StudyParams;
  - the sealed liquid chamber stiffness against A K / Vl;
  - conservation of volume and gas moles, with and without outflow;
  - the agreement of the two integration methods, which bounds the
    integration error at the chosen step;
  - the orifice flow, the tube volume step and the end of a delivery;
  - the slew limits, stall and force balance of the actuators;
  - the slave sensor chain against the legacy chain;
  - the wiring of the slave controller against its block called
    directly, and the scheduled controller's stiffness bound,
    integrator continuity and stop law;
  - reference values of the legacy model, and closed-loop runs of the
    slave controller;
  - the stop-law floor policies: the fixed policy reproduces the
    configured floor, the switched latch on synthetic signals, and the
    switched floor with the forward stop on a delivery;
  - the parallel runner against serial runs.

Usage:
    python3 design/pressure-loop/test_model.py

FILE DEPENDENCY:
./
    studyparams.py, controllers.py, runner.py, scenarios.py, sim.py
../../models/
    plant.py, sensor.py, slave.py
../../models/generated/
    FB_PressureController.py, FB_PressureSchedule.py
tests/run_all.sh reruns this script when a file under these paths
changes (paths from the repository root):
Depends on: design/pressure-loop models slave-src tools
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
from studyparams import STUDY_CFG, StudyParams  # noqa: E402
import modelpath  # noqa: F401, E402  (puts models/ on sys.path)
from plant import (Plant, iy_ndis, iy_nout, iy_ntot,  # noqa: E402
                   iy_Vl, iy_Vout, iy_Vvir)
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
    assert c['G_cfgScanTime'] == 0.004
    # The studies hold the settings of STUDY_CFG at their own values,
    # whatever the GVL has; as_configured gives the GVL's.
    par = StudyParams()
    assert all(par.cfg()[k] == v for k, v in STUDY_CFG.items())
    assert (par.Ts == 0.004 and par.vmax == 5e-3 and par.medN == 5
            and par.tauF == 0.02
            and par.cfg()['G_cfgPressureStopVelMin'] == 2.0)
    q = par.with_(as_configured=True)
    assert all(q.cfg()[k] == c[k] for k in STUDY_CFG)
    assert (q.medN == c['G_cfgAnalogMedianSize']
            and q.cfg(G_cfgPressureKp=3.0)['G_cfgPressureKp'] == 3.0)
    q = par.with_(G_cfgPressureKp=2.0, K=2e9)
    assert (q.cfg()['G_cfgPressureKp'] == 2.0 and q.K == 2e9
            and par.cfg()['G_cfgPressureKp'] == 1.5)
    q = StudyParams.from_overrides(['K=2e9', 'nsub=10',
                                    'liquid_depletes=false',
                                    'G_cfgScanTime=0.001', 'solver=euler'])
    assert (q.K == 2e9 and q.nsub == 10 and q.liquid_depletes is False
            and q.Ts == 0.001 and q.solver == 'euler')
    try:
        StudyParams.from_overrides(['nonsense=1'])
    except ValueError:
        pass
    else:
        raise AssertionError('unknown key accepted')
    ok('slave_config reads the GVL (TIME in s); StudyParams overrides '
       'route to fields and G_cfg')


# %% ===== CHAMBER ========================================================
def liquid_plant(par, P0=10e6, Qf=0.0, hose_open0=True):
    """Return a velocity-mode plant with a liquid-only chamber at P0
    [Pa] and fast slew limits, and its scenario. Qf prescribes the
    outflow with the valve open; the hose is open unless hose_open0 is
    False."""
    sc = scenarios.stiffstep(par, P0=P0, Qf=Qf, tube=False,
                             hose_open0=hose_open0)
    return Plant(par, sc, 'V', accel=1.0, decel=1.0), sc


def check_stiffness():
    par = StudyParams()
    plant, _ = liquid_plant(par, hose_open0=False)
    for _ in range(100):                    # reach 0.1 mm/s
        plant.scan(1e-4, 0.0)
    P1, x1 = plant.P, plant.x
    for _ in range(500):
        plant.scan(1e-4, 0.0)
    measured = (plant.P - P1) / (plant.x - x1)              # [Pa/m]
    # Liquid and series compliance, eqs. (4), (10) and (49)
    expected = par.A / (par.Vl0 / par.K + par.C_series)     # [Pa/m]
    assert abs(measured / expected - 1) < 0.02, (measured, expected)
    assert abs(plant.stiffness / expected - 1) < 0.02
    ok(f'sealed liquid chamber stiffness {measured / 1e9:.2f} MPa/mm = '
       f'A / (Vl / K + C_series) ({expected / 1e9:.2f})')


def check_conservation():
    par = StudyParams()

    # Gas-rich chamber at the supply pressure, sealed: the volume closes
    # and no gas is lost.
    sc = scenarios.make('compress', par)
    plant = Plant(par, sc, 'V', accel=1.0, decel=1.0)
    n0 = plant.y[iy_ntot]
    worst = 0.0
    for k in range(round(6 / par.Ts)):      # 6 s at 3 mm/s
        plant.scan(3e-3, 0.0)
        worst = max(worst, abs(plant.volume_closure()))
    assert worst < 1e-8, worst
    assert abs(plant.y[iy_ntot] - n0) < 1e-15
    ok(f'volume closes against piston travel in the gas-rich chamber '
       f'(max {worst:.1e} m^3 of {par.A * plant.x:.1e})')

    # Liquid-only chamber with a prescribed 1 mL/s outflow: the liquid
    # and its dissolved gas leave, and the moles and volume still
    # balance.
    plant, sc = liquid_plant(par, Qf=1e-6)
    plant.y[iy_Vl] += plant.y[iy_Vvir]   # the hose holds solution
    plant.y[iy_Vvir] = 0.0
    n0, Vl0 = plant.y[iy_ntot], plant.y[iy_Vl]
    nd0 = plant.y[iy_ndis]
    Q_int = 0.0                             # [m^3] outflow volume
    worst = 0.0
    for k in range(round(4 / par.Ts)):      # 4 s of 1 mL/s outflow
        plant.scan(1e-6 / par.A, valve=1.0)  # piston follows the flow
        Q_int += 1e-6 * par.Ts
        worst = max(worst, abs(plant.volume_closure()))
    y = plant.y
    removed = Vl0 - y[iy_Vl]                # [m^3] unstressed liquid
    assert abs((y[iy_ntot] + y[iy_nout]) / n0 - 1) < 1e-12, 'gas moles'
    assert abs(removed / (Q_int / (1 - plant.P / par.K)) - 1) < 0.002, \
        (removed, Q_int)
    assert abs(y[iy_ndis] / y[iy_Vl] / (nd0 / Vl0) - 1) < 2e-3, \
        'dissolved gas leaves with the liquid'
    assert worst < 1e-9, worst
    assert abs(y[iy_nout] / (nd0 * removed / Vl0) - 1) < 0.02
    assert abs(y[iy_Vout] - Q_int) < 1e-12
    ok(f'outflow removes liquid ({removed * 1e6:.2f} mL) and its '
       f'dissolved gas; moles and volume close')

    # Earlier model: the liquid volume stays fixed, so the volume does
    # not close under outflow.
    q = par.legacy()
    plant, _ = liquid_plant(q, Qf=1e-6)
    for k in range(round(4 / par.Ts)):      # 4 s
        plant.scan(1e-3, valve=1.0)
    assert (plant.y[iy_Vl] == q.Vl0
            and abs(plant.volume_closure()) > 1e-6 * 0.5)
    ok('legacy plant keeps the liquid volume fixed (volume does not close '
       'under outflow)')


def check_outlet():
    par = StudyParams()
    # Orifice flow at 10 MPa, eq. (36), and its pressure dependence
    Q10 = par.Q_orifice(10e6)
    expect = par.Cd * par.n_orifice * math.pi * par.d_orifice**2 / 4 \
        * math.sqrt(2 * 10e6 / par.rho)
    assert abs(Q10 - expect) < 1e-15 and abs(Q10 * 1e6 - 4.77) < 0.01
    assert abs(par.Q_orifice(2.5e6) / Q10 - 0.5) < 1e-12
    assert par.Q_orifice(-1e5) == 0.0

    # The plant passes the orifice flow with the valve open and the
    # tube's volume in the opening scan, eq. (37): the pressure drops by
    # about V_tube / C on a liquid-only chamber.
    plant, sc = liquid_plant(par)
    C = plant.capacitance()
    plant.scan(0.0, valve=1.0, Q_tube=par.V_tube / par.Ts)
    dP_step = 10e6 - plant.P
    dP_expect = (par.V_tube + Q10 * par.Ts) / C
    assert abs(dP_step / dP_expect - 1) < 0.05, (dP_step, dP_expect)
    assert abs(plant.Q / par.Q_orifice(plant.P) - 1) < 1e-9
    ok(f'outlet: {Q10 * 1e6:.2f} mL/s at 10 MPa; the tube volume drops '
       f'the pressure {dP_step / 1e6:.2f} MPa in the opening scan')

    # The run ends at the limit switch, with the passages, the hose and
    # their dilations full of solution, and reports the volume
    # delivered:
    # the saline and the hose's virgin saline, less what is left. The
    # liquid is stressed by P/K and the delivered volume is measured at
    # pressure, so both carry the factor (1 - P/K).
    r = sim.run(par, scenarios.deliver(par, T=300.0),
                ctl.SlaveController())
    P_end = r.P[-1]
    squeeze = 1 - P_end / par.K            # at the pressure reached
    left = (par.V_passages + par.V_hose
            + (par.C_series + par.C_hose) * P_end)              # [m^3]
    assert r.t[-1] < 299 and r.events[:2] == [1.0, 6.0]
    assert abs(r.V_l[-1] * squeeze - left) < 0.3e-6
    expect = ((par.Vl0 + par.V_hose) * squeeze - left) * 1e6    # [mL]
    assert abs(r['V_delivered'] - expect) < 3, (r['V_delivered'], expect)
    assert abs(r['V_solution'] - (expect - par.V_hose * 1e6)) < 3
    ok(f'delivery ends at the limit switch with {r.V_l[-1] * 1e6:.0f} mL '
       f'of dead volume left: {r["V_delivered"]:.0f} mL in '
       f'{r.t[-1]:.0f} s, {r["V_solution"]:.0f} mL of it solution')

    # Opening the proximal valve equalises the chamber with the hose's
    # virgin saline and compliance, eq. (43): the pressure drops to
    # where the stored compression fills both.
    plant, sc = liquid_plant(par, hose_open0=False)
    V_l = plant.V_l
    plant.open_hose()
    P1 = plant.P
    # Linearised: dP = P C_h / (C_cyl + C_h), with the cylinder's
    # C_cyl = V_l / K + C_series and the hose's C_h = V_hose / K +
    # C_hose, less the hose's own atmosphere start
    C_h = par.V_hose / par.K + par.C_hose
    dP_lin = 10e6 * C_h / (V_l / par.K + par.C_series + C_h)
    assert abs((10e6 - P1) / dP_lin - 1) < 0.05, (P1, dP_lin)
    assert abs(plant.volume_closure()) < 1e-12
    ok(f'opening the hose drops a liquid-only chamber from 10 to '
       f'{P1 / 1e6:.2f} MPa (linear estimate {(10e6 - dP_lin) / 1e6:.2f})')


def check_throttled_opening():
    par = StudyParams()
    # Hose pressure against the volume admitted, eq. (44): the admitted
    # solution and the hose's saline, stressed, fill the bore and its
    # dilation.
    plant, sc = liquid_plant(par, hose_open0=False)
    for V_in in (0.5e-6, 2e-6):
        P_h = plant.hose_pressure(V_in)
        lhs = (par.V_hose + V_in) * (1 - P_h / par.K)
        rhs = par.V_hose + par.C_hose * P_h
        assert abs(lhs - rhs) < 1e-15, (lhs, rhs)
    ok(f'hose pressure: 2 mL admitted raises the hose to '
       f'{plant.hose_pressure(2e-6) / 1e6:.2f} MPa, eq. (44)')

    # A throttled fill of a liquid-only chamber with the piston still
    # ends at the same pressure and admitted volume as the one-scan
    # equalisation, keeps the volume balance, and takes longer through
    # a smaller bore. Dissolution is frozen, or the gas that leaves the
    # falling-pressure solution during a slow fill would lift the
    # pressure above the one-scan result (as it does in the model).
    frozen = par.with_(tauDiss=1e12)
    ref, _ = liquid_plant(frozen, hose_open0=False)
    ref.open_hose()
    fills = {}
    for d in (6.35e-3, 0.3e-3, 0.1e-3):
        q = frozen.with_(d_prox=d, Topen_prox=0.5)
        s = scenarios.stiffstep(q, P0=10e6, tube=False, t_prox=0.0,
                                Topen_prox=0.5)
        p = Plant(q, s, 'V', accel=1.0, decel=1.0)
        worst, t_join = 0.0, None
        for k in range(round(60 / q.Ts)):       # up to 60 s
            t = k * q.Ts
            p.scan(0.0, 0.0, 0.0, s.prox(t))
            worst = max(worst, abs(p.volume_closure()))
            if p.hose_open:
                t_join = t
                break
        assert t_join is not None, d
        assert abs(p.P / ref.P - 1) < 2e-3, (d, p.P, ref.P)
        assert abs(p.V_in_total / ref.V_in_total - 1) < 0.02, d
        assert worst < 1e-9, worst
        assert abs(p.volume_closure()) < 1e-9
        fills[d] = t_join
    assert fills[6.35e-3] <= 0.5 < fills[0.3e-3] < fills[0.1e-3]
    ok(f'throttled fill ends where the one-scan equalisation does '
       f'({p.P / 1e6:.2f} MPa, {p.V_in_total * 1e6:.2f} mL admitted); '
       f'{fills[0.3e-3]:.1f} s through 0.3 mm, {fills[0.1e-3]:.0f} s '
       f'through 0.1 mm')

    # Degassed slug: in a closed-loop delivery at 2:1 the solution that
    # entered while the hose was below the saturation pressure is
    # P_sat (C_hose + V_hose / K) to first order, whatever the bore, and
    # it lies right behind the virgin saline; the purge is sized to it.
    q = par.with_(ratio=2.0, d_prox=0.3e-3, Topen_prox=0.5)
    r = sim.run(q, scenarios.deliver(q), ctl.ScheduledSlaveController(
        's', wc=16.0, V_dead_cfg=109e-6, C_hose_cfg=1.2e-13))
    P_sat = r.P_sat[0]
    V_expect = P_sat * (q.C_hose + q.V_hose / q.K)          # [m^3]
    assert abs(r['V_degas'] / (V_expect * 1e6) - 1) < 0.1, \
        (r['V_degas'], V_expect * 1e6)
    assert r.lumps[0][0] == 'virgin saline' and r.lumps[1][2] is True
    assert abs(r.V_purge * 1e6 - r['purgeNeeded']) < 0.5
    assert r['V_ok'] == r['V_delivered'] and r['t_fill'] > 1.0
    ok(f'2:1 fill through 0.3 mm: {r["V_degas"]:.2f} mL degassed '
       f'(P_sat C_h = {V_expect * 1e6:.2f}), behind the virgin saline; '
       f'purge {r["purgeNeeded"]:.1f} mL, fill {r["t_fill"]:.1f} s, '
       f'margin {r["degasMargin"]:.2f} MPa')


def check_solver():
    # Heun and forward Euler differ by about the local error of the
    # first-order method, so close agreement bounds the integration error
    # at the chosen step. study_convergence.py gives the full picture.
    worst = {}
    for name in ('compress', 'stiffstep', 'deliver'):
        r = {}
        for method in ('euler', 'heun'):
            par = StudyParams(solver=method)
            r[method] = sim.run(par, scenarios.make(name, par),
                                ctl.SlaveController())
        # The delivery is compared only while the loop holds; the
        # emptying chamber takes the GVL loop unstable near the end.
        n = min(len(r['euler'].P), len(r['heun'].P), round(40 / par.Ts))
        dP = np.abs(r['euler'].P[:n] - r['heun'].P[:n]).max()  # [Pa]
        worst[name] = dP
        assert dP < 0.05e6, (name, dP)
    ok('forward Euler and Heun agree: max pressure difference '
       + ', '.join(f'{k} {v / 1e3:.1f} kPa' for k, v in worst.items()))


# %% ===== ACTUATORS =====================================================
def check_actuators():
    par = StudyParams()

    # Slew limits (when enabled): 10 mm/s^2 up and 100 mm/s^2 down,
    # with no stall or dead time
    no_stall = par.with_(Fcap=1e9, slew_limited=True, t_dead=0.0)
    sc = scenarios.stiffstep(par, P0=0.0)
    plant = Plant(no_stall, sc, 'V', accel=0.010, decel=0.100)
    seen = []
    for k in range(1000):
        plant.scan(0.005 if k < 500 else 0.0, 0.0)
        seen.append(plant.v_ref)
    seen = np.array(seen)
    up = (seen[100] - seen[50]) / (50 * par.Ts)             # [m/s^2]
    # The 5 mm/s drop at 100 mm/s^2 lasts 50 ms, 12.5 scans: the
    # window stays inside it
    dn = (seen[508] - seen[502]) / (6 * par.Ts)             # [m/s^2]
    assert abs(up - 0.010) < 1e-4 and abs(dn + 0.100) < 1e-3, (up, dn)
    ok(f'velocity command slews at accel {up * 1e3:.1f} mm/s^2 up and '
       f'decel {-dn * 1e3:.1f} mm/s^2 down')

    # The command reaches the drive after the dead time, then follows
    # with the velocity-loop lag and no slew
    plant = Plant(par, sc, 'V')
    n_dead = round(par.t_dead / par.Ts)
    seen = []
    for k in range(40):
        plant.scan(0.002 if k >= 5 else 0.0)
        seen.append(plant.v)
    assert all(v == 0.0 for v in seen[:5 + n_dead])
    assert seen[5 + n_dead] > 0
    # Four scans after the first that moves, five scans of the lag of
    # eq. (12) have acted
    settle = seen[5 + n_dead + 4]           # 4 scans = 16 ms after start
    lag = 1 - math.exp(-5 * par.Ts / par.tauV)                  # [-]
    assert abs(settle / 0.002 - lag) < 0.01, (settle, lag)
    ok(f'velocity command acts after {n_dead} scans of dead time and '
       f'reaches {settle / 0.002 * 100:.0f} % in 16 ms (tauV '
       f'{par.tauV * 1e3:.1f} ms)')

    # Stall at the torque limit, eq. (13), with the load-dependent and
    # Coulomb friction
    plant = Plant(par, sc, 'V')
    for k in range(round(1.2 / par.Ts)):    # 1.2 s
        plant.scan(0.005)
    P_stall = (par.Fcap - par.Fc) / (par.A * (1 + par.mu))
    assert abs(plant.P - P_stall) / plant.P < 0.03 and plant.v == 0.0
    ok(f'velocity actuator stalls at the torque cap '
       f'({plant.P / 1e6:.1f} MPa)')

    # Torque mode settles where the force balances the pressure, within
    # the friction dead band (Fc + mu P A)
    plant = Plant(par, sc, 'T')
    F = par.A * 10e6 * (1 + par.mu) + 2000.0
    for k in range(round(6 / par.Ts)):      # 6 s
        plant.scan(F)
    P_balance = (F - 2000.0) / (par.A * (1 + par.mu))
    band = (par.Fc + par.mu * P_balance * par.A) / par.A
    assert abs(plant.P - P_balance) < band and abs(plant.v) < 1e-3
    ok(f'torque actuator settles within the friction dead band '
       f'({plant.P / 1e6:.1f} MPa for {P_balance / 1e6:.1f} MPa of '
       f'force)')


# %% ===== SENSOR ========================================================
def check_sensor():
    par = StudyParams().with_(noise=0.0)
    cfg = par.cfg()
    assert abs(filter_alpha(cfg) - 0.004 / 0.024) < 1e-15
    assert filter_alpha(par.cfg(G_cfgAnalogFilterTimeConst=0.0)) == 1.0

    # Without noise, the slave chain equals the legacy chain in the
    # same scan: PRG_Main scales the filter's output of the scan.
    rng = random.Random(3)
    seq = [rng.choice([2e6, 5e6, 9e6]) + rng.uniform(-1e5, 1e5)
           for _ in range(400)]
    slave = SlaveSensor(par, cfg, seq[0])
    leg = LegacySensor(par, cfg, seq[0])
    a = [slave.read(P, 0.0) for P in seq]
    b = [leg.read(P, 0.0) for P in seq]
    err = max(abs(a[k] - b[k]) for k in range(len(seq)))
    assert err < 1.0, err

    # With noise, the filtered reading is unbiased and much quieter than
    # the raw samples.
    par_n = StudyParams()
    s2 = SlaveSensor(par_n, cfg, 8e6)
    z = np.random.default_rng(0).standard_normal(4000)
    out = np.array([s2.read(8e6, zk) for zk in z])
    sd = out[10:].std()
    assert (abs(out[10:].mean() - 8e6 - par_n.P_offset) < 2e3
            and 0.02 * par_n.noise < sd < 0.4 * par_n.noise), sd
    ok(f'slave sensor chain = legacy chain in the same scan (max diff '
       f'{err:.1e} Pa); reads {par_n.P_offset / 1e6:+.2f} MPa off; noise '
       f'rms {sd:.0f} Pa after filtering')


# %% ===== CONTROLLERS ===================================================
def check_wiring():
    par = StudyParams()
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

    # Flow feedforward adds Q/A after the block, with Q the orifice flow
    # at the measured pressure.
    c = ctl.SlaveController(flow_ff=True)
    c.reset(par, sc)
    sc2 = scenarios.deliver(par)
    c.step(10e6, 10e6, 0.0, sc2)
    v = c.step(10e6, 10e6, sc2.t_open + 1.0, sc2)
    assert abs(v - par.Q_orifice(10e6) / par.A) < 1e-9, v
    ok(f'flow feedforward adds Q/A ({v * 1e3:.3f} mm/s) after the block')


# %% ===== CLOSED LOOP ===================================================
def check_scheduled():
    par = StudyParams()
    S = ctl.ScheduledSlaveController

    # The stiffness estimate is an upper bound on the true stiffness
    # when the configured volume and bulk modulus bound the true ones,
    # through a compression and a delivery.
    class Logged(S):
        def reset(self, p, sc):
            self.k_log = []
            return super().reset(p, sc)

        def step(self, P, tg, t, sc, x=0.0):
            u = super().step(P, tg, t, sc, x)
            self.k_log.append(self.k_est)
            return u
    q = par.with_(hose_before=True)     # the hose is open throughout
    for name in ('compress', 'deliver'):
        c = Logged('s', wc=11.0, K_cfg=par.K, V_l_cfg=par.Vl0,
                   C_hose_cfg=par.C_hose)
        r = sim.run(q, scenarios.make(name, q), c)
        ratio = np.array(c.k_log) / r.stiffness
        assert ratio.min() > 0.98, (name, ratio.min())
        assert r['V_ok'] == r['V_delivered']
        assert r['P_max'] < par.P_ceiling / 1e6, r['P_max']
    ok(f'scheduled controller: k_est >= k_true through compress and '
       f'deliver (lowest ratio {ratio.min():.3f}); the 1:1 delivery '
       f'runs to the end with control (hose-opening dip {r["dip"]:.2f} '
       f'MPa, peak {r["P_max"]:.2f} MPa at the purge closing)')

    # Changing Ki leaves the integral term continuous.
    c = S('s')
    sc = scenarios.stiffstep(par)
    c.reset(par, sc)
    c.arrived = True
    for k in range(200):
        c.step(10e6, 10.2e6, k * par.Ts, sc, 0.0)
    fb = c.fb
    integral_term = fb.Ki * fb.rInteg
    fb.Ki, fb.rInteg = 2.0 * fb.Ki, fb.rInteg / 2.0     # same term
    c.step(10e6, 10.2e6, 0.4, sc, 0.0)
    assert abs(fb.Ki * fb.rInteg - integral_term
               - fb.Ki * 0.2 * par.Ts) < 1e-12
    ok('scheduled controller: the integral term is continuous across a '
       'gain change')

    # With lag equal to one scan the stop law is the block's own.
    c = S('s', lag=par.Ts)
    c.reset(par, sc)
    cfg = par.cfg()
    D, Ts = cfg['G_cfgPressureDecel'], cfg['G_cfgScanTime']
    for e, k in ((0.3, 11.4), (2.0, 100.0), (8.0, 1.0)):
        step = D * Ts
        v_fb = -step + math.sqrt(step * step + 2 * D * e / k)
        v_fb = max(cfg['G_cfgPressureStopVelMin'], v_fb)
        assert abs(c.stop_velocity(e, k) - v_fb) < 1e-12
    ok('scheduled controller: the stop law with one scan of lag equals '
       "FB_PressureController's")


def check_legacy_reference():
    # Reference values for this case without noise or transducer offset,
    # computed with Heun at nsub = 5, a 2 ms scan, a drive lag of
    # 8.7 ms and a chamber of K 0.72 GPa with no series compliance; the
    # case pins them so that it guards the earlier model
    par = StudyParams().legacy().with_(noise=0.0, P_offset=0.0,
                                  G_cfgScanTime=0.002, tauV=0.0087,
                                  K=0.72e9, C_series=0.0)
    c = ctl.LegacyController('ref', 'V', 0.5, 0.2, 0, True, 50e-3)
    r = sim.run(par, scenarios.stiffstep(par), c, legacy_sensor=True)
    assert (abs(r['overshoot'] - 0.1662) < 1e-4
            and abs(r['tSettle'] - 2.704) < 1e-3
            and abs(r['peakErr'] - 2.0) < 1e-4), r.metrics
    ok('legacy plant, sensor and controller match the reference values '
       '(stiffstep overshoot 0.1662 MPa, settle 2.704 s)')


def check_closed_loop():
    par = StudyParams()
    t0 = time.time()
    r = sim.run(par, scenarios.compress(par), ctl.SlaveController())
    dt = time.time() - t0
    inband = slave_config()['G_cfgPressureInBand'] * 1e6    # [Pa]
    late = r.t > r.t[-1] - 5
    # The slave judges its status band on the measured pressure; the
    # true pressure sits above it by the transducer's zero error.
    err_late = np.abs(r.P_meas[late] - r.target[late])
    assert np.all(err_late <= inband), err_late.max()
    assert abs(np.mean(r.P[late] - r.target[late]) + par.P_offset) < 2e4
    assert (r['P_max'] < par.P_ceiling / 1e6
            and r['stroke_mm'] < slave_config()['G_cfgPosSoftLimitMax'])
    ok(f'compress 1:1 from the supply pressure with the slave controller '
       f'reaches the in-band status, the true pressure '
       f'{-par.P_offset / 1e6:.2f} MPa above the target (overshoot '
       f'{r["overshoot"]:.2f} MPa, stroke {r["stroke_mm"]:.1f} mm, 60 s '
       f'in {dt:.1f} s)')

    # With the earlier settings (Kp 0.5, Ki 0.2, a stop-law stiffness
    # of 100 MPa/mm and a StopVelMin of 0.5 mm/s) the stop law caps the
    # piston below the speed the outflow needs, so the loop holds a
    # steady error through the delivery. The hose is open from the
    # start, so the dip is the distal valve's.
    EARLIER = {'Kp': 0.5, 'Ki': 0.2, 'StopStiffness': 100.0}
    q = par.with_(hose_before=True)
    r = sim.run(q, scenarios.deliver(q),
                ctl.SlaveController('old', StopVelMin=0.5, **EARLIER))
    assert 1.5 < r['dipMax'] < 3.0, r['dipMax']
    assert math.isnan(r['tSettle']) and r['V_ok'] < 30, r['V_ok']
    ok(f'delivery 1:1 with the earlier gains and StopVelMin 0.5 mm/s: '
       f'dip {r["dipMax"]:.2f} MPa, never back in band, error above '
       f'1 MPa after {r["V_ok"]:.0f} of {r["V_delivered"]:.0f} mL')
    # With a StopVelMin of 2 mm/s the loops reach the limit switch with
    # the pressure in band. The hose's and the cylinder's compliance cap
    # the end stiffness even at the smallest dead volume, so no loop
    # loses margin there. The earlier gains let the pressure through
    # the ceiling when the purge closes the valve against the moving
    # piston; the GVL gains keep it below, and the scheduled loop does
    # no better than they do.
    q = par.with_(hose_before=True, V_passages=52e-6)
    sched = ctl.ScheduledSlaveController('s', wc=11.0, V_dead_cfg=109e-6,
                                         C_hose_cfg=1.2e-13)
    r0 = sim.run(q, scenarios.deliver(q),
                 ctl.SlaveController('e', **EARLIER))
    r1 = sim.run(q, scenarios.deliver(q), ctl.SlaveController('g'))
    r2 = sim.run(q, scenarios.deliver(q), sched)
    k_cap = par.A / ((q.V_dead - q.V_tube) / q.K + q.C_series
                     + q.C_hose)                                 # [Pa/m]
    assert abs(r1.stiffness[-1] / k_cap - 1) < 0.05, r1.stiffness[-1]
    assert r0['P_max'] > par.P_ceiling / 1e6 > r1['P_max'], \
        (r0['P_max'], r1['P_max'])
    assert abs(r1['P_max'] - r2['P_max']) < 0.1, r2['P_max']
    assert abs(r1['dipMax'] - r2['dipMax']) < 0.1, r2['dipMax']
    assert r1['V_ok'] == r1['V_delivered']
    assert r2['V_ok'] == r2['V_delivered']
    assert r1['ripple'] < 0.001 and r2['ripple'] < 0.001
    assert math.isnan(r1['tSettle']) is False and r1['rmsErr'] < 0.01
    ok(f'delivery to a {(q.V_dead - q.V_tube) * 1e6:.0f} mL dead volume: '
       f'the end stiffness is capped at {r1.stiffness[-1] / 1e9:.0f} '
       f'MPa/mm and the loops reach the limit switch in band; the '
       f'purge closing peaks at {r0["P_max"]:.2f} MPa with the earlier '
       f'gains, {r1["P_max"]:.2f} with the GVL gains and '
       f'{r2["P_max"]:.2f} scheduled')


# %% ===== STOP-LAW FLOOR POLICIES =======================================
def check_floor_block():
    """FB_PressureController's own floor and forward stop (translated
    from ST) against FloorPolicy's switched floor with the forward
    stop, eqs. (46) and (48)."""
    class Lockstep:
        """Runs the reference (the block with a FloorPolicy setting its
        floor) and the block with its own floor on the same inputs,
        and returns the reference's command."""
        mode = 'V'

        def __init__(self, **cfg):
            self.ref = ctl.SlaveController('ref', floor=ctl.FloorPolicy(
                'switched', forward_stop=True), **cfg)
            self.port = ctl.SlaveController('port', FloorEnable=True,
                                            **cfg)
            self.name, self.cfg = 'port', self.port.cfg

        def reset(self, p, sc):
            u = self.ref.reset(p, sc)
            assert self.port.reset(p, sc) == u
            assert self.port.fb.FloorEnable
            assert not self.ref.fb.FloorEnable
            self.accel, self.decel = self.ref.accel, self.ref.decel
            self.worst, self.latches, self.releases = 0.0, 0, 0
            self.floors = set()
            return u

        def step(self, P, tg, t, sc, x=0.0):
            was = self.port.fb.Latched
            a = self.ref.step(P, tg, t, sc, x)
            b = self.port.step(P, tg, t, sc, x)
            self.worst = max(self.worst, abs(a - b))
            q = self.port.fb
            assert not q.FloorError
            assert q.Latched == self.ref.floor.latched, t
            assert q.rFloor == self.ref.fb.StopVelMin, t
            self.latches += q.Latched and not was
            self.releases += was and not q.Latched
            self.floors.add(q.rFloor)
            return a

    # Scan for scan through the compression and the 1:1 delivery with
    # both hose procedures and a 1 s closing, at the GVL gains and at
    # the earlier, lower ones
    par = StudyParams()
    c = par.cfg()
    cases = (('hose before', par.with_(hose_before=True)),
             ('hose after', par),
             ('1 s closing', par.with_(hose_before=True, Tclose=1.0)))
    for label, q in cases:
        for cfg in ({}, {'Kp': 0.5, 'Ki': 0.2, 'StopStiffness': 100.0}):
            L = Lockstep(**cfg)
            sim.run(q, scenarios.compress(q), L)
            assert L.worst == 0.0, (label, 'compress', L.worst)
            r = sim.run(q, scenarios.deliver(q), L)
            assert L.worst == 0.0, (label, 'deliver', L.worst)
            # At the GVL gains the loop stops the piston through a slow
            # closing before the pressure rises fast enough to release
            # the latch, so only the other cases must release it.
            slow = label == '1 s closing' and not cfg
            assert L.latches >= 1 and (L.releases >= 1 or slow), \
                (label, L.latches, L.releases)
            assert L.floors == {c['G_cfgPressureFloorVelLow'],
                                c['G_cfgPressureStopVelMin']}
            assert r['V_ok'] == r['V_delivered'], label
    ok("FB_PressureController's floor and forward stop match "
       'FloorPolicy scan for scan through the compression and the 1:1 '
       'delivery, both hose procedures and a 1 s closing: the latch '
       'sets on the outflow and releases at the closing')

    # A floor setting out of range leaves the configured StopVelMin and
    # no forward stop: the run is the run with the floor off.
    q = par.with_(hose_before=True)
    sc = scenarios.deliver(q)
    bad = ctl.SlaveController('bad', FloorEnable=True, FloorWindow=1.0)
    r0, r1 = sim.run(q, sc, ctl.SlaveController()), sim.run(q, sc, bad)
    assert bad.fb.FloorError and not bad.fb.Latched
    assert np.array_equal(r0.u, r1.u)
    ok('a floor window longer than the ring is a FloorError: the loop '
       'runs on the configured StopVelMin')


def check_floor_policy():
    """FloorPolicy, eqs. (46) to (48)."""
    par = StudyParams().with_(hose_before=True)
    c = par.cfg()

    # The fixed policy at the configured floor, without the forward
    # stop, is the configured StopVelMin: identical runs.
    s = scenarios.compress(par, T=8.0)
    r0 = sim.run(par, s, ctl.SlaveController('gvl'))
    fixed = ctl.FloorPolicy('fixed', v_hi=c['G_cfgPressureStopVelMin'])
    r1 = sim.run(par, s, ctl.SlaveController('fixed', floor=fixed))
    assert np.array_equal(r0.u, r1.u) and np.array_equal(r0.P, r1.P)

    # Synthetic signals, one scan of Ts at a time: the piston moves at
    # 1.4 mm/s with the block clamped forward and the error 1 MPa.
    Ts = c['G_cfgScanTime']                                     # [s]
    f = ctl.FloorPolicy()
    f.reset(c)
    n_lag = f.W + f.d                       # [-] scans to fill history

    def feed(n, P0, rate, err, x0, v=1.4):
        """Feed n scans with the pressure rising at rate [MPa/s] from
        P0 [MPa] and the piston at v [mm/s] from x0 [mm]; return the
        last pressure and position."""
        P, x = P0, x0
        for _ in range(n):
            f.step(P, x, err, True, v)
            P += rate * Ts
            x += v * Ts
        return P, x
    # Sealed: the pressure rises at k v = 4.8 * 1.4 MPa/s, no latch.
    P, x = feed(n_lag + 200, 9.0, 4.8 * 1.4, 1.0, 0.0)
    assert not any(f.trace_latched) and f.floor == f.v_lo
    # Outflow balances the piston: the pressure is flat. The latch sets
    # once the flat window has passed the lag and held for t_on.
    i0 = len(f.trace_latched)
    P, x = feed(n_lag + f.n_on + 5, P, 0.0, 1.0, x)
    i_on = f.trace_latched.index(True, i0) - i0
    assert f.floor == f.v_hi
    assert f.W <= i_on <= n_lag + f.n_on + 1, i_on
    # Recovery from below the target with the pressure rising: held.
    P, x = feed(100, P, 2.0, 0.5, x)
    assert f.latched
    # The outlet shuts above the target: released.
    P, x = feed(f.W, P, 6.7, -0.05, x)
    assert not f.latched and f.floor == f.v_lo
    # The forward stop holds the piston only while unlatched and above
    # the target.
    g = ctl.FloorPolicy(forward_stop=True)
    g.reset(c)
    assert g.limit_forward(1.4, -0.01) == 0.0
    assert g.limit_forward(-1.0, -0.01) == -1.0
    assert g.limit_forward(1.4, 0.01) == 1.4
    g.latched = True
    assert g.limit_forward(1.4, -0.01) == 1.4

    # Closed loop, 1:1 with the hose open before the compression: the
    # switched floor with the forward stop sets no latch during the
    # compression, leaves no steady error in the delivery, and stops
    # the piston when the purge shuts the valve, so the closing peak is
    # well below that of the fixed floor of the GVL.
    pol = dict(mode='switched', forward_stop=True)
    sw = ctl.SlaveController('sw', floor=ctl.FloorPolicy(**pol))
    rc = sim.run(par, scenarios.compress(par), sw)
    assert not any(sw.floor.trace_latched)
    assert rc['overshoot'] < 0.2, rc['overshoot']
    sw = ctl.SlaveController('sw', floor=ctl.FloorPolicy(**pol))
    rd = sim.run(par, scenarios.deliver(par), sw)
    rg = sim.run(par, scenarios.deliver(par), ctl.SlaveController('g'))
    late = (rd.t > rd.events[-1] + 5) & (rd.t < rd.t[-1] - 1)
    e = (rd.target[late] - rd.P_meas[late]) / 1e6               # [MPa]
    assert abs(np.mean(e)) < 0.05 and np.std(e) < 0.1, (e.mean(),
                                                         e.std())
    assert par.P_ceiling / 1e6 > rg['P_max'] > rd['P_max'] + 0.3
    assert rd['V_ok'] == rd['V_delivered']
    ok(f'floor policies: fixed = configured StopVelMin; switched '
       f'latch sets {i_on} scans into a flat window, holds through a '
       f'recovery, releases on a rise above the target; with the '
       f'forward stop, compression overshoot {rc["overshoot"]:.2f} MPa '
       f'and the purge closing peaks at {rd["P_max"]:.2f} MPa '
       f'(GVL {rg["P_max"]:.2f})')


def check_integ_clear_block():
    """FB_PressureController's own integral clear past the target
    (translated from ST) against the 'opposed' rule of
    VariantController, eq. (52)."""
    class Lockstep:
        """Runs the reference (the block with the rule applied around
        it) and the block with its own rule on the same inputs, and
        returns the reference's command."""
        mode = 'V'

        def __init__(self, over, **cfg):
            self.ref = ctl.VariantController('ref', aw='opposed',
                                             over=over, **cfg)
            self.port = ctl.SlaveController(
                'port', IntegClearEnable=True, IntegClearError=over,
                **cfg)
            self.name, self.cfg = 'port', self.port.cfg

        def reset(self, p, sc):
            u = self.ref.reset(p, sc)
            assert self.port.reset(p, sc) == u
            assert self.port.fb.IntegClearEnable
            assert not self.ref.fb.IntegClearEnable
            self.accel, self.decel = self.ref.accel, self.ref.decel
            self.worst = 0.0
            return u

        def step(self, P, tg, t, sc, x=0.0):
            a = self.ref.step(P, tg, t, sc, x)
            b = self.port.step(P, tg, t, sc, x)
            self.worst = max(self.worst, abs(a - b))
            assert self.port.fb.rInteg == self.ref.fb.rInteg, t
            return a

    # Scan for scan through the soft chamber stepped to 5 atm, the
    # compression and the 1:1 delivery (hose opened before the
    # compression), at the configured threshold and a wider one, with
    # the block's floor off and on
    par = StudyParams()
    q = par.with_(hose_before=True)
    soft = scenarios.Scenario('soft', T=60.0, tEvent=0.0, P0=0.0,
                              ratio=0.98, target0=0.405e6,
                              target1=0.405e6)
    runs = ((par.with_(Vl0=279e-6), soft), (q, scenarios.compress(q)),
            (q, scenarios.deliver(q)))
    clears = 0
    for over in (0.02, 0.1):
        for cfg in ({}, {'FloorEnable': True}):
            for p, sc in runs:
                L = Lockstep(over, **cfg)
                sim.run(p, sc, L)
                assert L.worst == 0.0, (over, cfg, sc.name, L.worst)
                clears += len(L.ref.changed)
    assert clears >= 6, clears

    # Off, the default, the block is the block without the rule; on, it
    # removes the soft chamber's overshoot.
    c = par.cfg()
    assert not c['G_cfgPressureIntegClearEnable']
    p, sc = runs[0]
    r0 = sim.run(p, sc, ctl.SlaveController())
    r1 = sim.run(p, sc, ctl.SlaveController(IntegClearEnable=True))
    over0 = float(np.max(r0.P - r0.target)) / 1e6           # [MPa]
    over1 = float(np.max(r1.P - r1.target)) / 1e6           # [MPa]
    assert over0 > 0.3 and over1 < 0.05, (over0, over1)

    # A negative threshold with the clear enabled is a parameter error:
    # the block commands zero.
    bad = ctl.SlaveController('bad', IntegClearEnable=True,
                              IntegClearError=-0.01)
    bad.reset(par, sc)
    assert bad.step(0.0, 1e6, 0.0, sc) == 0.0 and bad.fb.bParamError
    ok(f"FB_PressureController's integral clear matches the 'opposed' "
       f'rule scan for scan through the soft step, the compression '
       f'and the delivery ({clears} clears); soft chamber stepped to '
       f'5 atm: overshoot {over0:.2f} MPa off, {over1:.2f} MPa on')


def check_variants():
    par = StudyParams()
    # The unchanged variant is the slave controller, scan for scan.
    sc = scenarios.compress(par, T=20.0)
    a = sim.run(par, sc, ctl.SlaveController())
    b = sim.run(par, sc, ctl.VariantController())
    assert np.array_equal(a.u, b.u)
    ok('VariantController with no change gives the SlaveController '
       'command exactly through a compression')

    def drive(c, readings, target=10e6):
        """Step controller c through the readings [Pa] and return the
        integral [MPa s] after each scan."""
        c.reset(par, sc)
        out = []
        for k, P in enumerate(readings):
            c.step(P, target, k * par.Ts, sc)
            out.append(c.fb.rInteg)
        return out

    # Readings 0.05 MPa below the target for 1 s, then above it: the
    # integral built below is cleared (or bled) once the error is past
    # the target by more than `over`; a target step re-arms 'arrival'.
    below, above = [9.95e6] * 250, [10.05e6] * 50
    I = drive(ctl.VariantController(), below + above)
    assert I[249] > 0.04 and I[-1] > 0.03, (I[249], I[-1])
    I = drive(ctl.VariantController(aw='opposed'), below + above)
    assert I[249] > 0.04 and I[250] < 0.0 and I[-1] < 0.0
    I = drive(ctl.VariantController(aw='crossing'), below + above)
    assert I[249] > 0.04 and I[250] < 0.0
    I = drive(ctl.VariantController(aw='bleed', Tb=0.1), below + above)
    assert 0.0 < I[-1] < 0.5 * I[249], (I[249], I[-1])
    # 'arrival' acts only after an approach: none starts within `step`
    # of the target, and one from 1 MPa below clears once and no more.
    I = drive(ctl.VariantController(aw='arrival'), below + above)
    assert I[-1] > 0.03, I[-1]
    c = ctl.VariantController(aw='arrival', StopVelMin=0.0)
    far = [9.0e6] * 250
    I = drive(c, far + above + below + above)
    assert I[249] > 0.2 and I[250] < 0.0, (I[249], I[250])
    assert len(c.changed) == 1 and 0.0 < I[-1] < I[550], I[-1]
    # The integration band holds the integral outside the band.
    I = drive(ctl.VariantController(aw='band', band=0.1),
              [9.0e6] * 100 + below)
    assert I[99] == 0.0 and I[-1] > 0.04
    # Clamped from the first scan, the block holds the integral at
    # zero; tracking settles it where u_raw - u = Ki Tt e, eq. (54).
    hold = drive(ctl.VariantController(Ki=5.0), [8.0e6] * 250)
    c = ctl.VariantController(aw='backcalc', Tt=0.1, Ki=5.0)
    track = drive(c, [8.0e6] * 250)
    # [MPa s] (u + Ki Tt e - Kp e) / Ki at e = 2 MPa
    want = (c.fb.rVelLimit + 5.0 * 0.1 * 2.0 - c.fb.Kp * 2.0) / 5.0
    assert hold[-1] == 0.0 and abs(track[-1] - want) < 1e-3, track[-1]
    ok('anti-windup variants: opposed, crossing and bleed act past the '
       'target, arrival once per approach, band outside its band, '
       'tracking while clamped')

    # The derivative term opposes a rising pressure.
    ramp = [9.0e6 + 2e3 * k for k in range(100)]        # 0.5 MPa/s
    c0, c1 = ctl.VariantController(), ctl.VariantController(Kd=0.5)
    c0.reset(par, sc)
    c1.reset(par, sc)
    for k, P in enumerate(ramp):
        v0 = c0.step(P, 10e6, k * par.Ts, sc)
        v1 = c1.step(P, 10e6, k * par.Ts, sc)
    assert abs((v0 - v1) * 1e3 - 0.5 * 0.5) < 0.01, (v0, v1)
    ok(f'derivative term: Kd 0.5 on a 0.5 MPa/s rise takes '
       f'{(v0 - v1) * 1e3:.2f} mm/s off the command')

    # Target schedule of a scenario
    s3 = scenarios.Scenario('s', T=3.0, tEvent=0.0, target0=1e6,
                            targets=((1.0, 2e6), (2.0, 5e6)))
    assert [s3.target(t) for t in (0.5, 1.0, 1.9, 2.5)] == \
        [1e6, 2e6, 2e6, 5e6]

    # Soft chamber: with free gas at a low pressure the integrator
    # carries the piston to the target and the block's own anti-windup
    # leaves its velocity to be unwound above it; clearing it past the
    # target removes the overshoot.
    q = par.with_(Vl0=279e-6)
    soft = scenarios.Scenario('soft', T=60.0, tEvent=0.0, P0=0.0,
                              ratio=0.98, target0=0.405e6,
                              target1=0.405e6)
    over = {}
    for aw in ('block', 'opposed'):
        r = sim.run(q, soft, ctl.VariantController(aw=aw))
        over[aw] = float(np.max(r.P - r.target)) / 1e6      # [MPa]
    assert over['block'] > 0.3 and over['opposed'] < 0.05, over
    ok(f'soft chamber stepped to 5 atm: overshoot {over["block"]:.2f} '
       f'MPa with the block alone, {over["opposed"]:.2f} MPa with the '
       f'integral cleared past the target')


def check_feedforward():
    """FeedforwardController's detector, eqs. (64) and (65), and its
    feedforward, eq. (66), on readings laid out by hand and in the
    delivery."""
    par = StudyParams()
    sc = scenarios.compress(par)

    def drive(c, readings):
        """Step c through readings [Pa] at a 10 MPa target and return
        the detector's state after each scan and the last command."""
        c.reset(par, sc)
        on = []
        for k, P in enumerate(readings):
            v = c.step(P, 10e6, k * par.Ts, sc)
            on.append(c.on)
        return on, v

    hold = [10e6] * 20
    # An outflow of 4.5 MPa/s: 18 kPa a scan
    fall = [10e6 - 18e3 * k for k in range(1, 16)]
    c = ctl.FeedforwardController()
    on, v = drive(c, hold + fall)
    k_on = on.index(True)
    assert not any(on[:20]) and 21 <= k_on <= 26, k_on
    # The command is the block's with c sqrt(T) added, eq. (66).
    b = ctl.SlaveController()
    b.reset(par, sc)
    for k, P in enumerate(hold + fall):
        vb = b.step(P, 10e6, k * par.Ts, sc)
    v_ff = par.Q_orifice(10e6) / par.A                      # [m/s]
    assert abs((v - vb) - v_ff) < 1e-9, (v, vb, v_ff)

    # The hose joining the chamber: 1 MPa a scan, far too deep
    deep = [10e6 - 1e6 * min(k, 6) for k in range(1, 40)]
    on, _ = drive(ctl.FeedforwardController(), hold + deep)
    assert not any(on)

    # Off: a sharp rise past the target; a slow one only beyond e_hi
    rise = [fall[-1] + 20e3 * k for k in range(1, 20)]
    on, _ = drive(ctl.FeedforwardController(), hold + fall + rise)
    k_off = len(on) - 1 - on[::-1].index(True) + 1
    P_off = (hold + fall + rise)[k_off]
    assert not on[-1] and 10.02e6 < P_off < 10.1e6, P_off
    back = [fall[-1] + 20e3 * k for k in range(1, 14)]      # to 9.99
    creep = [back[-1] + 2e3 * k for k in range(1, 100)]     # to 10.19
    on, _ = drive(ctl.FeedforwardController(),
                  hold + fall + back + creep)
    assert on[-1]
    on, _ = drive(ctl.FeedforwardController(),
                  hold + fall + back + creep + [10.35e6] * 3)
    assert not on[-1]

    # In the delivery it switches on at each opening and off at the
    # closing, with either hose procedure, and halves the dip.
    for q in (par.with_(hose_before=True), par):
        d = scenarios.deliver(q)
        c = ctl.FeedforwardController()
        r = sim.run(q, d, c)
        flow = r.Qout > 0.0
        assert len(c.ons) == 2 and len(c.offs) == 1, (c.ons, c.offs)
        assert all(flow[k] for k in c.ons)
        assert r.events[-2] < c.offs[0] * q.Ts < r.events[-2] + 0.1
    r0 = sim.run(q.with_(hose_before=True),
                 scenarios.deliver(q.with_(hose_before=True)),
                 ctl.SlaveController())
    c = ctl.FeedforwardController()
    r1 = sim.run(q.with_(hose_before=True),
                 scenarios.deliver(q.with_(hose_before=True)), c)
    assert r1['dipMax'] < 0.6 * r0['dipMax'], (r0['dipMax'],
                                               r1['dipMax'])
    ok(f'outflow feedforward: on {k_on - 20} scans into a 4.5 MPa/s '
       f'fall, not on a fall of 1 MPa a scan, off on a sharp rise past '
       f'the target; in the delivery, on at both openings and off at '
       f'the closing, dip {r1["dipMax"]:.2f} MPa '
       f'({r0["dipMax"]:.2f} without)')


def check_runner():
    par = StudyParams()
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
    check_outlet()
    check_throttled_opening()
    check_solver()
    check_actuators()
    check_sensor()
    check_wiring()
    check_scheduled()
    check_legacy_reference()
    check_closed_loop()
    check_floor_policy()
    check_floor_block()
    check_integ_clear_block()
    check_variants()
    check_feedforward()
    check_runner()
