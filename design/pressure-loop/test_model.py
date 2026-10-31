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
  - the orifice flow, the tube volume step and the end of a delivery;
  - the slew limits, stall and force balance of the actuators;
  - the slave sensor chain against the legacy chain;
  - the wiring of the slave controller against its block called
    directly, and the scheduled controller's stiffness bound,
    integrator continuity and stop law;
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
    assert c['G_cfgScanTime'] == 0.004 and c['G_cfgAnalogMedianSize'] == 5
    par = Params()
    assert (par.Ts == 0.004 and par.vmax == 5e-3 and par.medN == 5
            and par.tauF == 0.02
            and par.cfg()['G_cfgPressureStopVelMin'] == 2.0)
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
def liquid_plant(par, P0=10e6, Qf=0.0, hose_open0=True):
    """Return a velocity-mode plant with a liquid-only chamber at P0
    [Pa] and fast slew limits, and its scenario. Qf prescribes the
    outflow with the valve open; the hose is open unless hose_open0 is
    False."""
    sc = scenarios.stiffstep(par, P0=P0, Qf=Qf, tube=False,
                             hose_open0=hose_open0)
    return Plant(par, sc, 'V', accel=1.0, decel=1.0), sc


def check_stiffness():
    par = Params()
    plant, _ = liquid_plant(par, hose_open0=False)
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
    par = Params()
    # Orifice flow at 10 MPa, eq. (36), and its pressure dependence
    Q10 = par.Q_orifice(10e6)
    expect = par.Cd * par.n_orifice * math.pi * par.d_orifice**2 / 4 \
        * math.sqrt(2 * 10e6 / par.rho)
    assert abs(Q10 - expect) < 1e-15 and abs(Q10 * 1e6 - 4.83) < 0.01
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
    # its dilation full of solution, and reports the volume delivered:
    # the saline and the hose's virgin saline, less what is left. The
    # liquid is stressed by P/K and the delivered volume is measured at
    # pressure, so both carry the factor (1 - P/K).
    r = sim.run(par, scenarios.deliver(par, T=300.0),
                ctl.SlaveController())
    P_end = r.P[-1]
    squeeze = 1 - P_end / par.K            # at the pressure reached
    left = par.V_passages + par.V_hose + par.C_hose * P_end     # [m^3]
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
    # Linearised: dP = P (V_hose / K + C_hose) / (V_l / K + V_hose / K
    # + C_hose) less the hose's own atmosphere start
    C_h = par.V_hose / par.K + par.C_hose
    dP_lin = 10e6 * C_h / (V_l / par.K + C_h)
    assert abs((10e6 - P1) / dP_lin - 1) < 0.05, (P1, dP_lin)
    assert abs(plant.volume_closure()) < 1e-12
    ok(f'opening the hose drops a liquid-only chamber from 10 to '
       f'{P1 / 1e6:.2f} MPa (linear estimate {(10e6 - dP_lin) / 1e6:.2f})')


def check_throttled_opening():
    par = Params()
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
        's', wc=16.0, V_dead_cfg=81e-6, C_hose_cfg=1.2e-13))
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
            par = Params(solver=method)
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
    par = Params()

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
    settle = seen[5 + n_dead + 4]           # 4 scans = 16 ms after start
    assert 0.5 * 0.002 < settle < 0.8 * 0.002, settle
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
    par = Params().with_(noise=0.0)
    cfg = par.cfg()
    assert abs(filter_alpha(cfg) - 0.004 / 0.024) < 1e-15
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
    assert (abs(out[10:].mean() - 8e6 - par_n.P_offset) < 2e3
            and 0.02 * par_n.noise < sd < 0.4 * par_n.noise), sd
    ok(f'slave sensor chain = legacy chain two scans later (max diff '
       f'{err:.1e} Pa); reads {par_n.P_offset / 1e6:+.2f} MPa off; noise '
       f'rms {sd:.0f} Pa after filtering')


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
    par = Params()
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
    # computed with Heun at nsub = 5, a 2 ms scan and a drive lag of
    # 8.7 ms; the case pins both so that it guards the earlier model
    par = Params().legacy().with_(noise=0.0, P_offset=0.0,
                                  G_cfgScanTime=0.002, tauV=0.0087)
    c = ctl.LegacyController('ref', 'V', 0.5, 0.2, 0, True, 50e-3)
    r = sim.run(par, scenarios.stiffstep(par), c, legacy_sensor=True)
    assert (abs(r['overshoot'] - 0.1662) < 1e-4
            and abs(r['tSettle'] - 2.704) < 1e-3
            and abs(r['peakErr'] - 2.0) < 1e-4), r.metrics
    ok('legacy plant, sensor and controller match the reference values '
       '(stiffstep overshoot 0.1662 MPa, settle 2.704 s)')


def check_closed_loop():
    par = Params()
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

    # With the earlier StopVelMin of 0.5 mm/s the stop law caps the
    # piston below the speed the outflow needs, so the loop holds a
    # steady error through the delivery. The hose is open from the
    # start, so the dip is the distal valve's.
    q = par.with_(hose_before=True)
    r = sim.run(q, scenarios.deliver(q),
                ctl.SlaveController('old', StopVelMin=0.5))
    assert 1.5 < r['dipMax'] < 3.0, r['dipMax']
    assert math.isnan(r['tSettle']) and r['V_ok'] < 30, r['V_ok']
    ok(f'delivery 1:1 with StopVelMin 0.5 mm/s: dip {r["dipMax"]:.2f} '
       f'MPa, never back in band, error above 1 MPa after '
       f'{r["V_ok"]:.0f} of {r["V_delivered"]:.0f} mL')
    # With the GVL's StopVelMin of 2 mm/s both loops reach the limit
    # switch with the pressure in band. The hose's compliance caps the
    # end stiffness near 10 MPa/mm even at the smallest dead volume, so
    # neither loop loses margin there. The GVL gains let the pressure
    # through the ceiling when the purge closes the valve against the
    # moving piston; the scheduled loop keeps it below.
    q = par.with_(hose_before=True, V_passages=52e-6)
    relaxed = ctl.SlaveController('r')
    sched = ctl.ScheduledSlaveController('s', wc=11.0, V_dead_cfg=81e-6,
                                         C_hose_cfg=1.2e-13)
    r1 = sim.run(q, scenarios.deliver(q), relaxed)
    r2 = sim.run(q, scenarios.deliver(q), sched)
    k_cap = par.A / ((q.V_dead - q.V_tube) / q.K + q.C_hose)     # [Pa/m]
    assert abs(r1.stiffness[-1] / k_cap - 1) < 0.05, r1.stiffness[-1]
    assert r1['P_max'] > par.P_ceiling / 1e6 > r2['P_max'], \
        (r1['P_max'], r2['P_max'])
    assert r2['V_ok'] == r2['V_delivered']
    assert r1['ripple'] < 0.001 and r2['ripple'] < 0.001
    assert math.isnan(r1['tSettle']) is False and r1['rmsErr'] < 0.01
    ok(f'delivery to a 81 mL dead volume: the hose compliance caps the '
       f'end stiffness at {r1.stiffness[-1] / 1e9:.0f} MPa/mm, both '
       f'loops reach the limit switch in band; the purge closing peaks '
       f'at {r1["P_max"]:.2f} MPa (GVL) and {r2["P_max"]:.2f} MPa '
       f'(scheduled)')


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
    check_outlet()
    check_throttled_opening()
    check_solver()
    check_actuators()
    check_sensor()
    check_wiring()
    check_scheduled()
    check_legacy_reference()
    check_closed_loop()
    check_runner()
