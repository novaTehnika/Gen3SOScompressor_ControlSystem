#!/usr/bin/env python3
"""
test_linear.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script checks the linear model of the pressure loop (linear.py)
against scipy and against the nonlinear simulation. It prints "ok" and a
short result for each check and stops at the first failure.
tests/run_all.sh runs it.

The simulations run without sensor noise, in the liquid-only chamber, at
amplitudes small enough that the output clamp and the slew limit do not
act, so the linear model should match them closely. The checks are:

  plant         The zero-order-hold plant matches scipy's
                cont2discrete, and the delay count is as expected.
  margins       Scaling the loop gain by the gain margin puts a pole on
                the unit circle, the delay margin equals pm / wc, and the
                loop depends on Kp and k only through their product when
                Ki scales with Kp.
  V1 step       The closed-loop response to a small target step matches
                the simulation, and only the expected delay count fits.
  V2 boundary   At 0.8, 1.05 and 1.2 times the predicted largest stable
                Kp, the simulated oscillation decays or grows at the rate
                and frequency of the predicted closed-loop pole.
  V3 outflow    The pressure dip after an outflow step matches the
                simulation in size, time and shape.
  V4 derivative A target step with a derivative term on the measured
                pressure matches the simulation, and the command noise
                matches the noise gain.
  V5 soft       In a soft chamber the least damped mode follows the
                two-integrator reduction, eq. (60), and the overshoot
                from a stored integral velocity follows eq. (61).

Usage:
    python3 design/pressure-loop/test_linear.py

FILE DEPENDENCY:
./
    studyparams.py, controllers.py, linear.py, scenarios.py, sim.py
tests/run_all.sh reruns this script when a file under these paths
changes (paths from the repository root):
Depends on: design/pressure-loop models slave-src tools
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from scipy import signal  # noqa: E402

import controllers as ctl  # noqa: E402
import linear as lin  # noqa: E402
import scenarios  # noqa: E402
import sim  # noqa: E402
from studyparams import StudyParams  # noqa: E402

# %% ===== TEST CONDITIONS ===============================================
# No noise and no transducer offset: the linear model is about the
# response to a step, and the offset would shift the true pressure
# against which the step is measured.
PAR = StudyParams().with_(noise=0.0, P_offset=0.0)
# [Pa/m] Liquid-only chamber stiffness, eqs. (4), (10) and (49)
K_LIQ = PAR.A / (PAR.Vl0 / PAR.K + PAR.C_series)
CFG = PAR.cfg()


def ok(msg):
    print(f'ok  {msg}')


def run(kp, T, **sc):
    """Simulate the stiffness step for T [s] with gain kp, with scenario
    fields replaced by sc. Returns (Result, Scenario)."""
    s = scenarios.stiffstep(PAR, T=T, **sc)
    return sim.run(PAR, s, ctl.SlaveController(Kp=kp)), s


def model_trace(tf, n, n0, amp):
    """Return the linear response of tf to a step of size amp at scan n0,
    aligned with the simulation traces (entry j is the state after scan
    j)."""
    y = np.concatenate([np.zeros(n0), tf.step(n - n0 + 1, amp)])
    return y[1:n + 1]


# %% ===== PLANT AND MARGINS =============================================
def check_plant_discretisation():
    k = 3e9                                                 # [Pa/m]
    lp = lin.loop(PAR, k)
    num, den, _ = signal.cont2discrete(([k], [PAR.tauV, 1.0, 0.0]),
                                       PAR.Ts, 'zoh')[:3]
    assert np.allclose(np.trim_zeros(np.ravel(num), 'f'), lp['Gd'].num,
                       rtol=1e-9)
    assert np.allclose(den, lp['Gd'].den, rtol=1e-9)
    n_dead = round(PAR.t_dead / PAR.Ts)
    assert lp['delay'] == 5 + n_dead and lin.sample_delay(CFG) == 4
    ok(f'plant ZOH matches scipy cont2discrete; delay = median 2 + '
       f'pipeline 2 + command 1 + drive dead time {n_dead} scans')


def check_margins():
    lp = lin.loop(PAR, K_LIQ)
    m = lin.margins(lp['L'])
    assert lin.is_stable(lp) and m['gm'] > 1 and m['pm'] > 0

    # Gain margin: scaling the loop gain by gm puts a closed-loop pole on
    # the unit circle. The loop gain is proportional to k.
    kp = CFG['G_cfgPressureKp']
    edge = lin.loop(PAR, K_LIQ * m['gm'])
    assert abs(np.abs(lin.poles(edge)).max() - 1) < 1e-6

    # Delay margin: adding dm of delay removes the phase margin at the
    # same crossover.
    assert abs(m['dm'] * m['wc'] - np.radians(m['pm'])) < 1e-12

    # Kp and k enter only as Kp k when Ki scales with Kp.
    a = lin.margins(lin.loop(PAR, K_LIQ, G_cfgPressureKp=kp,
                             G_cfgPressureKi=0.4 * kp)['L'])
    b = lin.margins(lin.loop(PAR, 2 * K_LIQ, G_cfgPressureKp=kp / 2,
                             G_cfgPressureKi=0.2 * kp)['L'])
    assert abs(a['pm'] - b['pm']) < 1e-6 and abs(a['gm'] - b['gm']) < 1e-6
    ok(f'margins at the GVL gains, liquid chamber: '
       f'GM {m["gm_db"]:.1f} dB, '
       f'PM {m["pm"]:.1f} deg at {m["wc"]:.2f} rad/s, '
       f'DM {m["dm"] * 1e3:.0f} ms; gain scaling reaches the unit circle; '
       f'L depends on Kp k')


# %% ===== V1: TARGET STEP ===============================================
def check_v1_step():
    """Closed-loop target step, linear against nonlinear, and the delay
    count."""
    amp = 0.05e6                            # [Pa] step size
    kp = CFG['G_cfgPressureKp']
    r, s = run(kp, 6.0, target1=10e6 + amp)
    n0 = round(s.t_step / PAR.Ts)           # [-] scan of the step

    # The clamp and slew limit do not act: |Kp e| stays below StopVelMin
    # (the lowest the clamp goes), and the step's one-scan command jump
    # stays below Accel Ts.
    umax = np.abs(r.u).max() * 1e3                          # [mm/s]
    assert (umax < CFG['G_cfgPressureStopVelMin']
            and lin.slew_step_error(CFG, kp) > amp / 1e6), umax

    # Fit the sensor-path delay: only the expected count should match.
    n = len(r.t)
    errs = {}
    for d in range(2, 8):
        R = lin.loop(PAR, K_LIQ, delay=d)['R']
        y = model_trace(R, n, n0, amp)
        errs[d] = np.abs(r.P - 10e6 - y).max() / amp
    best = min(errs, key=errs.get)
    assert best == lin.sample_delay(CFG) and errs[best] < 0.005, errs
    assert min(v for d, v in errs.items() if d != best) > 3 * errs[best], \
        errs
    ok(f'V1 step response: linear vs sim max difference '
       f'{errs[best] * 100:.2f}% of the step (delay {best} + 1 scans fits '
       f'best; {best - 1} gives {errs[best - 1] * 100:.1f}%, {best + 1} '
       f'gives {errs[best + 1] * 100:.1f}%); max |u| {umax:.3f} mm/s, '
       f'clamp not active')


# %% ===== V2: STABILITY BOUNDARY ========================================
def osc_pole(lp):
    """Return the oscillatory closed-loop pole of largest magnitude."""
    return max((z for z in lin.poles(lp) if abs(z.imag) > 1e-3), key=abs)


def peaks_and_freq(r, t0, lo, hi):
    """Measure the oscillation of the error after t0 [s] between
    amplitudes lo and hi [Pa].

    Returns (growth rate [1/s] from a fit to the log of the |e| peaks,
    angular frequency [rad/s] from the zero crossings, number of peaks
    used).
    """
    e = r.P - r.target
    a = np.abs(e)
    pk = np.array([i for i in range(1, len(a) - 1)
                   if a[i] >= a[i - 1] and a[i] > a[i + 1]
                   and r.t[i] > t0])
    sel = (a[pk] > lo) & (a[pk] < hi)
    t = r.t[pk][sel]
    rate = np.polyfit(t, np.log(a[pk][sel]), 1)[0]
    zi = np.nonzero(e[:-1] * e[1:] < 0)[0]
    zt = r.t[zi]
    zt = zt[(zt > t[0]) & (zt < t[-1])]
    # Zero crossings come every half period.
    return rate, np.pi / np.mean(np.diff(zt)), sel.sum()


def check_v2_boundary():
    """Gains of 0.8, 1.05 and 1.2 times the predicted largest stable Kp,
    and the oscillation frequency."""
    kmax = lin.max_stable_kp(PAR, K_LIQ)
    out = []
    for f in (0.8, 1.05, 1.2):
        lp = lin.loop(PAR, K_LIQ, G_cfgPressureKp=f * kmax)
        pole = osc_pole(lp)
        r, _ = run(f * kmax, 9.0, target1=10.01e6)
        rate, w, npk = peaks_and_freq(r, 1.3, 1e3 if f > 1 else 300, 1e5)
        # Growth rate and angular frequency of the pole, from z = exp(s Ts)
        pr = np.log(abs(pole)) / PAR.Ts                     # [1/s]
        pw = abs(np.angle(pole)) / PAR.Ts                   # [rad/s]
        late = np.abs(r.P - r.target)[-400:].max()          # [Pa]
        if f < 1:
            # Decays to below 0.1 kPa
            assert pr < 0 and rate < 0 and late < 100, (f, rate, late)
            assert abs(rate / pr - 1) < 0.15, (rate, pr)
        else:
            assert pr > 0 and rate > 0, (f, rate)
        if f == 1.2:
            # Limit cycle tens of times the size of the step
            assert late > 0.1e6, late
            assert abs(rate / pr - 1) < 0.15, (rate, pr)
        assert abs(w / pw - 1) < 0.01, (w, pw)
        out.append(f'{f:g}x: rate {rate:+.2f}/s (linear {pr:+.2f}), '
                   f'{w:.2f} rad/s (linear {pw:.2f})')
    lp_max = lin.loop(PAR, K_LIQ, G_cfgPressureKp=kmax)
    wpc = lin.margins(lp_max['L'])['wpc']
    ok(f'V2 boundary: Kp max {kmax:.3f} mm/s/MPa (liquid, '
       f'{K_LIQ / 1e9:.1f} MPa/mm), phase crossover {wpc:.2f} rad/s; sim '
       + '; '.join(out))


# %% ===== V3: OUTFLOW DISTURBANCE =======================================
def check_v3_disturbance():
    """Outflow step: dip and recovery."""
    Q = 2e-7                                                # [m^3/s]
    kp = CFG['G_cfgPressureKp']
    # The hose is open so that the outlet flows, and without its volume
    # and compliance so that the linear chamber is the saline alone
    q = PAR.with_(V_hose=0.0, C_hose=0.0)
    s = scenarios.stiffstep(q, T=8.0, target1=10e6, Qf=Q, t_open=1.0,
                            Topen=0.0, tube=False, hose_open0=True)
    r = sim.run(q, s, ctl.SlaveController())
    n0 = round(s.t_open / PAR.Ts)
    lp = lin.loop(PAR, K_LIQ)
    y = model_trace(lp['D'], len(r.t), n0, Q)               # [Pa]
    dev = r.P - 10e6                                        # [Pa]
    dip_sim, dip_lin = -dev.min(), -y.min()
    assert abs(dip_sim / dip_lin - 1) < 0.03, (dip_sim, dip_lin)
    err = np.abs(dev - y).max() / dip_lin
    assert err < 0.05, err
    t_sim, t_lin = r.t[dev.argmin()], r.t[y.argmin()]
    assert abs(t_sim - t_lin) < 0.02
    rule = lin.static_dip_rule(PAR, Q, kp)                  # [MPa]
    ok(f'V3 outflow {Q * 1e6:.1f} mL/s: dip {dip_sim / 1e3:.1f} kPa sim, '
       f'{dip_lin / 1e3:.1f} kPa linear at {t_lin:.2f} s (sim '
       f'{t_sim:.2f} s), trace difference {err * 100:.1f}% of the dip; '
       f'Q/(A Kp) = {rule * 1e3:.1f} kPa')


# %% ===== V4: DERIVATIVE TERM AND NOISE =================================
def check_v4_derivative():
    """Target step and command noise with a derivative term on the
    measured pressure, eqs. (57) and (58), linear against nonlinear."""
    amp = 0.05e6                            # [Pa] step size
    Kd = 0.05                               # [(mm/s)/(MPa/s)]
    s = scenarios.stiffstep(PAR, T=6.0, target1=10e6 + amp)
    r = sim.run(PAR, s, ctl.VariantController(Kd=Kd))
    n0 = round(s.t_step / PAR.Ts)           # [-] scan of the step
    n = len(r.t)
    y = model_trace(lin.loop(PAR, K_LIQ, Kd=Kd)['R'], n, n0, amp)
    err = np.abs(r.P - 10e6 - y).max() / amp
    y0 = model_trace(lin.loop(PAR, K_LIQ)['R'], n, n0, amp)
    err0 = np.abs(r.P - 10e6 - y0).max() / amp
    assert err < 0.01 and err0 > 5 * err, (err, err0)

    # The derivative term adds phase lead: the phase margin rises.
    pm0 = lin.margins(lin.loop(PAR, K_LIQ)['L'])['pm']
    pm1 = lin.margins(lin.loop(PAR, K_LIQ, Kd=Kd)['L'])['pm']
    assert pm1 > pm0 + 5, (pm0, pm1)

    # Command noise at rest against the 2-norm of N. The median of five
    # lowers the variance of white noise and the model passes it as a
    # delay, so the model is the upper bound.
    q = PAR.with_(noise=1.7e3)
    s = scenarios.stiffstep(q, T=30.0, target1=10e6)
    gains = []
    for kd in (0.0, Kd):
        r = sim.run(q, s, ctl.VariantController(Kd=kd))
        late = r.t > 5.0
        got = float(np.std(r.u[late]))                      # [m/s]
        want = lin.noise_gain(lin.loop(PAR, K_LIQ, Kd=kd)) * q.noise
        assert 0.8 * want < got < 1.05 * want, (kd, got, want)
        gains.append(got * 1e6)                     # [m/s -> um/s]
    ok(f'V4 derivative term Kd {Kd}: step response linear vs sim max '
       f'difference {err * 100:.2f}% of the step ({err0 * 100:.0f}% '
       f'without it in the model); phase margin {pm0:.0f} to '
       f'{pm1:.0f} deg; command noise {gains[0]:.2f} to {gains[1]:.2f} '
       f'um/s rms, within the noise gain')


# %% ===== V5: SOFT CHAMBER ==============================================
def check_v5_soft():
    """Two-integrator reduction of the soft chamber, eqs. (59) to
    (61)."""
    Kp, Ki = CFG['G_cfgPressureKp'], CFG['G_cfgPressureKi']
    worst = 0.0
    for k in (0.003, 0.03, 0.1):                            # [MPa/mm]
        for Kd in (0.0, 0.05):
            z, wn = lin.least_damped(lin.loop(PAR, k * lin.MPA_MM,
                                              Kd=Kd))
            # Eq. (60)
            z_want = 0.5 * Kp * np.sqrt(k / (Ki * (1 + k * Kd)))
            wn_want = np.sqrt(k * Ki / (1 + k * Kd))        # [rad/s]
            worst = max(worst, abs(z / z_want - 1),
                        abs(wn / wn_want - 1))
    assert worst < 0.06, worst

    # Eq. (61) against the reduction integrated from e = 0 with the
    # integral holding w0: e' = -k (Kp e + w), w' = Ki e.
    k, w0, dt = 0.05, 2.0, 1e-3             # [MPa/mm], [mm/s], [s]
    e, w, peak = 0.0, w0, 0.0
    for _ in range(int(60 / dt)):
        e, w = e - k * (Kp * e + w) * dt, w + Ki * e * dt
        peak = max(peak, -e)
    z = 0.5 * Kp * np.sqrt(k / Ki)
    phi = np.arctan2(np.sqrt(1 - z * z), z)
    want = w0 * np.sqrt(k / Ki) * np.exp(-z * phi / np.sqrt(1 - z * z))
    assert abs(peak / want - 1) < 0.01, (peak, want)

    # A fixed gain set keeps its phase margin over a limited range of
    # stiffness: with Ki = 0.4 Kp at the stiff limit, about 36 to 1.
    kd = 7.5 * lin.MPA_MM                                   # [Pa/m]
    kp = lin.kp_for_margins(PAR, kd, 45.0, 6.0, 0.4)
    lo = lin.softest_stiffness(PAR, 45.0, kd, G_cfgPressureKp=kp,
                               G_cfgPressureKi=0.4 * kp)
    assert abs(kp - 1.49) < 0.02 and 30 < kd / lo < 42, (kp, kd / lo)
    ok(f'V5 soft chamber: least damped mode within {worst * 100:.0f}% '
       f'of eq. (60) from 0.003 to 0.1 MPa/mm; stored-velocity '
       f'overshoot {peak:.3f} MPa against eq. (61) {want:.3f}; Kp '
       f'{kp:.2f} holds 45 deg over a stiffness range of '
       f'{kd / lo:.0f} to 1')


if __name__ == '__main__':
    t0 = time.time()
    check_plant_discretisation()
    check_margins()
    check_v1_step()
    check_v2_boundary()
    check_v3_disturbance()
    check_v4_derivative()
    check_v5_soft()
    print(f'{time.time() - t0:.1f} s')
