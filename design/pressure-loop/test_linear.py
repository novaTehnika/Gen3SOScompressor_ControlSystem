#!/usr/bin/env python3
"""Checks of the linear pressure loop model against the nonlinear simulation.

    python3 design/pressure-loop/test_linear.py

Prints `ok  ...` per check and stops at the first failure. The simulation runs with sensor noise
off in the liquid-only chamber at small amplitudes, where the clamp and the slew limit do not bind.
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
from params import Params  # noqa: E402

P = Params().with_(noise=0.0)
K_LIQ = P.A * P.K / P.Vl0                      # Pa/m, liquid-only chamber
CFG = P.cfg()


def ok(msg):
    print(f'ok  {msg}')


def run(kp, T, **sc):
    s = scenarios.stiffstep(P, T=T, **sc)
    return sim.run(P, s, ctl.SlaveController(Kp=kp)), s


def check_plant_discretisation():
    k = 3e9
    lp = lin.loop(P, k)
    num, den, _ = signal.cont2discrete(([k], [P.tauV, 1.0, 0.0]), P.Ts, 'zoh')[:3]
    assert np.allclose(np.trim_zeros(np.ravel(num), 'f'), lp['Gd'].num, rtol=1e-9)
    assert np.allclose(den, lp['Gd'].den, rtol=1e-9)
    assert lp['delay'] == 5 and lin.sample_delay(CFG) == 4
    ok('plant ZOH matches scipy cont2discrete; sample delay = median 2 + pipeline 2 (+1 command scan)')


def check_v1_step():
    """V1: closed-loop target step, linear against nonlinear, and the delay count"""
    amp = 0.05e6
    kp = CFG['G_cfgPressureKp']
    r, s = run(kp, 6.0, target1=10e6 + amp)
    n0 = round(s.t_step / P.Ts)
    # clamp and slew do not bind: |Kp e| below StopVelMin (the lowest the clamp goes), step jump below Accel Ts
    umax = np.abs(r.u).max() * 1e3
    assert umax < CFG['G_cfgPressureStopVelMin'] and lin.slew_step_error(CFG, kp) > amp / 1e6, umax
    n = len(r.t)
    errs = {}
    for d in range(2, 8):                                         # total delay in the sensor path
        R = lin.loop(P, K_LIQ, delay=d)['R']
        y = np.concatenate([np.zeros(n0), R.step(n - n0 + 1, amp)])[1:n + 1]    # trace[j] is the state after scan j
        errs[d] = np.abs(r.P - 10e6 - y).max() / amp
    best = min(errs, key=errs.get)
    assert best == lin.sample_delay(CFG) and errs[best] < 0.005, errs
    assert min(v for d, v in errs.items() if d != best) > 3 * errs[best], errs
    ok(f'V1 step response: linear vs sim max difference {errs[best] * 100:.2f}% of the step (delay {best} + 1 scans fits best; '
       f'{best - 1} gives {errs[best - 1] * 100:.1f}%, {best + 1} gives {errs[best + 1] * 100:.1f}%); max |u| {umax:.3f} mm/s, clamp not active')


def osc_pole(lp):
    return max((z for z in lin.poles(lp) if abs(z.imag) > 1e-3), key=abs)


def peaks_and_freq(r, t0, lo, hi):
    """growth rate (1/s) and angular frequency (rad/s) of the oscillation of the error between
    amplitudes lo and hi (Pa), from its |e| peaks and zero crossings"""
    e = r.P - r.target
    a = np.abs(e)
    pk = np.array([i for i in range(1, len(a) - 1) if a[i] >= a[i - 1] and a[i] > a[i + 1] and r.t[i] > t0])
    sel = (a[pk] > lo) & (a[pk] < hi)
    t = r.t[pk][sel]
    rate = np.polyfit(t, np.log(a[pk][sel]), 1)[0]
    zi = np.nonzero(e[:-1] * e[1:] < 0)[0]
    zt = r.t[zi]
    zt = zt[(zt > t[0]) & (zt < t[-1])]
    return rate, np.pi / np.mean(np.diff(zt)), sel.sum()


def check_v2_boundary():
    """V2: gains 0.8x and 1.2x the predicted maximum stable Kp, and the oscillation frequency"""
    kmax = lin.max_stable_kp(P, K_LIQ)
    out = []
    for f in (0.8, 1.05, 1.2):
        lp = lin.loop(P, K_LIQ, G_cfgPressureKp=f * kmax)
        pole = osc_pole(lp)
        r, _ = run(f * kmax, 9.0, target1=10.01e6)
        rate, w, npk = peaks_and_freq(r, 1.3, 1e3 if f > 1 else 300, 4e4)
        pr, pw = np.log(abs(pole)) / P.Ts, abs(np.angle(pole)) / P.Ts
        late = np.abs(r.P - r.target)[-400:].max()
        if f < 1:
            assert pr < 0 and rate < 0 and late < 100, (f, rate, late)         # decays to < 0.1 kPa
            assert abs(rate / pr - 1) < 0.15, (rate, pr)
        else:
            assert pr > 0 and rate > 0, (f, rate)
        if f == 1.2:
            assert late > 0.1e6, late                                          # limit cycle, tens of times the step
            assert abs(rate / pr - 1) < 0.15, (rate, pr)
        assert abs(w / pw - 1) < 0.01, (w, pw)
        out.append(f'{f:g}x: rate {rate:+.2f}/s (linear {pr:+.2f}), {w:.2f} rad/s (linear {pw:.2f})')
    wpc = lin.margins(lin.loop(P, K_LIQ, G_cfgPressureKp=kmax)['L'])['wpc']
    ok(f'V2 boundary: Kp max {kmax:.3f} mm/s/MPa (K = 1 GPa, {K_LIQ / 1e9:.1f} MPa/mm), phase crossover {wpc:.2f} rad/s; sim ' + '; '.join(out))


def check_v3_disturbance():
    """V3: outflow step, dip and recovery"""
    Q = 2e-7
    kp = CFG['G_cfgPressureKp']
    s = scenarios.stiffstep(P, T=8.0, target1=10e6, Qf=Q, t_open=1.0, Topen=0.0)
    r = sim.run(P, s, ctl.SlaveController())
    n0 = round(s.t_open / P.Ts)
    n = len(r.t)
    lp = lin.loop(P, K_LIQ)
    y = np.concatenate([np.zeros(n0), lp['D'].step(n - n0 + 1, Q)])[1:n + 1]
    dev = r.P - 10e6
    dip_sim, dip_lin = -dev.min(), -y.min()
    assert abs(dip_sim / dip_lin - 1) < 0.03, (dip_sim, dip_lin)
    err = np.abs(dev - y).max() / dip_lin
    assert err < 0.05, err
    t_sim, t_lin = r.t[dev.argmin()], r.t[y.argmin()]
    assert abs(t_sim - t_lin) < 0.02
    rule = lin.static_dip_rule(P, Q, kp)
    ok(f'V3 outflow {Q * 1e6:.1f} mL/s: dip {dip_sim / 1e3:.1f} kPa sim, {dip_lin / 1e3:.1f} kPa linear at {t_lin:.2f} s '
       f'(sim {t_sim:.2f} s), trace difference {err * 100:.1f}% of the dip; Q/(A Kp) = {rule * 1e3:.1f} kPa')


def check_margins():
    lp = lin.loop(P, K_LIQ)
    m = lin.margins(lp['L'])
    assert lin.is_stable(lp) and m['gm'] > 1 and m['pm'] > 0
    # gain margin: scaling the loop gain by gm puts a closed-loop pole on the unit circle
    kp = CFG['G_cfgPressureKp']
    edge = lin.loop(P, K_LIQ * m['gm'])
    assert abs(np.abs(lin.poles(edge)).max() - 1) < 1e-6
    # delay margin: adding dm of delay takes the phase margin to zero (at the same crossover)
    assert abs(m['dm'] * m['wc'] - np.radians(m['pm'])) < 1e-12
    # Kp and k enter only as Kp k when Ki scales with Kp
    a = lin.margins(lin.loop(P, K_LIQ, G_cfgPressureKp=kp, G_cfgPressureKi=0.4 * kp)['L'])
    b = lin.margins(lin.loop(P, 2 * K_LIQ, G_cfgPressureKp=kp / 2, G_cfgPressureKi=0.2 * kp)['L'])
    assert abs(a['pm'] - b['pm']) < 1e-6 and abs(a['gm'] - b['gm']) < 1e-6
    ok(f'margins at the GVL gains, K = 1 GPa: GM {m["gm_db"]:.1f} dB, PM {m["pm"]:.1f} deg at {m["wc"]:.2f} rad/s, '
       f'DM {m["dm"] * 1e3:.0f} ms; gain scaling reaches the unit circle; L depends on Kp k')


if __name__ == '__main__':
    t0 = time.time()
    check_plant_discretisation()
    check_margins()
    check_v1_step()
    check_v2_boundary()
    check_v3_disturbance()
    print(f'{time.time() - t0:.1f} s')
