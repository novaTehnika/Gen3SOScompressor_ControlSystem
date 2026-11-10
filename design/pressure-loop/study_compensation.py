#!/usr/bin/env python3
"""
study_compensation.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
11/10/2026

PURPOSE/DESCRIPTION:
This script examines the choice of compensator for the slave's pressure
loop on the linear model of linear.py (README, "Choice of compensation",
eqs. (57) to (63)), and confirms what it finds in the nonlinear
simulation.

The loop is an integrator (the chamber, velocity to pressure, with gain
the stiffness k) behind about 70 ms of delay and lag. With the ratios
Ki/Kp and Kd/Kp fixed, the open loop is one curve scaled by Kp k, so a
change of stiffness slides the crossover along a fixed phase curve. The
script reports, in order:

  1. The GVL loop across the chamber's range of stiffness: margins,
     crossover, the damping of its slowest mode and the overshoot of a
     target step.
  2. The integral: for each Ki/Kp, the largest Kp that keeps the
     margins in the stiff design chamber, the outflow dip and its
     recovery, and the softest chamber that keeps the phase margin.
  3. The derivative: for each filter and each Kd/Kp, the largest Kp
     that keeps the margins, what it buys in dip and costs in margin
     at the stiffest chamber and in command noise.
  4. A grid of Kp and Kd in the nonlinear simulation: the stiff step,
     the 10 MPa cycle and the soft step.
  5. The candidate gain sets the linear model selects, in the same
     nonlinear cases, with and without the integral cleared past the
     target (controllers.VariantController, eq. (52)).
  6. The delay: where the loop's pure delay comes from, and the gain
     the margins allow with each part of it changed.

It prints the tables, writes them to compensation_summary.txt and
compensation.csv, and plots the root locus against stiffness, the phase
margin against stiffness, the open loops and the grid to
compensation_rlocus.png, compensation_margin.png, compensation_bode.png
and compensation_grid.png in the output directory.

Usage:
    python3 study_compensation.py [--set key=value ...] [--out DIR]
                                  [--workers N] [--seed N]

FILE DEPENDENCY:
./
    controllers.py
    linear.py
    runner.py
    scenarios.py
    study_antiwindup.py (the soft cases and their metrics)
    study_floor.py (delivery_stats)
    studyutil.py
    (and the modules they import: sim.py, metrics.py, and models/)
"""

import csv
import math

import numpy as np

import controllers as ctl
import linear as lin
import scenarios
import study_antiwindup as aw
import studyutil
from runner import Case, run_cases

plt = studyutil.plt


# %% ===== DESIGN CONDITIONS =============================================
PM_MIN = 45.0               # [deg] phase margin required
GM_MIN = 6.0                # [dB] gain margin required
# [MPa/mm] Soft stiffnesses reported: free gas at a low pressure
K_SOFT = (0.003, 0.01, 0.03, 0.1, 0.3, 1.0)
# [-] Ki/Kp ratios [1/s] and Kd/Kp ratios [s] examined
RATIOS = (0.1, 0.2, 0.4, 0.8, 1.6)
TDS = (0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.05)
# [s] Derivative filter time constants examined
TFS = (0.004, 0.01, 0.02, 0.05)
# Nonlinear grid: Kp [(mm/s)/MPa] and Kd [(mm/s)/(MPa/s)], Ki = 0.4 Kp
GRID_KP = (1.0, 1.5, 2.0, 2.5, 3.0)
GRID_KD = (0.0, 0.02, 0.04, 0.06, 0.08)
# [-] Damping ratio asked of the soft chamber for the "soft PI" set
ZETA_SOFT = 0.7


def chamber_stiffness(par):
    """Return the liquid chamber's stiffness [MPa/mm] in its three
    states, eqs. (4), (10), (42) and (49): with the hose open at a full
    fill (the delivery), with the hose shut at a full fill (the design
    chamber), and with no liquid left (the stiffest possible)."""
    C_liq = par.Vl0 / par.K + par.C_series                  # [m^3/Pa]
    C_open = C_liq + par.V_hose / par.K + par.C_hose        # [m^3/Pa]
    # [Pa/m -> MPa/mm]
    return (par.A / C_open / lin.MPA_MM, par.A / C_liq / lin.MPA_MM,
            par.A / par.C_series / lin.MPA_MM)


# %% ===== REPORT LOG ====================================================
LOG = []


def say(text=''):
    """Print a line and keep it for the summary file."""
    print(text, flush=True)
    LOG.append(text)


# %% ===== LINEAR MODEL ==================================================
def loop(par, k, Kp, Ki, Kd=0.0, Tf=0.02):
    """Return linear.loop() at stiffness k [MPa/mm] and the gains."""
    return lin.loop(par, k * lin.MPA_MM, Kd=Kd, Tf=Tf,
                    G_cfgPressureKp=Kp, G_cfgPressureKi=Ki)


def zeta_formula(k, Kp, Ki, Kd=0.0):
    """[-] Damping ratio of the soft chamber's mode, eq. (60)."""
    return 0.5 * Kp * math.sqrt(k / (Ki * (1.0 + k * Kd)))


def stored_overshoot(k, Kp, Ki, w0):
    """[MPa] Overshoot from an integral holding the velocity w0 [mm/s]
    as the error crosses zero in a chamber of stiffness k [MPa/mm],
    eq. (61)."""
    z = zeta_formula(k, Kp, Ki)
    if z >= 1.0:
        return math.nan
    phi = math.atan2(math.sqrt(1.0 - z * z), z)             # [rad]
    return w0 * math.sqrt(k / Ki) * math.exp(
        -z * phi / math.sqrt(1.0 - z * z))


def outflow(par, k, Kp, Ki, Kd, Tf, Q, T=120.0):
    """Return (dip [MPa], recovery [s]) of the linear loop for an
    outflow step Q [m^3/s] at stiffness k [MPa/mm]: the deepest fall,
    and the time after which the pressure stays within par.tol of the
    target (NaN if it has not within T)."""
    n = int(T / par.Ts)
    dip, _, y = lin.step_dip(loop(par, k, Kp, Ki, Kd, Tf), Q, n)
    out = np.flatnonzero(np.abs(y) > par.tol)
    if out.size == 0:
        return dip / 1e6, 0.0
    if out[-1] >= n - 1:
        return dip / 1e6, math.nan
    return dip / 1e6, (out[-1] + 1) * par.Ts


def summarize(par, Kp, Ki, Kd, Tf, ks):
    """Return the linear figures of one gain set as a dict. ks is the
    (delivery, design, stiffest) stiffness [MPa/mm]."""
    k_del, k_des, k_max = ks
    w = lin.freq_grid(par.Ts, 2500, 1e-3)
    out = {'Kp': Kp, 'Ki': Ki, 'Kd': Kd, 'Tf': Tf}
    for tag, k in (('des', k_des), ('max', k_max)):
        lp = loop(par, k, Kp, Ki, Kd, Tf)
        if lin.is_stable(lp):
            m = lin.margins(lp['L'], w)
            out['pm_' + tag], out['gm_' + tag] = m['pm'], m['gm_db']
            out['wc_' + tag] = m['wc']
        else:
            out['pm_' + tag] = out['gm_' + tag] = math.nan
            out['wc_' + tag] = math.nan
    out['dip'], out['rec'] = outflow(par, k_del, Kp, Ki, Kd, Tf,
                                     par.Q_orifice(10e6))
    # [(m/s)/Pa -> (um/s)/kPa]: 1e6 um/m times 1e3 Pa/kPa
    out['noise'] = lin.noise_gain(loop(par, k_del, Kp, Ki, Kd, Tf)) * 1e9
    out['zeta'] = lin.least_damped(loop(par, 0.03, Kp, Ki, Kd, Tf))[0]
    # The softest chamber with the phase margin, searched down from
    # the design chamber
    out['k_lo'] = lin.softest_stiffness(
        par, PM_MIN, k_des * lin.MPA_MM, Kd=Kd, Tf=Tf,
        G_cfgPressureKp=Kp, G_cfgPressureKi=Ki) / lin.MPA_MM
    return out


def kp_limit(par, k, r, td=0.0, Tf=0.02):
    """[(mm/s)/MPa] Largest Kp with the margins at k [MPa/mm]."""
    return lin.kp_for_margins(par, k * lin.MPA_MM, PM_MIN, GM_MIN, r,
                              td=td, Tf=Tf)


def linear_tables(par, ks):
    """Print the linear tables (parts 1 to 3) and return (CSV rows,
    the candidate gain sets as {name: (Kp, Ki, Kd, Tf)})."""
    k_del, k_des, k_max = ks
    cfg = par.cfg()
    Kp0, Ki0 = cfg['G_cfgPressureKp'], cfg['G_cfgPressureKi']
    w = lin.freq_grid(par.Ts, 2500, 1e-5)
    rows = []

    say(f'chamber stiffness: delivery {k_del:.1f}, design {k_des:.1f}, '
        f'stiffest {k_max:.1f} MPa/mm; margins asked: {PM_MIN:.0f} deg '
        f'and {GM_MIN:.0f} dB at the design chamber')

    say(f'\n=== 1. GVL loop (Kp {Kp0:g}, Ki {Ki0:g}) across stiffness')
    say(f'{"k MPa/mm":>9s} {"PM deg":>7s} {"GM dB":>6s} {"wc rad/s":>9s}'
        f' {"zeta":>5s} {"eq.(60)":>8s} {"step over %":>12s} '
        f'{"peak s":>7s}')
    for k in K_SOFT + ks:
        lp = loop(par, k, Kp0, Ki0)
        m = lin.margins(lp['L'], w)
        z, wn = lin.least_damped(lp)
        n = int(min(max(40.0 / wn, 5.0), 4000.0) / par.Ts)
        y = lp['R'].step(n)
        zf = zeta_formula(k, Kp0, Ki0)
        say(f'{k:9.3f} {m["pm"]:7.1f} {m["gm_db"]:6.1f} {m["wc"]:9.3f} '
            f'{z:5.2f} {zf:8.2f} {100 * (y.max() - 1):12.0f} '
            f'{np.argmax(y) * par.Ts:7.1f}')
        rows.append(['1 stiffness', k, m['pm'], m['gm_db'], m['wc'], z,
                     100 * (y.max() - 1)])

    say('\n=== 2. Integral: largest Kp with the margins at the design '
        'chamber, per Ki/Kp')
    head = (f'{"Ki/Kp":>6s} {"Kp":>5s} {"Ki":>5s} {"dip MPa":>8s} '
            f'{"recover s":>10s} {"k_lo MPa/mm":>12s} {"range":>6s} '
            f'{"zeta@0.03":>10s}')
    say(head)
    for r in (1e-4,) + RATIOS:
        Kp = kp_limit(par, k_des, r)
        s = summarize(par, Kp, r * Kp, 0.0, 0.02, ks)
        # A loop with no integral keeps its margin at any stiffness.
        lo, rng = ((f'{s["k_lo"]:.3f}', f'{k_des / s["k_lo"]:.0f}')
                   if s['k_lo'] > 1e-3 else ('any', '—'))
        say(f'{r:6.2g} {Kp:5.2f} {r * Kp:5.2f} {s["dip"]:8.2f} '
            f'{aw.fmt(s["rec"], ".1f"):>10s} {lo:>12s} {rng:>6s} '
            f'{s["zeta"]:10.2f}')
        rows.append(['2 integral', r, Kp, r * Kp, s['dip'], s['rec'],
                     s['k_lo'], s['zeta']])
    # The Ki that damps the soft chamber, eq. (60) solved for Ki
    Ki_soft = Kp0**2 * 0.03 / (4.0 * ZETA_SOFT**2)
    s = summarize(par, Kp0, Ki_soft, 0.0, 0.02, ks)
    say(f'for zeta {ZETA_SOFT:g} at 0.03 MPa/mm with Kp {Kp0:g}: Ki '
        f'{Ki_soft:.3f} (integral time {Kp0 / Ki_soft:.0f} s), outflow '
        f'recovery {aw.fmt(s["rec"], ".0f")} s')

    say('\n=== 3. Derivative: largest Kp with the margins at the design'
        ' chamber, Ki = 0.4 Kp')
    say(f'{"Tf s":>6s} {"Kd/Kp s":>8s} {"Kp":>5s} {"Kd":>6s} '
        f'{"vs PI %":>8s} {"PM/GM stiffest":>15s} {"dip MPa":>8s} '
        f'{"noise um/s/kPa":>15s} {"zeta@0.03":>10s}')
    best = {}
    Kp_pi = kp_limit(par, k_des, 0.4)
    for Tf in TFS:
        for td in TDS:
            Kp = kp_limit(par, k_des, 0.4, td, Tf)
            s = summarize(par, Kp, 0.4 * Kp, td * Kp, Tf, ks)
            say(f'{Tf:6.3f} {td:8.3f} {Kp:5.2f} {td * Kp:6.3f} '
                f'{100 * (Kp / Kp_pi - 1):8.0f} '
                f'{s["pm_max"]:8.0f} /{s["gm_max"]:5.1f} '
                f'{s["dip"]:8.2f} {s["noise"]:15.2f} {s["zeta"]:10.2f}')
            rows.append(['3 derivative', Tf, td, Kp, td * Kp,
                         s['pm_max'], s['gm_max'], s['dip'], s['noise'],
                         s['zeta']])
            if Tf not in best or Kp > best[Tf][0]:
                best[Tf] = (Kp, 0.4 * Kp, td * Kp, Tf)
    # The same search with the PI loop's own gain margin kept, so the
    # stiffest chamber is no nearer to instability than it is now
    w3 = lin.freq_grid(par.Ts, 2500, 1e-3)
    gm_pi = lin.margins(loop(par, k_des, Kp_pi, 0.4 * Kp_pi)['L'],
                        w3)['gm_db']
    say(f'with the PI loop\'s gain margin kept ({gm_pi:.1f} dB at the '
        f'design chamber):')
    for Tf in TFS:
        kps = [lin.kp_for_margins(par, k_des * lin.MPA_MM, PM_MIN,
                                  gm_pi - 0.01, 0.4, td=td, Tf=Tf)
               for td in TDS]
        j = int(np.nanargmax(kps))
        say(f'  Tf {Tf:5.3f} s: Kp {kps[j]:.2f} at Kd/Kp {TDS[j]:g} s '
            f'({100 * (kps[j] / Kp_pi - 1):+.0f} %)')
        rows.append(['3 derivative, PI gain margin', Tf, TDS[j], kps[j]])

    cand = {
        'P': (kp_limit(par, k_des, 1e-4), 0.0, 0.0, 0.02),
        'PI GVL': (Kp0, Ki0, 0.0, 0.02),
        'PI Ki/Kp 0.1': (kp_limit(par, k_des, 0.1),) * 1 + (
            0.1 * kp_limit(par, k_des, 0.1), 0.0, 0.02),
        'PI Ki/Kp 1.6': (kp_limit(par, k_des, 1.6),) * 1 + (
            1.6 * kp_limit(par, k_des, 1.6), 0.0, 0.02),
        'PI soft': (Kp0, Ki_soft, 0.0, 0.02),
        'PID, Tf 20 ms': best[0.02],
        'PID, Tf 4 ms': best[0.004],
    }
    return rows, cand


# %% ===== NONLINEAR CASES ===============================================
def nonlinear(par, sets, seed, legacy, workers, clear=()):
    """Run the gain sets {name: (Kp, Ki, Kd, Tf)} through the soft
    steps, the cycle and the stiff step, and return {name: metrics}.
    Names in clear also run with the integral cleared past the target,
    eq. (52), as 'name + clear'.

      soft5, soft20  [MPa] Overshoot of the 5 and 20 atm steps.
      band5          [s] Time to stay in band after the 5 atm step.
      over           [MPa] Compression overshoot of the cycle.
      dip            [MPa] Deepest dip of the cycle.
      rec            [s] From the reopening to the last scan outside
                     the band; NaN if the run ends outside it.
      close          [MPa] Purge-closing peak.
      V_ok           [mL] Delivered before the first violation.
      stiff          [MPa] Overshoot of the stiff step.
      act            [mm/s] Command activity at rest (stiff step).
    """
    todo = {c[0]: c for c in aw.cases(par)}
    use = ('soft steps', 'cycle, hose before', 'stiff step')
    arms = []
    for name, (Kp, Ki, Kd, Tf) in sets.items():
        arms.append((name, dict(Kp=Kp, Ki=Ki, Kd=Kd, Tf=Tf)))
        if name in clear:
            arms.append((name + ' + clear', dict(
                Kp=Kp, Ki=Ki, Kd=Kd, Tf=Tf, aw='opposed', over=0.02)))
    jobs, index = [], []
    for name, kw in arms:
        for case in use:
            _, q, sc = todo[case]
            jobs.append(Case(q, sc, ctl.VariantController(name, **kw),
                             legacy_sensor=legacy, seed=seed))
            index.append((name, case))
    res = {}
    for (name, case), r in zip(index, run_cases(jobs, workers=workers)):
        res.setdefault(name, {})[case] = r
    out = {}
    for name, kw in arms:
        m = {}
        r = res[name]['soft steps']
        q = todo['soft steps'][1]
        s5 = aw.step_stats(r, aw.T_SOFT[0], aw.T_SOFT[1], q.tol)
        s20 = aw.step_stats(r, aw.T_SOFT[1], aw.T_SOFT[2], q.tol)
        m.update(soft5=s5['over'], band5=s5['tBand'], soft20=s20['over'])
        r = res[name]['cycle, hose before']
        _, q, sc = todo['cycle, hose before']
        c = ctl.VariantController(name, **kw)
        c.reset(q, sc)
        cs = aw.cycle_stats(r, c, q, sc)
        m.update(over=cs['over'], dip=cs['dip'], close=cs['closePeak'],
                 V_ok=cs['V_ok'])
        m['rec'] = math.nan
        if len(r.events) >= 3:
            w = np.flatnonzero(r.t >= r.events[-1])
            outside = np.flatnonzero(
                np.abs(r.target[w] - r.P[w]) > q.tol)
            if outside.size == 0:
                m['rec'] = 0.0
            elif outside[-1] < len(w) - 1:
                m['rec'] = float(r.t[w][outside[-1] + 1] - r.events[-1])
        r = res[name]['stiff step']
        m.update(stiff=r['overshoot'], act=r['uAct'])
        out[name] = m
    return out


def grid_tables(par, ks, seed, legacy, workers):
    """Run the Kp and Kd grid, print its tables and return (CSV rows,
    {(Kp, Kd): metrics})."""
    _, k_des, k_max = ks
    w = lin.freq_grid(par.Ts, 2500, 1e-3)
    sets = {f'{kp:g}/{kd:g}': (kp, 0.4 * kp, kd, 0.02)
            for kp in GRID_KP for kd in GRID_KD}
    m = nonlinear(par, sets, seed, legacy, workers)
    grid = {(kp, kd): m[f'{kp:g}/{kd:g}']
            for kp in GRID_KP for kd in GRID_KD}
    for (kp, kd), g in grid.items():
        lp = loop(par, k_des, kp, 0.4 * kp, kd)
        g['pm'] = (lin.margins(lp['L'], w)['pm']
                   if lin.is_stable(lp) else math.nan)
        lp = loop(par, k_max, kp, 0.4 * kp, kd)
        g['gm'] = (lin.margins(lp['L'], w)['gm_db']
                   if lin.is_stable(lp) else math.nan)

    say('\n=== 4. Kp and Kd grid, Ki = 0.4 Kp, derivative filter 20 ms')
    items = (('pm', 'linear phase margin at the design chamber, deg',
              '.0f'),
             ('gm', 'linear gain margin at the stiffest chamber, dB '
              '(— unstable)', '.1f'),
             ('stiff', 'stiff step 10 to 12 MPa: overshoot, MPa', '.2f'),
             ('over', 'compression at 10 MPa: overshoot, MPa', '.2f'),
             ('dip', 'cycle: deepest dip, MPa', '.2f'),
             ('close', 'cycle: purge-closing peak, MPa', '.2f'),
             ('soft5', 'soft chamber, 5 atm step: overshoot, MPa', '.2f'),
             ('act', 'command activity at rest, mm/s per scan', '.4f'))
    rows = []
    for key, title, spec in items:
        say(f'\n{title}')
        say(f'{"Kp \\ Kd":>8s} '
            + ' '.join(f'{kd:8g}' for kd in GRID_KD))
        for kp in GRID_KP:
            vals = [grid[(kp, kd)][key] for kd in GRID_KD]
            say(f'{kp:8g} '
                + ' '.join(f'{aw.fmt(v, spec):>8s}' for v in vals))
            rows.append(['4 grid ' + key, kp] + vals)
    return rows, grid


def candidate_table(par, cand, ks, seed, legacy, workers):
    """Run the candidate gain sets, print their table and return the
    CSV rows."""
    m = nonlinear(par, cand, seed, legacy, workers,
                  clear=('PI GVL', 'PID, Tf 20 ms'))
    say('\n=== 5. Candidate gain sets: linear figures and the nonlinear'
        ' cases')
    say(f'{"set":24s} {"Kp":>5s} {"Ki":>6s} {"Kd":>6s} | {"PM":>4s} '
        f'{"GM":>5s} {"GMstf":>6s} {"zeta":>5s} | {"soft5":>6s} '
        f'{"band s":>7s} {"soft20":>7s} | {"over":>5s} {"dip":>5s} '
        f'{"rec s":>6s} {"close":>6s} {"V_ok":>5s} | {"stiff":>6s}')
    rows = []
    for name, v in m.items():
        Kp, Ki, Kd, Tf = cand[name.replace(' + clear', '')]
        s = summarize(par, Kp, max(Ki, 1e-4 * Kp), Kd, Tf, ks)
        say(f'{name:24s} {Kp:5.2f} {Ki:6.3f} {Kd:6.3f} | '
            f'{s["pm_des"]:4.0f} {s["gm_des"]:5.1f} {s["gm_max"]:6.1f} '
            f'{s["zeta"]:5.2f} | {v["soft5"]:6.2f} '
            f'{aw.fmt(v["band5"], ".0f"):>7s} {v["soft20"]:7.2f} | '
            f'{v["over"]:5.2f} {v["dip"]:5.2f} '
            f'{aw.fmt(v["rec"], ".1f"):>6s} {aw.fmt(v["close"]):>6s} '
            f'{v["V_ok"]:5.0f} | {v["stiff"]:6.2f}')
        rows.append(['5 candidates', name, Kp, Ki, Kd, Tf, s['pm_des'],
                     s['gm_des'], s['gm_max'], s['zeta'], v['soft5'],
                     v['band5'], v['soft20'], v['over'], v['dip'],
                     v['rec'], v['close'], v['V_ok'], v['stiff']])

    # Overshoot from the stored integral velocity, eq. (61), against
    # the simulation's 5 atm step
    cfg = par.cfg()
    Kp0, Ki0 = cfg['G_cfgPressureKp'], cfg['G_cfgPressureKi']
    w0 = cfg['G_cfgPressureStopVelMin']                     # [mm/s]
    say(f'\nstored-velocity overshoot, eq. (61), GVL gains, {w0:g} mm/s '
        f'held at the crossing:')
    for k in (0.03, 0.05, 0.1):
        say(f'  k {k:5.2f} MPa/mm: zeta {zeta_formula(k, Kp0, Ki0):.2f},'
            f' overshoot {stored_overshoot(k, Kp0, Ki0, w0):.2f} MPa')
    say(f'  simulation, 5 atm step: {m["PI GVL"]["soft5"]:.2f} MPa')
    return rows


# %% ===== DELAY BUDGET ==================================================
# Changes to the loop's delay: (label, StudyParams overrides, loop
# arguments). delay is the sensor path's scans (the median's) and
# cmd_delay the command path's (the drive's dead time in scans). The
# 8 ms dead time is what the bench's two settings of the Y_DirectControl
# command filter extrapolate to with the filter off (5 ms of response
# per 10 ms of filter); it has not been measured. The last row is the
# loop if PRG_Main read each block's output in the scan after the block
# ran (the filter, the scaling and the pressure loop: three scans).
DELAYS = [
    ('as built', {}, {}),
    ('median of 3', {}, {'G_cfgAnalogMedianSize': 3}),
    ('pressure filter 10 ms', {}, {'G_cfgAnalogFilterTimeConst': 0.010}),
    ('drive dead time 8 ms', {'t_dead': 0.0082}, {}),
    ('scan 2 ms', {'G_cfgScanTime': 0.002}, {}),
    ('median of 3 and filter 10 ms', {},
     {'G_cfgAnalogMedianSize': 3, 'G_cfgAnalogFilterTimeConst': 0.010}),
    ('those and the 8 ms dead time', {'t_dead': 0.0082},
     {'G_cfgAnalogMedianSize': 3, 'G_cfgAnalogFilterTimeConst': 0.010}),
    ('no pure delay', {}, {'delay': 0, 'cmd_delay': 0}),
    ('block outputs read a scan late', {}, {'delay': 4, 'cmd_delay': 6}),
]


def delay_table(par, ks):
    """Print the delay budget of the loop as built and the largest Kp
    with the margins at the design chamber for each change of DELAYS,
    Ki = 0.4 Kp, and return the CSV rows."""
    k_del, k_des, _ = ks
    cfg = par.cfg()
    Ts = cfg['G_cfgScanTime']                               # [s]
    n_med = lin.sample_delay(cfg)                           # [scans]
    say('\n=== 6. Delay: sources as built')
    for name, t in (
            (f'median of {cfg["G_cfgAnalogMedianSize"]}', n_med * Ts),
            ('drive dead time, in whole scans',
             round(par.t_dead / Ts) * Ts)):
        say(f'  pure delay  {name:44s} {t * 1e3:5.1f} ms')
    for name, t in (
            ('pressure low-pass filter',
             cfg['G_cfgAnalogFilterTimeConst']),
            ('drive velocity loop', par.tauV),
            ('sample and hold (half a scan)', Ts / 2)):
        say(f'  lag         {name:44s} {t * 1e3:5.1f} ms')

    say('\nlargest Kp with the margins at the design chamber, Ki = '
        '0.4 Kp:')
    say(f'{"change":42s} {"delay ms":>9s} {"Kp":>5s} {"vs built %":>11s}'
        f' {"Kp k rad/s":>11s} {"dip MPa":>8s} {"noise um/s/kPa":>15s} '
        f'{"zeta@0.03":>10s}')
    rows, Kp0 = [], None
    for name, pkw, lkw in DELAYS:
        q = par.with_(**pkw)
        Kp = lin.kp_for_margins(q, k_des * lin.MPA_MM, PM_MIN, GM_MIN,
                                0.4, **lkw)
        Kp0 = Kp if Kp0 is None else Kp0
        gains = {'G_cfgPressureKp': Kp, 'G_cfgPressureKi': 0.4 * Kp}
        lp = lin.loop(q, k_del * lin.MPA_MM, **{**lkw, **gains})
        dip = lin.step_dip(lp, q.Q_orifice(10e6),
                           int(60.0 / lp['Ts']))[0] / 1e6       # [MPa]
        # [(m/s)/Pa -> (um/s)/kPa]
        noise = lin.noise_gain(lp) * 1e9
        zeta = lin.least_damped(lin.loop(q, 0.03 * lin.MPA_MM,
                                         **{**lkw, **gains}))[0]
        t_d = lp['delay'] * lp['Ts'] * 1e3                      # [ms]
        say(f'{name:42s} {t_d:9.0f} {Kp:5.2f} '
            f'{100 * (Kp / Kp0 - 1):11.0f} {Kp * k_des:11.1f} '
            f'{dip:8.2f} {noise:15.2f} {zeta:10.2f}')
        rows.append(['6 delay', name, t_d, Kp, Kp * k_des, dip, noise,
                     zeta])
    return rows


# %% ===== PLOTS =========================================================
def plot_margin(path, par, cand, ks):
    """Plot the phase margin against stiffness for the candidate sets,
    with the chamber's states marked."""
    k = np.logspace(-3, math.log10(20.0), 90)               # [MPa/mm]
    w = lin.freq_grid(par.Ts, 1500, 1e-5)
    col = studyutil.colors(list(cand))
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for name, (Kp, Ki, Kd, Tf) in cand.items():
        pm = []
        for kk in k:
            lp = loop(par, kk, Kp, max(Ki, 1e-4 * Kp), Kd, Tf)
            pm.append(lin.margins(lp['L'], w)['pm']
                      if lin.is_stable(lp) else math.nan)
        ax.semilogx(k, pm, color=col[name], lw=1.4,
                    ls='--' if name == 'PI GVL' else '-', label=name)
    ax.axhline(PM_MIN, color='k', lw=0.8, ls=':')
    for kk, label in zip(ks, ('delivery', 'design', 'stiffest')):
        ax.axvline(kk, color='0.6', lw=0.8)
        ax.text(kk, 92, label, rotation=90, va='top', ha='right',
                fontsize=8, color='0.4')
    ax.axvspan(1e-3, 0.3, color='0.92', zorder=0)
    ax.text(1.2e-3, 91, 'free gas at low pressure', fontsize=8,
            color='0.4', va='top')
    ax.set_xlabel('chamber stiffness k (MPa/mm)')
    ax.set_ylabel('phase margin (deg)')
    ax.set_ylim(0, 95)
    ax.set_title('Phase margin against chamber stiffness')
    ax.legend(fontsize=8, frameon=False, loc='upper center',
              bbox_to_anchor=(0.5, -0.13), ncol=4)
    ax.grid(alpha=0.3, which='both')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def locus(par, Kp, Ki, Kd, Tf, k, w_max, **loop_kw):
    """Return the closed-loop poles [rad/s] of one gain set at each
    stiffness of k [MPa/mm], as (stiffness, pole) arrays: the roots in
    z mapped by s = ln(z) / Ts, upper half plane, up to w_max [rad/s]
    (which leaves out the poles the pure delay puts near the Nyquist
    frequency). loop_kw are further arguments of linear.loop()."""
    kk, ss = [], []
    for ki in k:
        z = lin.poles(lin.loop(
            par, ki * lin.MPA_MM, Kd=Kd, Tf=Tf, G_cfgPressureKp=Kp,
            G_cfgPressureKi=Ki, **loop_kw)).astype(complex)
        z = z[np.abs(z) > 1e-9]
        sp = np.log(z) / par.Ts
        sp = sp[(np.abs(sp) <= w_max) & (sp.imag >= -1e-9)]
        kk.extend([ki] * len(sp))
        ss.extend(sp)
    return np.array(kk), np.array(ss)


def plot_rlocus(path, par, cand, ks):
    """Plot the root locus against stiffness: the closed-loop poles as
    k runs from 0.001 to 25 MPa/mm (to 250 MPa/mm for the loop with its
    pure delay removed), over the whole loop (left) and near the
    origin, where the soft chamber's poles ride the circle of eq. (63)
    about the integral zero (right, drawn to equal scales). The poles
    at the chamber's states are marked, and the dotted rays are lines
    of constant damping ratio."""
    k = np.logspace(-3, math.log10(25.0), 500)              # [MPa/mm]
    marks = (0.03, 0.3) + ks                                # [MPa/mm]
    views = ((('PI GVL', 'PID, Tf 20 ms'), (-150.0, 30.0), (0.0, 110.0)),
             (('PI GVL', 'PI Ki/Kp 0.1', 'PID, Tf 20 ms'),
              (-1.0, 0.1), (0.0, 0.6)))
    col = studyutil.colors(['PI GVL', 'PID, Tf 20 ms', 'PI Ki/Kp 0.1'])
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.8))
    for a, (names, xlim, ylim) in zip(ax, views):
        for zeta in (0.2, 0.45, 0.7):
            # A ray of constant damping: Re = -zeta r, Im = r sqrt(1 -
            # zeta^2)
            r = 2.0 * max(-xlim[0], ylim[1])
            a.plot([0.0, -zeta * r], [0.0, r * math.sqrt(1 - zeta**2)],
                   color='0.6', lw=0.8, ls=':')
            y = 0.93 * ylim[1]
            x = -zeta / math.sqrt(1 - zeta**2) * y
            if x > xlim[0]:
                a.text(x, y, f'{zeta:g}', fontsize=7, color='0.4',
                       ha='right')
        for name in names:
            Kp, Ki, Kd, Tf = cand[name]
            _, sp = locus(par, Kp, Ki, Kd, Tf, k, 2.5 * ylim[1])
            a.plot(sp.real, sp.imag, '.', ms=2.5, color=col[name],
                   label=name)
            kk, sm = locus(par, Kp, Ki, Kd, Tf, marks, 2.5 * ylim[1])
            a.plot(sm.real, sm.imag, 'o', ms=5, mfc='none',
                   color=col[name])
            if name == 'PI GVL':
                for ki, si in zip(kk, sm):
                    # Label the dominant branch, not the delay's
                    # further ones
                    if (xlim[0] < si.real < xlim[1] and abs(si) < 40.0
                            and 0.02 * ylim[1] < si.imag < ylim[1]):
                        a.annotate(f'{ki:.3g}', (si.real, si.imag),
                                   textcoords='offset points',
                                   xytext=(5, 4), fontsize=8)
                # The integral zero, where the slow pole ends
                a.plot(-Ki / Kp, 0.0, 'o', ms=7, mfc='w', color='k',
                       zorder=5)
        if a is ax[0]:
            # The same PI loop with its pure delay removed: the lags
            # alone take it across the axis at a far higher stiffness.
            Kp, Ki, Kd, Tf = cand['PI GVL']
            k_far = np.logspace(-3, math.log10(250.0), 700)
            _, sp = locus(par, Kp, Ki, Kd, Tf, k_far, 400.0, delay=0,
                          cmd_delay=0)
            a.plot(sp.real, sp.imag, '.', ms=2.5, color='0.45',
                   label='PI GVL, no pure delay (to 250 MPa/mm)')
            kk, sm = locus(par, Kp, Ki, Kd, Tf, ks[1:], 400.0, delay=0,
                           cmd_delay=0)
            a.plot(sm.real, sm.imag, 'o', ms=5, mfc='none', color='0.45')
            a.text(-146.0, 44.0, 'further branches\nof the delay',
                   fontsize=8, color='0.4', va='top')
        a.axvline(0.0, color='k', lw=0.8)
        a.set_xlim(*xlim)
        a.set_ylim(*ylim)
        a.set_xlabel('real part (rad/s)')
        a.set_ylabel('imaginary part (rad/s)')
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8, frameon=False, loc='upper left',
                 bbox_to_anchor=(0.0, 0.3))
    ax[1].legend(fontsize=8, frameon=False, loc='upper left')
    ax[1].set_aspect('equal', adjustable='box')
    ax[0].set_title('Root locus against stiffness, 0.001 to 25 MPa/mm')
    ax[1].set_title('Near the origin: the soft chamber '
                    '(circles: MPa/mm)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_bode(path, par, cand, ks):
    """Plot the open loop per unit of stiffness for the PI and the PID
    sets: its gain, and its phase with the crossover of each stiffness
    marked."""
    w = np.logspace(-3, math.log10(math.pi / par.Ts), 1200)  # [rad/s]
    marks = (0.03, 0.3) + ks                                # [MPa/mm]
    fig, ax = plt.subplots(2, 1, figsize=(10, 7.5), sharex=True)
    col = studyutil.colors(['PI GVL', 'PID, Tf 20 ms'])
    for name in ('PI GVL', 'PID, Tf 20 ms'):
        Kp, Ki, Kd, Tf = cand[name]
        H = loop(par, 1.0, Kp, Ki, Kd, Tf)['L'].freq(w)
        mag = np.abs(H)                     # [-] per MPa/mm of stiffness
        ph = np.degrees(np.unwrap(np.angle(H)))
        ph -= 360.0 * np.ceil((ph[0] - 90.0) / 360.0)
        ax[0].loglog(w, mag, color=col[name], lw=1.3, label=name)
        ax[1].semilogx(w, ph, color=col[name], lw=1.3)
        for k in marks:
            j = np.flatnonzero(mag * k < 1.0)
            if j.size and j[0] > 0:
                ax[1].plot(w[j[0]], ph[j[0]], 'o', color=col[name],
                           ms=5)
                if name == 'PI GVL':
                    ax[1].annotate(f'{k:.3g}', (w[j[0]], ph[j[0]]),
                                   textcoords='offset points',
                                   xytext=(4, 6), fontsize=8)
    ax[1].axhline(-180.0 + PM_MIN, color='k', lw=0.8, ls=':')
    ax[1].axhline(-180.0, color='k', lw=0.8)
    ax[1].set_ylim(-270, -60)
    ax[0].set_ylabel('|L| per MPa/mm of stiffness')
    ax[1].set_ylabel('phase (deg)')
    ax[1].set_xlabel('frequency (rad/s)')
    ax[0].set_title('Open loop; dots mark the crossover at each '
                    'stiffness (MPa/mm)')
    ax[0].legend(fontsize=8, frameon=False)
    for a in ax:
        a.grid(alpha=0.3, which='both')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_grid(path, grid):
    """Plot the nonlinear grid: the stiff-step overshoot, the deepest
    dip and the soft 5 atm overshoot against Kp and Kd."""
    items = (('stiff', 'stiff step overshoot (MPa)'),
             ('dip', 'cycle: deepest dip (MPa)'),
             ('soft5', '5 atm step overshoot (MPa)'))
    fig, ax = plt.subplots(1, 3, figsize=(13, 4.2))
    for a, (key, title) in zip(ax, items):
        z = np.array([[grid[(kp, kd)][key] for kd in GRID_KD]
                      for kp in GRID_KP])
        im = a.imshow(z, origin='lower', aspect='auto', cmap='viridis')
        for i in range(len(GRID_KP)):
            for j in range(len(GRID_KD)):
                a.text(j, i, f'{z[i, j]:.2f}', ha='center', va='center',
                       color='w', fontsize=8)
        a.set_xticks(range(len(GRID_KD)), [f'{v:g}' for v in GRID_KD])
        a.set_yticks(range(len(GRID_KP)), [f'{v:g}' for v in GRID_KP])
        a.set_xlabel('Kd (mm/s per MPa/s)')
        a.set_ylabel('Kp (mm/s per MPa)')
        a.set_title(title, fontsize=10)
        fig.colorbar(im, ax=a, shrink=0.85)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# %% ===== MAIN ==========================================================
def main():
    ap = studyutil.parser(__doc__.split('\n\n')[1])
    args = ap.parse_args()
    par, legacy, out = studyutil.setup(args)
    ks = chamber_stiffness(par)

    rows, cand = linear_tables(par, ks)
    g_rows, grid = grid_tables(par, ks, args.seed, legacy, args.workers)
    rows += g_rows
    rows += candidate_table(par, cand, ks, args.seed, legacy,
                            args.workers)
    rows += delay_table(par, ks)

    (out / 'compensation_summary.txt').write_text('\n'.join(LOG) + '\n')
    with open(out / 'compensation.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['table', 'values'])
        w.writerows(rows)
    plot_rlocus(out / 'compensation_rlocus.png', par, cand, ks)
    plot_margin(out / 'compensation_margin.png', par, cand, ks)
    plot_bode(out / 'compensation_bode.png', par, cand, ks)
    plot_grid(out / 'compensation_grid.png', grid)
    print(f'\nwrote {out}/compensation_summary.txt, compensation.csv, '
          f'compensation_rlocus.png, compensation_margin.png, '
          f'compensation_bode.png, compensation_grid.png')


if __name__ == '__main__':
    main()
