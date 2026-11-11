#!/usr/bin/env python3
"""
study_antiwindup.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
11/10/2026

PURPOSE/DESCRIPTION:
This script compares ways of managing the integrator of the slave's
pressure loop, and a derivative term, against the loop as it is
(README, "Anti-windup and derivative study", eqs. (50) to (56)). The
arms are controllers.VariantController: the slave's block with one
anti-windup rule or a derivative gain changed around it.

With free gas at a low pressure the chamber is softer than the gains
are set for by two orders of magnitude. The proportional term then
moves the piston too slowly to arrive, the integrator carries the
piston to the target, and the velocity it holds on arrival is unwound
only by an error above the target. With the outlet open the same
integrator holds the velocity that balances the outflow at zero error.
The cases cover both:

  soft steps      The bench fill of 2026-11-09 (279 mL of saline, 293 mL
                  of air at atmosphere, hose shut) stepped to 5, 20 and
                  100 atm absolute.
  soft direct     The same fill taken to 100 atm in one step.
  cycle           The 1:1 oxygen cycle at 10 MPa (compress, hold, open
                  the hose, purge, close, deliver), with the hose
                  opened before the compression and after mixing.
  brief closing   The same cycle with the outlet shut for 0.5 s in
                  place of the 10 s pause: the integral that balanced
                  the outflow is needed again at once.
  delivery step   A delivery at 9 MPa whose target steps to 10 MPa
                  while the outlet is open: an approach with an outflow
                  to balance on arrival.
  stiff step      The liquid-only chamber stepped from 10 to 12 MPa
                  with the hose shut: the margin probe.

A last table repeats the cycle with more sensor noise for the rules
that act on the error's sign, and counts how often they change the
integral while the outlet is open.

It prints a table per case, writes them to antiwindup_summary.txt and
antiwindup.csv, and plots the soft steps and the purge closing to
antiwindup_soft.png and antiwindup_closing.png in the output directory.

Usage:
    python3 study_antiwindup.py [--set key=value ...] [--out DIR]
                                [--workers N] [--seed N]

FILE DEPENDENCY:
./
    controllers.py
    runner.py
    scenarios.py
    sim.py
    study_floor.py (delivery_stats)
    studyutil.py
    (and the modules they import: sim.py, metrics.py, and models/)
"""

import csv
import math
import multiprocessing as mp
import os

import numpy as np

import controllers as ctl
import scenarios
import sim
import studyutil
from runner import Case, run_cases
from study_floor import delivery_stats

plt = studyutil.plt


# %% ===== STUDY ARMS ====================================================
# (label, VariantController arguments). The first arm is the slave as
# configured in the GVL.
ARMS = [
    ('GVL', {}),
    ('arrival', {'aw': 'arrival'}),
    ('crossing', {'aw': 'crossing'}),
    ('opposed 0.02', {'aw': 'opposed', 'over': 0.02}),
    ('opposed 0.1', {'aw': 'opposed', 'over': 0.1}),
    ('bleed 0.2 s', {'aw': 'bleed', 'over': 0.02, 'Tb': 0.2}),
    ('backcalc 0.5 s', {'aw': 'backcalc', 'Tt': 0.5}),
    ('backcalc 2.5 s', {'aw': 'backcalc', 'Tt': 2.5}),
    ('band 0.1', {'aw': 'band', 'band': 0.1}),
    ('band 1.5', {'aw': 'band', 'band': 1.5}),
    ('PID Kd 0.02', {'Kd': 0.02}),
    ('PID Kd 0.05', {'Kd': 0.05}),
    ('PID Kd 0.1', {'Kd': 0.1}),
    ('PID Kd 0.05, bleed', {'Kd': 0.05, 'aw': 'bleed'}),
    ('GVL + floor', {'FloorEnable': True}),
    ('arrival + floor', {'aw': 'arrival', 'FloorEnable': True}),
    ('opposed + floor', {'aw': 'opposed', 'over': 0.02,
                         'FloorEnable': True}),
]
# Arms drawn in the plots
PLOT_ARMS = ('GVL', 'arrival', 'opposed 0.02', 'bleed 0.2 s', 'crossing',
             'PID Kd 0.05')
# Arms and sensor noise [Pa rms per sample] of the noise table: the
# measured 1.7 kPa with the capacitor, 4.6 kPa bare, and 10 and 30 kPa
NOISE_ARMS = ('GVL', 'arrival', 'opposed 0.02', 'opposed 0.1',
              'bleed 0.2 s')
NOISE = (1.7e3, 4.6e3, 10e3, 30e3)


# %% ===== CASES =========================================================
ATM = 101325.0                                  # [Pa] one atmosphere
# Bench fill of 2026-11-09 (bench/captures/trace_20261109_190837.csv)
V_SALINE = 279e-6           # [m^3] 222 mL swept and 57 mL of passages
V_AIR = 292.6e-6            # [m^3] air drawn in at atmosphere
# [s] Times of the soft steps and the end of the run
T_SOFT = (0.0, 100.0, 150.0, 180.0)
# [atm absolute] Targets of the soft steps
P_SOFT = (5.0, 20.0, 100.0)
# [s] Time of the target step in the delivery-step case
T_DSTEP = 45.0


def gauge(atm):
    """[Pa] Gauge pressure of atm atmospheres absolute."""
    return (atm - 1.0) * ATM                    # [atm -> Pa gauge]


def soft_params(par):
    """Return (StudyParams, ratio) of the bench fill: its saline
    volume, and its air as an STP ratio (the charge is n = ratio Vl0
    Patm / (R 273.15 K), so air at the model's temperature T counts
    273.15 / T of its volume)."""
    q = par.with_(Vl0=V_SALINE)
    return q, V_AIR / V_SALINE * 273.15 / q.T


def cases(par):
    """Return the cases as (label, StudyParams, Scenario)."""
    q, ratio = soft_params(par)
    steps = tuple((t, gauge(p)) for t, p in zip(T_SOFT, P_SOFT))
    hb = par.with_(hose_before=True)
    return [
        ('soft steps', q, scenarios.Scenario(
            'softsteps', T=T_SOFT[-1], tEvent=0.0, P0=0.0, ratio=ratio,
            target0=steps[0][1], targets=steps)),
        ('soft direct', q, scenarios.Scenario(
            'softdirect', T=40.0, tEvent=0.0, P0=0.0, ratio=ratio,
            target0=gauge(100.0), target1=gauge(100.0))),
        ('cycle, hose before', hb, scenarios.cycle(hb)),
        ('cycle, hose after', par, scenarios.cycle(par)),
        ('brief closing', hb, scenarios.cycle(hb, t_pause=0.5)),
        ('delivery step', hb, scenarios.deliver(
            hb, P0=9e6, target0=9e6, target1=10e6, t_step=T_DSTEP)),
        ('stiff step', par, scenarios.stiffstep(par)),
    ]


# %% ===== REPORT LOG ====================================================
LOG = []


def say(text=''):
    """Print a line and keep it for the summary file."""
    print(text, flush=True)
    LOG.append(text)


# %% ===== METRICS =======================================================
def step_stats(r, t0, t1, tol):
    """Return the metrics of the upward target step of run r in the
    window t0 <= t < t1 [s], with the band half-width tol [Pa]:

      P0      [MPa] Pressure at the step.
      tCross  [s] From the step to the first scan at the target; NaN
              if it is not reached.
      vCross  [mm/s] Piston velocity at that scan.
      over    [MPa] Highest pressure above the target.
      overPct [%] The same as a share of the step.
      tBand   [s] From the step to the last scan outside the band; NaN
              if the window ends outside it.
    """
    w = np.flatnonzero((r.t >= t0) & (r.t < t1))
    P, tgt = r.P[w], r.target[w][0]                         # [Pa]
    P0 = P[0]
    out = {'P0': P0 / 1e6}                                  # [Pa -> MPa]
    hit = np.flatnonzero(P >= tgt)
    if hit.size:
        out['tCross'] = float(r.t[w][hit[0]] - t0)
        out['vCross'] = float(r.v[w][hit[0]]) * 1e3     # [m/s -> mm/s]
    else:
        out['tCross'] = out['vCross'] = math.nan
    over = max(float(np.max(P - tgt)), 0.0)                 # [Pa]
    out['over'] = over / 1e6
    out['overPct'] = 100.0 * over / (tgt - P0)
    outside = np.flatnonzero(np.abs(P - tgt) > tol)
    if outside.size == 0:
        out['tBand'] = 0.0
    elif outside[-1] == len(w) - 1:
        out['tBand'] = math.nan
    else:
        out['tBand'] = float(r.t[w][outside[-1] + 1] - t0)
    return out


def cycle_stats(r, c, par, sc):
    """Return the metrics of cycle run r under controller c:

      over       [MPa] Compression overshoot, before the hose opens.
      tArrive    [s] First scan within the band.
      dip        [MPa] Deepest dip within t_recover of a valve event.
      closePeak  [MPa] Highest pressure after the purge closing.
      tHigh      [s] From the purge closing to the last scan above the
                 band before the reopening.
      reDip      [MPa] Deepest dip within t_recover of the reopening.
      eMean      [MPa] Mean and standard deviation of the error over
      eStd       the delivery stage (study_floor.delivery_stats).
      margin     [MPa] Smallest margin over the saturation pressure.
      V_ok       [mL] Delivered before the first violation.
    """
    pre = r.t < sc.tEvent
    out = {'over': float(np.max(r.P[pre] - r.target[pre])) / 1e6,
           'tArrive': r['t_arrive'], 'dip': r['dipMax'],
           'margin': r['degasMargin'], 'V_ok': r['V_ok']}
    d = delivery_stats(r, c, par)
    out.update(closePeak=d['closePeak'], eMean=d['eMean'],
               eStd=d['eStd'])
    out['tHigh'] = out['reDip'] = math.nan
    if len(r.events) >= 3:
        t_close, t_reopen = r.events[-2], r.events[-1]
        w = np.flatnonzero((r.t >= t_close) & (r.t < t_reopen))
        high = np.flatnonzero(r.P[w] - r.target[w] > par.tol)
        out['tHigh'] = (float(r.t[w][high[-1]] - t_close)
                        if high.size else 0.0)
        w = (r.t >= t_reopen) & (r.t < t_reopen + par.t_recover)
        if w.any():
            out['reDip'] = float(np.max(r.target[w] - r.P[w])) / 1e6
    return out


def dstep_stats(r, par):
    """Return the metrics of the delivery-step run r, whose target
    steps at T_DSTEP with the outlet open:

      tCross  [s] From the step to the first scan at the new target.
      over    [MPa] Highest pressure above the new target afterwards.
      redip   [MPa] Deepest fall below the new target after it was
              first reached.
      tBand   [s] From the step to the last scan outside the band; NaN
              if the run ends outside it.
    """
    w = np.flatnonzero(r.t >= T_DSTEP)
    P, tgt = r.P[w], r.target[w][0]                         # [Pa]
    out = {'tCross': math.nan, 'redip': math.nan}
    hit = np.flatnonzero(P >= tgt)
    if hit.size:
        out['tCross'] = float(r.t[w][hit[0]] - T_DSTEP)
        out['redip'] = float(tgt - np.min(P[hit[0]:])) / 1e6
    out['over'] = max(float(np.max(P - tgt)), 0.0) / 1e6
    outside = np.flatnonzero(np.abs(P - tgt) > par.tol)
    if outside.size == 0:
        out['tBand'] = 0.0
    elif outside[-1] >= len(w) - 2:
        out['tBand'] = math.nan
    else:
        out['tBand'] = float(r.t[w][outside[-1] + 1] - T_DSTEP)
    return out


# %% ===== TABLES ========================================================
def fmt(x, spec='.2f'):
    """Format a number, with a dash for NaN."""
    return '—' if (isinstance(x, float) and math.isnan(x)) \
        else format(x, spec)


def report(res, par_of, sc_of, ctl_of):
    """Print the tables of the results res[case][arm] and return the
    rows of the CSV."""
    rows = []
    names = [a for a, _ in ARMS]

    say('\n=== soft steps: bench fill stepped to 5, 20 and 100 atm')
    say('    crossing time s / overshoot MPa (share of the step) / '
        'in band after s')
    say(f'{"arm":22s} {"5 atm":>26s} {"20 atm":>26s} {"100 atm":>24s}')
    for a in names:
        r = res['soft steps'][a]
        cells = []
        for k, p in enumerate(P_SOFT):
            s = step_stats(r, T_SOFT[k], T_SOFT[k + 1],
                           par_of['soft steps'].tol)
            cells.append(f'{fmt(s["tCross"], ".1f"):>5s} / '
                         f'{s["over"]:.3f} ({s["overPct"]:3.0f} %) / '
                         f'{fmt(s["tBand"], ".1f"):>5s}')
            rows.append(['soft steps', a, f'{p:g} atm', s['tCross'],
                         s['over'], s['overPct'], s['tBand']])
        say(f'{a:22s} ' + '  '.join(cells))

    say('\n=== soft direct: bench fill taken to 100 atm in one step')
    say(f'{"arm":22s} {"cross s":>8s} {"over MPa":>9s} {"band s":>7s}')
    for a in names:
        r = res['soft direct'][a]
        s = step_stats(r, 0.0, r.t[-1] + 1.0, par_of['soft direct'].tol)
        say(f'{a:22s} {fmt(s["tCross"], ".1f"):>8s} {s["over"]:9.3f} '
            f'{fmt(s["tBand"], ".1f"):>7s}')
        rows.append(['soft direct', a, '100 atm', s['tCross'], s['over'],
                     s['overPct'], s['tBand']])

    for case in ('cycle, hose before', 'cycle, hose after',
                 'brief closing'):
        say(f'\n=== {case}: 1:1 oxygen at 10 MPa')
        say(f'{"arm":22s} {"over":>6s} {"arrive":>7s} {"dip":>6s} '
            f'{"close":>7s} {"high s":>7s} {"reDip":>6s} {"eMean":>7s} '
            f'{"eStd":>6s} {"margin":>7s} {"V_ok":>6s}')
        for a in names:
            r = res[case][a]
            s = cycle_stats(r, ctl_of[case][a], par_of[case],
                            sc_of[case])
            say(f'{a:22s} {s["over"]:6.2f} {fmt(s["tArrive"], ".1f"):>7s}'
                f' {s["dip"]:6.2f} {fmt(s["closePeak"]):>7s} '
                f'{fmt(s["tHigh"], ".1f"):>7s} {fmt(s["reDip"]):>6s} '
                f'{fmt(s["eMean"]):>7s} {fmt(s["eStd"]):>6s} '
                f'{fmt(s["margin"]):>7s} {s["V_ok"]:6.0f}')
            rows.append([case, a, 'cycle', s['over'], s['tArrive'],
                         s['dip'], s['closePeak'], s['tHigh'],
                         s['reDip'], s['eMean'], s['eStd'], s['margin'],
                         s['V_ok']])

    say('\n=== delivery step: 9 to 10 MPa with the outlet open')
    say(f'{"arm":22s} {"cross s":>8s} {"over MPa":>9s} '
        f'{"re-dip MPa":>11s} {"band s":>7s}')
    for a in names:
        s = dstep_stats(res['delivery step'][a], par_of['delivery step'])
        say(f'{a:22s} {fmt(s["tCross"], ".1f"):>8s} {s["over"]:9.2f} '
            f'{fmt(s["redip"]):>11s} {fmt(s["tBand"], ".1f"):>7s}')
        rows.append(['delivery step', a, '9 to 10 MPa', s['tCross'],
                     s['over'], s['redip'], s['tBand']])

    say('\n=== stiff step: liquid only, 10 to 12 MPa, hose shut')
    say(f'{"arm":22s} {"over MPa":>9s} {"settle s":>9s} '
        f'{"ripple MPa":>11s} {"cmd activity mm/s":>18s}')
    for a in names:
        r = res['stiff step'][a]
        say(f'{a:22s} {r["overshoot"]:9.2f} '
            f'{fmt(r["tSettle"], ".2f"):>9s} {r["ripple"]:11.4f} '
            f'{r["uAct"]:18.4f}')
        rows.append(['stiff step', a, '10 to 12 MPa', r['overshoot'],
                     r['tSettle'], r['ripple'], r['uAct']])
    return rows


# %% ===== NOISE TABLE ===================================================
def noise_job(job):
    """Run the cycle of one (StudyParams, Scenario, arm, arguments,
    seed) and return the metrics of its delivery stage (from t_recover
    after the reopening to 1 s before the end):

      changes    [-] Episodes in which a rule changed the integral
                 (runs of consecutive scans count once).
      eStd       [MPa] Standard deviation of the error.
      eMax       [MPa] Deepest fall below the target.
      closePeak  [MPa] Highest pressure after the purge closing.
    """
    par, sc, arm, kw, seed = job
    c = ctl.VariantController(arm, **kw)
    r = sim.run(par, sc, c, seed=seed)
    t_close, t_reopen = r.events[-2], r.events[-1]
    late = (r.t > t_reopen + par.t_recover) & (r.t < r.t[-1] - 1.0)
    k = np.flatnonzero(late)
    ch = np.array([n for n in c.changed if k[0] <= n <= k[-1]])
    episodes = 0 if ch.size == 0 else 1 + int(np.sum(np.diff(ch) > 1))
    e = (r.target[late] - r.P[late]) / 1e6                  # [MPa]
    w = (r.t >= t_close) & (r.t < t_reopen)
    return {'changes': episodes, 'eStd': float(np.std(e)),
            'eMax': float(np.max(e)),
            'closePeak': float(np.max(r.P[w])) / 1e6}


def noise_table(par, seed, workers):
    """Run the cycle (hose opened before the compression) at each
    noise level for the arms of NOISE_ARMS, print the table and return
    its CSV rows."""
    arms = dict(ARMS)
    jobs, index = [], []
    for noise in NOISE:
        q = par.with_(hose_before=True, noise=noise)
        sc = scenarios.cycle(q)
        for a in NOISE_ARMS:
            jobs.append((q, sc, a, arms[a], seed))
            index.append((noise, a))
    n = min(workers or os.cpu_count() or 1, len(jobs))
    with mp.get_context('spawn').Pool(n) as pool:
        stats = pool.map(noise_job, jobs, chunksize=1)
    say('\n=== sensor noise: cycle, hose before, delivery stage')
    say(f'{"noise kPa":>9s} {"arm":16s} {"changes":>8s} {"eStd":>6s} '
        f'{"eMax":>6s} {"close":>7s}')
    rows = []
    for (noise, a), s in zip(index, stats):
        say(f'{noise / 1e3:9.1f} {a:16s} {s["changes"]:8d} '
            f'{s["eStd"]:6.2f} {s["eMax"]:6.2f} {s["closePeak"]:7.2f}')
        rows.append(['noise', a, f'{noise / 1e3:g} kPa', s['changes'],
                     s['eStd'], s['eMax'], s['closePeak']])
    return rows


# %% ===== PLOTS =========================================================
def plot_window(path, title, runs, t0, t1, ylim=None):
    """Plot the pressure and the command of the runs {arm: Result}
    between t0 and t1 [s]."""
    col = studyutil.colors(list(runs))
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    first = next(iter(runs.values()))
    w = (first.t >= t0) & (first.t < t1)
    ax[0].plot(first.t[w], first.target[w] / 1e6, 'k--', lw=1,
               label='target')
    for name, r in runs.items():
        w = (r.t >= t0) & (r.t < t1)
        # The GVL arm is dashed and drawn on top, so it stays visible
        # where another arm coincides with it.
        ls, z = ('--', 5) if name == 'GVL' else ('-', 2)
        ax[0].plot(r.t[w], r.P[w] / 1e6, color=col[name], ls=ls, lw=1.1,
                   label=name, zorder=z)                # [Pa -> MPa]
        ax[1].plot(r.t[w], r.u[w] * 1e3, color=col[name], ls=ls,
                   lw=1.0, zorder=z)                    # [m/s -> mm/s]
    ax[0].set_ylabel('P (MPa)')
    ax[1].set_ylabel('velocity cmd (mm/s)')
    ax[1].set_xlabel('t (s)')
    if ylim:
        ax[0].set_ylim(*ylim)
    ax[0].set_title(title)
    ax[0].legend(loc='center left', bbox_to_anchor=(1.01, 0.5),
                 fontsize=8, frameon=False)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# %% ===== MAIN ==========================================================
def main():
    ap = studyutil.parser(__doc__.split('\n\n')[1])
    args = ap.parse_args()
    par, legacy, out = studyutil.setup(args)

    todo = cases(par)
    jobs, index = [], []
    ctl_of = {}
    for case, q, sc in todo:
        ctl_of[case] = {}
        for arm, kw in ARMS:
            c = ctl.VariantController(arm, **kw)
            c.reset(q, sc)          # keeps its floor field for the stats
            ctl_of[case][arm] = c
            jobs.append(Case(q, sc, ctl.VariantController(arm, **kw),
                             legacy_sensor=legacy, seed=args.seed))
            index.append((case, arm))
    results = run_cases(jobs, workers=args.workers)
    res = {case: {} for case, _, _ in todo}
    for (case, arm), r in zip(index, results):
        res[case][arm] = r
    par_of = {case: q for case, q, _ in todo}
    sc_of = {case: sc for case, _, sc in todo}

    rows = report(res, par_of, sc_of, ctl_of)
    rows += noise_table(par, args.seed, args.workers)
    (out / 'antiwindup_summary.txt').write_text('\n'.join(LOG) + '\n')
    with open(out / 'antiwindup.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['case', 'arm', 'item', 'values'])
        w.writerows(rows)

    soft = {a: res['soft steps'][a] for a in PLOT_ARMS}
    plot_window(out / 'antiwindup_soft.png',
                'Bench fill stepped to 5 and 20 atm', soft, 0.0,
                T_SOFT[2], ylim=(0.0, 3.0))
    cyc = {a: res['cycle, hose before'][a] for a in PLOT_ARMS}
    t_close = res['cycle, hose before']['GVL'].events[-2]
    plot_window(out / 'antiwindup_closing.png',
                'Purge closing and reopening, 1:1 at 10 MPa', cyc,
                t_close - 3.0, t_close + 25.0, ylim=(8.5, 11.5))
    print(f'\nwrote {out}/antiwindup_summary.txt, antiwindup.csv, '
          f'antiwindup_soft.png, antiwindup_closing.png')


if __name__ == '__main__':
    main()
