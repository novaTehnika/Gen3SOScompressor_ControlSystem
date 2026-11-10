#!/usr/bin/env python3
"""
study_feedforward.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
11/10/2026

PURPOSE/DESCRIPTION:
This script tests a velocity feedforward for the outflow that is
switched by the measured pressure alone (README, "Outflow feedforward
study", eqs. (64) to (66)). The arm is
controllers.FeedforwardController: the slave's block, with the outflow's
speed added while a detector finds the outlet open.

With the outlet open the block finds the speed that balances the
outflow only through its error, so the pressure dips at every opening
and the integral that then holds the speed runs the piston on when the
outlet shuts. A feedforward told of the valve removes most of both
(study_delivery.py carries it as a bound), but the valves are worked by
hand and the slave has no signal of them. The detector takes the
opening from the fall of the pressure and the closing from its rise.

The arms are the block as configured in the GVL, the block with its
integral clear past the target, the feedforward from the valve itself
(the bound), the detector's feedforward, and the last two again with a
shorter pressure filter (a median of 3 and a 5 ms low-pass in place of
the median of 5 and 20 ms).

  1. Nominal cases   Deliveries at 1:1, 2:1 and 2.5:1, at 5, 10 and
                     11 MPa, with the hose opened before the compression
                     and after mixing (the hose joining the chamber is
                     a fall the detector must not take for an outflow)
                     and with the valve worked slowly; and three cases
                     with no outflow, in which it must stay off.
  2. Settings        The detector's settings one at a time, a
                     feedforward that is too small or too large, and
                     more sensor noise.
  3. Monte Carlo     With --mc N, N Latin hypercube samples over the
                     ranges of robust.py for five charges and
                     procedures, against the requirements of robust.py,
                     with the feedforward sized for the nominal orifice.

It prints the tables, writes them to feedforward_summary.txt and
feedforward.csv (and feedforward_mc.csv), and plots the first opening
and the purge closing to feedforward_traces.png in the output
directory.

Usage:
    python3 study_feedforward.py [--set key=value ...] [--out DIR]
                                 [--workers N] [--seed N] [--mc N]

FILE DEPENDENCY:
./
    controllers.py
    robust.py
    scenarios.py
    sim.py
    study_antiwindup.py (cases)
    studyutil.py
    (and the modules they import: metrics.py, studyparams.py, and
    models/)
"""

import csv
import math
import multiprocessing as mp
import os
import time

import numpy as np

import controllers as ctl
import robust as R
import scenarios
import sim
import studyutil
from study_antiwindup import cases as soft_cases
from studyparams import StudyParams

plt = studyutil.plt


# %% ===== STUDY ARMS ====================================================
# (label, kind, controller arguments). kind 'slave' is
# controllers.SlaveController and 'detect' FeedforwardController.
CLEAR = {'IntegClearEnable': True}
# The shorter pressure filter: median of 3 and a 5 ms low-pass [s]
SHORT = {'AnalogMedianSize': 3, 'AnalogFilterTimeConst': 0.005}
ARMS = [
    ('GVL', 'slave', {}),
    ('GVL + clear', 'slave', CLEAR),
    ('valve signal + clear', 'slave', dict(flow_ff=True, **CLEAR)),
    ('detector', 'detect', {}),
    ('detector + clear', 'detect', CLEAR),
    ('detector + clear + floor', 'detect',
     dict(FloorEnable=True, **CLEAR)),
    ('GVL + clear, short filter', 'slave', dict(**CLEAR, **SHORT)),
    ('detector + clear, short filter', 'detect',
     dict(**CLEAR, **SHORT)),
]
# Arms of the Monte Carlo and of the plot
MC_ARMS = ('GVL + clear', 'valve signal + clear', 'detector + clear',
           'GVL + clear, short filter',
           'detector + clear, short filter')
PLOT_ARMS = ('GVL + clear', 'valve signal + clear', 'detector + clear',
             'detector + clear, short filter')
# The arm whose settings part 2 changes
SWEEP_ARM = 'detector + clear'
# (argument, values) of part 2; 'noise' is the sensor noise of the run
# [Pa rms per sample]
SWEEP = [
    ('scale', (0.7, 1.3, 1.5)),
    ('d_on', (0.02, 0.05, 0.1)),
    ('t_on', (0.0, 0.016)),
    ('window', (0.012, 0.04)),
    ('d_off', (0.0, 0.02)),
    ('e_hi', (0.15,)),
    ('e_max', (0.5, 100.0)),
    ('noise', (4.6e3, 10e3, 30e3)),
]
SWEEP_CASES = ('1:1', '1:1, hose after mixing', '1:1, slow valve')
# (label, ratio, target [Pa] or None, hose opened before) of part 3
MC_CASES = [
    ('1:1 at 10 MPa', 1.0, None, True),
    ('2:1 at 10 MPa', 2.0, None, True),
    ('2.5:1 at 10 MPa', 2.5, None, True),
    ('2.5:1 at 11 MPa', 2.5, 11e6, True),
    ('1:1 at 10 MPa, hose after mixing', 1.0, None, False),
]


def configured_coef():
    """[(mm/s)/sqrt(MPa)] Outflow speed at the piston per root of the
    target for the nominal orifice, eq. (36): what the feedforward is
    configured with, whatever the run's own orifice."""
    p = StudyParams()
    # [m^3/s / m^2 -> m/s, * 1e3 -> mm/s]
    return p.Q_orifice(1e6) / p.A * 1e3


def make_arm(label, kind, kw, coef):
    """Return the controller of an arm."""
    if kind == 'detect':
        return ctl.FeedforwardController(label, coef=coef, **kw)
    return ctl.SlaveController(label, **kw)


# %% ===== CASES =========================================================
def outflow_cases(par):
    """Return the delivery cases as (label, StudyParams, Scenario)."""
    hb = par.with_(hose_before=True)
    out = []
    for label, q in (
            ('1:1', hb),
            ('2:1', hb.with_(ratio=2.0)),
            ('2.5:1', hb.with_(ratio=2.5)),
            ('2.5:1 at 11 MPa', hb.with_(ratio=2.5, P_target=11e6)),
            ('1:1 at 5 MPa', hb.with_(P_target=5e6)),
            ('1:1, hose after mixing', par),
            # The distal valve opened over 0.5 s and shut over 1 s
            ('1:1, slow valve', hb.with_(Topen=0.5, Tclose=1.0))):
        out.append((label, q, scenarios.deliver(q)))
    return out


def sealed_cases(par):
    """Return the cases with no outflow, in which the detector must
    stay off, as (label, StudyParams, Scenario)."""
    hb = par.with_(hose_before=True, ratio=2.0)
    soft = [c for c in soft_cases(par) if c[0] == 'soft steps'][0]
    # A target taken down 2 MPa and back with the chamber sealed
    down = scenarios.Scenario(
        'down', T=20.0, tEvent=0.0, P0=par.P_target, ratio=par.ratio,
        dissolved=True, target0=par.P_target, hose_open0=True,
        targets=((5.0, par.P_target - 2e6), (12.0, par.P_target)))
    return [('compression, 2:1', hb, scenarios.compress(hb)),
            soft,
            ('target down 2 MPa and back', par, down)]


# %% ===== REPORT LOG ====================================================
LOG = []


def say(text=''):
    """Print a line and keep it for the summary file."""
    print(text, flush=True)
    LOG.append(text)


# %% ===== METRICS =======================================================
def ff_stats(r, c, par):
    """Return the metrics of run r under controller c:

      dip        [MPa] Deepest dip within t_recover of a valve event.
      tBack      [s] From the first opening to the pressure back within
                 the band after its dip; NaN if it does not return.
      closePeak  [MPa] Highest pressure between the purge closing and
                 the reopening; NaN without a purge.
      peak       [MPa] Highest pressure of the run.
      margin     [MPa] Smallest margin over the saturation pressure.
      V_ok       [mL] Delivered before the first violation.
      ons        [-] Times the feedforward switched on.
      falseOns   [-] Of those, with no flow through the outlet.
      tDet       [s] Longest time from an opening of the outlet to the
                 feedforward on; NaN if an opening was missed.
      tOff       [s] Longest time from a closing to it off.
    """
    m = r.metrics
    out = {'dip': m.get('dipMax', math.nan), 'peak': m['P_max'],
           'margin': m.get('degasMargin', math.nan),
           'V_ok': m.get('V_ok', math.nan),
           'tBack': math.nan, 'closePeak': math.nan}
    flow = r.Qout > 0.0
    edge = np.diff(flow.astype(int))
    opens = np.flatnonzero(edge == 1) + 1       # first scans with flow
    closes = np.flatnonzero(edge == -1) + 1     # first scans without
    if opens.size:
        w = np.arange(opens[0], min(len(r.t),
                                    opens[0] + round(10.0 / par.Ts)))
        e = r.target[w] - r.P[w]                            # [Pa]
        back = np.flatnonzero(np.abs(e[np.argmax(e):]) < par.tol)
        if back.size:
            out['tBack'] = float((np.argmax(e) + back[0]) * par.Ts)
    if len(r.events) >= 3:
        w = (r.t >= r.events[-2]) & (r.t < r.events[-1])
        if w.any():
            out['closePeak'] = float(np.max(r.P[w])) / 1e6
    ons = np.array(getattr(c, 'ons', []), dtype=int)
    offs = np.array(getattr(c, 'offs', []), dtype=int)
    out['ons'] = int(ons.size)
    out['falseOns'] = int(np.count_nonzero(~flow[ons])) if ons.size \
        else 0
    out['tDet'] = out['tOff'] = math.nan
    if hasattr(c, 'ons'):
        out['tDet'] = _longest_wait(opens, ons, closes, par.Ts)
        out['tOff'] = _longest_wait(closes, offs, opens, par.Ts)
    return out


def _longest_wait(starts, hits, ends, Ts):
    """[s] Longest time from a scan of starts to the first scan of
    hits at or after it; NaN if a start has no hit before the next scan
    of ends, or if there is no start."""
    worst = math.nan
    for k in starts:
        later = hits[hits >= k]
        stop = ends[ends > k]
        if later.size == 0 or (stop.size and later[0] > stop[0]):
            return math.nan
        wait = (later[0] - k) * Ts
        worst = wait if math.isnan(worst) else max(worst, wait)
    return worst


def run_job(job):
    """Run one arm on one case and return its metrics.

    job is (label, kind, controller arguments, coef, StudyParams,
    Scenario, seed).
    """
    label, kind, kw, coef, par, sc, seed = job
    c = make_arm(label, kind, kw, coef)
    r = sim.run(par, sc, c, seed=seed)
    return ff_stats(r, c, par)


def run_jobs(fn, jobs, workers):
    """Return fn(job) for each job, in order, on a spawn-started
    pool."""
    n = min(workers or os.cpu_count() or 1, len(jobs))
    if n <= 1:
        return [fn(j) for j in jobs]
    with mp.get_context('spawn').Pool(n) as pool:
        return pool.map(fn, jobs, chunksize=1)


COLS = [('dip', 'dip', '{:6.2f}'), ('tBack', 'back s', '{:7.2f}'),
        ('closePeak', 'close', '{:7.2f}'), ('margin', 'margin', '{:7.2f}'),
        ('V_ok', 'V_ok', '{:5.0f}'), ('ons', 'on', '{:3d}'),
        ('falseOns', 'false', '{:5d}'), ('tDet', 'tDet', '{:6.3f}'),
        ('tOff', 'tOff', '{:6.3f}')]


def fmt(f, v):
    """Format v with f, or a dash for a NaN."""
    if isinstance(v, float) and math.isnan(v):
        return '-'.rjust(len(f.format(0)))
    return f.format(v)


def table(title, keys, stats, width):
    """Print the rows of stats under title and return its CSV rows.
    keys is the (case, row label) of each row."""
    say(f'\n--- {title}')
    say(f'{"":{width}s} ' + ' '.join(
        f'{h:>{len(f.format(0))}s}' for _, h, f in COLS))
    rows = []
    for (case, label), s in zip(keys, stats):
        say(f'{label:{width}s} '
            + ' '.join(fmt(f, s[k]) for k, _, f in COLS))
        rows.append([case, label] + [s[k] for k, _, _ in COLS])
    return rows


# %% ===== 1. NOMINAL CASES ==============================================
def nominal(par, coef, seed, workers):
    """Run part 1 and return its CSV rows."""
    say('=== 1. nominal cases')
    say('dip, close, margin [MPa]; back: first opening to back in '
        'band; on, false: times the feedforward switched on, and of '
        'those with no flow; tDet, tOff: longest wait from an opening '
        'and from a closing [s]')
    todo = outflow_cases(par) + sealed_cases(par)
    jobs = [(a, kind, kw, coef, q, sc, seed)
            for _, q, sc in todo for a, kind, kw in ARMS]
    stats = run_jobs(run_job, jobs, workers)
    rows, n = [], len(ARMS)
    w = max(len(a) for a, _, _ in ARMS)
    for i, (case, _, _) in enumerate(todo):
        keys = [(case, a) for a, _, _ in ARMS]
        rows += table(case, keys, stats[i * n:(i + 1) * n], w)
    return [['nominal'] + r for r in rows]


# %% ===== 2. SETTINGS ===================================================
def sweep(par, coef, seed, workers):
    """Run part 2 and return its CSV rows."""
    arm = {a: (kind, kw) for a, kind, kw in ARMS}[SWEEP_ARM]
    d = ctl.FeedforwardController()
    base = ', '.join(f'{k}={getattr(d, k):g}' for k, _ in SWEEP
                     if k != 'noise')
    say(f'\n=== 2. settings of "{SWEEP_ARM}", one at a time '
        f'(defaults: {base}, noise={par.noise / 1e3:g} kPa)')
    todo = [c for c in outflow_cases(par) if c[0] in SWEEP_CASES]
    jobs, keys = [], []
    for case, q, sc in todo:
        for arg, values in [('', (None,))] + SWEEP:
            for v in values:
                kw = dict(arm[1])
                p = q
                label = 'defaults'
                if arg == 'noise':
                    p = q.with_(noise=v)
                    label = f'noise={v / 1e3:g} kPa'
                elif arg:
                    kw[arg] = v
                    label = f'{arg}={v:g}'
                jobs.append((label, arm[0], kw, coef, p, sc, seed))
                keys.append((case, label))
    stats = run_jobs(run_job, jobs, workers)
    rows, n = [], len(jobs) // len(todo)
    for i, (case, _, _) in enumerate(todo):
        rows += table(case, keys[i * n:(i + 1) * n],
                      stats[i * n:(i + 1) * n], 18)
    return [['settings'] + r for r in rows]


# %% ===== 3. MONTE CARLO ================================================
def mc_job(job):
    """Run the compression and the delivery of one arm at one sample
    and return the requirement evaluation of robust.py with the
    delivery's feedforward metrics.

    job is (label, kind, controller arguments, coef, StudyParams,
    seed).
    """
    label, kind, kw, coef, par, seed = job
    by_scen, out = {}, {}
    false_ons = 0
    for scen in R.SCENARIOS:
        c = make_arm(label, kind, kw, coef)
        sc = R.make_scenario(scen, par)
        m = {k: math.nan for k in R.METRIC_NAMES}
        try:
            with np.errstate(invalid='raise', over='raise',
                             divide='raise'):
                r = sim.run(par, sc, c, seed=seed)
            m.update(r.metrics)
            m.update(R.extras(r, sc))
            s = ff_stats(r, c, par)
            false_ons += s['falseOns']
            if scen == 'deliver':
                out.update(s)
        except (FloatingPointError, OverflowError, ValueError,
                ZeroDivisionError):
            pass
        by_scen[scen] = m
    out['falseOns'] = false_ons
    ev = R.evaluate_all(by_scen)
    out.update({f'pass.{k}': v for k, v in ev.items()})
    return out


MC_COLS = (('dip', 'opening dip [MPa]', (50, 100)),
           ('closePeak', 'peak after the purge closing [MPa]',
            (50, 100)),
           ('margin', 'margin over saturation [MPa]', (50, 0)),
           ('tDet', 'longest wait for the feedforward [s]', (50, 100)),
           ('falseOns', 'switchings on with no flow', (50, 100)))


def monte_carlo(par, coef, n, seed, workers, out):
    """Run part 3 and write feedforward_mc.csv."""
    values = R.lhs(n, seed)
    arms = [a for a in ARMS if a[0] in MC_ARMS]
    w = max(len(a) for a, _, _ in arms)
    say(f'\n=== 3. Monte Carlo, n={n} Latin hypercube samples over '
        f'robust.UNCERTAIN; the feedforward is sized for the nominal '
        f'orifice ({coef:.3f} mm/s per sqrt(MPa))')
    rows = []
    for case, ratio, P_target, hose_before in MC_CASES:
        over = dict(ratio=ratio, hose_before=hose_before,
                    tol=R.THRESH['inband'] * 1e6)       # [MPa -> Pa]
        if P_target is not None:
            over['P_target'] = P_target
        jobs = [(a, kind, kw, coef, par.with_(**over).with_(**v),
                 seed * 1000 + i)
                for a, kind, kw in arms for i, v in enumerate(values)]
        t0 = time.time()
        res = run_jobs(mc_job, jobs, workers)
        say(f'\n--- {case} ({time.time() - t0:.0f} s)')
        say(f'{"":{w}s} {"pass %":>7s}  ' + '  '.join(
            f'{k:>14s}' for k, _, _ in MC_COLS))
        for j, (a, _, _) in enumerate(arms):
            block = res[j * n:(j + 1) * n]
            rate = 100.0 * np.mean([m['pass.all'] for m in block])
            cells = []
            for key, _, pcts in MC_COLS:
                x = np.array([m.get(key, math.nan) for m in block],
                             float)
                x = x[np.isfinite(x)]
                cells.append(' / '.join(
                    f'{v:.2f}' for v in np.percentile(x, pcts))
                    if x.size else '-')
            say(f'{a:{w}s} {rate:7.0f}  '
                + '  '.join(f'{c:>14s}' for c in cells))
            for i, m in enumerate(block):
                rows.append([case, a, i] + [values[i][k]
                                            for k in R.NAMES]
                            + [m.get(k, math.nan) for k, _, _ in COLS]
                            + [m['pass.all']])
    say('columns: ' + '; '.join(
        f'{k}: {d}, p{p[0]} / ' + ('min' if p[1] == 0 else 'max')
        for k, d, p in MC_COLS))
    with open(out / 'feedforward_mc.csv', 'w', newline='') as fh:
        wr = csv.writer(fh)
        wr.writerow(['case', 'arm', 'sample'] + R.NAMES
                    + [k for k, _, _ in COLS] + ['pass'])
        wr.writerows(rows)


# %% ===== PLOT ==========================================================
def plot_traces(path, par, coef, seed):
    """Plot the pressure about the first opening and about the purge
    closing of the 1:1 delivery for the arms of PLOT_ARMS, each on the
    time axis of its own valve event."""
    hb = par.with_(hose_before=True)
    sc = scenarios.deliver(hb)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for a, kind, kw in ARMS:
        if a not in PLOT_ARMS:
            continue
        r = sim.run(hb, sc, make_arm(a, kind, kw, coef), seed=seed)
        for ax, t0 in zip(axes, (r.events[-3], r.events[-2])):
            w = (r.t >= t0 - 0.2) & (r.t < t0 + 2.0)
            # [Pa -> MPa]
            ax.plot(r.t[w] - t0, r.P[w] / 1e6, label=a, lw=1.2)
    for ax, title in zip(axes, ('first opening', 'purge closing')):
        ax.axhline(hb.P_target / 1e6, color='0.6', lw=0.8, ls=':')
        ax.set_title(f'1:1 at {hb.P_target / 1e6:g} MPa, {title}')
        ax.set_xlabel('time from the valve event (s)')
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('chamber pressure (MPa)')
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# %% ===== MAIN ==========================================================
def main():
    ap = studyutil.parser(__doc__.split('\n\n')[1])
    ap.add_argument('--mc', type=int, default=0, metavar='N',
                    help='Monte Carlo samples of part 3 (default 0: '
                         'skipped)')
    args = ap.parse_args()
    par, _, out = studyutil.setup(args)
    coef = configured_coef()

    rows = nominal(par, coef, args.seed, args.workers)
    rows += sweep(par, coef, args.seed, args.workers)
    if args.mc > 0:
        monte_carlo(par, coef, args.mc, args.seed, args.workers, out)
    (out / 'feedforward_summary.txt').write_text('\n'.join(LOG) + '\n')
    with open(out / 'feedforward.csv', 'w', newline='') as fh:
        wr = csv.writer(fh)
        wr.writerow(['part', 'case', 'row'] + [k for k, _, _ in COLS])
        wr.writerows(rows)
    plot_traces(out / 'feedforward_traces.png', par, coef, args.seed)
    print(f'\nwrote {out}/feedforward_summary.txt, feedforward.csv, '
          f'feedforward_traces.png'
          + (', feedforward_mc.csv' if args.mc > 0 else ''))


if __name__ == '__main__':
    main()
