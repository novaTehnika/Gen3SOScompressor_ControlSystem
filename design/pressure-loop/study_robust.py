#!/usr/bin/env python3
"""
study_robust.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script tests how robust the slave's pressure loop is to the plant
parameters that are not yet measured. For each slave configuration (an
"arm" of the study) it runs the compress, stiffstep, discharge and valve
scenarios and checks the requirements defined in robust.py. The study
has three parts:

  1. One-at-a-time sensitivity (tornado). Each uncertain parameter is
     set to its low and then its high value with the others at nominal.
     This part uses the first arm only.
  2. Corners. Every combination of the low and high values of the k
     most influential parameters from part 1 is run.
  3. Monte Carlo. A Latin hypercube sample over all the uncertain
     parameters gives the pass rate of each requirement, the percentiles
     of each metric, the parameters that drive the failures, and the
     worst cases.

Every arm sees the same samples, and sample i uses noise seed
seed*1000 + i in every arm, so the arms are compared on identical
conditions. The script writes robust_summary.txt, tornado.png,
mc_<arm>.csv, mc_scatter_<arm>.png and compare.png to the output
directory.

Usage:
    python3 study_robust.py [--n 200] [--seed 1] [--workers N]
                            [--out DIR] [--configs GVL,GVL+flowFF,...]
                            [--corner-n 4] [--cfg Kp=0.25 ...]
                            [--flow-ff] [--ff-scale 0.7]

--cfg and --flow-ff run a single arm named "custom" instead of the
built-in arms. --cfg takes the SlaveController short names (Kp, Ki,
Decel, StopStiffness, AnalogFilterTimeConst, ...) in GVL units.

FILE DEPENDENCY:
./
    robust.py
    (and the model modules it imports: controllers.py, params.py,
    plant.py, scenarios.py, sensor.py, sim.py, slave.py, solver.py)
"""

import argparse
import csv
import itertools
import math
import os
import time
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

import matplotlib
matplotlib.use('Agg')           # write image files; no display needed
import matplotlib.pyplot as plt  # noqa: E402

import robust as R  # noqa: E402
from robust import (ControllerSpec, UNCERTAIN, NAMES,  # noqa: E402
                    SCENARIOS, THRESH)

HERE = Path(__file__).resolve().parent
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
          '#8c564b']


# %% ===== STUDY ARMS ====================================================
# Each candidate arm changes one thing from the GVL, so the effect of
# each change can be attributed.
CONFIGS = {
    'GVL': ControllerSpec('GVL'),
    'GVL+flowFF': ControllerSpec('GVL+flowFF', flow_ff=True),
    'f20+vmin1': ControllerSpec(
        'f20+vmin1',
        {'AnalogFilterTimeConst': 0.02, 'StopVelMin': 1.0}),
    'f20+vmin1+flowFF': ControllerSpec(
        'f20+vmin1+flowFF',
        {'AnalogFilterTimeConst': 0.02, 'StopVelMin': 1.0}, True),
}


# %% ===== REPORT LOG ====================================================
# Every line printed by say() is also kept here and written to
# robust_summary.txt at the end.
LOG = []


def say(text=''):
    """Print a line and keep it for the summary file."""
    print(text, flush=True)
    LOG.append(text)


# %% ===== BATCH HELPERS =================================================
def run_values(spec, values, seeds, pool, scens=SCENARIOS):
    """Run arm spec at each parameter dict in values with the matching
    noise seed, and return a list of {scenario: metrics}."""
    pts = [(v, s, spec, seed) for v, seed in zip(values, seeds)
           for s in scens]
    out = R.run_points(pts, workers=1 if pool is None else None,
                       pool=pool)
    k = len(scens)
    return [{s: out[i * k + j] for j, s in enumerate(scens)}
            for i in range(len(values))]


def flat(by_scen):
    """Flatten {scenario: metrics} to {'scenario.metric': value}."""
    return {f'{s}.{k}': v for s, m in by_scen.items()
            for k, v in m.items() if k != 'diverged'}


def severity(by_scen):
    """Return the largest ratio of metric to limit over the key metrics.

    A value above 1 means a requirement fails. A non-finite metric
    counts as infinitely severe.
    """
    worst = 0.0
    for (s, k), lim in R.KEY_METRICS.items():
        v = by_scen[s][k]
        worst = max(worst, math.inf if not math.isfinite(v) else v / lim)
    return worst


def fmt_vals(values):
    """Format a parameter dict in display units."""
    return ' '.join(f'{u.name}={values[u.name] * u.scale:.3g}{u.unit}'
                    for u in UNCERTAIN)


# %% ===== 1. ONE-AT-A-TIME SENSITIVITY (TORNADO) ========================
def _swing(b, d, lim, k=None):
    """Return the range of a metric over (nominal b, low, high), divided
    by its limit.

    d is the (low, high) pair, or a dict of pairs indexed by k.
    """
    if k is not None:
        d = d[k]
    vals = [b, *d]
    if not all(math.isfinite(v) for v in vals):
        return math.inf
    return (max(vals) - min(vals)) / lim


def tornado(spec, pool, seed, out):
    """Run the one-at-a-time sensitivity for arm spec, print the ranking
    and plot it. Returns (parameter names by influence, score by
    name)."""
    nom = R.nominal()
    values = [nom]
    for u in UNCERTAIN:
        values += [{**nom, u.name: u.lo}, {**nom, u.name: u.hi}]
    res = run_values(spec, values, [seed] * len(values), pool)
    base = res[0]
    keys = list(R.KEY_METRICS)

    # (low, high) value of each key metric, per parameter
    rows = {}
    for j, u in enumerate(UNCERTAIN):
        lo, hi = res[1 + 2 * j], res[2 + 2 * j]
        rows[u.name] = {k: (lo[k[0]][k[1]], hi[k[0]][k[1]]) for k in keys}

    # Score: the largest swing over the key metrics
    score = {}
    for name, d in rows.items():
        score[name] = max(_swing(base[k[0]][k[1]], d[k],
                                 R.KEY_METRICS[k]) for k in keys)
    order = sorted(NAMES, key=lambda n: -score[n])

    say(f'\n=== 1. one-at-a-time sensitivity, {spec.name}')
    say('nominal: ' + ' '.join(f'{k[0]}.{k[1]}={base[k[0]][k[1]]:.3g}'
                               for k in keys))
    say('swing = max over key metrics of (range over low, nominal, '
        'high) / requirement limit')
    say(f'{"param":<8}{"range":>16}{"swing":>8}  largest effect')
    for n in order:
        u = R.BY_NAME[n]
        k = max(keys, key=lambda k: _swing(base[k[0]][k[1]], rows[n],
                                           R.KEY_METRICS[k], k))
        lo, hi = rows[n][k]
        rng = f'{u.lo * u.scale:g}-{u.hi * u.scale:g} {u.unit}'
        say(f'{n:<8}{rng:>16}{score[n]:8.2f}  {k[0]}.{k[1]} '
            f'{base[k[0]][k[1]]:.3g} -> low {lo:.3g}, high {hi:.3g}')
    plot_tornado(out / 'tornado.png', spec.name, base, rows, order, keys)
    return order, score


def plot_tornado(path, name, base, rows, order, keys):
    """Plot the change of each key metric from nominal, one panel per
    metric, with the parameters sorted by effect."""
    n = len(keys)
    cols = 4
    nrows = (n + cols - 1) // cols
    fig, axs = plt.subplots(nrows, cols, figsize=(17, 3.2 * nrows))
    for ax, k in zip(axs.flat, keys):
        b = base[k[0]][k[1]]

        def effect(nm):
            lo, hi = rows[nm][k]
            if not all(map(math.isfinite, (lo, hi))):
                return 1e9
            return max(abs(lo - b), abs(hi - b))
        names = sorted(order, key=effect)
        for i, nm in enumerate(names):
            lo, hi = rows[nm][k]
            ax.barh(i, lo - b, color='#1f77b4', height=0.6)
            ax.barh(i, hi - b, color='#d62728', height=0.6, left=0)
        ax.set_yticks(range(len(names)))
        ax.set_yticklabels(names, fontsize=8)
        ax.axvline(0, color='k', lw=0.8)
        lim = R.KEY_METRICS[k]
        ax.axvline(lim - b, color='k', ls='--', lw=0.8)
        ax.set_title(f'{k[0]} {k[1]} (nominal {b:.3g}, limit {lim:g})',
                     fontsize=9)
        ax.grid(alpha=0.3)
    for ax in list(axs.flat)[n:]:
        ax.axis('off')
    fig.suptitle(f'{name}: change from nominal, parameter at low (blue) '
                 f'/ high (red); dashed = requirement limit', fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


# %% ===== 2. CORNERS ====================================================
def corners(specs, params, pool, seed):
    """Run every low/high combination of params, the others at nominal,
    for each arm, and print the pass count and the worst corners."""
    nom = R.nominal()
    combos = list(itertools.product((0, 1), repeat=len(params)))
    values = []
    for bits in combos:
        corner = {p: (R.BY_NAME[p].lo, R.BY_NAME[p].hi)[b]
                  for p, b in zip(params, bits)}
        values.append({**nom, **corner})
    say(f'\n=== 2. corners: 2^{len(params)} combinations of '
        f'{", ".join(params)} (others nominal)')
    for spec in specs:
        res = run_values(spec, values, [seed] * len(values), pool)
        ev = [R.evaluate_all(r) for r in res]
        npass = sum(e['all'] for e in ev)
        say(f'{spec.name}: {npass}/{len(values)} corners pass every '
            f'requirement')

        # Failures counted by requirement
        bad = {}
        for e in ev:
            for k, v in e.items():
                if not v and k != 'all':
                    bad[k] = bad.get(k, 0) + 1
        if bad:
            say('   corner failures by requirement: '
                + ', '.join(f'{k} {v}' for k, v in sorted(bad.items())))

        # The two most severe corners
        worst2 = sorted(range(len(values)),
                        key=lambda i: -severity(res[i]))[:2]
        for i in worst2:
            tag = ' '.join(f'{p}={"lo" if b == 0 else "hi"}'
                           for p, b in zip(params, combos[i]))
            fails = [k for k, v in ev[i].items() if not v and k != 'all']
            say(f'   worst corner: {tag}; severity '
                f'{severity(res[i]):.2f}; fails {fails or "none"}')


# %% ===== 3. MONTE CARLO ================================================
def monte_carlo(spec, values, seeds, pool, out):
    """Run arm spec on the Latin hypercube samples, write its CSV, and
    return its results."""
    t = time.time()
    res = run_values(spec, values, seeds, pool)
    ev = [R.evaluate_all(r) for r in res]
    say(f'  {spec.name}: {len(values)} samples x {len(SCENARIOS)} '
        f'scenarios in {time.time() - t:.0f} s')
    write_csv(out / f'mc_{_slug(spec.name)}.csv', values, res, ev)
    return {'spec': spec, 'values': values, 'res': res, 'ev': ev}


def _slug(name):
    """Return name with characters unsafe in file names replaced."""
    return ''.join(c if c.isalnum() or c in '.-' else '_' for c in name)


def write_csv(path, values, res, ev):
    """Write one row per sample: the parameter values, every metric, and
    whether every requirement passed."""
    cols = list(flat(res[0]))
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(NAMES + cols + ['pass'])
        for v, r, e in zip(values, res, ev):
            fr = flat(r)
            w.writerow([f'{v[n]:.6g}' for n in NAMES]
                       + [f'{fr[c]:.5g}' for c in cols] + [int(e['all'])])


def pass_rate(arm, req):
    """Return the fraction of samples in arm that pass requirement
    req."""
    return float(np.mean([e[req] for e in arm['ev']]))


def report_mc(arms, n):
    """Print the pass rates, dip pass rates at other limits, metric
    percentiles and settling times of every arm."""
    reqs = list(R.REQUIREMENTS) + ['all']
    names = [a['spec'].name for a in arms]
    w = max(len(x) for x in names) + 2
    # 95 % binomial half-width at a 90 % pass rate, in percentage points:
    # 1.96 sqrt(p (1 - p) / n) * 100
    half_width = 196 * math.sqrt(.09 / n)
    say(f'\n=== 3. Monte Carlo, n={n} Latin hypercube samples (95% '
        f'binomial half-width at 90% = {half_width:.1f} points)')
    say('pass rate (%) per requirement')
    say(f'{"requirement":<22}' + ''.join(f'{x:>{w}}' for x in names))
    for r in reqs:
        say(f'{r:<22}'
            + ''.join(f'{100 * pass_rate(a, r):>{w}.0f}' for a in arms))
    say('  ' + '; '.join(f'{r}: {R.describe(r)}'
                         for r in R.REQUIREMENTS))

    say('\ndip pass rate (%) at other limits, discharge / valve')
    for lim in R.DIP_LEVELS:
        row = f'  dip <= {lim:g} MPa'.ljust(22)
        for a in arms:
            d = np.mean([r['discharge']['dip'] <= lim for r in a['res']])
            v = np.mean([r['valve']['dip'] <= lim for r in a['res']])
            row += f'{100 * d:>{w - 4}.0f} /{100 * v:<3.0f}'
        say(row)

    say('\nmetric distributions (p50 / p90 / p99 / max); limit in '
        'brackets')
    for (s, k), lim in R.KEY_METRICS.items():
        say(f'{s}.{k} [{lim:g}]')
        for a in arms:
            x = np.array([r[s][k] for r in a['res']], float)
            fin = x[np.isfinite(x)]
            n_bad = len(x) - len(fin)
            if len(fin):
                p = np.percentile(fin, [50, 90, 99, 100])
            else:
                p = [math.nan] * 4
            note = f'   ({n_bad} non-finite)' if n_bad else ''
            say(f'  {a["spec"].name:<{w}}'
                + ' / '.join(f'{v:.3g}' for v in p) + note)

    tset = {}
    for a in arms:
        x = np.array([r['compress']['tSettle'] for r in a['res']], float)
        tset[a['spec'].name] = (np.nanpercentile(x, [50, 90]),
                                int(np.isnan(x).sum()))
    say('compress time to settle in band (s) p50 / p90 (never settled): '
        + '; '.join(f'{k} {v[0][0]:.1f} / {v[0][1]:.1f} ({v[1]})'
                    for k, v in tset.items()))


def drivers(arm):
    """Print the failure drivers of arm: for each failing requirement,
    the two parameters whose lowest and highest thirds differ most in
    failure rate."""
    spec, values, ev = arm['spec'], arm['values'], arm['ev']
    n = len(values)
    say(f'\nfailure drivers, {spec.name} (fail rate in lowest third / '
        f'highest third of each parameter; two largest differences)')
    reqs = [r for r in list(R.REQUIREMENTS) + ['all']
            if not all(e[r] for e in ev)]
    if not reqs:
        say('  no failures')
    for r in reqs:
        fail = np.array([not e[r] for e in ev])
        scored = []
        for u in UNCERTAIN:
            x = np.array([v[u.name] for v in values])
            if u.log:
                x = np.log(x)
            lo = x <= np.percentile(x, 100 / 3)
            hi = x >= np.percentile(x, 200 / 3)
            fl, fh = fail[lo].mean(), fail[hi].mean()
            scored.append((abs(fh - fl), u.name, fl, fh))
        scored.sort(reverse=True)
        txt = '; '.join(f'{nm} {100 * fl:.0f}% / {100 * fh:.0f}%'
                        for d, nm, fl, fh in scored[:2])
        say(f'  {r:<20} fails {int(fail.sum()):>3}/{n}: {txt}')


def worst(arm, k=3):
    """Print the k most severe samples of arm."""
    spec, values, res, ev = (arm['spec'], arm['values'], arm['res'],
                             arm['ev'])
    say(f'\nworst cases, {spec.name} (severity = largest metric/limit '
        f'among key metrics)')
    order = sorted(range(len(values)), key=lambda i: -severity(res[i]))
    for i in order[:k]:
        fails = [r for r, v in ev[i].items() if not v and r != 'all']
        sv = severity(res[i])
        sv_text = 'inf' if math.isinf(sv) else f'{sv:.2f}'
        say(f'  #{i} severity {sv_text}: {fmt_vals(values[i])}')
        over = [f'{s}.{m} {res[i][s][m]:.3g}'
                for (s, m), lim in R.KEY_METRICS.items()
                if not res[i][s][m] <= lim]
        say(f'     fails {fails or "none"}; metrics over limit: '
            f'{", ".join(over) or "none"}')


def plot_scatter(path, arm):
    """Plot six key metrics, each against the parameter with which it
    has the largest Spearman rank correlation."""
    spec, values, res = arm['spec'], arm['values'], arm['res']
    keys = [('compress', 'overshoot'), ('stiffstep', 'overshoot'),
            ('discharge', 'dip'), ('valve', 'dip'),
            ('compress', 'stroke_mm'), ('valve', 'maxErrLate')]
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    for ax, k in zip(axs.flat, keys):
        y = np.array([r[k[0]][k[1]] for r in res], float)
        ok = np.isfinite(y)

        # Parameter with the strongest rank correlation
        best, brho = None, 0
        for u in UNCERTAIN:
            x = np.array([v[u.name] for v in values])
            if ok.sum() < 3 or np.ptp(y[ok]) == 0:
                continue
            rho = spearmanr(x[ok], y[ok])[0]
            if abs(rho) > abs(brho):
                best, brho = u, rho
        if best is None:
            continue

        x = np.array([v[best.name] for v in values]) * best.scale
        lim = R.KEY_METRICS[k]
        bad = ~(y <= lim)
        # Non-finite failures are drawn at the largest finite value.
        y_cap = np.nanmax(y[ok]) if ok.any() else 0
        ax.scatter(x[~bad], y[~bad], s=12, c='#1f77b4', label='pass')
        ax.scatter(x[bad], np.where(np.isfinite(y[bad]), y[bad], y_cap),
                   s=14, c='#d62728', label='fail')
        ax.axhline(lim, color='k', ls='--', lw=0.8)
        if best.log:
            ax.set_xscale('log')
        ax.set_xlabel(f'{best.name} ({best.unit})   Spearman {brho:+.2f}')
        ax.set_ylabel(f'{k[0]} {k[1]}')
        ax.grid(alpha=0.3)
    axs.flat[0].legend(fontsize=8)
    fig.suptitle(f'{spec.name}: key metric vs its most correlated '
                 f'parameter (dashed = limit)')
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_compare(path, arms):
    """Plot the pass rate of each requirement for every arm."""
    reqs = list(R.REQUIREMENTS) + ['all']
    fig, ax = plt.subplots(figsize=(13, 4.5))
    wd = 0.8 / len(arms)
    for i, a in enumerate(arms):
        ax.bar(np.arange(len(reqs)) + i * wd,
               [100 * pass_rate(a, r) for r in reqs], wd,
               label=a['spec'].name, color=COLORS[i % len(COLORS)])
    ax.set_xticks(np.arange(len(reqs)) + 0.4 - wd / 2)
    ax.set_xticklabels(reqs, rotation=30, ha='right', fontsize=8)
    ax.set_ylabel('pass rate (%)')
    ax.legend(fontsize=8, ncol=len(arms))
    ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


# %% ===== MAIN STUDY ====================================================
def parse_cfg(items):
    """Parse --cfg strings of the form key=value into a dict."""
    kw = {}
    for it in items:
        k, sep, v = it.partition('=')
        if not sep:
            raise SystemExit(f'bad --cfg {it!r}: expected key=value')
        kw[k.strip()] = float(v)
    return kw


def main(argv=None):
    # Command line
    ap = argparse.ArgumentParser(
        description='Robustness of the slave pressure loop over the '
                    'uncertain plant parameters.')
    ap.add_argument('--n', type=int, default=200,
                    help='Monte Carlo samples')
    ap.add_argument('--seed', type=int, default=1,
                    help='sampling and noise seed')
    ap.add_argument('--workers', type=int, default=None)
    ap.add_argument('--out', default=str(HERE / 'out'))
    ap.add_argument('--cfg', action='append', default=[],
                    metavar='KEY=VALUE',
                    help='slave controller override (SlaveController '
                         'names, GVL units); repeatable; runs one arm '
                         '"custom"')
    ap.add_argument('--flow-ff', action='store_true',
                    help='add flow feedforward to the custom arm')
    ap.add_argument('--ff-scale', type=float, default=1.0,
                    help='feedforward flow / actual flow, custom arm')
    ap.add_argument('--configs', default=None,
                    help='comma list of built-in arms (default all)')
    ap.add_argument('--corner-n', type=int, default=4,
                    help='parameters in the corner study')
    ap.add_argument('--no-tornado', action='store_true')
    args = ap.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Study arms
    if args.cfg or args.flow_ff:
        specs = [ControllerSpec('custom', parse_cfg(args.cfg),
                                args.flow_ff, args.ff_scale)]
    else:
        names = args.configs.split(',') if args.configs else list(CONFIGS)
        specs = [CONFIGS[n] for n in names]

    # Parameter ranges and limits
    t0 = time.time()
    say('parameter ranges (one table, robust.UNCERTAIN)')
    for u in UNCERTAIN:
        sampling = 'log ' if u.log else 'lin '
        say(f'  {u.name:<8}{u.lo * u.scale:>8g} - '
            f'{u.hi * u.scale:<8g}{u.unit:<6}{sampling} {u.why}')
    say('thresholds: ' + ', '.join(f'{k}={v:g}'
                                   for k, v in THRESH.items()))
    nw = args.workers or os.cpu_count() or 1
    pool = R.make_pool(nw) if nw > 1 else None

    # Collect data: tornado, corners and Monte Carlo
    try:
        order = []
        if not args.no_tornado:
            order, _ = tornado(specs[0], pool, args.seed, out)
        corner_params = (order or NAMES)[:args.corner_n]
        corners(specs, corner_params, pool, args.seed)

        values = R.lhs(args.n, args.seed)
        seeds = [args.seed * 1000 + i for i in range(args.n)]
        say('\nMonte Carlo runs')
        arms = [monte_carlo(s, values, seeds, pool, out) for s in specs]
    finally:
        if pool is not None:
            pool.close()
            pool.join()

    # Post-process and save
    report_mc(arms, args.n)
    for a in arms:
        drivers(a)
        worst(a)
        plot_scatter(out / f'mc_scatter_{_slug(a["spec"].name)}.png', a)
    plot_compare(out / 'compare.png', arms)
    say(f'\ntotal {time.time() - t0:.0f} s; wrote files to {out}')
    (out / 'robust_summary.txt').write_text('\n'.join(LOG) + '\n')


if __name__ == '__main__':
    main()
