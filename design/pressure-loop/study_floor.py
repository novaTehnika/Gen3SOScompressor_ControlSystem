#!/usr/bin/env python3
"""
study_floor.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
11/02/2026

PURPOSE/DESCRIPTION:
This script tests stop-law floors that follow the outflow
(controllers.FloorPolicy, eqs. (46) to (48)) against the fixed
StopVelMin of the GVL. A fixed floor high enough to pass the outflow
removes the stop law near the target in every state: the compression
overshoots further, and when the purge shuts the distal valve under the
moving piston the pressure runs through the ceiling. A floor that is
low until outflow is detected keeps the stop law in a sealed chamber
and passes the outflow in a delivery. The forward stop of eq. (48)
also stops a forward command above the target while no outflow is
detected, since the stop law limits the speed in either direction.

The study has three parts:

  1. Nominal cases. Every arm runs the compression and the delivery for
     each case (hose procedure, charge, target, valve closing time) at
     nominal parameters. The table gives the compression overshoot and
     time to arrive, the opening dip, the peak after the purge closes
     the valve, the delivered volume before a violation, the mean and
     standard deviation of the error and the command during the
     delivery, the share of the delivery the forward stop holds the
     piston, and the floor's latch transitions with the detection and
     release delays.
  2. Policy sweep. One policy setting at a time is varied around the
     default of the switched floor with the forward stop, on the
     1:1 delivery for both procedures and with a slow valve closing.
  3. Monte Carlo (--mc N). The arms run the robustness study's Latin
     hypercube over the measured parameter ranges (robust.py) and give
     the pass rate of each requirement, with the percentiles of the
     closing peak, the delivery error spread and the false latches in
     the sealed hold after the compression.

It writes floor_nominal.csv, floor_sweep.csv, floor_mc.csv (with
--mc), floor_traces.png and floor_summary.txt to the output directory.

Usage:
    python3 study_floor.py [--set key=value ...] [--out DIR]
                           [--workers N] [--seed 1] [--mc 100]
                           [--mc-ratio 1] [--mc-hose-after]
                           [--no-sweep]

FILE DEPENDENCY:
./
    studyparams.py, controllers.py, robust.py
    scenarios.py, sim.py
    studyutil.py
    (and the modules they import, and models/)
"""

import csv
import math
import multiprocessing as mp
import os
import time

import numpy as np

import controllers as ctl
import robust as R
import sim
import studyutil


# %% ===== STUDY ARMS ====================================================
# Arms as robust.ControllerSpec, so the Monte Carlo can run them. The
# scheduled arms use the measured bounds of study_delivery.py.
SCHED = {'V_dead_cfg': 81e-6, 'K_cfg': 1.0e9, 'C_hose_cfg': 1.2e-13,
         'wc': 11.0}
SW_FS = {'mode': 'switched', 'forward_stop': True}
ARMS = [
    R.ControllerSpec('GVL (fixed 2)'),
    R.ControllerSpec('fixed 0.5', {'StopVelMin': 0.5}),
    R.ControllerSpec('switched', floor={'mode': 'switched'}),
    R.ControllerSpec('fixed 2 +fs',
                     floor={'mode': 'fixed', 'forward_stop': True}),
    R.ControllerSpec('switched +fs', floor=SW_FS),
    R.ControllerSpec('estimated +fs',
                     floor={'mode': 'estimated', 'forward_stop': True}),
    R.ControllerSpec('sched11', sched=SCHED),
    R.ControllerSpec('sched11 switched +fs', sched=SCHED, floor=SW_FS),
]
# Arms run by the Monte Carlo by default
MC_ARMS = ('GVL (fixed 2)', 'switched +fs', 'estimated +fs', 'sched11',
           'sched11 switched +fs')

# Nominal cases: (label, StudyParams overrides)
CASES = [
    ('1:1 hose before', dict(ratio=1.0, hose_before=True)),
    ('1:1 hose after', dict(ratio=1.0, hose_before=False)),
    ('2:1 hose before', dict(ratio=2.0, hose_before=True)),
    ('2.5:1 11 MPa hose before', dict(ratio=2.5, hose_before=True,
                                      P_target=11e6)),
    ('1:1 hose before, 1 s close', dict(ratio=1.0, hose_before=True,
                                        Tclose=1.0)),
]
TRACE_CASE = '1:1 hose before'

# Policy sweep around the default of the switched floor with the
# forward stop: (FloorPolicy argument, values)
SWEEP = [
    ('v_lo', (0.0, 0.25, 0.5, 1.0)),
    ('v_hi', (1.6, 2.0, 3.0)),
    ('window', (0.04, 0.1, 0.2, 0.4)),
    ('t_on', (0.0, 0.04, 0.2)),
    ('r_on', (0.02, 0.1, 0.5)),
    ('e_on', (0.0, 0.05, 0.2)),
    ('r_off', (0.25, 1.0, 4.0)),
    ('e_off', (-0.1, 0.0, 0.1)),
]
SWEEP_CASES = ('1:1 hose before', '1:1 hose after',
               '1:1 hose before, 1 s close')


# %% ===== REPORT LOG ====================================================
LOG = []


def say(text=''):
    """Print a line and keep it for the summary file."""
    print(text, flush=True)
    LOG.append(text)


# %% ===== ONE RUN =======================================================
def delivery_stats(r, c, par):
    """Return the floor and delivery metrics of delivery run r under
    controller c with StudyParams par.

      closePeak  [MPa] Highest pressure between the purge closing and
                 the reopening.
      eMean      [MPa] Mean error of the true pressure over the
      eStd       delivery stage (from t_recover after the reopening to
                 1 s before the end), and its standard deviation.
      uMean      [mm/s] Mean and standard deviation of the command over
      uStd       the same stage.
      stopFrac   [-] Share of that stage with the command at or below
                 zero.
      nTrans     [-] Latch transitions in the run.
      tDetect    [s] From the first opening of the distal valve to the
                 latch; NaN if it does not latch.
      tRelease   [s] From the purge closing to the release; NaN if it
                 does not release before the reopening.
    """
    ev = r.events
    out = {}
    if len(ev) < 3:
        # The run ended before the purge closed and the valve reopened.
        for k in ('closePeak', 'eMean', 'eStd', 'uMean', 'uStd',
                  'stopFrac'):
            out[k] = math.nan
        out.update(latch_stats(r, c))
        return out
    t_open, t_close, t_reopen = ev[-3], ev[-2], ev[-1]
    w = (r.t >= t_close) & (r.t < t_reopen)
    out['closePeak'] = float(np.max(r.P[w])) / 1e6 if w.any() \
        else math.nan
    late = (r.t > t_reopen + par.t_recover) & (r.t < r.t[-1] - 1.0)
    if late.any():
        e = (r.target[late] - r.P[late]) / 1e6                  # [MPa]
        u = r.u[late] * 1e3                         # [m/s -> mm/s]
        out['eMean'] = float(np.mean(e))
        out['eStd'] = float(np.std(e))
        out['uMean'] = float(np.mean(u))
        out['uStd'] = float(np.std(u))
        out['stopFrac'] = float(np.mean(u <= 1e-9))
    else:
        for k in ('eMean', 'eStd', 'uMean', 'uStd', 'stopFrac'):
            out[k] = math.nan
    out.update(latch_stats(r, c, t_open, t_close, t_reopen))
    return out


def latch_stats(r, c, t_open=math.nan, t_close=math.nan,
                t_reopen=math.nan):
    """Return the latch transitions, detection and release delays
    [s] of run r under controller c (see delivery_stats)."""
    out = {'nTrans': 0, 'tDetect': math.nan, 'tRelease': math.nan}
    if c.floor is None:
        return out
    L = np.array(c.floor.trace_latched[:len(r.t)], dtype=int)
    i_on = np.flatnonzero(np.diff(L) == 1) + 1
    i_off = np.flatnonzero(np.diff(L) == -1) + 1
    out['nTrans'] = int(len(i_on) + len(i_off))
    t_on = r.t[i_on]
    t_off = r.t[i_off]
    on = t_on[t_on >= t_open]
    if math.isfinite(t_open):
        # Latched already at the opening counts as no delay.
        if L[np.searchsorted(r.t, t_open)]:
            out['tDetect'] = 0.0
        elif on.size:
            out['tDetect'] = float(on[0] - t_open)
    off = t_off[(t_off >= t_close) & (t_off < t_reopen)]
    if off.size:
        out['tRelease'] = float(off[0] - t_close)
    return out


def compress_stats(r, c, par):
    """Return the false latches of compression run r: the latches set
    during the approach (holdLatch counts those after the pressure
    first arrives within the band, in the sealed hold, where there is
    no outflow; compLatch those before) and whether the floor is
    latched at the end."""
    out = {'holdLatch': 0, 'compLatch': 0, 'latchedEnd': 0}
    if c.floor is None:
        return out
    L = np.array(c.floor.trace_latched[:len(r.t)], dtype=int)
    arrived = np.flatnonzero(np.abs(r.target - r.P) < par.tol)
    rise = np.flatnonzero(np.diff(L) == 1) + 1    # scans that latch
    i0 = arrived[0] if arrived.size else len(L)
    out['holdLatch'] = int(np.count_nonzero(rise >= i0))
    out['compLatch'] = int(np.count_nonzero(rise < i0)) + int(L[0])
    out['latchedEnd'] = int(L[-1])
    return out


def run_pair(job):
    """Run the compression and the delivery of one arm and case.

    job is (ControllerSpec, StudyParams, seed). Returns the metrics of
    both runs as one flat dict, keys prefixed 'c.' and 'd.', and the
    requirement evaluation of robust.py.
    """
    spec, par, seed = job
    out = {}
    by_scen = {}
    for scen in ('compress', 'deliver'):
        c = R.make_controller(spec)
        s = R.make_scenario(scen, par)
        try:
            with np.errstate(invalid='raise', over='raise',
                             divide='raise'):
                r = sim.run(par, s, c, seed=seed)
            m = {k: math.nan for k in R.METRIC_NAMES}
            m.update(r.metrics)
            m.update(R.extras(r, s))
            if scen == 'deliver':
                m.update(delivery_stats(r, c, par))
            else:
                m.update(compress_stats(r, c, par))
        except (FloatingPointError, OverflowError, ValueError,
                ZeroDivisionError):
            m = {k: math.nan for k in R.METRIC_NAMES}
        by_scen[scen] = m
        out.update({f'{scen[0]}.{k}': v for k, v in m.items()})
    ev = R.evaluate_all(by_scen)
    out.update({f'pass.{k}': v for k, v in ev.items()})
    return out


def run_jobs(jobs, workers):
    """Return run_pair() of every job, in order, in a pool of workers
    processes (one per CPU by default)."""
    workers = min(workers or os.cpu_count() or 1, len(jobs))
    if workers <= 1:
        return [run_pair(j) for j in jobs]
    with mp.get_context('spawn').Pool(workers) as pool:
        return pool.map(run_pair, jobs,
                        chunksize=max(1, len(jobs) // (4 * workers)))


def case_params(par, overrides):
    """Return StudyParams par with the band of the robustness study and
    the case's overrides."""
    return par.with_(tol=R.THRESH['inband'] * 1e6, **overrides)


# %% ===== 1. NOMINAL CASES ==============================================
NOMINAL_COLS = [
    ('c.overshoot', 'os', '{:6.2f}'), ('c.t_arrive', 'tArr', '{:6.1f}'),
    ('c.compLatch', 'cL', '{:3d}'), ('c.holdLatch', 'hL', '{:3d}'),
    ('d.dipMax', 'dip', '{:6.2f}'),
    ('d.closePeak', 'close', '{:6.2f}'), ('d.V_ok', 'V_ok', '{:5.0f}'),
    ('d.eMean', 'eMean', '{:6.2f}'), ('d.eStd', 'eStd', '{:6.2f}'),
    ('d.uStd', 'uStd', '{:6.2f}'), ('d.stopFrac', 'stop', '{:5.2f}'),
    ('d.nTrans', 'tr', '{:3d}'), ('d.tDetect', 'tDet', '{:6.2f}'),
    ('d.tRelease', 'tRel', '{:6.2f}'), ('pass.all', 'pass', '{:>5}'),
]


def fmt(f, v):
    """Format value v with format f, or '-' if it is NaN."""
    if isinstance(v, float) and math.isnan(v):
        return '-'.rjust(len(f.format(0)) if 'd' not in f else 3)
    if 'd' in f:
        return f.format(int(v))
    return f.format(v)


def nominal(par, seed, workers, out):
    """Run part 1 and write floor_nominal.csv. Returns the results by
    (case label, arm name)."""
    jobs, keys = [], []
    for label, ov in CASES:
        q = case_params(par, ov)
        for spec in ARMS:
            jobs.append((spec, q, seed))
            keys.append((label, spec.name))
    res = dict(zip(keys, run_jobs(jobs, workers)))
    width = max(len(s.name) for s in ARMS) + 2
    say('\n=== 1. nominal cases')
    say('os, tArr: compression overshoot [MPa] and arrival [s]; cL, '
        'hL: latches set during the compression and in the sealed '
        'hold after it; dip, close: opening dip '
        'and peak after the purge closing [MPa]; V_ok [mL]; eMean, '
        'eStd [MPa] and uStd [mm/s] over the delivery stage; stop: '
        'share of it held by the forward stop; tr: latch transitions; '
        'tDet, tRel [s]: detection after the opening and release '
        'after the closing; pass: every requirement of robust.py')
    for label, _ in CASES:
        say(f'\n--- {label}')
        say(f'{"arm":<{width}}' + ' '.join(
            h.rjust(len(fmt(f, 0))) for _, h, f in NOMINAL_COLS))
        for spec in ARMS:
            m = res[(label, spec.name)]
            say(f'{spec.name:<{width}}' + ' '.join(
                fmt(f, m[k]) for k, _, f in NOMINAL_COLS))
    with open(out / 'floor_nominal.csv', 'w', newline='') as f:
        w = csv.writer(f)
        cols = sorted(next(iter(res.values())))
        w.writerow(['case', 'arm'] + cols)
        for (label, name), m in res.items():
            w.writerow([label, name] + [m[k] for k in cols])
    return res


# %% ===== 2. POLICY SWEEP ===============================================
SWEEP_COLS = [('c.overshoot', 'os', '{:6.2f}'),
              ('c.compLatch', 'cL', '{:3d}'),
              ('c.holdLatch', 'hL', '{:3d}'),
              ('d.closePeak', 'close', '{:6.2f}'),
              ('d.eMean', 'eMean', '{:6.2f}'),
              ('d.eStd', 'eStd', '{:6.2f}'),
              ('d.stopFrac', 'stop', '{:5.2f}'),
              ('d.nTrans', 'tr', '{:3d}'),
              ('d.tRelease', 'tRel', '{:6.2f}'),
              ('pass.all', 'pass', '{:>5}')]


def sweep(par, seed, workers, out):
    """Run part 2 and write floor_sweep.csv."""
    cases = dict(CASES)
    jobs, keys = [], []
    for arg, values in SWEEP:
        for v in values:
            spec = R.ControllerSpec(f'{arg}={v:g}',
                                    floor={**SW_FS, arg: v})
            for label in SWEEP_CASES:
                jobs.append((spec, case_params(par, cases[label]), seed))
                keys.append((arg, v, label))
    res = dict(zip(keys, run_jobs(jobs, workers)))
    say('\n=== 2. policy sweep, switched floor with the forward stop '
        '(defaults: ' + ', '.join(
            f'{k}={v:g}' for k, v in default_policy().items()) + ')')
    for label in SWEEP_CASES:
        say(f'\n--- {label}')
        say(f'{"setting":<14}' + ' '.join(
            h.rjust(len(fmt(f, 0))) for _, h, f in SWEEP_COLS))
        for arg, values in SWEEP:
            for v in values:
                m = res[(arg, v, label)]
                say(f'{arg + "=" + format(v, "g"):<14}' + ' '.join(
                    fmt(f, m[k]) for k, _, f in SWEEP_COLS))
    with open(out / 'floor_sweep.csv', 'w', newline='') as f:
        w = csv.writer(f)
        cols = sorted(next(iter(res.values())))
        w.writerow(['setting', 'value', 'case'] + cols)
        for (arg, v, label), m in res.items():
            w.writerow([arg, v, label] + [m[k] for k in cols])


def default_policy():
    """Return the FloorPolicy defaults of the swept arguments."""
    p = ctl.FloorPolicy()
    return {arg: getattr(p, arg) for arg, _ in SWEEP}


# %% ===== 3. MONTE CARLO ================================================
MC_COLS = (('d.closePeak', 'peak after the purge closing [MPa]', True),
           ('d.eStd', 'delivery error std [MPa]', True),
           ('c.compLatch', 'latches during the compression', True),
           ('c.holdLatch', 'latches in the sealed hold', True),
           ('c.overshoot', 'compression overshoot [MPa]', True))


def monte_carlo(par, n, seed, workers, out, ratio, hose_before):
    """Run part 3 and write floor_mc.csv."""
    values = R.lhs(n, seed)
    specs = [s for s in ARMS if s.name in MC_ARMS]
    jobs, keys = [], []
    for spec in specs:
        for i, v in enumerate(values):
            q = case_params(par, dict(ratio=ratio,
                                      hose_before=hose_before)).with_(
                **v)
            jobs.append((spec, q, seed * 1000 + i))
            keys.append((spec.name, i))
    t0 = time.time()
    res = dict(zip(keys, run_jobs(jobs, workers)))
    proc = 'before' if hose_before else 'after mixing'
    say(f'\n=== 3. Monte Carlo, n={n} Latin hypercube samples over '
        f'robust.UNCERTAIN, {ratio:g}:1, hose opened {proc} '
        f'({time.time() - t0:.0f} s)')
    reqs = list(R.REQUIREMENTS) + ['all']
    w = max(len(s.name) for s in specs) + 2
    say('pass rate (%) per requirement')
    say(f'{"requirement":<20}' + ''.join(f'{s.name:>{w}}'
                                         for s in specs))
    for rq in reqs:
        rates = [100 * np.mean([res[(s.name, i)]['pass.' + rq]
                                for i in range(n)]) for s in specs]
        say(f'{rq:<20}' + ''.join(f'{x:>{w}.0f}' for x in rates))
    say('\nmetric percentiles p50 / p90 / max')
    for key, desc, _ in MC_COLS:
        say(desc)
        for s in specs:
            x = np.array([res[(s.name, i)][key] for i in range(n)],
                         float)
            x = x[np.isfinite(x)]
            if x.size:
                p = np.percentile(x, [50, 90, 100])
                say(f'  {s.name:<{w}}'
                    + ' / '.join(f'{v:.3g}' for v in p))
    with open(out / 'floor_mc.csv', 'w', newline='') as f:
        wr = csv.writer(f)
        cols = sorted(next(iter(res.values())))
        wr.writerow(['arm', 'sample'] + R.NAMES + cols)
        for (name, i), m in res.items():
            wr.writerow([name, i] + [values[i][k] for k in R.NAMES]
                        + [m[k] for k in cols])


# %% ===== PLOTS =========================================================
def plot_traces(par, seed, out):
    """Plot the 1:1 delivery, hose opened before the compression, around
    the purge closing and the reopening: pressure, command and floor of
    each arm."""
    plt = studyutil.plt
    q = case_params(par, dict(CASES)[TRACE_CASE])
    s = R.make_scenario('deliver', q)
    runs = []
    for spec in ARMS:
        c = R.make_controller(spec)
        r = sim.run(q, s, c, seed=seed)
        if c.floor is not None:
            fl = np.array(c.floor.trace_floor[:len(r.t)])
        else:
            fl = np.full(len(r.t), c.fb.StopVelMin)
        runs.append((spec.name, r, fl))
    col = studyutil.colors([n for n, _, _ in runs])
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    t_close = runs[0][1].events[-2]
    lo, hi = t_close - 3.0, t_close + 16.0                      # [s]
    for name, r, fl in runs:
        w = (r.t >= lo) & (r.t <= hi)
        ls = '--' if name.startswith('GVL') else '-'
        ax[0].plot(r.t[w], r.P[w] / 1e6, color=col[name], ls=ls,
                   lw=1.0, label=name)
        ax[1].plot(r.t[w], r.u[w] * 1e3, color=col[name], ls=ls,
                   lw=0.9)
        ax[2].plot(r.t[w], fl[w], color=col[name], ls=ls, lw=0.9)
    ax[0].axhline(q.P_ceiling / 1e6, color='k', ls=':', lw=0.8)
    ax[0].set_ylabel('P (MPa)')
    ax[1].set_ylabel('velocity cmd (mm/s)')
    ax[2].set_ylabel('floor (mm/s)')
    ax[2].set_xlabel('t (s)')
    ax[0].set_title(f'delivery {TRACE_CASE}: purge closing at '
                    f'{t_close:.1f} s and the reopening')
    ax[0].legend(loc='center left', bbox_to_anchor=(1.01, 0.5),
                 fontsize=8, frameon=False)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / 'floor_traces.png', dpi=110)
    plt.close(fig)


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    ap = studyutil.parser('Stop-law floors that follow the outflow '
                          'against the fixed StopVelMin.')
    ap.add_argument('--mc', type=int, default=0,
                    help='Monte Carlo samples (0 skips part 3)')
    ap.add_argument('--mc-ratio', type=float, default=1.0,
                    help='oxygen charge of the Monte Carlo')
    ap.add_argument('--mc-hose-after', action='store_true',
                    help='Monte Carlo with the hose opened after mixing')
    ap.add_argument('--no-sweep', action='store_true',
                    help='skip part 2')
    args = ap.parse_args(argv)
    par, _, out = studyutil.setup(args)
    t0 = time.time()
    nominal(par, args.seed, args.workers, out)
    if not args.no_sweep:
        sweep(par, args.seed, args.workers, out)
    if args.mc > 0:
        monte_carlo(par, args.mc, args.seed, args.workers, out,
                    args.mc_ratio, not args.mc_hose_after)
    plot_traces(par, args.seed, out)
    say(f'\ntotal {time.time() - t0:.0f} s; wrote files to {out}')
    (out / 'floor_summary.txt').write_text('\n'.join(LOG) + '\n')


if __name__ == '__main__':
    main()
