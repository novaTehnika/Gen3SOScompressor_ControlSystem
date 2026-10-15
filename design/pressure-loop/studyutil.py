"""Shared command line, trace CSV and plot helpers for the study scripts."""

import argparse
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from params import Params  # noqa: E402

HERE = Path(__file__).resolve().parent
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#17becf', '#7f7f7f', '#bcbd22']


def parser(description, sweeps=False):
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument('--out', default=str(HERE / 'out'), help='output directory (default: out/ here)')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='override a Params field or a G_cfg* slave setting; repeatable')
    ap.add_argument('--legacy', action='store_true',
                    help='fixed liquid volume, one slew limit, and the median plus first-order sensor chain in pressure units')
    ap.add_argument('--workers', type=int, default=None, help='parallel processes (default: one per CPU)')
    ap.add_argument('--seed', type=int, default=1, help='sensor noise seed')
    if sweeps:
        ap.add_argument('--sweeps', action='store_true', help='also run the gain sweeps')
    return ap


def setup(args):
    """(Params, legacy_sensor flag, output directory) from parsed arguments."""
    p = Params.from_overrides(args.set)
    if args.legacy:
        p = p.legacy()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    return p, args.legacy, out


def write_traces(path, results, p, step=5):
    """CSV of target, pressure (MPa) and command (mm/s, or MPa of force/A for
    'T') per result, every `step`th scan."""
    names = [r.name for r in results]
    r0 = results[0]
    cols = [r0.t, r0.target / 1e6] + [r.P / 1e6 for r in results] + [r.u_plot() for r in results]
    head = ','.join(['t', 'target'] + [f'P_{n}' for n in names] + [f'u_{n}' for n in names])
    np.savetxt(path, np.column_stack(cols)[::step], delimiter=',', header=head, comments='', fmt='%.5g')


def colors(names):
    return {n: COLORS[i % len(COLORS)] for i, n in enumerate(names)}


def style(name):
    """dashed for the slave's own controller so it stays visible over coincident lines"""
    return '--' if name.startswith('slave') else '-'


def plot_traces(path, title, results, tmax=None):
    """Pressure, velocity command and force command panels for one scenario."""
    col = colors([r.name for r in results])
    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    ax[0].plot(results[0].t, results[0].target / 1e6, 'k--', lw=1, label='target')
    for r in results:
        ax[0].plot(r.t, r.P / 1e6, color=col[r.name], ls=style(r.name), lw=1.1, label=r.name)
        ax_u = ax[1] if r.mode == 'V' else ax[2]
        ax_u.plot(r.t, r.u_plot(), color=col[r.name], ls=style(r.name), lw=1.0, label=r.name)
    ax[0].set_ylabel('P (MPa)')
    ax[1].set_ylabel('velocity cmd (mm/s)')
    ax[2].set_ylabel('force cmd / A (MPa)')
    ax[2].set_xlabel('t (s)')
    if tmax:
        ax[2].set_xlim(0, tmax)
    ax[0].set_title(title)
    ax[0].legend(loc='center left', bbox_to_anchor=(1.01, 0.5), fontsize=8, frameon=False)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
