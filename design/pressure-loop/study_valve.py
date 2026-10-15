#!/usr/bin/env python3
"""Pressure dip when the discharge valve opens, versus valve opening time,
for velocity-command controllers, the torque feedforward reference and the
slave's own controller (alone and with flow feedforward added).

Prints the dip and recovery tables and writes valve_dip.csv / valve_dip.png
(dip versus opening time) and valve03.csv / valve03.png (traces for a 0.3 s
opening) in --out.

    python3 study_valve.py [--set key=value ...] [--out DIR] [--legacy]
"""

import numpy as np

import controllers as ctl
import scenarios
import studyutil
from runner import Case, run_cases

L = ctl.LegacyController
TOPENS = [0, 0.1, 0.3, 1, 3, 10]
TRACE_TOPEN = 0.3


def controller_set():
    return [L('V-PI.25-.2 a5', 'V', 0.25, 0.2, 0, False, 5e-3),
            L('V-PI.25-.2 a50', 'V', 0.25, 0.2, 0, False, 50e-3),
            L('V-PI.5-.2 a50', 'V', 0.5, 0.2, 0, False, 50e-3),
            L('V-PI1-.5 a50 tf20', 'V', 1.0, 0.5, 0, False, 50e-3, tauF=0.02),  # Kp 1 needs the shorter sensor filter
            L('V-PID.5-.2-.05 a50', 'V', 0.5, 0.2, 0.05, False, 50e-3),
            L('V-PI.5-.2 a50 +FF', 'V', 0.5, 0.2, 0, False, 50e-3, vff=True),   # velocity feedforward of the expected flow
            L('T-FF-Kd100', 'T', 0, 0, 100),
            ctl.SlaveController('slave (GVL)'),
            ctl.SlaveController('slave +flowFF', flow_ff=True)]


def main(argv=None):
    ap = studyutil.parser(__doc__.split('\n')[0])
    args = ap.parse_args(argv)
    p, legacy_sensor, out = studyutil.setup(args)
    cs = controller_set()
    names = [c.name for c in cs]
    width = max(22, max(map(len, names)))

    cases = []
    for c in cs:
        for to in TOPENS:
            q = p.with_(Topen=to, valveT=max(8.0, 1 + 1.5 * to + 3))
            cases.append(Case(q, scenarios.make('valve', q), c, legacy_sensor, args.seed))
    results = run_cases(cases, args.workers)

    dip = np.zeros((len(TOPENS), len(cs)))
    rec = np.zeros_like(dip)
    traces = [None] * len(cs)
    for ci in range(len(cs)):
        for ti, to in enumerate(TOPENS):
            r = results[ci * len(TOPENS) + ti]
            ie = r.t >= 1.0
            dip[ti, ci] = (r.target[0] - r.P[ie].min()) / 1e6
            bad = np.flatnonzero((np.abs(r.target - r.P) > p.tol) & ie)
            rec[ti, ci] = r.t[bad[-1]] - 1.0 if bad.size else 0.0
            if to == TRACE_TOPEN:
                traces[ci] = r

    print(f'\n=== valve opening: Qf = {p.Qf * 1e6:.1f} mL/s, dip (MPa) vs opening time (s)')
    print(f'{"controller":<{width}}' + ''.join(f'{to:9.1f}' for to in TOPENS))
    for ci, n in enumerate(names):
        print(f'{n:<{width}}' + ''.join(f'{d:9.3f}' for d in dip[:, ci]))
    print(f'\nrecovery to within {p.tol / 1e6:.1f} MPa (s after the valve starts opening)')
    for ci, n in enumerate(names):
        print(f'{n:<{width}}' + ''.join(f'{d:9.2f}' for d in rec[:, ci]))
    print('\nstroke used (mm), 0.3 s opening')
    print('  '.join(f'{r.name}: {r["stroke_mm"]:.2f}' for r in traces))

    np.savetxt(out / 'valve_dip.csv', np.column_stack([TOPENS, dip]), delimiter=',',
               header=','.join(['Topen'] + names), comments='', fmt='%.4g')
    studyutil.write_traces(out / 'valve03.csv', traces, p)
    plot(out, names, dip, traces)
    print(f'\nwrote CSV and PNG to {out}')


def plot(out, names, dip, traces):
    plt = studyutil.plt
    col = studyutil.colors(names)
    fig, ax = plt.subplots(figsize=(9, 5.5))
    x = [0.07 if to == 0 else to for to in TOPENS]
    for ci, n in enumerate(names):
        ax.plot(x, dip[:, ci], marker='o', ls=studyutil.style(n), color=col[n], label=n, lw=1.2, ms=4)
    ax.set_xscale('log')
    ax.set_xticks(x)
    ax.set_xticklabels(['step' if to == 0 else f'{to:g}' for to in TOPENS])
    ax.set_xlabel('valve opening time (s)')
    ax.set_ylabel('dip (MPa)')
    ax.set_title('Pressure dip vs valve opening time, 1 mL/s final flow, liquid-only chamber at 10 MPa')
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out / 'valve_dip.png', dpi=110)
    plt.close(fig)
    studyutil.plot_traces(out / 'valve03.png', 'valve opens over 0.3 s to 1 mL/s', traces, tmax=6)


if __name__ == '__main__':
    main()
