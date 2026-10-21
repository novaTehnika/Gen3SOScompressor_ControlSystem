#!/usr/bin/env python3
"""
study_valve.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script finds how far the chamber pressure dips when the discharge
valve opens, as a function of the valve opening time. It compares
velocity-command PI and PID controllers, the torque feedforward
reference, and the slave's own controller with and without an added flow
feedforward.

It prints the dip and the recovery time for each controller and opening
time, and writes valve_dip.csv and valve_dip.png (dip against opening
time) and valve03.csv and valve03.png (traces for a 0.3 s opening) to
the output directory.

Usage:
    python3 study_valve.py [--set key=value ...] [--out DIR] [--legacy]

FILE DEPENDENCY:
./
    controllers.py
    runner.py
    scenarios.py
    studyutil.py
    (and the model modules they import: params.py, plant.py,
    sensor.py, sim.py, slave.py, solver.py)
"""

import numpy as np

import controllers as ctl
import scenarios
import studyutil
from runner import Case, run_cases


# %% ===== STUDY VARIABLES ===============================================
TOPENS = [0, 0.1, 0.3, 1, 3, 10]    # [s] valve opening times; 0 = step
TRACE_TOPEN = 0.3                   # [s] opening time of the traces


def controller_set():
    """Return the controllers compared."""
    L = ctl.LegacyController
    return [
        L('V-PI.25-.2 a5', 'V', 0.25, 0.2, 0, False, 5e-3),
        L('V-PI.25-.2 a50', 'V', 0.25, 0.2, 0, False, 50e-3),
        L('V-PI.5-.2 a50', 'V', 0.5, 0.2, 0, False, 50e-3),
        # Kp = 1 is stable only with the shorter sensor filter.
        L('V-PI1-.5 a50 tf20', 'V', 1.0, 0.5, 0, False, 50e-3,
          tauF=0.02),
        L('V-PID.5-.2-.05 a50', 'V', 0.5, 0.2, 0.05, False, 50e-3),
        # Velocity feedforward of the expected outflow
        L('V-PI.5-.2 a50 +FF', 'V', 0.5, 0.2, 0, False, 50e-3,
          vff=True),
        L('T-FF-Kd100', 'T', 0, 0, 100),
        ctl.SlaveController('slave (GVL)'),
        ctl.SlaveController('slave +flowFF', flow_ff=True),
    ]


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    # Simulation parameters from the command line
    ap = studyutil.parser('Pressure dip versus discharge valve opening '
                          'time.')
    args = ap.parse_args(argv)
    par, legacy_sensor, out = studyutil.setup(args)
    cs = controller_set()
    names = [c.name for c in cs]
    width = max(22, max(map(len, names)))

    # Collect data. Each run lasts long enough for the valve to open and
    # the pressure to recover: at least 8 s, or 1 s before the event plus
    # 1.5 opening times plus 3 s.
    cases = []
    for c in cs:
        for t_open in TOPENS:
            q = par.with_(Topen=t_open,
                          valveT=max(8.0, 1 + 1.5 * t_open + 3))
            cases.append(Case(q, scenarios.make('valve', q), c,
                              legacy_sensor, args.seed))
    results = run_cases(cases, args.workers)

    # Post-process: the dip below the starting target and the time after
    # the valve starts to open at which the error last leaves the band
    dip = np.zeros((len(TOPENS), len(cs)))                  # [MPa]
    recovery = np.zeros_like(dip)                           # [s]
    traces = [None] * len(cs)
    for ci in range(len(cs)):
        for ti, t_open in enumerate(TOPENS):
            r = results[ci * len(TOPENS) + ti]
            after_open = r.t >= 1.0
            dip[ti, ci] = (r.target[0] - r.P[after_open].min()) / 1e6
            outside = np.flatnonzero((np.abs(r.target - r.P) > par.tol)
                                     & after_open)
            if outside.size:
                recovery[ti, ci] = r.t[outside[-1]] - 1.0
            if t_open == TRACE_TOPEN:
                traces[ci] = r

    # Report
    print(f'\n=== valve opening: Qf = {par.Qf * 1e6:.1f} mL/s, '
          f'dip (MPa) vs opening time (s)')
    print(f'{"controller":<{width}}'
          + ''.join(f'{t_open:9.1f}' for t_open in TOPENS))
    for ci, n in enumerate(names):
        print(f'{n:<{width}}' + ''.join(f'{d:9.3f}' for d in dip[:, ci]))
    print(f'\nrecovery to within {par.tol / 1e6:.1f} MPa '
          f'(s after the valve starts opening)')
    for ci, n in enumerate(names):
        print(f'{n:<{width}}'
              + ''.join(f'{d:9.2f}' for d in recovery[:, ci]))
    print('\nstroke used (mm), 0.3 s opening')
    print('  '.join(f'{r.name}: {r["stroke_mm"]:.2f}' for r in traces))

    # Save data and plots
    np.savetxt(out / 'valve_dip.csv', np.column_stack([TOPENS, dip]),
               delimiter=',', header=','.join(['Topen'] + names),
               comments='', fmt='%.4g')
    studyutil.write_traces(out / 'valve03.csv', traces, par)
    plot(out, names, dip, traces)
    print(f'\nwrote CSV and PNG to {out}')


# %% ===== PLOTS =========================================================
def plot(out, names, dip, traces):
    """Plot the dip against opening time, and the traces for the 0.3 s
    opening."""
    plt = studyutil.plt
    col = studyutil.colors(names)
    fig, ax = plt.subplots(figsize=(9, 5.5))
    # A step (0 s) is drawn at 0.07 s so it fits on the log axis.
    x = [0.07 if t_open == 0 else t_open for t_open in TOPENS]
    for ci, n in enumerate(names):
        ax.plot(x, dip[:, ci], marker='o', ls=studyutil.style(n),
                color=col[n], label=n, lw=1.2, ms=4)
    ax.set_xscale('log')
    ax.set_xticks(x)
    ax.set_xticklabels(['step' if t_open == 0 else f'{t_open:g}'
                        for t_open in TOPENS])
    ax.set_xlabel('valve opening time (s)')
    ax.set_ylabel('dip (MPa)')
    ax.set_title('Pressure dip vs valve opening time, 1 mL/s final flow, '
                 'liquid-only chamber at 10 MPa')
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out / 'valve_dip.png', dpi=110)
    plt.close(fig)
    studyutil.plot_traces(out / 'valve03.png',
                          'valve opens over 0.3 s to 1 mL/s', traces,
                          tmax=6)


if __name__ == '__main__':
    main()
