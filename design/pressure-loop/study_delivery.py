#!/usr/bin/env python3
"""
study_delivery.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/26/2026

PURPOSE/DESCRIPTION:
This script runs the delivery, from the hose opening through the purge
to the empty chamber, for a set of controllers over a range of oxygen
charges and for the two hose procedures, and reports what each can
deliver without degassing the solution, exceeding the ceiling or losing
control. The pressure drops when the hose and the distal valve open,
the overshoot when the purge closes the valve against the moving
piston, and the loss of control as the emptying chamber stiffens all
limit the deliverable volume, and the saturation pressure rises with
the charge, so the result is a trade-off between the oxygen ratio and
the volume that can be delivered. The procedures are the present one
(the hose opened after mixing, at pressure) and the proposed one (the
hose opened before the compression). The controllers compared are the
slave as configured, the slave with the earlier stop-law floor, and the
scheduled slave controller at several settings.

For each procedure, controller and ratio it prints the deepest dip at
a valve event, the lowest and highest pressures, the margin above
saturation, the volume delivered before the first violation (V_ok) and
the solution delivered in all. It writes delivery.csv,
delivery_front.png (V_ok against the ratio, one panel per procedure)
and delivery_traces.png (the 1:1 delivery, proposed procedure) to the
output directory.

Usage:
    python3 study_delivery.py [--set key=value ...] [--out DIR]
                              [--ratios 1,1.5,2,2.5,3]

FILE DEPENDENCY:
./
    studyparams.py, controllers.py
    runner.py
    scenarios.py
    studyutil.py
    (and the modules they import: sim.py, and models/)
"""

import numpy as np

import controllers as ctl
import scenarios
import studyutil
from runner import Case, run_cases


# %% ===== STUDY VARIABLES ===============================================
RATIOS = (1.0, 1.5, 2.0, 2.5, 3.0)      # [-] STP oxygen to saline
TRACE_RATIO = 1.0                       # [-] ratio of the plotted traces
# Hose procedures: StudyParams.hose_before for each
PROCEDURES = {'hose opened after mixing': False,
              'hose opened before compression': True}


def controller_set():
    """Return the controllers compared.

    The slave arms are the GVL (StopVelMin 2 mm/s, which passes the
    1.4 mm/s the measured outflow needs) and the earlier StopVelMin of
    0.5 mm/s as a reference. The scheduled arms are
    ScheduledSlaveController at three crossovers with the measured
    bounds (K 1.0 GPa, dead volume 81 mL, hose compliance
    0.12 mL/MPa). The flow-feedforward arm is a bound; the feedforward
    is not implementable on the machine.
    """
    S = ctl.SlaveController
    Sch = ctl.ScheduledSlaveController
    sched = dict(V_dead_cfg=81e-6, K_cfg=1.0e9, C_hose_cfg=1.2e-13)
    return [
        S('slave (GVL)'),
        S('slave vmin0.5 (earlier)', StopVelMin=0.5),
        Sch('sched wc6', wc=6.0, **sched),
        Sch('sched wc11', wc=11.0, **sched),
        Sch('sched wc11 ki1 v7', wc=11.0, ki_ratio=1.0, VelMax=7.0,
            **sched),
        Sch('sched wc16', wc=16.0, **sched),
        S('slave +flowFF (bound)', flow_ff=True),
    ]


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    ap = studyutil.parser('Deliverable volume against oxygen charge for '
                          'a set of controllers.')
    ap.add_argument('--ratios', default=','.join(map(str, RATIOS)),
                    help='comma list of STP oxygen to saline ratios')
    args = ap.parse_args(argv)
    par, legacy_sensor, out = studyutil.setup(args)
    ratios = [float(x) for x in args.ratios.split(',')]
    cs = controller_set()
    names = [c.name for c in cs]
    width = max(34, max(map(len, names)))

    # Collect data: every procedure, controller and ratio
    cases = []
    for before in PROCEDURES.values():
        for c in cs:
            for ratio in ratios:
                q = par.with_(ratio=ratio, hose_before=before)
                cases.append(Case(q, scenarios.make('deliver', q), c,
                                  legacy_sensor, args.seed))
    results = run_cases(cases, args.workers)
    n_per = len(cs) * len(ratios)           # results per procedure

    def result(pi, ci, ri):
        return results[pi * n_per + ci * len(ratios) + ri]

    # Report
    cols = ('dipMax', 'P_min', 'degasMargin', 'P_max', 'V_ok',
            'V_solution')
    heads = ('dip MPa', 'P_min MPa', 'margin MPa', 'P_max MPa',
             'V_ok mL', 'V_sol mL')
    rows = []
    for pi, proc in enumerate(PROCEDURES):
        for ri, ratio in enumerate(ratios):
            q = par.with_(ratio=ratio)
            P_sat = q.P_sat(min(q.n_charge(ratio),
                                q.kH * (q.P_target + q.Patm) * q.Vl0),
                            q.Vl0)
            print(f'\n=== deliver, {proc}, {ratio:g}:1 (P_sat '
                  f'{P_sat / 1e6:.2f} MPa, Q '
                  f'{q.Q_orifice(q.P_target) * 1e6:.2f} mL/s at '
                  f'{q.P_target / 1e6:g} MPa, Cd {q.Cd})')
            print(f'{"controller":<{width}}'
                  + ''.join(f'{h:>11}' for h in heads))
            for ci, c in enumerate(cs):
                m = result(pi, ci, ri).metrics
                print(f'{c.name:<{width}}'
                      + ''.join(f'{m[k]:11.2f}' for k in cols))
                rows.append([proc, ratio, c.name] + [m[k] for k in cols])

    # Save
    with open(out / 'delivery.csv', 'w') as f:
        f.write('procedure,ratio,controller,' + ','.join(cols) + '\n')
        for row in rows:
            f.write(f'{row[0]},{row[1]:g},{row[2]},'
                    + ','.join(f'{x:.5g}' for x in row[3:]) + '\n')
    plot_front(out, names, ratios, result)
    if TRACE_RATIO in ratios:
        pi = list(PROCEDURES.values()).index(True)
        traces = [result(pi, ci, ratios.index(TRACE_RATIO))
                  for ci in range(len(cs))]
        studyutil.plot_traces(out / 'delivery_traces.png',
                              f'delivery, {TRACE_RATIO:g}:1, hose opened '
                              f'before compression', traces)
    print(f'\nwrote CSV and PNG to {out}')


# %% ===== PLOTS =========================================================
def plot_front(out, names, ratios, result):
    """Plot the volume deliverable without a violation against the
    oxygen ratio for each controller, one panel per hose procedure.
    result(pi, ci, ri) returns the run of procedure pi, controller ci
    and ratio ri."""
    plt = studyutil.plt
    col = studyutil.colors(names)
    fig, axes = plt.subplots(1, len(PROCEDURES), figsize=(12, 5.5),
                             sharey=True)
    for pi, (proc, ax) in enumerate(zip(PROCEDURES, axes)):
        for ci, n in enumerate(names):
            y = [result(pi, ci, ri)['V_ok'] for ri in range(len(ratios))]
            ax.plot(ratios, y, marker='o', ls=studyutil.style(n),
                    color=col[n], label=n, lw=1.2, ms=4)
        ax.set_xlabel('oxygen charge (STP volume per saline volume)')
        ax.set_title(proc)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('saline delivered before a violation (mL)')
    axes[0].legend(fontsize=8, frameon=False)
    fig.suptitle('Deliverable volume against oxygen charge')
    fig.tight_layout()
    fig.savefig(out / 'delivery_front.png', dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
