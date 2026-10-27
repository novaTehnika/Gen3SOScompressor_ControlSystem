#!/usr/bin/env python3
"""
study_delivery.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/26/2026

PURPOSE/DESCRIPTION:
This script runs the delivery, from the valve opening to the empty
chamber, for a set of controllers over a range of oxygen charges, and
reports what each controller can deliver without degassing the solution
or losing control. The pressure dip when the valve opens and the loss
of control as the emptying chamber stiffens both limit the deliverable
volume, and the saturation pressure rises with the charge, so the
result is a trade-off between the oxygen ratio and the volume that can
be delivered. The controllers compared are the slave as configured, the
slave with the stop law relaxed, and the scheduled slave controller at
several settings.

For each controller and ratio it prints the opening dip, the lowest
pressure, the margin above saturation, the volume delivered before the
first violation (V_ok) and the volume delivered in all. It writes
delivery.csv, delivery_front.png (V_ok against the ratio) and
delivery_traces.png (the 1:1 delivery) to the output directory.

Usage:
    python3 study_delivery.py [--set key=value ...] [--out DIR]
                              [--ratios 1,1.5,2,2.5,3]

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
RATIOS = (1.0, 1.5, 2.0, 2.5, 3.0)      # [-] STP oxygen to saline
TRACE_RATIO = 1.0                       # [-] ratio of the plotted traces


def controller_set():
    """Return the controllers compared.

    The slave arms are the GVL and the GVL with StopVelMin 2 mm/s (the
    measured outflow needs 1.4 mm/s). The scheduled arms are
    ScheduledSlaveController at three crossovers with the measured
    bounds (K 1.0 GPa, dead volume 45 mL). The flow-feedforward arm is
    a bound; the feedforward is not implementable on the machine.
    """
    S = ctl.SlaveController
    Sch = ctl.ScheduledSlaveController
    vmin2 = dict(StopVelMin=2.0)
    sched = dict(V_dead_cfg=45e-6, K_cfg=1.0e9)
    return [
        S('slave (GVL)'),
        S('slave vmin2', **vmin2),
        Sch('sched wc6 vmin2', wc=6.0, **sched, **vmin2),
        Sch('sched wc11 vmin2', wc=11.0, **sched, **vmin2),
        Sch('sched wc11 ki1 vmin2 v7', wc=11.0, ki_ratio=1.0, VelMax=7.0,
            **sched, **vmin2),
        Sch('sched wc16 vmin2', wc=16.0, **sched, **vmin2),
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

    # Collect data: every controller at every ratio
    cases = []
    for c in cs:
        for ratio in ratios:
            q = par.with_(ratio=ratio)
            cases.append(Case(q, scenarios.make('deliver', q), c,
                              legacy_sensor, args.seed))
    results = run_cases(cases, args.workers)

    # Report
    cols = ('dip', 'P_min', 'degasMargin', 'V_ok', 'V_delivered', 'P_max')
    heads = ('dip MPa', 'P_min MPa', 'margin MPa', 'V_ok mL', 'V_del mL',
             'P_max MPa')
    rows = []
    for ratio in ratios:
        q = par.with_(ratio=ratio)
        P_sat = q.P_sat(min(q.n_charge(ratio),
                            q.kH * (q.P_target + q.Patm) * q.Vl0),
                        q.Vl0)
        print(f'\n=== deliver, {ratio:g}:1 (P_sat {P_sat / 1e6:.2f} MPa, '
              f'Q {q.Q_orifice(q.P_target) * 1e6:.2f} mL/s at '
              f'{q.P_target / 1e6:g} MPa, Cd {q.Cd})')
        print(f'{"controller":<{width}}'
              + ''.join(f'{h:>11}' for h in heads))
        for ci, c in enumerate(cs):
            r = results[ci * len(ratios) + ratios.index(ratio)]
            m = r.metrics
            print(f'{c.name:<{width}}'
                  + ''.join(f'{m[k]:11.2f}' for k in cols))
            rows.append([ratio, c.name] + [m[k] for k in cols])

    # Save
    with open(out / 'delivery.csv', 'w') as f:
        f.write('ratio,controller,' + ','.join(cols) + '\n')
        for row in rows:
            f.write(f'{row[0]:g},{row[1]},'
                    + ','.join(f'{x:.5g}' for x in row[2:]) + '\n')
    plot_front(out, names, ratios, results)
    traces = [results[ci * len(ratios) + ratios.index(TRACE_RATIO)]
              for ci in range(len(cs))] if TRACE_RATIO in ratios else []
    if traces:
        studyutil.plot_traces(out / 'delivery_traces.png',
                              f'delivery, {TRACE_RATIO:g}:1', traces)
    print(f'\nwrote CSV and PNG to {out}')


# %% ===== PLOTS =========================================================
def plot_front(out, names, ratios, results):
    """Plot the volume deliverable without a violation against the
    oxygen ratio for each controller."""
    plt = studyutil.plt
    col = studyutil.colors(names)
    fig, ax = plt.subplots(figsize=(8, 5.5))
    for ci, n in enumerate(names):
        y = [results[ci * len(ratios) + ri]['V_ok']
             for ri in range(len(ratios))]
        ax.plot(ratios, y, marker='o', ls=studyutil.style(n),
                color=col[n], label=n, lw=1.2, ms=4)
    ax.set_xlabel('oxygen charge (STP volume per saline volume)')
    ax.set_ylabel('saline delivered before a violation (mL)')
    ax.set_title('Deliverable volume against oxygen charge')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(out / 'delivery_front.png', dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
