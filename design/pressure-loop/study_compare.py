#!/usr/bin/env python3
"""Velocity-command versus torque-command pressure loops, and the slave's own
controller, over the compress, stiffstep and discharge scenarios.

Prints a metrics table per scenario and writes <scenario>.csv and
<scenario>.png in --out. --sweeps also runs the gain sweeps that led to the
legacy controller set, plus sweeps of the slave controller's gains.

    python3 study_compare.py [--sweeps] [--set key=value ...] [--out DIR] [--legacy]
"""

import math

import controllers as ctl
import scenarios
import studyutil
from metrics import loop_cost
from runner import Case, run_cases

SCENARIOS = ('compress', 'stiffstep', 'discharge')


def controller_set():
    return list(ctl.legacy_set().values()) + [ctl.SlaveController('slave (GVL)')]


def table(title, header, rows):
    print(f'\n=== {title}')
    print(header)
    for row in rows:
        print(row)


def sweeps(p, legacy_sensor, workers, seed):
    L, A_N = ctl.LegacyController, p.A * 1e6

    def go(sc, cs):
        s = scenarios.make(sc, p)
        return run_cases([Case(p, s, c, legacy_sensor, seed) for c in cs], workers, traces=False)

    print()
    gains = [0.1, 0.25, 0.5, 1, 2, 4]
    rs = go('stiffstep', [L('', 'V', k, 0, 0) for k in gains])
    table('sweep V-P Kp on stiffstep (mm/s per MPa)', f'{"Kp":>8} {"settle s":>9} {"oversh":>9} {"ripple":>9} {"cost":>9}',
          [f'{k:8.2f} {r["tSettle"]:9.2f} {r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}' for k, r in zip(gains, rs)])

    grid = [(kp, ki, kd) for kp in (0.25, 0.5, 1) for ki in (0, 0.1, 0.5, 2) for kd in (0, 0.02, 0.05)]
    rs = go('discharge', [L('', 'V', *g) for g in grid])
    table('sweep V-PI/PID on discharge (Kp, Ki, Kd)',
          f'{"Kp":>6} {"Ki":>6} {"Kd":>6} {"peak|e|":>9} {"oversh":>9} {"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {g[2]:6.3f} {r["peakErr"]:9.3f} {r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])
    b = min(zip(grid, rs), key=lambda gr: gr[1].cost)[0]
    print(f'best V on discharge: Kp {b[0]:.2f} Ki {b[1]:.2f} Kd {b[2]:.3f}')

    grid = [(kp, ki, kd) for kp in (0, 0.05, 0.2, 0.5) for ki in (0, 0.2, 1) for kd in (0, 30, 100, 300)]
    rs = go('stiffstep', [L('', 'T', kp * A_N, ki * A_N, kd) for kp, ki, kd in grid])
    table(f'sweep T-PID on stiffstep (gains as multiples of A = {A_N:.0f} N/MPa; Kd in N per MPa/s)',
          f'{"Kp/A":>6} {"Ki/A":>6} {"Kd":>6} {"settle s":>9} {"oversh":>9} {"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {g[2]:6.0f} {r["tSettle"]:9.2f} {r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])
    b = min(zip(grid, rs), key=lambda gr: gr[1].cost)[0]
    print(f'best T on stiffstep: Kp {b[0]:.2f}A Ki {b[1]:.2f}A Kd {b[2]:.0f}')

    grid = [(kp, ki) for kp in (0.25, 0.5, 1, 2) for ki in (0, 0.2, 0.5, 1)]
    rs = go('discharge', [ctl.SlaveController(Kp=kp, Ki=ki) for kp, ki in grid])
    table('sweep slave controller on discharge (G_cfgPressureKp, Ki)',
          f'{"Kp":>6} {"Ki":>6} {"peak|e|":>9} {"oversh":>9} {"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {r["peakErr"]:9.3f} {r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])

    grid = [(st, ac) for st in (20, 50, 100, 200) for ac in (20, 50, 100)]
    rs = go('compress', [ctl.SlaveController(StopStiffness=st, Accel=ac, Decel=ac) for st, ac in grid])
    table('sweep slave stop law on compress (G_cfgPressureStopStiffness MPa/mm, Accel = Decel mm/s^2)',
          f'{"stiff":>6} {"accel":>6} {"settle s":>9} {"oversh":>9} {"stroke mm":>10} {"cost":>9}',
          [f'{g[0]:6.0f} {g[1]:6.0f} {r["tSettle"]:9.2f} {r["overshoot"]:9.3f} {r["stroke_mm"]:10.1f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])


def main(argv=None):
    ap = studyutil.parser(__doc__.split('\n')[0], sweeps=True)
    args = ap.parse_args(argv)
    p, legacy_sensor, out = studyutil.setup(args)
    if args.sweeps:
        sweeps(p, legacy_sensor, args.workers, args.seed)

    cs = controller_set()
    width = max(20, max(len(c.name) for c in cs))
    cases = [Case(p, scenarios.make(sc, p), c, legacy_sensor, args.seed) for sc in SCENARIOS for c in cs]
    results = run_cases(cases, args.workers)
    for i, sc in enumerate(SCENARIOS):
        rs = results[i * len(cs):(i + 1) * len(cs)]
        s = scenarios.make(sc, p)
        print(f'\n=== {sc}  (T={s.T:g} s, event at {s.tEvent:g} s)')
        print(f'{"controller":<{width}} {"settle s":>9} {"oversh":>9} {"peak|e|":>9} {"rms e":>9} {"ripple":>9} {"cmd act":>12} {"stroke mm":>10}')
        for r in rs:
            m = r.metrics
            settle = 'nan' if math.isnan(m['tSettle']) else f'{m["tSettle"]:.2f}'
            print(f'{r.name:<{width}} {settle:>9} {m["overshoot"]:9.3f} {m["peakErr"]:9.3f} {m["rmsErr"]:9.4f} '
                  f'{m["ripple"]:9.4f} {m["uAct"]:12.4f} {m["stroke_mm"]:10.1f}')
        r = rs[-1]
        print(f'stiffness dP/dx: start {r.stiffness[0] / 1e9:.3g}, end {r.stiffness[-1] / 1e9:.3g} MPa/mm   ({r.name})')
        studyutil.write_traces(out / f'{sc}.csv', rs, p)
        studyutil.plot_traces(out / f'{sc}.png', sc, rs)
    print(f'\nwrote CSV and PNG to {out}')


if __name__ == '__main__':
    main()
