#!/usr/bin/env python3
"""
study_compare.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script compares pressure loops that command the piston velocity
with loops that command the piston force, together with the slave's own
controller, over the compress, deliver and stiffstep scenarios.

For each scenario it prints a table of metrics and writes the traces to
<scenario>.csv and a plot to <scenario>.png in the output directory.
With --sweeps it first runs the gain sweeps from which the legacy
controller set was chosen, and sweeps of the slave controller's gains
and stop law.

Usage:
    python3 study_compare.py [--sweeps] [--set key=value ...]
                             [--out DIR] [--legacy]

FILE DEPENDENCY:
./
    studyparams.py, controllers.py
    metrics.py
    runner.py
    scenarios.py
    studyutil.py
    (and the modules they import: sim.py, and models/)
"""

import math

import controllers as ctl
import scenarios
import studyutil
from runner import Case, run_cases


# %% ===== STUDY DEFINITION ==============================================
SCENARIOS = ('compress', 'deliver', 'stiffstep')


def controller_set():
    """Return the controllers compared: the legacy set and the slave's
    controller with the GVL configuration."""
    return (list(ctl.legacy_set().values())
            + [ctl.SlaveController('slave (GVL)')])


def table(title, header, rows):
    """Print a titled table."""
    print(f'\n=== {title}')
    print(header)
    for row in rows:
        print(row)


# %% ===== GAIN SWEEPS ===================================================
def sweeps(par, legacy_sensor, workers, seed):
    """Run the gain sweeps and print one table per sweep."""
    L = ctl.LegacyController
    A_N = par.A * 1e6           # [N/MPa] force per MPa on the piston

    def run_set(scenario_name, controllers):
        s = scenarios.make(scenario_name, par)
        cases = [Case(par, s, c, legacy_sensor, seed) for c in controllers]
        return run_cases(cases, workers, traces=False)

    print()

    # Velocity-command P controller on the stiffness step
    gains = [0.1, 0.25, 0.5, 1, 2, 4]               # [(mm/s)/MPa]
    rs = run_set('stiffstep', [L('', 'V', k, 0, 0) for k in gains])
    table('sweep V-P Kp on stiffstep (mm/s per MPa)',
          f'{"Kp":>8} {"settle s":>9} {"oversh":>9} {"ripple":>9} '
          f'{"cost":>9}',
          [f'{k:8.2f} {r["tSettle"]:9.2f} {r["overshoot"]:9.3f} '
           f'{r["ripple"]:9.4f} {r.cost:9.3f}'
           for k, r in zip(gains, rs)])

    # Velocity-command PI and PID controllers on the delivery
    grid = [(kp, ki, kd) for kp in (0.25, 0.5, 1)
            for ki in (0, 0.1, 0.5, 2) for kd in (0, 0.02, 0.05)]
    rs = run_set('deliver', [L('', 'V', *g) for g in grid])
    table('sweep V-PI/PID on deliver (Kp, Ki, Kd)',
          f'{"Kp":>6} {"Ki":>6} {"Kd":>6} {"peak|e|":>9} {"oversh":>9} '
          f'{"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {g[2]:6.3f} {r["peakErr"]:9.3f} '
           f'{r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])
    b = min(zip(grid, rs), key=lambda gr: gr[1].cost)[0]
    print(f'best V on deliver: Kp {b[0]:.2f} Ki {b[1]:.2f} '
          f'Kd {b[2]:.3f}')

    # Force-command PID controllers on the stiffness step. Kp and Ki are
    # given as multiples of A, so Kp = 1 A gives 1 MPa of force per MPa
    # of error.
    grid = [(kp, ki, kd) for kp in (0, 0.05, 0.2, 0.5)
            for ki in (0, 0.2, 1) for kd in (0, 30, 100, 300)]
    rs = run_set('stiffstep', [L('', 'T', kp * A_N, ki * A_N, kd)
                               for kp, ki, kd in grid])
    table(f'sweep T-PID on stiffstep (gains as multiples of '
          f'A = {A_N:.0f} N/MPa; Kd in N per MPa/s)',
          f'{"Kp/A":>6} {"Ki/A":>6} {"Kd":>6} {"settle s":>9} '
          f'{"oversh":>9} {"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {g[2]:6.0f} {r["tSettle"]:9.2f} '
           f'{r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])
    b = min(zip(grid, rs), key=lambda gr: gr[1].cost)[0]
    print(f'best T on stiffstep: Kp {b[0]:.2f}A Ki {b[1]:.2f}A '
          f'Kd {b[2]:.0f}')

    # Slave controller gains on the delivery
    grid = [(kp, ki) for kp in (0.25, 0.5, 1, 2)
            for ki in (0, 0.2, 0.5, 1)]
    rs = run_set('deliver', [ctl.SlaveController(Kp=kp, Ki=ki)
                             for kp, ki in grid])
    table('sweep slave controller on deliver (G_cfgPressureKp, Ki)',
          f'{"Kp":>6} {"Ki":>6} {"peak|e|":>9} {"oversh":>9} '
          f'{"ripple":>9} {"cost":>9}',
          [f'{g[0]:6.2f} {g[1]:6.2f} {r["peakErr"]:9.3f} '
           f'{r["overshoot"]:9.3f} {r["ripple"]:9.4f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])

    # Slave stop law on the gas-rich compression
    grid = [(st, ac) for st in (20, 50, 100, 200) for ac in (20, 50, 100)]
    rs = run_set('compress', [ctl.SlaveController(StopStiffness=st,
                                                  Accel=ac, Decel=ac)
                              for st, ac in grid])
    table('sweep slave stop law on compress (G_cfgPressureStopStiffness '
          'MPa/mm, Accel = Decel mm/s^2)',
          f'{"stiff":>6} {"accel":>6} {"settle s":>9} {"oversh":>9} '
          f'{"stroke mm":>10} {"cost":>9}',
          [f'{g[0]:6.0f} {g[1]:6.0f} {r["tSettle"]:9.2f} '
           f'{r["overshoot"]:9.3f} {r["stroke_mm"]:10.1f} {r.cost:9.3f}'
           for g, r in zip(grid, rs)])


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    # Simulation parameters from the command line
    ap = studyutil.parser('Velocity versus torque command and the slave '
                          'controller over three scenarios.',
                          sweeps=True)
    args = ap.parse_args(argv)
    par, legacy_sensor, out = studyutil.setup(args)
    if args.sweeps:
        sweeps(par, legacy_sensor, args.workers, args.seed)

    # Collect data: every controller in every scenario
    cs = controller_set()
    cases = [Case(par, scenarios.make(sc, par), c, legacy_sensor,
                  args.seed) for sc in SCENARIOS for c in cs]
    results = run_cases(cases, args.workers)

    # Report and save, one scenario at a time
    width = max(20, max(len(c.name) for c in cs))
    for i, sc in enumerate(SCENARIOS):
        rs = results[i * len(cs):(i + 1) * len(cs)]
        s = scenarios.make(sc, par)
        print(f'\n=== {sc}  (T={s.T:g} s, event at {s.tEvent:g} s)')
        print(f'{"controller":<{width}} {"settle s":>9} {"oversh":>9} '
              f'{"peak|e|":>9} {"rms e":>9} {"ripple":>9} '
              f'{"P_max":>7} {"margin":>7} {"V_ok mL":>8} '
              f'{"stroke mm":>10}')
        for r in rs:
            m = r.metrics
            if math.isnan(m['tSettle']):
                settle = 'nan'
            else:
                settle = f'{m["tSettle"]:.2f}'
            print(f'{r.name:<{width}} {settle:>9} {m["overshoot"]:9.3f} '
                  f'{m["peakErr"]:9.3f} {m["rmsErr"]:9.4f} '
                  f'{m["ripple"]:9.4f} {m["P_max"]:7.2f} '
                  f'{m["degasMargin"]:7.2f} {m.get("V_ok", 0):8.1f} '
                  f'{m["stroke_mm"]:10.1f}')
        # Stiffness at the start and end of the slave controller's run,
        # [Pa/m / 1e9 -> MPa/mm]
        r = rs[-1]
        print(f'stiffness dP/dx: start {r.stiffness[0] / 1e9:.3g}, '
              f'end {r.stiffness[-1] / 1e9:.3g} MPa/mm   ({r.name})')
        studyutil.write_traces(out / f'{sc}.csv', rs, par)
        studyutil.plot_traces(out / f'{sc}.png', sc, rs)
    print(f'\nwrote CSV and PNG to {out}')


if __name__ == '__main__':
    main()
