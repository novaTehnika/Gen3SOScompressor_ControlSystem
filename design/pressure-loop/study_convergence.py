#!/usr/bin/env python3
"""
study_convergence.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/26/2026

PURPOSE/DESCRIPTION:
This script measures the integration error of the plant model against
the time step, so that the step used in the studies can be shown to be
converged. For each scenario it runs the slave controller with both
integration methods at 1, 2, 5, 10, 20 and 50 steps per scan, and
compares each run with a reference run of Heun's method at 100 steps per
scan (dt = 0.02 ms). It reports the largest and the RMS difference in
chamber pressure and the change in the overshoot, settling time and
stroke metrics.

The result at the default step shows how much of each metric is
integration error, and the way the error falls with the step shows the
order of each method. The slew limit (eq. (11)) is applied once per
step, so on scenarios it dominates the error falls at first order with
either method.

Usage:
    python3 study_convergence.py [--set key=value ...] [--out DIR]

FILE DEPENDENCY:
./
    studyparams.py, controllers.py
    scenarios.py
    sim.py
    studyutil.py
    (and the models they import, in models/)
"""

import time

import numpy as np

import controllers as ctl
import scenarios
import sim
import studyutil


# %% ===== STUDY VARIABLES ===============================================
SCENARIOS = ('compress', 'stiffstep', 'deliver')
NSUB = (1, 2, 5, 10, 20, 50)        # [-] integration steps per scan
NSUB_REF = 100                      # [-] steps per scan of the reference
SOLVERS = ('euler', 'heun')


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    ap = studyutil.parser('Integration error of the plant model against '
                          'the time step.')
    args = ap.parse_args(argv)
    par, _, out = studyutil.setup(args)
    rows = []
    for name in SCENARIOS:
        # Reference run
        ref_par = par.with_(nsub=NSUB_REF, solver='heun')
        ref = sim.run(ref_par, scenarios.make(name, ref_par),
                      ctl.SlaveController(), seed=args.seed)
        print(f'\n=== {name}: reference Heun, {NSUB_REF} steps per scan; '
              f'overshoot {ref["overshoot"]:.4f} MPa, settle '
              f'{ref["tSettle"]:.3f} s, stroke {ref["stroke_mm"]:.3f} mm')
        print(f'{"solver":>6} {"nsub":>5} {"dt ms":>6} '
              f'{"max|dP| kPa":>12} {"rms dP kPa":>11} '
              f'{"d oversh":>9} {"d settle":>9} '
              f'{"d stroke":>9} {"s/run":>6}')

        # Runs at each step with each method, compared with the reference
        for solver in SOLVERS:
            for n in NSUB:
                q = par.with_(nsub=n, solver=solver)
                t0 = time.time()
                r = sim.run(q, scenarios.make(name, q),
                            ctl.SlaveController(), seed=args.seed)
                elapsed = time.time() - t0
                dP = r.P - ref.P                                # [Pa]
                row = (name, solver, n, q.Ts / n * 1e3,
                       np.abs(dP).max() / 1e3,                  # [kPa]
                       np.sqrt(np.mean(dP**2)) / 1e3,           # [kPa]
                       r['overshoot'] - ref['overshoot'],       # [MPa]
                       r['tSettle'] - ref['tSettle'],           # [s]
                       r['stroke_mm'] - ref['stroke_mm'],       # [mm]
                       elapsed)
                rows.append(row)
                print(f'{solver:>6} {n:5d} {row[3]:6.2f} {row[4]:12.3f} '
                      f'{row[5]:11.3f} {row[6]:+9.4f} {row[7]:+9.3f} '
                      f'{row[8]:+9.4f} {row[9]:6.2f}')

    # Save
    with open(out / 'convergence.csv', 'w') as f:
        f.write('scenario,solver,nsub,dt_ms,maxdP_kPa,rmsdP_kPa,'
                'd_overshoot_MPa,d_tSettle_s,d_stroke_mm,seconds\n')
        for row in rows:
            f.write(','.join(str(x) if isinstance(x, str) else f'{x:.6g}'
                             for x in row) + '\n')
    print(f'\nwrote convergence.csv to {out}')


if __name__ == '__main__':
    main()
