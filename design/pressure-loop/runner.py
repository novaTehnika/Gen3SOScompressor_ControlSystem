"""
runner.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module runs many simulation cases in parallel. A Case bundles the
parameters, scenario and controller of one run. run_cases() runs a list
of cases in worker processes and returns their results in the same
order. For example:

    cases = [Case(par, scenarios.make('stiffstep', par),
                  SlaveController(Kp=k)) for k in (0.25, 0.5)]
    results = run_cases(cases)

The worker processes are started with the 'spawn' method, so each one
imports the modules afresh. A Case therefore holds only objects that can
be pickled: Params and Scenario are frozen dataclasses, and a controller
holds only its configuration until reset() is called.

FILE DEPENDENCY:
./
    sim.py
"""

import multiprocessing as mp
import os
from dataclasses import dataclass
from typing import Any

import sim


# %% ===== CASE DEFINITION ===============================================
@dataclass
class Case:
    """One simulation run: the arguments of sim.run()."""

    params: Any                 # Params
    scenario: Any               # Scenario
    controller: Any             # SlaveController or LegacyController
    legacy_sensor: bool = False
    seed: int = 1               # noise seed
    label: str = ''


def run_case(case):
    """Run one case and return its sim.Result."""
    return sim.run(case.params, case.scenario, case.controller,
                   legacy_sensor=case.legacy_sensor, seed=case.seed)


def _strip(result):
    """Remove the traces from a Result, keeping its metrics, so that it
    transfers quickly between processes."""
    for name in ('P_meas', 'u', 'v', 'x', 'target', 'stiffness', 'Qout',
                 'P', 't'):
        setattr(result, name, None)
    return result


def _run_light(case):
    """Run one case and return its Result without traces."""
    return _strip(run_case(case))


# %% ===== PARALLEL EXECUTION ============================================
def run_cases(cases, workers=None, traces=True):
    """Return the Results of cases, in order.

    workers=1 runs the cases in this process; the default is one worker
    per CPU. With traces=False each Result holds only its metrics, and
    its trace fields are None.
    """
    cases = list(cases)
    workers = min(workers or os.cpu_count() or 1, len(cases))
    run_one = run_case if traces else _run_light
    if workers <= 1:
        return [run_one(c) for c in cases]
    with mp.get_context('spawn').Pool(workers) as pool:
        return pool.map(run_one, cases, chunksize=1)
