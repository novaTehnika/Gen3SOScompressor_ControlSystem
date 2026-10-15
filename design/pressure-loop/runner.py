"""Run many (params, scenario, controller) cases, in parallel.

A Case holds picklable specs only (frozen dataclasses and plain controller
objects), so it crosses a spawn-started process boundary; each worker imports
the modules afresh and runs one case with sim.run.

    cases = [Case(p, scenarios.make('stiffstep', p), SlaveController(Kp=k)) for k in (0.25, 0.5)]
    results = run_cases(cases)            # list of Result, same order
"""

import multiprocessing as mp
import os
from dataclasses import dataclass
from typing import Any

import sim


@dataclass
class Case:
    params: Any
    scenario: Any
    controller: Any
    legacy_sensor: bool = False
    seed: int = 1
    label: str = ''


def run_case(case):
    return sim.run(case.params, case.scenario, case.controller,
                   legacy_sensor=case.legacy_sensor, seed=case.seed)


def _strip(result):
    """Result without its traces, to keep the transfer between processes small."""
    for name in ('P_meas', 'u', 'v', 'x', 'target', 'stiffness', 'Qout', 'P', 't'):
        setattr(result, name, None)
    return result


def _run_light(case):
    return _strip(run_case(case))


def run_cases(cases, workers=None, traces=True):
    """Results of `cases` in order. workers=1 runs in this process; the default
    is one per CPU. traces=False returns only the metrics (Result fields
    that hold arrays are None)."""
    cases = list(cases)
    workers = min(workers or os.cpu_count() or 1, len(cases))
    fn = run_case if traces else _run_light
    if workers <= 1:
        return [fn(c) for c in cases]
    with mp.get_context('spawn').Pool(workers) as pool:
        return pool.map(fn, cases, chunksize=1)
