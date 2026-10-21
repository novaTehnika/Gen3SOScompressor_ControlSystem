#!/usr/bin/env python3
"""
test_robust.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script checks the helper functions of the robustness study
(robust.py). It prints "ok" and a short result for each check and stops
at the first failure. tests/run_all.sh runs it.

The checks are:
  sampler   The Latin hypercube stays in range, puts one sample in each
            stratum of every parameter, and is reproducible; the sampled
            flow reaches both outflow scenarios.
  criteria  The requirement checks give the expected verdicts on a
            hand-made pressure trace and metrics, including NaN (which
            fails) and the stroke requirement across scenarios.
  run       A small parallel run gives the same metrics as a serial run.

Usage:
    python3 design/pressure-loop/test_robust.py

FILE DEPENDENCY:
./
    robust.py, scenarios.py, sim.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import robust as R  # noqa: E402
import scenarios  # noqa: E402
from sim import Result  # noqa: E402


def ok(msg):
    print(f'ok  {msg}')


# %% ===== SAMPLER =======================================================
def check_sampler():
    n = 50
    a = R.lhs(n, seed=3)
    assert len(a) == n and set(a[0]) == set(R.NAMES)
    for u in R.UNCERTAIN:
        x = np.array([s[u.name] for s in a])
        assert x.min() >= u.lo and x.max() <= u.hi, u.name
        # One sample in each of n equal strata, in log space for log
        # parameters
        if u.log:
            f = np.log(x / u.lo) / math.log(u.hi / u.lo)
        else:
            f = (x - u.lo) / (u.hi - u.lo)
        assert sorted(np.floor(f * n).astype(int)) == list(range(n)), \
            f'{u.name} not stratified'
    assert R.lhs(n, seed=3) == a and R.lhs(n, seed=4) != a
    ok('Latin hypercube: in range, one sample per stratum, reproducible')

    par = R.make_params(a[0])
    assert par.K == a[0]['K'] and par.tol == R.THRESH['inband'] * 1e6
    s = R.make_scenario('discharge', par)
    assert s.Qf == par.Qf and R.make_scenario('valve', par).Qf == par.Qf
    ok('sampled flow reaches the discharge and valve scenarios')


# %% ===== REQUIREMENT CRITERIA ==========================================
def hand_result(P, scen, target=10e6):
    """Return a Result with pressure trace P [Pa] at 2 ms per sample,
    with enough fields for extras()."""
    t = np.arange(len(P)) * 0.002                           # [s]
    z = np.zeros_like(t)
    return Result('x', scen.name, 'V', t, np.asarray(P, float), z, z, z, z,
                  np.full_like(t, target), z, z, {}, 0.0, 3.4e-3)


def check_criteria():
    th = R.THRESH
    s = scenarios.Scenario('valve', 10.0, 1.0)
    n = 5000
    P = np.full(n, 10e6)
    # 0.6 MPa dip after the event at 1 s (sample 500), and 0.05 MPa off
    # at the end
    P[600:900] = 9.4e6
    P[-200:] = 10.05e6
    r = hand_result(P, s)
    e = R.extras(r, s)
    assert abs(e['dip'] - 0.6) < 1e-9, e
    assert abs(e['maxErrLate'] - 0.05) < 1e-9, e

    m = {'overshoot': 0.05, 'tSettle': 1.2, 'ripple': 0.001,
         'stroke_mm': 40.0, **e}
    ev = R.evaluate('valve', m)
    assert ev['valve.dip'] and ev['valve.band'] and ev['stroke@valve']
    assert not R.evaluate('valve', {**m, 'dip': 1.01})['valve.dip']
    assert not R.evaluate('valve', {**m, 'maxErrLate': 0.11})['valve.band']
    assert not R.evaluate('valve',
                          {**m, 'tSettle': math.nan})['valve.band']
    assert not R.evaluate('valve', {**m, 'dip': math.nan})['valve.dip']

    c = R.evaluate('compress', m)
    assert c == {'compress.band': True, 'compress.overshoot': True,
                 'compress.ripple': True, 'stroke@compress': True}
    assert not R.evaluate('compress',
                          {**m, 'overshoot': 0.51})['compress.overshoot']
    assert not R.evaluate('compress',
                          {**m, 'ripple': 0.05})['compress.ripple']
    assert not R.evaluate('stiffstep',
                          {**m, 'overshoot': 0.61})['stiffstep.overshoot']
    assert not R.evaluate('stiffstep',
                          {**m, 'stroke_mm': th['stroke_mm'] + 1}
                          )['stroke@stiffstep']

    # The stroke requirement must hold in every scenario.
    allm = {k: m for k in R.SCENARIOS}
    assert R.evaluate_all(allm)['all']
    allm['compress'] = {**m, 'stroke_mm': 500.0}
    res = R.evaluate_all(allm)
    assert not res['stroke'] and not res['all'] and res['valve.dip']
    ok('criteria on a hand-made trace and metrics (limits, NaN fails, '
       'stroke over all scenarios)')


# %% ===== PARALLEL RUN ==================================================
def check_run():
    vals = R.lhs(4, seed=5)
    spec = R.ControllerSpec('t')
    pts = [(v, s, spec, 10 + i) for i, v in enumerate(vals)
           for s in ('stiffstep', 'valve')]
    serial = R.run_points(pts, workers=1)
    parallel = R.run_points(pts, workers=4)
    for a, b in zip(serial, parallel):
        # NaN (never settled) counts as equal to NaN.
        same = all(a[k] == b[k] or (a[k] != a[k] and b[k] != b[k])
                   for k in a)
        assert a.keys() == b.keys() and same, \
            'parallel run differs from serial'
    assert all(not m['diverged'] for m in serial)
    m = serial[0]
    assert set(m) >= {'overshoot', 'tSettle', 'ripple', 'stroke_mm', 'dip',
                      'maxErrLate'}
    ok('4-sample parallel run equals serial run')


if __name__ == '__main__':
    check_sampler()
    check_criteria()
    check_run()
