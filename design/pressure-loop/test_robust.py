#!/usr/bin/env python3
"""Self-checks of the robustness study helpers.

    python3 design/pressure-loop/test_robust.py

Prints `ok  ...` per check and stops at the first failure.
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


def check_sampler():
    n = 50
    a = R.lhs(n, seed=3)
    assert len(a) == n and set(a[0]) == set(R.NAMES)
    for u in R.UNCERTAIN:
        x = np.array([s[u.name] for s in a])
        assert x.min() >= u.lo and x.max() <= u.hi, u.name
        # one sample in each of n equal strata (log space for log parameters)
        f = (np.log(x / u.lo) / math.log(u.hi / u.lo)) if u.log else (x - u.lo) / (u.hi - u.lo)
        assert sorted(np.floor(f * n).astype(int)) == list(range(n)), f'{u.name} not stratified'
    assert R.lhs(n, seed=3) == a and R.lhs(n, seed=4) != a
    ok('Latin hypercube: in range, one sample per stratum, reproducible')
    p = R.make_params(a[0])
    assert p.K == a[0]['K'] and p.tol == R.THRESH['inband'] * 1e6
    s = R.make_scenario('discharge', p)
    assert s.Qf == p.Qf and R.make_scenario('valve', p).Qf == p.Qf
    ok('sampled flow reaches the discharge and valve scenarios')


def hand_result(P, scen, target=10e6):
    """Result with a given pressure trace (Pa) at 2 ms, enough for extras()."""
    t = np.arange(len(P)) * 0.002
    z = np.zeros_like(t)
    return Result('x', scen.name, 'V', t, np.asarray(P, float), z, z, z, z,
                  np.full_like(t, target), z, z, {}, 0.0, 3.4e-3)


def check_criteria():
    th = R.THRESH
    s = scenarios.Scenario('valve', 10.0, 1.0)
    n = 5000
    P = np.full(n, 10e6)
    P[600:900] = 9.4e6                      # 0.6 MPa dip after the event at 1 s (index 500)
    P[-200:] = 10.05e6                      # 0.05 MPa off at the end
    r = hand_result(P, s)
    e = R.extras(r, s)
    assert abs(e['dip'] - 0.6) < 1e-9 and abs(e['maxErrLate'] - 0.05) < 1e-9, e
    m = {'overshoot': 0.05, 'tSettle': 1.2, 'ripple': 0.001, 'stroke_mm': 40.0, **e}
    ev = R.evaluate('valve', m)
    assert ev['valve.dip'] and ev['valve.band'] and ev['stroke@valve']
    assert not R.evaluate('valve', {**m, 'dip': 1.01})['valve.dip']
    assert not R.evaluate('valve', {**m, 'maxErrLate': 0.11})['valve.band']
    assert not R.evaluate('valve', {**m, 'tSettle': math.nan})['valve.band']
    assert not R.evaluate('valve', {**m, 'dip': math.nan})['valve.dip']
    c = R.evaluate('compress', m)
    assert c == {'compress.band': True, 'compress.overshoot': True, 'compress.ripple': True, 'stroke@compress': True}
    assert not R.evaluate('compress', {**m, 'overshoot': 0.51})['compress.overshoot']
    assert not R.evaluate('compress', {**m, 'ripple': 0.05})['compress.ripple']
    assert not R.evaluate('stiffstep', {**m, 'overshoot': 0.61})['stiffstep.overshoot']
    assert not R.evaluate('stiffstep', {**m, 'stroke_mm': th['stroke_mm'] + 1})['stroke@stiffstep']
    allm = {k: m for k in R.SCENARIOS}
    assert R.evaluate_all(allm)['all']
    allm['compress'] = {**m, 'stroke_mm': 500.0}
    res = R.evaluate_all(allm)
    assert not res['stroke'] and not res['all'] and res['valve.dip']
    ok('criteria on a hand-made trace and metrics (limits, NaN fails, stroke over all scenarios)')


def check_run():
    vals = R.lhs(4, seed=5)
    spec = R.ControllerSpec('t')
    pts = [(v, s, spec, 10 + i) for i, v in enumerate(vals) for s in ('stiffstep', 'valve')]
    serial = R.run_points(pts, workers=1)
    par = R.run_points(pts, workers=4)
    for a, b in zip(serial, par):
        assert a.keys() == b.keys() and all(a[k] == b[k] or (a[k] != a[k] and b[k] != b[k]) for k in a), \
            'parallel run differs from serial'   # NaN (never settled) equals NaN
    assert all(not m['diverged'] for m in serial)
    m = serial[0]
    assert set(m) >= {'overshoot', 'tSettle', 'ripple', 'stroke_mm', 'dip', 'maxErrLate'}
    ok('4-sample parallel run equals serial run')


if __name__ == '__main__':
    check_sampler()
    check_criteria()
    check_run()
