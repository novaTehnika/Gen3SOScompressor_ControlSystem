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
  criteria  The process metrics (dip, saturation margin, deliverable
            volume) come out right on a hand-made pressure trace, and
            the requirement checks give the expected verdicts on
            metrics, including NaN (which fails) and the stroke
            requirement across scenarios.
  run       A small parallel run gives the same metrics as a serial run.

Usage:
    python3 design/pressure-loop/test_robust.py

FILE DEPENDENCY:
./
    metrics.py, params.py, robust.py, scenarios.py, sim.py
tests/run_all.sh reruns this script when a file under these paths
changes (paths from the repository root):
Depends on: design/pressure-loop slave-src tools
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import metrics  # noqa: E402
import robust as R  # noqa: E402
import scenarios  # noqa: E402
from params import Params  # noqa: E402
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

    par = R.make_params(a[0], ratio=2.0)
    assert par.K == a[0]['K'] and par.Cd == a[0]['Cd']
    assert par.tol == R.THRESH['inband'] * 1e6 and par.ratio == 2.0
    s = R.make_scenario('deliver', par)
    assert s.ratio == 2.0 and s.P0 == par.P_target and s.dissolved
    s = R.make_scenario('compress', par)
    assert s.P0 == par.P_supply and s.ratio == 2.0 and not s.dissolved
    ok('sampled Cd and supply pressure and the charge reach the scenarios')


# %% ===== PROCESS METRICS ===============================================
def hand_result(P, scen, P_sat, V_out, target=10e6):
    """Return a Result with pressure trace P [Pa] at 2 ms per sample,
    saturation pressure P_sat [Pa] and delivered volume V_out [m^3]."""
    t = np.arange(len(P)) * 0.002                           # [s]
    z = np.zeros_like(t)
    P = np.asarray(P, float)                # measured taken as true
    return Result('x', scen.name, 'V', t, P, P, z, z, z,
                  np.full_like(t, target), z, z, {}, 0.0, 3.4e-3,
                  np.full_like(t, P_sat), np.asarray(V_out, float), z)


def check_process_metrics():
    par = Params(tol=0.1e6, degas_margin=0.5e6, P_ceiling=12e6,
                 e_crash=1.0e6)
    s = scenarios.Scenario('deliver', 10.0, 1.0, P0=10e6, t_open=1.0)
    n = 5000
    P = np.full(n, 10e6)
    P[500:800] = 9.4e6          # 0.6 MPa dip from the opening at 1 s
    P[4000:] = 10.3e6           # 0.3 MPa high at the end: no violation
    # Delivery starts at the opening (sample 500)
    V_out = np.concatenate([np.zeros(500),
                            np.linspace(0, 300e-6, n - 500)])
    r = hand_result(P, s, P_sat=3.5e6, V_out=V_out)
    m = metrics.process(r, par, s)
    assert abs(m['dip'] - 0.6) < 1e-9 and abs(m['P_max'] - 10.3) < 1e-9
    assert abs(m['P_min'] - 9.4) < 1e-9
    assert abs(m['degasMargin'] - 5.9) < 1e-9, m
    assert abs(m['V_delivered'] - 300) < 1e-9
    assert abs(m['V_ok'] - 300) < 1e-9, m
    ok('process metrics on a clean delivery: dip, P_min, margin, V_ok = '
       'V_delivered')

    # The dip degasses a solution saturated near the target: V_ok is 0.
    m = metrics.process(hand_result(P, s, 9.5e6, V_out), par, s)
    assert m['degasMargin'] < 0 and m['V_ok'] == 0.0, m
    # The dip stays above saturation but inside the margin: the same.
    m = metrics.process(hand_result(P, s, 9.2e6, V_out), par, s)
    assert m['degasMargin'] > 0 and m['V_ok'] == 0.0, m
    # Control is lost late: V_ok stops at the first error above e_crash
    # after the dip has recovered.
    P2 = P.copy()
    P2[3000:] = 8.5e6
    m = metrics.process(hand_result(P2, s, 3.5e6, V_out), par, s)
    assert abs(m['V_ok'] - V_out[3000] * 1e6) < 1e-6, m
    # The ceiling counts as a violation too.
    P3 = P.copy()
    P3[2000:2100] = 12.5e6
    m = metrics.process(hand_result(P3, s, 3.5e6, V_out), par, s)
    assert abs(m['V_ok'] - V_out[2000] * 1e6) < 1e-6, m
    ok('V_ok stops at degassing, at loss of control and at the ceiling')


# %% ===== REQUIREMENT CRITERIA ==========================================
def check_criteria():
    th = R.THRESH
    good = {'maxErrLate': 0.05, 'tSettle': 1.2, 'P_max': 10.3,
            'degasMargin': 2.0, 'V_ok': 300.0, 'V_delivered': 300.0,
            'stroke_mm': 40.0}
    ev = R.evaluate('deliver', good)
    assert ev['deliver.ceiling'] and ev['deliver.degas']
    assert ev['deliver.control'] and ev['stroke@deliver']
    assert not R.evaluate('deliver', {**good, 'P_max': 12.1})[
        'deliver.ceiling']
    assert not R.evaluate('deliver', {**good, 'degasMargin': 0.4})[
        'deliver.degas']
    assert not R.evaluate('deliver', {**good, 'degasMargin': math.nan})[
        'deliver.degas']
    assert not R.evaluate('deliver', {**good, 'V_ok': 250.0})[
        'deliver.control']
    c = R.evaluate('compress', good)
    assert c == {'compress.band': True, 'compress.ceiling': True,
                 'compress.degas': True, 'stroke@compress': True}
    assert not R.evaluate('compress', {**good, 'maxErrLate': 0.11})[
        'compress.band']
    assert not R.evaluate('compress', {**good, 'tSettle': math.nan})[
        'compress.band']
    assert not R.evaluate('compress',
                          {**good, 'stroke_mm': th['stroke_mm'] + 1}
                          )['stroke@compress']

    # The stroke requirement must hold in every scenario.
    allm = {k: good for k in R.SCENARIOS}
    assert R.evaluate_all(allm)['all']
    allm['compress'] = {**good, 'stroke_mm': 500.0}
    res = R.evaluate_all(allm)
    assert not res['stroke'] and not res['all'] and res['deliver.degas']
    ok('criteria on hand-made metrics (ceiling, degas, control, band, '
       'NaN fails, stroke over all scenarios)')

    # KeyMetric.excess is above 1 exactly when the metric fails.
    km = R.KEY_BY_KEY[('deliver', 'degasMargin')]
    assert km.excess(th['degas'] + 0.1) < 1 < km.excess(th['degas'] - 0.1)
    assert km.excess(math.nan) == math.inf
    km = R.KEY_BY_KEY[('deliver', 'P_max')]
    assert km.excess(11.9) < 1 < km.excess(12.1)
    ok('KeyMetric.excess crosses 1 at the limit in both senses')


# %% ===== PARALLEL RUN ==================================================
def check_run():
    vals = R.lhs(4, seed=5)
    spec = R.ControllerSpec('t')
    pts = [(v, s, spec, 10 + i) for i, v in enumerate(vals)
           for s in ('compress', 'deliver')]
    serial = R.run_points(pts, workers=1)
    parallel = R.run_points(pts, workers=4)
    for a, b in zip(serial, parallel):
        # NaN (never settled) counts as equal to NaN.
        same = all(a[k] == b[k] or (a[k] != a[k] and b[k] != b[k])
                   for k in a)
        assert a.keys() == b.keys() and same, \
            'parallel run differs from serial'
    assert all(not m['diverged'] for m in serial)
    m = serial[1]
    assert set(m) >= set(R.METRIC_NAMES)
    assert m['V_delivered'] > 200 and math.isfinite(m['degasMargin'])
    ok('4-sample parallel run equals serial run')


if __name__ == '__main__':
    check_sampler()
    check_process_metrics()
    check_criteria()
    check_run()
