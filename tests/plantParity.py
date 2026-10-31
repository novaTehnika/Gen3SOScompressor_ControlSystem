#!/usr/bin/env python3
"""Parity of the C plant (models/c/plant.c) with the Python plant
(models/plant.py).

    python3 tests/plantParity.py      (from anywhere; needs python3 and cc)

Builds plant.c as a shared library, constructs each case's Plant in
Python and hands its parameters and initial state to the C plant, then
drives both with the same inputs every scan and compares every state, the
slewed reference, the outflow, the hose state and the parcel log. The
cases cover compression into the stall, dissolution, opening the hose in
one scan, a delivery through the orifices with the tube's filling flow, a
throttled proximal valve that logs parcels and joins the hose, torque
mode against friction, and the legacy options (slew limit, Euler, fixed
liquid).

Depends on: models slave-src tools
"""

import ctypes
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'models'))
from params import Params              # noqa: E402
from plant import Plant, N_STATES      # noqa: E402
from types import SimpleNamespace     # noqa: E402

N_EXTRA = 7
NAMES = ['x', 'v', 'P', 'n_dis', 'n_tot', 'V_l', 'n_out', 'V_out', 'V_vir', 'V_in',
         'v_ref', 'Q', 'Vch0', 'hose_open', 'V_in_total', 'V_degas', 'n_parcels']
# Absolute floors of the comparison, per entry of NAMES; it is otherwise
# relative, 1e-9.
FLOOR = [1e-12, 1e-15, 1e-3, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15, 1e-15,
         1e-15, 1e-15, 1e-15, 0, 1e-15, 1e-15, 0]
TOL = 1e-9


def build(tmp):
    so = tmp / 'libplant.so'
    subprocess.run([os.environ.get('CC', 'cc'), '-O2', '-shared', '-fPIC', '-o', str(so),
                    str(ROOT / 'models' / 'c' / 'plant.c'), '-lm'], check=True)
    lib = ctypes.CDLL(str(so))
    lib.plant_size.restype = ctypes.c_int
    lib.plant_scan.restype = ctypes.c_double
    lib.plant_scan.argtypes = [ctypes.c_void_p] + [ctypes.c_double] * 4
    lib.plant_open_hose.argtypes = [ctypes.c_void_p]
    lib.plant_state.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.plant_init_flat.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int,
                                    ctypes.c_void_p, ctypes.c_double, ctypes.c_int]
    count = ctypes.c_int.in_dll(lib, 'plant_param_count').value
    names = (ctypes.c_char_p * count).in_dll(lib, 'plant_param_names')
    return lib, [n.decode() for n in names]


def c_plant(lib, names, py):
    """A C plant with the parameters and initial state of Plant py."""
    attr = {'heun': None, 'n_dead': len(py.cmd_queue)}
    values = []
    for n in names:
        if n == 'heun':
            values.append(1.0 if py.step.__name__ == 'heun_step' else 0.0)
        elif n in attr:
            values.append(float(attr[n]))
        else:
            values.append(float(getattr(py, n)))
    par = (ctypes.c_double * len(values))(*values)
    y0 = (ctypes.c_double * N_STATES)(*py.y)
    buf = ctypes.create_string_buffer(lib.plant_size())
    lib.plant_init_flat(buf, par, int(py.velocity_mode), y0, py.Vch0, int(py.hose_open))
    return buf


def py_state(py):
    return list(py.y) + [py.v_ref, py.Q, py.Vch0, float(py.hose_open), py.V_in_total,
                         py.V_degas, float(len(py.parcels))]


def c_state(lib, buf):
    out = (ctypes.c_double * (N_STATES + N_EXTRA))()
    lib.plant_state(buf, out)
    return list(out)


def run(lib, names, case, par, mode, inputs, seconds):
    sc = SimpleNamespace(Qf=0.0, hose_open0=case.get('hose_open0', False),
                         initial=lambda p: p.contents(p.P_supply, 1.0))
    py = Plant(par, sc, mode)
    c = c_plant(lib, names, py)
    n = int(round(seconds / par.Ts))
    worst = 0.0
    pmax = 0.0
    stalls = 0
    for k in range(n):
        t = k * par.Ts
        u, valve, Q_tube, prox, open_hose = inputs(t)
        if open_hose:
            py.open_hose()
            lib.plant_open_hose(c)
        v_before = py.v
        py.scan(u, valve, Q_tube, prox)
        lib.plant_scan(c, u, valve, Q_tube, prox)
        pmax = max(pmax, py.P)
        stalls += mode == 'V' and v_before > 0 and py.v == 0.0 and u > 0
        a, b = py_state(py), c_state(lib, c)
        for i, (va, vb) in enumerate(zip(a, b)):
            err = abs(va - vb)
            bound = TOL * max(abs(va), abs(vb)) + FLOOR[i]
            if err > bound:
                sys.exit(f'{case["name"]}: {NAMES[i]} at {t:.3f} s: python {va!r}, c {vb!r}')
            if bound > 0:
                worst = max(worst, err / bound)
    print(f'ok  {case["name"]}: {n} scans, largest difference {worst:.2g} of the tolerance, '
          f'P up to {pmax / 1e6:.2f} MPa, {len(py.parcels)} parcels, {stalls} stalls')


def main():
    base = Params()
    Ts = base.Ts
    cases = [
        ({'name': 'compress, stall, dissolve, open the hose, deliver'}, base, 'V',
         lambda t: (2e-3 if t < 10 else -0.5e-3 if t < 11 else 0.2e-3 if t > 13 else 0.0,
                    1.0 if t >= 13 else 0.0,
                    base.V_tube / Ts if abs(t - 13) < Ts / 2 else 0.0,
                    0.0, abs(t - 12) < Ts / 2), 16.0),
        ({'name': 'throttled proximal valve'}, base, 'V',
         lambda t: (0.3e-3 if t < 2 else 0.0, 0.0, 0.0,
                    min(max((t - 2.5) / 0.5, 0.0), 1.0), False), 5.0),
        ({'name': 'torque mode with an open hose', 'hose_open0': True}, base, 'T',
         lambda t: (20e3 if t < 1.5 else -5e3 if t < 2.5 else 8e3,
                    1.0 if t > 3 else 0.0, 0.0, 0.0, False), 5.0),
        ({'name': 'legacy: slew limit, Euler, fixed liquid', 'hose_open0': True},
         base.legacy().with_(solver='euler'), 'V',
         lambda t: (0.4e-3 if t < 1.5 else -0.2e-3, 1.0 if t > 2 else 0.0, 0.0, 0.0, False),
         4.0),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        lib, names = build(Path(tmp))
        for case, par, mode, inputs, seconds in cases:
            run(lib, names, case, par, mode, inputs, seconds)
    print('all passed')


if __name__ == '__main__':
    main()
