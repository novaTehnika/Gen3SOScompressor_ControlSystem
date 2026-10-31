#!/usr/bin/env python3
"""Parity of the py target of tools/st_translate.py against the c target.

    python3 tests/stTargetParity.py          (from anywhere; needs python3 and cc)

Regenerates both translations into temporary directories, then drives every
function block whose inputs are all scalars with one seeded random input
sequence, once through the generated Python and once through the generated
C (a driver program written here and compiled with the c target), and
compares every output and internal variable after every call. Integer
inputs stay in their type's range, as on the controller.

Depends on: tools slave-src slave-sim/c
"""

import csv
import os
import importlib
import math
import random
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
import st_translate as stt  # noqa: E402
import st_target_c as stc  # noqa: E402

STEPS = 3000                # calls for the blocks named in CORE, STEPS_OTHER for the rest
STEPS_OTHER = 800
CORE = {'FB_PressureController', 'FB_AnalogInputFilter', 'FB_PressureInput',
        'FB_PositionPathShaper', 'FB_DigitalInputFilter',
        'FB_PressureSchedule'}
TOL = 1e-9
SIGNALS = {'Target', 'Actual', 'Volts', 'RawInput', 'ActualPosition', 'Position',
           'Pressure', 'Velocity'}
PARAM_EDGES = [0.0, -1.0, 1e-9, 1e6]
TIME_POOL = [0.0, 0.002, 0.01, 0.05, 0.5]
SCALAR = stt.NUM_TYPES | {'BOOL', 'TIME'}


def flat_py(x, out):
    if isinstance(x, list):
        out.extend(float(v) for v in x)
    elif hasattr(x, '__dict__'):
        for k in sorted(vars(x)):
            flat_py(vars(x)[k], out)
    else:
        out.append(float(x))
    return out


def make_inputs(rng, inputs, ctx, steps):
    """Rows of (dt, input values): signals redraw every call, flags toggle,
    parameters hold a value and occasionally jump to an edge case."""
    nominal = {d['name']: rng.uniform(0.1, 10.0) for d in inputs}
    state = {d['name']: 0.0 for d in inputs}
    rows = []
    for _ in range(steps):
        row = [rng.choice([0.001, 0.002, 0.005])]
        for d in inputs:
            n, typ = d['name'], d['type']
            if typ == 'BOOL':
                if rng.random() < (0.08 if n != 'Enable' else 0.03):
                    state[n] = 1.0 - state[n] if state[n] in (0.0, 1.0) else 1.0
                if _ == 0 and n == 'Enable':
                    state[n] = 1.0
            elif typ == 'TIME':
                if rng.random() < 0.02:
                    state[n] = rng.choice(TIME_POOL)
            elif typ in ctx['enums']:
                if rng.random() < 0.1:
                    state[n] = float(rng.randrange(len(ctx['enums'][typ])))
            elif typ in stt.INT_TYPES:
                if rng.random() < 0.05:
                    state[n] = float(rng.choice([0, 1, 2, 3, 4, 5, 7] + ([] if typ in UNSIGNED else [-1])))
            elif n in SIGNALS:
                state[n] = rng.uniform(-12.0, 12.0) if rng.random() < 0.97 else rng.choice([0.0, 1e6])
            elif rng.random() < 0.02:
                state[n] = rng.choice(PARAM_EDGES) if rng.random() < 0.3 else nominal[n] * rng.uniform(0.5, 2)
            elif _ == 0:
                state[n] = nominal[n]
            row.append(state[n])
        rows.append(row)
    return rows


UNSIGNED = {'UINT', 'UDINT', 'USINT', 'BYTE', 'WORD'}


def c_flatten(typ, ref, ctx, lines):
    """C statements that print every variable of ref (of ST type typ) in
    the order flat_py visits the Python namespace: fields sorted by name,
    arrays element by element."""
    for name, ftyp in sorted(stc.fields_of(typ, ctx)):
        m = stc.ARRAY_RE.match(ftyp)
        base = m.group(3) if m else ftyp
        if not m and (base in ctx['structs'] or base in ctx['fbs'] or base in stc.STD_FBS):
            c_flatten(base, f'{ref}.{name}', ctx, lines)
        elif m:
            n = int(m.group(2)) + 1
            lines.append(f'    for (int i = 0; i < {n}; i++) put(out, (double){ref}.{name}[i]);')
        else:
            lines.append(f'    put(out, (double){ref}.{name});')


def c_driver(cases, ctx):
    """One C program that runs any of the blocks in cases on an input CSV:
    drv FB_Name in.csv out.csv."""
    lines = ['#include <stdio.h>', '#include <string.h>', '#include <math.h>',
             '#include "st_slave.h"', '',
             'static int first;',
             'static void put(FILE *out, double v)',
             '{ fprintf(out, first ? "%.17g" : ",%.17g", v); first = 0; }', '']
    for name, (inputs, _) in cases.items():
        lines += [f'static void flat_{name}(FILE *out, const {name}_t *p)', '{', '    first = 1;']
        c_flatten(name, '(*p)', ctx, lines)
        lines += ['    fputc(\'\\n\', out);', '}', '',
                  f'static void drive_{name}(FILE *in, FILE *out)', '{',
                  f'    static {name}_t s;', f'    init_{name}(&s);', f'    flat_{name}(out, &s);',
                  f'    double v[{len(inputs) + 1}];',
                  '    for (;;) {']
        for k in range(len(inputs) + 1):
            sep = '' if k == 0 else ','
            lines.append(f'        if (fscanf(in, "{sep}%lf", &v[{k}]) != 1) return;')
        for k, d in enumerate(inputs, 1):
            ct = stc.c_type(d['type'], ctx)[0]
            if d['type'] == 'BOOL':
                lines.append(f'        s.{d["name"]} = v[{k}] != 0;')
            elif ct in ('double', 'float'):
                lines.append(f'        s.{d["name"]} = v[{k}];')
            else:
                lines.append(f'        s.{d["name"]} = ({ct})(long)v[{k}];')
        lines += [f'        {name}(&s, v[0]);', f'        flat_{name}(out, &s);', '    }', '}', '']
    lines += ['int main(int argc, char **argv)', '{',
              '    FILE *in = fopen(argv[2], "r"), *out = fopen(argv[3], "w");',
              '    if (!in || !out) return 2;']
    for name in cases:
        lines.append(f'    if (!strcmp(argv[1], "{name}")) drive_{name}(in, out);')
    lines += ['    fclose(out);', '    return 0;', '}']
    return '\n'.join(lines) + '\n'


def close(a, b):
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= TOL * max(1.0, abs(b))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for out in ('c', 'py'):
            r = subprocess.run([sys.executable, str(ROOT / 'tools' / 'st_translate.py'), '--target', out,
                                str(ROOT / 'slave-src'), str(tmp / out)], capture_output=True, text=True)
            assert r.returncode == 0, r.stdout + r.stderr
        sys.path.insert(0, str(tmp / 'py'))

        ctx, _ = stt.load_project(ROOT / 'slave-src')
        cases = {}
        for name in sorted(ctx['fbs']):
            decls = ctx['pou_decls'][name]
            inputs = [d for d in decls if d['section'] == 'VAR_INPUT']
            if any(d['type'] not in SCALAR and d['type'] not in ctx['enums'] for d in inputs):
                print(f'skip {name} (non-scalar inputs)')
                continue
            rng = random.Random(name)
            rows = make_inputs(rng, inputs, ctx, STEPS if name in CORE else STEPS_OTHER)
            with open(tmp / f'{name}_in.csv', 'w', newline='') as f:
                csv.writer(f).writerows(([repr(v) for v in r] for r in rows))
            cases[name] = (inputs, rows)

        (tmp / 'drv.c').write_text(c_driver(cases, ctx))
        r = subprocess.run([os.environ.get('CC', 'cc'), '-O0', '-o', str(tmp / 'drv'), str(tmp / 'drv.c'),
                            str(tmp / 'c' / 'st_slave.c'), f'-I{tmp / "c"}',
                            f'-I{ROOT / "slave-sim" / "c"}', '-lm'], capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr
        for name in cases:
            r = subprocess.run([str(tmp / 'drv'), name, str(tmp / f'{name}_in.csv'),
                                str(tmp / f'{name}_out.csv')], capture_output=True, text=True)
            assert r.returncode == 0, f'{name}: driver failed {r.returncode} {r.stderr}'

        for name, (inputs, rows) in cases.items():
            mod = importlib.import_module(name)
            s = getattr(mod, f'new_{name}')()
            fb = getattr(mod, name)
            ref = [[float(v) for v in line.split(',')] for line in
                   (tmp / f'{name}_out.csv').read_text().split()]
            got = [flat_py(s, [])]
            for row in rows:
                for d, v in zip(inputs, row[1:]):
                    setattr(s, d['name'], v != 0 if d['type'] == 'BOOL' else
                            int(v) if d['type'] in stt.INT_TYPES or d['type'] in ctx['enums'] else v)
                assert fb(s, row[0]) is None
                got.append(flat_py(s, []))
            fields = []
            flat_py_names(s, '', fields)
            assert len(got[0]) == len(ref[0]), f'{name}: state width {len(got[0])} vs {len(ref[0])}'
            worst = 0.0
            for k, (g, m) in enumerate(zip(got, ref)):
                for j, (a, b) in enumerate(zip(g, m)):
                    assert close(a, b), (f'{name}: call {k}, {fields[j]}: python {a!r} '
                                         f'c {b!r}')
                    if math.isfinite(a) and math.isfinite(b):
                        worst = max(worst, abs(a - b))
            print(f'ok  {name}: {len(rows)} calls, {len(got[0])} variables, max diff {worst:.1e}')


def flat_py_names(x, prefix, out):
    if isinstance(x, list):
        out.extend(f'{prefix}[{i}]' for i in range(len(x)))
    elif hasattr(x, '__dict__'):
        for k in sorted(vars(x)):
            flat_py_names(vars(x)[k], f'{prefix}.{k}' if prefix else k, out)
    else:
        out.append(prefix)


if __name__ == '__main__':
    try:
        main()
    except AssertionError as err:
        print(f'FAIL {err}')
        sys.exit(1)
