#!/usr/bin/env python3
"""Parity of tools/st2py.py against tools/st2m.py.

    python3 tests/st2pyParity.py          (from anywhere; needs python3 and octave-cli)

Regenerates both translations into temporary directories, then drives every
function block whose inputs are all scalars with one seeded random input
sequence, once through the generated Python and once through the generated
Octave, and compares every output and internal variable after every call.

Depends on: tools slave-src slave-sim
"""

import csv
import importlib
import math
import random
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
import st2m  # noqa: E402

STEPS = 3000                # calls for the blocks named in CORE, STEPS_OTHER for the rest
STEPS_OTHER = 800
CORE = {'FB_PressureController', 'FB_AnalogInputFilter', 'FB_PressureInput',
        'FB_PositionCommandGate', 'FB_DigitalInputFilter',
        'FB_PressureSchedule'}
TOL = 1e-9
SIGNALS = {'Target', 'Actual', 'Volts', 'RawInput', 'ActualPosition', 'Position',
           'Pressure', 'Velocity'}
PARAM_EDGES = [0.0, -1.0, 1e-9, 1e6]
TIME_POOL = [0.0, 0.002, 0.01, 0.05, 0.5]
SCALAR = st2m.NUM_TYPES | {'BOOL', 'TIME'}


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
            elif typ in st2m.INT_TYPES:
                if rng.random() < 0.05:
                    state[n] = float(rng.choice([0, 1, 2, 3, 4, 5, 7, -1]))
            elif n in SIGNALS:
                state[n] = rng.uniform(-12.0, 12.0) if rng.random() < 0.97 else rng.choice([0.0, 1e6])
            elif rng.random() < 0.02:
                state[n] = rng.choice(PARAM_EDGES) if rng.random() < 0.3 else nominal[n] * rng.uniform(0.5, 2)
            elif _ == 0:
                state[n] = nominal[n]
            row.append(state[n])
        rows.append(row)
    return rows


FLAT_M = '''function out = st2py_flat(x)
if isstruct(x)
    out = [];
    for k = sort(fieldnames(x))'
        out = [out, st2py_flat(x.(k{1}))]; %#ok<AGROW>
    end
else
    out = double(x(:))';
end
end
'''


def driver(name, inputs, decls, init_tail):
    names = ', '.join(f"'{d['name']}'" for d in inputs)
    isbool = ', '.join('true' if d['type'] == 'BOOL' else 'false' for d in inputs)
    fixes = ''
    for d in decls:        # arrays start as a scalar in the generated init; size them
        if d['type'].startswith('ARRAY'):
            lo, hi = map(int, st2m.re.findall(r'\d+', d['type'])[:2])
            fixes += f"s.{d['name']} = zeros(1, {hi - lo + 1});\n"
    return f'''function st2py_drv_{name}(inCsv, outCsv)
E = ST_enums();
s = init_{name}(E);
{fixes}names = {{{names}}};
isbool = [{isbool}];
M = dlmread(inCsv, ',');
f0 = st2py_flat(s);
out = zeros(rows(M) + 1, numel(f0));
out(1, :) = f0;
for k = 1:rows(M)
    for j = 1:numel(names)
        if isbool(j)
            s.(names{{j}}) = M(k, j + 1) ~= 0;
        else
            s.(names{{j}}) = M(k, j + 1);
        end
    end
    s = {name}(s, E, M(k, 1));
    out(k + 1, :) = st2py_flat(s);
end
dlmwrite(outCsv, out, 'precision', '%.17g');
end
''' + init_tail


def close(a, b):
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= TOL * max(1.0, abs(b))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for tool, out in (('st2m.py', 'm'), ('st2py.py', 'py')):
            r = subprocess.run([sys.executable, str(ROOT / 'tools' / tool), str(ROOT / 'slave-src'),
                                str(tmp / out)], capture_output=True, text=True)
            assert r.returncode == 0, r.stdout + r.stderr
        sys.path.insert(0, str(tmp / 'py'))

        ctx, _ = st2m.load_project(ROOT / 'slave-src')
        init_m = (tmp / 'm' / 'ST_init.m').read_text()
        init_tail = init_m[init_m.index('function s = init_PRG_Main'):]
        (tmp / 'm' / 'st2py_flat.m').write_text(FLAT_M)

        cases, run = {}, []
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
            (tmp / 'm' / f'st2py_drv_{name}.m').write_text(driver(name, inputs, decls, init_tail))
            cases[name] = (inputs, rows)
            run.append(f"st2py_drv_{name}('{tmp / (name + '_in.csv')}', '{tmp / (name + '_out.csv')}');")

        r = subprocess.run(['octave-cli', '--quiet', '--eval',
                            f"addpath('{tmp / 'm'}', '{ROOT / 'slave-sim'}'); " + ' '.join(run)],
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stdout + r.stderr

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
                            int(v) if d['type'] in st2m.INT_TYPES or d['type'] in ctx['enums'] else v)
                assert fb(s, row[0]) is None
                got.append(flat_py(s, []))
            fields = []
            flat_py_names(s, '', fields)
            assert len(got[0]) == len(ref[0]), f'{name}: state width {len(got[0])} vs {len(ref[0])}'
            worst = 0.0
            for k, (g, m) in enumerate(zip(got, ref)):
                for j, (a, b) in enumerate(zip(g, m)):
                    assert close(a, b), (f'{name}: call {k}, {fields[j]}: python {a!r} '
                                         f'octave {b!r}')
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
