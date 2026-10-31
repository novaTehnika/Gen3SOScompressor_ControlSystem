"""Python target of tools/st_translate.py: the function blocks as modules.

    python3 tools/st_translate.py --target py slave-src models/generated

The ST subset and the parsing are those of tools/st_translate.py; this
target subclasses its Translator and overrides only the target-language
spellings. Programs (PRG_*) are not translated.

Generated code (flat imports: put the output directory on sys.path)
    st_runtime.py       E (enumerations), TON, R_TRIG, F_TRIG, st_* helpers,
                        new_<Struct>() for the DataTypes.st structures
    FB_<Name>.py        new_FB_<Name>() -> instance, FB_<Name>(s, dt)
    __init__.py

    s = new_FB_PressureController()   # SimpleNamespace, every VAR_INPUT,
    s.Enable = True                   # VAR_OUTPUT and VAR at its declared
    FB_PressureController(s, dt)      # default; nested blocks are namespaces

A call mutates the instance in place and returns None; dt is the scan time in
seconds and drives TON. As in IEC 61131-3, inputs that a call leaves out keep
their previous value. TIME values are seconds, ST arrays are 0-based lists,
and each generated statement ends with the ST source line as `# :N`.
Function blocks that read G_* globals or I/O points are rejected: the
translated blocks are the ones that are self-contained.
"""

import re
import sys
import copy
from pathlib import Path

from st_translate import (STError, STD_FBS, Translator, OctaveInit, init_value,
                          load_project)

HEADER = '# Generated from slave-src by tools/st_translate.py --target py - do not edit.'

RUNTIME = '''\
import math
from types import SimpleNamespace

_INF, _NAN = math.inf, math.nan


def st_min(a, b):
    return b if (a != a or b < a) else a


def st_max(a, b):
    return b if (a != a or b > a) else a


def st_limit(mn, x, mx):
    """IEC LIMIT(MN, IN, MX)."""
    return st_min(st_max(x, mn), mx)


def st_sqrt(x):
    return math.sqrt(x) if x >= 0 else (x if x != x else _NAN)


def st_round(x):
    """Round half away from zero."""
    if x != x or x in (_INF, -_INF):
        return x
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(0.5 - x))


def st_div(a, b):
    """Real division with IEEE results for a zero divisor."""
    if b:
        return a / b
    if a != a or not a:
        return _NAN
    return math.copysign(_INF, a) * math.copysign(1.0, b)


def st_idiv(a, b):
    """Integer division truncating toward zero."""
    q = st_div(a, b)
    return q if q != q or q in (_INF, -_INF) else int(q)


def st_mod(a, b):
    return a % b if b else a


def st_bitor(a, b):
    return int(a) | int(b)


def st_bitand(a, b):
    return int(a) & int(b)


def st_bitxor(a, b):
    return int(a) ^ int(b)


def st_range(lo, hi, step=1):
    """FOR lo TO hi BY step, bounds inclusive."""
    return range(lo, hi + (1 if step > 0 else -1), step)


def new_TON():
    return SimpleNamespace(IN=False, PT=0.0, Q=False, ET=0.0, M=False)


def TON(s, dt):
    """IEC 61131-3 on-delay timer: Q is set once IN has been TRUE for PT
    seconds; a call with IN FALSE resets it."""
    if not s.IN:
        s.ET = 0.0
        s.Q = False
    else:
        s.ET = s.ET + dt if s.M else 0.0
        s.Q = s.ET >= s.PT
    s.M = s.IN


def new_R_TRIG():
    return SimpleNamespace(CLK=False, Q=False, M=False)


def R_TRIG(s, dt):
    """IEC 61131-3 rising-edge detector."""
    s.Q = bool(s.CLK) and not s.M
    s.M = s.CLK


def new_F_TRIG():
    return SimpleNamespace(CLK=False, Q=False, M=True)


def F_TRIG(s, dt):
    """IEC 61131-3 falling-edge detector."""
    s.Q = not s.CLK and not s.M
    s.M = not s.CLK
'''


class PyInit(OctaveInit):
    true, false = 'True', 'False'

    def array(self, elem, n):
        zero = {'BOOL': 'False', 'LREAL': '0.0', 'REAL': '0.0'}.get(elem, '0')
        return f'[{zero}] * {n}'

    def instance(self, typ):
        return f'new_{typ}()'

    def number(self, typ, text):
        m = re.match(r'ARRAY\s*\[\s*(\d+)\s*\.\.\s*(\d+)\s*\] OF (\w+)', typ)
        if m:     # array types are spelled without a space after ARRAY in the headers
            if int(m.group(1)) != 0:
                raise STError(f'array {typ} must start at 0')
            return self.array(m.group(3), int(m.group(2)) - int(m.group(1)) + 1)
        return f'float({text})' if typ in ('LREAL', 'REAL') else text


class PyTranslator(Translator):
    COMMENT = '#'
    EOS = ''
    FUNCTIONS = {'ABS': 'abs', 'SQRT': 'st_sqrt', 'MIN': 'st_min', 'MAX': 'st_max',
                 'LIMIT': 'st_limit'}
    OPS = {'OR': 'or', 'AND': 'and', '&': 'and', '=': '==', '<>': '!='}

    def __init__(self, *args):
        super().__init__(*args)
        self.calls = set()          # function block types this POU calls

    def open_if(self, cond, line):
        self.emit(f'if {cond}:', line)

    def open_elif(self, cond, line):
        self.emit(f'elif {cond}:', line)

    def open_else(self, line):
        self.emit('else:', line)

    def close_block(self):
        pass

    def empty_block(self):
        self.emit('pass')

    def open_for(self, var, lo, hi, step, line):
        rng = f'{lo}, {hi}, {step}' if step else f'{lo}, {hi}'
        self.emit(f'for st_k_ in st_range({rng}):', line)
        self.depth += 1
        self.emit(f'{var} = st_k_')
        self.depth -= 1

    def emit_fb_call(self, inst, typ, line):
        self.calls.add(typ)
        self.emit(f'{typ}({inst}, dt)', line)

    def copy_value(self, code, typ):
        if typ in self.ctx['structs']:
            return f'copy.deepcopy({code})'
        if typ and typ.startswith('ARRAY'):
            return f'list({code})'
        return code

    def bitwise(self, op, lc, rc):
        return f'st_bit{ {"OR": "or", "XOR": "xor"}.get(op, "and") }({lc}, {rc})'

    def logical_xor(self, lc, rc):
        return f'(bool({lc}) != bool({rc}))'

    def logical_not(self, code):
        return f'(not {code})'

    def modulo(self, lc, rc):
        return f'st_mod({lc}, {rc})'

    def divide(self, lc, rc, both_int):
        if both_int:
            return f'st_idiv({lc}, {rc})'
        if re.fullmatch(r'[0-9.]*[1-9][0-9.]*(?:[eE][-+]?\d+)?', rc):
            return f'({lc} / {rc})'         # nonzero literal divisor
        return f'st_div({lc}, {rc})'

    def bool_literal(self, value):
        return 'True' if value else 'False'

    def index(self, code, idx):
        return f'{code}[{idx}]'

    def to_int(self, code):
        return f'st_round({code})'

    def resolve(self, name, line):
        if name not in self.vars and name.startswith(('G_', 'MO1_')):
            raise STError(f'{self.pou} line {line}: {name} is a global or I/O '
                          'point; the py target translates only self-contained blocks')
        return super().resolve(name, line)


def used_helpers(body):
    names = set(re.findall(r'\b(?:st_(?!k_)\w+|TON|R_TRIG|F_TRIG)\b', body))
    return sorted(names)


def gen_runtime(ctx):
    spell = PyInit()
    lines = [HEADER, '"""Runtime support for the translated function blocks."""',
             RUNTIME, '', '# Enumerations from DataTypes.st and the motion library.',
             'E = SimpleNamespace(']
    for enum, members in sorted(ctx['enums'].items()):
        items = ', '.join(f'{m}={v}' for v, m in enumerate(members))
        lines.append(f'    {enum}=SimpleNamespace({items}),')
    lines += [')', '']
    for name, fields in sorted(ctx['structs'].items()):
        lines += ['', '', f'def new_{name}():', '    s = SimpleNamespace()']
        lines += [f'    s.{n} = {init_value({"name": n, "type": t}, ctx, spell)}'
                  for n, t in fields]
        lines += ['    return s']
    return lines


def gen_module(name, decls, body, calls, ctx):
    spell = PyInit()
    text = '\n'.join(body)
    nested = sorted({d['type'] for d in decls if d['type'] in ctx['fbs']} | (calls & ctx['fbs']))
    structs = sorted({re.sub(r'.*\bOF ', '', d['type']) for d in decls} & set(ctx['structs']))
    std = sorted({d['type'] for d in decls} & STD_FBS)
    rt = sorted(set(used_helpers(text)) | {f'new_{t}' for t in std}
                | {f'new_{t}' for t in structs} | (calls & STD_FBS))
    if 'E.' in text or any(d['type'] in ctx['enums'] for d in decls):
        rt.append('E')
    lines = [HEADER] + (['import copy'] if 'copy.' in text else [])
    lines.append('from types import SimpleNamespace')
    if rt:
        lines.append(f'from st_runtime import {", ".join(rt)}')
    for fb in nested:
        lines.append(f'from {fb} import {fb}, new_{fb}')
    lines += ['', '', f'def new_{name}():', '    s = SimpleNamespace()']
    lines += [f'    s.{d["name"]} = {init_value(d, ctx, spell)}' for d in decls]
    lines += ['    return s', '', '', f'def {name}(s, dt):'] + (body or ['    pass'])
    return lines


def generate(src, out):
    src, out = Path(src), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob('*.py'):
        old.unlink()
    ctx, sources = load_project(src)
    # Parsed but unused here: the programs need the globals and the I/O image.
    blocks = {n: t for n, t in sources.items() if n in ctx['fbs']}

    errors = []
    for name, text in blocks.items():
        tr = PyTranslator(name, ctx['pou_decls'][name], ctx)
        try:
            body = tr.translate(text)
        except STError as err:
            errors.append(str(err))
            continue
        lines = gen_module(name, ctx['pou_decls'][name], body, tr.calls, ctx)
        (out / f'{name}.py').write_text('\n'.join(lines) + '\n')

    (out / 'st_runtime.py').write_text('\n'.join(gen_runtime(ctx)) + '\n')
    (out / '__init__.py').write_text(
        f'{HEADER}\n# Modules import each other by flat name: put this directory on sys.path.\n')

    for err in errors:
        print(f'error: {err}')
    print(f'{len(blocks) - len(errors)} of {len(blocks)} function blocks translated to {out}')
    return 1 if errors else 0

