"""C target of tools/st_translate.py: the slave simulator, compiled.

    python3 tools/st_translate.py --target c slave-src slave-sim/generated-c

Generated code
    st_slave.h   enumeration constants (<Enum>__<Member>), one struct type
                 per DataTypes.st structure, function block and program
                 (<Name>_t), G_t (the globals), IO_t (the I/O image), and
                 the prototypes
    st_slave.c   one function per POU and an init_<Name>() per type:
                     void FB_<Name>(FB_<Name>_t *s, double dt)
                     void PRG_<Name>(PRG_<Name>_t *s, G_t *G, IO_t *IO, double dt)
    st_names.c   st_names[]: every variable of G, IO and the PRG_Main
                 instance (nested blocks included) by path, type and
                 offset; st_enums[]: every enumeration member; and
                 st_holding_regs[] / st_input_regs[]: the Modbus
                 registers, from the globals' AT addresses (MODBUS_GROUPS)

The runtime it relies on (TON, R_TRIG, F_TRIG, st_* helpers) is the
hand-written slave-sim/c/st_runtime.h. Variables keep the controller's
types: BOOL is bool, INT int16_t, UINT uint16_t, UDINT uint32_t, LREAL
double and REAL float. TIME is double seconds and enumerations are
int32_t, as in the py target. Function block instances are structs passed by
pointer; inputs that a call leaves out keep their previous value. Each
statement ends with its ST source line as `// :N`.
"""

import re
from pathlib import Path

from st_translate import (INT_WIDTH, STD_FBS, STError, Translator, OctaveInit,
                          init_value, load_project)

TOOL = 'tools/st_translate.py --target c'
HEADER = f'/* Generated from slave-src by {TOOL} - do not edit. */'

SCALARS = {'BOOL': 'bool', 'SINT': 'int8_t', 'USINT': 'uint8_t', 'BYTE': 'uint8_t',
           'INT': 'int16_t', 'UINT': 'uint16_t', 'WORD': 'uint16_t',
           'DINT': 'int32_t', 'UDINT': 'uint32_t', 'DWORD': 'uint32_t', 'LREAL': 'double', 'REAL': 'float',
           'TIME': 'double'}
# Type codes of st_names[], shared with slave-sim/c/st_names.h.
TYPE_CODES = {'bool': 'ST_BOOL', 'int8_t': 'ST_I8', 'uint8_t': 'ST_U8', 'int16_t': 'ST_I16',
              'uint16_t': 'ST_U16', 'int32_t': 'ST_I32', 'uint32_t': 'ST_U32',
              'float': 'ST_F32', 'double': 'ST_F64'}
STD_FB_FIELDS = {'TON': [('IN', 'BOOL'), ('PT', 'TIME'), ('Q', 'BOOL'), ('ET', 'TIME'),
                         ('M', 'BOOL')],
                 'R_TRIG': [('CLK', 'BOOL'), ('Q', 'BOOL'), ('M', 'BOOL')],
                 'F_TRIG': [('CLK', 'BOOL'), ('Q', 'BOOL'), ('M', 'BOOL')]}
# The Modbus register groups of the controller's Hardware Configuration:
# register k is the word at byte base + 2 (k - 1) of the group's area
# (docs/master/ModbusRegisterMap.md, "Addressing"). A 32-bit value spans
# two registers, low word first.
MODBUS_GROUPS = {'holding': (('MW3.', 'MD3.'), 483328),
                 'input': (('QW', 'QD'), 73792)}
REG_TYPES = {'UINT': ('ST_U16', 1), 'WORD': ('ST_U16', 1), 'INT': ('ST_I16', 1),
             'UDINT': ('ST_U32', 2), 'DWORD': ('ST_U32', 2), 'DINT': ('ST_I32', 2),
             'REAL': ('ST_F32', 2)}
ARRAY_RE = re.compile(r'ARRAY\s*\[\s*(\d+)\s*\.\.\s*(\d+)\s*\]\s*OF\s+(\w+)')


class CInit(OctaveInit):
    true, false = 'true', 'false'


class CTranslator(Translator):
    COMMENT = '//'
    FUNCTIONS = {'ABS': 'st_abs', 'SQRT': 'st_sqrt', 'MIN': 'st_min', 'MAX': 'st_max',
                 'LIMIT': 'st_limit'}
    OPS = {'OR': '||', 'AND': '&&', '&': '&&', '=': '==', '<>': '!='}

    def open_if(self, cond, line):
        self.emit(f'if ({cond}) {{', line)

    def open_elif(self, cond, line):
        self.emit(f'}} else if ({cond}) {{', line)

    def open_else(self, line):
        self.emit('} else {', line)

    def close_block(self):
        self.emit('}')

    def open_for(self, var, lo, hi, step, line):
        # The bounds are evaluated once, and the loop counter is the
        # block's own, as in the py target.
        if step is None:
            self.emit(f'for (long st_k_ = {lo}, st_hi_ = {hi}; st_k_ <= st_hi_; st_k_++) {{',
                      line)
        else:
            self.emit(f'for (long st_k_ = {lo}, st_hi_ = {hi}, st_by_ = {step}; '
                      f'st_by_ > 0 ? st_k_ <= st_hi_ : st_k_ >= st_hi_; st_k_ += st_by_) {{',
                      line)
        self.depth += 1
        self.emit(f'{var} = st_k_;')
        self.depth -= 1

    def emit_fb_call(self, inst, typ, line):
        self.emit(f'{typ}(&{inst}, dt);', line)

    def copy_value(self, code, typ):
        if typ and typ.startswith('ARRAY'):
            raise STError(f'{self.pou}: array assignment is not supported: {code}')
        return code

    def instance_ref(self, name):
        return f's->{name}'

    def enum_literal(self, typ, member):
        return f'{typ}__{member}'

    def bitwise(self, op, lc, rc):
        return f'({lc} { {"OR": "|", "XOR": "^"}.get(op, "&") } {rc})'

    def logical_xor(self, lc, rc):
        return f'(!({lc}) != !({rc}))'

    def logical_not(self, code):
        return f'(!{code})'

    def modulo(self, lc, rc):
        return f'st_mod({lc}, {rc})'

    def divide(self, lc, rc, both_int):
        return f'st_idiv({lc}, {rc})' if both_int else f'((double)({lc}) / ({rc}))'

    def index(self, code, idx):
        # [an index computed in double (MOD, mixed arithmetic) to integer]
        return f'{code}[(long)({idx})]'

    def to_int(self, code):
        return f'st_round({code})'

    def wrap(self, code, bits, signed):
        # [two's complement wrap of an integer to the target width]
        if signed:
            return f'(st_mod({code} + {2 ** (bits - 1)}, {2 ** bits}) - {2 ** (bits - 1)})'
        return f'st_mod({code}, {2 ** bits})'

    def shift(self, op, code, n, typ):
        if op == 'SHR':
            return f'st_shr({code}, {n})'
        bits, signed = INT_WIDTH[typ]
        return self.wrap(f'st_shl({code}, {n})', bits, signed)

    def resolve(self, name, line):
        code, typ = super().resolve(name, line)
        if code.startswith('s.'):
            return 's->' + code[2:], typ
        if code.startswith(('G.', 'IO.')):
            if self.pou not in PROGRAMS:
                raise STError(f'{self.pou} line {line}: {name} is a global or I/O point, '
                              'which only programs may use')
            root, rest = code.split('.', 1)
            return f'{root}->{rest}', typ
        return code, typ


PROGRAMS = ('PRG_Input', 'PRG_Main', 'PRG_Output')


# --------------------------------------------------------------------------
# Types
# --------------------------------------------------------------------------

def c_type(typ, ctx):
    """C type of an ST type, and the array length (None for scalars)."""
    m = ARRAY_RE.match(typ)
    if m:
        if int(m.group(1)) != 0:
            raise STError(f'array {typ} must start at 0')
        return c_type(m.group(3), ctx)[0], int(m.group(2)) + 1
    if typ in SCALARS:
        return SCALARS[typ], None
    if typ in ctx['enums']:
        return 'int32_t', None
    if typ in ctx['structs'] or typ in ctx['fbs'] or typ in STD_FBS:
        return f'{typ}_t', None
    raise STError(f'no C type for {typ}')


def fields_of(typ, ctx):
    """(name, ST type) of every field of a structure or block type."""
    if typ in STD_FB_FIELDS:
        return STD_FB_FIELDS[typ]
    if typ in ctx['structs']:
        return list(ctx['structs'][typ])
    return [(d['name'], d['type']) for d in ctx['pou_decls'][typ]]


def struct_decl(name, fields, ctx):
    lines = [f'typedef struct {name}_t {{']
    for fname, ftyp in fields:
        ct, n = c_type(ftyp, ctx)
        lines.append(f'    {ct} {fname}{f"[{n}]" if n else ""};')
    if not fields:
        lines.append('    char unused_;')
    return lines + [f'}} {name}_t;', '']


def type_order(ctx):
    """Structure and block types, each after the types of its fields."""
    order, seen = [], set()

    def visit(t):
        if t in seen:
            return
        seen.add(t)
        for _, ft in fields_of(t, ctx):
            base = ARRAY_RE.match(ft).group(3) if ARRAY_RE.match(ft) else ft
            if base in ctx['structs'] or base in ctx['fbs'] or base in STD_FBS:
                visit(base)
        order.append(t)

    for t in sorted(STD_FBS) + sorted(ctx['structs']) + sorted(ctx['fbs']) + list(PROGRAMS):
        visit(t)
    return order


def global_fields(ctx):
    fields = [(n, ctx['globals'][n]['type']) for n in sorted(ctx['globals'])]
    return fields + [(n, 'LREAL') for n in sorted(ctx['undeclared'])]


def io_fields(ctx):
    return [(n, 'LREAL' if '_A' in n else 'BOOL') for n in sorted(ctx['io'])]


# --------------------------------------------------------------------------
# Files
# --------------------------------------------------------------------------

def gen_header(ctx):
    lines = [HEADER, '#ifndef ST_SLAVE_H', '#define ST_SLAVE_H', '',
             '#include <stdbool.h>', '#include <stdint.h>', '']
    for enum, members in sorted(ctx['enums'].items()):
        for value, member in enumerate(members):
            lines.append(f'#define {enum}__{member} {value}')
    lines.append('')
    for t in type_order(ctx):
        lines += struct_decl(t, fields_of(t, ctx), ctx)
    lines += struct_decl('G', global_fields(ctx), ctx)
    lines += struct_decl('IO', io_fields(ctx), ctx)
    for t in sorted(ctx['fbs']):
        lines.append(f'void {t}({t}_t *s, double dt);')
    for p in PROGRAMS:
        lines.append(f'void {p}({p}_t *s, G_t *G, IO_t *IO, double dt);')
    for t in type_order(ctx):
        lines.append(f'void init_{t}({t}_t *s);')
    lines += ['void init_G(G_t *G);', 'void init_IO(IO_t *IO);', '', '#endif', '']
    return lines


def init_body(target, fields, ctx):
    spell = CInit()
    lines = [f'    memset({target}, 0, sizeof *{target});']
    for fname, ftyp in fields:
        if ARRAY_RE.match(ftyp):
            continue                    # zero
        if ftyp in ctx['structs'] or ftyp in ctx['fbs'] or ftyp in STD_FBS:
            lines.append(f'    init_{ftyp}(&{target}->{fname});')
            continue
        value = init_value({'name': fname, 'type': ftyp, 'init': None}, ctx, spell)
        lines.append(f'    {target}->{fname} = {value};')
    return lines


def gen_source(ctx, bodies, decl_inits):
    lines = [HEADER, '#include <math.h>', '#include <string.h>', '#include "st_slave.h"',
             '#include "st_runtime.h"', '']
    for name in sorted(ctx['fbs']) + list(PROGRAMS):
        sig = (f'void {name}({name}_t *s, G_t *G, IO_t *IO, double dt)' if name in PROGRAMS
               else f'void {name}({name}_t *s, double dt)')
        lines += [sig, '{', '    (void)s; (void)dt;'
                  + (' (void)G; (void)IO;' if name in PROGRAMS else '')]
        lines += bodies[name] + ['}', '']
    for t in type_order(ctx):
        lines += [f'void init_{t}({t}_t *s)', '{'] + decl_inits(t) + ['}', '']
    lines += ['void init_G(G_t *G)', '{'] + init_body_vals('G', global_fields(ctx), ctx)
    lines += ['}', '', 'void init_IO(IO_t *IO)', '{']
    lines += init_body_vals('IO', io_fields(ctx), ctx) + ['}', '']
    return lines


def init_body_vals(target, fields, ctx):
    """Like init_body, with the declared initial values of globals."""
    spell = CInit()
    lines = [f'    memset({target}, 0, sizeof *{target});']
    for fname, ftyp in fields:
        decl = ctx['globals'].get(fname, {'name': fname, 'type': ftyp, 'init': None})
        if ftyp in ctx['structs']:
            lines.append(f'    init_{ftyp}(&{target}->{fname});')
        elif not ARRAY_RE.match(ftyp):
            lines.append(f'    {target}->{fname} = {init_value(decl, ctx, spell)};')
    return lines


def name_entries(prefix, root, cpath, typ, ctx, out):
    """st_names[] rows for variable cpath of root (G, IO or S) and its fields."""
    m = ARRAY_RE.match(typ)
    base = m.group(3) if m else typ
    if not m and (base in ctx['structs'] or base in ctx['fbs'] or base in STD_FBS):
        for fname, ftyp in fields_of(base, ctx):
            name_entries(f'{prefix}.{fname}', root, f'{cpath}.{fname}', ftyp, ctx, out)
        return
    ct, n = c_type(typ, ctx)
    rtype = {'G': 'G_t', 'IO': 'IO_t', 'S': 'PRG_Main_t'}[root]
    out.append(f'    {{"{prefix}", ST_ROOT_{root}, {TYPE_CODES[ct]}, '
               f'offsetof({rtype}, {cpath}), {n or 1}}},')


def register_rows(ctx):
    """st_reg_t rows of each Modbus group, from the globals' AT addresses."""
    rows = {g: [] for g in MODBUS_GROUPS}
    for name in sorted(ctx['globals']):
        at = ctx['globals'][name].get('at')
        if not at:
            continue
        for group, (prefixes, base) in MODBUS_GROUPS.items():
            prefix = next((pf for pf in prefixes if at.startswith(pf)), None)
            if prefix is None:
                continue
            typ = ctx['globals'][name]['type']
            m = ARRAY_RE.match(typ)
            elem, count = (m.group(3), int(m.group(2)) - int(m.group(1)) + 1) if m else (typ, 1)
            if elem not in REG_TYPES:
                raise STError(f'{name} AT %{at}: no Modbus register form for {typ}')
            code, words = REG_TYPES[elem]
            if (prefix[1] == 'D') != (words == 2):
                raise STError(f'{name} AT %{at}: a {elem} needs a {"D" if words == 2 else "W"} address')
            offset = int(at[len(prefix):]) - base
            if offset < 0 or offset % 2:
                raise STError(f'{name} AT %{at}: not a register of the {group} group')
            rows[group].append((offset // 2 + 1, words * count, code, name, count))
    for group, regs in rows.items():
        regs.sort()
        for a, b in zip(regs, regs[1:]):
            if a[0] + a[1] > b[0]:
                raise STError(f'{a[3]} and {b[3]} overlap in the {group} registers')
    return rows


def gen_names(ctx):
    rows = []
    for fname, ftyp in global_fields(ctx):
        name_entries(f'G.{fname}', 'G', fname, ftyp, ctx, rows)
    for fname, ftyp in io_fields(ctx):
        name_entries(f'IO.{fname}', 'IO', fname, ftyp, ctx, rows)
    for fname, ftyp in fields_of('PRG_Main', ctx):
        name_entries(f's.{fname}', 'S', fname, ftyp, ctx, rows)
    enums = [f'    {{"{e}.{m}", {v}}},' for e, ms in sorted(ctx['enums'].items())
             for v, m in enumerate(ms)]
    regs = register_rows(ctx)
    reg_lines = []
    for group, entries in regs.items():
        reg_lines += [f'const st_reg_t st_{group}_regs[] = {{']
        reg_lines += [f'    {{"G.{n}", {first}, {words}, {code}, offsetof(G_t, {n}), {count}}},'
                      for first, words, code, n, count in entries]
        reg_lines += ['};', f'const int st_{group}_regs_count = {len(entries)};', '']
    return [HEADER, '#include <stddef.h>', '#include "st_slave.h"', '#include "st_names.h"', '',
            'const st_name_t st_names[] = {'] + rows + ['};',
            f'const int st_names_count = {len(rows)};', '',
            'const st_enum_t st_enums[] = {'] + enums + ['};',
            f'const int st_enums_count = {len(enums)};', ''] + reg_lines


def generate(src, out):
    src, out = Path(src), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    ctx, sources = load_project(src)

    errors, bodies = [], {}
    for name, text in sources.items():
        tr = CTranslator(name, ctx['pou_decls'][name], ctx)
        try:
            bodies[name] = tr.translate(text)
        except STError as err:
            errors.append(str(err))
    for err in errors:
        print(f'error: {err}')
    if errors:
        return 1

    def decl_inits(t):
        if t in STD_FB_FIELDS:
            return ['    memset(s, 0, sizeof *s);'] + (['    s->M = true;'] if t == 'F_TRIG' else [])
        if t in ctx['structs']:
            return init_body('s', fields_of(t, ctx), ctx)
        spell = CInit()
        lines = ['    memset(s, 0, sizeof *s);']
        for d in ctx['pou_decls'][t]:
            typ = d['type']
            if ARRAY_RE.match(typ):
                continue
            if typ in ctx['structs'] or typ in ctx['fbs'] or typ in STD_FBS:
                lines.append(f'    init_{typ}(&s->{d["name"]});')
            else:
                lines.append(f'    s->{d["name"]} = {init_value(d, ctx, spell)};')
        return lines

    files = {'st_slave.h': gen_header(ctx),
             'st_slave.c': gen_source(ctx, bodies, decl_inits),
             'st_names.c': gen_names(ctx)}
    for fname, lines in files.items():
        (out / fname).write_text('\n'.join(lines) + '\n')
    for name in sorted(ctx['undeclared']):
        print(f'warning: {name} is used but not declared in GlobalVariables_Reference.st')
    print(f'{len(sources)} of {len(sources)} POUs translated to {out}')
    return 0
