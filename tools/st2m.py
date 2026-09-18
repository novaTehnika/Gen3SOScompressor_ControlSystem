#!/usr/bin/env python3
"""Translate the slave's Structured Text into MATLAB/Octave functions.

    python3 tools/st2m.py slave-src slave-sim/generated

Covers the ST subset used in slave-src: assignments, IF/ELSIF/ELSE, FOR,
RETURN, function block calls with named inputs, typed literals, TIME
literals, one-dimensional arrays and a handful of standard functions.
Anything outside that subset is reported as an error, not guessed at.

MotionWorks IEC keeps variable declarations in its GUI, so they are read
from the "Variables (define in MWiec GUI)" header comment of each file,
from GVL/GlobalVariables_Reference.st and from DUT/DataTypes.st.

Generated code
    ST_enums.m      E = ST_enums()            enumerations
    ST_init.m       [s, G, IO] = ST_init(E)   initial PRG_Main state, globals, I/O image
    FB_<Name>.m     s = FB_<Name>(s, E, dt)   one per function block
    PRG_Main.m      [s, G] = PRG_Main(s, G, E, dt)
    PRG_Input.m     G = PRG_Input(G, IO)
    PRG_Output.m    IO = PRG_Output(G, IO)

Function block inputs, outputs and locals are fields of the instance struct;
a call assigns the named inputs and then invokes the block, so inputs that a
call leaves out keep their previous value as in IEC 61131-3. TIME values are
seconds. Each generated statement ends with the ST source line as `% :N`.
"""

import re
import sys
from pathlib import Path

INT_TYPES = {'INT', 'DINT', 'UINT', 'UDINT', 'SINT', 'USINT', 'BYTE', 'WORD'}
NUM_TYPES = INT_TYPES | {'LREAL', 'REAL'}
STD_FBS = {'TON', 'R_TRIG', 'F_TRIG'}
# Library enumerations that are not declared in DataTypes.st.
EXTERNAL_ENUMS = {
    'MC_Direction': ['Positive_Direction', 'Shortest_Way', 'Negative_Direction',
                     'Current_Direction'],
    'Y_ControlMode': ['PositionMode', 'VelocityTLMode', 'TorqueVLMode'],
}
FUNCTIONS = {'ABS': 'abs', 'SQRT': 'sqrt', 'MIN': 'min', 'MAX': 'max',
             'LIMIT': 'ST_LIMIT'}


class STError(Exception):
    pass


# --------------------------------------------------------------------------
# Declarations
# --------------------------------------------------------------------------

def strip_comments(text):
    """Blank out (* ... *) comments (non-nesting), preserving line breaks."""
    def blank(m):
        return re.sub(r'[^\n]', ' ', m.group(0))
    return re.sub(r'\(\*.*?\*\)', blank, text, flags=re.S)


def parse_datatypes(text):
    code = strip_comments(text)
    enums, structs = dict(EXTERNAL_ENUMS), {}
    for m in re.finditer(r'(\w+)\s*:\s*\(([^()]*)\)\s*;', code):
        enums[m.group(1)] = [n.strip() for n in m.group(2).split(',') if n.strip()]
    for m in re.finditer(r'(\w+)\s*:\s*STRUCT(.*?)END_STRUCT', code, flags=re.S):
        fields = re.findall(r'(\w+)\s*:\s*(\w+)\s*;', m.group(2))
        structs[m.group(1)] = fields
    return enums, structs


DECL_RE = re.compile(
    r'^\s*([A-Za-z_]\w*(?:\s*,\s*[A-Za-z_]\w*)*)\s*:\s*'
    r'(ARRAY\s*\[\s*\d+\s*\.\.\s*\d+\s*\]\s*OF\s+\w+|\w+)'
    r'(?:\s*:=\s*([^\s(]+))?')


def parse_header_decls(text, known_types):
    """Variable declarations from the header comment of a POU."""
    start = text.find('Variables (define in MWiec GUI)')
    if start < 0:
        return []
    end = text.find('====', start)
    decls, section = [], None
    for line in text[start:end].splitlines()[1:]:
        s = line.strip()
        m = re.match(r'^(VAR_INPUT|VAR_OUTPUT|VAR CONSTANT|VAR)\s*:', s)
        if m:
            section = m.group(1)
            continue
        if section is None or s.startswith('(*'):
            continue
        m = DECL_RE.match(line)
        if not m:
            continue
        typ = re.sub(r'\s+', ' ', m.group(2))
        base = typ.split(' OF ')[-1] if typ.startswith('ARRAY') else typ
        if base not in known_types:
            continue
        for name in re.split(r'\s*,\s*', m.group(1)):
            decls.append({'name': name, 'type': typ, 'init': m.group(3),
                          'section': section})
    return decls


def parse_globals(text, known_types):
    decls = {}
    for line in text.splitlines():
        m = re.match(r'^\s*(G_\w+)\s*(?:AT\s+%\S+\s*)?:\s*(\w+)\s*(?::=\s*([^\s(]+))?', line)
        if m and m.group(2) in known_types:
            decls[m.group(1)] = {'name': m.group(1), 'type': m.group(2),
                                 'init': m.group(3), 'section': 'VAR_GLOBAL'}
    return decls


# --------------------------------------------------------------------------
# Tokens
# --------------------------------------------------------------------------

TOKEN_RE = re.compile(r'''
    (?P<time>T\#[0-9A-Za-z_.]+)
  | (?P<typed>[A-Za-z_]\w*\#[-+]?[A-Za-z0-9_.]+)
  | (?P<num>\d+\.\d*(?:[eE][-+]?\d+)?|\d+)
  | (?P<id>[A-Za-z_]\w*)
  | (?P<op>:=|=>|<>|<=|>=|[-+*/=<>()\[\],;.:])
  | (?P<ws>\s+)
''', re.X)


def tokenize(code):
    tokens, pos, line = [], 0, 1
    while pos < len(code):
        m = TOKEN_RE.match(code, pos)
        if not m:
            raise STError(f'line {line}: cannot tokenize {code[pos:pos + 20]!r}')
        kind = m.lastgroup
        if kind != 'ws':
            tokens.append((kind, m.group(0), line))
        line += m.group(0).count('\n')
        pos = m.end()
    tokens.append(('eof', '', line))
    return tokens


def time_seconds(lit):
    total, body = 0.0, lit[2:].upper().replace('_', '')
    units = {'D': 86400, 'H': 3600, 'M': 60, 'S': 1, 'MS': 0.001}
    for value, unit in re.findall(r'(\d+(?:\.\d+)?)(MS|D|H|M|S)', body):
        total += float(value) * units[unit]
    return repr(total)


# --------------------------------------------------------------------------
# Parser / code generator
# --------------------------------------------------------------------------

class Translator:
    def __init__(self, pou, decls, ctx):
        self.pou = pou
        self.vars = {d['name']: d for d in decls}
        self.ctx = ctx              # shared: enums, globals, used globals, io names
        self.out = []
        self.depth = 1

    # -- token helpers --
    def load(self, code):
        self.toks, self.i = tokenize(code), 0

    def peek(self, k=0):
        return self.toks[self.i + k]

    def next(self):
        tok = self.toks[self.i]
        self.i += 1
        return tok

    def accept(self, value):
        if self.peek()[1].upper() == value and self.peek()[0] in ('id', 'op'):
            return self.next()
        return None

    def expect(self, value):
        tok = self.next()
        if tok[1].upper() != value:
            raise STError(f'{self.pou} line {tok[2]}: expected {value}, got {tok[1]!r}')
        return tok

    def emit(self, text, line=None):
        tag = f'  % :{line}' if line else ''
        self.out.append('    ' * self.depth + text + tag)

    # -- statements --
    def statements(self, terminators):
        while self.peek()[0] != 'eof' and self.peek()[1].upper() not in terminators:
            self.statement()

    def statement(self):
        kind, text, line = self.peek()
        upper = text.upper()
        if text == ';':
            self.next()
        elif upper == 'IF':
            self.next()
            cond = self.expr()[0]
            self.expect('THEN')
            self.emit(f'if {cond}', line)
            self.block(('ELSIF', 'ELSE', 'END_IF'))
            while True:
                tok = self.next()
                if tok[1].upper() == 'ELSIF':
                    cond = self.expr()[0]
                    self.expect('THEN')
                    self.emit(f'elseif {cond}', tok[2])
                    self.block(('ELSIF', 'ELSE', 'END_IF'))
                elif tok[1].upper() == 'ELSE':
                    self.emit('else', tok[2])
                    self.block(('END_IF',))
                else:
                    break
            self.emit('end')
            self.accept(';')
        elif upper == 'FOR':
            self.next()
            var = self.postfix()[0]
            self.expect(':=')
            lo = self.expr()[0]
            self.expect('TO')
            hi = self.expr()[0]
            step = None
            if self.accept('BY'):
                step = self.expr()[0]
            self.expect('DO')
            rng = f'{lo}:{step}:{hi}' if step else f'{lo}:{hi}'
            self.emit(f'for st_k_ = {rng}', line)
            self.depth += 1
            self.emit(f'{var} = st_k_;')
            self.depth -= 1
            self.block(('END_FOR',))
            self.expect('END_FOR')
            self.emit('end')
            self.accept(';')
        elif upper == 'RETURN':
            self.next()
            self.emit('return;', line)
            self.accept(';')
        elif kind == 'id' and self.peek(1)[1] == '(':
            self.fb_call()
        elif kind == 'id':
            target = self.postfix()[0]
            self.expect(':=')
            value = self.expr()[0]
            self.expect(';')
            self.emit(f'{target} = {value};', line)
        else:
            raise STError(f'{self.pou} line {line}: unexpected {text!r}')

    def block(self, terminators):
        self.depth += 1
        self.statements(terminators)
        self.depth -= 1

    def fb_call(self):
        _, name, line = self.next()
        decl = self.vars.get(name)
        if decl is None or not (decl['type'] in STD_FBS or decl['type'] in self.ctx['fbs']):
            raise STError(f'{self.pou} line {line}: {name} is not a function block instance')
        inst = f's.{name}'
        self.expect('(')
        while not self.accept(')'):
            arg = self.next()[1]
            self.expect(':=')
            value = self.expr()[0]
            self.emit(f'{inst}.{arg} = {value};', line)
            self.accept(',')
        self.expect(';')
        self.emit(f'{inst} = {decl["type"]}({inst}, E, dt);', line)
        self.ctx['called'].add(decl['type'])

    # -- expressions: (code, type) --
    def expr(self):
        return self.binary(0)

    LEVELS = [({'OR'}, '||'), ({'XOR'}, 'xor'), ({'AND', '&'}, '&&'),
              ({'=', '<>'}, None), ({'<', '>', '<=', '>='}, None),
              ({'+', '-'}, None), ({'*', '/', 'MOD'}, None)]

    def binary(self, level):
        if level == len(self.LEVELS):
            return self.unary()
        ops = self.LEVELS[level][0]
        left = self.binary(level + 1)
        while self.peek()[1].upper() in ops and self.peek()[0] in ('id', 'op'):
            op = self.next()[1].upper()
            right = self.binary(level + 1)
            left = self.combine(op, left, right)
        return left

    def combine(self, op, left, right):
        (lc, lt), (rc, rt) = left, right
        both_int = lt in INT_TYPES and rt in INT_TYPES
        if op in ('OR', 'AND', '&', 'XOR'):
            # Bitwise on integer operands, logical on BOOL operands.
            if both_int:
                fn = {'OR': 'bitor', 'XOR': 'bitxor'}.get(op, 'bitand')
                return f'{fn}({lc}, {rc})', lt
            if lt != 'BOOL' or rt != 'BOOL':
                raise STError(f'{self.pou}: {op} needs two BOOL or two integer '
                              f'operands, got {lt} and {rt}: {lc} {op} {rc}')
            if op == 'OR':
                return f'({lc} || {rc})', 'BOOL'
            if op == 'XOR':
                return f'xor({lc}, {rc})', 'BOOL'
            return f'({lc} && {rc})', 'BOOL'
        if op == '=':
            return f'({lc} == {rc})', 'BOOL'
        if op == '<>':
            return f'({lc} ~= {rc})', 'BOOL'
        if op in ('<', '>', '<=', '>='):
            return f'({lc} {op} {rc})', 'BOOL'
        if op == 'MOD':
            return f'mod({lc}, {rc})', lt
        if op == '/' and both_int:
            return f'fix({lc} / {rc})', lt
        return f'({lc} {op} {rc})', (lt if both_int else 'LREAL')

    def unary(self):
        if self.accept('NOT'):
            code, typ = self.unary()
            if typ != 'BOOL':
                raise STError(f'{self.pou}: NOT needs a BOOL operand, got {typ}: {code}')
            return f'(~{code})', 'BOOL'
        if self.accept('-'):
            code, typ = self.unary()
            return f'(-{code})', typ
        if self.accept('+'):
            return self.unary()
        return self.primary()

    def primary(self):
        kind, text, line = self.peek()
        if text == '(':
            self.next()
            code, typ = self.expr()
            self.expect(')')
            return f'({code})', typ
        if kind == 'num':
            self.next()
            return text, ('LREAL' if '.' in text or 'e' in text.lower() else 'INT')
        if kind == 'time':
            self.next()
            return time_seconds(text), 'TIME'
        if kind == 'typed':
            self.next()
            prefix, value = text.split('#', 1)
            if prefix in self.ctx['enums']:
                if value not in self.ctx['enums'][prefix]:
                    raise STError(f'{self.pou} line {line}: {value} is not in {prefix}')
                return f'E.{prefix}.{value}', prefix
            if prefix.upper() in NUM_TYPES:
                return value, prefix.upper()
            raise STError(f'{self.pou} line {line}: unknown typed literal {text}')
        if kind == 'id':
            upper = text.upper()
            if upper in ('TRUE', 'FALSE'):
                self.next()
                return upper.lower(), 'BOOL'
            if self.peek(1)[1] == '(' and text not in self.vars:
                return self.function_call()
            return self.postfix()
        raise STError(f'{self.pou} line {line}: unexpected {text!r} in expression')

    def function_call(self):
        _, name, line = self.next()
        self.expect('(')
        args = []
        while not self.accept(')'):
            args.append(self.expr())
            self.accept(',')
        codes = ', '.join(a[0] for a in args)
        upper = name.upper()
        if upper in FUNCTIONS:
            return f'{FUNCTIONS[upper]}({codes})', args[-1][1]
        if upper == 'TIME_TO_DINT':
            return f'round({codes} * 1000)', 'DINT'
        m = re.match(r'^(\w+)_TO_(\w+)$', upper)
        if m and len(args) == 1:
            to = m.group(2)
            if to in INT_TYPES and m.group(1) not in INT_TYPES:
                return f'round({codes})', to
            return f'({codes})', to
        raise STError(f'{self.pou} line {line}: unsupported function {name}')

    def postfix(self):
        _, name, line = self.next()
        code, typ = self.resolve(name, line)
        while True:
            if self.accept('.'):
                field = self.next()[1]
                code, typ = f'{code}.{field}', self.member_type(typ, field)
            elif self.accept('['):
                index = self.expr()[0]
                self.expect(']')
                code = f'{code}({index} + 1)'
                typ = typ.split(' OF ')[-1] if typ and typ.startswith('ARRAY') else None
            else:
                return code, typ

    STD_FB_MEMBERS = {'TON': {'IN': 'BOOL', 'PT': 'TIME', 'Q': 'BOOL', 'ET': 'TIME'},
                      'R_TRIG': {'CLK': 'BOOL', 'Q': 'BOOL'},
                      'F_TRIG': {'CLK': 'BOOL', 'Q': 'BOOL'}}

    def member_type(self, typ, field):
        if typ in self.STD_FB_MEMBERS:
            return self.STD_FB_MEMBERS[typ].get(field)
        if typ in self.ctx['structs']:
            return dict(self.ctx['structs'][typ]).get(field)
        if typ in self.ctx['pou_decls']:
            for d in self.ctx['pou_decls'][typ]:
                if d['name'] == field:
                    return d['type']
        return None

    def resolve(self, name, line):
        if name in self.vars:
            return f's.{name}', self.vars[name]['type']
        if name.startswith('G_'):
            if name not in self.ctx['globals']:
                self.ctx['undeclared'].add(name)
            self.ctx['used_globals'].add(name)
            return f'G.{name}', self.ctx['globals'].get(name, {}).get('type')
        if name.startswith('MO1_'):
            self.ctx['io'].add(name)
            return f'IO.{name}', None
        raise STError(f'{self.pou} line {line}: undeclared identifier {name}')

    def translate(self, text):
        self.load(strip_comments(text))
        self.statements(())
        return self.out


# --------------------------------------------------------------------------
# Initial values
# --------------------------------------------------------------------------

def init_value(decl, ctx):
    typ, init = decl['type'], decl.get('init')
    m = re.match(r'ARRAY \[\s*(\d+)\s*\.\.\s*(\d+)\s*\] OF (\w+)', typ)
    if m:
        return f'zeros(1, {int(m.group(2)) - int(m.group(1)) + 1})'
    if typ in ctx['enums']:
        member = (init or '').split('#')[-1]
        return f'E.{typ}.{member}' if member in ctx['enums'][typ] else '0'
    if typ in ctx['structs']:
        return f'init_{typ}(E)'
    if typ in ctx['fbs'] or typ in STD_FBS:
        return f'init_{typ}(E)'
    if typ == 'BOOL':
        return 'true' if (init or '').upper() == 'TRUE' else 'false'
    if typ == 'TIME':
        return time_seconds(init) if init else '0'
    if init:
        return init.split('#')[-1]
    return '0'


def gen_init(ctx, main_decls):
    lines = ['function [s, G, IO] = ST_init(E)',
             '%ST_INIT Initial PRG_Main state, global variables and I/O image.',
             '%   Generated by tools/st2m.py - do not edit.', '',
             's = init_PRG_Main(E);', '', 'G = struct();']
    for name in sorted(ctx['globals']):
        lines.append(f'G.{name} = {init_value(ctx["globals"][name], ctx)};')
    for name in sorted(ctx['undeclared']):
        lines.append(f'G.{name} = 0;    % not declared in GlobalVariables_Reference.st')
    lines += ['', 'IO = struct();']
    for name in sorted(ctx['io']):
        lines.append(f'IO.{name} = {"0" if "_A" in name else "false"};')
    lines += ['end', '']

    def struct_fn(fname, decls):
        body = [f'function s = init_{fname}(E) %#ok<INUSD>', 's = struct();']
        body += [f's.{d["name"]} = {init_value(d, ctx)};' for d in decls]
        return body + ['end', '']

    lines += struct_fn('PRG_Main', main_decls)
    for fb in sorted(ctx['fbs']):
        lines += struct_fn(fb, ctx['pou_decls'][fb])
    for st, fields in sorted(ctx['structs'].items()):
        lines += struct_fn(st, [{'name': n, 'type': t} for n, t in fields])
    lines += ['function s = init_TON(E) %#ok<INUSD>',
              's = struct(\'IN\', false, \'PT\', 0, \'Q\', false, \'ET\', 0, \'M\', false);',
              'end', '',
              'function s = init_R_TRIG(E) %#ok<INUSD>',
              's = struct(\'CLK\', false, \'Q\', false, \'M\', false);', 'end', '',
              'function s = init_F_TRIG(E) %#ok<INUSD>',
              's = struct(\'CLK\', false, \'Q\', false, \'M\', true);', 'end']
    return lines


def gen_enums(ctx):
    lines = ['function E = ST_enums()',
             '%ST_ENUMS Enumerations from DataTypes.st and the motion library.',
             '%   Generated by tools/st2m.py - do not edit.', '']
    for enum, members in sorted(ctx['enums'].items()):
        for value, member in enumerate(members):
            lines.append(f'E.{enum}.{member} = {value};')
    return lines + ['end']


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

def main(src, out):
    src, out = Path(src), Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob('*.m'):
        old.unlink()

    enums, structs = parse_datatypes((src / 'DUT/DataTypes.st').read_text())
    fb_files = sorted((src / 'FB').glob('FB_*.st'))
    fbs = {f.stem for f in fb_files}
    known = (NUM_TYPES | {'BOOL', 'TIME'} | STD_FBS | fbs | set(enums) | set(structs))

    ctx = {'enums': enums, 'structs': structs, 'fbs': fbs, 'io': set(),
           'globals': parse_globals((src / 'GVL/GlobalVariables_Reference.st').read_text(), known),
           'used_globals': set(), 'undeclared': set(), 'called': set(), 'pou_decls': {}}

    sources = {f.stem: f.read_text() for f in fb_files}
    for prg in ('PRG_Main', 'PRG_Input', 'PRG_Output'):
        sources[prg] = (src / 'PRG' / f'{prg}.st').read_text()
    for name, text in sources.items():
        ctx['pou_decls'][name] = parse_header_decls(text, known)

    signatures = {'PRG_Main': '[s, G] = PRG_Main(s, G, E, dt)',
                  'PRG_Input': 'G = PRG_Input(G, IO)',
                  'PRG_Output': 'IO = PRG_Output(G, IO)'}
    errors = []
    for name, text in sources.items():
        tr = Translator(name, ctx['pou_decls'][name], ctx)
        try:
            body = tr.translate(text)
        except STError as err:
            errors.append(str(err))
            continue
        sig = signatures.get(name, f's = {name}(s, E, dt)')
        head = [f'function {sig} %#ok<INUSD>',
                f'%{name.upper()} Generated from slave-src by tools/st2m.py - do not edit.', '']
        (out / f'{name}.m').write_text('\n'.join(head + body + ['end']) + '\n')

    (out / 'ST_enums.m').write_text('\n'.join(gen_enums(ctx)) + '\n')
    (out / 'ST_init.m').write_text('\n'.join(gen_init(ctx, ctx['pou_decls']['PRG_Main'])) + '\n')

    for name in sorted(ctx['undeclared']):
        print(f'warning: {name} is used but not declared in GlobalVariables_Reference.st')
    for err in errors:
        print(f'error: {err}')
    print(f'{len(sources) - len(errors)} of {len(sources)} POUs translated to {out}')
    return 1 if errors else 0


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1], sys.argv[2]))
