"""
slave.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module gives the design studies access to the slave's function
blocks and configuration, so that the studies run the slave's own code
rather than a copy of it.

Importing the module does three things. It regenerates design/generated
with tools/st2py.py when that directory is missing or older than the
slave sources, it puts design/generated on sys.path so that statements
such as "from FB_PressureController import ..." work, and it puts tools/
on sys.path so that this module can use the Structured Text parser in
st2m.py.

slave_config() returns the default value of every G_cfg* setting in the
slave's global variable list (GVL), slave-src/GVL/
GlobalVariables_Reference.st. TIME values are converted to seconds;
other values keep the units given in the GVL comments.

FILE DEPENDENCY:
../../tools/
    st2py.py (run as a program to regenerate the blocks)
    st2m.py
../../slave-src/
    FB/FB_*.st
    DUT/DataTypes.st
    GVL/GlobalVariables_Reference.st
"""

import subprocess
import sys
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]      # repository root
SLAVE_SRC = ROOT / 'slave-src'
GENERATED = ROOT / 'design' / 'generated'
TOOLS = ROOT / 'tools'


# %% ===== GENERATED FUNCTION BLOCKS =====================================
def _sources():
    """Return the files that design/generated is translated from."""
    files = list((SLAVE_SRC / 'FB').glob('FB_*.st'))
    files.append(SLAVE_SRC / 'DUT' / 'DataTypes.st')
    files += [TOOLS / 'st2py.py', TOOLS / 'st2m.py']
    return files


def ensure_generated():
    """Regenerate design/generated unless every generated file is newer
    than every source file."""
    out = list(GENERATED.glob('*.py'))
    missing = (not out
               or not (GENERATED / 'FB_PressureController.py').exists())
    stale = (missing
             or min(f.stat().st_mtime for f in out)
             < max(f.stat().st_mtime for f in _sources()))
    if stale:
        subprocess.run([sys.executable, str(TOOLS / 'st2py.py'),
                        str(SLAVE_SRC), str(GENERATED)],
                       check=True, cwd=ROOT)


ensure_generated()
if str(GENERATED) not in sys.path:
    sys.path.insert(0, str(GENERATED))
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))


import st2m  # noqa: E402  (tools/ is on the path from above)


# %% ===== SLAVE CONFIGURATION ===========================================
def _literal(typ, init):
    """Convert the initial value text init of a GVL variable of type typ
    to a Python value, or return None if it is not a plain literal."""
    if init is None:
        return None
    if typ == 'BOOL':
        return init.upper() == 'TRUE'
    if typ == 'TIME':
        return float(st2m.time_seconds(init))           # [s]
    try:
        # A typed literal such as REAL#0.5 keeps its value after the '#'
        value = float(init.split('#')[-1])
    except ValueError:
        return None
    return int(value) if typ in st2m.INT_TYPES else value


@lru_cache(maxsize=1)
def _defaults():
    """Return the G_cfg* defaults of the GVL as a dict. The result is
    cached because parsing the slave project takes time."""
    ctx, _ = st2m.load_project(SLAVE_SRC)
    cfg = {}
    for name, decl in ctx['globals'].items():
        if name.startswith('G_cfg'):
            value = _literal(decl['type'], decl['init'])
            if value is not None:
                cfg[name] = value
    return cfg


def slave_config(**overrides):
    """Return the G_cfg* defaults of the GVL as a dict, with any
    overrides applied.

    TIME values are in seconds; other values are in the units the GVL
    comments give. An override for a name the GVL does not define raises
    KeyError.
    """
    cfg = dict(_defaults())
    for key, value in overrides.items():
        if key not in cfg:
            raise KeyError(f'unknown slave configuration {key}')
        cfg[key] = value
    return cfg
