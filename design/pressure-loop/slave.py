"""Access to the slave's translated function blocks and configuration.

Importing this module puts design/generated on sys.path, after regenerating
it with tools/st2py.py when it is missing or older than its sources, so
`from FB_PressureController import ...` works in every script.
"""

import subprocess
import sys
from functools import lru_cache
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SLAVE_SRC = ROOT / 'slave-src'
GENERATED = ROOT / 'design' / 'generated'
TOOLS = ROOT / 'tools'


def _sources():
    files = list((SLAVE_SRC / 'FB').glob('FB_*.st'))
    files.append(SLAVE_SRC / 'DUT' / 'DataTypes.st')
    files += [TOOLS / 'st2py.py', TOOLS / 'st2m.py']
    return files


def ensure_generated():
    """Regenerate design/generated unless every generated file is newer than
    every source."""
    out = list(GENERATED.glob('*.py'))
    stale = (not out or not (GENERATED / 'FB_PressureController.py').exists()
             or min(f.stat().st_mtime for f in out)
             < max(f.stat().st_mtime for f in _sources()))
    if stale:
        subprocess.run([sys.executable, str(TOOLS / 'st2py.py'),
                        str(SLAVE_SRC), str(GENERATED)], check=True, cwd=ROOT)


ensure_generated()
if str(GENERATED) not in sys.path:
    sys.path.insert(0, str(GENERATED))
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))


import st2m  # noqa: E402  (tools/ is on the path from above)


def _literal(typ, init):
    if init is None:
        return None
    if typ == 'BOOL':
        return init.upper() == 'TRUE'
    if typ == 'TIME':
        return float(st2m.time_seconds(init))
    try:
        value = float(init.split('#')[-1])
    except ValueError:
        return None
    return int(value) if typ in st2m.INT_TYPES else value


@lru_cache(maxsize=1)
def _defaults():
    ctx, _ = st2m.load_project(SLAVE_SRC)
    cfg = {}
    for name, decl in ctx['globals'].items():
        if name.startswith('G_cfg'):
            value = _literal(decl['type'], decl['init'])
            if value is not None:
                cfg[name] = value
    return cfg


def slave_config(**overrides):
    """The G_cfg* defaults of GlobalVariables_Reference.st as a dict (TIME in
    seconds, angles and lengths in the units the GVL comments give)."""
    cfg = dict(_defaults())
    for key, value in overrides.items():
        if key not in cfg:
            raise KeyError(f'unknown slave configuration {key}')
        cfg[key] = value
    return cfg
