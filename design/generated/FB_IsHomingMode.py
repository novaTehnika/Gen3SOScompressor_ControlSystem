# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_IsHomingMode():
    s = SimpleNamespace()
    s.Mode = 0
    s.Result = False
    return s


def FB_IsHomingMode(s, dt):
    s.Result = (((s.Mode == E.E_OperatingMode.MODE_GO_HOME)) or ((s.Mode == E.E_OperatingMode.MODE_HOME_LIMIT)))  # :21
