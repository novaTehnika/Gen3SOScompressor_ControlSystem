# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_IsOperationalMode():
    s = SimpleNamespace()
    s.Mode = 0
    s.Result = False
    return s


def FB_IsOperationalMode(s, dt):
    s.Result = ((((((s.Mode == E.E_OperatingMode.MODE_BRAKE_HOLD)) or ((s.Mode == E.E_OperatingMode.MODE_POSITION))) or ((s.Mode == E.E_OperatingMode.MODE_VELOCITY))) or ((s.Mode == E.E_OperatingMode.MODE_TORQUE))) or ((s.Mode == E.E_OperatingMode.MODE_PRESSURE)))  # :24
