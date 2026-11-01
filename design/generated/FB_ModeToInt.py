# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_ModeToInt():
    s = SimpleNamespace()
    s.Mode = 0
    s.Value = 0
    return s


def FB_ModeToInt(s, dt):
    if (s.Mode == E.E_OperatingMode.MODE_IDLE):  # :21
        s.Value = 0  # :22
    elif (s.Mode == E.E_OperatingMode.MODE_BRAKE_HOLD):  # :23
        s.Value = 1  # :24
    elif (s.Mode == E.E_OperatingMode.MODE_POSITION):  # :25
        s.Value = 2  # :26
    elif (s.Mode == E.E_OperatingMode.MODE_VELOCITY):  # :27
        s.Value = 3  # :28
    elif (s.Mode == E.E_OperatingMode.MODE_TORQUE):  # :29
        s.Value = 4  # :30
    elif (s.Mode == E.E_OperatingMode.MODE_GO_HOME):  # :31
        s.Value = 5  # :32
    elif (s.Mode == E.E_OperatingMode.MODE_HOME_LIMIT):  # :33
        s.Value = 6  # :34
    elif (s.Mode == E.E_OperatingMode.MODE_PRESSURE):  # :35
        s.Value = 7  # :36
    else:  # :37
        s.Value = 0  # :38
