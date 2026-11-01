# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_IntToMode():
    s = SimpleNamespace()
    s.Value = 0
    s.Mode = 0
    return s


def FB_IntToMode(s, dt):
    if (s.Value == 0):  # :22
        s.Mode = E.E_OperatingMode.MODE_IDLE  # :23
    elif (s.Value == 1):  # :24
        s.Mode = E.E_OperatingMode.MODE_BRAKE_HOLD  # :25
    elif (s.Value == 2):  # :26
        s.Mode = E.E_OperatingMode.MODE_POSITION  # :27
    elif (s.Value == 3):  # :28
        s.Mode = E.E_OperatingMode.MODE_VELOCITY  # :29
    elif (s.Value == 4):  # :30
        s.Mode = E.E_OperatingMode.MODE_TORQUE  # :31
    elif (s.Value == 5):  # :32
        s.Mode = E.E_OperatingMode.MODE_GO_HOME  # :33
    elif (s.Value == 6):  # :34
        s.Mode = E.E_OperatingMode.MODE_HOME_LIMIT  # :35
    elif (s.Value == 7):  # :36
        s.Mode = E.E_OperatingMode.MODE_PRESSURE  # :37
    else:  # :38
        s.Mode = E.E_OperatingMode.MODE_IDLE  # :39
