# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_FaultToInt():
    s = SimpleNamespace()
    s.Fault = 0
    s.Value = 0
    return s


def FB_FaultToInt(s, dt):
    if (s.Fault == E.E_FaultCode.FAULT_NONE):  # :21
        s.Value = 0  # :22
    elif (s.Fault == E.E_FaultCode.FAULT_HANDSHAKE):  # :23
        s.Value = 1  # :24
    elif (s.Fault == E.E_FaultCode.FAULT_DRIVE):  # :25
        s.Value = 2  # :26
    elif (s.Fault == E.E_FaultCode.FAULT_POSITION):  # :27
        s.Value = 3  # :28
    elif (s.Fault == E.E_FaultCode.FAULT_COMM):  # :29
        s.Value = 4  # :30
    elif (s.Fault == E.E_FaultCode.FAULT_PISTON_EXIT):  # :31
        s.Value = 5  # :32
    elif (s.Fault == E.E_FaultCode.FAULT_LIMIT_SWITCH):  # :33
        s.Value = 6  # :34
    elif (s.Fault == E.E_FaultCode.FAULT_ENCODER):  # :35
        s.Value = 7  # :36
    elif (s.Fault == E.E_FaultCode.FAULT_PRESSURE_SENSOR):  # :37
        s.Value = 8  # :38
    else:  # :39
        s.Value = 0  # :40
