# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_IntToFaultCode():
    s = SimpleNamespace()
    s.Value = 0
    s.Fault = 0
    return s


def FB_IntToFaultCode(s, dt):
    if (s.Value == 0):  # :23
        s.Fault = E.E_FaultCode.FAULT_NONE  # :24
    elif (s.Value == 1):  # :25
        s.Fault = E.E_FaultCode.FAULT_HANDSHAKE  # :26
    elif (s.Value == 2):  # :27
        s.Fault = E.E_FaultCode.FAULT_DRIVE  # :28
    elif (s.Value == 3):  # :29
        s.Fault = E.E_FaultCode.FAULT_POSITION  # :30
    elif (s.Value == 4):  # :31
        s.Fault = E.E_FaultCode.FAULT_COMM  # :32
    elif (s.Value == 5):  # :33
        s.Fault = E.E_FaultCode.FAULT_PISTON_EXIT  # :34
    elif (s.Value == 6):  # :35
        s.Fault = E.E_FaultCode.FAULT_LIMIT_SWITCH  # :36
    elif (s.Value == 7):  # :37
        s.Fault = E.E_FaultCode.FAULT_ENCODER  # :38
    elif (s.Value == 8):  # :39
        s.Fault = E.E_FaultCode.FAULT_PRESSURE_SENSOR  # :40
    else:  # :41
        s.Fault = E.E_FaultCode.FAULT_NONE  # :42
