# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_FaultResetHandler():
    s = SimpleNamespace()
    s.Enable = False
    s.FaultReset = False
    s.MotionEnable = False
    s.InFaultState = False
    s.CodeStable = False
    s.ActiveFaultCode = 0
    s.MasterFaultCode = 0
    s.FaultResetValid = False
    return s


def FB_FaultResetHandler(s, dt):
    if (not s.Enable):  # :38
        s.FaultResetValid = False  # :39
        return  # :40
    s.FaultResetValid = (((((s.FaultReset and ((not s.MotionEnable))) and s.InFaultState) and ((s.ActiveFaultCode != E.E_FaultCode.FAULT_NONE))) and s.CodeStable) and ((s.MasterFaultCode == s.ActiveFaultCode)))  # :50
