# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace


def new_FB_PistonExitGuard():
    s = SimpleNamespace()
    s.Enable = False
    s.ActualPosition = float(0)
    s.ActualVelocity = float(0)
    s.ActualTorque = float(0)
    s.CylinderEntry = float(0)
    s.GuardMargin = float(0)
    s.VelocityThreshold = float(0)
    s.TorqueThreshold = float(0)
    s.FaultTriggered = False
    s.GuardActive = False
    s.InDangerZone = False
    s.ApproachingLimit = False
    s.rGuardBoundary = float(0)
    return s


def FB_PistonExitGuard(s, dt):
    if (not s.Enable):  # :58
        s.FaultTriggered = False  # :60
        s.GuardActive = False  # :61
        s.InDangerZone = False  # :62
        s.ApproachingLimit = False  # :63
        return  # :64
    s.rGuardBoundary = (s.CylinderEntry + s.GuardMargin)  # :68
    s.InDangerZone = ((s.ActualPosition <= s.rGuardBoundary))  # :71
    s.ApproachingLimit = ((s.ActualVelocity < (-s.VelocityThreshold)))  # :72
    s.GuardActive = True  # :75
    s.FaultTriggered = False  # :78
    if s.InDangerZone:  # :95
        if (((s.ActualTorque > s.TorqueThreshold)) and ((s.ActualVelocity < (-s.VelocityThreshold)))):  # :98
            s.FaultTriggered = True  # :101
