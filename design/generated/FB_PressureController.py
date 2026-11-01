# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import st_div, st_limit, st_max, st_min, st_sqrt


def new_FB_PressureController():
    s = SimpleNamespace()
    s.Enable = False
    s.Reset = False
    s.Target = float(0)
    s.Actual = float(0)
    s.Kp = float(0)
    s.Ki = float(0)
    s.VelMax = float(0)
    s.DecelMax = float(0)
    s.Stiffness = float(0)
    s.StopVelMin = float(0)
    s.CycleTime = float(0)
    s.IntegScale = float(1.0)
    s.Velocity = float(0)
    s.Error = float(0)
    s.VelLimit = float(0)
    s.Saturated = False
    s.ParamError = False
    s.rInteg = float(0.0)
    s.rRaw = float(0)
    s.rDist = float(0)
    s.rStep = float(0)
    s.rVelStop = float(0)
    return s


def FB_PressureController(s, dt):
    s.ParamError = ((((((s.VelMax <= 0.0)) or ((s.DecelMax <= 0.0))) or ((s.Stiffness <= 0.0))) or ((s.CycleTime <= 0.0))) or ((s.StopVelMin < 0.0)))  # :74
    s.rInteg = (s.rInteg * s.IntegScale)  # :78
    if ((((not s.Enable)) or s.Reset) or s.ParamError):  # :80
        s.rInteg = 0.0  # :81
    if (((not s.Enable)) or s.ParamError):  # :84
        s.Velocity = 0.0  # :85
        s.Error = 0.0  # :86
        s.VelLimit = 0.0  # :87
        s.Saturated = False  # :88
        return  # :89
    s.Error = (s.Target - s.Actual)  # :92
    s.rDist = st_div(abs(s.Error), s.Stiffness)  # :99
    s.rStep = (s.DecelMax * s.CycleTime)  # :100
    s.rVelStop = ((-s.rStep) + st_sqrt(((s.rStep * s.rStep) + ((2.0 * s.DecelMax) * s.rDist))))  # :101
    s.VelLimit = st_min(s.VelMax, st_max(s.StopVelMin, s.rVelStop))  # :102
    s.rRaw = ((s.Kp * s.Error) + (s.Ki * s.rInteg))  # :107
    s.Velocity = st_limit((-s.VelLimit), s.rRaw, s.VelLimit)  # :108
    s.Saturated = ((s.rRaw != s.Velocity))  # :109
    if ((((not s.Saturated)) or ((((s.rRaw > s.Velocity)) and ((s.Error < 0.0))))) or ((((s.rRaw < s.Velocity)) and ((s.Error > 0.0))))):  # :111
        s.rInteg = (s.rInteg + (s.Error * s.CycleTime))  # :114
