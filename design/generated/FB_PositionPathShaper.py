# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import st_limit, st_min, st_sqrt


def new_FB_PositionPathShaper():
    s = SimpleNamespace()
    s.Enable = False
    s.Reset = False
    s.Target = float(0)
    s.ActualPosition = float(0)
    s.VelMax = float(0)
    s.AccelMax = float(0)
    s.MaxDeviation = float(0)
    s.CycleTime = float(0)
    s.CommandedPosition = float(0)
    s.CommandedRate = float(0)
    s.RateLimiting = False
    s.TetherHold = False
    s.ParamError = False
    s.rCmdPos = float(0.0)
    s.rCmdRate = float(0.0)
    s.rDist = float(0)
    s.rLead = float(0)
    s.rAdt = float(0)
    s.rRateStop = float(0)
    s.rRateTgt = float(0)
    s.bHold = False
    return s


def FB_PositionPathShaper(s, dt):
    if (((not s.Enable)) or s.Reset):  # :88
        s.rCmdPos = s.ActualPosition  # :89
        s.rCmdRate = 0.0  # :90
        s.CommandedPosition = s.rCmdPos  # :91
        s.CommandedRate = s.rCmdRate  # :92
        s.RateLimiting = False  # :93
        s.TetherHold = False  # :94
        s.ParamError = False  # :95
        return  # :96
    s.ParamError = ((((s.VelMax < 0.0)) or ((s.AccelMax <= 0.0))) or ((s.CycleTime <= 0.0)))  # :103
    if s.ParamError:  # :105
        s.rCmdRate = 0.0  # :106
        s.CommandedPosition = s.rCmdPos  # :107
        s.CommandedRate = s.rCmdRate  # :108
        s.RateLimiting = True  # :109
        s.TetherHold = False  # :110
        return  # :111
    s.rDist = (s.Target - s.rCmdPos)  # :120
    s.rLead = (s.rCmdPos - s.ActualPosition)  # :121
    s.bHold = ((((s.MaxDeviation > 0.0)) and ((abs(s.rLead) >= s.MaxDeviation))) and ((((s.rLead * s.rDist)) > 0.0)))  # :123
    if s.bHold:  # :127
        s.rCmdRate = 0.0  # :128
    else:  # :130
        s.rAdt = (s.AccelMax * s.CycleTime)  # :134
        s.rRateStop = ((-s.rAdt) + st_sqrt((((s.rAdt * s.rAdt)) + (((2.0 * s.AccelMax) * abs(s.rDist))))))  # :135
        s.rRateTgt = st_min(s.VelMax, s.rRateStop)  # :137
        if (s.rDist < 0.0):  # :138
            s.rRateTgt = (-s.rRateTgt)  # :139
        s.rCmdRate = (s.rCmdRate + st_limit((-s.rAdt), (s.rRateTgt - s.rCmdRate), s.rAdt))  # :145
        s.rCmdPos = (s.rCmdPos + ((s.rCmdRate * s.CycleTime)))  # :146
    s.CommandedPosition = s.rCmdPos  # :153
    s.CommandedRate = s.rCmdRate  # :154
    s.TetherHold = s.bHold  # :155
    s.RateLimiting = (s.bHold or ((abs((s.Target - s.rCmdPos)) > ((s.VelMax * s.CycleTime)))))  # :156
