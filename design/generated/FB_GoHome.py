# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import R_TRIG, new_R_TRIG, new_ST_CmdMoveAbsolute, new_ST_CmdStop, new_ST_StaMoveAbsolute, new_ST_StaStop, E


def new_FB_GoHome():
    s = SimpleNamespace()
    s.Enable = False
    s.Execute = False
    s.HomePosition = float(0)
    s.HomeVelocity = float(0)
    s.Acceleration = float(0)
    s.Deceleration = float(0)
    s.StopDeceleration = float(0)
    s.Abort = False
    s.StaMoveAbsolute = new_ST_StaMoveAbsolute()
    s.StaStop = new_ST_StaStop()
    s.Active = False
    s.Done = False
    s.AtPosition = False
    s.InMotion = False
    s.Error = False
    s.CmdMoveAbsolute = new_ST_CmdMoveAbsolute()
    s.CmdStop = new_ST_CmdStop()
    s.eState = 0
    s.rTrigExecute = new_R_TRIG()
    s.rPosTolerance = float(0.1)
    s.GOH_IDLE = 0
    s.GOH_MOVING = 3
    s.GOH_AT_POSITION = 4
    s.GOH_ABORTING = 5
    s.GOH_ERROR = 6
    return s


def FB_GoHome(s, dt):
    if (not s.Enable):  # :56
        s.Active = False  # :57
        s.Done = False  # :58
        s.AtPosition = False  # :59
        s.InMotion = False  # :60
        s.Error = False  # :61
        s.eState = s.GOH_IDLE  # :62
        s.CmdMoveAbsolute.Execute = False  # :63
        s.CmdStop.Execute = False  # :64
        return  # :65
    s.rTrigExecute.CLK = s.Execute  # :69
    R_TRIG(s.rTrigExecute, dt)  # :69
    if ((s.Abort and ((s.eState != s.GOH_IDLE))) and ((s.eState != s.GOH_ERROR))):  # :72
        s.CmdMoveAbsolute.Execute = False  # :73
        s.eState = s.GOH_ABORTING  # :74
    if (s.eState == s.GOH_IDLE):  # :81
        s.Active = False  # :84
        s.Done = False  # :85
        s.AtPosition = False  # :86
        s.InMotion = False  # :87
        s.Error = False  # :88
        s.CmdMoveAbsolute.Execute = False  # :90
        s.CmdStop.Execute = False  # :91
        if (s.rTrigExecute.Q or s.Execute):  # :93
            s.Active = True  # :94
            s.eState = s.GOH_MOVING  # :95
    elif (s.eState == s.GOH_MOVING):  # :99
        s.CmdMoveAbsolute.Execute = True  # :102
        s.CmdMoveAbsolute.Position = s.HomePosition  # :103
        s.CmdMoveAbsolute.Velocity = s.HomeVelocity  # :104
        s.CmdMoveAbsolute.Acceleration = s.Acceleration  # :105
        s.CmdMoveAbsolute.Deceleration = s.Deceleration  # :106
        s.CmdMoveAbsolute.Direction = E.MC_Direction.Current_Direction  # :107
        s.InMotion = s.StaMoveAbsolute.Busy  # :109
        if s.StaMoveAbsolute.Done:  # :111
            s.CmdMoveAbsolute.Execute = False  # :112
            s.Done = True  # :113
            s.AtPosition = True  # :114
            s.InMotion = False  # :115
            s.eState = s.GOH_AT_POSITION  # :116
        elif s.StaMoveAbsolute.Error:  # :118
            s.Error = True  # :119
            s.CmdMoveAbsolute.Execute = False  # :120
            s.eState = s.GOH_ERROR  # :121
        elif s.StaMoveAbsolute.CommandAborted:  # :123
            s.CmdMoveAbsolute.Execute = False  # :124
            s.eState = s.GOH_ABORTING  # :125
    elif (s.eState == s.GOH_AT_POSITION):  # :129
        s.Done = True  # :132
        s.AtPosition = True  # :133
        s.InMotion = False  # :134
        s.Active = True  # :135
        if (not s.Execute):  # :137
            s.Done = False  # :138
            s.eState = s.GOH_IDLE  # :139
    elif (s.eState == s.GOH_ABORTING):  # :143
        s.CmdStop.Deceleration = s.StopDeceleration  # :146
        s.CmdStop.Execute = True  # :147
        if s.StaStop.Done:  # :149
            s.CmdStop.Execute = False  # :150
            s.Active = False  # :151
            s.InMotion = False  # :152
            s.eState = s.GOH_IDLE  # :153
        elif s.StaStop.Error:  # :155
            s.CmdStop.Execute = False  # :156
            s.eState = s.GOH_ERROR  # :157
    elif (s.eState == s.GOH_ERROR):  # :161
        s.Active = False  # :164
        s.InMotion = False  # :165
        s.Done = False  # :166
        if (not s.Execute):  # :168
            s.Error = False  # :169
            s.eState = s.GOH_IDLE  # :170
