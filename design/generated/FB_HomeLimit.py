# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import R_TRIG, TON, new_R_TRIG, new_ST_CmdJog, new_ST_CmdMoveAbsolute, new_ST_CmdSetPosition, new_ST_StaJog, new_ST_StaMoveAbsolute, new_ST_StaSetPosition, new_TON, E


def new_FB_HomeLimit():
    s = SimpleNamespace()
    s.Enable = False
    s.Execute = False
    s.FastApproachVelocity = float(0)
    s.SlowApproachVelocity = float(0)
    s.BackoffVelocity = float(0)
    s.RetractDist = float(0)
    s.Acceleration = float(0)
    s.Deceleration = float(0)
    s.SetPosition = float(0)
    s.Timeout = 0
    s.OvertravelNegActive = False
    s.OvertravelPosActive = False
    s.Abort = False
    s.ActualPosition = float(0)
    s.StaJog = new_ST_StaJog()
    s.StaMoveAbsolute = new_ST_StaMoveAbsolute()
    s.StaSetPosition = new_ST_StaSetPosition()
    s.Active = False
    s.Done = False
    s.InMotion = False
    s.Error = False
    s.CmdJog = new_ST_CmdJog()
    s.CmdMoveAbsolute = new_ST_CmdMoveAbsolute()
    s.CmdSetPosition = new_ST_CmdSetPosition()
    s.eState = 0
    s.rTrigExecute = new_R_TRIG()
    s.tTimeout = new_TON()
    s.bTimeoutIN = False
    s.rSwitchPos = float(0)
    return s


def FB_HomeLimit(s, dt):
    if (not s.Enable):  # :101
        s.Active = False  # :102
        s.Done = False  # :103
        s.InMotion = False  # :104
        s.Error = False  # :105
        s.eState = E.E_HomeLimitState.IDLE  # :106
        s.CmdJog.Forward = False  # :107
        s.CmdJog.Reverse = False  # :108
        s.CmdJog.Velocity = 0.0  # :109
        s.CmdMoveAbsolute.Execute = False  # :110
        s.CmdSetPosition.Execute = False  # :111
        s.bTimeoutIN = False  # :112
    else:  # :114
        s.rTrigExecute.CLK = s.Execute  # :116
        R_TRIG(s.rTrigExecute, dt)  # :116
        if (((s.Abort and ((s.eState != E.E_HomeLimitState.IDLE))) and ((s.eState != E.E_HomeLimitState.ERROR))) and ((s.eState != E.E_HomeLimitState.DONE))):  # :119
            s.CmdJog.Forward = False  # :120
            s.CmdJog.Reverse = False  # :121
            s.CmdMoveAbsolute.Execute = False  # :122
            s.CmdSetPosition.Execute = False  # :123
            s.eState = E.E_HomeLimitState.ABORTING  # :124
        if ((((s.OvertravelPosActive and ((s.eState != E.E_HomeLimitState.IDLE))) and ((s.eState != E.E_HomeLimitState.ERROR))) and ((s.eState != E.E_HomeLimitState.DONE))) and ((s.eState != E.E_HomeLimitState.ABORTING))):  # :128
            s.CmdJog.Forward = False  # :130
            s.CmdJog.Reverse = False  # :131
            s.CmdMoveAbsolute.Execute = False  # :132
            s.CmdSetPosition.Execute = False  # :133
            s.Error = True  # :134
            s.eState = E.E_HomeLimitState.ERROR  # :135
        s.bTimeoutIN = (((((s.eState != E.E_HomeLimitState.IDLE)) and ((s.eState != E.E_HomeLimitState.DONE))) and ((s.eState != E.E_HomeLimitState.ERROR))) and ((s.eState != E.E_HomeLimitState.ABORTING)))  # :139
        if s.tTimeout.Q:  # :142
            s.CmdJog.Forward = False  # :143
            s.CmdJog.Reverse = False  # :144
            s.CmdMoveAbsolute.Execute = False  # :145
            s.CmdSetPosition.Execute = False  # :146
            s.Error = True  # :147
            s.bTimeoutIN = False  # :148
            s.eState = E.E_HomeLimitState.ERROR  # :149
        if (s.eState == E.E_HomeLimitState.IDLE):  # :156
            s.Active = False  # :159
            s.Done = False  # :160
            s.InMotion = False  # :161
            s.Error = False  # :162
            s.CmdJog.Forward = False  # :164
            s.CmdJog.Reverse = False  # :165
            s.CmdMoveAbsolute.Execute = False  # :166
            s.CmdSetPosition.Execute = False  # :167
            if (s.rTrigExecute.Q or s.Execute):  # :169
                s.Active = True  # :170
                s.eState = E.E_HomeLimitState.FAST_APPROACH  # :171
        elif (s.eState == E.E_HomeLimitState.FAST_APPROACH):  # :175
            s.CmdJog.Reverse = True  # :178
            s.CmdJog.Forward = False  # :179
            s.CmdJog.Velocity = s.FastApproachVelocity  # :180
            s.CmdJog.Acceleration = s.Acceleration  # :181
            s.CmdJog.Deceleration = s.Deceleration  # :182
            s.InMotion = s.StaJog.Busy  # :184
            if s.OvertravelNegActive:  # :186
                s.CmdJog.Reverse = False  # :187
                s.eState = E.E_HomeLimitState.FAST_AWAIT  # :188
            elif s.StaJog.Error:  # :190
                s.CmdJog.Reverse = False  # :191
                s.Error = True  # :192
                s.eState = E.E_HomeLimitState.ERROR  # :193
        elif (s.eState == E.E_HomeLimitState.FAST_AWAIT):  # :197
            s.CmdJog.Forward = False  # :200
            s.CmdJog.Reverse = False  # :201
            s.InMotion = s.StaJog.Busy  # :202
            if s.StaJog.Done:  # :204
                s.eState = E.E_HomeLimitState.INTER_BACKOFF  # :205
            elif s.StaJog.Error:  # :207
                s.Error = True  # :208
                s.eState = E.E_HomeLimitState.ERROR  # :209
        elif (s.eState == E.E_HomeLimitState.INTER_BACKOFF):  # :213
            s.CmdJog.Forward = True  # :216
            s.CmdJog.Reverse = False  # :217
            s.CmdJog.Velocity = s.BackoffVelocity  # :218
            s.CmdJog.Acceleration = s.Acceleration  # :219
            s.CmdJog.Deceleration = s.Deceleration  # :220
            s.InMotion = s.StaJog.Busy  # :222
            if (not s.OvertravelNegActive):  # :224
                s.CmdJog.Forward = False  # :225
                s.eState = E.E_HomeLimitState.INTER_AWAIT  # :226
            elif s.StaJog.Error:  # :228
                s.CmdJog.Forward = False  # :229
                s.Error = True  # :230
                s.eState = E.E_HomeLimitState.ERROR  # :231
        elif (s.eState == E.E_HomeLimitState.INTER_AWAIT):  # :235
            s.CmdJog.Forward = False  # :238
            s.CmdJog.Reverse = False  # :239
            s.InMotion = s.StaJog.Busy  # :240
            if s.StaJog.Done:  # :242
                s.eState = E.E_HomeLimitState.SLOW_APPROACH  # :243
            elif s.StaJog.Error:  # :245
                s.Error = True  # :246
                s.eState = E.E_HomeLimitState.ERROR  # :247
        elif (s.eState == E.E_HomeLimitState.SLOW_APPROACH):  # :251
            s.CmdJog.Reverse = True  # :256
            s.CmdJog.Forward = False  # :257
            s.CmdJog.Velocity = s.SlowApproachVelocity  # :258
            s.CmdJog.Acceleration = s.Acceleration  # :259
            s.CmdJog.Deceleration = s.Deceleration  # :260
            s.InMotion = s.StaJog.Busy  # :262
            if s.OvertravelNegActive:  # :264
                s.rSwitchPos = s.ActualPosition  # :265
                s.CmdJog.Reverse = False  # :266
                s.eState = E.E_HomeLimitState.SLOW_AWAIT  # :267
            elif s.StaJog.Error:  # :269
                s.CmdJog.Reverse = False  # :270
                s.Error = True  # :271
                s.eState = E.E_HomeLimitState.ERROR  # :272
        elif (s.eState == E.E_HomeLimitState.SLOW_AWAIT):  # :276
            s.CmdJog.Forward = False  # :279
            s.CmdJog.Reverse = False  # :280
            s.InMotion = s.StaJog.Busy  # :281
            if s.StaJog.Done:  # :283
                s.eState = E.E_HomeLimitState.RETRACT  # :284
            elif s.StaJog.Error:  # :286
                s.Error = True  # :287
                s.eState = E.E_HomeLimitState.ERROR  # :288
        elif (s.eState == E.E_HomeLimitState.RETRACT):  # :292
            s.CmdMoveAbsolute.Execute = True  # :298
            s.CmdMoveAbsolute.Position = (s.rSwitchPos + s.RetractDist)  # :299
            s.CmdMoveAbsolute.Velocity = s.BackoffVelocity  # :300
            s.CmdMoveAbsolute.Acceleration = s.Acceleration  # :301
            s.CmdMoveAbsolute.Deceleration = s.Deceleration  # :302
            s.CmdMoveAbsolute.Direction = E.MC_Direction.Positive_Direction  # :303
            s.InMotion = s.StaMoveAbsolute.Busy  # :305
            s.eState = E.E_HomeLimitState.RETRACT_AWAIT  # :306
        elif (s.eState == E.E_HomeLimitState.RETRACT_AWAIT):  # :309
            s.CmdMoveAbsolute.Execute = False  # :312
            s.InMotion = s.StaMoveAbsolute.Busy  # :313
            if s.StaMoveAbsolute.Done:  # :315
                s.eState = E.E_HomeLimitState.SETREF  # :316
            elif (s.StaMoveAbsolute.Error or s.StaMoveAbsolute.CommandAborted):  # :318
                s.Error = True  # :319
                s.eState = E.E_HomeLimitState.ERROR  # :320
        elif (s.eState == E.E_HomeLimitState.SETREF):  # :324
            s.CmdJog.Forward = False  # :329
            s.CmdJog.Reverse = False  # :330
            s.InMotion = False  # :331
            s.CmdSetPosition.Execute = True  # :333
            s.CmdSetPosition.Position = s.SetPosition  # :334
            s.CmdSetPosition.Mode = False  # :335
            if s.StaSetPosition.Done:  # :337
                s.CmdSetPosition.Execute = False  # :338
                s.Done = True  # :339
                s.eState = E.E_HomeLimitState.DONE  # :340
            elif s.StaSetPosition.Error:  # :342
                s.CmdSetPosition.Execute = False  # :343
                s.Error = True  # :344
                s.eState = E.E_HomeLimitState.ERROR  # :345
        elif (s.eState == E.E_HomeLimitState.DONE):  # :349
            s.Done = True  # :352
            s.Active = True  # :353
            s.InMotion = False  # :354
            if (not s.Execute):  # :356
                s.Done = False  # :357
                s.eState = E.E_HomeLimitState.IDLE  # :358
        elif (s.eState == E.E_HomeLimitState.ABORTING):  # :362
            s.CmdJog.Forward = False  # :366
            s.CmdJog.Reverse = False  # :367
            s.CmdMoveAbsolute.Execute = False  # :368
            s.InMotion = (s.StaJog.Busy or s.StaMoveAbsolute.Busy)  # :369
            if (((s.StaJog.Done or (not s.StaJog.Busy))) and ((not s.StaMoveAbsolute.Busy))):  # :371
                s.Active = False  # :372
                s.InMotion = False  # :373
                s.eState = E.E_HomeLimitState.IDLE  # :374
            elif (s.StaJog.Error or s.StaMoveAbsolute.Error):  # :376
                s.eState = E.E_HomeLimitState.ERROR  # :377
        elif (s.eState == E.E_HomeLimitState.ERROR):  # :381
            s.Active = False  # :384
            s.InMotion = False  # :385
            s.Done = False  # :386
            if (not s.Execute):  # :388
                s.Error = False  # :389
                s.eState = E.E_HomeLimitState.IDLE  # :390
    s.tTimeout.IN = s.bTimeoutIN  # :399
    s.tTimeout.PT = s.Timeout  # :399
    TON(s.tTimeout, dt)  # :399
