# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import E


def new_FB_SafetyMonitor():
    s = SimpleNamespace()
    s.Enable = False
    s.OvertravelNegActive = False
    s.OvertravelPosActive = False
    s.ActualPosition = float(0)
    s.SoftLimitMin = float(0)
    s.SoftLimitMax = float(0)
    s.DriveFault = False
    s.CurrentState = 0
    s.InCommandedMotion = False
    s.InMotionWatched = False
    s.InHoming = False
    s.InRecovery = False
    s.InFault = False
    s.ExitGuardTriggered = False
    s.CommLost = False
    s.PressureSensorFault = False
    s.FaultDetected = False
    s.FaultCode = 0
    s.FaultLimitSwitch = False
    s.FaultPosition = False
    s.FaultDrive = False
    s.FaultComm = False
    s.FaultPistonExit = False
    s.FaultPressureSensor = False
    s.bLatchedFault = False
    s.eLatchedFaultCode = 0
    return s


def FB_SafetyMonitor(s, dt):
    if (not s.Enable):  # :73
        s.FaultDetected = False  # :75
        s.FaultCode = E.E_FaultCode.FAULT_NONE  # :76
        s.FaultLimitSwitch = False  # :77
        s.FaultPosition = False  # :78
        s.FaultDrive = False  # :79
        s.FaultComm = False  # :80
        s.FaultPistonExit = False  # :81
        s.FaultPressureSensor = False  # :82
        s.bLatchedFault = False  # :83
        s.eLatchedFaultCode = E.E_FaultCode.FAULT_NONE  # :84
        return  # :85
    s.FaultDrive = s.DriveFault  # :93
    s.FaultComm = (s.CommLost and s.InCommandedMotion)  # :105
    if s.InHoming:  # :114
        if s.OvertravelPosActive:  # :117
            s.FaultLimitSwitch = True  # :118
        else:  # :119
            s.FaultLimitSwitch = False  # :120
    elif s.InRecovery:  # :122
        if (s.OvertravelNegActive and s.OvertravelPosActive):  # :127
            s.FaultLimitSwitch = True  # :128
        else:  # :129
            s.FaultLimitSwitch = False  # :130
    elif s.InMotionWatched:  # :132
        if (s.OvertravelNegActive or s.OvertravelPosActive):  # :134
            s.FaultLimitSwitch = True  # :135
        else:  # :136
            s.FaultLimitSwitch = False  # :137
    else:  # :139
        s.FaultLimitSwitch = False  # :141
    if ((((s.CurrentState != E.E_SystemState.ST_HOME_LIMIT)) and ((not s.InRecovery))) and ((((s.ActualPosition < s.SoftLimitMin)) or ((s.ActualPosition > s.SoftLimitMax))))):  # :160
        s.FaultPosition = True  # :163
    else:  # :164
        s.FaultPosition = False  # :165
    s.FaultPistonExit = s.ExitGuardTriggered  # :173
    s.FaultPressureSensor = (s.PressureSensorFault and ((s.CurrentState == E.E_SystemState.ST_PRESSURE_CTRL)))  # :182
    if s.FaultDrive:  # :193
        s.FaultDetected = True  # :194
        s.FaultCode = E.E_FaultCode.FAULT_DRIVE  # :195
    elif s.FaultComm:  # :197
        s.FaultDetected = True  # :198
        s.FaultCode = E.E_FaultCode.FAULT_COMM  # :199
    elif s.FaultLimitSwitch:  # :201
        s.FaultDetected = True  # :202
        s.FaultCode = E.E_FaultCode.FAULT_LIMIT_SWITCH  # :203
    elif s.FaultPosition:  # :205
        s.FaultDetected = True  # :206
        s.FaultCode = E.E_FaultCode.FAULT_POSITION  # :207
    elif s.FaultPressureSensor:  # :209
        s.FaultDetected = True  # :210
        s.FaultCode = E.E_FaultCode.FAULT_PRESSURE_SENSOR  # :211
    elif s.FaultPistonExit:  # :213
        s.FaultDetected = True  # :214
        s.FaultCode = E.E_FaultCode.FAULT_PISTON_EXIT  # :215
    else:  # :217
        s.FaultDetected = False  # :218
        s.FaultCode = E.E_FaultCode.FAULT_NONE  # :219
    if ((not s.InFault) and (not s.FaultDetected)):  # :231
        s.bLatchedFault = False  # :232
        s.eLatchedFaultCode = E.E_FaultCode.FAULT_NONE  # :233
    if ((s.FaultDetected and (not s.bLatchedFault)) and (not s.InFault)):  # :236
        s.bLatchedFault = True  # :237
        s.eLatchedFaultCode = s.FaultCode  # :238
    if (s.bLatchedFault and (not s.FaultDetected)):  # :242
        s.FaultDetected = True  # :243
        s.FaultCode = s.eLatchedFaultCode  # :244
