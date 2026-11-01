# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import TON, new_TON, E


def new_FB_HandshakeManager():
    s = SimpleNamespace()
    s.Enable = False
    s.RequestedMode = 0
    s.ModeStable = False
    s.MotionEnable = False
    s.HandshakeTimeout = 0.5
    s.ConfirmedMode = 0
    s.HandshakeActive = False
    s.HandshakeComplete = False
    s.HandshakeTimeout_Q = False
    s.eState = 0
    s.tTimeout = new_TON()
    s.bTimeoutIN = False
    s.eModeAtStart = 0
    s.bModeStableAtStart = False
    s.HS_IDLE = 0
    s.HS_WAIT_MOTION_EN = 1
    s.HS_VERIFY_MODE = 2
    s.HS_COMPLETE = 3
    s.HS_TIMEOUT = 4
    return s


def FB_HandshakeManager(s, dt):
    if (not s.Enable):  # :50
        s.ConfirmedMode = E.E_OperatingMode.MODE_IDLE  # :52
        s.HandshakeActive = False  # :53
        s.HandshakeComplete = False  # :54
        s.HandshakeTimeout_Q = False  # :55
        s.eState = s.HS_IDLE  # :56
        s.bTimeoutIN = False  # :57
    else:  # :59
        s.bTimeoutIN = ((s.eState == s.HS_WAIT_MOTION_EN))  # :61
        if (s.eState == s.HS_IDLE):  # :67
            s.HandshakeActive = False  # :70
            s.HandshakeComplete = False  # :71
            s.HandshakeTimeout_Q = False  # :72
            if ((s.ModeStable and ((s.RequestedMode != E.E_OperatingMode.MODE_IDLE))) and (not s.MotionEnable)):  # :76
                s.ConfirmedMode = s.RequestedMode  # :77
                s.eModeAtStart = s.RequestedMode  # :78
                s.bModeStableAtStart = True  # :79
                s.HandshakeActive = True  # :80
                s.eState = s.HS_WAIT_MOTION_EN  # :81
                s.bTimeoutIN = True  # :82
            elif (s.ModeStable and ((s.RequestedMode == E.E_OperatingMode.MODE_IDLE))):  # :83
                s.ConfirmedMode = E.E_OperatingMode.MODE_IDLE  # :84
        elif (s.eState == s.HS_WAIT_MOTION_EN):  # :87
            if s.tTimeout.Q:  # :90
                s.bTimeoutIN = False  # :91
                s.eState = s.HS_TIMEOUT  # :92
            elif (s.RequestedMode != s.eModeAtStart):  # :94
                s.bTimeoutIN = False  # :95
                s.eState = s.HS_IDLE  # :96
            elif s.MotionEnable:  # :98
                s.eState = s.HS_VERIFY_MODE  # :99
        elif (s.eState == s.HS_VERIFY_MODE):  # :102
            s.bTimeoutIN = False  # :105
            if (s.RequestedMode == s.eModeAtStart):  # :106
                s.HandshakeComplete = True  # :107
                s.HandshakeActive = False  # :108
                s.eState = s.HS_COMPLETE  # :109
            else:  # :110
                s.eState = s.HS_TIMEOUT  # :111
        elif (s.eState == s.HS_COMPLETE):  # :114
            if (not s.MotionEnable):  # :117
                s.HandshakeComplete = False  # :119
                s.eState = s.HS_IDLE  # :120
        elif (s.eState == s.HS_TIMEOUT):  # :123
            s.HandshakeTimeout_Q = True  # :126
            s.HandshakeActive = False  # :127
            s.ConfirmedMode = E.E_OperatingMode.MODE_IDLE  # :128
            s.bTimeoutIN = False  # :129
            if (not s.MotionEnable):  # :131
                s.HandshakeTimeout_Q = False  # :132
                s.eState = s.HS_IDLE  # :133
    s.tTimeout.IN = s.bTimeoutIN  # :142
    s.tTimeout.PT = s.HandshakeTimeout  # :142
    TON(s.tTimeout, dt)  # :142
