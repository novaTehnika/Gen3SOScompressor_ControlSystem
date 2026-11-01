# Generated from slave-src by tools/st_translate.py --target py - do not edit.
"""Runtime support for the translated function blocks."""
import math
from types import SimpleNamespace

_INF, _NAN = math.inf, math.nan


def st_min(a, b):
    return b if (a != a or b < a) else a


def st_max(a, b):
    return b if (a != a or b > a) else a


def st_limit(mn, x, mx):
    """IEC LIMIT(MN, IN, MX)."""
    return st_min(st_max(x, mn), mx)


def st_sqrt(x):
    return math.sqrt(x) if x >= 0 else (x if x != x else _NAN)


def st_round(x):
    """Round half away from zero."""
    if x != x or x in (_INF, -_INF):
        return x
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(0.5 - x))


def st_div(a, b):
    """Real division with IEEE results for a zero divisor."""
    if b:
        return a / b
    if a != a or not a:
        return _NAN
    return math.copysign(_INF, a) * math.copysign(1.0, b)


def st_idiv(a, b):
    """Integer division truncating toward zero."""
    q = st_div(a, b)
    return q if q != q or q in (_INF, -_INF) else int(q)


def st_mod(a, b):
    return a % b if b else a


def st_bitor(a, b):
    return int(a) | int(b)


def st_bitand(a, b):
    return int(a) & int(b)


def st_bitxor(a, b):
    return int(a) ^ int(b)


def st_range(lo, hi, step=1):
    """FOR lo TO hi BY step, bounds inclusive."""
    return range(lo, hi + (1 if step > 0 else -1), step)


def new_TON():
    return SimpleNamespace(IN=False, PT=0.0, Q=False, ET=0.0, M=False)


def TON(s, dt):
    """IEC 61131-3 on-delay timer: Q is set once IN has been TRUE for PT
    seconds; a call with IN FALSE resets it."""
    if not s.IN:
        s.ET = 0.0
        s.Q = False
    else:
        s.ET = s.ET + dt if s.M else 0.0
        s.Q = s.ET >= s.PT
    s.M = s.IN


def new_R_TRIG():
    return SimpleNamespace(CLK=False, Q=False, M=False)


def R_TRIG(s, dt):
    """IEC 61131-3 rising-edge detector."""
    s.Q = bool(s.CLK) and not s.M
    s.M = s.CLK


def new_F_TRIG():
    return SimpleNamespace(CLK=False, Q=False, M=True)


def F_TRIG(s, dt):
    """IEC 61131-3 falling-edge detector."""
    s.Q = not s.CLK and not s.M
    s.M = not s.CLK


# Enumerations from DataTypes.st and the motion library.
E = SimpleNamespace(
    E_FaultCode=SimpleNamespace(FAULT_NONE=0, FAULT_HANDSHAKE=1, FAULT_DRIVE=2, FAULT_POSITION=3, FAULT_COMM=4, FAULT_PISTON_EXIT=5, FAULT_LIMIT_SWITCH=6, FAULT_ENCODER=7, FAULT_PRESSURE_SENSOR=8),
    E_HomeLimitState=SimpleNamespace(IDLE=0, FAST_APPROACH=1, FAST_AWAIT=2, INTER_BACKOFF=3, INTER_AWAIT=4, SLOW_APPROACH=5, SLOW_AWAIT=6, RETRACT=7, RETRACT_AWAIT=8, SETREF=9, DONE=10, ABORTING=11, ERROR=12),
    E_OperatingMode=SimpleNamespace(MODE_IDLE=0, MODE_BRAKE_HOLD=1, MODE_POSITION=2, MODE_VELOCITY=3, MODE_TORQUE=4, MODE_GO_HOME=5, MODE_HOME_LIMIT=6, MODE_PRESSURE=7),
    E_SystemState=SimpleNamespace(ST_INIT=0, ST_IDLE=1, ST_DRIVE_RESET=2, ST_DRIVE_ENABLE=3, ST_BRAKE_RELEASE=4, ST_BRAKE_HOLD=5, ST_POSITION_CTRL=6, ST_VELOCITY_CTRL=7, ST_TORQUE_CTRL=8, ST_PRESSURE_CTRL=9, ST_GO_HOME=10, ST_HOME_LIMIT=11, ST_HOME_COMPLETE=12, ST_HOLD_POSITION=13, ST_BRAKE_ENGAGE=14, ST_DRIVE_DISABLE=15, ST_FAULT=16, ST_FAULT_IDLE=17, ST_RECOVERY=18, ST_RECOVERY_POSITION=19, ST_RECOVERY_VELOCITY=20),
    MC_Direction=SimpleNamespace(Positive_Direction=0, Shortest_Way=1, Negative_Direction=2, Current_Direction=3),
    Y_ControlMode=SimpleNamespace(NoControlMode=0, PositionMode=1, VelocityTLMode=2, TorqueVLMode=3, PositionTrqFFMode=4, VelocityTrqFFMode=5),
)



def new_ST_AxisLimits():
    s = SimpleNamespace()
    s.PosMax = float(0)
    s.PosMin = float(0)
    s.VelMax = float(0)
    s.VelMin = float(0)
    s.TorqueMax = float(0)
    s.TorqueMin = float(0)
    s.AccelMax = float(0)
    s.DecelMax = float(0)
    return s


def new_ST_CmdDirectControl():
    s = SimpleNamespace()
    s.Enable = False
    s.ControlMode = 0
    s.Position = float(0)
    s.Velocity = float(0)
    s.Torque = float(0)
    s.Acceleration = float(0)
    s.FilterTimeConstant = float(0)
    return s


def new_ST_CmdJog():
    s = SimpleNamespace()
    s.Forward = False
    s.Reverse = False
    s.Velocity = float(0)
    s.Acceleration = float(0)
    s.Deceleration = float(0)
    return s


def new_ST_CmdMoveAbsolute():
    s = SimpleNamespace()
    s.Execute = False
    s.Position = float(0)
    s.Velocity = float(0)
    s.Acceleration = float(0)
    s.Deceleration = float(0)
    s.Direction = 0
    return s


def new_ST_CmdPower():
    s = SimpleNamespace()
    s.Enable = False
    s.EnablePositive = False
    s.EnableNegative = False
    return s


def new_ST_CmdReset():
    s = SimpleNamespace()
    s.Execute = False
    return s


def new_ST_CmdSetPosition():
    s = SimpleNamespace()
    s.Execute = False
    s.Position = float(0)
    s.Mode = False
    return s


def new_ST_CmdStop():
    s = SimpleNamespace()
    s.Execute = False
    s.Deceleration = float(0)
    return s


def new_ST_HomingParams():
    s = SimpleNamespace()
    s.VelFast = float(0)
    s.VelSlow = float(0)
    s.BackoffDist = float(0)
    s.SetPosition = float(0)
    s.Timeout = 0
    s.Direction = 0
    return s


def new_ST_StaDirectControl():
    s = SimpleNamespace()
    s.Active = False
    s.Error = False
    return s


def new_ST_StaJog():
    s = SimpleNamespace()
    s.InVelocity = False
    s.Done = False
    s.Busy = False
    s.Error = False
    s.ErrorID = 0
    return s


def new_ST_StaMoveAbsolute():
    s = SimpleNamespace()
    s.Done = False
    s.Busy = False
    s.Active = False
    s.CommandAborted = False
    s.Error = False
    return s


def new_ST_StaPower():
    s = SimpleNamespace()
    s.Status = False
    s.Error = False
    return s


def new_ST_StaReset():
    s = SimpleNamespace()
    s.Done = False
    s.Busy = False
    s.Error = False
    return s


def new_ST_StaSetPosition():
    s = SimpleNamespace()
    s.Done = False
    s.Busy = False
    s.Error = False
    return s


def new_ST_StaStop():
    s = SimpleNamespace()
    s.Done = False
    s.Busy = False
    s.Error = False
    return s
