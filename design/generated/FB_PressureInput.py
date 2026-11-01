# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import TON, new_TON, st_div


def new_FB_PressureInput():
    s = SimpleNamespace()
    s.Enable = False
    s.Volts = float(0)
    s.ShuntOhms = float(0)
    s.ZeroMa = float(0)
    s.SpanMa = float(0)
    s.RangeMin = float(0)
    s.RangeMax = float(0)
    s.MinMa = float(0)
    s.MaxMa = float(0)
    s.FaultTime = 0
    s.PressureMPa = float(0)
    s.LoopMa = float(0)
    s.OutOfRange = False
    s.SensorFault = False
    s.ParamError = False
    s.tonOutOfRange = new_TON()
    return s


def FB_PressureInput(s, dt):
    if (not s.Enable):  # :41
        s.PressureMPa = 0.0  # :42
        s.LoopMa = 0.0  # :43
        s.OutOfRange = False  # :44
        s.SensorFault = False  # :45
        s.ParamError = False  # :46
        s.tonOutOfRange.IN = False  # :47
        s.tonOutOfRange.PT = s.FaultTime  # :47
        TON(s.tonOutOfRange, dt)  # :47
        return  # :48
    s.ParamError = (((s.ShuntOhms <= 0.0)) or ((s.SpanMa <= 0.0)))  # :51
    if s.ParamError:  # :52
        s.PressureMPa = 0.0  # :54
        s.LoopMa = 0.0  # :55
        s.OutOfRange = True  # :56
    else:  # :57
        s.LoopMa = (st_div(s.Volts, s.ShuntOhms) * 1000.0)  # :58
        s.PressureMPa = (s.RangeMin + (st_div(((s.LoopMa - s.ZeroMa)), s.SpanMa) * ((s.RangeMax - s.RangeMin))))  # :59
        s.OutOfRange = (((s.LoopMa < s.MinMa)) or ((s.LoopMa > s.MaxMa)))  # :60
    s.tonOutOfRange.IN = s.OutOfRange  # :63
    s.tonOutOfRange.PT = s.FaultTime  # :63
    TON(s.tonOutOfRange, dt)  # :63
    s.SensorFault = s.tonOutOfRange.Q  # :64
