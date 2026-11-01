# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import TON, new_TON


def new_FB_DigitalInputFilter():
    s = SimpleNamespace()
    s.RawInput = False
    s.DebounceTime = 0.01
    s.FilteredOutput = False
    s.RisingEdge = False
    s.FallingEdge = False
    s.Stable = False
    s.RawPassthrough = False
    s.tDebounce = new_TON()
    s.bDebounceIN = False
    s.bLastRaw = False
    s.bLastFiltered = False
    s.bPendingState = False
    s.bFirstScan = True
    return s


def FB_DigitalInputFilter(s, dt):
    s.RawPassthrough = s.RawInput  # :40
    if s.bFirstScan:  # :43
        s.FilteredOutput = s.RawInput  # :44
        s.bLastRaw = s.RawInput  # :45
        s.bLastFiltered = s.RawInput  # :46
        s.bPendingState = s.RawInput  # :47
        s.Stable = True  # :48
        s.RisingEdge = False  # :49
        s.FallingEdge = False  # :50
        s.bDebounceIN = False  # :51
        s.bFirstScan = False  # :52
    else:  # :53
        if (s.RawInput != s.bLastRaw):  # :55
            s.bPendingState = s.RawInput  # :56
            s.bDebounceIN = False  # :57
        elif (s.bPendingState != s.FilteredOutput):  # :58
            s.bDebounceIN = True  # :59
            s.Stable = False  # :60
        else:  # :61
            s.bDebounceIN = False  # :62
            s.Stable = True  # :63
        s.bLastRaw = s.RawInput  # :66
    s.tDebounce.IN = s.bDebounceIN  # :70
    s.tDebounce.PT = s.DebounceTime  # :70
    TON(s.tDebounce, dt)  # :70
    if (s.tDebounce.Q and (not s.bFirstScan)):  # :72
        s.FilteredOutput = s.bPendingState  # :73
        s.bDebounceIN = False  # :74
        s.Stable = True  # :75
    s.RisingEdge = (s.FilteredOutput and (not s.bLastFiltered))  # :79
    s.FallingEdge = ((not s.FilteredOutput) and s.bLastFiltered)  # :80
    s.bLastFiltered = s.FilteredOutput  # :81
