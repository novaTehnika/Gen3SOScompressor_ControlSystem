# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import TON, new_TON, E
from FB_IntToMode import FB_IntToMode, new_FB_IntToMode


def new_FB_ModeDecoder():
    s = SimpleNamespace()
    s.Enable = False
    s.ModeCode = 0
    s.DebounceTime = 0.05
    s.DecodedMode = 0
    s.ModeValue = 0
    s.ModeStable = False
    s.ModeChanged = False
    s.nPreviousValue = 0
    s.nFilteredCode = 0
    s.nPendingCode = 0
    s.nLastRaw = 0
    s.bFirstScan = True
    s.tDebounce = new_TON()
    s.bDebounceIN = False
    s.fbIntToMode = new_FB_IntToMode()
    return s


def FB_ModeDecoder(s, dt):
    if (not s.Enable):  # :37
        s.DecodedMode = E.E_OperatingMode.MODE_IDLE  # :39
        s.ModeValue = 0  # :40
        s.ModeStable = False  # :41
        s.ModeChanged = False  # :42
        s.nPreviousValue = 0  # :43
        return  # :44
    if s.bFirstScan:  # :49
        s.bFirstScan = False  # :50
        s.nFilteredCode = s.ModeCode  # :51
        s.nPendingCode = s.ModeCode  # :52
        s.nLastRaw = s.ModeCode  # :53
        s.bDebounceIN = False  # :54
    elif (s.ModeCode != s.nLastRaw):  # :55
        s.nPendingCode = s.ModeCode  # :56
        s.bDebounceIN = False  # :57
    else:  # :58
        s.bDebounceIN = ((s.nPendingCode != s.nFilteredCode))  # :59
    s.nLastRaw = s.ModeCode  # :61
    s.tDebounce.IN = s.bDebounceIN  # :63
    s.tDebounce.PT = s.DebounceTime  # :63
    TON(s.tDebounce, dt)  # :63
    if s.tDebounce.Q:  # :65
        s.nFilteredCode = s.nPendingCode  # :66
        s.bDebounceIN = False  # :67
    s.fbIntToMode.Value = s.nFilteredCode  # :72
    FB_IntToMode(s.fbIntToMode, dt)  # :72
    s.DecodedMode = s.fbIntToMode.Mode  # :73
    if (s.nFilteredCode <= 7):  # :75
        s.ModeValue = (s.nFilteredCode)  # :76
    else:  # :77
        s.ModeValue = 0  # :78
    s.ModeStable = ((s.ModeValue == s.nPreviousValue))  # :82
    s.ModeChanged = ((s.ModeValue != s.nPreviousValue))  # :85
    s.nPreviousValue = s.ModeValue  # :88
