# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import TON, new_TON


def new_FB_CommWatchdog():
    s = SimpleNamespace()
    s.Heartbeat = 0
    s.Timeout = 0
    s.Lost = False
    s.bArmed = False
    s.bSame = False
    s.nLast = 0
    s.tonStale = new_TON()
    return s


def FB_CommWatchdog(s, dt):
    s.bSame = ((s.Heartbeat == s.nLast))  # :29
    s.nLast = s.Heartbeat  # :30
    if (not s.bSame):  # :31
        s.bArmed = True  # :32
    s.tonStale.IN = s.bSame  # :35
    s.tonStale.PT = s.Timeout  # :35
    TON(s.tonStale, dt)  # :35
    s.Lost = (s.bArmed and s.tonStale.Q)  # :36
