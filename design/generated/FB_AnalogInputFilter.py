# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import st_limit, st_max, st_min, st_range


def new_FB_AnalogInputFilter():
    s = SimpleNamespace()
    s.Enable = True
    s.RawInput = float(0)
    s.Alpha = float(0)
    s.MedianFilterSize = 3
    s.FilteredOutput = float(0)
    s.aMedianBuffer = [0.0] * 5
    s.nMedianIndex = 0
    s.rFilteredValue = float(0.0)
    s.rAlpha = float(0)
    s.bInitialized = False
    s.i = 0
    s.rS0 = float(0)
    s.rS1 = float(0)
    s.rS2 = float(0)
    s.rS3 = float(0)
    s.rS4 = float(0)
    s.rTemp = float(0)
    s.rMedianResult = float(0)
    s.nSize = 0
    return s


def FB_AnalogInputFilter(s, dt):
    if (not s.Enable):  # :53
        s.FilteredOutput = s.RawInput  # :55
        return  # :56
    s.rAlpha = st_limit(0.0, s.Alpha, 1.0)  # :60
    if (not s.bInitialized):  # :66
        for st_k_ in st_range(0, 4):  # :68
            s.i = st_k_
            s.aMedianBuffer[s.i] = s.RawInput  # :69
        s.rFilteredValue = s.RawInput  # :71
        s.bInitialized = True  # :72
    if (s.MedianFilterSize >= 5):  # :80
        s.nSize = 5  # :81
    else:  # :82
        s.nSize = 3  # :83
    s.aMedianBuffer[s.nMedianIndex] = s.RawInput  # :87
    s.nMedianIndex = (s.nMedianIndex + 1)  # :88
    if (s.nMedianIndex >= s.nSize):  # :89
        s.nMedianIndex = 0  # :90
    s.rS0 = s.aMedianBuffer[0]  # :96
    s.rS1 = s.aMedianBuffer[1]  # :97
    s.rS2 = s.aMedianBuffer[2]  # :98
    if (s.nSize == 3):  # :100
        s.rTemp = st_min(s.rS0, s.rS1)  # :102
        s.rS1 = st_max(s.rS0, s.rS1)  # :102
        s.rS0 = s.rTemp  # :102
        s.rTemp = st_min(s.rS1, s.rS2)  # :103
        s.rS2 = st_max(s.rS1, s.rS2)  # :103
        s.rS1 = s.rTemp  # :103
        s.rTemp = st_min(s.rS0, s.rS1)  # :104
        s.rS1 = st_max(s.rS0, s.rS1)  # :104
        s.rS0 = s.rTemp  # :104
        s.rMedianResult = s.rS1  # :105
    else:  # :106
        s.rS3 = s.aMedianBuffer[3]  # :107
        s.rS4 = s.aMedianBuffer[4]  # :108
        s.rTemp = st_min(s.rS0, s.rS1)  # :111
        s.rS1 = st_max(s.rS0, s.rS1)  # :111
        s.rS0 = s.rTemp  # :111
        s.rTemp = st_min(s.rS2, s.rS3)  # :112
        s.rS3 = st_max(s.rS2, s.rS3)  # :112
        s.rS2 = s.rTemp  # :112
        if (s.rS0 > s.rS2):  # :117
            s.rTemp = s.rS0  # :118
            s.rS0 = s.rS2  # :118
            s.rS2 = s.rTemp  # :118
            s.rTemp = s.rS1  # :119
            s.rS1 = s.rS3  # :119
            s.rS3 = s.rTemp  # :119
        s.rTemp = st_min(s.rS1, s.rS4)  # :125
        s.rS4 = st_max(s.rS1, s.rS4)  # :125
        s.rS1 = s.rTemp  # :125
        if (s.rS1 > s.rS2):  # :126
            s.rTemp = s.rS1  # :127
            s.rS1 = s.rS2  # :127
            s.rS2 = s.rTemp  # :127
            s.rTemp = s.rS4  # :128
            s.rS4 = s.rS3  # :128
            s.rS3 = s.rTemp  # :128
        s.rMedianResult = st_min(s.rS4, s.rS2)  # :130
    s.rFilteredValue = (((s.rAlpha * s.rMedianResult)) + ((((1.0 - s.rAlpha)) * s.rFilteredValue)))  # :138
    s.FilteredOutput = s.rFilteredValue  # :144
