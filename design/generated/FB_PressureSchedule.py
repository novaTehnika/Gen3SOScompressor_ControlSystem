# Generated from slave-src by tools/st_translate.py --target py - do not edit.
from types import SimpleNamespace
from st_runtime import st_div, st_limit, st_max, st_min, st_sqrt


def new_FB_PressureSchedule():
    s = SimpleNamespace()
    s.Enable = False
    s.Reset = False
    s.Target = float(0)
    s.Actual = float(0)
    s.Position = float(0)
    s.EndOfTravel = float(0)
    s.VelMax = float(0)
    s.DecelMax = float(0)
    s.StopVelMin = float(0)
    s.CycleTime = float(0)
    s.FilterTime = float(0)
    s.MedianSize = 0
    s.DriveDeadTime = float(0)
    s.DriveLag = float(0)
    s.Crossover = float(0)
    s.KiRatio = float(0)
    s.KpMin = float(0)
    s.KpMax = float(0)
    s.PistonArea = float(0)
    s.SalineVolume = float(0)
    s.PassagesVolume = float(0)
    s.HoseVolume = float(0)
    s.HoseCompliance = float(0)
    s.HoseOpen = False
    s.BulkModulus = float(0)
    s.OxygenRatio = float(0)
    s.Henry = float(0)
    s.GasTemp = float(0)
    s.Patm = float(0)
    s.ArriveBand = float(0)
    s.Kp = float(0)
    s.Ki = float(0)
    s.VelLimit = float(0)
    s.Stiffness = float(0)
    s.IntegScale = float(0)
    s.IntegHold = False
    s.KEst = float(0)
    s.KEstTarget = float(0)
    s.Lag = float(0)
    s.ParamError = False
    s.bArrived = False
    s.rPRef = float(0.0)
    s.rKiPrev = float(0.0)
    s.rMedScans = float(0)
    s.rPAbs = float(0)
    s.rArea = float(0)
    s.rHoseVolume = float(0)
    s.rHoseComp = float(0)
    s.rVch = float(0)
    s.rVlCfg = float(0)
    s.rVl = float(0)
    s.rNCharge = float(0)
    s.rNSat = float(0)
    s.rNGas = float(0)
    s.rRT = float(0)
    s.rVGas = float(0)
    s.rC = float(0)
    s.rPAbsT = float(0)
    s.rVGasT = float(0)
    s.rCT = float(0)
    s.rKSched = float(0)
    s.rErr = float(0)
    s.rDist = float(0)
    s.rStep = float(0)
    s.rVelStop = float(0)
    return s


def FB_PressureSchedule(s, dt):
    s.ParamError = (((((((((((((((((((((((s.VelMax <= 0.0)) or ((s.DecelMax <= 0.0))) or ((s.StopVelMin < 0.0))) or ((s.CycleTime <= 0.0))) or ((s.FilterTime < 0.0))) or ((s.DriveDeadTime < 0.0))) or ((s.DriveLag < 0.0))) or ((s.Crossover <= 0.0))) or ((s.KiRatio < 0.0))) or ((s.KpMin <= 0.0))) or ((s.KpMax < s.KpMin))) or ((s.PistonArea <= 0.0))) or ((not ((s.SalineVolume > 0.0))))) or ((s.PassagesVolume <= 0.0))) or ((s.HoseVolume < 0.0))) or ((s.HoseCompliance < 0.0))) or ((s.BulkModulus <= 0.0))) or ((not ((s.OxygenRatio >= 0.0))))) or ((s.Henry < 0.0))) or ((s.GasTemp <= 0.0))) or ((s.Patm <= 0.0))) or ((s.ArriveBand < 0.0)))  # :125
    if ((((not s.Enable)) or s.Reset) or s.ParamError):  # :138
        s.bArrived = False  # :139
        s.rPRef = 0.0  # :140
        s.rKiPrev = 0.0  # :141
    if (((not s.Enable)) or s.ParamError):  # :144
        s.Kp = 0.0  # :145
        s.Ki = 0.0  # :146
        s.VelLimit = 0.0  # :147
        s.Stiffness = 0.0  # :148
        s.IntegScale = 1.0  # :149
        s.IntegHold = False  # :150
        s.KEst = 0.0  # :151
        s.KEstTarget = 0.0  # :152
        s.Lag = 0.0  # :153
        return  # :154
    if (s.MedianSize >= 5):  # :160
        s.rMedScans = 2.0  # :161
    else:  # :162
        s.rMedScans = 1.0  # :163
    s.Lag = (((s.FilterTime + (((s.rMedScans + 3.0)) * s.CycleTime)) + s.DriveDeadTime) + s.DriveLag)  # :165
    s.rPAbs = ((st_max(s.Actual, 0.0) * 1.0E6) + s.Patm)  # :170
    s.rPRef = st_max(s.rPRef, s.rPAbs)  # :171
    s.rArea = (s.PistonArea / 1.0E6)  # :172
    if s.HoseOpen:  # :173
        s.rHoseVolume = s.HoseVolume  # :174
        s.rHoseComp = s.HoseCompliance  # :175
    else:  # :176
        s.rHoseVolume = 0.0  # :177
        s.rHoseComp = 0.0  # :178
    s.rVch = (st_max((s.rArea * ((((s.EndOfTravel - s.Position)) / 1.0E3))), 0.0) + (((s.PassagesVolume + s.rHoseVolume)) / 1.0E6))  # :180
    s.rVlCfg = (s.SalineVolume / 1.0E6)  # :182
    s.rVl = st_min(s.rVlCfg, s.rVch)  # :183
    s.rNCharge = st_div(((s.OxygenRatio * s.rVlCfg) * s.Patm), ((8.314 * 273.15)))  # :184
    s.rNSat = ((s.Henry * s.rPRef) * s.rVlCfg)  # :185
    s.rNGas = st_max((s.rNCharge - s.rNSat), 0.0)  # :186
    s.rRT = (8.314 * s.GasTemp)  # :187
    s.rVGas = st_div((s.rNGas * s.rRT), s.rPAbs)  # :188
    s.rC = ((st_div(s.rVGas, s.rPAbs) + st_div(s.rVl, s.BulkModulus)) + s.rHoseComp)  # :189
    s.KEst = (st_div(s.rArea, s.rC) / 1.0E9)  # :190
    s.rPAbsT = ((st_max(st_max(s.Actual, s.Target), 0.0) * 1.0E6) + s.Patm)  # :191
    s.rVGasT = st_div((s.rNGas * s.rRT), s.rPAbsT)  # :192
    s.rCT = ((st_div(s.rVGasT, s.rPAbsT) + st_div(s.rVl, s.BulkModulus)) + s.rHoseComp)  # :193
    s.KEstTarget = (st_div(s.rArea, s.rCT) / 1.0E9)  # :194
    s.rKSched = st_max(s.KEst, s.KEstTarget)  # :195
    s.Kp = st_limit(s.KpMin, st_div(s.Crossover, s.rKSched), s.KpMax)  # :200
    s.Ki = (s.KiRatio * s.Kp)  # :201
    if ((((s.rKiPrev > 0.0)) and ((s.Ki > 0.0))) and ((s.Ki != s.rKiPrev))):  # :202
        s.IntegScale = st_div(s.rKiPrev, s.Ki)  # :203
    else:  # :204
        s.IntegScale = 1.0  # :205
    s.rKiPrev = s.Ki  # :207
    s.rErr = (s.Target - s.Actual)  # :212
    s.rDist = st_div(abs(s.rErr), s.rKSched)  # :213
    s.rStep = (s.DecelMax * s.Lag)  # :214
    s.rVelStop = ((-s.rStep) + st_sqrt(((s.rStep * s.rStep) + ((2.0 * s.DecelMax) * s.rDist))))  # :215
    s.VelLimit = st_min(s.VelMax, st_max(s.StopVelMin, s.rVelStop))  # :216
    s.Stiffness = 1.0E-6  # :217
    s.IntegHold = (not s.bArrived)  # :222
    if (((not s.bArrived)) and ((abs(s.rErr) < s.ArriveBand))):  # :223
        s.bArrived = True  # :224
