"""Pressure measurement chain, chamber gauge pressure in, measured pressure out (Pa).

SlaveSensor: the 4-20 mA loop and the slave's own blocks. Gauge pressure maps
to loop current over the configured transducer range, the current across the
shunt gives volts, white noise is added, and the volts pass through the
generated FB_AnalogInputFilter and FB_PressureInput. The blocks are called in
PRG_Main's order and wired the way it wires them: the controller reads
G_sysActualPressure from the previous scan's FB_PressureInput, which scaled
the filter output of the scan before that, so the reading the controller sees
is the filter output of two scans earlier.

LegacySensor: median of medN samples then a first-order filter, directly in
pressure, with the reading available in the scan it is sampled (the slave
chain's output is this one delayed by two scans, up to scaling).

`read(P, z)` takes the chamber pressure and a standard normal draw z and
returns the measurement for this scan's controller call.
"""

from collections import deque

import slave  # noqa: F401  (puts the generated blocks on sys.path)
from FB_AnalogInputFilter import FB_AnalogInputFilter, new_FB_AnalogInputFilter
from FB_PressureInput import FB_PressureInput, new_FB_PressureInput


def filter_alpha(cfg):
    """rAnalogFilterAlpha as PRG_Main computes it at first scan."""
    tau = round(cfg['G_cfgAnalogFilterTimeConst'] * 1000.0) / 1000.0     # TIME_TO_DINT is in ms
    Ts = cfg['G_cfgScanTime']
    alpha = Ts / (tau + Ts) if cfg['G_cfgAnalogFilterTimeConst'] > 0 else 1.0
    return min(max(alpha, 0.0), 1.0)


class SlaveSensor:
    def __init__(self, p, cfg, P0=0.0):
        """cfg: effective slave configuration (Params.cfg(...)); p supplies the noise."""
        self.cfg = cfg
        self.dt = cfg['G_cfgScanTime']
        self.alpha = filter_alpha(cfg)
        rmin, rmax = cfg['G_cfgPressureRangeMin'], cfg['G_cfgPressureRangeMax']
        zero, span = cfg['G_cfgPressureZeroMa'], cfg['G_cfgPressureSpanMa']
        shunt = cfg['G_cfgPressureShuntOhms']
        self.v_zero = zero * shunt / 1000.0                       # V at RangeMin
        self.v_per_pa = span / (rmax - rmin) * shunt / 1000.0 / 1e6
        self.noise_v = p.noise * self.v_per_pa
        self.v_offset = self.v_zero - rmin * 1e6 * self.v_per_pa
        self.filt = new_FB_AnalogInputFilter()
        self.filt.Enable = True
        self.filt.Alpha = self.alpha
        self.filt.MedianFilterSize = int(cfg['G_cfgAnalogMedianSize'])
        self.inp = i = new_FB_PressureInput()
        i.Enable = True
        i.ShuntOhms = shunt
        i.ZeroMa, i.SpanMa = zero, span
        i.RangeMin, i.RangeMax = rmin, rmax
        i.MinMa, i.MaxMa = cfg['G_cfgPressureSensorMinMa'], cfg['G_cfgPressureSensorMaxMa']
        i.FaultTime = cfg['G_cfgPressureSensorFaultTime']
        for _ in range(3):                    # the blocks hold a settled reading when the mode is entered
            self.read(P0, 0.0)

    @property
    def fault(self):
        return self.inp.SensorFault

    def read(self, P, z):
        filt, inp, dt = self.filt, self.inp, self.dt
        actual = inp.PressureMPa                 # G_sysActualPressure
        volts = filt.FilteredOutput              # rFilteredPressureVolts
        filt.RawInput = P * self.v_per_pa + self.v_offset + self.noise_v * z
        FB_AnalogInputFilter(filt, dt)
        inp.Volts = volts
        FB_PressureInput(inp, dt)
        return actual * 1e6


class LegacySensor:
    def __init__(self, p, cfg, P0=0.0):
        tau = cfg['G_cfgAnalogFilterTimeConst']
        self.alpha = cfg['G_cfgScanTime'] / (tau + cfg['G_cfgScanTime'])
        self.noise = p.noise
        n = int(cfg['G_cfgAnalogMedianSize'])
        self.buf = deque([P0] * n, maxlen=n)
        self.mid = n // 2
        self.Pf = P0
        self.fault = False

    def read(self, P, z):
        buf = self.buf
        buf.append(P + self.noise * z)
        med = sorted(buf)[self.mid]
        self.Pf += self.alpha * (med - self.Pf)
        return self.Pf
