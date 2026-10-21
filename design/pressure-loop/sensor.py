"""
sensor.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module models the pressure measurement chain from the chamber gauge
pressure to the value the pressure controller reads. Two models share
one interface: read(P, z) takes the chamber gauge pressure P [Pa] and a
standard normal random number z for the noise, and returns the measured
pressure [Pa] that the controller uses in the present scan.

SlaveSensor reproduces the slave's chain. The pressure transducer maps
gauge pressure to a 4-20 mA loop current over its configured range, the
current flows through a shunt resistor, and the slave samples the shunt
voltage once per scan with white noise added (README eq. (15)). The
voltage then passes through the slave's own function blocks, translated
from Structured Text: FB_AnalogInputFilter (a median filter followed by
a first-order low-pass filter, eqs. (16) and (17)) and FB_PressureInput
(scaling from volts to MPa). The blocks are called in the order PRG_Main
calls them. PRG_Main reads the outputs of both blocks at the top of the
scan and calls the blocks at the end, so the controller sees the filter
output from two scans earlier (eq. (18)).

LegacySensor is the earlier, simpler chain. It applies the median and
the first-order filter directly to the pressure, and the reading is
available in the scan it is sampled. Its output equals the slave chain's
output two scans earlier, apart from the scaling.

FILE DEPENDENCY:
./
    slave.py
../generated/ (translated by tools/st2py.py from slave-src/FB/)
    FB_AnalogInputFilter.py
    FB_PressureInput.py
"""

from collections import deque

import slave  # noqa: F401  (puts the generated blocks on sys.path)
from FB_AnalogInputFilter import (FB_AnalogInputFilter,
                                  new_FB_AnalogInputFilter)
from FB_PressureInput import FB_PressureInput, new_FB_PressureInput


# %% ===== FILTER COEFFICIENT ============================================
def filter_alpha(cfg):
    """Return the first-order filter coefficient alpha as PRG_Main
    computes it on its first scan, eq. (16).

    alpha = Ts / (tau + Ts), limited to [0, 1], and 1 (no filtering) when
    the time constant is zero. PRG_Main converts the TIME value to whole
    milliseconds first, so tau is rounded to 1 ms here.
    """
    # [s -> ms -> s] round the time constant to whole milliseconds
    tau = round(cfg['G_cfgAnalogFilterTimeConst'] * 1000.0) / 1000.0
    Ts = cfg['G_cfgScanTime']                                   # [s]
    if cfg['G_cfgAnalogFilterTimeConst'] > 0:
        alpha = Ts / (tau + Ts)
    else:
        alpha = 1.0
    return min(max(alpha, 0.0), 1.0)


# %% ===== SLAVE MEASUREMENT CHAIN =======================================
class SlaveSensor:
    """The slave's measurement chain: transducer, shunt, sampling noise,
    FB_AnalogInputFilter and FB_PressureInput.

    Arguments:
      par  Params; supplies the noise level.
      cfg  Effective slave configuration, from Params.cfg().
      P0   [Pa] Pressure at which the filters start settled.
    """

    def __init__(self, par, cfg, P0=0.0):
        self.cfg = cfg
        self.dt = cfg['G_cfgScanTime']                          # [s]
        self.alpha = filter_alpha(cfg)

        # Transducer and shunt, eq. (15). The range is in MPa, the loop
        # currents in mA and the shunt in ohms.
        P_min = cfg['G_cfgPressureRangeMin']                    # [MPa]
        P_max = cfg['G_cfgPressureRangeMax']                    # [MPa]
        I_zero = cfg['G_cfgPressureZeroMa']                     # [mA]
        I_span = cfg['G_cfgPressureSpanMa']                     # [mA]
        R_shunt = cfg['G_cfgPressureShuntOhms']                 # [ohm]
        # [V] Shunt voltage at the bottom of the range:
        # [mA * ohm / 1000 -> V]
        V_zero = I_zero * R_shunt / 1000.0
        # [V/Pa] Voltage gain: [mA/MPa * ohm / 1000 -> V/MPa],
        # then [V/MPa / 1e6 -> V/Pa]
        self.V_per_Pa = I_span / (P_max - P_min) * R_shunt / 1000.0 / 1e6
        # [V] Shunt voltage at zero gauge pressure, [MPa * 1e6 -> Pa]
        self.V_offset = V_zero - P_min * 1e6 * self.V_per_Pa
        # [V] RMS noise per sample, the pressure noise times the gain
        self.noise_V = par.noise * self.V_per_Pa

        # FB_AnalogInputFilter, configured as PRG_Main configures it
        self.filt = new_FB_AnalogInputFilter()
        self.filt.Enable = True
        self.filt.Alpha = self.alpha
        self.filt.MedianFilterSize = int(cfg['G_cfgAnalogMedianSize'])

        # FB_PressureInput, configured as PRG_Main configures it
        self.inp = new_FB_PressureInput()
        self.inp.Enable = True
        self.inp.ShuntOhms = R_shunt
        self.inp.ZeroMa = I_zero
        self.inp.SpanMa = I_span
        self.inp.RangeMin = P_min
        self.inp.RangeMax = P_max
        self.inp.MinMa = cfg['G_cfgPressureSensorMinMa']
        self.inp.MaxMa = cfg['G_cfgPressureSensorMaxMa']
        self.inp.FaultTime = cfg['G_cfgPressureSensorFaultTime']

        # The blocks hold a settled reading when the pressure mode is
        # entered, so the chain is primed with three noise-free scans.
        for _ in range(3):
            self.read(P0, 0.0)

    @property
    def fault(self):
        """True while FB_PressureInput reports a sensor fault."""
        return self.inp.SensorFault

    def read(self, P, z):
        """Sample pressure P [Pa] with noise draw z and return the reading
        [Pa] the controller uses in this scan."""
        # Outputs of the previous scan's block calls, read at the top of
        # the scan as PRG_Main reads them, eq. (18). PRG_Main names them
        # G_sysActualPressure and rFilteredPressureVolts.
        P_actual = self.inp.PressureMPa                         # [MPa]
        V_filtered = self.filt.FilteredOutput                   # [V]

        # Sample the shunt voltage with noise and run the filter,
        # eqs. (15) to (17)
        self.filt.RawInput = (P * self.V_per_Pa + self.V_offset
                              + self.noise_V * z)                # [V]
        FB_AnalogInputFilter(self.filt, self.dt)

        # Scale the previous filter output to pressure
        self.inp.Volts = V_filtered
        FB_PressureInput(self.inp, self.dt)

        return P_actual * 1e6                   # [MPa -> Pa]


# %% ===== LEGACY MEASUREMENT CHAIN ======================================
class LegacySensor:
    """Median and first-order filter applied directly to the pressure,
    with no pipeline delay.

    Arguments are the same as for SlaveSensor.
    """

    def __init__(self, par, cfg, P0=0.0):
        tau = cfg['G_cfgAnalogFilterTimeConst']                 # [s]
        Ts = cfg['G_cfgScanTime']                               # [s]
        self.alpha = Ts / (tau + Ts)            # eq. (16), unrounded
        self.noise = par.noise                  # [Pa] RMS per sample
        n = int(cfg['G_cfgAnalogMedianSize'])
        self.window = deque([P0] * n, maxlen=n)  # [Pa] recent samples
        self.i_mid = n // 2                     # index of the median
        self.P_filt = P0                        # [Pa] filter output
        self.fault = False

    def read(self, P, z):
        """Sample pressure P [Pa] with noise draw z and return the
        filtered reading [Pa]."""
        self.window.append(P + self.noise * z)
        P_median = sorted(self.window)[self.i_mid]
        # First-order low-pass filter, eq. (17)
        self.P_filt += self.alpha * (P_median - self.P_filt)
        return self.P_filt
