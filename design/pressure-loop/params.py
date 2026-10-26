"""
params.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module defines Params, the set of plant, actuator, sensor and
simulation parameters used by every study. All values are in SI units
unless a comment states otherwise.

The parameters describe four parts of the model:

  Chamber     A piston of area A compresses a chamber that holds liquid
              of effective bulk modulus K and oxygen gas. The gas is
              isothermal and dissolves into the liquid according to
              Henry's law with time constant tauDiss. The chamber
              stiffness dP/dx therefore rises from about 0.002 MPa/mm
              when the chamber is gas-rich to 10 to 25 MPa/mm once the
              gas has dissolved.
  Actuator    In velocity mode the drive limits the rate of change of the
              commanded velocity, follows it with lag tauV, and stalls
              when the chamber force reaches the torque limit Fcap. In
              torque mode the commanded force drives the reflected mass
              mr against the chamber force, viscous friction b and
              Coulomb friction Fc.
  Sensor      A 4-20 mA pressure transducer drives a current through a
              shunt resistor, and the slave samples the voltage once per
              scan. The noise parameter sets the white noise on each
              sample, referred to pressure.
  Simulation  The plant takes nsub integration steps per slave scan
              using the method named by solver.

The slave's own settings (controller gains, velocity clamp, slew limits,
stop law, filter, median size, scan time, shunt and transducer range) are
not stored here. They are read from the slave's global variable list
(GVL) through slave.slave_config(), and the field slave_cfg overrides
individual G_cfg* entries for a study.

FILE DEPENDENCY:
./
    slave.py
"""

import math
from dataclasses import dataclass, field, fields, replace

from slave import slave_config


@dataclass(frozen=True)
class Params:
    """Plant, actuator, sensor and simulation parameters.

    Params is immutable. Use with_() to make a copy with changed values.
    """

    # ----- Chamber -------------------------------------------------------
    # [m^2] Piston area, for a bore of 2.602 in
    A: float = 3.431e-3
    # [J/(mol K)] Universal gas constant
    R: float = 8.314
    # [K] Gas temperature, held constant (isothermal gas)
    T: float = 293.0
    # [Pa] Atmospheric pressure, the reference for gauge pressure
    Patm: float = 101325.0
    # [Pa] Effective bulk modulus of the liquid. Water alone is 2.2 GPa;
    # hose and seal compliance lower the effective value.
    K: float = 1.0e9
    # [m^3] Liquid volume at the start of a run
    Vl0: float = 300e-6
    # [m^3] Gas volume, measured at atmospheric pressure, in the chamber
    # at the start of a gas-rich run
    Vg0: float = 200e-6
    # [mol/(m^3 Pa)] Henry's-law constant for oxygen in water
    kH: float = 1.28e-5
    # [s] Time constant of gas dissolution
    tauDiss: float = 30.0

    # ----- Discharge valve -----------------------------------------------
    # These apply to the valve scenario.
    # [m^3/s] Outflow once the valve is fully open
    Qf: float = 1e-6
    # [s] Valve opening time; 0 opens the valve in one step
    Topen: float = 0.3
    # [s] Simulated duration of the valve scenario
    valveT: float = 8.0

    # ----- Simulation ----------------------------------------------------
    # [-] Plant integration steps per slave scan
    nsub: int = 5
    # Integration method from solver.METHODS: 'heun' or 'euler'
    solver: str = 'heun'

    # ----- Actuator ------------------------------------------------------
    # [s] Time constant of the drive's velocity loop
    tauV: float = 0.02
    # [N] Piston force at 100 % rated motor torque. The motor gives
    # 1.27 N m through a 70:1 gearbox and a ball screw of 5.08 mm lead
    # at 85 % efficiency:
    #   Fcap = 1.27 [N m] * 2 pi [rad/rev] * 70 / 5.08e-3 [m/rev] * 0.85
    Fcap: float = 1.27 * 2 * math.pi * 70 / 5.08e-3 * 0.85
    # [kg] Motor and gearbox inertia reflected to the piston
    mr: float = 5e4
    # [N s/m] Viscous friction coefficient
    b: float = 5e3
    # [N] Coulomb friction of the seals and screw
    Fc: float = 500.0
    # [m/s] Velocity over which the Coulomb friction is smoothed from
    # -Fc to +Fc, which avoids a discontinuity at zero velocity
    veps: float = 1e-5

    # ----- Sensor --------------------------------------------------------
    # [Pa] RMS noise per sample, referred to pressure
    noise: float = 0.005e6
    # [s] Time constant of the derivative filter in the legacy
    # controller
    Tf: float = 0.02

    # ----- Metrics -------------------------------------------------------
    # [Pa] Half-width of the band used for the settling time
    tol: float = 0.1e6

    # ----- Model options -------------------------------------------------
    # Setting both options to False gives the earlier model: a fixed
    # liquid volume and a single slew limit.
    # The outflow removes liquid and the gas dissolved in it.
    liquid_depletes: bool = True
    # The velocity command has separate accel and decel limits.
    slew_asym: bool = True

    # ----- Slave configuration overrides ---------------------------------
    # G_cfg* entries that replace the GVL defaults, by full name
    slave_cfg: dict = field(default_factory=dict)

    # ----- Values derived from the slave configuration -------------------
    def cfg(self, **more):
        """Return the effective slave configuration as a dict.

        The GVL defaults are overridden first by slave_cfg and then by the
        keyword arguments in more.
        """
        return slave_config(**{**self.slave_cfg, **more})

    @property
    def Ts(self):
        """[s] Slave scan time (G_cfgScanTime)"""
        return self.cfg()['G_cfgScanTime']

    @property
    def vmax(self):
        """[m/s] Velocity clamp (G_cfgPressureVelMax)"""
        # [mm/s -> m/s]
        return self.cfg()['G_cfgPressureVelMax'] * 1e-3

    @property
    def amax(self):
        """[m/s^2] Velocity command accel limit used when the controller
        does not supply one (G_cfgDirectCtrlAccel)"""
        # [mm/s^2 -> m/s^2]
        return self.cfg()['G_cfgDirectCtrlAccel'] * 1e-3

    @property
    def dmax(self):
        """[m/s^2] Velocity command decel limit used when the controller
        does not supply one (G_cfgDirectCtrlDecel)"""
        # [mm/s^2 -> m/s^2]
        return self.cfg()['G_cfgDirectCtrlDecel'] * 1e-3

    @property
    def medN(self):
        """[-] Median filter window (G_cfgAnalogMedianSize)"""
        return self.cfg()['G_cfgAnalogMedianSize']

    @property
    def tauF(self):
        """[s] Sensor first-order filter time constant
        (G_cfgAnalogFilterTimeConst)"""
        return self.cfg()['G_cfgAnalogFilterTimeConst']

    # ----- Copies with changed values ------------------------------------
    def with_(self, **kw):
        """Return a copy with the given fields replaced.

        Keys that start with G_cfg are slave settings and go into
        slave_cfg; all other keys must be Params fields.
        """
        cfg = {k: kw.pop(k) for k in list(kw) if k.startswith('G_cfg')}
        if cfg:
            kw['slave_cfg'] = {**self.slave_cfg, **cfg}
        return replace(self, **kw)

    def legacy(self):
        """Return a copy with the earlier model: a fixed liquid volume and
        a single slew limit."""
        return replace(self, liquid_depletes=False, slew_asym=False)

    @classmethod
    def from_overrides(cls, items, base=None):
        """Build Params from strings of the form "key=value".

        This serves the --set command-line option. Each key is a Params
        field or a G_cfg* name. Each value is parsed as a bool, an int, a
        float or, failing those, a string.
        """
        par = base or cls()
        names = {f.name for f in fields(cls)} - {'slave_cfg'}
        kw = {}
        for item in items or []:
            key, sep, text = item.partition('=')
            key = key.strip()
            if not sep or not (key in names or key.startswith('G_cfg')):
                raise ValueError(
                    f'bad override {item!r}: expected FIELD=value or '
                    f'G_cfg...=value')
            kw[key] = _parse(text.strip())
        return par.with_(**kw)


def _parse(text):
    """Convert a command-line value to a bool, int, float or string."""
    if text.lower() in ('true', 'false'):
        return text.lower() == 'true'
    for convert in (int, float):
        try:
            return convert(text)
        except ValueError:
            pass
    return text
