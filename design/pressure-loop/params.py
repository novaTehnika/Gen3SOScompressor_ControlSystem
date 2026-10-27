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

The parameters describe five parts of the model:

  Chamber     A piston of area A compresses a chamber that holds saline
              of effective bulk modulus K and oxygen gas. The gas is
              isothermal and dissolves into the saline according to
              Henry's law with time constant tauDiss. The chamber
              stiffness dP/dx therefore rises from about 0.02 MPa/mm at
              the start of a run to 10 to 25 MPa/mm once the gas has
              dissolved.
  Charge      The oxygen charge is given as the ratio of its STP volume
              to the saline volume, and is admitted at the supply
              pressure, where the run starts. The saturation pressure of
              the dissolved oxygen follows from the charge; the pressure
              must stay above it or the solution degasses.
  Outlet      A ball valve, a tube and a plate of micro-orifices. The
              orifices set the outflow as a function of chamber
              pressure, and the air in the tube is lost from the chamber
              as a volume step when the valve opens.
  Actuator    In velocity mode the drive follows the commanded velocity
              after a dead time t_dead with lag tauV, and stalls when the
              chamber force plus friction reaches the torque limit Fcap.
              In torque mode the commanded force drives the reflected
              mass mr against the chamber force, the load-dependent
              friction mu P A, viscous friction b and Coulomb friction
              Fc.
  Sensor      A 4-20 mA pressure transducer drives a current through a
              shunt resistor, and the slave samples the voltage once per
              scan. The noise parameter sets the white noise on each
              sample, referred to pressure, and P_offset the
              transducer's zero error.
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
    # [Pa] Effective bulk modulus of the saline-filled chamber. Water
    # alone is 2.2 GPa; the seals, hoses and residual gas lower it.
    # Measured 0.72 GPa (0.67 to 0.78 over the segments, 0.81 with the
    # gas-like curvature removed) on 2026-10-26, bench/characterisation.
    K: float = 0.72e9
    # [m^3] Saline volume at the start of a run, including the dead
    # volume: the fill at the 280 mm start position of the bench runs
    Vl0: float = 343e-6
    # [mol/(m^3 Pa)] Henry's-law constant for oxygen in saline: 1.28e-5
    # for pure water, reduced 5 % by salting out at 0.9 % NaCl
    kH: float = 1.22e-5
    # [s] Time constant of gas dissolution
    tauDiss: float = 30.0
    # [kg/m^3] Saline density
    rho: float = 1000.0

    # ----- Oxygen charge -------------------------------------------------
    # [-] Oxygen charge as the ratio of its volume at STP (0 degC, 1 atm)
    # to the saline volume; 1.0 is the 1:1 recipe
    ratio: float = 1.0
    # [Pa] Gauge pressure of the oxygen supply when the charge is admitted.
    # The chamber starts a run at this pressure. The supply is 75 to
    # 200 psi (0.52 to 1.38 MPa) depending on the cylinder's state of
    # charge.
    P_supply: float = 1.0e6

    # ----- Outlet: ball valve, tube and orifice bank ---------------------
    # The outlet is a 1/4 in ball valve, a 1 mm bore tube 300 mm long,
    # and a plate of 32 orifices of 40 um, discharging to atmosphere. The
    # ball valve's bore is 450 times the orifice area, so once it cracks
    # the orifices set the flow, eq. (36).
    # [-] Number of orifices
    n_orifice: int = 32
    # [m] Orifice diameter
    d_orifice: float = 40e-6
    # [-] Orifice discharge coefficient. Measured 0.85 (0.84 to 0.87
    # over 1.6 to 8.3 MPa, exponent 0.48): 4.8 mL/s at 10 MPa.
    Cd: float = 0.85
    # [m^3] Volume of the tube between the valve and the orifices. It
    # holds air at atmosphere before the valve opens, so the chamber loses
    # this volume of liquid in the scan the valve opens, eq. (37).
    V_tube: float = 0.236e-6
    # [s] Valve opening time; 0 opens the valve in one scan. The ball
    # valve is opened rapidly.
    Topen: float = 0.0
    # [m^3] Dead volume: the chamber volume the piston cannot sweep
    # (passages and fittings), left when the piston reaches the limit
    # switch at the end of its stroke. The machine halts there and the
    # delivery ends; so does the simulation. Measured: 50 g of water
    # fills it.
    V_dead: float = 50e-6

    # ----- Derived outlet quantities -------------------------------------
    @property
    def A_orifice(self):
        """[m^2] Total orifice area"""
        return self.n_orifice * math.pi * self.d_orifice**2 / 4

    def Q_orifice(self, P):
        """[m^3/s] Orifice flow at chamber gauge pressure P [Pa] with the
        valve fully open, eq. (36): Q = Cd A sqrt(2 P / rho)."""
        return self.Cd * self.A_orifice * math.sqrt(2 * max(P, 0.0)
                                                    / self.rho)

    def n_charge(self, ratio=None):
        """[mol] Oxygen charge for an STP ratio (default self.ratio), by
        the ideal gas law at 0 degC and 1 atm"""
        r = self.ratio if ratio is None else ratio
        return r * self.Vl0 * self.Patm / (self.R * 273.15)

    def P_sat(self, n_dis, V_l):
        """[Pa] Gauge saturation pressure of n_dis [mol] of oxygen
        dissolved in V_l [m^3] of saline, eq. (38). Below this pressure
        the solution degasses."""
        return n_dis / (self.kH * V_l) - self.Patm

    # ----- Simulation ----------------------------------------------------
    # [-] Plant integration steps per slave scan
    nsub: int = 5
    # Integration method from solver.METHODS: 'heun' or 'euler'
    solver: str = 'heun'

    # ----- Actuator ------------------------------------------------------
    # The drive's response to a velocity command was measured on
    # 2026-10-26 (bench/friction, bench/characterisation): the command is
    # not slewed at the configured accel, and the motor velocity follows
    # it after a dead time of 12 ms with a first-order lag of 8.7 ms, the
    # same for 0.3 to 2.5 mm/s steps in both directions. The lag at the
    # piston (behind the gearbox and screw) was not resolved; it is below
    # 25 ms.
    # [s] Dead time between the velocity command and the start of the
    # response, applied in whole scans
    t_dead: float = 0.012
    # [s] Time constant of the drive's velocity loop
    tauV: float = 0.0087
    # True if the drive slews the velocity command at the accel and
    # decel limits. The drive does not; the option keeps the earlier
    # model available.
    slew_limited: bool = False
    # [N] Piston force at 100 % rated motor torque through a lossless
    # gearbox and screw: 1.27 N m times 2 pi times 70 over 5.08 mm of
    # lead. The drivetrain losses are in mu below.
    Fcap: float = 1.27 * 2 * math.pi * 70 / 5.08e-3
    # [-] Load-dependent friction: a force mu P A opposes the motion in
    # addition to the Coulomb and viscous terms, so the efficiency
    # pushing against the load is 1 / (1 + mu). Measured 0.45.
    mu: float = 0.45
    # [kg] Motor and gearbox inertia reflected to the piston
    mr: float = 5e4
    # [N s/m] Viscous friction coefficient. Measured 2.3e6 (11 kN at
    # 5 mm/s).
    b: float = 2.3e6
    # [N] Coulomb friction of the seals, screw and gearbox. Measured
    # 6.1 kN; the breakaway from rest is about 24 kN.
    Fc: float = 6.1e3
    # [m/s] Velocity over which the Coulomb friction is smoothed from
    # -Fc to +Fc, which avoids a discontinuity at zero velocity
    veps: float = 1e-5

    # ----- Sensor --------------------------------------------------------
    # [Pa] RMS noise per sample, referred to pressure. Measured 2.0 mV
    # rms on the 434 ohm shunt (4.6 kPa) bare and 0.8 mV (1.8 kPa) with
    # a 4.7 uF capacitor across the shunt; broadband, no lines.
    noise: float = 4.6e3
    # [Pa] Transducer zero offset: the reading with the chamber open.
    # Measured -0.11 MPa. The loop regulates the reading, so the true
    # pressure sits above the target by this much.
    P_offset: float = -0.11e6
    # [s] Time constant of the derivative filter in the legacy
    # controller
    Tf: float = 0.02

    # ----- Process -------------------------------------------------------
    # [Pa] Target pressure for the compression, hold and delivery
    P_target: float = 10e6

    # ----- Metrics and requirements --------------------------------------
    # [Pa] Half-width of the band used for the settling time
    tol: float = 0.1e6
    # [Pa] Required margin of the chamber pressure above the saturation
    # pressure during the hold and delivery (no degassing)
    degas_margin: float = 0.5e6
    # [Pa] Highest pressure allowed at any time (mechanical limit)
    P_ceiling: float = 12e6
    # [Pa] Largest error tolerated during a delivery once the opening
    # dip has had t_recover to settle; a larger error counts as loss of
    # control
    e_crash: float = 1.0e6
    # [s] Time after the valve opens allowed for the opening dip to
    # recover; the dip metric is the lowest pressure in this window
    t_recover: float = 5.0

    # ----- Model options -------------------------------------------------
    # Setting both options to False (with slew_limited True) gives the
    # earlier model: a fixed liquid volume and a single slew limit.
    # The outflow removes liquid and the gas dissolved in it.
    liquid_depletes: bool = True
    # The velocity command has separate accel and decel limits when it
    # is slewed.
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
        """Return a copy with the earlier model: a fixed liquid volume, a
        single slew limit on the command, and no dead time."""
        return replace(self, liquid_depletes=False, slew_asym=False,
                       slew_limited=True, t_dead=0.0)

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
