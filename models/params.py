"""
params.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module defines Params, the parameters of the models: the plant
(plant.py, ported to C in c/plant.c), the sensor chain (sensor.py) and
their integration. The simulated slave takes its defaults (through
plant_export.py), and the design studies extend it with their own
settings and may override its defaults
(design/pressure-loop/studyparams.py). All values are in SI units unless
a comment states otherwise.

The parameters describe five parts of the model:

  Chamber     A piston of area A compresses a chamber that holds saline
              of bulk modulus K and oxygen gas. The gas is isothermal
              and dissolves into the saline according to Henry's law
              with time constant tauDiss. A pocket of air V_air in the
              passage to the pressure transducer is a second isothermal
              gas that never dissolves. The chamber stiffness dP/dx
              therefore rises from about 0.02 MPa/mm at the start of a
              run to 10 to 25 MPa/mm once the gas has dissolved.
  Charge      The oxygen charge is given as the ratio of its STP volume
              to the saline volume, and is admitted at the supply
              pressure, where the run starts. The saturation pressure of
              the dissolved oxygen follows from the charge; the pressure
              must stay above it or the solution degasses.
  Outlet      A hose with a valve at each end, a tube and a plate of
              micro-orifices. The hose holds virgin saline and is shut
              during compression; opening it costs the chamber the
              volume that pressurises the hose. The orifices set the
              outflow as a function of chamber pressure, and the air in
              the tube is lost from the chamber as a volume step when
              the distal valve opens. The virgin saline is purged
              through the orifices before the solution is delivered.
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
individual G_cfg* entries for a study. The charge (ratio, P_supply) is
the default start of a run; Params.contents() gives the chamber's
contents for any start.

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
    # With the air pocket V_air below explicit, K is the liquid value,
    # to be re-read from the fits' liquid-only term; the 0.72 GPa is
    # an effective value that includes the pocket's compliance.
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
    # [m^3] Air trapped in the passage to the pressure transducer (about
    # 2 cm long) and in the transducer's cavity, at atmospheric
    # pressure. It cannot be purged, so it is present in every run, and
    # it dissolves only very slowly through the narrow passage, so the
    # model holds it as a fixed amount of isothermal gas that never
    # dissolves, eq. (46). The passage alone (1 to 2 mm bore) holds 0.02
    # to 0.06 mL, plus whatever the cavity holds. The default, 0.65 mL,
    # is a placeholder of the size of the gas-like term the bulk-modulus
    # fits found in the closed chamber. That term is a volume at the
    # fit's mean pressure P0, so the same air at atmosphere would be
    # larger by (P0 + Patm) / Patm, tens of times at 5 MPa; the fits at
    # two fill levels are to settle V_air.
    V_air: float = 0.65e-6

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
    # [m^3] Dead volume of the cylinder head: the passages the piston
    # cannot sweep, left when the piston reaches the limit switch at the
    # end of its stroke, with the proximal valve shut. The machine halts
    # there and the delivery ends; so does the simulation. Measured:
    # 57 g of water drawn from the passages with the hose shut, with the
    # present fittings.
    V_passages: float = 57e-6

    # ----- Hose ----------------------------------------------------------
    # A 3 ft hose of 1/4 in bore joins the cylinder head to the orifice
    # tube, with a ball valve at each end. It is prefilled with virgin
    # saline at atmosphere, the proximal valve is shut while the charge
    # is compressed and dissolved, and it is opened before the delivery.
    # Its liquid, its own compliance and the saline it holds then join
    # the chamber, eqs. (42) and (43).
    # [m^3] Hose bore volume, 0.914 m of 6.35 mm bore
    V_hose: float = 29e-6
    # [m^3/Pa] Hose compliance (wall dilation and any air it traps),
    # measured by difference between K runs with the hose shut and
    # open: 0.24 mL per MPa
    C_hose: float = 2.4e-13
    # ----- Proximal valve ------------------------------------------------
    # The proximal valve throttles the solution that fills the hose when
    # it is opened at pressure, eqs. (44) and (45). Its area rises to the
    # bore of a d_prox valve over the opening time a study gives; opened
    # in one scan it is the equalisation of eq. (43). A needle valve in
    # its place is a smaller d_prox.
    # [m] Bore of the proximal valve: 1/4 in ball valve
    d_prox: float = 6.35e-3
    # [-] Discharge coefficient of the proximal valve
    Cd_prox: float = 0.6

    # ----- Derived outlet quantities -------------------------------------
    @property
    def V_dead(self):
        """[m^3] Dead volume with the hose open: passages, hose and
        tube, about 86 mL"""
        return self.V_passages + self.V_hose + self.V_tube

    @property
    def A_prox(self):
        """[m^2] full area of the proximal valve"""
        return math.pi * self.d_prox**2 / 4

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

    def contents(self, P0, ratio, dissolved=False):
        """Return the chamber contents at the start of a run at gauge
        pressure P0 [Pa] with a charge of STP ratio ratio, as (P0 [Pa],
        free gas [mol], dissolved gas [mol]).

        The saline can hold n_sat = kH (P0 + Patm) Vl0 by Henry's law.
        If the charge starts dissolved, n_dis = min(charge, n_sat) and
        the rest is free; otherwise the charge is free and the saline
        carries n_sat, the equilibrium it reached during the fill.
        """
        n_sat = self.kH * (P0 + self.Patm) * self.Vl0       # [mol]
        if ratio <= 0:
            return P0, 0.0, n_sat
        n_charge = self.n_charge(ratio)                     # [mol]
        if dissolved:
            n_dis = min(n_charge, n_sat)                    # [mol]
            return P0, n_charge - n_dis, n_dis
        return P0, n_charge, n_sat

    # ----- Simulation ----------------------------------------------------
    # [-] Plant integration steps per slave scan
    nsub: int = 5
    # Integration method from solver.METHODS: 'heun' or 'euler'
    solver: str = 'heun'

    # ----- Actuator ------------------------------------------------------
    # The drive's response to a velocity command was measured on
    # 2026-10-26 (bench/friction, bench/characterisation): the command is
    # not slewed at the configured accel, and the motor velocity follows
    # it after a dead time of 22.8 ms with a first-order lag of 18.2 ms,
    # the same for 0.3 to 2.5 mm/s steps in both directions, fitted at
    # the 4 ms row spacing of the slave scan. The lag at the piston
    # (behind the gearbox and screw) was not resolved.
    # [s] Dead time between the velocity command and the start of the
    # response, applied in whole scans (6 scans of 4 ms)
    t_dead: float = 0.0228
    # [s] Time constant of the drive's velocity loop
    tauV: float = 0.0182
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
    # rms across the shunt (4.6 kPa) bare and 0.8 mV (1.8 kPa) with
    # a 4.7 uF capacitor across the shunt; broadband, no lines.
    noise: float = 4.6e3
    # [Pa] Transducer zero offset: the reading with the chamber open.
    # The loop regulates the reading, so the true pressure sits above
    # the target by minus this much. Zero with the input calibrated
    # (G_cfgPressureShuntOhms the effective 421.4 ohm, measured
    # +0.002 MPa vented); -0.11e6 with the 434 ohm setting.
    P_offset: float = 0.0

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
        does not supply one (G_cfgDirectCtrlAccel, the block's one
        rate for both directions)"""
        # [mm/s^2 -> m/s^2]
        return self.cfg()['G_cfgDirectCtrlAccel'] * 1e-3

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
        single slew limit on the command, no dead time and no air pocket
        at the transducer."""
        return replace(self, liquid_depletes=False, slew_asym=False,
                       slew_limited=True, t_dead=0.0, V_air=0.0)

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
