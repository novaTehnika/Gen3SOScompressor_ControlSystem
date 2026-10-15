"""Plant, actuator, sensor and timing parameters, SI units.

Plant: a piston of area A compresses a chamber holding liquid of effective
bulk modulus K and oxygen gas (isothermal) that dissolves into the liquid per
Henry's law with time constant tauDiss. Chamber stiffness dP/dx therefore runs
from ~0.002 MPa/mm when gas-rich to ~10-25 MPa/mm once the gas has dissolved.

Velocity actuator (Y_DirectControl VelocityTLMode): the command is slewed at
the accel limit while its magnitude grows and the decel limit while it
shrinks or reverses, then lagged by tauV for the drive velocity loop, and
stalls when the chamber force exceeds Fcap. Torque actuator (TorqueVLMode):
reflected mass mr driven by force F against chamber pressure with viscous and
Coulomb friction, velocity clamped at vmax.

Sensor: 4-20 mA transducer across a shunt, sampled every scan with white
noise, then the slave's own FB_AnalogInputFilter and FB_PressureInput.

Everything the slave configures (gains, velocity clamp, accel and decel, stop
law, filter, median size, scan time, shunt and transducer range) is read from
the GVL through slave.slave_config(); `slave_cfg` overrides individual G_cfg*
entries for a study.
"""

from dataclasses import dataclass, field, fields, replace

from slave import slave_config


@dataclass(frozen=True)
class Params:
    # chamber
    A: float = 3.431e-3        # m^2 piston area, 2.602 in bore
    R: float = 8.314           # J/(mol K)
    T: float = 293.0           # K
    Patm: float = 101325.0     # Pa
    K: float = 1.0e9           # Pa effective liquid bulk modulus (water 2.2e9; seals and hose lower it)
    Vl0: float = 300e-6        # m^3 liquid
    Vg0: float = 200e-6        # m^3 gas at atmosphere for the gas-rich start
    kH: float = 1.28e-5        # mol/(m^3 Pa) Henry constant, O2 in water
    tauDiss: float = 30.0      # s dissolution time constant

    # discharge valve (used by the 'valve' scenario)
    Qf: float = 1e-6           # m^3/s final outflow once the valve is open
    Topen: float = 0.3         # s valve opening time, 0 for a step
    valveT: float = 8.0        # s simulated time for the valve scenario

    # timing
    nsub: int = 5              # plant substeps per scan

    # actuator
    tauV: float = 0.02         # s drive velocity loop lag
    Fcap: float = 1.27 * 2 * 3.141592653589793 * 70 / 5.08e-3 * 0.85  # N at 100 % rated torque: 1.27 N m, 70:1 gearbox, 5.08 mm lead, 85 % efficiency
    mr: float = 5e4            # kg reflected motor and gearbox inertia at the piston
    b: float = 5e3             # N s/m viscous friction
    Fc: float = 500.0          # N Coulomb friction (seals, screw)
    veps: float = 1e-5         # m/s stiction regularisation

    # sensor
    noise: float = 0.005e6     # Pa rms per sample, referred to pressure
    Tf: float = 0.02           # s derivative filter of the legacy controller

    # metrics
    tol: float = 0.1e6         # Pa settling band

    # model options; False for both gives a fixed-liquid, single-slew-limit chamber and actuator
    liquid_depletes: bool = True   # outflow removes liquid and the gas dissolved in it
    slew_asym: bool = True         # separate accel and decel limits on the velocity command

    slave_cfg: dict = field(default_factory=dict)   # G_cfg* overrides

    # --- derived from the slave configuration
    def cfg(self, **more):
        """Effective slave configuration: GVL defaults, then slave_cfg, then `more`."""
        return slave_config(**{**self.slave_cfg, **more})

    @property
    def Ts(self):
        """s slave scan"""
        return self.cfg()['G_cfgScanTime']

    @property
    def vmax(self):
        """m/s velocity clamp (G_cfgPressureVelMax)"""
        return self.cfg()['G_cfgPressureVelMax'] * 1e-3

    @property
    def amax(self):
        """m/s^2 velocity slew when the controller names none (G_cfgDirectCtrlAccel)"""
        return self.cfg()['G_cfgDirectCtrlAccel'] * 1e-3

    @property
    def dmax(self):
        """m/s^2 velocity decel when the controller names none (G_cfgDirectCtrlDecel)"""
        return self.cfg()['G_cfgDirectCtrlDecel'] * 1e-3

    @property
    def medN(self):
        """median window (G_cfgAnalogMedianSize)"""
        return self.cfg()['G_cfgAnalogMedianSize']

    @property
    def tauF(self):
        """s first-order filter (G_cfgAnalogFilterTimeConst)"""
        return self.cfg()['G_cfgAnalogFilterTimeConst']

    # --- overrides
    def with_(self, **kw):
        """Copy with fields replaced. Keys starting G_cfg go to slave_cfg."""
        cfg = {k: kw.pop(k) for k in list(kw) if k.startswith('G_cfg')}
        if cfg:
            kw['slave_cfg'] = {**self.slave_cfg, **cfg}
        return replace(self, **kw)

    def legacy(self):
        """Fixed liquid volume and one slew limit."""
        return replace(self, liquid_depletes=False, slew_asym=False)

    @classmethod
    def from_overrides(cls, items, base=None):
        """Build from ["key=value", ...] (command-line --set): a Params field
        or a G_cfg* name; values are parsed as bool, int or float."""
        p = base or cls()
        names = {f.name for f in fields(cls)} - {'slave_cfg'}
        kw = {}
        for item in items or []:
            key, sep, text = item.partition('=')
            key = key.strip()
            if not sep or not (key in names or key.startswith('G_cfg')):
                raise ValueError(f'bad override {item!r}: expected FIELD=value or G_cfg...=value')
            kw[key] = _parse(text.strip())
        return p.with_(**kw)


def _parse(text):
    if text.lower() in ('true', 'false'):
        return text.lower() == 'true'
    try:
        return int(text)
    except ValueError:
        return float(text)
