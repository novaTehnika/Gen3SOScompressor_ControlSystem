"""Pressure controllers behind one interface.

    c.mode                 'V' (command is a velocity, m/s) or 'T' (a force, N)
    c.accel, c.decel       m/s^2 slew limits the actuator applies ('V'); None
                           means the plant parameters' own
    c.cfg                  G_cfg* overrides this controller implies for the
                           slave configuration (sensor chain included)
    c.reset(p, scenario)   prepare a run; returns the command in force before
                           the first scan
    c.step(p_meas, target, t, scenario, x=0.0)
                           one scan: measured pressure and target in Pa, time
                           in s, piston position x in m; returns the command

Whatever step returns takes effect in the next scan, matching the call order
in PRG_Main. Controllers are plain configuration until reset(), so they pickle.
"""

import math

import slave  # noqa: F401
from FB_PressureController import FB_PressureController, new_FB_PressureController

# short names accepted by SlaveController, mapped to G_cfg* entries
_ALIASES = ('Pressure', 'Analog', '')


def _cfg_name(key, known):
    for prefix in _ALIASES:
        name = f'G_cfg{prefix}{key}'
        if name in known:
            return name
    raise KeyError(f'no slave configuration matches {key!r}')


class SlaveController:
    """The slave's FB_PressureController wired as PRG_Main wires it.

    Overrides are G_cfg* entries by full name or by name without the G_cfg
    (and optional Pressure/Analog) prefix, e.g. Kp=1.0, Accel=100, StopStiffness=20,
    AnalogFilterTimeConst=0.02, in the GVL's units.

    flow_ff=True adds the expected flow / A to the velocity after the block
    (a study-only extension: the slave has no flow feedforward).
    """

    mode = 'V'

    def __init__(self, name='slave', flow_ff=False, **overrides):
        known = slave.slave_config()
        self.name = name
        self.flow_ff = flow_ff
        self.cfg = {_cfg_name(k, known): v for k, v in overrides.items()}

    def reset(self, p, scenario):
        self.A, self.dt = p.A, p.Ts
        c = p.cfg(**self.cfg)
        self.c = c
        self.accel = c['G_cfgPressureAccel'] * 1e-3
        self.decel = c['G_cfgPressureDecel'] * 1e-3
        self.tmax = c['G_cfgPressureTargetMax']
        self.vmax = c['G_cfgPressureVelMax']
        f = self.fb = new_FB_PressureController()
        f.Enable = True
        f.Kp, f.Ki = c['G_cfgPressureKp'], c['G_cfgPressureKi']
        f.VelMax = self.vmax
        f.DecelMax = c['G_cfgPressureDecel']
        f.Stiffness = c['G_cfgPressureStopStiffness']
        f.StopVelMin = c['G_cfgPressureStopVelMin']
        f.CycleTime = c['G_cfgScanTime']
        self.first = True
        return 0.0

    def step(self, p_meas, target, t, scenario, x=0.0):
        f = self.fb
        f.Reset = self.first                       # bEnterState on the entry scan
        self.first = False
        f.Target = min(max(target * 1e-6, 0.0), self.tmax)
        f.Actual = p_meas * 1e-6
        FB_PressureController(f, self.dt)
        v = min(max(f.Velocity, -self.vmax), self.vmax)
        if self.flow_ff:
            v = min(max(v + scenario.Qff(t) / self.A * 1e3, -self.vmax), self.vmax)
        return v * 1e-3


class LegacyController:
    """PID with a filtered derivative, an online stiffness-estimating stop law,
    optional flow feedforward, in velocity ('V') or torque ('T') command.

    mode     'V' closes the loop through the drive velocity loop; 'T' through
             the torque loop, with feedforward A*target always applied.
    Kp Ki Kd 'V': mm/s per MPa, per s, per 1/s. 'T': N per MPa, per s, per 1/s.
    stopLaw  clamp |v| to the rate from which the slew-limited actuator can
             still stop within the pressure error, using dP/dx estimated over
             the last ten scans
    amax     velocity slew (accel and decel) in m/s^2; None for the plant's
    vff      'V' only: add the expected flow / A as velocity feedforward
    tauF     sensor filter time constant override (s), applied as the slave
             filter configuration
    """

    def __init__(self, name, mode, Kp, Ki, Kd, stopLaw=False, amax=None, vff=False, tauF=None):
        self.name, self.mode, self.stopLaw, self.vff = name, mode, stopLaw, vff
        self.amax = amax
        scale = 1e-9 if mode == 'V' else 1e-6
        self.Kp, self.Ki, self.Kd = Kp * scale, Ki * scale, Kd * scale
        self.cfg = {} if tauF is None else {'G_cfgAnalogFilterTimeConst': tauF}
        self.accel = self.decel = amax

    def reset(self, p, scenario):
        self.A, self.Ts, self.Tf = p.A, p.Ts, p.Tf
        self.amax_ = p.amax if self.amax is None else self.amax
        self.accel = self.decel = self.amax_
        self.lim0 = p.vmax if self.mode == 'V' else p.Fcap
        self.kEst = 25e9
        self.dPbuf = [0.0] * 10
        self.dxbuf = [0.0] * 10
        self.xPrev = 0.0
        self.PfEst = scenario.P0
        self.Pfprev = scenario.P0
        self.d = 0.0
        self.I = 0.0
        return 0.0 if self.mode == 'V' else p.A * scenario.P0

    def step(self, Pf, tgt, t, scenario, x=0.0):
        Ts = self.Ts
        e = tgt - Pf
        self.d += Ts / (self.Tf + Ts) * ((Pf - self.Pfprev) / Ts - self.d)
        self.Pfprev = Pf
        ff = 0.0
        if self.mode == 'T':
            ff = self.A * tgt
        elif self.vff:
            ff = scenario.Qff(t) / self.A
        raw = ff + self.Kp * e + self.Ki * self.I - self.Kd * self.d
        lim = self.lim0
        if self.stopLaw:
            self.dPbuf = self.dPbuf[1:] + [Pf - self.PfEst]
            self.dxbuf = self.dxbuf[1:] + [x - self.xPrev]
            self.PfEst, self.xPrev = Pf, x
            sx = sum(self.dxbuf)
            if abs(sx) > 5e-6:
                self.kEst = min(max(abs(sum(self.dPbuf)) / abs(sx), 0.01e9), 100e9)
            a = self.amax_
            rate = -a * Ts + math.sqrt((a * Ts) ** 2 + 2 * a * abs(e) / self.kEst)
            lim = min(lim, rate)
        u = min(max(raw, -lim), lim)
        # conditional anti-windup: integrate unless saturated with the error
        # pushing further into saturation
        if raw == u or (raw > u and e < 0) or (raw < u and e > 0):
            self.I += e * Ts
        return u


def legacy_set():
    """The legacy controller variants (PI with and without the stop law, PID, torque feedforward), by name."""
    return {
        'V-PI.25-.2-a5': LegacyController('V-PI.25-.2-a5', 'V', 0.25, 0.2, 0, False, 5e-3),
        'V-PI.25-.2-a50': LegacyController('V-PI.25-.2-a50', 'V', 0.25, 0.2, 0, False, 50e-3),
        'V-PI.5-.2-a50-stop': LegacyController('V-PI.5-.2-a50-stop', 'V', 0.5, 0.2, 0, True, 50e-3),
        'V-PI.5-.2-a200-stop': LegacyController('V-PI.5-.2-a200-stop', 'V', 0.5, 0.2, 0, True, 200e-3),
        'V-PID+Kd.05-a50-stop': LegacyController('V-PID+Kd.05-a50-stop', 'V', 0.5, 0.2, 0.05, True, 50e-3),
        'T-FF-Kd100': LegacyController('T-FF-Kd100', 'T', 0, 0, 100),
    }
