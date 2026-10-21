"""
controllers.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module defines the pressure controllers compared in the studies.
Every controller has the same interface, so the simulation can run any
of them:

  c.mode         'V' if the command is a piston velocity [m/s], or 'T'
                 if it is a piston force [N].
  c.accel        [m/s^2] Slew limits the velocity actuator applies to
  c.decel        the command in velocity mode. None selects the plant
                 defaults, Params.amax and Params.dmax.
  c.cfg          G_cfg* settings this controller requires of the slave
                 configuration, including the sensor chain.
  c.reset(par, scenario)
                 Prepares a run and returns the command in force before
                 the first scan.
  c.step(P_meas, target, t, scenario, x=0.0)
                 Computes one scan's command from the measured pressure
                 and the target [Pa], the time t [s] and the piston
                 position x [m].

The command returned by step() takes effect in the next scan, which
matches the call order in PRG_Main. A controller holds only its
configuration until reset() is called, so it can be sent to a worker
process.

SlaveController runs the slave's FB_PressureController, translated from
Structured Text, wired as PRG_Main wires it. It is a PI controller whose
output is clamped by a stop law (README eq. (19)). It can optionally add
a flow feedforward (eq. (20)), which the slave does not have.

LegacyController is a PID controller written for the earlier studies
(eqs. (21) to (25)). It can estimate the chamber stiffness online for its
stop law, add a flow feedforward, and command either velocity or force.
It exists to explore structures that the slave does not have.

FILE DEPENDENCY:
./
    slave.py
../generated/ (translated by tools/st2py.py from slave-src/FB/)
    FB_PressureController.py
"""

import math

import slave  # noqa: F401  (puts the generated blocks on sys.path)
from FB_PressureController import (FB_PressureController,
                                   new_FB_PressureController)


# %% ===== SLAVE CONTROLLER ==============================================
# Prefixes tried, in order, when SlaveController resolves a short
# setting name such as Kp to a full G_cfg* name
_PREFIXES = ('Pressure', 'Analog', '')


def _cfg_name(key, known):
    """Return the G_cfg* name in known that the short name key refers
    to."""
    for prefix in _PREFIXES:
        name = f'G_cfg{prefix}{key}'
        if name in known:
            return name
    raise KeyError(f'no slave configuration matches {key!r}')


class SlaveController:
    """The slave's FB_PressureController wired as PRG_Main wires it.

    Arguments:
      name         Label used in tables and plots.
      flow_ff      If True, add the expected outflow divided by the
                   piston area to the velocity after the block, eq. (20).
                   The slave has no flow feedforward; this is a study
                   option.
      **overrides  Slave settings, by full G_cfg* name or by the name
                   without G_cfg and an optional Pressure or Analog
                   prefix, in the units of the GVL. For example Kp=1.0,
                   Accel=100, StopStiffness=20 or
                   AnalogFilterTimeConst=0.02.
    """

    mode = 'V'

    def __init__(self, name='slave', flow_ff=False, **overrides):
        known = slave.slave_config()
        self.name = name
        self.flow_ff = flow_ff
        self.cfg = {_cfg_name(k, known): v for k, v in overrides.items()}

    def reset(self, par, scenario):
        self.A = par.A                                          # [m^2]
        self.dt = par.Ts                                        # [s]
        c = par.cfg(**self.cfg)
        self.c = c
        # Slew limits the drive applies in pressure mode,
        # [mm/s^2 -> m/s^2]
        self.accel = c['G_cfgPressureAccel'] * 1e-3
        self.decel = c['G_cfgPressureDecel'] * 1e-3
        self.target_max = c['G_cfgPressureTargetMax']           # [MPa]
        self.vel_max = c['G_cfgPressureVelMax']                 # [mm/s]

        # Configure the block as PRG_Main configures it. The block works
        # in MPa, mm/s and seconds.
        fb = new_FB_PressureController()
        fb.Enable = True
        fb.Kp = c['G_cfgPressureKp']            # [(mm/s)/MPa]
        fb.Ki = c['G_cfgPressureKi']            # [(mm/s)/(MPa s)]
        fb.VelMax = self.vel_max                # [mm/s]
        fb.DecelMax = c['G_cfgPressureDecel']   # [mm/s^2]
        fb.Stiffness = c['G_cfgPressureStopStiffness']  # [MPa/mm]
        fb.StopVelMin = c['G_cfgPressureStopVelMin']    # [mm/s]
        fb.CycleTime = c['G_cfgScanTime']       # [s]
        self.fb = fb
        self.first_scan = True
        return 0.0

    def step(self, P_meas, target, t, scenario, x=0.0):
        fb = self.fb
        # PRG_Main resets the block on the scan that enters the mode.
        fb.Reset = self.first_scan
        self.first_scan = False

        # PRG_Main limits the target to [0, TargetMax].
        # [Pa -> MPa]
        fb.Target = min(max(target * 1e-6, 0.0), self.target_max)
        fb.Actual = P_meas * 1e-6
        FB_PressureController(fb, self.dt)

        # PRG_Main clamps the block output to VelMax.
        v = min(max(fb.Velocity, -self.vel_max), self.vel_max)  # [mm/s]

        # Flow feedforward, eq. (20): the piston velocity that displaces
        # the expected outflow. [m^3/s / m^2 -> m/s, * 1e3 -> mm/s]
        if self.flow_ff:
            v_ff = scenario.Qff(t) / self.A * 1e3
            v = min(max(v + v_ff, -self.vel_max), self.vel_max)
        return v * 1e-3                                 # [mm/s -> m/s]


# %% ===== LEGACY CONTROLLER =============================================
class LegacyController:
    """PID controller with a filtered derivative, an optional stop law
    based on an online stiffness estimate, optional flow feedforward, and
    velocity or force command.

    Arguments:
      name      Label used in tables and plots.
      mode      'V' closes the loop through the drive's velocity loop.
                'T' closes it through the torque loop and always adds the
                force feedforward A * target.
      Kp Ki Kd  Gains on the error, its integral and the derivative of
                the measurement. In 'V' mode they are in mm/s per MPa,
                per MPa s and per MPa/s. In 'T' mode they are in N per
                MPa, per MPa s and per MPa/s.
      stopLaw   If True, limit |command| to the velocity from which the
                slew-limited actuator can still stop within the pressure
                error, eq. (24), using the stiffness estimated over the
                last ten scans, eq. (23).
      amax      [m/s^2] Slew limit for both accel and decel. None selects
                Params.amax.
      vff       'V' mode only. If True, add the expected outflow divided
                by the piston area as velocity feedforward.
      tauF      [s] Sensor filter time constant. It is applied as the
                slave setting G_cfgAnalogFilterTimeConst. None keeps the
                GVL value.
    """

    def __init__(self, name, mode, Kp, Ki, Kd, stopLaw=False, amax=None,
                 vff=False, tauF=None):
        self.name = name
        self.mode = mode
        self.stopLaw = stopLaw
        self.vff = vff
        self.amax = amax
        # Convert the gains to SI. In 'V' mode
        # [(mm/s)/MPa -> (m/s)/Pa] is a factor of 1e-3 / 1e6 = 1e-9.
        # In 'T' mode [N/MPa -> N/Pa] is a factor of 1e-6.
        scale = 1e-9 if mode == 'V' else 1e-6
        self.Kp = Kp * scale
        self.Ki = Ki * scale
        self.Kd = Kd * scale
        if tauF is None:
            self.cfg = {}
        else:
            self.cfg = {'G_cfgAnalogFilterTimeConst': tauF}
        self.accel = self.decel = amax

    def reset(self, par, scenario):
        self.A = par.A                          # [m^2]
        self.Ts = par.Ts                        # [s]
        self.Tf = par.Tf                        # [s] derivative filter
        self.amax_run = par.amax if self.amax is None else self.amax
        self.accel = self.decel = self.amax_run           # [m/s^2]
        # Command limit: the velocity clamp [m/s] or the force cap [N]
        self.u_limit = par.vmax if self.mode == 'V' else par.Fcap

        # Stiffness estimator for the stop law, eq. (23)
        self.k_est = 25e9                       # [Pa/m] initial estimate
        self.dP_hist = [0.0] * 10               # [Pa] last ten changes
        self.dx_hist = [0.0] * 10               # [m] last ten changes
        self.P_prev_est = scenario.P0           # [Pa]
        self.x_prev = 0.0                       # [m]

        # Derivative filter and integrator states
        self.P_prev = scenario.P0               # [Pa]
        self.dPdt_filt = 0.0                    # [Pa/s]
        self.err_int = 0.0                      # [Pa s]
        if self.mode == 'V':
            return 0.0
        return par.A * scenario.P0              # [N] holds P0

    def step(self, P_meas, target, t, scenario, x=0.0):
        Ts = self.Ts
        err = target - P_meas                                   # [Pa]

        # Filtered derivative of the measurement, eq. (21)
        dPdt_raw = (P_meas - self.P_prev) / Ts                  # [Pa/s]
        self.dPdt_filt += Ts / (self.Tf + Ts) * (dPdt_raw
                                                 - self.dPdt_filt)
        self.P_prev = P_meas

        # Feedforward: the force that balances the target pressure in 'T'
        # mode, or the velocity that displaces the expected outflow in 'V'
        # mode with vff
        if self.mode == 'T':
            u_ff = self.A * target                              # [N]
        elif self.vff:
            u_ff = scenario.Qff(t) / self.A                     # [m/s]
        else:
            u_ff = 0.0

        # PID control law before the limit, eq. (22)
        u_raw = (u_ff + self.Kp * err + self.Ki * self.err_int
                 - self.Kd * self.dPdt_filt)

        # Command limit, reduced by the stop law if it is enabled
        u_limit = self.u_limit
        if self.stopLaw:
            # Stiffness estimate from the pressure and position changes
            # over the last ten scans, eq. (23). The estimate is updated
            # only when the piston has moved more than 5 um, and it is
            # held within 0.01 to 100 MPa/mm.
            self.dP_hist = self.dP_hist[1:] + [P_meas - self.P_prev_est]
            self.dx_hist = self.dx_hist[1:] + [x - self.x_prev]
            self.P_prev_est = P_meas
            self.x_prev = x
            dx_sum = sum(self.dx_hist)                          # [m]
            if abs(dx_sum) > 5e-6:
                k = abs(sum(self.dP_hist)) / abs(dx_sum)        # [Pa/m]
                self.k_est = min(max(k, 0.01e9), 100e9)

            # Stop-law velocity, eq. (24): the speed from which the
            # actuator, decelerating at amax, stops within the error
            a = self.amax_run                                   # [m/s^2]
            v_stop = (-a * Ts
                      + math.sqrt((a * Ts)**2
                                  + 2 * a * abs(err) / self.k_est))
            u_limit = min(u_limit, v_stop)

        u = min(max(u_raw, -u_limit), u_limit)

        # Conditional anti-windup, eq. (25). The integrator advances
        # unless the command is saturated and the error would drive it
        # further into saturation.
        unsaturated = u_raw == u
        unwinding = (u_raw > u and err < 0) or (u_raw < u and err > 0)
        if unsaturated or unwinding:
            self.err_int += err * Ts
        return u


def legacy_set():
    """Return the legacy controller variants, by name.

    The set holds velocity-mode PI controllers with and without the stop
    law at several slew limits, a PID with the stop law, and the torque
    feedforward reference with derivative damping.
    """
    L = LegacyController
    return {
        'V-PI.25-.2-a5':
            L('V-PI.25-.2-a5', 'V', 0.25, 0.2, 0, False, 5e-3),
        'V-PI.25-.2-a50':
            L('V-PI.25-.2-a50', 'V', 0.25, 0.2, 0, False, 50e-3),
        'V-PI.5-.2-a50-stop':
            L('V-PI.5-.2-a50-stop', 'V', 0.5, 0.2, 0, True, 50e-3),
        'V-PI.5-.2-a200-stop':
            L('V-PI.5-.2-a200-stop', 'V', 0.5, 0.2, 0, True, 200e-3),
        'V-PID+Kd.05-a50-stop':
            L('V-PID+Kd.05-a50-stop', 'V', 0.5, 0.2, 0.05, True, 50e-3),
        'T-FF-Kd100':
            L('T-FF-Kd100', 'T', 0, 0, 100),
    }
