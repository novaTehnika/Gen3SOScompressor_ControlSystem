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
                 defaults, StudyParams.amax and StudyParams.dmax.
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
output is clamped by a stop law (README eq. (19)), with the block's own
floor that follows the outflow and its forward stop, eqs. (46) and (48),
when G_cfgPressureFloorEnable is set (the override FloorEnable=True). It
can optionally add a flow feedforward (eq. (20)), which the slave does
not have.

ScheduledSlaveController runs the same block with its gains and its
stop-law stiffness set every scan from an estimate of the chamber
stiffness, eqs. (39) to (41). The estimate uses only what the slave
has: the measured pressure, the piston position against a configured
contact position, and the configured saline volume and charge. It is an
upper bound on the stiffness, so the scheduled gain is a lower bound on
the gain that gives the chosen crossover. It is a comparison arm of the
studies and is not in the slave: with the measured chamber it gives the
results of a fixed gain.

VariantController runs the same block with one change to how its
integrator is managed (the anti-windup variants of eqs. (50) to (55))
and an optional derivative term, eq. (56). It is a comparison arm of
study_antiwindup.py and is not in the slave.

FloorPolicy sets the stop-law floor (StopVelMin) of either controller
every scan from the detected outflow instead of the fixed configured
value, eqs. (46) to (48), and can add a forward stop above the target.
Its switched mode with the forward stop is what FB_PressureController
has; the other modes are study arms. A controller given a FloorPolicy
runs the block with its own floor off.

LegacyController is a PID controller written for the earlier studies
(eqs. (21) to (25)). It can estimate the chamber stiffness online for its
stop law, add a flow feedforward, and command either velocity or force.
It exists to explore structures that the slave does not have.

FILE DEPENDENCY:
./
    modelpath.py
../../models/
    slave.py
../../models/generated/ (translated by tools/st_translate.py from
slave-src/FB/)
    FB_PressureController.py
"""

import math

import modelpath  # noqa: F401  (puts models/ on sys.path)
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


# %% ===== STOP-LAW FLOOR POLICIES =======================================
def measurement_lag_scans(c):
    """[-] Scans by which the measured pressure lags the chamber in the
    slave configuration c: the filter time constant, the median's
    half-width and the two pipeline scans. The floor policies delay the
    piston position by this much so that the two signals they compare
    describe the same interval."""
    Ts = c['G_cfgScanTime']                                     # [s]
    n_med = 5 if c['G_cfgAnalogMedianSize'] >= 5 else 3         # [-]
    return (round(c['G_cfgAnalogFilterTimeConst'] / Ts)
            + (n_med - 1) // 2 + 2)


class FloorPolicy:
    """A stop-law floor that follows the outflow instead of a fixed
    StopVelMin.

    The stop law clamps the velocity to zero at zero error, which is
    right for a sealed chamber and wrong with the outlet open, where the
    piston must keep moving to hold the pressure. A fixed floor covers
    the outflow at the cost of the stop law near the target in every
    state. A policy sets the floor each scan from what the slave
    measures: the piston position x [mm], the measured pressure
    P [MPa] and the pressure error e [MPa]. Both use a window of W
    scans, with the position delayed by the measurement lag d so that
    the two describe the same interval:

      dx = x[n - d] - x[n - d - W],   dP = P[n] - P[n - W]

    In a sealed chamber dP = k dx with k > 0, so a piston that moves
    forward while the pressure does not rise is displacing at least
    that much outflow.

    mode 'switched', eq. (46): the floor is v_lo until the outflow is
    detected and v_hi while it is latched. The latch is set when, for
    t_on, the block was clamped with a forward output, the error
    exceeded e_on, the piston moved at least v_on and the pressure
    rose at less than r_on. It is released when the pressure rises at
    more than r_off with the error at or below e_off (the pressure at
    or above the target): the outlet has shut under a moving piston.
    No stiffness value is used.

    mode 'estimated', eq. (47): the floor is the outflow speed estimated
    with an assumed stiffness k_lo, plus a margin, held within
    [v_lo, v_hi]:

      v_out = (dx - dP / k_lo) / (W Ts)
      floor = clamp(v_out + margin, v_lo, v_hi)

    k_lo below the true stiffness overstates the outflow while the
    pressure falls and understates it while the pressure rises.

    mode 'fixed': the floor is v_hi throughout and never latched, as
    the configured StopVelMin; it isolates the effect of
    forward_stop.

    forward_stop, eq. (48): while the floor is not latched (mode
    'switched') or is at v_lo (mode 'estimated') and the error is
    negative, the forward command is limited to zero. The stop law
    limits the speed in either direction, so without it an
    integrator that holds a forward velocity keeps the piston moving
    into a chamber already above the target.

    Arguments (slave units: mm, mm/s, MPa, s):
      mode      'fixed', 'switched' or 'estimated'.
      v_lo      [mm/s] Floor with no outflow detected.
      v_hi      [mm/s] Floor while outflow is detected (switched), or
                the highest floor (estimated).
      window    [s] Window W Ts over which dx and dP are taken.
      lag       [s] Delay d Ts of the position; None derives it from
                the slave configuration, measurement_lag_scans().
      v_on      [mm/s] Least piston speed over the window for the
                trigger.
      r_on      [MPa/s] Pressure rise rate below which the trigger
                counts.
      e_on      [MPa] Error above which the trigger counts.
      t_on      [s] Time the trigger condition must hold.
      r_off     [MPa/s] Pressure rise rate above which the release
                counts.
      e_off     [MPa] Error at or below which the release counts.
      k_lo      [MPa/mm] Stiffness assumed by the estimate.
      margin    [mm/s] Added to the estimated outflow speed.
      forward_stop  Apply eq. (48).
    """

    def __init__(self, mode='switched', v_lo=0.5, v_hi=2.0, window=0.1,
                 lag=None, v_on=0.2, r_on=0.1, e_on=0.05, t_on=0.04,
                 r_off=1.0, e_off=0.0, k_lo=4.0, margin=0.3,
                 forward_stop=False):
        if mode not in ('fixed', 'switched', 'estimated'):
            raise ValueError(f'unknown floor mode {mode!r}')
        self.mode = mode
        self.v_lo, self.v_hi = v_lo, v_hi                       # [mm/s]
        self.window, self.lag = window, lag                     # [s]
        self.v_on, self.r_on = v_on, r_on              # [mm/s], [MPa/s]
        self.e_on, self.t_on = e_on, t_on                 # [MPa], [s]
        self.r_off, self.e_off = r_off, e_off          # [MPa/s], [MPa]
        self.k_lo, self.margin = k_lo, margin       # [MPa/mm], [mm/s]
        self.forward_stop = forward_stop

    def reset(self, c):
        """Prepare a run with the slave configuration c."""
        self.Ts = c['G_cfgScanTime']                            # [s]
        self.W = max(1, round(self.window / self.Ts))           # [-]
        if self.lag is None:
            self.d = measurement_lag_scans(c)                   # [-]
        else:
            self.d = round(self.lag / self.Ts)                  # [-]
        self.P_hist = []                    # [MPa] last W + 1 readings
        self.x_hist = []                    # [mm] last W + d + 1
        self.n_on = round(self.t_on / self.Ts)                  # [-]
        self.count = 0                      # [-] scans the trigger held
        self.latched = False
        if self.mode == 'fixed':
            self.floor = self.v_hi                              # [mm/s]
        else:
            self.floor = self.v_lo                              # [mm/s]
        # Per-scan record, for the studies
        self.trace_floor = []                                   # [mm/s]
        self.trace_latched = []

    def step(self, P, x, err, saturated, v_out_prev):
        """Return this scan's floor [mm/s] from the measured pressure P
        [MPa], the piston position x [mm], the error err [MPa], and the
        block's saturation flag and velocity [mm/s] of the previous
        scan (the block's bSaturated and Velocity)."""
        self.P_hist.append(P)
        self.x_hist.append(x)
        if len(self.P_hist) > self.W + 1:
            del self.P_hist[0]
        if len(self.x_hist) > self.W + self.d + 1:
            del self.x_hist[0]
        full = len(self.x_hist) == self.W + self.d + 1
        if full:
            T = self.W * self.Ts                                # [s]
            dP = self.P_hist[-1] - self.P_hist[0]               # [MPa]
            dx = (self.x_hist[self.W] - self.x_hist[0])         # [mm]
            if self.mode == 'fixed':
                self.floor = self.v_hi
            elif self.mode == 'switched':
                self._switch(dP / T, dx / T, err, saturated,
                             v_out_prev)
            else:
                # Outflow speed with the assumed stiffness, eq. (47)
                # [mm - MPa / (MPa/mm)] / [s] -> [mm/s]
                v_out = (dx - dP / self.k_lo) / T
                self.floor = min(max(v_out + self.margin, self.v_lo),
                                 self.v_hi)
                self.latched = self.floor > self.v_lo
        self.trace_floor.append(self.floor)
        self.trace_latched.append(self.latched)
        return self.floor

    def _switch(self, rate, speed, err, saturated, v_out_prev):
        """Update the latch of eq. (46) from the pressure rise rate
        [MPa/s] and the piston speed [mm/s] over the window."""
        if self.latched:
            if rate > self.r_off and err <= self.e_off:
                self.latched = False
                self.count = 0
        else:
            stuck = (saturated and v_out_prev > 0.0
                     and err > self.e_on and speed >= self.v_on
                     and rate < self.r_on)
            self.count = self.count + 1 if stuck else 0
            if self.count >= self.n_on:
                self.latched = True
        self.floor = self.v_hi if self.latched else self.v_lo

    def limit_forward(self, v, err):
        """Return the velocity command v [mm/s] with eq. (48) applied
        at the error err [MPa]."""
        if self.forward_stop and not self.latched and err < 0.0:
            return min(v, 0.0)
        return v


class SlaveController:
    """The slave's FB_PressureController wired as PRG_Main wires it.

    Arguments:
      name         Label used in tables and plots.
      flow_ff      If True, add the expected outflow divided by the
                   piston area to the velocity after the block, eq. (20).
                   The slave has no flow feedforward and the machine
                   cannot supply the flow, so this is a bound for the
                   studies, not a candidate.
      floor        A FloorPolicy that sets the block's StopVelMin every
                   scan, or None for the configured StopVelMin.
      **overrides  Slave settings, by full G_cfg* name or by the name
                   without G_cfg and an optional Pressure or Analog
                   prefix, in the units of the GVL. For example Kp=1.0,
                   Accel=100, StopStiffness=20 or
                   AnalogFilterTimeConst=0.02.
    """

    mode = 'V'

    def __init__(self, name='slave', flow_ff=False, floor=None,
                 **overrides):
        known = slave.slave_config()
        self.name = name
        self.flow_ff = flow_ff
        self.floor = floor
        self.cfg = {_cfg_name(k, known): v for k, v in overrides.items()}

    def reset(self, par, scenario):
        self.par = par
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
        # The block's own floor, eqs. (46) and (48); off when a
        # FloorPolicy sets the floor in its place
        fb.FloorEnable = (c['G_cfgPressureFloorEnable']
                          and self.floor is None)
        fb.VelLow = c['G_cfgPressureFloorVelLow']       # [mm/s]
        fb.Window = c['G_cfgPressureFloorWindow']       # [s]
        fb.FilterTime = c['G_cfgAnalogFilterTimeConst']  # [s]
        fb.MedianSize = c['G_cfgAnalogMedianSize']      # [-]
        fb.TrigVel = c['G_cfgPressureFloorTrigVel']     # [mm/s]
        fb.TrigRate = c['G_cfgPressureFloorTrigRate']   # [MPa/s]
        fb.TrigError = c['G_cfgPressureFloorTrigError']  # [MPa]
        fb.TrigTime = c['G_cfgPressureFloorTrigTime']   # [s]
        fb.RelRate = c['G_cfgPressureFloorRelRate']     # [MPa/s]
        fb.RelError = c['G_cfgPressureFloorRelError']   # [MPa]
        fb.ForwardStop = c['G_cfgPressureFloorForwardStop']
        self.fb = fb
        self.first_scan = True
        if self.floor is not None:
            self.floor.reset(c)
        self.floor_done = False     # floor already set for this scan
        return 0.0

    def update_floor(self, P_meas, target, x):
        """Set the block's StopVelMin for this scan from the floor
        policy and return it [mm/s]. The configured value stands when
        there is no policy."""
        fb = self.fb
        if self.floor is not None and not self.floor_done:
            tgt = min(max(target * 1e-6, 0.0), self.target_max) # [MPa]
            err = tgt - P_meas * 1e-6                           # [MPa]
            # [Pa -> MPa], [m -> mm]
            fb.StopVelMin = self.floor.step(P_meas * 1e-6, x * 1e3,
                                            err, fb.bSaturated,
                                            fb.Velocity)
        self.floor_done = True
        return fb.StopVelMin

    def step(self, P_meas, target, t, scenario, x=0.0):
        fb = self.fb
        # PRG_Main resets the block on the scan that enters the mode.
        fb.Reset = self.first_scan
        self.first_scan = False
        self.update_floor(P_meas, target, x)
        self.floor_done = False

        # PRG_Main limits the target to [0, TargetMax].
        # [Pa -> MPa]
        fb.Target = min(max(target * 1e-6, 0.0), self.target_max)
        fb.Actual = P_meas * 1e-6
        fb.Position = x * 1e3                           # [m -> mm]
        FB_PressureController(fb, self.dt)

        # PRG_Main clamps the block output to VelMax.
        v = min(max(fb.Velocity, -self.vel_max), self.vel_max)  # [mm/s]
        if self.floor is not None:
            # Forward stop above the target, eq. (48)
            v = self.floor.limit_forward(v, fb.rError)

        # Flow feedforward, eq. (20): the piston velocity that displaces
        # the expected outflow. [m^3/s / m^2 -> m/s, * 1e3 -> mm/s]
        if self.flow_ff:
            v_ff = scenario.Qff(t, self.par, P_meas) / self.A * 1e3
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
                StudyParams.amax.
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
        self.par = par
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
            u_ff = scenario.Qff(t, self.par, P_meas) / self.A   # [m/s]
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


# %% ===== SCHEDULED SLAVE CONTROLLER ====================================
def limit_position(par, scenario):
    """[m] Position, in the run's coordinates, at which the chamber is
    down to the cylinder head's passages (the limit switch)."""
    RT = par.R * par.T                                          # [J/mol]
    P0, ng0, nd0 = scenario.initial(par)
    # Stressed liquid, free gas and the air pocket at the transducer,
    # less the cylinder's dilation, eq. (9) with eqs. (46) and (49)
    Vch0 = (par.Vl0 * (1 - P0 / par.K)
            + ng0 * RT / (P0 + par.Patm)
            + par.Patm * par.V_air / (P0 + par.Patm)
            - par.C_series * P0)                                # [m^3]
    return (Vch0 - par.V_passages) / par.A                      # [m]


class ScheduledSlaveController(SlaveController):
    """FB_PressureController with Kp, Ki and the stop law scheduled on
    the estimated chamber stiffness.

    The estimate, eq. (39), is the stiffest chamber consistent with what
    the slave knows. The chamber volume is V_ch = A (x_limit - x) +
    V_dead_cfg, with x_limit the limit-switch position where the piston
    halts and V_dead_cfg the configured dead volume with the hose open
    (a lower bound). The free gas is the least there can be: the
    configured charge less what the configured saline volume can hold
    at the highest pressure seen so far (dissolution lags, so the true
    free gas is never less, and a dip cannot have dissolved more). The
    saline volume is the configured value, or the chamber volume when
    that is smaller (as it is late in a delivery). With K_cfg the
    largest plausible bulk modulus and C_hose_cfg the smallest plausible
    hose compliance (0 when the hose is shut or unknown),

      V_gas_est = max(n_cfg - kH P_ref V_l_cfg, 0) R T / P_abs
      C_est = V_gas_est / P_abs + min(V_l_cfg, V_ch) / K_cfg
              + C_hose_cfg
      k_est = A / C_est

    is never below the true stiffness while V_l_cfg is at or below the
    true saline volume and the hose is open. The gain follows from the
    chosen crossover wc, eq. (40), Kp = wc / k_est, held within
    [kp_min, kp_max], with Ki in the fixed ratio ki_ratio to Kp.

    The integrator is held at zero (through the block's Reset input)
    until the pressure first comes within e_int of the target, because
    with a low scheduled gain the approach would otherwise wind it up
    and the stored velocity would carry the pressure past the target.
    After that the block's own conditional anti-windup applies.

    The stop law, eq. (41), brakes for the chamber there is and for the
    measurement lag: the piston must stop within the error after
    travelling for the lag at its present speed and then decelerating,
    v lag + v^2 / (2 D) <= |e| / k_est, so

      v_stop = -D lag + sqrt((D lag)^2 + 2 D |e| / k_est)

    with lag the filter, median, pipeline and command delays. The block's
    own stop law is this with lag equal to one scan and a fixed
    stiffness; here it is disabled (a negligible Stiffness) and the limit
    is applied through the block's VelMax input, which is equivalent to
    giving the block a Lag input and a scheduled Stiffness.

    FB_PressureController multiplies its stored error integral by Ki, so
    when Ki changes the integral is rescaled by the old over the new Ki
    to keep the integral term continuous; the Structured Text port must
    do the same or integrate Ki * e instead.

    Arguments beyond SlaveController's:
      wc         [rad/s] Target crossover of the pressure loop.
      K_cfg      [Pa] Bulk modulus assumed by the estimate, the largest
                 plausible: 1.0 GPa against the measured 0.72.
      V_l_cfg    [m^3] Saline volume assumed by the estimate, a lower
                 bound on the true value. None uses the run's Vl0.
      V_dead_cfg [m^3] Dead volume (passages, hose and tube) assumed
                 by the estimate, a lower bound on the true value. None
                 uses the run's V_dead.
      C_hose_cfg [m^3/Pa] Hose compliance assumed by the estimate, a
                 lower bound on the true value; 0 leaves it out.
      ratio_cfg  [-] Charge assumed by the estimate. None uses the run's
                 ratio (the recipe is known to the slave); a fixed value
                 stands for a slave that does not know it.
      ki_ratio   [1/s] Ki / Kp.
      kp_min     [(mm/s)/MPa] Bounds on the scheduled Kp.
      kp_max
      lag        [s] Lag used by the stop law: the measurement chain
                 plus the drive's dead time and velocity-loop lag. None
                 derives it from the slave configuration and the plant
                 parameters, as the slave would hold it configured.
      stop_from_k  If False the block's own stop law is kept as
                 configured.
      e_int      [MPa] Error below which the integrator is first
                 released.
    """

    def __init__(self, name='scheduled', wc=6.0, K_cfg=1.0e9, V_l_cfg=None,
                 V_dead_cfg=None, C_hose_cfg=0.0, ratio_cfg=None,
                 ki_ratio=0.4, kp_min=0.001, kp_max=5.0, lag=None,
                 stop_from_k=True, e_int=0.5, flow_ff=False, floor=None,
                 **overrides):
        super().__init__(name, flow_ff, floor, **overrides)
        self.V_dead_cfg = V_dead_cfg
        self.C_hose_cfg = C_hose_cfg                            # [m^3/Pa]
        self.e_int = e_int
        self.wc = wc
        self.K_cfg = K_cfg
        self.V_l_cfg = V_l_cfg
        self.ratio_cfg = ratio_cfg
        self.ki_ratio = ki_ratio
        self.kp_min, self.kp_max = kp_min, kp_max
        self.lag = lag
        self.stop_from_k = stop_from_k

    def reset(self, par, scenario):
        u0 = super().reset(par, scenario)
        c = self.c
        self.RT = par.R * par.T                                 # [J/mol]
        self.Patm = par.Patm                                    # [Pa]
        self.kH = par.kH                                   # [mol/(m^3 Pa)]
        V_l_cfg = par.Vl0 if self.V_l_cfg is None else self.V_l_cfg
        self.V_l_cfg_run = V_l_cfg                              # [m^3]
        if self.ratio_cfg is None:
            ratio = scenario.ratio
        else:
            ratio = self.ratio_cfg
        self.n_cfg = par.n_charge(ratio)                        # [mol]
        # Limit-switch position: where the cylinder is down to its
        # passages. The slave knows it from its own position, and the
        # configured dead volume is the lower bound of the range.
        self.x_limit = limit_position(par, scenario)            # [m]
        if self.V_dead_cfg is None:
            self.V_dead_run = par.V_dead                        # [m^3]
        else:
            self.V_dead_run = self.V_dead_cfg
        # Stop-law constants
        self.decel = c['G_cfgPressureDecel'] * 1e-3             # [m/s^2]
        self.D_mm = c['G_cfgPressureDecel']                     # [mm/s^2]
        self.v_floor = c['G_cfgPressureStopVelMin']             # [mm/s]
        if self.lag is None:
            # Filter time constant, median and pipeline scans, the scan
            # in which the command takes effect, and the drive's dead
            # time and velocity-loop lag
            Ts = c['G_cfgScanTime']
            n_med = 5 if c['G_cfgAnalogMedianSize'] >= 5 else 3
            self.lag_run = (c['G_cfgAnalogFilterTimeConst']
                            + ((n_med - 1) // 2 + 2 + 1) * Ts
                            + par.t_dead + par.tauV)            # [s]
        else:
            self.lag_run = self.lag
        if self.stop_from_k:
            self.fb.Stiffness = 1e-6        # the block's stop law is off
        self.P_ref = 0.0                    # [Pa] highest pressure seen
        self.arrived = False
        self.k_est = math.nan
        self.k_target = math.nan
        self.v_lim = math.nan
        return u0

    def estimate_stiffness(self, P_meas, x):
        """[Pa/m] Upper bound on the chamber stiffness from the measured
        pressure P_meas [Pa] and piston position x [m], eq. (39). The
        highest pressure seen is updated here, once per scan."""
        P_abs = max(P_meas, 0.0) + self.Patm                    # [Pa]
        self.P_ref = max(self.P_ref, P_abs)                     # [Pa]
        return self.stiffness_at(P_meas, x)

    def stiffness_at(self, P, x):
        """[Pa/m] The bound of eq. (39) at the gauge pressure P [Pa] and
        piston position x [m], with the highest pressure seen as it
        stands. It does not update that pressure."""
        P_abs = max(P, 0.0) + self.Patm                         # [Pa]
        V_ch = (max(self.A * (self.x_limit - x), 0.0)
                + self.V_dead_run)                              # [m^3]
        V_l = min(self.V_l_cfg_run, V_ch)                       # [m^3]
        n_sat = self.kH * self.P_ref * self.V_l_cfg_run         # [mol]
        n_gas = max(self.n_cfg - n_sat, 0.0)                    # [mol]
        V_gas = n_gas * self.RT / P_abs                         # [m^3]
        C = (V_gas / P_abs + V_l / self.K_cfg
             + self.C_hose_cfg)                                 # [m^3/Pa]
        return self.A / C

    def stop_velocity(self, err_mpa, k_mm):
        """[mm/s] Stop-law limit for error err_mpa [MPa] at stiffness k_mm
        [MPa/mm] with the measurement lag, eq. (41), never below the
        configured StopVelMin."""
        D, lag = self.D_mm, self.lag_run
        dist = abs(err_mpa) / k_mm                              # [mm]
        v = -D * lag + math.sqrt((D * lag)**2 + 2 * D * dist)
        return max(self.v_floor, v)

    def step(self, P_meas, target, t, scenario, x=0.0):
        fb = self.fb
        k_now = self.estimate_stiffness(P_meas, x)              # [Pa/m]
        self.k_est = k_now
        # The bound at the target pressure, where a gas-dominated
        # chamber is stiffest on the way there, eq. (39a).
        tgt = min(max(target * 1e-6, 0.0), self.target_max)     # [MPa]
        self.k_target = self.stiffness_at(
            max(P_meas, tgt * 1e6), x)                          # [Pa/m]
        k = max(k_now, self.k_target)                           # [Pa/m]
        k_mm = k / 1e9                          # [Pa/m -> MPa/mm]
        # Scheduled gain, eq. (40): [rad/s / (MPa/mm) -> (mm/s)/MPa]
        Kp = min(max(self.wc / k_mm, self.kp_min), self.kp_max)
        Ki = self.ki_ratio * Kp
        # Rescale the stored integral so Ki * rInteg is continuous
        if fb.Ki > 0.0 and Ki != fb.Ki:
            fb.rInteg *= fb.Ki / Ki
        fb.Kp, fb.Ki = Kp, Ki
        err = (min(max(target * 1e-6, 0.0), self.target_max)
               - P_meas * 1e-6)                                 # [MPa]
        if self.stop_from_k:
            # The floor policy, if any, sets the floor of eq. (41).
            self.v_floor = self.update_floor(P_meas, target, x)
            self.v_lim = min(self.vel_max, self.stop_velocity(err, k_mm))
            fb.VelMax = self.v_lim
        # Hold the integrator until the first arrival near the target
        if not self.arrived and abs(err) < self.e_int:
            self.arrived = True
        hold = not self.arrived
        u = super().step(P_meas, target, t, scenario, x)
        if hold:
            fb.rInteg = 0.0
        return u


# %% ===== ANTI-WINDUP AND DERIVATIVE VARIANTS ===========================
class VariantController(SlaveController):
    """The slave's block with a different anti-windup rule or a
    derivative term: comparison arms that are not in the slave.

    The block (FB_PressureController) integrates the error unless its
    output is clamped and the error drives it further into the clamp.
    Each variant changes the stored integral rInteg [MPa s] around the
    block's call and leaves the rest of the block as it is, so the stop
    law, the floor and the forward stop are the slave's. With e the
    error [MPa], I the integral and u_raw, u the block's output before
    and after its clamp [mm/s]:

      'block'     No change: the block's own conditional integration.
      'arrival'   Eq. (50). An approach starts on the first scan, or
                  when the target steps by more than step, with |e|
                  above step. Once e has changed sign, the first error
                  beyond over on the far side clears the integral that
                  drives away from the target, and an error back beyond
                  over on the near side ends the approach with the
                  integral kept. At most one clear per approach.
      'crossing'  Eq. (51). The integral is cleared on every scan in
                  which e changes sign.
      'opposed'   Eq. (52). The integral is cleared whenever the error
                  is beyond over on the side the integral drives
                  toward.
      'bleed'     Eq. (53). In the same condition the integral decays
                  with time constant Tb instead of being cleared.
      'backcalc'  Eq. (54). While the output is clamped the integral
                  tracks the clamp with time constant Tt:
                  dI/dt = e + (u - u_raw) / (Ki Tt).
      'band'      Eq. (55). The integral advances only while |e| is at
                  most band, and is held outside it.

    The derivative term, eq. (56), subtracts Kd times the filtered rate
    of the measured pressure from the block's output before the forward
    stop, within the block's clamp of that scan. The block's anti-windup
    does not see it.

    Arguments:
      name         Label used in tables and plots.
      aw           Anti-windup variant, one of the names above.
      Kd           [(mm/s)/(MPa/s)] Derivative gain; 0 for none.
      Tf           [s] Time constant of the derivative filter.
      step         [MPa] Target step, and error after it, that start an
                   approach ('arrival').
      over         [MPa] Error beyond the target that clears the
                   integral ('arrival', 'opposed') or lets it decay
                   ('bleed').
      Tb           [s] Decay time constant ('bleed').
      Tt           [s] Tracking time constant ('backcalc').
      band         [MPa] Half-width of the integration band ('band').
      **overrides  Slave settings, as for SlaveController.
    """

    VARIANTS = ('block', 'arrival', 'crossing', 'opposed', 'bleed',
                'backcalc', 'band')

    def __init__(self, name='variant', aw='block', Kd=0.0, Tf=0.02,
                 step=0.1, over=0.02, Tb=0.2, Tt=0.5, band=0.1,
                 **overrides):
        if aw not in self.VARIANTS:
            raise ValueError(f'unknown anti-windup variant {aw!r}')
        super().__init__(name, **overrides)
        self.aw = aw
        self.Kd = Kd
        self.Tf = Tf
        self.e_step = step
        self.e_over = over
        self.Tb = Tb
        self.Tt = Tt
        self.e_band = band

    def reset(self, par, scenario):
        u0 = super().reset(par, scenario)
        # Approach state of eq. (50): 0 none, 1 from below, 2 passed
        # from below, -1 from above, -2 passed from above
        self.approach = 0
        self.tgt_prev = None                    # [MPa] previous target
        self.err_prev = None                    # [MPa] previous error
        self.P_prev = None                      # [MPa] previous reading
        self.dP = 0.0                           # [MPa/s] filtered rate
        # [-] Scan numbers in which a rule cleared or bled the integral
        self.scan = 0
        self.changed = []
        return u0

    def clear_before(self, err, tgt):
        """Apply the rules that clear the integral before the block
        runs, eqs. (50) to (53), for the error err and target tgt
        [MPa] of this scan."""
        fb = self.fb
        I0 = fb.rInteg
        if self.aw == 'arrival':
            # Start of an approach: the first scan or a target step
            if (self.tgt_prev is None
                    or abs(tgt - self.tgt_prev) > self.e_step):
                if err > self.e_step:
                    self.approach = 1
                elif err < -self.e_step:
                    self.approach = -1
                else:
                    self.approach = 0
            # The error has changed sign: watch the far side
            if self.approach == 1 and err <= 0.0:
                self.approach = 2
            elif self.approach == -1 and err >= 0.0:
                self.approach = -2
            if self.approach == 2:
                if err < -self.e_over:
                    fb.rInteg = min(fb.rInteg, 0.0)
                    self.approach = 0
                elif err > self.e_over:
                    self.approach = 0
            elif self.approach == -2:
                if err > self.e_over:
                    fb.rInteg = max(fb.rInteg, 0.0)
                    self.approach = 0
                elif err < -self.e_over:
                    self.approach = 0
        elif self.aw == 'crossing':
            if self.err_prev is not None and err * self.err_prev < 0.0:
                fb.rInteg = 0.0
        elif self.aw == 'opposed':
            if err < -self.e_over:
                fb.rInteg = min(fb.rInteg, 0.0)
            elif err > self.e_over:
                fb.rInteg = max(fb.rInteg, 0.0)
        elif self.aw == 'bleed':
            if ((err < -self.e_over and fb.rInteg > 0.0)
                    or (err > self.e_over and fb.rInteg < 0.0)):
                fb.rInteg *= 1.0 - self.dt / self.Tb
        if fb.rInteg != I0:
            self.changed.append(self.scan)
        self.scan += 1

    def step(self, P_meas, target, t, scenario, x=0.0):
        fb = self.fb
        first = self.first_scan
        self.first_scan = False
        # PRG_Main limits the target to [0, TargetMax]. [Pa -> MPa]
        tgt = min(max(target * 1e-6, 0.0), self.target_max)
        act = P_meas * 1e-6                             # [Pa -> MPa]
        err = tgt - act                                 # [MPa]

        self.clear_before(err, tgt)
        # The block clears the integral itself on the entry scan.
        I0 = 0.0 if first else fb.rInteg                # [MPa s]
        fb.Reset = first
        fb.Target = tgt
        fb.Actual = act
        fb.Position = x * 1e3                           # [m -> mm]
        FB_PressureController(fb, self.dt)

        if self.aw == 'backcalc' and fb.bSaturated and fb.Ki > 0.0:
            # Tracking anti-windup, eq. (54), in place of the block's
            # conditional integration while the output is clamped
            fb.rInteg = I0 + (err + (fb.rVelPI - fb.rRaw)
                              / (fb.Ki * self.Tt)) * self.dt
        elif self.aw == 'band' and abs(err) > self.e_band:
            # Integration band, eq. (55): held outside the band
            fb.rInteg = I0

        v = fb.Velocity                                 # [mm/s]
        if self.Kd > 0.0:
            # Filtered rate of the measured pressure, eq. (21), and the
            # derivative term, eq. (56), inside the block's clamp
            if self.P_prev is not None:
                rate = (act - self.P_prev) / self.dt    # [MPa/s]
                self.dP += self.dt / (self.Tf + self.dt) * (rate - self.dP)
            v = fb.rVelPI - self.Kd * self.dP
            v = min(max(v, -fb.rVelLimit), fb.rVelLimit)
            # The block's forward stop, eq. (48), applied again
            if (fb.bFloorOn and fb.ForwardStop and not fb.Latched
                    and err < 0.0 and v > 0.0):
                v = 0.0

        self.tgt_prev = tgt
        self.err_prev = err
        self.P_prev = act
        # PRG_Main clamps the block output to VelMax.
        v = min(max(v, -self.vel_max), self.vel_max)
        return v * 1e-3                                 # [mm/s -> m/s]
