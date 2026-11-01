"""
linear.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module builds a discrete-time linear model of the slave's pressure
loop in velocity-command mode, sampled at the slave scan time Ts, and
computes its stability margins. The studies use it to find how the
margins change with the chamber stiffness k = dP/dx [Pa/m], which varies
by four orders of magnitude between a gas-rich and a liquid-only
chamber.

The loop has five parts, listed in the order a sample travels around
it. README equations (26) to (32) give each part.

  Plant       The drive's velocity loop is a first-order lag
              1/(tauV s + 1), and the chamber integrates the velocity to
              pressure with gain k. The product k/(s (tauV s + 1)) is
              discretized with a zero-order hold, because the command is
              constant during a scan: Gd(z), eq. (26).
  Command     The command computed in scan n acts in scan n + 1, and the
              drive responds a dead time t_dead later: z^-(1 + n_dead).
  Controller  The PI law of FB_PressureController, C(z), eq. (27).
  Sensor      The median of N samples delays a slowly varying signal by
              (N - 1)/2 scans, and the first-order filter is F(z),
              eq. (28).
  Pipeline    PRG_Main reads the block outputs at the top of the scan and
              calls the blocks at the end, which adds two scans: z^-2.

The open loop is L(z), eq. (29). The sensitivity S, complementary
sensitivity T, target response R and outflow response D follow from it,
eqs. (30) to (32).

The model omits the output clamp and stop law, the slew limits on the
command, the nonlinearity of the median filter, and gas dissolution.
clamp_report() and slew_amplitude() give the amplitudes at which the
clamp and the slew limit start to act, so the range of validity of the
model can be checked.

A transfer function is held as a TF: the numerator and denominator in
descending powers of z, and optionally a function that evaluates the
frequency response in factored form. The factored form is needed at low
frequency, where the double pole at z = 1 makes the expanded polynomials
lose precision.

FILE DEPENDENCY:
None. The functions take a StudyParams object (studyparams.py).
"""

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from scipy import signal

# %% ===== UNIT CONVERSIONS ==============================================
# [MPa/mm -> Pa/m] multiply a stiffness in MPa/mm by this
MPA_MM = 1e9
# [(mm/s)/MPa -> (m/s)/Pa] multiply a gain in (mm/s)/MPa by this
KP_UNIT = 1e-9


# %% ===== TRANSFER FUNCTION CONTAINER ===================================
@dataclass
class TF:
    """Discrete-time transfer function with sample time Ts [s].

    num and den hold polynomial coefficients in descending powers of z.
    fr, if given, evaluates the frequency response in factored form and
    takes precedence over the polynomials in freq().
    """

    num: np.ndarray
    den: np.ndarray
    Ts: float
    fr: Optional[Callable] = None

    def freq(self, w):
        """Return the complex response at angular frequencies w
        [rad/s]."""
        w = np.asarray(w, float)
        if self.fr is not None:
            return self.fr(w)
        return _poly_ratio(self.num, self.den, w, self.Ts)

    def step(self, n, scale=1.0):
        """Return the first n samples of the response to a step of size
        scale applied at sample 0."""
        den = np.asarray(self.den, float)
        # Pad the numerator to the length of the denominator so that
        # lfilter treats both as polynomials in z^-1 with the same
        # leading power.
        num = np.concatenate([np.zeros(len(den) - len(self.num)),
                              self.num])
        return signal.lfilter(num, den, scale * np.ones(n))


def _poly(*coefficients):
    """Return polynomial coefficients as a float array."""
    return np.array(coefficients, float)


def _poly_ratio(num, den, w, Ts):
    """Evaluate num(z) / den(z) on the unit circle, z = exp(j w Ts)."""
    z = np.exp(1j * np.asarray(w, float) * Ts)
    return np.polyval(num, z) / np.polyval(den, z)


def _delay(w, Ts, d):
    """Frequency response of a pure delay of d samples, z^-d."""
    return np.exp(-1j * np.asarray(w, float) * Ts * d)


# %% ===== LOOP PARAMETERS FROM THE SLAVE CONFIGURATION ==================
def sample_delay(cfg):
    """Return the scans of pure delay between the chamber pressure and
    the controller input, excluding the first-order filter.

    The delay is (N - 1)/2 for the median filter plus two for the
    PRG_Main pipeline. FB_AnalogInputFilter uses a median of 5 for any
    setting of 5 or more, and a median of 3 otherwise.
    """
    n = 5 if cfg['G_cfgAnalogMedianSize'] >= 5 else 3
    return (n - 1) // 2 + 2


def filter_alpha(cfg):
    """Return the first-order filter coefficient alpha = Ts/(tau + Ts),
    eq. (16), or 1 when the filter is off."""
    tau = cfg['G_cfgAnalogFilterTimeConst']                 # [s]
    Ts = cfg['G_cfgScanTime']                               # [s]
    return Ts / (tau + Ts) if tau > 0 else 1.0


# %% ===== LOOP TRANSFER FUNCTIONS =======================================
def loop(par, k, delay=None, **cfg_overrides):
    """Return the loop transfer functions at chamber stiffness k [Pa/m].

    par supplies the piston area A, the drive lag tauV and the slave
    configuration. cfg_overrides are G_cfg* settings such as
    G_cfgPressureKp=1.0. delay replaces the sensor-path delay from
    sample_delay(); the tests use it to confirm the delay count.

    The result is a dict with these entries:
      L, S, T     open loop, sensitivity, complementary sensitivity
      R           target to pressure (the target reaches the controller
                  without passing through the sensor)
      D           outflow Q [m^3/s] to pressure [Pa]; negative
      Gd, C, F    plant, controller and filter
      cl          closed-loop characteristic polynomial
      Kp, Ki      [(mm/s)/MPa], [(mm/s)/(MPa s)] controller gains
      Ts          [s] scan time
      delay       [scans] total pure delay, including the command's scan
                  and the drive's dead time
      k           [Pa/m] stiffness
    """
    cfg = par.cfg(**cfg_overrides)
    Ts = cfg['G_cfgScanTime']                               # [s]
    Kp = cfg['G_cfgPressureKp']                             # [(mm/s)/MPa]
    Ki = cfg['G_cfgPressureKi']                         # [(mm/s)/(MPa s)]
    d_sensor = sample_delay(cfg) if delay is None else delay
    n_dead = int(round(par.t_dead / Ts))        # [scans] drive dead time
    d = d_sensor + 1 + n_dead           # [scans] plus the command's scan

    # Plant, eq. (26): zero-order-hold discretization of
    # k / (s (tauV s + 1)). With a_v = exp(-Ts/tauV) and
    # t_v = tauV (1 - a_v),
    #   Gd(z) = k ((Ts - t_v) z + (t_v - a_v Ts)) / ((z - 1)(z - a_v))
    a_v = np.exp(-Ts / par.tauV)
    t_v = par.tauV * (1 - a_v)                              # [s]
    Gd_num = k * _poly(Ts - t_v, t_v - a_v * Ts)
    Gd_den = np.polymul(_poly(1, -1), _poly(1, -a_v))

    def Gd_freq(w):
        return _poly_ratio(Gd_num, Gd_den, w, Ts)
    Gd = TF(Gd_num, Gd_den, Ts, Gd_freq)

    # Controller, eq. (27): C(z) = Kp + Ki Ts / (z - 1)
    #                            = (Kp z + (Ki Ts - Kp)) / (z - 1),
    # with the gains converted from (mm/s)/MPa to (m/s)/Pa
    C_num = _poly(Kp * KP_UNIT, (-Kp + Ki * Ts) * KP_UNIT)
    C_den = _poly(1, -1)

    def C_freq(w):
        return _poly_ratio(C_num, C_den, w, Ts)
    C = TF(C_num, C_den, Ts, C_freq)

    # Sensor filter, eq. (28): F(z) = alpha z / (z - (1 - alpha))
    alpha = filter_alpha(cfg)
    F_num = _poly(alpha, 0)
    F_den = _poly(1, -(1 - alpha))

    def F_freq(w):
        return _poly_ratio(F_num, F_den, w, Ts)
    F = TF(F_num, F_den, Ts, F_freq)

    # Open loop, eq. (29): L(z) = C(z) z^-(1+n_dead) Gd(z) z^-d_sensor
    # F(z). The d scans of delay appear as z^d in the denominator.
    z_pow_d = np.zeros(d + 1)
    z_pow_d[0] = 1.0                                        # z^d
    L_num = np.polymul(np.polymul(C_num, Gd_num), F_num)
    L_den = np.polymul(np.polymul(np.polymul(C_den, Gd_den), F_den),
                       z_pow_d)

    def L_freq(w):
        """L evaluated factor by factor"""
        return C_freq(w) * Gd_freq(w) * F_freq(w) * _delay(w, Ts, d)
    L = TF(L_num, L_den, Ts, L_freq)

    # Closed-loop characteristic polynomial: the numerator of 1 + L
    cl = np.polyadd(L_den, L_num)

    # Sensitivity and complementary sensitivity, eq. (30):
    #   S = 1 / (1 + L),   T = L / (1 + L)
    def S_freq(w):
        return 1.0 / (1.0 + L_freq(w))

    def T_freq(w):
        return L_freq(w) / (1.0 + L_freq(w))
    S = TF(L_den, cl, Ts, S_freq)
    T = TF(L_num, cl, Ts, T_freq)

    # Target to pressure, eq. (31): R = C z^-(1+n_dead) Gd S. The target
    # enters at the controller, so the sensor path does not act on it.
    z_pow_rest = np.zeros(d_sensor + 1)
    z_pow_rest[0] = 1.0                                     # z^d_sensor
    R_num = np.polymul(np.polymul(np.polymul(C_num, Gd_num), F_den),
                       z_pow_rest)

    def R_freq(w):
        return (C_freq(w) * Gd_freq(w) * _delay(w, Ts, 1 + n_dead)
                * S_freq(w))
    R = TF(R_num, cl, Ts, R_freq)

    # Outflow to pressure, eq. (32): D = -(k Ts / A) / (z - 1) S.
    # The outflow changes the chamber volume directly, so it enters at
    # the integrator, after the drive lag. -(k Ts / A) / (z - 1) is the
    # zero-order-hold integrator driven by the velocity -Q/A.
    Dq_num = _poly(-k * Ts / par.A)
    Dq_den = _poly(1, -1)

    def D_freq(w):
        return _poly_ratio(Dq_num, Dq_den, w, Ts) * S_freq(w)
    D = TF(np.polymul(Dq_num, L_den), np.polymul(Dq_den, cl), Ts, D_freq)

    return dict(L=L, S=S, T=T, R=R, Gd=Gd, D=D, C=C, F=F, Kp=Kp, Ki=Ki,
                Ts=Ts, delay=d, k=k, cl=cl)


def poles(lp):
    """Return the closed-loop poles of the loop dict lp."""
    return np.roots(lp['cl'])


def is_stable(lp):
    """Return True if every closed-loop pole lies inside the unit
    circle."""
    return bool(np.max(np.abs(poles(lp))) < 1.0)


# %% ===== STABILITY MARGINS =============================================
def freq_grid(Ts, n=6000, wmin=1e-6):
    """Return n log-spaced angular frequencies [rad/s] from wmin to the
    Nyquist frequency pi/Ts."""
    return np.logspace(np.log10(wmin), np.log10(np.pi / Ts), n)


def _cross(x, y, level, descending=True):
    """Find where y crosses level going down (or up if descending is
    False).

    Returns a list of (j, f, x_cross): the crossing lies between samples
    j and j+1 at fraction f of the way, and x_cross is interpolated
    linearly in log(x).
    """
    s = (y - level) if descending else (level - y)
    i = np.nonzero((s[:-1] > 0) & (s[1:] <= 0))[0]
    out = []
    for j in i:
        f = s[j] / (s[j] - s[j + 1])
        log_x = np.log(x[j]) + f * (np.log(x[j + 1]) - np.log(x[j]))
        out.append((j, f, np.exp(log_x)))
    return out


def margins(L, w=None):
    """Return the gain, phase and delay margins of open loop L under
    negative feedback.

    The result is a dict:
      gm       [-] gain margin, the factor by which the loop gain can
               rise before instability; above 1 is stable, and inf if the
               phase never reaches -180 deg
      gm_db    [dB] gain margin
      wpc      [rad/s] phase crossover, where the phase is -180 deg
      pm       [deg] phase margin, the smallest over all crossings of
               |L| = 1; NaN if |L| stays below 1
      wc       [rad/s] gain crossover at which pm occurs
      dm       [s] delay margin, the added delay that removes the phase
               margin: dm = pm / wc (pm in rad)
      wc_all   [rad/s] every gain crossover
    """
    w = freq_grid(L.Ts) if w is None else w
    H = L.freq(w)
    mag = np.abs(H)
    phase = np.unwrap(np.angle(H))                          # [rad]
    # Shift the unwrapped phase by a whole number of turns so that it
    # starts in (-3 pi/2, pi/2]. An integrator then starts near -pi/2 and
    # the -pi crossing is the first one below it.
    phase -= 2 * np.pi * np.ceil((phase[0] - np.pi / 2) / (2 * np.pi))

    r = dict(gm=np.inf, gm_db=np.inf, wpc=np.nan, pm=np.nan, wc=np.nan,
             dm=np.nan, wc_all=[])

    # Gain margin: 1/|L| at each -180 deg crossing; keep the smallest.
    # |L| is interpolated linearly in log(|L|).
    for j, f, w_x in _cross(w, phase, -np.pi):
        log_mag = np.log(mag[j]) + f * (np.log(mag[j + 1])
                                        - np.log(mag[j]))
        g = 1.0 / np.exp(log_mag)
        if g < r['gm']:
            r.update(gm=g, gm_db=20 * np.log10(g), wpc=w_x)

    # Phase margin: 180 deg plus the phase at each |L| = 1 crossing;
    # keep the smallest.
    best = np.inf
    for j, f, w_x in _cross(w, mag, 1.0):
        phase_x = phase[j] + f * (phase[j + 1] - phase[j])  # [rad]
        pm = np.degrees(phase_x + np.pi)                    # [deg]
        r['wc_all'].append(w_x)
        if pm < best:
            best = pm
            r.update(pm=pm, wc=w_x, dm=np.radians(pm) / w_x)
    return r


# %% ===== GAIN LIMITS ===================================================
def _kp_grid(k, lo, hi, n):
    """Return a log grid of Kp [(mm/s)/MPa].

    The default range puts Kp k, which approximates the gain crossover
    [rad/s] (Kp in (mm/s)/MPa and k in MPa/mm), between 1e-4 and 1e3.
    """
    k_mpa_mm = k / MPA_MM                       # [Pa/m -> MPa/mm]
    return np.logspace(np.log10(lo or 1e-4 / k_mpa_mm),
                       np.log10(hi or 1e3 / k_mpa_mm), n)


def _loop_kp(par, k, kp, ki_ratio, cfg_overrides):
    """Return loop() at gain kp. If ki_ratio is given, Ki = ki_ratio kp,
    so that the loop depends on kp and k only through kp k."""
    o = {**cfg_overrides, 'G_cfgPressureKp': kp}
    if ki_ratio is not None:
        o['G_cfgPressureKi'] = ki_ratio * kp
    return loop(par, k, **o)


def _refine(a, b, passes, n_bisect):
    """Bisect in log space between a (passes) and b (fails) and return
    the last value that passes."""
    for _ in range(n_bisect):
        m = np.sqrt(a * b)
        if passes(m):
            a = m
        else:
            b = m
    return a


def max_stable_kp(par, k, ngrid=160, lo=None, hi=None, ki_ratio=None,
                  **cfg_overrides):
    """Return the largest Kp [(mm/s)/MPa] for which the closed loop is
    stable at stiffness k [Pa/m].

    The other settings come from the GVL and cfg_overrides. The search
    scans a log grid upward and refines the last stable-to-unstable
    transition by bisection. It returns inf if the loop is stable over
    the whole grid, and 0 if it is unstable even at the smallest Kp.
    ki_ratio sets Ki = ki_ratio Kp at each Kp.
    """
    def stable(kp):
        return is_stable(_loop_kp(par, k, kp, ki_ratio, cfg_overrides))

    grid = _kp_grid(k, lo, hi, ngrid)
    ok = [stable(x) for x in grid]
    if not any(ok):
        return 0.0
    i = max(j for j, s in enumerate(ok) if s)
    if i == len(grid) - 1:
        return np.inf
    return _refine(grid[i], grid[i + 1], stable, 40)


def kp_for_pm(par, k, pm_deg, ngrid=120, lo=None, hi=None, ki_ratio=None,
              **cfg_overrides):
    """Return the largest Kp [(mm/s)/MPa] whose phase margin is at least
    pm_deg [deg] at stiffness k [Pa/m], or NaN if no Kp on the grid
    achieves it."""
    def phase_margin(kp):
        lp = _loop_kp(par, k, kp, ki_ratio, cfg_overrides)
        if not is_stable(lp):
            return -np.inf
        return margins(lp['L'], freq_grid(lp['Ts'], 1500))['pm']

    def passes(kp):
        return phase_margin(kp) >= pm_deg

    grid = _kp_grid(k, lo, hi, ngrid)
    good = [j for j, x in enumerate(grid) if passes(x)]
    if not good:
        return np.nan
    i = max(good)
    if i == len(grid) - 1:
        return np.inf
    return _refine(grid[i], grid[i + 1], passes, 30)


# %% ===== OUTFLOW DISTURBANCE ===========================================
def static_dip_rule(par, Q, Kp):
    """Return the quasi-static pressure dip [MPa] for outflow Q [m^3/s]
    and gain Kp [(mm/s)/MPa], eq. (33).

    In steady state the proportional term must command the velocity
    Q/A that displaces the outflow, so the error is
    dip = Q / (A Kp).
    """
    # [m^3/s / m^2 -> m/s], [(mm/s)/MPa * 1e-3 -> (m/s)/MPa]
    return Q / par.A / (Kp * 1e-3)


def step_dip(lp, Q, n):
    """Return the response of loop lp to an outflow step of Q [m^3/s].

    Returns (dip [Pa], positive for a fall in pressure; time of the dip
    [s]; the first n samples of the pressure deviation [Pa]).
    """
    y = lp['D'].step(n, Q)
    j = int(np.argmin(y))
    return -y[j], j * lp['Ts'], y


# %% ===== RANGE OF VALIDITY OF THE LINEAR MODEL =========================
def clamp_report(cfg, Kp, e_mpa):
    """Check whether the output clamp of FB_PressureController acts for
    a constant error e_mpa [MPa].

    The clamp is the stop law, eq. (19):
      v_stop = -D Ts + sqrt((D Ts)^2 + 2 D |e| / S_stop)
      v_lim  = min(VelMax, max(StopVelMin, v_stop))
    with D the decel limit [mm/s^2] and S_stop the configured stop-law
    stiffness [MPa/mm].

    Returns (|Kp e| [mm/s], v_lim [mm/s], True if the clamp acts).
    """
    D = cfg['G_cfgPressureDecel']                           # [mm/s^2]
    S_stop = cfg['G_cfgPressureStopStiffness']              # [MPa/mm]
    Ts = cfg['G_cfgScanTime']                               # [s]
    dv = D * Ts                         # [mm/s] velocity change per scan
    v_stop = -dv + np.sqrt(dv * dv + 2 * D * abs(e_mpa) / S_stop)
    v_lim = min(cfg['G_cfgPressureVelMax'],
                max(cfg['G_cfgPressureStopVelMin'], v_stop))
    v = abs(Kp * e_mpa)
    return v, v_lim, v > v_lim


def clamp_error(cfg, Kp):
    """Return the smallest constant error [MPa] for which the
    proportional term alone exceeds the clamp, or inf if none up to
    100 MPa does."""
    errors = np.logspace(-4, 2, 6000)                       # [MPa]
    acts = [clamp_report(cfg, Kp, e)[2] for e in errors]
    return errors[acts.index(True)] if any(acts) else np.inf


def slew_amplitude(cfg, lp, w):
    """Return the amplitude [MPa] of a sinusoidal error at frequency w
    [rad/s] for which the rate of change of the command, w |C(w)| e,
    reaches the accel limit [mm/s^2]."""
    C_mag = np.abs(lp['C'].freq(w)) / KP_UNIT    # [(mm/s)/MPa] |C| at w
    return cfg['G_cfgPressureAccel'] / (np.asarray(w) * C_mag)


def slew_step_error(cfg, Kp):
    """Return the step in error [MPa] for which the one-scan jump in
    command, Kp e, reaches the accel limit times one scan."""
    return cfg['G_cfgPressureAccel'] * cfg['G_cfgScanTime'] / Kp
