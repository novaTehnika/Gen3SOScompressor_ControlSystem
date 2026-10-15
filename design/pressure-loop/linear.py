"""Linear model of the slave pressure loop (velocity command), discrete time at the slave scan Ts.

Loop, in the order a sample travels around it (SI inside, pressure in Pa, k in Pa/m):

    plant       P/v = k/s behind the drive velocity lag 1/(tauV s + 1), zero-order-hold
                discretised per scan (the command is constant during a scan): Gd(z)
    command     FB_PressureController output computed in scan n acts in scan n+1: z^-1
    controller  v = Kp e + Ki I, I the sum of the errors of the earlier scans:
                C(z) = Kp + Ki Ts / (z - 1)  (Kp, Ki converted from mm/s per MPa to m/s per Pa)
    sensor      median of n samples is a delay of (n-1)/2 scans for a slowly varying signal,
                then y = a x + (1 - a) y with a = Ts / (tau + Ts): F(z) = a z / (z - (1 - a))
    pipeline    PRG_Main reads the filter and scaling outputs at the top of the scan and calls
                the blocks at the end, so the controller sees the filter output of two scans
                earlier: z^-2

    L(z) = C(z) z^-1 Gd(z) z^-((n-1)/2) z^-2 F(z)        (5 scans of pure delay for n = 5)
    S = 1/(1 + L),  T = L/(1 + L)  (measurement to measurement),
    R = C z^-1 Gd S (target to pressure: the target skips the sensor path)
    outflow Q acts on the chamber volume, past the drive lag: P/Q = -(k Ts / A) / (z - 1) S
    (the integrator's ZOH, equal to the velocity -Q/A entering at the integrator)

Ignored: the clamp and stop law, the accel/decel slew of the command, the median's nonlinearity,
dissolution. `clamp_report` and `slew_amplitude` give the amplitudes at which the first two bind.

A transfer function is a TF: num and den in descending powers of z and, for the response at low
frequency where polynomials lose precision next to the double pole at z = 1, `fr`, a callable
that evaluates it in factored form.
"""

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from scipy import signal

MPA_MM = 1e9          # Pa/m per MPa/mm
KP_UNIT = 1e-9        # (m/s)/Pa per (mm/s)/MPa


@dataclass
class TF:
    num: np.ndarray
    den: np.ndarray
    Ts: float
    fr: Optional[Callable] = None

    def freq(self, w):
        """complex response at angular frequencies w (rad/s)"""
        w = np.asarray(w, float)
        if self.fr is not None:
            return self.fr(w)
        z = np.exp(1j * w * self.Ts)
        return np.polyval(self.num, z) / np.polyval(self.den, z)

    def step(self, n, scale=1.0):
        """response y[0..n-1] to scale * unit step starting at sample 0"""
        den = np.asarray(self.den, float)
        num = np.concatenate([np.zeros(len(den) - len(self.num)), self.num])
        return signal.lfilter(num, den, scale * np.ones(n))


def _p(*c):
    return np.array(c, float)


def sample_delay(cfg):
    """scans of pure delay between the chamber pressure and the controller input, excluding the
    filter: the median's (n-1)/2 (FB_AnalogInputFilter uses 5 for a setting >= 5, else 3) plus the
    two-scan pipeline"""
    n = 5 if cfg['G_cfgAnalogMedianSize'] >= 5 else 3
    return (n - 1) // 2 + 2


def filter_alpha(cfg):
    tau = cfg['G_cfgAnalogFilterTimeConst']
    return cfg['G_cfgScanTime'] / (tau + cfg['G_cfgScanTime']) if tau > 0 else 1.0


def loop(p, k, delay=None, **cfg_overrides):
    """Loop transfer functions at stiffness k (Pa/m).

    p: Params (A, tauV and the slave configuration); cfg_overrides: G_cfg* entries such as
    G_cfgPressureKp=1.0. delay: scans of pure delay in the sensor path (default `sample_delay`),
    for checking the count. Returns dict with L, S, T, R (target to pressure; the target reaches
    the controller without the sensor path), Gd (plant, v to P), D (outflow Q to P, Pa per
    m^3/s, negative), cl (closed-loop characteristic polynomial), C (controller), F (filter), Kp, Ki, Ts, delay (total scans in the loop incl. the
    command's) and k.
    """
    cfg = p.cfg(**cfg_overrides)
    Ts = cfg['G_cfgScanTime']
    Kp, Ki = cfg['G_cfgPressureKp'], cfg['G_cfgPressureKi']
    d_sens = sample_delay(cfg) if delay is None else delay
    d = d_sens + 1                                            # plus the command's scan

    # plant: zero-order-hold discretisation of k / (s (tauV s + 1))
    a_v = np.exp(-Ts / p.tauV)
    tv = p.tauV * (1 - a_v)
    gnum = k * _p(Ts - tv, tv - a_v * Ts)
    gden = np.polymul(_p(1, -1), _p(1, -a_v))
    Gd = TF(gnum, gden, Ts, lambda w: _ev(gnum, gden, w, Ts))

    cnum = _p(Kp * KP_UNIT, (-Kp + Ki * Ts) * KP_UNIT)        # (Kp (z-1) + Ki Ts) / (z - 1)
    cden = _p(1, -1)
    C = TF(cnum, cden, Ts, lambda w: _ev(cnum, cden, w, Ts))

    a = filter_alpha(cfg)
    fnum, fden = _p(a, 0), _p(1, -(1 - a))
    F = TF(fnum, fden, Ts, lambda w: _ev(fnum, fden, w, Ts))

    zd = np.zeros(d + 1)
    zd[0] = 1.0                                               # z^d
    Lnum = np.polymul(np.polymul(cnum, gnum), fnum)
    Lden = np.polymul(np.polymul(np.polymul(cden, gden), fden), zd)
    Lfr = lambda w: C.freq(w) * Gd.freq(w) * F.freq(w) * np.exp(-1j * w * Ts * d)
    cl = np.polyadd(Lden, Lnum)
    L = TF(Lnum, Lden, Ts, Lfr)
    S = TF(Lden, cl, Ts, lambda w: 1.0 / (1.0 + Lfr(w)))
    T = TF(Lnum, cl, Ts, lambda w: Lfr(w) / (1.0 + Lfr(w)))
    zr = np.zeros(d)
    zr[0] = 1.0                                               # z^(d-1)
    R = TF(np.polymul(np.polymul(np.polymul(cnum, gnum), fden), zr), cl, Ts,
           lambda w: C.freq(w) * Gd.freq(w) * np.exp(-1j * w * Ts) / (1.0 + Lfr(w)))
    dnum, dden = _p(-k * Ts / p.A), _p(1, -1)                 # outflow acts on the integrator, past the drive lag
    D = TF(np.polymul(dnum, Lden), np.polymul(dden, cl), Ts,
           lambda w: _ev(dnum, dden, w, Ts) / (1.0 + Lfr(w)))
    return dict(L=L, S=S, T=T, R=R, Gd=Gd, D=D, C=C, F=F, Kp=Kp, Ki=Ki, Ts=Ts, delay=d, k=k, cl=cl)


def _ev(num, den, w, Ts):
    z = np.exp(1j * np.asarray(w, float) * Ts)
    return np.polyval(num, z) / np.polyval(den, z)


def poles(lp):
    return np.roots(lp['cl'])


def is_stable(lp):
    return bool(np.max(np.abs(poles(lp))) < 1.0)


def freq_grid(Ts, n=6000, wmin=1e-6):
    return np.logspace(np.log10(wmin), np.log10(np.pi / Ts), n)


def _cross(x, y, level, descending=True):
    """interpolated x (log-linear in x) where y crosses `level` going down (or up)"""
    s = (y - level) if descending else (level - y)
    i = np.nonzero((s[:-1] > 0) & (s[1:] <= 0))[0]
    out = []
    for j in i:
        f = s[j] / (s[j] - s[j + 1])
        out.append((j, f, np.exp(np.log(x[j]) + f * (np.log(x[j + 1]) - np.log(x[j])))))
    return out


def margins(L, w=None):
    """Gain and phase margins of a TF L (open loop, negative feedback).

    Returns dict: gm (ratio, >1 stable side; inf if the phase never reaches -180), gm_db, wpc
    (phase crossover), pm (deg, the smallest over the |L| = 1 crossings), wc (that crossing),
    dm (delay margin s = pm / wc), wc_all (all crossings). pm and wc are nan if |L| stays below 1.
    """
    w = freq_grid(L.Ts) if w is None else w
    H = L.freq(w)
    mag = np.abs(H)
    ph = np.unwrap(np.angle(H))
    ph -= 2 * np.pi * np.ceil((ph[0] - np.pi / 2) / (2 * np.pi))          # start in (-3pi/2, pi/2]
    r = dict(gm=np.inf, gm_db=np.inf, wpc=np.nan, pm=np.nan, wc=np.nan, dm=np.nan, wc_all=[])
    for j, f, wx in _cross(w, ph, -np.pi):
        g = 1.0 / np.exp(np.log(mag[j]) + f * (np.log(mag[j + 1]) - np.log(mag[j])))
        if g < r['gm']:
            r.update(gm=g, gm_db=20 * np.log10(g), wpc=wx)
    best = np.inf
    for j, f, wx in _cross(w, mag, 1.0):
        pm = np.degrees(ph[j] + f * (ph[j + 1] - ph[j]) + np.pi)
        r['wc_all'].append(wx)
        if pm < best:
            best = pm
            r.update(pm=pm, wc=wx, dm=np.radians(pm) / wx)
    return r


def max_stable_kp(p, k, ngrid=160, lo=None, hi=None, ki_ratio=None, **cfg_overrides):
    """Largest Kp (mm/s per MPa) at which the closed loop is stable, others at the overrides / GVL.

    Scans a log grid upward and refines the last stable to unstable transition by bisection;
    inf if stable over the whole grid, 0 if unstable even at the smallest Kp. ki_ratio sets
    Ki = ki_ratio Kp at each Kp (then L depends on Kp k only).
    """
    ok = lambda kp: is_stable(_loop_kp(p, k, kp, ki_ratio, cfg_overrides))
    grid = _kp_grid(k, lo, hi, ngrid)
    st = [ok(x) for x in grid]
    if not any(st):
        return 0.0
    i = max(j for j, s in enumerate(st) if s)
    if i == len(grid) - 1:
        return np.inf
    a, b = grid[i], grid[i + 1]
    for _ in range(40):
        m = np.sqrt(a * b)
        a, b = (m, b) if ok(m) else (a, m)
    return a


def _kp_grid(k, lo, hi, n):
    """log grid of Kp around the range where Kp k, the nominal crossover (rad/s), is 1e-4 to 1e3"""
    kmm = k / MPA_MM
    return np.logspace(np.log10(lo or 1e-4 / kmm), np.log10(hi or 1e3 / kmm), n)


def _loop_kp(p, k, kp, ki_ratio, cfg_overrides):
    o = {**cfg_overrides, 'G_cfgPressureKp': kp}
    if ki_ratio is not None:
        o['G_cfgPressureKi'] = ki_ratio * kp
    return loop(p, k, **o)


def kp_for_pm(p, k, pm_deg, ngrid=120, lo=None, hi=None, ki_ratio=None, **cfg_overrides):
    """Largest Kp (mm/s per MPa) whose phase margin is at least pm_deg (nan if none)."""
    def pm(kp):
        lp = _loop_kp(p, k, kp, ki_ratio, cfg_overrides)
        return margins(lp['L'], freq_grid(lp['Ts'], 1500))['pm'] if is_stable(lp) else -np.inf
    grid = _kp_grid(k, lo, hi, ngrid)
    good = [j for j, x in enumerate(grid) if pm(x) >= pm_deg]
    if not good:
        return np.nan
    i = max(good)
    if i == len(grid) - 1:
        return np.inf
    a, b = grid[i], grid[i + 1]
    for _ in range(30):
        m = np.sqrt(a * b)
        a, b = (m, b) if pm(m) >= pm_deg else (a, m)
    return a


def static_dip_rule(p, Q, Kp):
    """README's quasi-static rule Q / (A Kp), MPa, for outflow Q (m^3/s) and Kp in mm/s per MPa"""
    return Q / p.A / (Kp * 1e-3)


def step_dip(lp, Q, n):
    """peak pressure deviation (Pa, positive = dip) and its time (s) for outflow stepping to Q"""
    y = lp['D'].step(n, Q)
    j = int(np.argmin(y))
    return -y[j], j * lp['Ts'], y


def clamp_report(cfg, Kp, e_mpa):
    """Whether FB_PressureController's output clamp binds for a constant error e (MPa): returns
    (|Kp e| mm/s, VelLimit mm/s, binds). VelLimit = min(VelMax, max(StopVelMin, v_stop))."""
    Dm, St, Ts = cfg['G_cfgPressureDecel'], cfg['G_cfgPressureStopStiffness'], cfg['G_cfgScanTime']
    step = Dm * Ts
    vstop = -step + np.sqrt(step * step + 2 * Dm * abs(e_mpa) / St)
    lim = min(cfg['G_cfgPressureVelMax'], max(cfg['G_cfgPressureStopVelMin'], vstop))
    v = abs(Kp * e_mpa)
    return v, lim, v > lim


def clamp_error(cfg, Kp):
    """smallest error (MPa) at which Kp |e| exceeds the clamp (the proportional term alone)"""
    es = np.logspace(-4, 2, 6000)
    b = [clamp_report(cfg, Kp, e)[2] for e in es]
    return es[b.index(True)] if any(b) else np.inf


def slew_amplitude(cfg, lp, w):
    """error amplitude (MPa) at which a sinusoid at w (rad/s) makes the command's rate w |C| e
    reach the accel limit (mm/s^2); also the step of e (MPa) whose one-scan command jump does"""
    c = np.abs(lp['C'].freq(w)) / KP_UNIT              # mm/s per MPa at w
    return cfg['G_cfgPressureAccel'] / (np.asarray(w) * c)


def slew_step_error(cfg, Kp):
    """error step (MPa) whose one-scan command jump Kp e reaches Accel Ts"""
    return cfg['G_cfgPressureAccel'] * cfg['G_cfgScanTime'] / Kp
