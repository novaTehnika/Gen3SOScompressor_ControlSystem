#!/usr/bin/env python3
"""
study_linear.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This script uses the linear model in linear.py to find the stability
margins of the slave's pressure loop across the range of chamber
stiffness, from a gas-rich chamber to a liquid-only one. It has four
parts, each of which prints a table:

  1. Gain, phase and delay margins against stiffness at the GVL gains.
  2. The largest stable Kp and the Kp that gives 45 and 60 deg of phase
     margin, against stiffness, with Ki fixed or proportional to Kp.
  3. Open- and closed-loop frequency responses and the pressure dip
     after an outflow step, at three stiffness values. It also reports
     the error amplitudes at which the clamp and the slew limit, which
     the linear model omits, start to act.
  4. The largest stable Kp against the sensor filter time constant and
     the drive velocity lag.

It writes the tables as CSV and the plots as PNG to the output
directory. --set takes StudyParams fields and G_cfg* names, as in the other
studies.

Usage:
    python3 study_linear.py [--set key=value ...] [--out DIR]

FILE DEPENDENCY:
./
    studyparams.py, linear.py
    studyutil.py
"""

import numpy as np

import linear as lin
import studyutil
from linear import MPA_MM
from studyutil import plt


# %% ===== STIFFNESS VALUES OF INTEREST ==================================
def marks(par):
    """Return the named stiffness values [Pa/m] marked on every plot.

      gas-rich          the gas-rich chamber at the start of compress
      liquid K=1 GPa    the liquid-only chamber, k = A K / Vl0, at the
                        nominal bulk modulus
      liquid K=2.2 GPa  the same with the bulk modulus of water
      stop-law          the stiffness the slave's stop law assumes
    """
    cfg = par.cfg()
    return {'gas-rich': 0.002 * MPA_MM,
            'liquid K=1 GPa': par.A * 1e9 / par.Vl0,
            'liquid K=2.2 GPa': par.A * 2.2e9 / par.Vl0,
            'stop-law': cfg['G_cfgPressureStopStiffness'] * MPA_MM}


def fmt(x, f='.3g'):
    """Format a number, or a dash if it is missing or not finite."""
    return '  -  ' if x is None or not np.isfinite(x) else format(x, f)


# %% ===== 1. MARGINS AGAINST STIFFNESS ==================================
def study_margins(par, out, m):
    """Print and plot the margins at the GVL gains against stiffness."""
    print('\n=== 1. margins versus stiffness (GVL gains)')
    cfg = par.cfg()
    print(f'Kp {cfg["G_cfgPressureKp"]} mm/s/MPa, '
          f'Ki {cfg["G_cfgPressureKi"]} 1/s, '
          f'Ts {cfg["G_cfgScanTime"]} s, '
          f'filter {cfg["G_cfgAnalogFilterTimeConst"]} s, '
          f'median {cfg["G_cfgAnalogMedianSize"]}, tauV {par.tauV} s')

    # Stiffness grid from 0.002 to 200 MPa/mm plus the marked values
    ks = np.unique(np.concatenate([
        np.logspace(np.log10(0.002), np.log10(200), 61) * MPA_MM,
        list(m.values())]))                                 # [Pa/m]
    rows = []
    for k in ks:
        lp = lin.loop(par, k)
        r = lin.margins(lp['L'])
        rows.append((k / MPA_MM, r['gm_db'], r['pm'], r['wc'], r['wpc'],
                     r['dm'] * 1e3, lin.is_stable(lp)))
    rows = np.array(rows, float)
    np.savetxt(out / 'linear_margins.csv', rows, delimiter=',',
               fmt='%.6g', comments='',
               header='k_MPa_per_mm,GM_dB,PM_deg,wc_rad_s,wpc_rad_s,'
                      'DM_ms,stable')

    print(f'{"stiffness":>18} {"k MPa/mm":>9} {"GM dB":>7} {"PM deg":>7} '
          f'{"wc rad/s":>9} {"w180 rad/s":>10} {"DM ms":>9} stable')
    for name, k in m.items():
        lp = lin.loop(par, k)
        r = lin.margins(lp['L'])
        print(f'{name:>18} {k / MPA_MM:9.4g} {fmt(r["gm_db"], ".1f"):>7} '
              f'{r["pm"]:7.1f} {r["wc"]:9.3g} {fmt(r["wpc"]):>10} '
              f'{r["dm"] * 1e3:9.4g} {lin.is_stable(lp)}')

    # Plot: one panel per margin, columns of rows as listed below
    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    panels = ((1, 'gain margin (dB)'), (2, 'phase margin (deg)'),
              (3, 'crossover (rad/s)'), (5, 'delay margin (ms)'))
    for a, (col, label) in zip(ax.flat, panels):
        y = np.where(np.isfinite(rows[:, col]), rows[:, col], np.nan)
        a.semilogx(rows[:, 0], y, 'k-')
        for (name, k), c in zip(m.items(), studyutil.COLORS):
            a.axvline(k / MPA_MM, color=c, ls='--', lw=1, label=name)
        a.set_ylabel(label)
        a.grid(alpha=0.3, which='both')
    ax[0, 0].axhline(0, color='r', lw=0.8)
    ax[1, 0].set_yscale('log')
    ax[1, 1].set_yscale('log')
    ax[0, 1].axhline(45, color='gray', lw=0.8)
    ax[1, 0].set_xlabel('stiffness dP/dx (MPa/mm)')
    ax[1, 1].set_xlabel('stiffness dP/dx (MPa/mm)')
    ax[0, 0].legend(fontsize=8, frameon=False)
    ax[0, 0].set_title('margins at the GVL gains')
    fig.tight_layout()
    fig.savefig(out / 'linear_margins.png', dpi=110)
    plt.close(fig)


# %% ===== 2. STABILITY BOUNDARY AND GAIN SCHEDULE =======================
def boundary_row(par, k, ratio):
    """Return the gain limits [(mm/s)/MPa] at stiffness k [Pa/m]: the
    largest stable Kp with Ki at the GVL value, Ki = 0 and Ki = ratio Kp,
    then the Kp for 45 and 60 deg phase margin with Ki at the GVL value
    and with Ki = ratio Kp."""
    return [lin.max_stable_kp(par, k),
            lin.max_stable_kp(par, k, G_cfgPressureKi=0.0),
            lin.max_stable_kp(par, k, ki_ratio=ratio),
            lin.kp_for_pm(par, k, 45), lin.kp_for_pm(par, k, 60),
            lin.kp_for_pm(par, k, 45, ki_ratio=ratio),
            lin.kp_for_pm(par, k, 60, ki_ratio=ratio)]


def study_boundary(par, out, m):
    """Print and plot the largest stable Kp and the phase-margin gains
    against stiffness."""
    cfg = par.cfg()
    ratio = cfg['G_cfgPressureKi'] / cfg['G_cfgPressureKp']     # [1/s]
    print('\n=== 2. stability boundary and gain schedule '
          '(Kp in mm/s per MPa)')
    ks = np.logspace(np.log10(0.002), np.log10(200), 25) * MPA_MM
    rows = np.array([[k / MPA_MM] + boundary_row(par, k, ratio)
                     for k in ks], float)
    np.savetxt(out / 'linear_boundary.csv', rows, delimiter=',',
               fmt='%.6g', comments='',
               header='k_MPa_per_mm,Kpmax_Ki_GVL,Kpmax_Ki0,Kpmax_Ki_ratio,'
                      'Kp_PM45_Ki_GVL,Kp_PM60_Ki_GVL,Kp_PM45_Ki_ratio,'
                      'Kp_PM60_Ki_ratio')

    print(f'Ki = {ratio:g} Kp rows scale Ki with Kp, so L depends on '
          f'Kp k only: Kp ~ 1/k')
    print(f'{"stiffness":>18} {"k":>8} | {"Kpmax":>8} {"Kpmax":>8} '
          f'{"Kpmax":>8} | {"Kp PM45":>8} {"Kp PM60":>8} | '
          f'{"Kp PM45":>8} {"Kp PM60":>8}')
    print(f'{"":>18} {"MPa/mm":>8} | {"Ki GVL":>8} {"Ki 0":>8} '
          f'{"Ki=r Kp":>8} | {"Ki GVL":>8} {"Ki GVL":>8} | '
          f'{"Ki=r Kp":>8} {"Ki=r Kp":>8}')
    sched = {}
    for name, k in m.items():
        v = boundary_row(par, k, ratio)
        sched[name] = v
        print(f'{name:>18} {k / MPA_MM:8.4g} | {fmt(v[0]):>8} '
              f'{fmt(v[1]):>8} {fmt(v[2]):>8} | {fmt(v[3]):>8} '
              f'{fmt(v[4]):>8} | {fmt(v[5]):>8} {fmt(v[6]):>8}')

    # With Ki = ratio Kp the loop depends only on Kp k, which
    # approximates the gain crossover [rad/s] (Kp in (mm/s)/MPa, k in
    # MPa/mm). A stiffness-scheduled gain is therefore Kp = wc / k.
    k0 = m['liquid K=1 GPa'] / MPA_MM                       # [MPa/mm]
    s0 = sched['liquid K=1 GPa']
    print(f'Kp k product (rad/s, nominal crossover), Ki = r Kp: '
          f'stability limit {s0[2] * k0:.3g}, 45 deg {s0[5] * k0:.3g}, '
          f'60 deg {s0[6] * k0:.3g}')
    print('Kp = wc / k (mm/s per MPa, k in MPa/mm) for a crossover '
          'target wc:')
    for wc in (2.0, 5.0, 10.0):
        print(f'  wc {wc:4.1f} rad/s: '
              + ', '.join(f'{n} {wc / (k / MPA_MM):.3g}'
                          for n, k in m.items()))

    # Plot the gain limits against stiffness
    fig, ax = plt.subplots(figsize=(8, 5.5))
    kk = rows[:, 0]
    curves = ((1, 'max stable, Ki GVL', '-'),
              (2, 'max stable, Ki 0', '--'),
              (3, 'max stable, Ki = r Kp', ':'),
              (4, 'PM 45, Ki GVL', '-'), (5, 'PM 60, Ki GVL', '--'),
              (6, 'PM 45, Ki = r Kp', '-.'), (7, 'PM 60, Ki = r Kp', ':'))
    for col, label, ls in curves:
        y = rows[:, col]
        ax.loglog(kk, np.where(np.isfinite(y) & (y > 0), y, np.nan), ls,
                  label=label)
    ax.axhline(cfg['G_cfgPressureKp'], color='k', lw=0.8)
    for (name, k), c in zip(m.items(), studyutil.COLORS):
        ax.axvline(k / MPA_MM, color=c, ls='--', lw=1, label=name)
    ax.set_xlabel('stiffness dP/dx (MPa/mm)')
    ax.set_ylabel('Kp (mm/s per MPa)')
    ax.set_title('stability boundary and phase-margin gains '
                 '(black: GVL Kp)')
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=7, frameon=False, loc='lower left')
    fig.tight_layout()
    fig.savefig(out / 'linear_boundary.png', dpi=110)
    plt.close(fig)


# %% ===== 3. FREQUENCY RESPONSE AND OUTFLOW DISTURBANCE =================
def study_bode(par, out, m):
    """Print and plot the frequency responses and the outflow-step dip,
    then report where the clamp and slew limit start to act."""
    print('\n=== 3. Bode, closed loop and outflow disturbance '
          '(GVL gains)')
    cfg = par.cfg()
    Ts, Kp = cfg['G_cfgScanTime'], cfg['G_cfgPressureKp']
    Q = 1e-6                                    # [m^3/s] outflow step
    names = ['liquid K=1 GPa', 'liquid K=2.2 GPa', 'stop-law']
    ks = {'1 MPa/mm': 1.0 * MPA_MM, **{n: m[n] for n in names}}
    w = lin.freq_grid(Ts, 1500, 1e-2)                       # [rad/s]
    n = int(8.0 / Ts)                           # [-] 8 s of samples
    fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex=True)

    print(f'{"k":>18} {"k MPa/mm":>9} {"peak |S| dB":>12} '
          f'{"peak |T| dB":>12} {"|P/Q| peak dB":>14} '
          f'{"dip 1 mL/s MPa":>15} {"at s":>6} {"Q/(A Kp)":>9} '
          f'{"CL pole |z|":>12}')
    for (name, k), c in zip(ks.items(), studyutil.COLORS):
        lp = lin.loop(par, k)
        L, S, T, D = (lp[x].freq(w) for x in 'LSTD')
        # |P/Q| in dB relative to 1 MPa per mL/s:
        # [Pa/(m^3/s) * 1e-6 (MPa/Pa) * 1e-6 (m^3/s per mL/s)]
        D_db = 20 * np.log10(abs(D) * 1e-12)
        ax[0, 0].semilogx(w, 20 * np.log10(abs(L)), color=c, label=name)
        ax[1, 0].semilogx(w, np.degrees(np.unwrap(np.angle(L))), color=c)
        ax[0, 1].semilogx(w, 20 * np.log10(abs(S)), color=c,
                          label='S ' + name)
        ax[0, 1].semilogx(w, 20 * np.log10(abs(T)), color=c, ls='--')
        ax[1, 1].semilogx(w, D_db, color=c)
        dip, t_dip, y = lin.step_dip(lp, Q, n)
        stable = lin.is_stable(lp)
        ax[2, 0].plot(np.arange(n) * Ts, y / 1e6, color=c)
        dip_text = f'{dip / 1e6:.3f}' if stable else 'unstable'
        print(f'{name:>18} {k / MPA_MM:9.4g} '
              f'{20 * np.log10(abs(S).max()):12.2f} '
              f'{20 * np.log10(abs(T).max()):12.2f} {D_db.max():14.1f} '
              f'{dip_text:>15} {t_dip:6.2f} '
              f'{lin.static_dip_rule(par, Q, Kp):9.3f} '
              f'{np.abs(lin.poles(lp)).max():12.5f}')
    ax[0, 0].set_ylabel('|L| (dB)')
    ax[1, 0].set_ylabel('phase L (deg)')
    ax[0, 1].set_ylabel('|S| solid, |T| dashed (dB)')
    ax[1, 1].set_ylabel('|P/Q| (dB re 1 MPa per mL/s)')
    ax[2, 0].set_ylabel('P for 1 mL/s outflow step (MPa)')
    ax[2, 0].set_xlabel('t (s)')
    ax[2, 1].axis('off')
    ax[1, 0].set_xlabel('rad/s')
    ax[1, 1].set_xlabel('rad/s')
    ax[0, 0].legend(fontsize=8, frameon=False)
    for a in ax.flat:
        a.grid(alpha=0.3, which='both')
    ax[0, 0].axhline(0, color='k', lw=0.6)
    fig.tight_layout()
    fig.savefig(out / 'linear_bode.png', dpi=110)
    plt.close(fig)

    # Range of validity: the error amplitudes at which the clamp and the
    # slew limit, which the linear model omits, start to act
    lp = lin.loop(par, m['liquid K=1 GPa'])
    wc = lin.margins(lp['L'])['wc']
    print('\nwhere the nonlinear limits the model ignores bind '
          '(GVL gains):')
    print(f'  clamp: Kp |e| exceeds VelLimit = min(VelMax, '
          f'max(StopVelMin, stop law at '
          f'{cfg["G_cfgPressureStopStiffness"]:g} MPa/mm)) for a '
          f'constant error above {lin.clamp_error(cfg, Kp):.2f} MPa')
    print(f'  slew:  a step error above '
          f'{lin.slew_step_error(cfg, Kp):.2f} MPa makes the one-scan '
          f'command jump reach the accel limit '
          f'({cfg["G_cfgPressureAccel"]:g} mm/s^2)')
    for name in ('liquid K=1 GPa', 'stop-law'):
        lpn = lin.loop(par, m[name])
        wcn = lin.margins(lpn['L'])['wc']
        amp = float(lin.slew_amplitude(cfg, lpn, wcn))
        print(f'         {name}: a sinusoidal error at the crossover '
              f'({wcn:.2f} rad/s) above {amp:.3g} MPa amplitude')
    print(f'  (liquid-only crossover {wc:.2f} rad/s)')


# %% ===== 4. SENSOR FILTER AND DRIVE LAG ================================
def study_lags(par, out, m):
    """Print the largest stable Kp against the sensor filter time
    constant and the drive velocity lag."""
    print('\n=== 4. max stable Kp (mm/s per MPa, Ki GVL) versus sensor '
          'filter and drive lag')
    taus = [0.0, 0.01, 0.02, 0.05, 0.1]         # [s] filter
    tvs = [0.005, 0.01, 0.02, 0.04, 0.08]       # [s] drive lag
    for name in ('liquid K=1 GPa', 'liquid K=2.2 GPa'):
        k = m[name]
        print(f'{name} (k {k / MPA_MM:.1f} MPa/mm): rows filter tau (s), '
              f'columns tauV (s)')
        print(f'{"":>8}' + ''.join(f'{tv:>9g}' for tv in tvs))
        for tf in taus:
            kps = [lin.max_stable_kp(par.with_(tauV=tv), k,
                                     G_cfgAnalogFilterTimeConst=tf)
                   for tv in tvs]
            print(f'{tf:8g}' + ''.join(f'{x:9.3g}' for x in kps))

    # Contribution of each lag at the nominal liquid-only stiffness
    k = m['liquid K=1 GPa']
    base = lin.max_stable_kp(par, k)
    no_filter = lin.max_stable_kp(par, k, G_cfgAnalogFilterTimeConst=0.0)
    fast_drive = lin.max_stable_kp(par.with_(tauV=1e-3), k)
    neither = lin.max_stable_kp(par.with_(tauV=1e-3), k,
                                G_cfgAnalogFilterTimeConst=0.0)
    print(f'\nat the GVL filter and tauV ({par.tauF} s, {par.tauV} s): '
          f'{base:.3g}; filter removed: {no_filter:.3g}; '
          f'tauV 1 ms: {fast_drive:.3g}; both (only the five scans of '
          f'delay left): {neither:.3g}')
    delay = lin.loop(par, k)['delay']                       # [scans]
    print(f'lag budget at the liquid-only crossover limit: filter '
          f'{par.tauF * 1e3:.0f} ms, drive {par.tauV * 1e3:.0f} ms, '
          f'pure delay {delay * par.Ts * 1e3:.0f} ms ({delay} scans)')


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    ap = studyutil.parser('Linear stability margins of the slave '
                          'pressure loop across the chamber stiffness '
                          'range.')
    args = ap.parse_args(argv)
    par, _, out = studyutil.setup(args)
    m = marks(par)
    study_margins(par, out, m)
    study_boundary(par, out, m)
    study_bode(par, out, m)
    study_lags(par, out, m)
    print(f'\nwrote CSV and PNG to {out}')


if __name__ == '__main__':
    main()
