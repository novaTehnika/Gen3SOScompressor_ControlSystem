#!/usr/bin/env python3
"""Linear stability margins of the slave pressure loop across the chamber stiffness range.

Prints four tables (margins versus stiffness, stability boundary and scheduled gain, Bode
and disturbance data, sensor filter and drive lag sweep) and writes PNG and CSV in --out.
--set takes Params fields and G_cfg* names, as in the other studies.

    python3 study_linear.py [--set key=value ...] [--out DIR]
"""

import numpy as np

import linear as lin
import studyutil
from linear import MPA_MM
from studyutil import plt


def marks(p):
    """named stiffnesses, Pa/m"""
    cfg = p.cfg()
    return {'gas-rich': 0.002 * MPA_MM,
            'liquid K=1 GPa': p.A * 1e9 / p.Vl0,
            'liquid K=2.2 GPa': p.A * 2.2e9 / p.Vl0,
            'stop-law': cfg['G_cfgPressureStopStiffness'] * MPA_MM}


def fmt(x, f='.3g'):
    return '  -  ' if x is None or not np.isfinite(x) else format(x, f)


def study_margins(p, out, m):
    print('\n=== 1. margins versus stiffness (GVL gains)')
    cfg = p.cfg()
    print(f'Kp {cfg["G_cfgPressureKp"]} mm/s/MPa, Ki {cfg["G_cfgPressureKi"]} 1/s, Ts {cfg["G_cfgScanTime"]} s, '
          f'filter {cfg["G_cfgAnalogFilterTimeConst"]} s, median {cfg["G_cfgAnalogMedianSize"]}, tauV {p.tauV} s')
    ks = np.unique(np.concatenate([np.logspace(np.log10(0.002), np.log10(200), 61) * MPA_MM, list(m.values())]))
    rows = []
    for k in ks:
        lp = lin.loop(p, k)
        r = lin.margins(lp['L'])
        rows.append((k / MPA_MM, r['gm_db'], r['pm'], r['wc'], r['wpc'], r['dm'] * 1e3, lin.is_stable(lp)))
    rows = np.array(rows, float)
    np.savetxt(out / 'linear_margins.csv', rows, delimiter=',', fmt='%.6g',
               header='k_MPa_per_mm,GM_dB,PM_deg,wc_rad_s,wpc_rad_s,DM_ms,stable', comments='')
    print(f'{"stiffness":>18} {"k MPa/mm":>9} {"GM dB":>7} {"PM deg":>7} {"wc rad/s":>9} {"w180 rad/s":>10} {"DM ms":>9} stable')
    for name, k in m.items():
        lp = lin.loop(p, k)
        r = lin.margins(lp['L'])
        print(f'{name:>18} {k / MPA_MM:9.4g} {fmt(r["gm_db"], ".1f"):>7} {r["pm"]:7.1f} {r["wc"]:9.3g} '
              f'{fmt(r["wpc"]):>10} {r["dm"] * 1e3:9.4g} {lin.is_stable(lp)}')

    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for a, col, lab in zip(ax.flat, (1, 2, 3, 5), ('gain margin (dB)', 'phase margin (deg)', 'crossover (rad/s)', 'delay margin (ms)')):
        y = np.where(np.isfinite(rows[:, col]), rows[:, col], np.nan)
        a.semilogx(rows[:, 0], y, 'k-')
        for (name, k), c in zip(m.items(), studyutil.COLORS):
            a.axvline(k / MPA_MM, color=c, ls='--', lw=1, label=name)
        a.set_ylabel(lab)
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


def study_boundary(p, out, m):
    cfg = p.cfg()
    ratio = cfg['G_cfgPressureKi'] / cfg['G_cfgPressureKp']
    print('\n=== 2. stability boundary and gain schedule (Kp in mm/s per MPa)')
    ks = np.logspace(np.log10(0.002), np.log10(200), 25) * MPA_MM
    rows = []
    for k in ks:
        rows.append([k / MPA_MM,
                     lin.max_stable_kp(p, k), lin.max_stable_kp(p, k, G_cfgPressureKi=0.0),
                     lin.max_stable_kp(p, k, ki_ratio=ratio),
                     lin.kp_for_pm(p, k, 45), lin.kp_for_pm(p, k, 60),
                     lin.kp_for_pm(p, k, 45, ki_ratio=ratio), lin.kp_for_pm(p, k, 60, ki_ratio=ratio)])
    rows = np.array(rows, float)
    np.savetxt(out / 'linear_boundary.csv', rows, delimiter=',', fmt='%.6g', comments='',
               header='k_MPa_per_mm,Kpmax_Ki_GVL,Kpmax_Ki0,Kpmax_Ki_ratio,Kp_PM45_Ki_GVL,Kp_PM60_Ki_GVL,Kp_PM45_Ki_ratio,Kp_PM60_Ki_ratio')

    print(f'Ki = {ratio:g} Kp rows scale Ki with Kp, so L depends on Kp k only: Kp ~ 1/k')
    print(f'{"stiffness":>18} {"k":>8} | {"Kpmax":>8} {"Kpmax":>8} {"Kpmax":>8} | {"Kp PM45":>8} {"Kp PM60":>8} | '
          f'{"Kp PM45":>8} {"Kp PM60":>8}')
    print(f'{"":>18} {"MPa/mm":>8} | {"Ki GVL":>8} {"Ki 0":>8} {"Ki=r Kp":>8} | {"Ki GVL":>8} {"Ki GVL":>8} | '
          f'{"Ki=r Kp":>8} {"Ki=r Kp":>8}')
    sched = {}
    for name, k in m.items():
        v = [lin.max_stable_kp(p, k), lin.max_stable_kp(p, k, G_cfgPressureKi=0.0), lin.max_stable_kp(p, k, ki_ratio=ratio),
             lin.kp_for_pm(p, k, 45), lin.kp_for_pm(p, k, 60),
             lin.kp_for_pm(p, k, 45, ki_ratio=ratio), lin.kp_for_pm(p, k, 60, ki_ratio=ratio)]
        sched[name] = v
        print(f'{name:>18} {k / MPA_MM:8.4g} | {fmt(v[0]):>8} {fmt(v[1]):>8} {fmt(v[2]):>8} | {fmt(v[3]):>8} {fmt(v[4]):>8} | '
              f'{fmt(v[5]):>8} {fmt(v[6]):>8}')
    k0 = m['liquid K=1 GPa']
    print('Kp k product (rad/s, nominal crossover), Ki = r Kp: '
          f'stability limit {sched["liquid K=1 GPa"][2] * k0 / MPA_MM:.3g}, '
          f'45 deg {sched["liquid K=1 GPa"][5] * k0 / MPA_MM:.3g}, 60 deg {sched["liquid K=1 GPa"][6] * k0 / MPA_MM:.3g}')
    print('Kp = wc / k (mm/s per MPa, k in MPa/mm) for a crossover target wc:')
    for wc in (2.0, 5.0, 10.0):
        print(f'  wc {wc:4.1f} rad/s: ' + ', '.join(f'{n} {wc / (k / MPA_MM):.3g}' for n, k in m.items()))

    fig, ax = plt.subplots(figsize=(8, 5.5))
    kk = rows[:, 0]
    for col, lab, st in ((1, 'max stable, Ki GVL', '-'), (2, 'max stable, Ki 0', '--'), (3, 'max stable, Ki = r Kp', ':'),
                         (4, 'PM 45, Ki GVL', '-'), (5, 'PM 60, Ki GVL', '--'), (6, 'PM 45, Ki = r Kp', '-.'), (7, 'PM 60, Ki = r Kp', ':')):
        ax.loglog(kk, np.where(np.isfinite(rows[:, col]) & (rows[:, col] > 0), rows[:, col], np.nan), st, label=lab)
    ax.axhline(cfg['G_cfgPressureKp'], color='k', lw=0.8)
    for (name, k), c in zip(m.items(), studyutil.COLORS):
        ax.axvline(k / MPA_MM, color=c, ls='--', lw=1, label=name)
    ax.set_xlabel('stiffness dP/dx (MPa/mm)')
    ax.set_ylabel('Kp (mm/s per MPa)')
    ax.set_title('stability boundary and phase-margin gains (black: GVL Kp)')
    ax.grid(alpha=0.3, which='both')
    ax.legend(fontsize=7, frameon=False, loc='lower left')
    fig.tight_layout()
    fig.savefig(out / 'linear_boundary.png', dpi=110)
    plt.close(fig)


def study_bode(p, out, m):
    print('\n=== 3. Bode, closed loop and outflow disturbance (GVL gains)')
    cfg = p.cfg()
    Ts, Kp = cfg['G_cfgScanTime'], cfg['G_cfgPressureKp']
    Q = 1e-6
    names = ['liquid K=1 GPa', 'liquid K=2.2 GPa', 'stop-law']
    ks = {'1 MPa/mm': 1.0 * MPA_MM, **{n: m[n] for n in names}}
    w = lin.freq_grid(Ts, 1500, 1e-2)
    fig, ax = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    n = int(8.0 / Ts)
    print(f'{"k":>18} {"k MPa/mm":>9} {"peak |S| dB":>12} {"peak |T| dB":>12} {"|P/Q| peak dB":>14} '
          f'{"dip 1 mL/s MPa":>15} {"at s":>6} {"Q/(A Kp)":>9} {"CL pole |z|":>12}')
    for (name, k), c in zip(ks.items(), studyutil.COLORS):
        lp = lin.loop(p, k)
        L, S, T, D = (lp[x].freq(w) for x in 'LSTD')
        ax[0, 0].semilogx(w, 20 * np.log10(abs(L)), color=c, label=name)
        ax[1, 0].semilogx(w, np.degrees(np.unwrap(np.angle(L))) + 0, color=c)
        ax[0, 1].semilogx(w, 20 * np.log10(abs(S)), color=c, label='S ' + name)
        ax[0, 1].semilogx(w, 20 * np.log10(abs(T)), color=c, ls='--')
        ax[1, 1].semilogx(w, 20 * np.log10(abs(D) * 1e-12), color=c)
        dip, tdip, y = lin.step_dip(lp, Q, n)
        st = lin.is_stable(lp)
        ax[2, 0].plot(np.arange(n) * Ts, y / 1e6, color=c)
        print(f'{name:>18} {k / MPA_MM:9.4g} {20 * np.log10(abs(S).max()):12.2f} {20 * np.log10(abs(T).max()):12.2f} '
              f'{20 * np.log10(abs(D).max() * 1e-12):14.1f} {(f"{dip / 1e6:.3f}" if st else "unstable"):>15} {tdip:6.2f} '
              f'{lin.static_dip_rule(p, Q, Kp):9.3f} {np.abs(lin.poles(lp)).max():12.5f}')
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

    lp = lin.loop(p, m['liquid K=1 GPa'])
    wc = lin.margins(lp['L'])['wc']
    print('\nwhere the nonlinear limits the model ignores bind (GVL gains):')
    print(f'  clamp: Kp |e| exceeds VelLimit = min(VelMax, max(StopVelMin, stop law at {cfg["G_cfgPressureStopStiffness"]:g} MPa/mm)) '
          f'for a constant error above {lin.clamp_error(cfg, Kp):.2f} MPa')
    print(f'  slew:  a step error above {lin.slew_step_error(cfg, Kp):.2f} MPa makes the one-scan command jump reach the accel limit '
          f'({cfg["G_cfgPressureAccel"]:g} mm/s^2)')
    for name in ('liquid K=1 GPa', 'stop-law'):
        lpn = lin.loop(p, m[name])
        wcn = lin.margins(lpn['L'])['wc']
        print(f'         {name}: a sinusoidal error at the crossover ({wcn:.2f} rad/s) above '
              f'{float(lin.slew_amplitude(cfg, lpn, wcn)):.3g} MPa amplitude')
    print(f'  (liquid-only crossover {wc:.2f} rad/s)')


def study_lags(p, out, m):
    print('\n=== 4. max stable Kp (mm/s per MPa, Ki GVL) versus sensor filter and drive lag')
    taus = [0.0, 0.01, 0.02, 0.05, 0.1]
    tvs = [0.005, 0.01, 0.02, 0.04, 0.08]
    for name in ('liquid K=1 GPa', 'liquid K=2.2 GPa'):
        k = m[name]
        print(f'{name} (k {k / MPA_MM:.1f} MPa/mm): rows filter tau (s), columns tauV (s)')
        print(f'{"":>8}' + ''.join(f'{tv:>9g}' for tv in tvs))
        for tf in taus:
            print(f'{tf:8g}' + ''.join(
                f'{lin.max_stable_kp(p.with_(tauV=tv), k, G_cfgAnalogFilterTimeConst=tf):9.3g}' for tv in tvs))
    k = m['liquid K=1 GPa']
    base = lin.max_stable_kp(p, k)
    print(f'\nat the GVL filter and tauV ({p.tauF} s, {p.tauV} s): {base:.3g}; '
          f'filter removed: {lin.max_stable_kp(p, k, G_cfgAnalogFilterTimeConst=0.0):.3g}; '
          f'tauV 1 ms: {lin.max_stable_kp(p.with_(tauV=1e-3), k):.3g}; '
          f'both (only the five scans of delay left): {lin.max_stable_kp(p.with_(tauV=1e-3), k, G_cfgAnalogFilterTimeConst=0.0):.3g}')
    ts = p.Ts
    print(f'lag budget at the liquid-only crossover limit: filter {p.tauF * 1e3:.0f} ms, drive {p.tauV * 1e3:.0f} ms, '
          f'pure delay {lin.loop(p, k)["delay"] * ts * 1e3:.0f} ms ({lin.loop(p, k)["delay"]} scans)')


def main(argv=None):
    ap = studyutil.parser(__doc__.split('\n')[0])
    args = ap.parse_args(argv)
    p, _, out = studyutil.setup(args)
    m = marks(p)
    study_margins(p, out, m)
    study_boundary(p, out, m)
    study_bode(p, out, m)
    study_lags(p, out, m)
    print(f'\nwrote CSV and PNG to {out}')


if __name__ == '__main__':
    main()
