#!/usr/bin/env python3
"""
study_hose_opening.py script

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/29/2026

PURPOSE/DESCRIPTION:
This script studies the opening of the proximal valve at pressure, the
one valve event whose rate the operator can set. The hose behind the
valve holds virgin saline at atmosphere; opening the valve lets
solution through to stress that saline and dilate the hose wall, and
the chamber must supply it. Opened in one scan the chamber falls by the
equalisation drop of eq. (43) before the loop can act. Opened through
a restriction, eq. (45), the chamber sees a flow it can follow, and the
only solution that degasses is what entered while the hose was still
below the saturation pressure; that solution lies right behind the
virgin saline and leaves with it in the purge.

The study runs the delivery scenario for a set of proximal valve
bores, from the 1/4 in ball valve down to needle-valve openings, at a
set of oxygen charges and for the slave as configured and the
scheduled controller, and reports for each the time the hose takes to
fill, the lowest chamber pressure and its margin above saturation, the
solution admitted into the hose, the part of it that degassed and the
purge that carries it out. It then prints the contents of the hose at
the end of the fill for one case, from the distal valve back to the
chamber, as volumes and as lengths along the hose bore. It writes
hose_opening.csv and hose_opening.png to the output directory.

Usage:
    python3 study_hose_opening.py [--set key=value ...] [--out DIR]
                                  [--ratios 1,2,2.5] [--bores 6.35,...]

FILE DEPENDENCY:
./
    studyparams.py, controllers.py
    runner.py
    scenarios.py
    studyutil.py
    (and the modules they import: sim.py, and models/)
"""

import math

import controllers as ctl
import scenarios
import studyutil
from runner import Case, run_cases


# %% ===== STUDY VARIABLES ===============================================
# [mm] Proximal valve bores: the ball valve, then needle-valve openings
BORES_MM = (6.35, 1.0, 0.5, 0.3, 0.2, 0.1)
# [-] Oxygen charges and [MPa] the target each is run at
CHARGES = ((1.0, 10.0), (2.0, 10.0), (2.5, 11.0))
TOPEN_PROX = 0.5            # [s] hand's time on the valve
LUMPS_CASE = (2.0, 0.3)     # (ratio, bore mm) whose hose contents print
# [m^2] hose bore area, 1/4 in, for the lengths along the hose
A_HOSE = math.pi * (6.35e-3)**2 / 4


def controller_set():
    """Return the controllers compared: the slave as configured and the
    scheduled controller at two crossovers with the measured bounds."""
    sched = dict(V_dead_cfg=81e-6, K_cfg=1.0e9, C_hose_cfg=1.2e-13)
    return [
        ctl.SlaveController('slave (GVL)'),
        ctl.ScheduledSlaveController('sched wc11', wc=11.0, **sched),
        ctl.ScheduledSlaveController('sched wc16', wc=16.0, **sched),
    ]


# %% ===== MAIN STUDY ====================================================
def main(argv=None):
    ap = studyutil.parser('Opening the proximal valve at pressure: '
                          'chamber dip and degassed slug against the '
                          'valve bore.')
    ap.add_argument('--ratios', default=None,
                    help='comma list of STP oxygen to saline ratios '
                         '(default 1, 2 at 10 MPa and 2.5 at 11 MPa)')
    ap.add_argument('--bores', default=','.join(map(str, BORES_MM)),
                    help='comma list of proximal valve bores [mm]')
    args = ap.parse_args(argv)
    par, legacy_sensor, out = studyutil.setup(args)
    if args.ratios is None:
        charges = CHARGES
    else:
        charges = [(float(x), par.P_target / 1e6)
                   for x in args.ratios.split(',')]
    bores = [float(x) for x in args.bores.split(',')]
    cs = controller_set()
    width = max(14, max(len(c.name) for c in cs))

    # Collect data: every charge, bore and controller
    cases = []
    for ratio, P_mpa in charges:
        for d_mm in bores:
            q = par.with_(ratio=ratio, P_target=P_mpa * 1e6,
                          d_prox=d_mm * 1e-3, Topen_prox=TOPEN_PROX,
                          hose_before=False)
            for c in cs:
                cases.append(Case(q, scenarios.make('deliver', q), c,
                                  legacy_sensor, args.seed))
    results = run_cases(cases, args.workers)
    n_c = len(cs)

    def result(ci_charge, bi, ci):
        return results[(ci_charge * len(bores) + bi) * n_c + ci]

    # Report
    cols = ('t_fill', 'dipMax', 'P_min', 'degasMargin', 'V_in',
            'V_degas', 'purgeNeeded', 'V_ok')
    heads = ('fill s', 'dip MPa', 'P_min MPa', 'margin MPa', 'V_in mL',
             'degas mL', 'purge mL', 'V_ok mL')
    rows = []
    for k, (ratio, P_mpa) in enumerate(charges):
        print(f'\n=== proximal valve opened at pressure, {ratio:g}:1 at '
              f'{P_mpa:g} MPa, opened over {TOPEN_PROX:g} s')
        print(f'{"bore mm":>8} {"controller":<{width}}'
              + ''.join(f'{h:>11}' for h in heads))
        for bi, d_mm in enumerate(bores):
            for ci, c in enumerate(cs):
                m = result(k, bi, ci).metrics
                print(f'{d_mm:8.2f} {c.name:<{width}}'
                      + ''.join(f'{m[key]:11.2f}' for key in cols))
                rows.append([ratio, P_mpa, d_mm, c.name]
                            + [m[key] for key in cols])

    # Hose contents at the end of the fill for one case, distal end
    # first: what the purge expels, in order
    ratio_l, d_l = LUMPS_CASE
    k = [c[0] for c in charges].index(ratio_l) \
        if ratio_l in [c[0] for c in charges] else 0
    bi = bores.index(d_l) if d_l in bores else 0
    r = result(k, bi, len(cs) - 1)
    print(f'\nhose contents after the fill, {charges[k][0]:g}:1 through '
          f'{bores[bi]:g} mm ({r.name}), from the distal valve back to '
          f'the chamber:')
    print(f'{"lump":<16}{"mL":>8}{"from distal, mm":>18}'
          f'{"degassed":>10}')
    pos = 0.0                                               # [m^3]
    for kind, V, degassed in r.lumps:
        print(f'{kind:<16}{V * 1e6:8.2f}'
              f'{pos / A_HOSE * 1e3:9.0f}-{(pos + V) / A_HOSE * 1e3:<8.0f}'
              f'{"yes" if degassed else "no":>10}')
        pos += V
    print(f'purge needed {r["purgeNeeded"]:.1f} mL; solution delivered '
          f'{r["V_solution"]:.0f} mL')

    # Pressure wave along the hose: the lead a distal transducer could
    # give over the chamber transducer. The hose line's compressibility
    # is the saline's plus the wall's per unit volume.
    beta = 1 / par.K + par.C_hose / par.V_hose          # [1/Pa]
    c = math.sqrt(1 / (par.rho * beta))                 # [m/s]
    L = par.V_hose / A_HOSE                             # [m]
    print(f'pressure wave along the hose: {c:.0f} m/s, {L:.2f} m, '
          f'{L / c * 1e3:.1f} ms from the distal end to the chamber')

    # Save
    with open(out / 'hose_opening.csv', 'w') as f:
        f.write('ratio,P_target_MPa,bore_mm,controller,'
                + ','.join(cols) + '\n')
        for row in rows:
            f.write(f'{row[0]:g},{row[1]:g},{row[2]:g},{row[3]},'
                    + ','.join(f'{x:.5g}' for x in row[4:]) + '\n')
    plot(out, cs, charges, bores, result)
    print(f'\nwrote CSV and PNG to {out}')


# %% ===== PLOTS =========================================================
def plot(out, cs, charges, bores, result):
    """Plot the lowest chamber pressure against the valve bore for each
    controller, one panel per charge, with the saturation pressure
    plus margin as the floor."""
    plt = studyutil.plt
    names = [c.name for c in cs]
    col = studyutil.colors(names)
    fig, axes = plt.subplots(1, len(charges), figsize=(4.2 * len(charges),
                                                       4.5), sharey=True)
    if len(charges) == 1:
        axes = [axes]
    for k, ((ratio, P_mpa), ax) in enumerate(zip(charges, axes)):
        for ci, n in enumerate(names):
            y = [result(k, bi, ci)['P_min'] for bi in range(len(bores))]
            ax.plot(bores, y, marker='o', ls=studyutil.style(n),
                    color=col[n], label=n, lw=1.2, ms=4)
        r = result(k, 0, 0)
        floor = (r.P_sat[0] + 0.5e6) / 1e6             # [MPa]
        ax.axhline(floor, color='k', lw=0.8, ls=':',
                   label='P_sat + 0.5 MPa')
        ax.set_xscale('log')
        ax.set_xlabel('proximal valve bore (mm)')
        ax.set_title(f'{ratio:g}:1 at {P_mpa:g} MPa')
        ax.grid(alpha=0.3, which='both')
    axes[0].set_ylabel('lowest chamber pressure (MPa)')
    axes[0].legend(fontsize=8, frameon=False)
    fig.suptitle('Opening the hose at pressure: chamber floor against '
                 'the valve bore')
    fig.tight_layout()
    fig.savefig(out / 'hose_opening.png', dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    main()
