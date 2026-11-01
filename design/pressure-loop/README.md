# Pressure loop design study

This directory holds the studies run on a Python model of the slave's
pressure loop; the model's plant and sensor chain are in `models/`. The studies were used to choose between closing the
loop through the drive's velocity loop or its torque loop, to choose the
controller structure and gains, and to find how much oxygen the machine
can dissolve and deliver without degassing the solution.

The slave's own pressure blocks run inside the model. The sensor chain is
the slave's `FB_AnalogInputFilter` and `FB_PressureInput`, and the slave
controller is its `FB_PressureController`. All three are translated from
the Structured Text source, wired as `PRG_Main` wires them, and configured
from the slave's global variable list (GVL). Nothing here runs on the
controllers. The slave implementation is in `slave-src/` and its
closed-loop tests are in `tests/`.

## Running

```sh
pip install -r design/requirements.txt           # numpy, scipy, matplotlib
python3 design/pressure-loop/test_model.py       # model checks, a few seconds
python3 design/pressure-loop/study_compare.py    # three scenarios, seven controllers, a few seconds
python3 design/pressure-loop/study_delivery.py   # deliverable volume against oxygen charge
python3 design/pressure-loop/study_linear.py     # margins against chamber stiffness, about 30 s
python3 design/pressure-loop/study_robust.py     # Monte Carlo over the uncertain parameters, about 5 min on 4 CPUs
python3 design/pressure-loop/study_convergence.py # integration error against the time step, about 1 min
python3 design/pressure-loop/test_linear.py      # linear model against the nonlinear simulation
python3 design/pressure-loop/test_robust.py      # sampler, criteria and parallel runner
```

The scripts run from the repository root or from this directory. They
accept these options:

| Option | Effect |
|---|---|
| `--sweeps` (compare only) | Also runs the gain sweeps of the legacy and slave controllers. |
| `--set key=value` | Overrides a `StudyParams` field (a model parameter or a study setting) (`K=2.2e9`, `solver=heun`) or a slave setting by its GVL name (`G_cfgPressureKp=1`). The option can be repeated. |
| `--out DIR` | Sets where the CSV traces and PNG plots go. The default is `out/`, which git ignores. |
| `--legacy` | Uses the earlier plant and sensor: a fixed liquid volume, a single slew limit, and the median and first-order filter applied directly to pressure. |
| `--workers N`, `--seed N` | Set the number of processes (default one per CPU) and the sensor noise seed. |

The plant, the sensor chain and their parameters are in `models/`
(`models/README.md`), with the slave blocks translated into
`models/generated/` by `tools/st_translate.py --target py`;
`models/slave.py` regenerates them when they are missing or older than
`slave-src/FB/`.

## Files

| File | Role |
|---|---|
| `study_convergence.py` | Measures the integration error against the time step for both methods. |
| `studyparams.py` | Defines `StudyParams`: the models' `Params` (`models/params.py`) and the studies' own settings, the valve timings and procedure, the legacy controller's filter, the target and the requirement limits. A study overrides a model default by redeclaring the field. |
| `modelpath.py` | Puts `models/` on the import path for the studies. |
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound), `ScheduledSlaveController` (the same block with its gains and stop law scheduled on an estimate of the chamber stiffness, equations (39) to (41)) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). All share the `reset` and `step` interface. |
| `scenarios.py` | Defines the operating cases `compress`, `deliver`, `cycle` and the `stiffstep` probe, with the hose procedure and the valve timings (proximal opening, purge, pause, delivery). |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces, the valve event times and metrics. It sequences the valves: the hose opening, the purge, its closing and the reopening. |
| `metrics.py` | Computes the loop metrics (overshoot, settling time, ripple, stroke) and the process metrics (pressure ceiling, saturation margin, the deepest dip at a valve event, volume delivered before a violation, solution delivered). |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `study_compare.py` | Compares velocity and torque command and the slave controller over three scenarios. |
| `study_hose_opening.py` | Studies the proximal valve opened at pressure: the chamber floor, the solution admitted into the hose, the part of it that degassed and where it lies, against the valve bore (ball valve to needle valve), per charge and controller. |
| `study_delivery.py` | Runs the delivery to the empty chamber over a range of oxygen charges and reports the deliverable volume. |
| `linear.py`, `study_linear.py` | Build the discrete-time linear model of the velocity-command loop, equations (26) to (33), and use it to find the margins, the stability boundary and the frequency and disturbance responses. |
| `robust.py`, `study_robust.py` | Define the uncertain parameters, requirement limits and sampler, and run the tornado, corner and Monte Carlo studies for each controller configuration. |
| `studyutil.py` | Holds command-line, CSV and plotting helpers for the studies. |
| `test_model.py`, `test_linear.py`, `test_robust.py` | Check the model, the linear model and the robustness helpers. `tests/run_all.sh` runs them. |

## Reading the code

Each file opens with a header that gives its purpose, the model or
method it implements, and the files it depends on. Sections are marked
with `# %% ===== NAME =====` lines, which editors such as VS Code and
Spyder treat as cells, and subsections inside a class with
`# ----- Name -----`.

Quantities are in SI units inside the model. A comment in square
brackets gives the unit of a value, for example `# [Pa]`, and a
conversion is written `# [mm/s -> m/s]`. The slave works in MPa, mm and
mm/s, so values are converted where they pass to or from its blocks.

A comment such as `eq. (5)` refers to the numbered equations, here and in
`models/README.md`. The
symbols in the equations match the variable names in the code.

## Controllers and scenarios

The plant and the sensor chain the studies run on are the machine's
models, in `models/`; `models/README.md` describes them, gives their
equations (1) to (18), (34) to (38) and (42) to (45) and the measured
values of their parameters. This section and the next cover what the
studies add: the controllers, the scenarios and the linear model.

**Slave controller.** The model runs `FB_PressureController` with the
target clamped to `G_cfgPressureTargetMax`, the integrator reset on the
scan that enters the mode, the output clamped to `G_cfgPressureVelMax`,
and the accel and decel limits from the GVL. The block is a PI
controller whose output is limited by a stop law that uses a configured
stiffness and a minimum velocity. It has no derivative or feedforward
term. `SlaveController(flow_ff=True)` adds the expected flow divided by
A to the velocity after the block, which the slave does not do.

**Scenarios.** `compress` starts at the supply pressure with the charge
as free gas and drives to the 10 MPa target. `deliver` starts at the
target with the charge dissolved as far as saturation allows, opens the
valve at 1 s and runs until the saline is gone (about 80 s). `cycle`
does both in one run with a 60 s hold. `stiffstep` (a liquid-only
10 → 12 MPa step) is kept as a stiff-regime margin probe; it is not a
process case.

**Scheduled slave controller.** The candidate for the slave. It runs
`FB_PressureController` with Kp, Ki and the stop law set every scan from
an upper bound on the chamber stiffness computed from what the slave
has: the measured pressure, the piston position against the limit
switch, and the configured dead volume, saline volume, charge and bulk
modulus, eq. (39). The free gas is taken as the least there can be
(the charge less what the saline could hold at the highest pressure
seen), the saline volume as the configured value or the chamber volume
when that is smaller, and the bulk modulus as the largest plausible, so
the bound never falls below the true stiffness while the configured
saline volume is at or below the true one. The gain is Kp = wc/k_est for
a chosen crossover wc, eq. (40), with Ki in a fixed ratio to Kp, so the
loop's crossover is at most wc whatever the chamber is doing: soft on
the approach, stiffening as the gas dissolves, and stiffening again as
the chamber empties. The stop law, eq. (41), brakes for the estimated
stiffness and for the measurement lag, through the block's VelMax
input. The integrator is held at zero until the pressure first comes
within 0.5 MPa of the target, because the low scheduled gain would
otherwise let it wind up on the approach. The block multiplies its
stored integral by Ki, so the integral is rescaled when Ki changes; the
Structured Text port must do the same.

**Legacy controller.** This is a PID controller with a filtered
derivative, an output clamp and conditional anti-windup: the integrator
holds while the output is saturated and the error pushes it further into
saturation. Two options are available. A stop law limits the velocity
command to the speed from which the slew-limited actuator can still stop
within the pressure error, using the stiffness dP/dx estimated over the
last ten scans. A velocity feedforward adds the expected valve flow
divided by A. In torque mode the controller adds A times the target as a
feedforward force. The legacy controller exists to explore structures the
slave does not have.

## Controller and linear-model equations

The numbering continues that of `models/README.md`. P is the chamber
gauge pressure [Pa], x the piston position [m] (positive into the
cylinder) and v the piston velocity [m/s]; the other symbols are the
`StudyParams` fields of the same name.

### Slave controller

Stop-law clamp of `FB_PressureController`, with D the decel limit
[mm/s^2], S_stop the configured stop-law stiffness [MPa/mm] and e the
pressure error [MPa]:

    (19) v_stop = -D Ts + sqrt((D Ts)^2 + 2 D |e| / S_stop)
         v_lim  = min(VelMax, max(StopVelMin, v_stop))

Optional flow feedforward, added after the block:

    (20) v_ff = Q_ff / A

### Legacy controller

Filtered derivative of the measurement, with filter time constant Tf:

    (21) d[n] = d[n-1] + Ts/(Tf + Ts) ((P_meas[n] - P_meas[n-1])/Ts - d[n-1])

Control law before the limit, with e = target - P_meas and I the error
integral:

    (22) u_raw = u_ff + Kp e + Ki I - Kd d

Stiffness estimate from the pressure and position changes over the last
ten scans, updated only when the piston has moved more than 5 um and held
within 0.01 to 100 MPa/mm:

    (23) k_est = |sum(dP)| / |sum(dx)|

Stop-law velocity, the speed from which the actuator decelerating at a
stops within the error:

    (24) v_stop = -a Ts + sqrt((a Ts)^2 + 2 a |e| / k_est)

Conditional anti-windup. With u the limited command:

    (25) I <- I + e Ts unless u != u_raw and e drives u_raw further
         from u

### Scheduled slave controller

Stiffness bound from the measured pressure P and piston position x,
with V_ch = A (x_limit − x) + V_dead_cfg the chamber volume (x_limit the
limit-switch position and V_dead_cfg the configured dead volume, a lower
bound), V_l_cfg the configured saline volume (a lower bound), n_cfg the
configured charge, K_cfg the largest plausible bulk modulus, C_hose_cfg
the smallest plausible hose compliance (0 while the hose is shut) and
P_ref the highest absolute pressure seen in the run:

    (39) V_gas_est = max(n_cfg - kH P_ref V_l_cfg, 0) R T / P_abs
         C_est = V_gas_est / P_abs + min(V_l_cfg, V_ch) / K_cfg
                 + C_hose_cfg
         k_est = A / C_est

Scheduled gains for a target crossover wc, bounded, with Ki in a fixed
ratio to Kp:

    (40) Kp = clamp(wc / k_est, Kp_min, Kp_max),   Ki = r_i Kp

Stop law with the measurement lag: the piston travels for the lag at its
speed and then decelerates at D, and must stop within the error at the
estimated stiffness, v lag + v^2 / (2 D) <= |e| / k_est, so

    (41) v_stop = -D lag + sqrt((D lag)^2 + 2 D |e| / k_est)
         v_lim = min(VelMax, max(StopVelMin, v_stop))

The block's own stop law, eq. (19), is this with lag = Ts and a fixed
stiffness.

### Linear model

Plant, the zero-order-hold discretization of k / (s (tauV s + 1)), with
a_v = exp(-Ts/tauV) and t_v = tauV (1 - a_v):

    (26) Gd(z) = k ((Ts - t_v) z + (t_v - a_v Ts)) / ((z - 1)(z - a_v))

PI controller:

    (27) C(z) = Kp + Ki Ts / (z - 1)

Sensor filter:

    (28) F(z) = alpha z / (z - (1 - alpha))

Open loop, with d_s = (N - 1)/2 + 2 scans of delay in the sensor path
(four for N = 5), one scan for the command and n_dead = t_dead / Ts
scans of drive dead time (six):

    (29) L(z) = C(z) z^-(1 + n_dead) Gd(z) z^-d_s F(z)

Sensitivity and complementary sensitivity:

    (30) S = 1 / (1 + L),   T = L / (1 + L)

Target to pressure. The target enters at the controller, so the sensor
path does not act on it:

    (31) R = C z^-(1 + n_dead) Gd S

Outflow to pressure. The outflow changes the chamber volume directly, so
it enters at the integrator, after the drive lag:

    (32) D = -(k Ts / A) / (z - 1) S

Quasi-static dip. In steady state the proportional term must command
the velocity Q/A, so the error is

    (33) dip = Q / (A Kp)


## Linear analysis

`linear.py` models the slave's velocity-command loop at the scan time Ts
to find its stability margins across the chamber stiffness k = dP/dx.
The open loop is eq. (29):

- `C(z)` is the PI law of `FB_PressureController`, eq. (27).
- `z^-(1+n_dead)` is the scan between computing a command and applying
  it, plus the drive's six scans of dead time.
- `Gd(z)` is the drive velocity loop and the chamber integrator,
  eq. (26).
- `z^-4` is the median of 5 (two scans for a slowly varying signal) and
  the `PRG_Main` read order (two scans).
- `F(z)` is the first-order filter, eq. (28).

The outflow enters at the chamber integrator, after the drive lag,
eq. (32). The model omits the clamp, the stop law, the slew limits, the
nonlinearity of the median filter and gas dissolution. With Ki
proportional to Kp the loop depends on Kp·k only, so a
stiffness-scheduled gain is Kp = wc/k for a chosen crossover wc.
`test_linear.py` compares the model with the nonlinear simulation: the
step response, the stability boundary and oscillation frequency, and the
outflow dip.

## Robustness study

`study_robust.py` checks a slave configuration against the machine's
requirements over the plausible ranges of the uncertain parameters, at a
given oxygen charge (`--ratio`, default 1:1), and shows which parameters
drive the failures. `robust.py` holds the parameter table (`UNCERTAIN`),
the limits (`THRESH`), the requirements and the sampler.

| Parameter | Range | Sampling | Basis |
|---|---|---|---|
| K | 0.6–1.0 GPa | log | measured 0.72 (segments 0.67–0.78, gas-corrected 0.81) |
| Vl0 | 320–360 mL | linear | fill at the 280 mm start, 343 mL from the geometry |
| passages | 52–62 mL | linear | measured 57 mL with the hose shut |
| hose compliance | 0.12–0.36 mL/MPa | log | 0.24 mL/MPa by difference of two bulk-modulus runs |
| tauDiss | 10–100 s | log | not measured |
| tauV | 10–50 ms | log | 18.2 ms at the motor; the piston side is not resolved |
| sensor noise | 1.5–5 kPa rms | log | measured 1.8 (with capacitor) to 4.6 kPa |
| orifice Cd | 0.8–0.9 | linear | measured 0.84–0.87 |
| supply pressure | 0.52–1.38 MPa | linear | 75–200 psi with the cylinder's charge |

The ranges are the bench measurements of 2026-10-26 with their spread
(`bench/characterisation/README.md` section 8, `bench/friction`,
`bench/pressure`). `mr`, `b`, `Fc` and `mu` act only in torque mode and
the stall, so they are not varied; the dead time is fixed at its
measured 22.8 ms. Each
sample runs compress and deliver and checks these requirements:

- The pressure never exceeds the ceiling, 12 MPa (`StudyParams.P_ceiling`).
  The machine's mechanical limit is to be confirmed; 15 MPa may break
  the piston.
- The pressure never falls below the saturation pressure plus the
  margin, 0.5 MPa (`StudyParams.degas_margin`), from the moment the target is
  reached until the delivery ends. This is the no-degassing requirement,
  and the margin sets where the trade-off between charge and deliverable
  volume is read.
- Control is kept to the end of the delivery: once the opening dip has
  had 5 s to recover, the error never exceeds 1 MPa (`StudyParams.e_crash`),
  so the volume delivered before a violation, `V_ok`, equals the volume
  delivered.
- Compression ends within the status band (`G_cfgPressureInBand`).
- The stroke is at most 105 mm, which is `G_cfgPosSoftLimitMax` minus the
  chamber contact position of the slave simulator.

Overshoot, settling time and ripple are reported but not judged; only
the ceiling limits the overshoot.

The study runs a one-at-a-time sensitivity (tornado), then all corners of
the four most influential parameters, then a Latin hypercube sample
(`--n`, default 200) for each configuration. The default configurations
are the GVL (the 20 ms filter and StopVelMin 2 mm/s, which passes the
1.4 mm/s the measured outflow needs); the earlier StopVelMin of 0.5 mm/s
as a reference; the scheduled controller at wc = 6, 11 and 16 rad/s with
the measured bounds (K_cfg 1.0 GPa, dead volume 81 mL, hose compliance
0.12 mL/MPa); and wc = 11 with VelMax 7 mm/s. `--hose-before` runs the
proposed procedure (the hose opened before the compression). Every
configuration sees the same samples and noise seeds.
`--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine cannot
supply the flow, so no built-in configuration uses it.

## Results with the measured parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa·s (Ki). "GVL" is the
slave's `FB_PressureController` as now configured: Kp 0.5, Ki 0.2,
VelMax 5 mm/s, accel and decel 50 mm/s² (which the drive does not
apply), stop-law stiffness 100 MPa/mm, StopVelMin 2 mm/s and the 20 ms
filter. StopVelMin was 0.5 mm/s until this study; that setting appears
below as "StopVelMin 0.5". The time-domain results use
Heun's method at five steps per scan and the measured plant: K 0.72 GPa,
343 mL of saline with 57 mL of passages and a 29 mL hose, Cd 0.85, a
22.8 ms dead time
and 18.2 ms lag in the drive, and a −0.11 MPa transducer offset, which
puts the true pressure 0.11 MPa above every target. That offset is the
slave's reading at zero with the 434 Ω setting; with the input
calibrated it is zero (the `Params` default), and
`--set P_offset=-0.11e6` reproduces the results below. The 2.9 % gain
error of the uncalibrated reading is not in them.

**Velocity versus torque command.** Velocity command wins, now more
clearly. The drive's own velocity loop absorbs the 6 kN of Coulomb and
2.3 MN s/m of viscous friction and the mass-spring resonance, so the
pressure loop sees an integrator behind a short lag. In torque command
the measured friction is a dead band of 6 MPa and more at pressure, so a
force command cannot place the pressure at all. Torque remains only as
the stall cap, about 20 MPa at 100 % rated torque.

**Compression** (1:1 charge from a 1 MPa supply, hose open): the piston
sweeps 12 mm, and the GVL loop arrives in band after 4.8 s with
0.73 MPa of overshoot; the scheduled loop at wc = 11 rad/s arrives in
2.4 s with 0.11 MPa, which is the transducer offset, not an overshoot
of the reading. The chamber stiffness rises from 0.1 to 4.2 MPa/mm
(6.5 with the hose shut). The legacy controllers without the configured
stop law overshoot to 13–20 MPa; the stop law is what makes the
approach safe.

**Opening the hose at pressure is the first limit.** In the present
procedure the proximal valve is opened after mixing, with the chamber
at 10 MPa and the hose at atmosphere. The stored compression of 343 mL
of saline must then stress 29 mL more and dilate the hose wall, and
with the ball valve the pressure falls from 10 to 6.4 MPa inside one
scan, eq. (43). It is the same for every controller, because the loop
can only recover it afterwards (the piston must advance 0.5 mm, which
takes the scheduled loop about a second and the GVL loop longer). A
1:1 charge keeps a 2.8 MPa margin through it; a 1.5:1 charge keeps
1.0 MPa; a 2:1 charge degasses (−0.8 MPa). Two remedies exist. One is
procedural: open the proximal valve before the compression, so that
the hose's saline is compressed with the charge (`StudyParams.hose_before`).
The other is to open it slowly enough that the chamber keeps up, which
the next paragraph quantifies. Every result below is for the first
remedy unless marked.

**Opening the hose through a needle valve.** The opening time of the
ball valve hardly matters: 1 % of its bore passes tens of mL per
second at 10 MPa, so the hose fills in the first moments of any ramp
and the chamber falls almost as if the valve were snapped open. What
matters is the smallest area the operator can hold, which is a needle
valve's. `study_hose_opening.py` opens the valve over 0.5 s to a set
of bores, at pressure, with the slave then purging what the fill
degassed. The lowest chamber pressure during the fill, in MPa, with
the time the fill takes:

| Bore | Fill | 2:1 at 10 MPa: GVL / wc 11 / wc 16 | 2.5:1 at 11 MPa: GVL / wc 11 / wc 16 |
|---|---|---|---|
| 6.35 mm (ball valve) | 0.5–0.9 s | 6.4 / 6.4 / 6.4 | 7.0 / 7.0 / 7.0 |
| 1.0 mm | 0.5–1.0 s | 6.7 / 7.3 / 7.4 | 7.4 / 8.0 / 8.1 |
| 0.5 mm | 0.7–1.2 s | 7.5 / 8.6 / 9.0 | 8.3 / 9.5 / 9.8 |
| 0.3 mm | 1.3–1.8 s | 8.0 / 9.1 / 9.2 | 8.9 / 10.0 / 10.2 |
| 0.2 mm | 2.4–2.9 s | 8.0 / 9.1 / 9.2 | 8.9 / 10.0 / 10.2 |
| 0.1 mm | 8.5–9.1 s | 8.0 / 9.1 / 9.2 | 8.9 / 10.0 / 10.2 |

Below a 0.3 mm bore (an effective area of 0.04 mm², a Cv of about
0.003) the fill is a flow the loop follows, the chamber floor is set
by the loop's own dip (the same as at the distal opening: 2.0 MPa for
the GVL gains, 0.9 at wc = 11, 0.8 at wc = 16), and the fill takes one
to nine seconds. At 2:1 that keeps every arm above saturation with the
required margin (0.8 MPa for the GVL, 1.9 and 2.0 for the scheduled
arms); at 2.5:1 and 11 MPa only the scheduled arms clear it. The
saturation floor plus the 0.5 MPa margin is the dotted line of
`hose_opening.png`.

The solution that degasses is the part admitted while the hose was
still below the saturation pressure, P_sat times the hose's compliance:
1.0 mL at 1:1, 2.0 mL at 2:1, 2.6 mL at 2.5:1, whatever the bore and
whichever controller. It is the first solution through the valve, so
by plug flow it lies right behind the virgin saline, in the first
6 cm of hose at the cylinder end (the 29 mL of virgin saline fill the
other 92 cm), and about 0.9 mL of sound solution follows it. The purge
that carries it out is 31 mL at 2:1 and 32 mL (31.6) at 2.5:1 instead of 29,
and the scenario sizes it so (`V_purge` of None). Against the
procedural remedy this costs 2 to 3 mL of solution and needs a
metering path; it leaves the compression and mixing as they are now.

A proximal valve assembly can do this with no skill from the operator:
a fixed metering orifice in a bypass around the ball valve, opened
first to equalise the hose, and the ball valve opened once it has
(the ball valve then passes nothing, since the pressures are equal).
The bore sweep above is that orifice: 0.3 mm or less, about 0.04 mm²
of effective area, passes 6 mL/s at the full 10 MPa difference and
fills the hose in one to two seconds, the loop keeping the chamber
within its own dip. A smaller orifice is slower and no safer, since
the chamber floor is the loop's dip, not the fill's, below 0.3 mm.
The solution crossing the orifice sees, by Bernoulli, the pressure of
the hose it enters: the throat velocity is √(2(P − P_h)/ρ), 141 m/s
at the start of the fill, and the kinetic energy it carries is what
the pressure difference became, so the throat pressure is P_h (less
only what the orifice recovers downstream, which a sharp orifice does
not). The degassing flag at the hose pressure is therefore right to
first order, and the exposure, 2 µm of orifice at 141 m/s, is too
short for bubbles to grow in any case.

**A distal transducer gives no usable lead.** A transducer at the
distal end of the hose would see the distal valve open before the
chamber transducer does, by the time the pressure wave takes to run
up the hose. The hose line's compressibility is the saline's plus the
wall's per unit volume, 1/K + C_hose/V_hose = 9.7e-9 per Pa (an
effective bulk modulus of 0.1 GPa), so the wave runs at 320 m/s (850
in rigid saline) and crosses the 0.92 m hose in 2.8 ms, under one
scan. The loop's own delay from a reading to piston motion is about
80 ms: the 20 ms filter, the median and pipeline scans (16 ms), the
command's scan, the 22.8 ms drive dead time and the 18.2 ms
velocity-loop lag. A lead of 2.8 ms
cannot be turned into feedforward that acts before the dip; the hose
would have to be 15 m long. What a transducer downstream of the
distal valve, in the tube, would give is a measurement of the outflow
itself, Q = Cd·A_o·√(2P_tube/ρ) by eq. (36), and a valve-state signal.
That is the flow feedforward the studies carry as a bound, and it is
limited by the same 50 ms: the dip it leaves is of the order of the
delivery speed times the delay times the chamber stiffness,
1.4 mm/s × 0.05 s × 4.8 MPa/mm = 0.3 MPa, against the 0.6 to 0.8 MPa
the scheduled controller leaves on feedback alone. It would need a
second analog input and a transducer rated for the tube's pressure
cycling; the model would take it as the `flow_ff` option with the
measurement delayed by the sensor chain, which is not yet built.

**Delivery and the stop-law floor.** The orifice bank passes 4.8 mL/s at
10 MPa (the bench measured it), which needs the piston at 1.4 mm/s, and
the run from the first opening to the limit switch takes 75 s: the
29 mL purge, the pause and 249 mL of solution. With StopVelMin at its
earlier 0.5 mm/s the loop sat low for the whole delivery: the stop law,
which with 100 MPa/mm and 50 mm/s² admits the 1.4 mm/s the outflow
needs only at an error of about 2 MPa, holds the integrator through its
anti-windup. The bench saw this: the loop held 8.1 MPa for a 10 MPa
target. `StopVelMin` of 2 mm/s, now the GVL value, removes it. With the
hose open its compliance caps the end stiffness at 9.8 MPa/mm (K
0.72 GPa, 86 mL, 0.24 mL/MPa), where the GVL gain keeps 62° of phase
margin and 13 dB of gain margin, so the emptying chamber no longer
threatens the loop.

**Closing the purge valve is the second limit.** When the distal valve
is shut at the end of the purge the piston is running at 1.4 mm/s into
a chamber of 4.8 MPa/mm, and the pressure rises at 7 MPa/s until the
loop has stopped it. The GVL loop, with Kp 0.5 mm/s per MPa and an
integral time of 2.5 s, lets it rise 2.3 MPa, to 12.3 MPa from a 10 MPa
target: through the ceiling. The scheduled loop at wc = 11 stops it in
1.1 MPa, at wc = 16 in 0.9 MPa. A slow hand on the valve helps little
(a 1 s closing takes 0.1 MPa off the peak), because the loop, not the
valve, is the slow element. A gain the chamber allows is the remedy,
or a procedure that stops the piston before the valve is shut.

**Scheduled controller.** `ScheduledSlaveController` (equations (39) to
(41)) with the measured bounds (K_cfg 1.0 GPa, dead volume 81 mL, hose
compliance 0.12 mL/MPa) holds its margins constant through the run,
58° / 11 dB at wc = 6 rad/s, 37° / 5.8 dB at wc = 11 and 19° / 2.6 dB at
wc = 16 (the 44 ms of pure delay in the loop leaves little phase at a
crossover of 16 rad/s). Nominal results, 1:1 at 10 MPa, hose opened before the
compression, all with StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / peak MPa / V_ok mL |
|---|---|---|
| StopVelMin 0.5 (earlier) | 5.0 / 0.55 | 2.01 / 4.4 / 10.6 / 22 (steady error) |
| GVL | 4.8 / 0.73 | 2.00 / 4.5 / 12.3 / 29 (ceiling at the purge closing) |
| scheduled, wc 6 | 3.0 / 0.13 | 1.38 / 5.1 / 11.6 / 249 (all) |
| scheduled, wc 11 | 2.4 / 0.11 | 0.91 / 5.5 / 11.1 / 249 (all) |
| scheduled, wc 11, Ki = Kp, VelMax 7 | 2.0 / 0.14 | 0.90 / 5.5 / 11.2 / 249 (all) |
| scheduled, wc 16 | 2.4 / 0.18 | 0.78 / 5.7 / 10.9 / 249 (all) |
| flow feedforward (bound, not implementable) | — | 2.24 / 4.2 / 12.7 / 29 |

(The 0.11 MPa "overshoot" of the scheduled arms at wc = 6 and 11 is the
transducer offset; at wc = 16 the loop overshoots the reading by a
further 0.07 MPa, the sign of its thin phase margin. V_ok is given here as solution: the metric itself counts the
29 mL purge too, so 249 mL of solution reads as 278 mL delivered.) The
scheduled arms compress twice as fast as the GVL with
no overshoot of the reading (wc = 16 aside), because the stop law brakes for the
estimated stiffness and the whole lag (measurement, dead time and
drive) and the integrator is held until arrival; they cut the opening
dip to under half, because the gain at the opening is set by the measured
stiffness, hose included, rather than a fixed worst case; and they are
the only arms that keep the purge closing under the ceiling. VelMax
7 mm/s shortens the compression by 0.5 s and does not change the dip;
the drive does not slew, so accel and decel settings do nothing.

**The dip and the closing peak set the charge.** The dip at an opening
is the 0.3 MPa tube step plus the quasi-static Q/(A·Kp) of eq. (33),
recovered over the integral time: 2.00 MPa at Kp 0.5, 0.91 at wc 11
(Kp 2.3 at 4.8 MPa/mm), 0.78 at wc 16. The saturation pressure is
3.7 MPa per unit of ratio in saline, so each megapascal of dip costs
about 0.27 in the ratio, and a higher target buys 0.27 per megapascal
the other way, until the closing peak meets the ceiling. Solution
delivered before a violation at nominal parameters, 0.5 MPa margin
(the margin or the peak in parentheses where it fails):

| Charge | P_sat MPa | StopVelMin 0.5 | GVL | sched wc 6 | sched wc 11 | sched wc 16 |
|---|---|---|---|---|---|---|
| 1:1 | 3.6 | 0 mL (steady error) | 0 (peak 12.3) | 249 | 249 | 249 |
| 2:1 | 7.2 | 0 | 0 (peak 12.3) | 249 (margin 1.4) | 249 (1.9) | 249 (2.0) |
| 2.5:1 at 10 MPa | 9.0 | 0 | 0 | 0 (−0.4) | 0 (0.1) | 0 (0.2) |
| 2.5:1 at 11 MPa | 9.0 | 0 | 0 (peak 13.3) | 0 (peak 12.7) | 0 (1.0, peak 12.1) | 249 (1.1, peak 12.0) |
| 3:1 at 11 MPa | 10.9 | 0 | 0 | 0 | 0 (−0.8) | 0 (−0.7) |
| 2:1, hose opened after mixing | 7.2 | 0 | 0 | 0 (−0.8) | 0 (−0.8) | 0 (−0.8) |

So 2:1 is deliverable at 10 MPa by the scheduled controller at any of
the three crossovers, 2.5:1 needs wc = 16 and an 11 MPa target (where
the closing peak reaches 12.0 MPa, at the ceiling; wc = 11 peaks at
12.1 and fails), and 3:1 is saturated at
10.9 MPa and out of reach below the ceiling. The
GVL gains deliver nothing without a violation, because the purge
closing takes the pressure to 12.3 MPa; were the ceiling 12.5 MPa they
would deliver 1:1 and 2:1 (margin 0.8). The target is a stronger lever
than the gains for the dip, and the closing peak is what caps it:
`--set P_target=11e6` runs any study at another target, and
`--set hose_before=0` the present procedure.

**Robustness** (100 Latin hypercube samples over the measured ranges;
pass rate for every requirement, in percent):

| Charge, target, procedure | GVL | sched wc 11 | sched wc 16 |
|---|---|---|---|
| 1:1 at 10 MPa, hose before | 0 | **100** | **100** |
| 2:1 at 10 MPa, hose before | 0 | **100** | **100** |
| 2.5:1 at 10 MPa, hose before | 0 | 0 | 0 |
| 2.5:1 at 11 MPa, hose before | 0 | 0 | 33 |
| 1:1 at 10 MPa, hose after mixing | 0 | **100** | 98 |
| 2:1 at 10 MPa, hose after mixing | 0 | 0 | 0 |

The GVL fails every sample on the ceiling at the purge closing (its
peak is 12.3 MPa at nominal and rises with K and the hose compliance's
lower end); it passes every sample on degassing at 1:1 and 2:1 with the
hose opened before the compression. The scheduled arms pass every
sample at 1:1 and 2:1 with that procedure: the measured ranges are
narrow enough that nothing in them threatens the saturation margin,
the ceiling or the end of the delivery. With the hose opened after
mixing, every arm degasses every sample at 2:1, by the equalisation
drop, and 1:1 passes with wc = 11; wc = 16 ends the compression outside
the band in two samples (98 %), both with a drive lag of 45 ms or more,
where its 19° of nominal phase margin is gone. At 2.5:1 the 10 MPa
target leaves 1.0 MPa to saturation, less than any arm's dip plus the
margin; at 11 MPa the closing peak sits at the ceiling, so wc = 16
passes 33 % and wc = 11 none; for wc = 16 the failures fall from 100 %
in the slowest third of the drive lags to 41 % in the fastest, and are
more frequent at low hose compliance (94 % against 41 %).

**What this means for the machine.**
- Open the proximal valve before the compression. Opened at pressure,
  the hose drops the chamber 3.6 MPa in one scan, which no controller
  can prevent and which degasses any charge above about 1.8:1.
- `G_cfgPressureStopVelMin` is now 2 mm/s. That alone removes the
  steady error the bench saw (8.1 MPa for a 10 MPa target).
- The GVL gains let the purge closing carry the pressure to 12.3 MPa,
  through the 12 MPa ceiling, in every case; until the gain is raised,
  stop the piston before shutting the distal valve, or confirm that
  the mechanical ceiling allows 12.5 MPa.
- The scheduled controller cuts the opening dip to under half (0.9 MPa
  at wc = 11 against 2.0), holds the closing peak to 11.1 MPa and
  compresses twice as fast with no overshoot; it delivers 2:1 at 10 MPa
  in every plausible case. Its margin is the same at every point of the
  run, but with 44 ms of loop delay it is 37° at wc = 11 and 19° at
  wc = 16: wc = 11 is the crossover to take, and 2.5:1 at 11 MPa is out
  of reach (wc = 16 delivers it in a third of the plausible cases, with
  the closing peak at the ceiling).
- 3:1 is saturated at 10.9 MPa and cannot be delivered without bubbles
  below the 12 MPa ceiling by any controller.
- The drive does not apply the accel and decel settings, so they are not
  tuning levers; VelMax 7 mm/s shortens the compression only.

**Margins against stiffness** (linear model, GVL gains, 20 ms filter,
22.8 ms dead time):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL + hose, K = 0.72 GPa, hose open | 4.5 | 19.6 dB | 69° | 2.3 | 4.9 |
| liquid, 343 mL, hose shut | 7.2 | 15.5 dB | 66° | 3.6 | 3.0 |
| liquid, 120 mL, hose open | 8.4 | 14.2 dB | 64° | 4.2 | 2.6 |
| liquid, 86 mL dead volume (limit switch), hose open | 9.6 | 13.0 dB | 62° | 4.8 | 2.3 |
| stop-law stiffness | 100 | −7.3 dB | −65° | 34 | 0.21 |

With the hose open its compliance caps the stiffness at 9.8 MPa/mm, so
the GVL gain keeps most of its margin to the limit switch; the price is
a soft chamber at the openings and the closing, where the fixed gain is
four times too low for the dip and the peak. The scheduled gain
Kp = wc/k holds 37° at wc = 11 everywhere.

**Settled:**
- The mechanical pressure ceiling is 12 MPa (15 MPa may break the
  piston). With the 434 Ω setting the true pressure is
  (reading + 0.104) / 0.971 MPa, so an 11 MPa target holds at 11.4 MPa;
  with the calibrated 421.4 Ω the reading is true to the meter the
  calibration used (about 1 %, 0.1 MPa at 10 MPa).
- The saline volume and the oxygen charge, which the schedule's
  stiffness bound needs, reach the slave over the protocol as holding
  registers.
- The proximal valve stays shut during the compression and opens after
  mixing (`StudyParams.hose_before` False). A switched bypass orifice is not
  fitted.

**Open questions:**
- The calibration's reference: a hand-held meter's mA range. A loop
  calibrator would tighten it.
- The lag at the piston (not resolved) and the dissolution time constant
  are the two unmeasured plant parameters; the robustness study covers
  the first, and the second sets only the hold time.
- The hose: how much of its 0.24 mL/MPa is wall and how much trapped
  air (air would make the proximal opening worse than eq. (43) at low
  pressure); whether the whole 29 mL is purged; how long the pause
  between the purge and the delivery is, and whether the tube drains
  during it (`tube_refills` in the scenario, off by default); and how
  fast the distal valve is shut (`StudyParams.Tclose`, 0 by default, which
  is the worst case and changes the peak by 0.1 MPa).
- Whether a bypass orifice of 0.3 mm or less (Cv 0.003), opened ahead
  of the ball valve at pressure, is worth fitting later
  (`study_hose_opening.py`).
