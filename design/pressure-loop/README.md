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
python3 design/pressure-loop/study_floor.py      # stop-law floors that follow the outflow, about 1 min; --mc 100 adds 3 min
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
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound), `ScheduledSlaveController` (the same block with its gains and stop law scheduled on an estimate of the chamber stiffness, equations (39) to (41)), `ScheduledSlaveControllerST` (the ported Structured Text blocks, wired as PRG_Main wires them) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). All share the `reset` and `step` interface. `FloorPolicy` sets the stop-law floor of the slave controllers from the detected outflow, equations (46) to (48). |
| `scenarios.py` | Defines the operating cases `compress`, `deliver`, `cycle` and the `stiffstep` probe, with the hose procedure and the valve timings (proximal opening, purge, pause, delivery). |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces, the valve event times and metrics. It sequences the valves: the hose opening, the purge, its closing and the reopening. |
| `metrics.py` | Computes the loop metrics (overshoot, settling time, ripple, stroke) and the process metrics (pressure ceiling, saturation margin, the deepest dip at a valve event, volume delivered before a violation, solution delivered). |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `study_compare.py` | Compares velocity and torque command and the slave controller over three scenarios. |
| `study_hose_opening.py` | Studies the proximal valve opened at pressure: the chamber floor, the solution admitted into the hose, the part of it that degassed and where it lies, against the valve bore (ball valve to needle valve), per charge and controller. |
| `study_delivery.py` | Runs the delivery to the empty chamber over a range of oxygen charges and reports the deliverable volume. |
| `study_floor.py` | Compares stop-law floors that follow the outflow, with and without the forward stop, against the fixed `StopVelMin`: nominal cases, a sweep of the policy settings and a Monte Carlo over the measured ranges. |
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
equations (1) to (18), (34) to (38) and (42) to (46) and the measured
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
valve at 1 s and runs until the saline is gone (about 75 s). `cycle`
does both in one run with a 60 s hold. `stiffstep` (a liquid-only
10 → 12 MPa step) is kept as a stiff-regime margin probe; it is not a
process case.

**Valve event times.** In a delivery the proximal opening and the first
opening of the distal valve are at fixed times, but the purge ends on
volume: the distal valve closes once the purge volume (the hose's
virgin saline plus any solution that degassed entering it) has left,
and reopens a fixed `Tclose + t_pause` later. The orifice flow rises
with the chamber pressure, eq. (36), so a controller that holds a
higher pressure through the purge closes the valve sooner, and the
closing and reopening fall at a different time in each run: the purge
of the 6 ft hose closes at about 18.4 s for the GVL at 1:1, 18.2 s for
the scheduled controller, whose dip is smaller, and 19.5 s for a
StopVelMin of 0.5 mm/s, which sits 2 MPa low. The degassed volume also depends on the controller when the
hose is opened at pressure. Each run records its own event times
(`Result.events`), and the metrics use them.

**Scheduled slave controller.** A comparison arm that is not in the
slave: with the measured chamber it gives the results of the fixed
gain it arrives at (see the results). It runs
`FB_PressureController` with Kp, Ki and the stop law set every scan from
an upper bound on the chamber stiffness computed from what the slave
has: the measured pressure, the piston position against the limit
switch, and the configured dead volume, saline volume, charge and bulk
modulus, eq. (39). The free gas is taken as the least there can be
(the charge less what the saline could hold at the highest pressure
seen), the saline volume as the configured value or the chamber volume
when that is smaller, and the bulk modulus as the largest plausible, so
the bound never falls below the true stiffness while the configured
saline volume is at or below the true one. The bound is taken at the
present pressure and again at the larger of the present and the target
pressure, with the highest pressure as it stands; a gas-dominated
chamber is stiffest at the target, so the larger of the two, k_sched,
is used for the gain and the stop law. The gain is Kp = wc/k_sched for
a chosen crossover wc, eq. (40), with Ki in a fixed ratio to Kp, so the
loop's crossover is at most wc whatever the chamber is doing: soft on
the approach, stiffening as the gas dissolves, and stiffening again as
the chamber empties. The stop law, eq. (41), brakes for the estimated
stiffness and for the measurement lag, through the block's VelMax
input. The integrator is held at zero until the pressure first comes
within 0.5 MPa of the target, because the low scheduled gain would
otherwise let it wind up on the approach. The block multiplies its
stored integral by Ki, so the integral is rescaled when Ki changes.

**Stop-law floor policies.** The stop law clamps the velocity to zero
at zero error, which suits a sealed chamber; with the outlet open the
piston must keep moving at the outflow speed to hold the pressure, and
`StopVelMin` is the floor that lets it. `FloorPolicy` replaces the fixed
floor with one set every scan from what the slave measures, and can add
a forward stop. It applies to `SlaveController` and
`ScheduledSlaveController` through their `floor` argument; the block
has the `switched` policy with the forward stop built in, behind
`G_cfgPressureFloorEnable` (the override `FloorEnable=True`), and a
controller given a `FloorPolicy` runs with the block's own floor off.
The
`switched` policy, eq. (46), detects the outflow without a stiffness
value: in a sealed chamber a piston moving forward raises the pressure,
so a forward-moving piston under a clamped command with the pressure
not rising is displacing outflow. It then raises the floor from `v_lo`
to `v_hi`, and drops it when the pressure rises above the target. The
`estimated` policy, eq. (47), sets the floor from the outflow speed
estimated with an assumed stiffness. The forward stop, eq. (48), holds
a forward command at zero above the target while no outflow is
detected. The `fixed` policy holds `v_hi` with no detection, to isolate
the forward stop.

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

The same expression evaluated with P_abs taken at max(P, P_target) (and
P_ref unchanged) gives k_target, and k_sched = max(k_est, k_target).

Scheduled gains for a target crossover wc, bounded, with Ki in a fixed
ratio to Kp:

    (40) Kp = clamp(wc / k_sched, Kp_min, Kp_max),   Ki = r_i Kp

Stop law with the measurement lag: the piston travels for the lag at its
speed and then decelerates at D, and must stop within the error at the
estimated stiffness, v lag + v^2 / (2 D) <= |e| / k_sched, so

    (41) v_stop = -D lag + sqrt((D lag)^2 + 2 D |e| / k_sched)
         v_lim = min(VelMax, max(StopVelMin, v_stop))

The block's own stop law, eq. (19), is this with lag = Ts and a fixed
stiffness.

### Stop-law floor policies

The floor policies of `FloorPolicy` replace the constant StopVelMin of
eqs. (19) and (41) with v_min[n]. Over a window of W scans, with the
piston position x [mm] delayed by the measurement lag of d scans so that
it and the measured pressure P_meas [MPa] describe the same interval,
the pressure rise rate r [MPa/s] and the piston speed s [mm/s] are

    (46) r = (P_meas[n] - P_meas[n-W]) / (W Ts)
         s = (x[n-d] - x[n-d-W]) / (W Ts)

In a sealed chamber r = k s with k > 0. The switched floor latches L
when, for t_on, the block's previous output was clamped and forward,
e > e_on, s >= v_on and r < r_on; it releases L when r > r_off and
e <= e_off; and

    (46a) v_min = v_hi if L else v_lo

The estimated floor, with an assumed stiffness k_lo [MPa/mm], is

    (47) v_out = s - r / k_lo
         v_min = clamp(v_out + margin, v_lo, v_hi)

with L taken as v_min > v_lo. The forward stop limits the command u
after the block:

    (48) u <- min(u, 0)   if e < 0 and not L

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
  it, plus the drive's five scans of dead time.
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
| K | 1.4–2.2 GPa | log | the liquid, 1.70 GPa from two fill levels; water is 2.2 |
| C_series | 0.20–0.32 mL/MPa | log | series compliance of the seals, cylinder and frame, 0.26 mL/MPa from the same fits |
| Vl0 | 320–360 mL | linear | fill at the 280 mm start, 343 mL from the geometry |
| passages | 52–62 mL | linear | measured 57 mL with the hose shut |
| hose compliance | 0.45–0.75 mL/MPa | log | 6 ft hose: 0.58 purged, 0.62 unpurged, by difference of bulk-modulus runs |
| tauDiss | 10–100 s | log | not measured |
| tauV | 5–30 ms | log | 7.5 ms at the motor; the piston side is not resolved beyond it |
| sensor noise | 1.5–5 kPa rms | log | measured 1.7 (with capacitor) to 4.6 kPa |
| orifice Cd | 0.8–0.9 | linear | measured 0.83–0.87 |
| supply pressure | 0.52–1.38 MPa | linear | 75–200 psi with the cylinder's charge |

The ranges are the bench measurements of 2026-11-05 with their spread
(`bench/characterisation/README.md` section 8, `bench/friction`,
`bench/pressure`). `mr`, `b`, `Fc` and `mu` act only in torque mode and
the stall, so they are not varied; the dead time is fixed at its
measured 18.2 ms. The air pocket at the pressure transducer, `V_air`,
is explicit in the model (eq. (46) of `models/README.md`) and is held at
its default, 0.1 mL; it is not sampled, since its compliance is below
0.1 % of the liquid's above 1 MPa. Each sample runs compress and deliver and checks these requirements:

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
the measured bounds (K_cfg 1.0 GPa, dead volume 109 mL, hose compliance
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
Heun's method at five steps per scan and the measured plant of
2026-11-05: saline of K 1.70 GPa in series with the cylinder's
0.26 mL/MPa, 343 mL of it with 57 mL of passages and the 6 ft hose
(58 mL, 0.58 mL/MPa), Cd 0.84, an 18.2 ms dead time and 7.5 ms lag in
the drive, and the calibrated transducer (no offset).

**Velocity versus torque command.** Velocity command wins. The drive's
own velocity loop absorbs the 5 kN of Coulomb and 2.3 MN s/m of
viscous friction and the mass-spring resonance, so the pressure loop
sees an integrator behind a short lag. In torque command the measured
friction is a dead band of 6 MPa and more at pressure, so a force
command cannot place the pressure at all. Torque remains only as the
stall cap, about 21 MPa at 100 % rated torque.

**Compression** (1:1 charge from a 1 MPa supply, hose open): the piston
sweeps 12.5 mm, and the GVL loop arrives in band after 5.3 s with
0.72 MPa of overshoot; the scheduled loop at wc = 11 rad/s arrives in
2.9 s with 0.01 MPa. The chamber stiffness rises from 0.1 to
3.1 MPa/mm (6.7 with the hose shut). The legacy controllers without the
configured stop law overshoot to 13.5–20 MPa; the stop law is what
makes the approach safe.

**Opening the hose at pressure is the first limit.** In the present
procedure the proximal valve is opened after mixing, with the chamber
at 10 MPa and the hose at atmosphere. The stored compression of 343 mL
of saline must then stress 58 mL more and dilate the hose wall, and
with the ball valve the pressure falls from 10 to 4.3 MPa inside one
scan, eq. (43). It is the same for every controller, because the loop
can only recover it afterwards (the piston must advance about 1.8 mm).
A 1:1 charge keeps 0.7 MPa above saturation through it, just over the
0.5 MPa margin; a 1.5:1 charge degasses (−1.1 MPa), and so does any
charge above about 1.1:1. The 6 ft hose holds twice the 3 ft one's
liquid at twice its compliance, which is what took the drop from 3.6
to 5.7 MPa. Two remedies exist. One is procedural: open the proximal
valve before the compression, so that the hose's saline is compressed
with the charge (`StudyParams.hose_before`). The other is to open it
slowly enough that the chamber keeps up, which the next paragraph
quantifies. Every result below is for the first remedy unless marked.

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
| 6.35 mm (ball valve) | 0.7–1.7 s | 4.3 / 4.3 / 4.3 | 4.7 / 4.7 / 4.7 |
| 1.0 mm | 0.8–1.8 s | 5.1 / 6.3 / 6.4 | 5.6 / 7.0 / 7.1 |
| 0.5 mm | 1.2–2.1 s | 6.4 / 8.1 / 8.1 | 7.1 / 9.0 / 8.9 |
| 0.3 mm | 2.4–3.3 s | 7.9 / 9.1 / 9.3 | 8.7 / 10.0 / 10.3 |
| 0.2 mm | 4.9–5.6 s | 8.1 / 9.1 / 9.4 | 9.0 / 10.1 / 10.3 |
| 0.1 mm | 18–19 s | 8.1 / 9.1 / 9.4 | 9.0 / 10.1 / 10.3 |

At a 0.2 mm bore and below (an effective area of 0.02 mm², a Cv of
about 0.0013), and nearly at 0.3 mm, the fill is a flow the loop
follows: the chamber floor is set by the loop's own dip (the same as at
the distal opening: 1.9 MPa for the GVL gains, 0.9 at wc = 11, 0.7 at
wc = 16), and the fill takes five to nineteen seconds. At 2:1 that
keeps every arm above saturation with the required margin (0.9 MPa
for the GVL, 1.9 and 2.1 for the scheduled arms); at 2.5:1 and 11 MPa
only the scheduled arms clear it (1.1 and 1.3 MPa). The saturation
floor plus the 0.5 MPa margin is the dotted line of
`hose_opening.png`.

The solution that degasses is the part admitted while the hose was
still below the saturation pressure, P_sat times the hose's
compliance with its liquid (0.61 mL/MPa): 2.2 mL at 1:1, 4.4 mL at 2:1,
5.6 mL at 2.5:1, whatever the bore and whichever controller. It is the
first solution through the valve, so by plug flow it lies right
behind the virgin saline, in the first 14 cm of hose at the cylinder
end (the 58 mL of virgin saline fill the other 183 cm), and about
1.8 mL of sound solution follows it. The purge that carries it out is
62 mL at 2:1 and 63 mL at 2.5:1 instead of 58, and the scenario sizes
it so (`V_purge` of None). Against the procedural remedy this costs 4
to 6 mL of solution and needs a metering path; it leaves the
compression and mixing as they are now.

A proximal valve assembly can do this with no skill from the operator:
a fixed metering orifice in a bypass around the ball valve, opened
first to equalise the hose, and the ball valve opened once it has
(the ball valve then passes nothing, since the pressures are equal).
The bore sweep above is that orifice: 0.2 mm, about 0.02 mm² of
effective area, passes 3 mL/s at the full 10 MPa difference and fills
the 6 ft hose in about five seconds, the loop keeping the chamber
within its own dip. A smaller orifice is slower and no safer, since
the chamber floor is the loop's dip, not the fill's, below 0.2 mm.
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
wall's per unit volume, 1/K + C_hose/V_hose = 1.06e-8 per Pa (an
effective bulk modulus of 0.09 GPa), so the wave runs at 307 m/s
(1300 in rigid saline) and crosses the 1.83 m hose in 6.0 ms, under
two scans. The loop's own delay from a reading to piston motion is
about 66 ms: the 20 ms filter, the median and pipeline scans (16 ms),
the command's scan, the 18.2 ms drive dead time and the 7.5 ms
velocity-loop lag. A lead of 6 ms cannot be turned into feedforward
that acts before the dip; the hose would have to be 20 m long. What a
transducer downstream of the distal valve, in the tube, would give is
a measurement of the outflow itself, Q = Cd·A_o·√(2P_tube/ρ) by
eq. (36), and a valve-state signal. That is the flow feedforward the
studies carry as a bound, and it is limited by the same 50 ms: the
dip it leaves is of the order of the delivery speed times the delay
times the chamber stiffness, 1.4 mm/s × 0.05 s × 3.2 MPa/mm =
0.2 MPa, against the 0.7 to 0.8 MPa the scheduled controller leaves on
feedback alone. It would need a second analog input and a transducer
rated for the tube's pressure cycling; the model would take it as the
`flow_ff` option with the measurement delayed by the sensor chain,
which is not yet built.

**Delivery and the stop-law floor.** The orifice bank passes 4.8 mL/s
at 10 MPa (the bench measured it), which needs the piston at
1.4 mm/s, and the run from the first opening to the limit switch is
the 58 mL purge, the pause and 217 mL of solution. With StopVelMin at
its earlier 0.5 mm/s the loop sat low for the whole delivery: the stop
law, which with 100 MPa/mm and 50 mm/s² admits the 1.4 mm/s the
outflow needs only at an error of about 2 MPa, holds the integrator
through its anti-windup. The bench saw this: the loop held 8.1 MPa for
a 10 MPa target. `StopVelMin` of 2 mm/s, now the GVL value, removes
it. With the hose open its compliance caps the end stiffness at
3.8 MPa/mm (K 1.70 GPa and the series 0.26 mL/MPa, 115 mL,
0.58 mL/MPa), where the GVL gain keeps 71° of phase margin and 23 dB of
gain margin, so the emptying chamber no longer threatens the loop.

**Closing the purge valve is the second limit.** When the distal valve
is shut at the end of the purge the piston is running at 1.4 mm/s into
a chamber of 3.2 MPa/mm, and the pressure rises at 4.5 MPa/s until the
loop has stopped it. The GVL loop, with Kp 0.5 mm/s per MPa and an
integral time of 2.5 s, lets it rise 2.1 MPa, to 12.1 MPa from a
10 MPa target: through the ceiling. The scheduled loop at wc = 11 stops
it in 0.9 MPa, at wc = 16 in 0.7 MPa. A slow hand on the valve helps
little, because the loop, not the valve, is the slow element. A gain
the chamber allows is the remedy, or a procedure that stops the piston
before the valve is shut.

**Scheduled controller.** `ScheduledSlaveController` (equations (39) to
(41)) with the measured bounds (K_cfg 1.0 GPa, dead volume 109 mL, hose
compliance 0.12 mL/MPa) holds its margins constant through the run,
62° / 12.7 dB at wc = 6 rad/s, 46° / 7.5 dB at wc = 11 and 29° / 4.2 dB
at wc = 16 (the 40 ms of pure delay in the loop leaves little phase at
a crossover of 16 rad/s). K_cfg 1.0 GPa is below the saline's 1.70, but
the schedule has no series term, and with it the bound stays above the
chamber's stiffness up to 620 mL (`charScheduleCheck` confirms it on
the bench). Nominal results, 1:1 at 10 MPa, hose opened before the
compression, all with StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / peak MPa / solution mL |
|---|---|---|
| StopVelMin 0.5 (earlier) | 5.6 / 0.44 | 2.04 / 4.4 / 10.5 / 0 (steady error) |
| GVL | 5.3 / 0.72 | 1.94 / 4.5 / 12.1 / 0 (ceiling at the purge closing) |
| scheduled, wc 6 | 3.8 / 0.03 | 1.36 / 5.1 / 11.5 / 217 (all) |
| scheduled, wc 11 | 2.9 / 0.01 | 0.84 / 5.6 / 10.9 / 217 (all) |
| scheduled, wc 11, Ki = Kp, VelMax 7 | 2.3 / 0.05 | 0.81 / 5.6 / 10.8 / 217 (all) |
| scheduled, wc 16 | 2.7 / 0.01 | 0.66 / 5.8 / 10.7 / 217 (all) |
| flow feedforward (bound, not implementable) | — | 2.14 / 4.3 / 12.4 / 0 |

(The solution column is what was delivered before a violation, less
the 58 mL purge, which the metric `V_ok` counts too: 217 mL of solution
reads as 275 mL delivered.) The scheduled arms compress about twice as
fast as the GVL with no overshoot, because the stop law brakes for the
estimated stiffness and the whole lag (measurement, dead time and
drive) and the integrator is held until arrival; they cut the opening
dip to under half, because the gain at the opening is set by the
measured stiffness, hose included, rather than a fixed worst case; and
they are the only arms that keep the purge closing under the ceiling.
VelMax 7 mm/s shortens the compression by 0.6 s and does not change the
dip; the drive does not slew, so accel and decel settings do nothing.

**The dip and the closing peak set the charge.** The dip at an opening
is the tube step plus the quasi-static Q/(A·Kp) of eq. (33), recovered
over the integral time: 1.94 MPa at Kp 0.5, 0.84 at wc 11 (Kp 3.4 at
3.2 MPa/mm), 0.66 at wc 16 (Kp at its 5 mm/s per MPa limit). The
saturation pressure is 3.6 MPa per unit of ratio in saline, so each
megapascal of dip costs about 0.27 in the ratio, and a higher target
buys 0.27 per megapascal the other way, until the closing peak meets
the ceiling. Solution delivered before a violation at nominal
parameters, 0.5 MPa margin (the margin or the peak in parentheses
where it fails):

| Charge | P_sat MPa | StopVelMin 0.5 | GVL | sched wc 6 | sched wc 11 | sched wc 16 |
|---|---|---|---|---|---|---|
| 1:1 | 3.6 | 0 mL (steady error) | 0 (peak 12.1) | 217 | 217 | 217 |
| 2:1 | 7.2 | 0 | 0 (peak 12.1) | 217 (margin 1.4) | 217 (2.0) | 217 (2.1) |
| 2.5:1 at 10 MPa | 9.0 | 0 | 0 | 0 (−0.4) | 0 (0.1) | 0 (0.3) |
| 2.5:1 at 11 MPa | 9.0 | 0 | 0 (peak 13.2) | 0 (peak 12.5) | 216 (1.1, peak 11.9) | 216 (1.3, peak 11.7) |
| 3:1 at 11 MPa | 10.9 | 0 | 0 | 0 | 0 (−0.7) | 0 (−0.6) |
| 1:1, hose opened after mixing | 3.6 | 0 | 0 (peak 12.1) | 217 (0.7) | 217 (0.7) | 217 (0.7) |
| 2:1, hose opened after mixing | 7.2 | 0 | 0 | 0 (−2.9) | 0 (−2.9) | 0 (−2.9) |

So 2:1 is deliverable at 10 MPa by the scheduled controller at any of
the three crossovers, 2.5:1 needs an 11 MPa target and wc = 11 or 16
(the closing peak then 11.9 or 11.7 MPa, under the ceiling; wc = 6
peaks at 12.5 and fails), and 3:1 is saturated at 10.9 MPa and out of
reach below the ceiling. With the hose opened after mixing only 1:1 is
deliverable, with 0.2 MPa to spare over the margin. The GVL gains
deliver nothing without a violation, because the purge closing takes
the pressure to 12.1 MPa; were the ceiling 12.5 MPa they would deliver
1:1 and 2:1 (margin 0.9). The target is a stronger lever than the gains
for the dip, and the closing peak is what caps it:
`--set P_target=11e6` runs any study at another target, and
`--set hose_before=0` the present procedure.

**Robustness** (100 Latin hypercube samples over the measured ranges;
pass rate for every requirement, in percent):

| Charge, target, procedure | GVL | sched wc 11 | sched wc 16 |
|---|---|---|---|
| 1:1 at 10 MPa, hose before | 1 | **100** | **100** |
| 2:1 at 10 MPa, hose before | 1 | **100** | **100** |
| 2.5:1 at 10 MPa, hose before | 0 | 0 | 0 |
| 2.5:1 at 11 MPa, hose before | 0 | 81 | **100** |
| 1:1 at 10 MPa, hose after mixing | 1 | 69 | 69 |
| 2:1 at 10 MPa, hose after mixing | 0 | 0 | 0 |

The GVL fails all but one sample on the ceiling at the purge closing
(its peak is 12.2 MPa at p50); it passes every sample on degassing at
1:1 and 2:1 with the hose opened before the compression. The scheduled
arms pass every sample at 1:1 and 2:1 with that procedure, with at
least 1.8 MPa over saturation at 2:1. With the hose opened after
mixing, every arm degasses every sample at 2:1, by the equalisation
drop, and 31 % of the samples at 1:1, whatever the controller: the
drop leaves 0.7 MPa at nominal, and a hose compliance in the upper
third of its range (68 % of failures there against 3 % in the lower)
or a small series compliance takes it under the 0.5 MPa margin. At
2.5:1 the 10 MPa target leaves too little to saturation for any arm's
dip plus the margin (and a fast dissolution leaves the compression
itself short in a quarter of the samples); at 11 MPa wc = 16 passes
every sample, with the closing peak at 11.7 to 11.9 MPa, and wc = 11
passes 81 %, failing on the ceiling where its peak reaches 12.0 to
12.1 MPa, more often with a high Cd and a low hose compliance.

**What this means for the machine.**
- The 6 ft hose makes opening the proximal valve at pressure the
  limit of the present procedure: the chamber drops 5.7 MPa in one
  scan, which no controller can prevent, and 1:1 is the most it can
  deliver, in only 69 % of the plausible cases. Opening the valve
  before the compression avoids the drop, and so does a bypass orifice
  of 0.2 mm opened ahead of the ball valve (five seconds to fill the
  hose); neither is fitted. The 3 ft hose dropped it 3.6 MPa.
- `G_cfgPressureStopVelMin` is now 2 mm/s. That alone removes the
  steady error the bench saw (8.1 MPa for a 10 MPa target).
- A floor that switches on detected outflow, with a forward stop above
  the target, keeps that and also removes the purge closing peak
  (see the stop-law floor study below).
- The GVL gains let the purge closing carry the pressure to 12.1 MPa,
  through the 12 MPa ceiling, in every case; until the gain is raised,
  stop the piston before shutting the distal valve, or confirm that
  the mechanical ceiling allows 12.5 MPa.
- The scheduled controller cuts the opening dip to under half (0.8 MPa
  at wc = 11 against 1.9), holds the closing peak to 10.9 MPa and
  compresses twice as fast with no overshoot; with the hose in the
  chamber from the start it delivers 2:1 at 10 MPa in every plausible
  case. Its margin is the same at every point of the run: with the
  faster drive (40 ms of pure delay in the loop) it is 46° at wc = 11
  and 29° at wc = 16. wc = 11 is the crossover to take for 2:1;
  2.5:1 at 11 MPa needs wc = 16 (every case, the closing peak 0.1 to
  0.3 MPa under the ceiling) or the forward stop (below).
- 3:1 is saturated at 10.9 MPa and cannot be delivered without bubbles
  below the 12 MPa ceiling by any controller.
- The drive does not apply the accel and decel settings, so they are not
  tuning levers; VelMax 7 mm/s shortens the compression only.
- The solution delivered from the 343 mL fill is 217 mL: the 6 ft hose
  takes 58 mL of virgin saline to purge and 58 mL more of dead volume.

**Margins against stiffness** (linear model, GVL gains, 20 ms filter,
18.2 ms dead time, 7.5 ms lag):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL + hose, hose open | 3.2 | 24.2 dB | 70° | 1.6 | 8.2 |
| liquid, 343 mL, hose shut | 7.5 | 16.8 dB | 69° | 3.8 | 3.5 |
| liquid, 115 mL dead volume (limit switch), hose open | 3.8 | 22.7 dB | 71° | 1.9 | 6.9 |
| stop-law stiffness | 100 | −5.7 dB | −51° | 37 | 0.26 |

With the hose open its compliance and the cylinder's cap the stiffness
at 3.8 MPa/mm, so the GVL gain keeps most of its margin to the limit
switch; the price is a soft chamber at the openings and the closing,
where the fixed gain is seven times too low for the dip and the peak.
The scheduled gain Kp = wc/k holds 46° at wc = 11 everywhere. The
series compliance also caps the hose-shut chamber: however little
liquid is left, it is no stiffer than 13 MPa/mm.

**Settled:**
- The mechanical pressure ceiling is 12 MPa (15 MPa may break the
  piston). With the calibrated 421.4 Ω the reading is true to the meter
  the calibration used (about 1 %, 0.1 MPa at 10 MPa).
- The saline volume and the oxygen charge, which the schedule's
  stiffness bound needs, reach the slave over the protocol as holding
  registers.
- The proximal valve stays shut during the compression and opens after
  mixing (`StudyParams.hose_before` False). A switched bypass orifice is not
  fitted.
- The chamber's compliance is the saline's (1.70 GPa) plus a constant
  0.26 mL/MPa of the cylinder and frame (eq. (49) of
  `models/README.md`); the hose's is mostly wall (0.58 of 0.62 mL/MPa
  after a purge).

**Open questions:**
- The dissolution time constant is the one unmeasured plant parameter;
  it sets only the hold time, and the robustness study covers it. The
  lag at the piston is not resolved beyond the motor's.
- The hose: whether the 6 ft length is needed (with the 3 ft one the
  opening drop was 3.6 MPa, and it held 29 mL less); whether the whole
  58 mL is purged; how long the pause between the purge and the
  delivery is, and whether the tube drains during it (`tube_refills` in
  the scenario, off by default); and how fast the distal valve is shut
  (`StudyParams.Tclose`, 0 by default, which is the worst case).
- Whether a bypass orifice of 0.2 mm (Cv 0.0013), opened ahead of the
  ball valve at pressure, is worth fitting (`study_hose_opening.py`);
  with the 6 ft hose it is the only way to deliver more than 1:1 with
  the present procedure.

## Stop-law floor study

`study_floor.py` tests floors that follow the outflow, eqs. (46) to
(48), against the fixed StopVelMin of the GVL (2 mm/s). The arms are the
GVL; the earlier fixed 0.5 mm/s; the switched floor (0.5 to 2 mm/s)
alone; a fixed 2 mm/s floor with the forward stop; the switched and the
estimated floors with the forward stop; and the scheduled controller at
wc = 11 (the measured bounds of `study_delivery.py`) alone and with the
switched floor and the forward stop. The policy defaults are a 0.1 s
window, the position delayed by the measurement lag (9 scans), the
trigger at v_on 0.2 mm/s, r_on 0.1 MPa/s, e_on 0.05 MPa held for
40 ms, the release at r_off 1 MPa/s with e_off 0, and for the estimate
k_lo 4 MPa/mm and a 0.3 mm/s margin.

**Nominal, 1:1 at 10 MPa, hose opened before the compression**
(compression overshoot; the peak after the purge closes the valve; the
error's mean and standard deviation and the share of time the forward
stop holds the piston over the delivery stage; the detection delay
after the first opening and the release delay after the closing):

| Arm | Overshoot MPa | Closing peak MPa | V_ok mL | Error mean / std MPa | Stopped | Detect / release s |
|---|---|---|---|---|---|---|
| GVL (fixed 2) | 0.72 | 12.14 | 58 | 0.02 / 0.07 | 0 | — |
| fixed 0.5 | 0.44 | 10.51 | 22 | 2.04 / 0.00 | 0 | — |
| switched | 0.44 | 12.10 | 58 | 0.02 / 0.07 | 0 | 0.96 / 0.06 |
| fixed 2, forward stop | 0.08 | 10.26 | 276 | 0.08 / 0.10 | 14 % | — |
| switched, forward stop | 0.05 | 10.39 | 275 | 0.01 / 0.04 | 0 | 0.96 / 0.06 |
| estimated, forward stop | 0.06 | 10.80 | 275 | 0.02 / 0.07 | 0 | 0.03 / 0.21 |
| sched wc 11 | 0.01 | 10.88 | 275 | 0.01 / 0.02 | 0 | — |
| sched wc 11, switched, forward stop | 0.01 | 10.39 | 275 | 0.01 / 0.02 | 0 | 0.75 / 0.06 |

(V_ok counts the 58 mL purge: 58 mL is the purge alone, 275 mL the
purge and all 217 mL of solution.)

**Switching the floor alone does not stop the closing peak.** The
switched floor releases 60 ms after the valve shuts, yet the pressure
peaks at 12.10 MPa against the GVL's 12.14. The stop law limits the
speed in either direction, and with the error just below zero the
integrator still holds about 1.4 mm/s forward; once the error grows,
the stop-law clamp grows with it and lets that velocity through. What
it does do is keep the stop law in the sealed compression, where the
overshoot falls to that of the 0.5 mm/s floor.

**The forward stop removes the peak, and needs the detection.** With
the forward stop the closing peaks at 10.3 to 10.4 MPa (10.8 with the
estimated floor) and the compression overshoots by 0.05 to 0.08 MPa,
for every floor. Without detection (fixed 2 mm/s with the forward
stop) the delivery chatters: every excursion of the noisy reading
above the target stops the piston, so it is stopped 14 % of the time
and the error's standard deviation rises from 0.07 to 0.10 MPa, with a
mean error of 0.08 MPa. The latch lets the piston run through those
excursions while the outlet is open, which removes the chatter.

The closing peak, the delivery-stage window and the detection and
release delays are measured from each run's own valve events. The trace
plot, `floor_traces.png`, shows every arm on the GVL run's time axis
around its purge closing, so the other arms' closings and reopenings
fall a little earlier or later in it (see Valve event times).

**Monte Carlo** (100 Latin hypercube samples over the ranges of
`robust.py`, hose opened before the compression unless stated; pass
rate for every requirement, in percent; the closing peak at p50 / max):

| Case | GVL | switched, fs | estimated, fs | sched wc 11 | sched wc 11, switched, fs |
|---|---|---|---|---|---|
| 1:1 at 10 MPa | 1 | **100** | 99 | **100** | **100** |
| 1:1 at 10 MPa, hose after mixing | 1 | 69 | 69 | 69 | 69 |
| 2:1 at 10 MPa | 1 | **100** | 99 | **100** | **100** |
| 2.5:1 at 11 MPa | 0 | 0 | 0 | 81 | **100** |
| closing peak, 1:1, MPa | 12.2 / 12.4 | 10.4 / 10.5 | 10.8 / 11.2 | 10.9 / 11.0 | 10.4 / 10.5 |
| closing peak, 2.5:1 at 11 MPa, MPa | 13.3 / 13.5 | 11.4 / 11.6 | 11.9 / 12.8 | 12.0 / 12.1 | 11.4 / 11.6 |

The GVL fails all but one sample on the ceiling at the purge closing.
The switched floor with the forward stop clears the ceiling everywhere
at 10 MPa; with the hose opened after mixing every arm fails the same
31 % of samples, on the degassing at the opening drop, which no floor
or gain can help; and at 2.5:1 and 11 MPa the GVL gain's dip degasses
every sample whatever the floor. With the scheduled controller the
forward stop takes 2.5:1 at 11 MPa from 81 % to 100 %: the scheduled
loop fails there on the closing peak (p50 12.0 MPa), which the forward
stop brings to 11.4. No arm latches in the sealed hold after the
compression except the estimated floor (up to 8 latches at 2:1 and 11
at 2.5:1), whose k_lo reads the dissolution as outflow; it is also the
arm whose closing peak spreads to 12.8 MPa in one sample at 2.5:1. The
switched floor latches once during the compression in some samples,
where the gas-rich chamber rises slowly, and releases on the approach.

**Policy settings** (the sweep, on the 1:1 delivery for both procedures
and with the valve shut over 1 s):
- The latch needs a hold time: with t_on = 0 it chatters (more than 200
  transitions in a delivery and 15 to 18 latches in the sealed hold),
  and the closing peak returns to 12.0 MPa.
- r_off must stay below the slowest closing's rise rate: at 4 MPa/s the
  1 s closing never releases the latch and peaks at 12.1 MPa; 0.25 to
  1 MPa/s all work.
- r_on of 0.5 MPa/s lets the latch set during the compression, where
  the soft chamber rises slowly; 0.02 to 0.1 MPa/s do not.
- The window works from 0.04 to 0.2 s; at 0.4 s the release is late
  and the peak rises by 0.3 MPa (0.25 MPa with the 1 s closing).
- e_off from −0.1 to 0.1 MPa and e_on from 0 to 0.2 MPa change the
  peak by at most 0.1 MPa.
- With the forward stop the floors themselves matter little: v_hi from
  1.6 to 3 mm/s changes nothing, and v_lo from 0 to 1 mm/s only the
  compression overshoot (0.00 MPa at v_lo = 0, 0.06 MPa at 1 mm/s).
- The estimated floor follows a slow closing poorly: with the 1 s
  closing it releases after 1.03 s and peaks at 11.5 MPa, against
  0.34 s and 10.3 MPa for the switched floor.

**What this means for the machine.** The switched floor with the
forward stop is the candidate: it needs no stiffness value, keeps the
stop law in a sealed chamber, passes the outflow in a delivery and
stops the piston when the outlet shuts. On the GVL gains it removes the
closing peak that makes the GVL fail every sample; on the scheduled
controller at wc = 11 it extends delivery to 2.5:1 at 11 MPa in every
plausible case. The floor switching on its own is not enough; the
forward stop is what acts at the closing, and the latch is what lets
the forward stop run in a delivery. In the slave it is `FB_PressureFloor`,
ahead of the schedule and the controller ("Porting the floor"). None of it helps the present procedure with the
6 ft hose, whose limit is the opening drop.

## The floor in the slave

`FB_PressureController` (`slave-src/FB/`) holds the stop law, the
switched floor of eq. (46) and the forward stop of eq. (48). The floor
is off by default and is selected by `G_cfgPressureFloorEnable`; its
settings are the `G_cfgPressureFloor*` values of the GVL, with the
latched floor at `G_cfgPressureStopVelMin`.

The policy's arguments map to the settings as follows: `v_lo` to
`FloorVelLow`, `v_hi` to `StopVelMin`, `window` to `FloorWindow`,
`v_on`, `r_on`, `e_on` and `t_on` to `FloorTrigVel`, `FloorTrigRate`,
`FloorTrigError` and `FloorTrigTime`, `r_off` and `e_off` to
`FloorRelRate` and `FloorRelError`, and `forward_stop` to
`FloorForwardStop`. The position delay is the block's own, from the
filter time constant and the median window, as
`measurement_lag_scans()` computes it. The window and the delay are
held in rings of 64 samples, so the window plus the delay is at most
63 scans (25 and 9 at the defaults); a longer setting is the block's
`FloorError`, which leaves the configured floor and no forward stop.
The window and the latch are cleared while the block is disabled, so
each entry to pressure mode starts them afresh. The forward stop acts
on the output after the clamp, and the integrator and its anti-windup
see the output before it, as `FloorPolicy.limit_forward` is applied
after the block in the studies.

`check_floor_block` in `test_model.py` runs the block with a
`FloorPolicy` setting its floor and the block with its own floor in
lockstep through the compression and the 1:1 delivery, with both hose
procedures and a 1 s closing, at the GVL gains and at the earlier ones,
and requires the command, the floor and the latch to be equal on every
scan. `tests/slavePressureMode.m` runs the compiled slave through an
outflow and an abrupt closing: the latch sets and releases, and the
closing peaks at 10.25 MPa against 10.70 without the floor.
