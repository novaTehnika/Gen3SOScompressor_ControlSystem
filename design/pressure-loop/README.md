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
python3 design/pressure-loop/study_antiwindup.py # anti-windup rules and a derivative term, about 1 min
python3 design/pressure-loop/study_compensation.py # choice of compensator: linear design map and a gain grid, about 1 min
python3 design/pressure-loop/study_feedforward.py # outflow feedforward switched by the pressure, about 2 min; --mc 100 adds 20 min
python3 design/pressure-loop/study_linear.py     # margins against chamber stiffness, about 30 s
python3 design/pressure-loop/study_robust.py     # Monte Carlo over the uncertain parameters, about 7 min on 4 CPUs
python3 design/pressure-loop/study_convergence.py # integration error against the time step, about 1 min
python3 design/pressure-loop/test_linear.py      # linear model against the nonlinear simulation
python3 design/pressure-loop/test_robust.py      # sampler, criteria and parallel runner
```

The scripts run from the repository root or from this directory. They
accept these options:

| Option | Effect |
|---|---|
| `--sweeps` (compare only) | Also runs the gain sweeps of the legacy and slave controllers. |
| `--set key=value` | Overrides a `StudyParams` field (a model parameter or a study setting) (`K=2.2e9`, `solver=heun`) or a slave setting by its GVL name (`G_cfgPressureKp=1`). The option can be repeated. `as_configured=True` drops the settings the studies hold (`STUDY_CFG` in `studyparams.py`: the gains, the pressure filter and the integral clear the results here were computed with) and runs the slave as the GVL has it. |
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
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound), `ScheduledSlaveController` (the same block with its gains and stop law scheduled on an estimate of the chamber stiffness, equations (39) to (41)), `ScheduledSlaveControllerST` (the ported Structured Text blocks, wired as PRG_Main wires them) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). All share the `reset` and `step` interface. `FloorPolicy` sets the stop-law floor of the slave controllers from the detected outflow, equations (46) to (48). `VariantController` runs the slave's block with one anti-windup rule changed around it or a derivative term added, equations (50) to (56). `FeedforwardController` runs the block with a velocity feedforward for the outflow that a detector on the measured pressure switches, equations (64) to (66). |
| `scenarios.py` | Defines the operating cases `compress`, `deliver`, `cycle` and the `stiffstep` probe, with the hose procedure and the valve timings (proximal opening, purge, pause, delivery). A scenario's target is one step or a schedule of steps (`targets`). |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces, the valve event times and metrics. It sequences the valves: the hose opening, the purge, its closing and the reopening. |
| `metrics.py` | Computes the loop metrics (overshoot, settling time, ripple, stroke) and the process metrics (pressure ceiling, saturation margin, the deepest dip at a valve event, volume delivered before a violation, solution delivered). |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `study_compare.py` | Compares velocity and torque command and the slave controller over three scenarios. |
| `study_hose_opening.py` | Studies the proximal valve opened at pressure: the chamber floor, the solution admitted into the hose, the part of it that degassed and where it lies, against the valve bore (ball valve to needle valve), per charge and controller. |
| `study_delivery.py` | Runs the delivery to the empty chamber over a range of oxygen charges and reports the deliverable volume. |
| `study_floor.py` | Compares stop-law floors that follow the outflow, with and without the forward stop, against the fixed `StopVelMin`: nominal cases, a sweep of the policy settings and a Monte Carlo over the measured ranges. |
| `study_antiwindup.py` | Compares ways of managing the loop's integrator (clearing it past the target, tracking, an integration band) and a derivative term against the block as it is, on a soft gas-filled chamber at low pressure and on the 10 MPa cycle. |
| `linear.py`, `study_linear.py` | Build the discrete-time linear model of the velocity-command loop, equations (26) to (33), and use it to find the margins, the stability boundary and the frequency and disturbance responses. `linear.py` also takes a derivative term and gives the noise gain, the least damped mode and the gain and stiffness limits of a margin, equations (57) to (62). |
| `study_compensation.py` | Examines the choice of compensator on the linear model (the loop across stiffness, the integral, the derivative) and confirms it on a grid of Kp and Kd and on candidate gain sets in the nonlinear simulation. |
| `study_feedforward.py` | Tests a velocity feedforward for the outflow that a detector on the measured pressure switches on at an opening and off at a closing, against the block alone and against the feedforward told of the valve, with the pressure filter as configured and shorter: nominal cases, the detector's settings and a Monte Carlo. |
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

**Anti-windup and derivative variants.** `VariantController` is a
set of comparison arms. Of its rules the slave's block carries
`opposed` itself, selected by `G_cfgPressureIntegClearEnable` (the
override `IntegClearEnable=True`); the others and the derivative term
are not in the slave. It runs
`FB_PressureController` and changes only how the stored integral is
managed, by adjusting it before or after the block's call, so the stop
law, the floor and the forward stop stay the slave's. The block itself
integrates the error unless its output is clamped and the error drives
it further into the clamp. The variants clear the integral once per
approach after the target is passed (`arrival`, eq. (50)), on every
change of the error's sign (`crossing`, eq. (51)), or whenever the
pressure is past the target on the side the integral drives toward
(`opposed`, eq. (52)); let it decay there (`bleed`, eq. (53)); make it
track the clamp (`backcalc`, eq. (54)); or integrate only inside a band
about the target (`band`, eq. (55)). A derivative gain `Kd` subtracts
the filtered rate of the measured pressure from the block's output,
inside the block's clamp, eq. (56).

**Outflow feedforward.** `FeedforwardController` is a comparison arm
that is not in the slave. It runs `FB_PressureController` and adds the
piston speed that displaces the outflow, by the orifice law at the
target, eq. (66), while a detector finds the outlet open. The valves
are worked by hand and the slave has no signal of them, so the detector
reads the pressure the loop already measures: a fall the piston did not
cause, deeper than `d_on` over the window and no deeper than an outflow
makes, held for `t_on`, switches the feedforward on, eq. (64), and a
pressure past the target and rising switches it off, eq. (65). The
block's integral clear, floor and forward stop are as configured. The
`flow_ff` option of `SlaveController` is the same feedforward told of
the valve by the simulation, a bound that the machine cannot reach.

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

Optional flow feedforward, added after the block, with Q_ff the
outflow of eq. (36) through the valve fraction of the scan before:

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

### Anti-windup variants and derivative term

The variants of `VariantController` act on the block's stored integral
I [MPa s]. With e the error [MPa], T the target [MPa], u_raw = Kp e +
Ki I the block's output before its clamp and u after it [mm/s]:

Clear once per approach. An approach state a is 0 (none), ±1
(approaching from below or above) or ±2 (the target passed):

    (50) a <- sign(e)   on the first scan, or when |T[n] - T[n-1]| >
                        e_step, if |e| > e_step (else a <- 0)
         a <- 2 a       when |a| = 1 and e has changed sign
         a = 2:   I <- min(I, 0), a <- 0   if e < -e_over
                  a <- 0                   if e > e_over
         a = -2:  I <- max(I, 0), a <- 0   if e > e_over
                  a <- 0                   if e < -e_over

Clear on every change of the error's sign:

    (51) I <- 0   if e[n] e[n-1] < 0

Clear whenever the pressure is past the target, by more than e_over, on
the side the integral drives toward:

    (52) I <- min(I, 0)   if e < -e_over
         I <- max(I, 0)   if e > e_over

Decay in the same condition, with time constant T_b:

    (53) I <- I (1 - Ts / T_b)   if (e < -e_over and I > 0)
                                 or (e > e_over and I < 0)

Tracking (back-calculation) with time constant T_t, in place of the
block's conditional integration while the output is clamped:

    (54) I <- I + (e + (u - u_raw) / (Ki T_t)) Ts   while u != u_raw

Integration band of half-width e_band:

    (55) I advances only while |e| <= e_band

Derivative term, with d the filtered rate of the measured pressure of
eq. (21) [MPa/s] and v_lim the block's clamp of eq. (19):

    (56) u <- min(max(u - Kd d, -v_lim), v_lim)

### Compensation: derivative term, noise and the soft chamber

Controller with a derivative term on the measured pressure, with
a = Ts / (Tf + Ts) the coefficient of the derivative filter of eq. (21).
The term joins the loop; the target's path, eq. (31), keeps the PI part
alone:

    (57) C(z) = Kp + Ki Ts / (z - 1)
                + Kd a (z - 1) / (Ts (z - (1 - a)))

Sensor noise to command, with d_s the scans of delay in the sensor path,
and the RMS command per unit RMS of white noise per sample:

    (58) N = -C z^-d_s F S
         g^2 = (1 / pi) integral over 0..pi of |N(e^(j theta))|^2 d theta

Soft chamber. Where the crossover lies far below the delay and the
lags, the loop is the chamber's integrator and the controller's, with
k the stiffness [MPa/mm], v the piston velocity [mm/s] and w = Ki I the
velocity held by the integral [mm/s]:

    (59) dP/dt = k v,   v = Kp e + w - Kd dP/dt,   dw/dt = Ki e
         (1 + k Kd) d2e/dt2 + k Kp de/dt + k Ki e = 0

Its natural frequency and damping ratio:

    (60) wn = sqrt(k Ki / (1 + k Kd))
         zeta = (Kp / 2) sqrt(k / (Ki (1 + k Kd)))

Overshoot when the error crosses zero with the integral holding the
velocity w0, for Kd = 0 and zeta < 1:

    (61) e_peak = w0 sqrt(k / Ki) exp(-zeta phi / sqrt(1 - zeta^2))
         phi = atan(sqrt(1 - zeta^2) / zeta)

One loop for every stiffness. With Ki/Kp, Kd/Kp and Tf fixed, the open
loop of eq. (29) is a fixed function L_1 scaled by Kp k, so the
stiffness sets where the gain crosses one and the phase margin is read
from the one phase curve:

    (62) L(z) = Kp k L_1(z)
         PM(k) = 180 deg + arg L_1 at the frequency where
                 |L_1| = 1 / (Kp k)

Root locus of the soft chamber. The stiffness is the gain of the
locus. For Kd = 0 the complex roots of eq. (59) lie on a circle about
the integral zero at -Ki/Kp, and they meet on the real axis at the
stiffness k_b:

    (63) |s + Ki/Kp| = Ki/Kp,   k_b = 4 Ki / Kp^2

### Outflow feedforward from the pressure

The detector of `FeedforwardController` reads the measured pressure P
[MPa]. With e the error [MPa], W the window [scans], u the command
[mm/s] and d the drop of the pressure over the window, the feedforward
switches on at a fall the piston did not cause, no deeper than an
outflow makes, that lasts t_on:

    (64) d[n] = P[n - W] - P[n]
         on   when  d >= d_on,  e_on < e < e_max  and  u[n-1] >= 0
                    on every scan for t_on

It switches off when the pressure is past the target and rising, or far
past it:

    (65) off  when  e < -e_off  and  -d >= d_off,
                or  e < -e_hi

While it is on, the outflow's speed at the piston by the orifice law of
eq. (36) at the target T [MPa] is added to the block's output inside
the velocity clamp, with c = Cd A_o sqrt(2 / rho) / A the speed at
1 MPa (0.440 mm/s per sqrt(MPa) for the measured orifice bank):

    (66) v_ff = c sqrt(T)
         u <- min(max(u + v_ff, -VelMax), VelMax)

### Linear model

Plant, the zero-order-hold discretization of k / (s (tauV s + 1)), with
a_v = exp(-Ts/tauV) and t_v = tauV (1 - a_v):

    (26) Gd(z) = k ((Ts - t_v) z + (t_v - a_v Ts)) / ((z - 1)(z - a_v))

PI controller:

    (27) C(z) = Kp + Ki Ts / (z - 1)

Sensor filter:

    (28) F(z) = alpha z / (z - (1 - alpha))

Open loop, with d_s = (N - 1)/2 scans of delay in the sensor path (two
for N = 5) and n_dead = t_dead / Ts scans of drive dead time (five);
`PRG_Main` runs the sensor chain and the controller and writes the
command in one scan, so the program adds none:

    (29) L(z) = C(z) z^-n_dead Gd(z) z^-d_s F(z)

Sensitivity and complementary sensitivity:

    (30) S = 1 / (1 + L),   T = L / (1 + L)

Target to pressure. The target enters at the controller, so the sensor
path does not act on it:

    (31) R = C z^-n_dead Gd S

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
- `z^-n_dead` is the drive's five scans of dead time.
- `Gd(z)` is the drive velocity loop and the chamber integrator,
  eq. (26).
- `z^-2` is the median of 5 (two scans for a slowly varying signal).
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
are the GVL (Kp 1.5, Ki 0.6, a stop-law stiffness of 25 MPa/mm, the
20 ms filter and StopVelMin 2 mm/s, which passes the 1.4 mm/s the
measured outflow needs); the GVL with the block's floor that follows
the outflow; the GVL with the block's integral clear past the target,
alone and with the floor; the earlier gains (Kp 0.5, Ki 0.2,
100 MPa/mm) as a reference; as a comparison, the scheduled controller at wc = 11
and 16 rad/s with the measured bounds (K_cfg 1.0 GPa, dead volume
109 mL, hose compliance 0.12 mL/MPa); and `configured`, the loop with
the gains, pressure filter and integral clear the GVL has now. `--hose-before` runs the
proposed procedure (the hose opened before the compression). Every
configuration sees the same samples and noise seeds.
`--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine has
no signal of the valve's state, so no built-in configuration uses it.

## Results with the measured parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa·s (Ki). "GVL" is the
slave's `FB_PressureController` at the settings the studies hold
(`studyparams.STUDY_CFG`): Kp 1.5, Ki 0.6, the median of 5 and the
20 ms filter on the pressure input, and the integral clear off. Every
other setting is the GVL's: VelMax 5 mm/s, accel and decel 50 mm/s²
(which the drive does not apply), stop-law stiffness 25 MPa/mm and
StopVelMin 2 mm/s, with the floor that follows the outflow off. The
GVL itself has moved on from those five settings, on the bench's
evidence; "The loop as configured" at the end gives the figures for
what it has now, and `--set as_configured=True` runs a study with it. "GVL + floor" turns
that floor and its forward stop on (`G_cfgPressureFloorEnable`).
"Earlier gains" is the setting before these studies' retune: Kp 0.5,
Ki 0.2 and a stop-law stiffness of 100 MPa/mm. "Scheduled" is
`ScheduledSlaveController`, a comparison that is not in the slave. The
time-domain results use Heun's method at five steps per scan and the
measured plant of 2026-11-05: saline of K 1.70 GPa in series with the
cylinder's 0.26 mL/MPa, 343 mL of it with 57 mL of passages and the 6 ft
hose (58 mL, 0.58 mL/MPa), Cd 0.84, an 18.2 ms dead time and 7.5 ms lag
in the drive, and the calibrated transducer (no offset).

**Velocity versus torque command.** Velocity command wins. The drive's
own velocity loop absorbs the 5 kN of Coulomb and 2.3 MN s/m of
viscous friction and the mass-spring resonance, so the pressure loop
sees an integrator behind a short lag. In torque command the measured
friction is a dead band of 6 MPa and more at pressure, so a force
command cannot place the pressure at all. Torque remains only as the
stall cap, about 21 MPa at 100 % rated torque.

**The chamber's stiffness stays within a factor of four once the gas
is dissolved, so a fixed gain serves.** The saline alone would stiffen
without limit as the chamber empties, but the cylinder's series
compliance and the hose do not shrink with it: the liquid chamber is
3.2 MPa/mm with the hose open at a full fill, 3.8 at the limit switch,
7.5 with the hose shut at a full fill, and never above 13 (the series
compliance with no liquid left). The loop's crossover is about Kp times
the stiffness, so Kp 1.5 puts it at 5 rad/s with the hose open and
11 rad/s with it shut at a full fill, with 70° and 52° of phase margin.
A gain scheduled on a bound of the stiffness (`ScheduledSlaveController`
at wc = 11 rad/s, eqs. (39) to (41)) arrives at the same gain: its Kp is
1.48 at the target, and it then falls to 0.75 through a delivery because
its bound, which has no series compliance, stiffens to 14.6 MPa/mm while
the chamber goes from 3.2 to 3.8. Its results equal the fixed gain's in
every table below, which is why the slave has no schedule.

**Compression** (1:1 charge from a 1 MPa supply): the piston sweeps
12.5 mm with the hose open (10.9 with it shut), and the GVL loop arrives
in band after 2.9 s with 0.09 MPa of overshoot (2.3 s and 0.04 MPa with
the hose shut); 2:1 takes 4.9 s with 0.10 MPa. The floor's forward stop
brings the overshoot to 0.02 MPa. The earlier gains took 5.3 s with
0.72 MPa of overshoot, their stop law braking for a chamber of
100 MPa/mm and their integrator winding up on the way; the scheduled
loop takes the same 2.9 s as the GVL with 0.02 MPa. The chamber
stiffness rises from 0.1 to 3.1 MPa/mm (6.7 with the hose shut). The
legacy controllers without the configured stop law overshoot to
13.5–20 MPa; the stop law is what makes the approach safe. The stop-law
stiffness trades time for overshoot: 15 MPa/mm arrives 0.2 s sooner
with twice the overshoot, 100 MPa/mm two seconds later with 0.08 MPa.

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
the time the fill takes at the GVL gains:

| Bore | Fill | 2:1 at 10 MPa: GVL / earlier gains | 2.5:1 at 11 MPa: GVL / earlier gains |
|---|---|---|---|
| 6.35 mm (ball valve) | 0.8 s | 4.3 / 4.3 | 4.7 / 4.7 |
| 1.0 mm | 0.9 s | 6.3 / 5.2 | 6.9 / 5.7 |
| 0.5 mm | 1.3 s | 8.1 / 6.4 | 8.8 / 7.1 |
| 0.3 mm | 2.5 s | 9.1 / 7.9 | 10.1 / 8.8 |
| 0.2 mm | 4.9 s | 9.2 / 8.1 | 10.2 / 9.0 |
| 0.1 mm | 18 s | 9.2 / 8.1 | 10.1 / 9.0 |

At a 0.3 mm bore and below (an effective area of 0.04 mm², a Cv of
about 0.003) the fill is a flow the loop follows: the chamber floor is
set by the loop's own dip (the same as at the distal opening: 0.8 to
0.9 MPa for the GVL gains, 1.9 for the earlier ones), and the fill takes
two and a half to eighteen seconds. At 2:1 that keeps the GVL loop
1.9 MPa above saturation, well over the margin, and at 2.5:1 and 11 MPa
1.0 MPa. The saturation floor plus the 0.5 MPa margin is the dotted
line of `hose_opening.png`.

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
The bore sweep above is that orifice: 0.3 mm, about 0.04 mm² of
effective area, passes 6 mL/s at the full 10 MPa difference and fills
the 6 ft hose in two and a half seconds, the loop keeping the chamber
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
wall's per unit volume, 1/K + C_hose/V_hose = 1.06e-8 per Pa (an
effective bulk modulus of 0.09 GPa), so the wave runs at 307 m/s
(1300 in rigid saline) and crosses the 1.83 m hose in 6.0 ms, under
two scans. The loop's own delay from a reading to piston motion is
about 54 ms: the 20 ms filter, the median's scans (8 ms), the 18.2 ms
drive dead time and the 7.5 ms velocity-loop lag. A lead of 6 ms cannot be turned into feedforward
that acts before the dip; the hose would have to be 20 m long. What a
transducer downstream of the distal valve, in the tube, would give is
a measurement of the outflow itself, Q = Cd·A_o·√(2P_tube/ρ) by
eq. (36), and a valve-state signal. That is the flow feedforward the
studies carry as a bound, and the valve state is the part of it that
matters: a velocity of Q/A added from the scan after the valve opens
and removed in the scan after it shuts takes the opening dip from 0.81
to 0.35 MPa (the 0.22 MPa the tube takes as a step, and the loop's
delay) and the closing peak from 10.84 to 10.14 MPa, and it leaves
0.6 MPa over saturation at 2.5:1 and 10 MPa, which the loop alone does
not deliver. A transducer in the tube would need a second analog
input and a rating for the tube's pressure cycling; a switch on the
valve gives the state alone, and the orifice law of eq. (36) the flow.

**Delivery and the stop-law floor.** The orifice bank passes 4.8 mL/s
at 10 MPa (the bench measured it), which needs the piston at
1.4 mm/s, and the run from the first opening to the limit switch is
the 58 mL purge, the pause and 217 mL of solution. The stop law admits
that speed only at an error large enough for its assumed stiffness:
2 MPa at 100 MPa/mm, 0.5 MPa at 25. With a StopVelMin below the outflow
speed the loop therefore sits low for the whole delivery, holding the
integrator through its anti-windup. The bench saw this with the earlier
settings: the loop held 8.1 MPa for a 10 MPa target. `StopVelMin` of
2 mm/s removes it; the same loop with 0.5 mm/s holds 0.6 MPa low at the
GVL gains. With the hose open its compliance caps the end stiffness at
3.8 MPa/mm, where the GVL gain keeps 67° of phase margin and 15 dB of
gain margin, so the emptying chamber does not threaten the loop.

**Closing the purge valve is the second limit.** When the distal valve
is shut at the end of the purge the piston is running at 1.4 mm/s into
a chamber of 3.2 MPa/mm, and the pressure rises at 4.5 MPa/s until the
loop has stopped it. The earlier gains, with Kp 0.5 and an integral
time of 2.5 s, let it rise 2.1 MPa, to 12.1 MPa from a 10 MPa target:
through the ceiling. The GVL gains stop it in 0.8 MPa, and with the
floor's forward stop in 0.3 MPa. A slow hand on the valve helps little,
because the loop, not the valve, is the slow element.

**Nominal results**, 1:1 at 10 MPa, hose opened before the
compression, all with StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / peak MPa / solution mL |
|---|---|---|
| earlier gains | 5.3 / 0.72 | 1.93 / 4.5 / 12.1 / 0 (ceiling at the purge closing) |
| GVL | 2.9 / 0.09 | 0.81 / 5.6 / 10.8 / 217 (all) |
| GVL + floor | 2.9 / 0.02 | 0.81 / 5.6 / 10.3 / 217 (all) |
| scheduled, wc 11 | 2.9 / 0.02 | 0.82 / 5.6 / 10.9 / 217 (all) |
| scheduled, wc 16 | 2.7 / 0.01 | 0.62 / 5.8 / 10.6 / 217 (all) |
| GVL, flow feedforward from the valve state (bound: the machine has no such signal) | — | 0.35 / 6.1 / 10.1 / 217 |

(The solution column is what was delivered before a violation, less
the 58 mL purge, which the metric `V_ok` counts too: 217 mL of solution
reads as 275 mL delivered.) The GVL gains and the schedule at wc = 11
give the same compression, dip and closing peak. The schedule at
wc = 16 buys 0.2 MPa of dip for a phase margin of 39°. VelMax and the accel
and decel settings are not levers: the drive does not slew, and the
compression is limited by the stop law, not the clamp.

**The dip and the closing peak set the charge.** The dip at an opening
is the tube step plus the quasi-static Q/(A·Kp) of eq. (33), recovered
over the integral time: 0.81 MPa at Kp 1.5 against 1.93 at 0.5. The
saturation pressure is 3.6 MPa per unit of ratio in saline, so each
megapascal of dip costs about 0.27 in the ratio, and a higher target
buys 0.27 per megapascal the other way, until the closing peak meets
the ceiling. Solution delivered before a violation at nominal
parameters, 0.5 MPa margin (the margin or the peak in parentheses
where it fails):

| Charge | P_sat MPa | earlier gains | GVL | GVL + floor | sched wc 11 |
|---|---|---|---|---|---|
| 1:1 | 3.6 | 0 mL (peak 12.1) | 217 | 217 | 217 |
| 2:1 | 7.2 | 0 (peak 12.1) | 217 (margin 2.0) | 217 (2.0) | 217 (2.0) |
| 2.5:1 at 10 MPa | 9.0 | 0 | 0 (0.1) | 0 (0.1) | 0 (0.1) |
| 2.5:1 at 11 MPa | 9.0 | 0 (peak 13.2) | 216 (1.1, peak 11.9) | 216 (1.1, peak 11.3) | 216 (1.1, peak 11.9) |
| 3:1 at 11 MPa | 10.9 | 0 | 0 (−0.7) | 0 (−0.7) | 0 (−0.7) |
| 1:1, hose opened after mixing | 3.6 | 0 (peak 12.1) | 217 (0.7) | 217 (0.7) | 217 (0.7) |
| 2:1, hose opened after mixing | 7.2 | 0 | 0 (−2.9) | 0 (−2.9) | 0 (−2.9) |

So 2:1 is deliverable at 10 MPa, 2.5:1 needs an 11 MPa target (where
the closing peak is 11.9 MPa, 0.1 under the ceiling, or 11.3 with the
floor), and 3:1 is saturated at 10.9 MPa and out of reach below the
ceiling. With the hose opened after mixing only 1:1 is deliverable,
with 0.2 MPa to spare over the margin. The target is a stronger lever
than the gain for the dip, and the closing peak is what caps it:
`--set P_target=11e6` runs any study at another target, and
`--set hose_before=0` the present procedure.

**Robustness** (100 Latin hypercube samples over the measured ranges;
pass rate for every requirement, in percent):

| Charge, target, procedure | earlier gains | GVL | GVL + floor | sched wc 11 |
|---|---|---|---|---|
| 1:1 at 10 MPa, hose before | 2 | **100** | **100** | **100** |
| 2:1 at 10 MPa, hose before | 2 | **100** | **100** | **100** |
| 2.5:1 at 10 MPa, hose before | 0 | 0 | 0 | 0 |
| 2.5:1 at 11 MPa, hose before | 0 | **100** | **100** | 91 |
| 1:1 at 10 MPa, hose after mixing | 1 | 69 | 69 | 69 |
| 2:1 at 10 MPa, hose after mixing | 0 | 0 | 0 | 0 |

The earlier gains fail all but one or two samples on the ceiling at the
purge closing (12.2 MPa at p50). The GVL gains pass every sample at 1:1 and
2:1 with the hose opened before the compression, with the closing peak
at 10.9 MPa and at least 1.9 MPa over saturation at 2:1, and the
compression arrives in 3.0 s at p50 (5.0 s at 2:1). With the hose
opened after mixing, every arm degasses every sample at 2:1, by the
equalisation drop, and 31 % of the samples at 1:1, whatever the
controller: the drop leaves 0.7 MPa at nominal, and a hose compliance
in the upper third of its range (68 % of failures there against 3 % in
the lower) or a small series compliance takes it under the 0.5 MPa
margin. At 2.5:1 the 10 MPa target leaves too little to saturation for
the dip plus the margin (and a fast dissolution leaves the compression
itself short in a quarter of the samples); at 11 MPa the GVL gains pass
every sample with nothing to spare, the closing peak between 11.8 and
12.0 MPa (11.999 at worst), and the floor's forward stop holds it to
11.3 to 11.5 MPa. The schedule does no better than the fixed gain
anywhere, and worse at 2.5:1, where its peak passes the ceiling in 9 %
of the samples.

**What this means for the machine.**
- The pressure loop runs on fixed gains, Kp 1.5 and Ki 0.6, with the
  stop law at 25 MPa/mm. Against the earlier gains they halve the
  compression time, take the overshoot from 0.7 to 0.1 MPa, cut the
  opening dip from 1.9 to 0.8 MPa and bring the purge-closing peak
  from 12.1 MPa, through the ceiling, to 10.8. A gain schedule adds
  nothing to them on the measured chamber.
- The 6 ft hose makes opening the proximal valve at pressure the
  limit of the present procedure: the chamber drops 5.7 MPa in one
  scan, which no controller can prevent, and 1:1 is the most it can
  deliver, in only 69 % of the plausible cases. Opening the valve
  before the compression avoids the drop, and so does a bypass orifice
  of 0.3 mm opened ahead of the ball valve (two and a half seconds to
  fill the hose); neither is fitted. The 3 ft hose dropped it 3.6 MPa.
- With the hose in the chamber from the start the loop delivers 2:1 at
  10 MPa in every plausible case.
- The floor that follows the outflow, with its forward stop
  (`G_cfgPressureFloorEnable`), takes the closing peak a further
  0.5 MPa down and the compression overshoot to 0.02 MPa. At 2.5:1 and
  11 MPa, where the fixed gains pass every plausible case with the
  peak at the ceiling, it keeps 0.5 MPa below it (see the stop-law
  floor study below).
- `G_cfgPressureStopVelMin` stays at 2 mm/s: below the outflow's
  1.4 mm/s the loop holds a steady error through the delivery.
- 3:1 is saturated at 10.9 MPa and cannot be delivered without bubbles
  below the 12 MPa ceiling by any controller.
- The drive does not apply the accel and decel settings, so they are not
  tuning levers.
- The solution delivered from the 343 mL fill is 217 mL: the 6 ft hose
  takes 58 mL of virgin saline to purge and 58 mL more of dead volume.
- The gain is fixed, so the stiffest chamber the loop runs on sets its
  margin. Pressure mode with the hose shut and little liquid left is
  the stiff corner (up to 13 MPa/mm, 29° of phase margin); the
  procedure does not run it, and the bench's K ramps check the
  crossover before a new chamber is run
  (`bench/characterisation/README.md` section 9).

**Margins against stiffness** (linear model, GVL gains, 20 ms filter,
18.2 ms dead time, 7.5 ms lag):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL + hose, hose open | 3.2 | 17.0 dB | 70° | 4.8 | 10.7 |
| liquid, 115 mL dead volume (limit switch), hose open | 3.8 | 15.5 dB | 67° | 5.7 | 9.0 |
| liquid, 343 mL, hose shut | 7.5 | 9.6 dB | 52° | 10.9 | 4.6 |
| series compliance alone, hose shut (the stiffest possible) | 13.4 | 4.6 dB | 29° | 18.4 | 2.6 |

With the hose open its compliance and the cylinder's cap the stiffness
at 3.8 MPa/mm, so the loop keeps its margin to the limit switch. The
series compliance caps the hose-shut chamber too: however little
liquid is left, it is no stiffer than 13.4 MPa/mm, where Kp 1.5 is
still stable. A step of the target in the hose-shut liquid chamber
shows the thinner margin there: 10 to 12 MPa overshoots by 0.3 MPa.

**Settled:**
- The mechanical pressure ceiling is 12 MPa (15 MPa may break the
  piston). With the calibrated 421.4 Ω the reading is true to the meter
  the calibration used (about 1 %, 0.1 MPa at 10 MPa).
- The slave's pressure loop is one block with fixed gains; the saline
  volume and the oxygen charge still reach the slave over the protocol
  as holding registers, and the loop does not use them.
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
  delivery is, and whether the tube drains during it (the bench's second
  opening, 15 s after the first closing, drops the chamber 2.2 MPa in
  one exchange, so something downstream does; `tube_refills` in
  the scenario, off by default); and how fast the distal valve is shut
  (`StudyParams.Tclose`, 0 by default, which is the worst case).
- Whether a bypass orifice of 0.3 mm (Cv 0.003), opened ahead of the
  ball valve at pressure, is worth fitting (`study_hose_opening.py`);
  with the 6 ft hose it is the only way to deliver more than 1:1 with
  the present procedure.
- The gains are tuned on the model with one bench session's series
  compliance; the bench's first pressure-mode runs check them.

## Stop-law floor study

`study_floor.py` tests floors that follow the outflow, eqs. (46) to
(48), against the fixed StopVelMin of the GVL (2 mm/s). The arms are the
GVL; the earlier gains; a fixed 0.5 mm/s floor; the switched floor
(0.5 to 2 mm/s) alone; a fixed 2 mm/s floor with the forward stop; the
switched and the estimated floors with the forward stop; and the
scheduled controller at wc = 11 (the measured bounds of
`study_delivery.py`) alone and with the switched floor and the forward
stop. The switched floor with the forward stop is the one in the slave
(`G_cfgPressureFloorEnable`). The policy defaults are a 0.1 s window,
the position delayed by the measurement lag (7 scans), the trigger at
v_on 0.2 mm/s, r_on 0.1 MPa/s, e_on 0.05 MPa held for 40 ms, the release
at r_off 1 MPa/s with e_off 0, and for the estimate k_lo 4 MPa/mm and a
0.3 mm/s margin.

**Nominal, 1:1 at 10 MPa, hose opened before the compression**
(compression overshoot; the peak after the purge closes the valve; the
error's mean and standard deviation and the share of time the forward
stop holds the piston over the delivery stage; the detection delay
after the first opening and the release delay after the closing):

| Arm | Overshoot MPa | Closing peak MPa | V_ok mL | Error mean / std MPa | Stopped | Detect / release s |
|---|---|---|---|---|---|---|
| GVL (fixed 2) | 0.09 | 10.84 | 275 | 0.01 / 0.02 | 0 | — |
| earlier gains | 0.72 | 12.12 | 58 | 0.02 / 0.07 | 0 | — |
| fixed 0.5 | 0.09 | 10.29 | 276 | 0.59 / 0.00 | 0 | — |
| switched | 0.09 | 10.84 | 275 | 0.01 / 0.02 | 0 | 1.40 / 0.05 |
| fixed 2, forward stop | 0.02 | 10.19 | 276 | 0.06 / 0.09 | 19 % | — |
| switched, forward stop | 0.02 | 10.33 | 275 | 0.01 / 0.02 | 0 | 1.40 / 0.05 |
| estimated, forward stop | 0.02 | 10.70 | 275 | 0.01 / 0.02 | 0 | 0.02 / 0.18 |
| sched wc 11 | 0.02 | 10.87 | 275 | 0.01 / 0.02 | 0 | — |
| sched wc 11, switched, forward stop | 0.01 | 10.33 | 275 | 0.01 / 0.03 | 0 | 1.16 / 0.05 |

(V_ok counts the 58 mL purge: 58 mL is the purge alone, 275 mL the
purge and all 217 mL of solution.)

**A low fixed floor leaves a steady error.** With StopVelMin at
0.5 mm/s the stop law admits the 1.4 mm/s the outflow needs only at an
error of 0.5 MPa (at 25 MPa/mm), so the delivery runs 0.59 MPa low; the
floor must be above the outflow speed while the outlet is open.

**Switching the floor alone does not stop the closing peak.** The
switched floor releases 50 ms after the valve shuts, yet the pressure
peaks at the GVL's own 10.84 MPa. The stop law limits the
speed in either direction, and with the error just below zero the
integrator still holds about 1.4 mm/s forward; once the error grows,
the stop-law clamp grows with it and lets that velocity through.

**The forward stop removes most of the peak, and needs the
detection.** With the forward stop the closing peaks at 10.2 to
10.3 MPa (10.7 with the estimated floor) and the compression
overshoots by 0.02 MPa, for every floor. Without detection (fixed
2 mm/s with the forward stop) the delivery chatters: every excursion of
the noisy reading above the target stops the piston, so it is stopped
19 % of the time and the error's standard deviation rises from 0.02 to
0.09 MPa, with a mean error of 0.06 MPa. The latch lets the piston run
through those excursions while the outlet is open, which removes the
chatter.

The closing peak, the delivery-stage window and the detection and
release delays are measured from each run's own valve events. The trace
plot, `floor_traces.png`, shows every arm on the GVL run's time axis
around its purge closing, so the other arms' closings and reopenings
fall a little earlier or later in it (see Valve event times).

**Monte Carlo** (100 Latin hypercube samples over the ranges of
`robust.py`, hose opened before the compression unless stated; pass
rate for every requirement, in percent; the closing peak at p50 / max):

| Case | earlier gains | GVL | switched, fs | estimated, fs | sched wc 11 | sched wc 11, switched, fs |
|---|---|---|---|---|---|---|
| 1:1 at 10 MPa | 2 | **100** | **100** | **100** | **100** | **100** |
| 1:1 at 10 MPa, hose after mixing | 1 | 69 | 69 | 69 | 69 | 69 |
| 2:1 at 10 MPa | 2 | **100** | **100** | **100** | **100** | **100** |
| 2.5:1 at 11 MPa | 0 | **100** | **100** | **100** | 91 | **100** |
| closing peak, 1:1, MPa | 12.2 / 12.3 | 10.9 / 11.0 | 10.4 / 10.5 | 10.7 / 10.9 | 10.9 / 11.0 | 10.4 / 10.5 |
| closing peak, 2.5:1 at 11 MPa, MPa | 13.3 / 13.5 | 11.9 / 12.0 | 11.4 / 11.5 | 11.8 / 11.9 | 11.9 / 12.0 | 11.4 / 11.5 |

At 10 MPa the GVL gains clear the ceiling without the floor; the floor
takes the closing peak from 10.9 to 10.4 MPa. With the hose opened
after mixing every arm fails the same 31 % of samples, on the degassing
at the opening drop, which no floor or gain can help. At 2.5:1 and
11 MPa the GVL gains pass every sample with the closing peak at the
ceiling (12.0 MPa at worst), and the switched floor with the forward
stop holds it at 11.3 to 11.5 MPa. No arm latches in the
sealed hold after the compression except the estimated floor (up to 11
latches at 2:1 and 13 at 2.5:1), whose k_lo reads the dissolution as
outflow. The switched floor latches once during the compression in some
samples at 2:1 and 2.5:1, where the gas-rich chamber rises slowly, and
releases on the approach.

**Policy settings** (the sweep, on the 1:1 delivery for both procedures
and with the valve shut over 1 s):
- The latch needs a hold time: with t_on = 0 it chatters (about 70
  transitions in a delivery and up to 2 latches in the sealed hold),
  and the closing peak rises to 10.8 MPa.
- With a 1 s closing the GVL loop stops the piston before the pressure
  rises at r_off, so the latch stays set and the peak is the loop's own,
  10.76 MPa; r_off at 0.25 MPa/s releases it (after 0.14 s) and brings
  the peak to 10.20. r_off of 4 MPa/s does not release even on the
  abrupt closing: the rise over the window stays below it, and the peak
  is the loop's own 10.84 MPa.
- r_on from 0.02 to 0.5 MPa/s, e_on from 0 to 0.2 MPa and e_off from
  −0.1 to 0.1 MPa change nothing at these gains.
- The window works from 0.04 to 0.2 s; at 0.4 s the release is late
  and the peak rises by 0.24 MPa.
- With the forward stop the floors themselves matter little: v_hi from
  1.6 to 3 mm/s and v_lo from 0 to 1 mm/s change the compression
  overshoot by 0.01 MPa.
- The estimated floor follows a slow closing poorly: with the 1 s
  closing it releases after 0.97 s.

**What this means for the machine.** The switched floor with the
forward stop is in the slave, off by default. It needs no stiffness
value, keeps the stop law in a sealed chamber, passes the outflow in a
delivery and stops the piston when the outlet shuts. On the GVL gains
it takes the closing peak from 10.8 to 10.3 MPa and the compression
overshoot from 0.09 to 0.02 MPa, and at 2.5:1 and 11 MPa, where the
fixed gains reach the ceiling, it keeps the peak 0.5 MPa below it. The floor switching on its
own is not enough; the forward stop is what acts at the closing, and
the latch is what lets the forward stop run in a delivery. A slow
closing argues for a lower `G_cfgPressureFloorRelRate` (0.25 MPa/s)
once the closing speed on the machine is known. None of it helps the
present procedure with the 6 ft hose, whose limit is the opening drop.

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
63 scans (25 and 7 at the defaults); a longer setting is the block's
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
closing peaks at 10.13 MPa against 10.15 without the floor: the
integral clear, on in the GVL, has taken the peak the floor's forward
stop would.

## Anti-windup and derivative study

`study_antiwindup.py` asks what should be done about the integrator in
a soft chamber. With free gas at a low pressure the chamber is softer
than the gains are set for by two orders of magnitude: 0.03 MPa/mm at
5 atm with a 1:1 charge of gas admitted at atmosphere, against the 3.2
to 7.5 MPa/mm of the liquid chamber. The proportional term then moves
the piston at a small fraction of a mm/s, the integrator carries the
piston to the target at the stop-law floor (2 mm/s), and the velocity it
holds on arrival balances nothing: only an error above the target
unwinds it. The loop is then two integrators in series, the
controller's and the chamber's, with a damping ratio of
(Kp / 2) sqrt(k / Ki), 0.2 at 0.05 MPa/mm. A sealed chamber needs no
integral at rest, since the chamber is itself an integrator from
velocity to pressure. A delivery does: the integral is what holds the
1.4 mm/s that balances the outflow at zero error. A rule that removes
the integral must tell the two apart.

The arms are `VariantController` (eqs. (50) to (56)) at the GVL gains,
with the floor that follows the outflow off unless marked. The cases:

- **Soft steps**: the bench fill of 2026-11-09 (279 mL of saline,
  293 mL of air at atmosphere, hose shut) stepped to 5, 20 and 100 atm
  absolute, and the same fill taken to 100 atm in one step.
- **Cycle**: the 1:1 oxygen cycle at 10 MPa, with the hose opened before
  the compression and after mixing.
- **Brief closing**: the cycle with the outlet shut for 0.5 s in place
  of the 10 s pause, so the integral that balanced the outflow is
  needed again at once.
- **Delivery step**: a delivery at 9 MPa whose target steps to 10 MPa
  with the outlet open.
- **Stiff step**: the liquid-only chamber stepped from 10 to 12 MPa
  with the hose shut.

**Results** (overshoot in MPa, with the time until the pressure stays
within 0.1 MPa of the target in seconds; the cycle is the one with the
hose opened before the compression):

| Arm | 5 atm step: overshoot / in band | 20 atm step: overshoot / in band | Compression at 10 MPa: overshoot / arrive | Closing: peak MPa / time above band | Dip at the reopening after 0.5 s shut | Delivery error: mean / std |
|---|---|---|---|---|---|---|
| GVL | 0.47 (117 %) / 72 | 0.53 (35 %) / 11.3 | 0.09 / 2.9 | 10.84 / 5.4 | 0.14 | 0.01 / 0.02 |
| arrival, eq. (50) | 0.02 / 34 | 0.05 / 5.8 | 0.04 / 2.9 | 10.84 / 5.4 | 0.14 | 0.01 / 0.02 |
| crossing, eq. (51) | 0.00 / 34 | 0.04 / 5.8 | 0.02 / 2.9 | 10.19 / 0.2 | 0.81 | 0.12 / 0.20 |
| opposed, e_over 0.02 MPa, eq. (52) | 0.02 / 34 | 0.05 / 5.8 | 0.04 / 2.9 | 10.24 / 0.2 | 0.81 | 0.01 / 0.02 |
| opposed, e_over 0.1 MPa | 0.10 / 39 | 0.13 / 6.5 | 0.09 / 2.9 | 10.33 / 0.3 | 0.81 | 0.01 / 0.02 |
| bleed, T_b 0.2 s, eq. (53) | 0.03 / 34 | 0.12 / 6.7 | 0.05 / 2.9 | 10.50 / 0.6 | 0.78 | 0.01 / 0.02 |
| backcalc, T_t 0.5 s, eq. (54) | 0.47 / 72 | 0.55 / 11.3 | 0.00 / 8.3 | 10.84 / 5.4 | 0.14 | 0.01 / 0.02 |
| backcalc, T_t 2.5 s | 0.50 / 72 | 0.83 / 11.6 | 1.52 / 2.7 | 10.84 / 5.4 | 0.14 | 0.01 / 0.02 |
| band 0.1 MPa, eq. (55) | never arrives | 0.01 / 17.3 | 0.00 / 3.0 | 10.01 / 0 | 0.88 | 0.88 / 0.00 |
| band 1.5 MPa | 0.47 / 72 | 0.53 / 11.3 | 0.09 / 2.9 | 10.84 / 5.4 | 0.14 | 0.01 / 0.02 |
| PID, Kd 0.02, eq. (56) | 0.47 / 72 | 0.53 / 11.3 | 0.09 / 3.0 | 10.82 / 5.4 | 0.14 | 0.01 / 0.02 |
| PID, Kd 0.05 | 0.47 / 72 | 0.53 / 11.4 | 0.10 / 3.1 | 10.81 / 5.4 | 0.14 | 0.01 / 0.02 |
| PID, Kd 0.1 | 0.47 / 72 | 0.53 / 11.4 | 0.11 / 3.2 | 10.79 / 5.4 | 0.13 | 0.01 / 0.02 |
| GVL + floor | 0.47 / 72 | 0.53 / 11.3 | 0.02 / 2.9 | 10.32 / 7.9 | 0.48 | 0.01 / 0.02 |
| arrival + floor | 0.02 / 34 | 0.05 / 5.8 | 0.02 / 2.9 | 10.32 / 7.9 | 0.48 | 0.01 / 0.02 |
| opposed 0.02 + floor | 0.02 / 34 | 0.05 / 5.8 | 0.02 / 2.9 | 10.24 / 0.2 | 0.81 | 0.01 / 0.02 |

**Only removing the integral past the target cures the soft
overshoot.** The GVL loop crosses the 5 atm target at 1.9 mm/s and
overshoots by 0.47 MPa, more than the step. `arrival`, `crossing`,
`opposed` and `bleed` all stop the piston within 0.04 MPa of the
target and are in band 38 s sooner; at 20 atm they take the
overshoot from 0.53 to 0.05 MPa (0.12 for the decay). None of them
shortens the approach itself, which the stop-law floor paces.

**Clearing on every crossing breaks the delivery.** With the outlet
open the pressure sits at the target with the integral holding the
outflow speed, and every crossing of the noisy error clears it:
`crossing` runs the delivery with an error of 0.12 MPa mean and 0.20 MPa
standard deviation against 0.01 and 0.02, and never reaches the new
target of the delivery step.

**Back-calculation and an integration band do not help.** Tracking
acts only while the output is clamped, and in the soft chamber the
output reaches the clamp at the floor, where the tracked integral is
the floor itself: the 5 atm overshoot is unchanged. A short tracking
time unwinds the integral on every clamped approach and takes 8.3 s
over the 10 MPa compression in place of 2.9; a long one lets the
integral wind up while clamped and overshoots the compression by
1.52 MPa (3.0 MPa when the bench fill is taken to 100 atm in one
step). A band narrower than the opening dip leaves the proportional
term alone outside it: at 0.1 MPa the 5 atm step does not arrive in
100 s and the delivery runs 0.88 MPa low. A band wider than the dip
(1.5 MPa) contains the whole 5 atm step and changes nothing.

**A derivative term does nothing for the soft chamber and is bounded
by the stiff one.** The loop gain of the derivative path is at most
Kd times the stiffness at any frequency, so a Kd below 0.07 mm/s per
MPa/s (one over 13.4 MPa/mm) is safe in the stiffest chamber whatever
the delays, and at 0.03 MPa/mm such a gain changes the loop by a fifth
of a percent: the 5 and 20 atm overshoots are those of the GVL.
Where the chamber is stiff it helps: Kd 0.05 takes the stiff step's
overshoot from 0.29 to 0.04 MPa, at three times the command activity
(0.0006 against 0.0002 mm/s per scan), and the arrival of the 20 to
100 atm step from 0.12 to 0.05 MPa. It moves the opening dip and the
closing peak by 0.03 MPa.

**`opposed` also acts at a closing.** When the outlet shuts, the
pressure rises above the target with the integral still holding the
outflow speed. `opposed` clears it 0.02 MPa above the target: the
closing peaks at 10.24 MPa against 10.84, and the pressure is back in
band after 0.2 s against 5.4 s (7.9 s with the forward stop, which
holds the piston while the integral unwinds). `arrival` leaves a
closing alone, since no approach is in progress. The cost is at a
reopening that follows at once: with the integral cleared, a 0.5 s
closing is followed by a full opening dip, 0.81 MPa against 0.14. With
the 10 s pause of the procedure every arm has unwound by the reopening
and dips the same 0.8 MPa.

**A false clear needs the reading 0.02 MPa above the target with the
outlet open.** Through a delivery the error stays positive (the
pressure about 8 kPa low), and the filtered reading's noise is under a
kPa. No rule changes the integral during the delivery stage at the
measured noise (1.7 kPa rms per sample with the capacitor, 4.6 bare) or
at 10 kPa. At 30 kPa `opposed` with 0.02 MPa clears six times and the
error's standard deviation rises from 0.02 to 0.22 MPa, with dips of
0.82 MPa; `bleed` acts five times without effect, because a decay
removes little in the scans a noise excursion lasts; `opposed` with
0.1 MPa does not act.

**The model against the bench** (`bench/captures/`, 2026-11-09). With
the GVL gains and the floor on, `trace_20261109_193852.csv` shows 0.35
and 0.51 MPa of overshoot at the 5 and 20 atm steps (from 0.14 MPa and
another fill), a closing peak of 10.40 MPa and 8 s above the band
after it; the `GVL + floor` arm gives 0.47 and 0.53 MPa, 10.32 MPa and
7.9 s. With the earlier gains, `trace_20261109_190837.csv` crosses the
5 atm target after 45 s at 1.97 mm/s and peaks at 1.77 MPa; the model
with those gains (`--set G_cfgPressureKp=0.5 --set G_cfgPressureKi=0.2
--set G_cfgPressureStopStiffness=100`) gives 44.6 s, 1.97 mm/s and
1.83 MPa. The stiff
arrival is where the model is pessimistic: the 20 to 100 atm step
overshoots by 0.02 MPa on the bench and 0.12 MPa in the model, so the
derivative term's benefit is not confirmed by a measurement.

**What this means for the machine.**
- The soft-chamber overshoot is the integrator's stored velocity, and
  the remedy is to remove the integral once the pressure is past the
  target. Neither a derivative term nor another form of anti-windup at
  the clamp reaches it.
- `opposed`, eq. (52), is the candidate for the slave: one comparison
  and one assignment per scan, with no state. At 0.02 MPa it takes the
  5 atm overshoot from 0.47 to 0.02 MPa and the closing peak from 10.84
  to 10.24 MPa, and it lets the pressure return after a closing in 0.2 s
  in place of 5 to 8 s. `bleed` trades a little of each for tolerance
  of a noisy reading; `arrival` gives the same soft-chamber result and
  leaves the delivery and the closing untouched.
- Clearing the integral at every crossing is right only for a sealed
  chamber and must not be used: the delivery depends on the integral at
  zero error.
- A derivative term is a separate lever, for the stiff arrival with the
  hose shut, and the bench has not shown that arrival to need it.
- `opposed` is in the slave's block, off by default (see the next
  section). The other variants are not in the slave.

## The integral clear in the slave

`FB_PressureController` (`slave-src/FB/`) holds the rule of eq. (52):
after the error is formed and before the PI terms, a positive integral
is cleared while the pressure is above the target by more than the
threshold, and a negative one while it is below it by more. It is off
by default and is selected by `G_cfgPressureIntegClearEnable`; the
threshold e_over is `G_cfgPressureIntegClearError`, 0.02 MPa (the
overrides `IntegClearEnable` and `IntegClearError`). A negative
threshold with the clear enabled is a parameter error of the block,
which then commands zero. The rule has no state of its own, and the
block's conditional integration, its stop law, its floor and its
forward stop are unchanged around it.

`check_integ_clear_block` in `test_model.py` runs the block with its
own rule and the block with `VariantController`'s `opposed` rule
applied around it in lockstep through the soft chamber stepped to
5 atm, the compression and the 1:1 delivery, at thresholds of 0.02 and
0.1 MPa and with the block's floor off and on, and requires the
command and the integral to be equal on every scan.
`tests/slavePressureMode.m` runs the compiled slave through a
compression, an outflow and an abrupt closing: with the clear the
compression overshoots by 0.03 MPa against 0.04 and the closing peaks
at 10.15 MPa against 10.52 (at the GVL's Kp 2.0 and Ki 0.8), and the
loop follows the outflow with its integral as it does without it.

**Monte Carlo** (`study_robust.py`, arms `GVL+clear` and
`GVL+floor+clear`; 100 Latin hypercube samples, the sensor noise from
1.5 to 5 kPa rms among them; pass rate for every requirement in
percent, and the closing peak at p50 / max in MPa):

| Case | GVL | GVL + floor | GVL + clear | GVL + floor + clear |
|---|---|---|---|---|
| 1:1 at 10 MPa, hose before | 100, 10.9 / 11.0 | 100, 10.4 / 10.5 | 100, 10.3 / 10.3 | 100, 10.3 / 10.4 |
| 2:1 at 10 MPa, hose before | 100, 10.9 / 11.0 | 100, 10.4 / 10.5 | 100, 10.3 / 10.3 | 100, 10.3 / 10.4 |
| 2.5:1 at 11 MPa, hose before | 100, 11.9 / 12.0 | 100, 11.4 / 11.5 | 100, 11.3 / 11.4 | 100, 11.3 / 11.4 |
| 1:1 at 10 MPa, hose after mixing | 69, 10.9 / 11.0 | 69, 10.4 / 10.5 | 69, 10.3 / 10.3 | 69, 10.3 / 10.4 |

The clear passes wherever the block passes without it, and fails the
same samples of the present procedure, which degas at the opening
drop. The margin over saturation through the delivery and the time to
arrive are the same to the digits printed in every case, so within the
measured noise the rule does not act falsely in a delivery. The
closing peak is 0.6 MPa lower than the block's own and 0.1 MPa lower
than the floor's; at 2.5:1 and 11 MPa that is 11.4 MPa at worst
against 12.0. The compression peaks within 0.05 MPa of the target in
every sample.

**On the machine** (`bench/captures/trace_20261110_193733.csv`: the
GVL gains with the clear at 0.02 MPa and, as in every trace of that
day, the pressure filter at a median of 3 and 5 ms, on about 430 mL of
liquid and 142 mL of air). The steps to 5, 20 and 100 atm overshoot by 0.023,
0.040 and 0.020 MPa. Without the clear the steps to 5 and 20 atm
overshot by 0.35 and 0.51 MPa (`trace_20261109_193852.csv`, another
fill). The piston crosses the 5 atm target at 1.86 mm/s, the stop-law
floor's speed held by the integral, and is at rest two exchanges
later. The study's soft steps, on its own fill, give 0.02, 0.05 and
0.12 MPa. In the same trace a proximal valve opened onto a hose full
of air takes the chamber from 10 to 0.4 MPa; the loop returns at
5 mm/s into a chamber that stiffens to 6 MPa/mm in the last half
millimetre and overshoots by 0.015 MPa. At Kp 2.0 and Ki 0.8
(`trace_20261110_195235.csv`) a step from atmosphere to 100 atm
overshoots by 0.018 MPa, with the same 2 kPa of spread within an
exchange at the hold as at Kp 1.5.

A delivery at Kp 2.0 and Ki 0.8 with the clear on
(`trace_20261110_195648.csv`, a chamber that held undissolved air)
agrees with the model where the loop is concerned and not where the
outlet is. The steady outflow takes the piston at 1.38 mm/s, 4.73 mL/s
against the orifice law's 4.78, with the error under 0.005 MPa. The
pressure peaks at 10.23 MPa when the distal valve shuts, at both
closings, and is back in band 0.2 s later; the simulation gives
10.2 MPa. A second after an opening the error is 0.6 to 1.0 MPa and
decays over the integral time, as the quasi-static dip of eq. (33)
(0.7 MPa at this gain) does. But the opening itself takes the chamber from 10.03 to 4.57 MPa
inside one 50 ms exchange, and a second opening 15 s after the first
closing to 7.88 MPa, where the model's tube takes 0.2 MPa: at about
1 mL/MPa the outlet takes some 5 mL at the first opening and 2 mL at
the second, not the tube's 0.24 mL, and takes it again after a pause.
A proximal valve opened slowly by hand in the same trace fills the
hose with a dip of 0.2 to 0.4 MPa.

At Kp 2.4 and Ki 0.96 on a small chamber (`trace_20261110_200342.csv`,
129 mL at the hold, 7 MPa/mm at the arrival) the step from atmosphere
to 100 atm overshoots by 0.14 MPa and swings 0.12 MPa below the target
before it settles, 0.3 s after the crossing, and the pressure's spread
within an exchange at the hold is 3.1 kPa against 2.1 to 2.4 kPa at
the lower gains, where the linear model gives that gain 50° and
7.9 dB at 7.5 MPa/mm with this filter. Its two distal openings drop the chamber by 1.4 and
0.9 MPa within an exchange, so the volume the outlet takes at an
opening varies from run to run, from about 1 to 5 mL; the recovery to
within 0.1 MPa takes 4.4 s against 5 to 5.5 s at Kp 2.0. The delivery
ends at the end-of-travel stop without a fault.

With `G_cfgDirectCtrlFilterTime` at 0 and the same gains
(`trace_20261110_204936.csv`, 237 mL at the hold with little air,
9.4 MPa/mm at the arrival) the step from 20 to 100 atm overshoots by
0.026 MPa and swings 0.024 MPa below the target, and the spread within
an exchange at the hold is back at 2.4 kPa. The torque reads 100 % for
one exchange at that step: without the filter the velocity command
reaches the drive as a step. Eight distal openings drop the chamber by
0.89 MPa at the first and 0.54 to 0.74 MPa at the others, each within
one exchange, with the error at 0.4 MPa a second later and back within
0.1 MPa after 4 to 4.5 s; the closings peak at 10.16 to 10.20 MPa. On
a chamber and hose with little air the opening is close to the
model's, whose tube step and loop give 0.5 MPa at this gain: the
larger drops of the two traces before belong to their air.

## Choice of compensation

`study_compensation.py` asks which compensator the loop should have,
and why, on the linear model of `linear.py`, and then confirms the
answer in the nonlinear simulation. The design chamber is the liquid
chamber with the hose shut at a full fill (7.5 MPa/mm); the delivery
runs at 3.2 MPa/mm and the stiffest possible chamber is 13.4. The
margins asked are 45° of phase and 6 dB of gain at the design chamber.

**The loop is one curve, and the stiffness slides the crossover along
it.** The plant is an integrator, the chamber, whose gain is the
stiffness, behind 28 ms of pure delay, the drive's 7.5 ms lag and the
20 ms filter. By eq. (62) a change of stiffness changes nothing but
the loop gain. The phase curve is fixed: it starts at −180° (two
integrators, the chamber's and the controller's), rises toward −90°
above the integral corner Ki/Kp, and falls again where the delay takes
over (`compensation_bode.png`). The GVL loop across the chamber's
range:

| k MPa/mm | Phase margin | Gain margin dB | Crossover rad/s | Damping of the least damped mode | Target step overshoot |
|---|---|---|---|---|---|
| 0.003 | 6° | 78 | 0.04 | 0.05 | 85 % |
| 0.01 | 11° | 67 | 0.08 | 0.09 | 76 % |
| 0.03 | 19° | 58 | 0.14 | 0.16 | 63 % |
| 0.1 | 33° | 47 | 0.27 | 0.30 | 45 % |
| 0.3 | 52° | 38 | 0.56 | 0.52 | 29 % |
| 1.0 | 70° | 27 | 1.5 | 0.99 | 15 % |
| 3.2 (delivery) | 70° | 17.0 | 4.8 | 1.00 | 7 % |
| 7.5 (design) | 52° | 9.6 | 10.9 | 0.54 | 19 % |
| 13.4 (stiffest) | 29° | 4.6 | 18.4 | 0.22 | 59 % |

The loop is sound over a decade and a half of stiffness and loses its
margin at both ends, for different reasons. At the stiff end the delay has
taken the phase. At the soft end the crossover has fallen below the
integral corner, where the controller's integrator adds its 90° to the
chamber's; the damping there is that of eq. (60), which the model's
poles follow within 3 %.

**The root locus against stiffness shows it in one picture**
(`compensation_rlocus.png`). The stiffness multiplies the loop gain, so
it is the gain of the locus. Two poles start at the origin, the
chamber's integrator and the controller's, and the PI term puts a zero
at −Ki/Kp, 0.4 rad/s. In the soft chamber the poles leave the origin
straight up the imaginary axis and ride a circle of radius Ki/Kp about
that zero, eq. (63): undamped at first, a damping ratio of 0.16 at
0.03 MPa/mm and 0.52 at 0.3. They meet on the real axis at
4 Ki/Kp², 1.07 MPa/mm. One then closes on the zero (−0.44 rad/s at the
delivery's 3.2 MPa/mm): it is the integral's mode, the 2.5 s with which
a dip recovers. The other runs left, meets the filter's pole coming in
from −45 rad/s, and the pair leaves the axis on the branch that the
delay bends into the right half plane: a damping of 0.54 at 7.5 MPa/mm,
0.22 at 13.4, and across the axis at 22.7 MPa/mm and 28 rad/s.

The zero is two things at once: the centre of the circle the soft
chamber's poles ride, and the place the slow pole ends in a delivery.
Moving it toward the origin (a smaller Ki/Kp) shrinks the circle, so
the poles reach the real axis in a softer chamber, and slows the
delivery's recovery in the same proportion: at Ki/Kp of 0.1 the radius
is 0.1 rad/s and the recovery 20 s. A derivative term adds a second
zero, at −28 rad/s for the set with the 20 ms filter below, two hundred
times farther from the origin than the soft chamber's poles at
0.13 rad/s, so its locus lies on the PI's circle. Where it acts is on
the delay's branch, which it lifts in frequency and which crosses the
axis sooner, at 14.9 MPa/mm against 22.7, with the stiffest chamber at
13.4.

**The delay sets the proportional gain.** A loop gain Kp k of
13.9 rad/s is the most that keeps 45° and 6 dB, whatever the stiffness.
At the design chamber that is Kp 1.85, with the phase margin binding;
a proportional gain alone could be 1.94. The GVL's 1.5 is 81 % of the
limit and has 52° and 9.6 dB there. A pure delay has no poles or zeros
of its own; on the locus it is the bend that takes the stiff chamber's
branch across the axis, and the further branches that come in from the
far left. The grey locus of `compensation_rlocus.png` is the same PI
loop with its pure delay removed: its two lags alone take it across the
axis at 117 MPa/mm and 77 rad/s, against 22.7 MPa/mm and 28 rad/s as
built.

**Where the delay comes from, and what each part is worth.** The loop
as built has 28 ms of pure delay and three lags:

| Source | Kind | Amount | Set by |
|---|---|---|---|
| Median of 5 | delay | 8 ms (2 scans) | `G_cfgAnalogMedianSize` |
| Drive dead time | delay | 18.2 ms (20 ms in whole scans) | the drive and `G_cfgDirectCtrlFilterTime` |
| Pressure low-pass filter | lag | 20 ms | `G_cfgAnalogFilterTimeConst` |
| Drive velocity loop | lag | 7.5 ms | the drive's tuning |
| Sample and hold | lag | 2 ms (half a scan) | `G_cfgScanTime` |

The program itself adds none: `PRG_Main` runs the filter, the scaling,
the controller and the command in that order, each reading the output
the block before it produced in the same scan. The largest Kp with the
margins at the design chamber (Ki = 0.4 Kp), with each part changed:

| Change | Pure delay | Kp | Dip MPa | Command noise µm/s per kPa |
|---|---|---|---|---|
| as built | 28 ms | 1.85 | 0.70 | 0.60 |
| median of 3 | 24 ms | 2.02 (+9 %) | 0.65 | 0.65 |
| pressure filter 10 ms | 28 ms | 2.20 (+19 %) | 0.60 | 0.94 |
| drive dead time 8 ms | 16 ms | 2.46 (+33 %) | 0.54 | 0.79 |
| scan 2 ms | 22 ms | 2.15 (+16 %) | 0.61 | 0.50 |
| median of 3 and filter 10 ms | 24 ms | 2.42 (+31 %) | 0.54 | 1.04 |
| those and the 8 ms dead time | 12 ms | 3.50 (+89 %) | 0.38 | 1.52 |
| no pure delay | 0 | 4.59 (+148 %) | 0.31 | 1.45 |
| block outputs read a scan late | 40 ms | 1.49 (−20 %) | 0.86 | 0.48 |

The drive's dead time is the largest single part. `Y_DirectControl`'s
command filter is inside it: the bench measured 22.8 ms of dead time
and an 18.2 ms lag with the filter at 50 ms and 18.2 and 7.5 ms at
20 ms, so each 10 ms of filter was 5 ms of response, and the 8 ms row
is that trend taken to a filter of zero; it has not been measured. A
2 ms scan halves the median's delay, the only one counted in scans that
the program sets, but the program takes about 2.5 ms a scan. The last
row is the cost of the order of a program: with the filter and the
scaling read before they are called (two scans) and the command taken
from the controller's previous call (one scan), the loop has 12 ms more
delay and a fifth less gain, more than the derivative term gives back
at an equal gain margin. A shorter delay raises the gain the stiff
chamber allows and moves the soft chamber little: its damping at
0.03 MPa/mm goes from 0.18 to 0.29 with no pure delay at all.

**The integral is for the outflow, and it sets the soft end.** With
the largest Kp for each Ki/Kp, on the orifice flow at 10 MPa
(4.8 mL/s) in the delivery chamber:

| Ki/Kp 1/s | Kp | Ki | Dip MPa | Back within 0.1 MPa after | Softest chamber with 45° | Stiffness range | Damping at 0.03 MPa/mm |
|---|---|---|---|---|---|---|---|
| 0 (P) | 1.94 | 0 | 0.72 | never | any | — | 1.00 |
| 0.1 | 1.92 | 0.19 | 0.71 | 19.8 s | 0.038 MPa/mm | 200 to 1 | 0.38 |
| 0.2 | 1.90 | 0.38 | 0.71 | 9.9 s | 0.077 | 97 to 1 | 0.26 |
| 0.4 | 1.85 | 0.74 | 0.70 | 5.0 s | 0.16 | 46 to 1 | 0.18 |
| 0.8 | 1.76 | 1.41 | 0.71 | 2.5 s | 0.37 | 20 to 1 | 0.12 |
| 1.6 | 1.55 | 2.48 | 0.73 | 1.3 s | 1.0 | 7 to 1 | 0.08 |

Without the integral the pressure stays 0.72 MPa low for the whole
delivery, eq. (33). With it the dip recovers in about two integral
times, and the range of stiffness over which the loop keeps its phase
margin is roughly 18 times the integral time in seconds: the crossover
must stay between the integral corner and the delay limit. The chamber
spans 0.003 to 13.4 MPa/mm, 4500 to 1. A fixed gain set covers that
only with an integral time of about four minutes, and a delivery that
takes over eight minutes to recover. Damping the 0.03 MPa/mm chamber to
0.7 alone takes Ki 0.034 (an integral time of 44 s) and leaves the
delivery 97 s to recover.

**A derivative term buys little, and only at the stiff end.** With
Ki = 0.4 Kp and the largest Kp for each Kd/Kp:

| Derivative filter | Best Kd/Kp | Kp | Kd | Dip MPa | Gain margin, stiffest chamber | Command noise | Kp with the PI's gain margin kept |
|---|---|---|---|---|---|---|---|
| none (PI) | — | 1.85 | 0 | 0.70 | 2.7 dB | 0.60 µm/s per kPa | 1.85 |
| 50 ms | 0.020 s | 2.00 (+8 %) | 0.040 | 0.63 | 1.0 dB | 0.83 | 1.85 (+0 %) |
| 20 ms | 0.015 s | 2.24 (+21 %) | 0.034 | 0.58 | 1.0 dB | 0.99 | 1.89 (+2 %) |
| 10 ms | 0.015 s | 2.38 (+28 %) | 0.036 | 0.55 | 1.3 dB | 1.17 | 2.03 (+9 %) |
| 4 ms | 0.020 s | 2.66 (+43 %) | 0.053 | 0.50 | 1.0 dB | 1.81 | 2.22 (+20 %) |

The lead lifts the phase near the crossover, so Kp can rise until the
gain margin binds in place of the phase margin. The gain margin is set
at the phase crossover, 28 rad/s, by the delay, and the derivative
raises the gain there; past Kd/Kp of 0.02 s the largest Kp falls again.
With the 20 ms filter the term is worth 21 % of gain (0.12 MPa of dip)
for a gain margin in the stiffest chamber of 1.0 dB in place of 2.7 and
1.7 times the command noise. Held to the PI loop's own gain margin it
is worth 2 %. A faster filter gives more, 43 % at 4 ms, for 3.0 times
the noise. In the soft chamber the term does nothing: by eq. (60) it
enters as 1 + k Kd, which is 1.001 at 0.03 MPa/mm. A lead placed at the
soft chamber's crossover, 0.14 rad/s, would be a Kd of about 10, and
the loop gain of the derivative path in the stiffest chamber, Kd k,
would be over a hundred.

**The grid in the nonlinear simulation** (Ki = 0.4 Kp, 20 ms derivative
filter; rows Kp, columns Kd):

Stiff step, 10 to 12 MPa with the hose shut, overshoot in MPa:

| Kp \ Kd | 0 | 0.02 | 0.04 | 0.06 | 0.08 |
|---|---|---|---|---|---|
| 1.0 | 0.13 | 0.11 | 0.11 | 0.12 | 0.13 |
| 1.5 | 0.29 | 0.09 | 0.03 | 0.04 | 0.05 |
| 2.0 | 0.41 | 0.22 | 0.05 | 0.02 | 0.03 |
| 2.5 | 0.49 | 0.31 | 0.15 | 0.03 | 0.02 |
| 3.0 | 0.55 | 0.37 | 0.22 | 0.09 | 0.04 |

Cycle at 10 MPa, hose opened before the compression, deepest dip in
MPa:

| Kp \ Kd | 0 | 0.02 | 0.04 | 0.06 | 0.08 |
|---|---|---|---|---|---|
| 1.0 | 1.12 | 1.11 | 1.09 | 1.08 | 1.07 |
| 1.5 | 0.81 | 0.79 | 0.78 | 0.78 | 0.77 |
| 2.0 | 0.64 | 0.63 | 0.62 | 0.61 | 0.60 |
| 2.5 | 0.56 | 0.54 | 0.52 | 0.50 | 0.50 |
| 3.0 | 0.51 | 0.49 | 0.47 | 0.46 | 0.44 |

Soft chamber, the bench fill stepped to 5 atm, overshoot in MPa:

| Kp \ Kd | 0 | 0.02 | 0.04 | 0.06 | 0.08 |
|---|---|---|---|---|---|
| 1.0 | 0.70 | 0.70 | 0.69 | 0.69 | 0.69 |
| 1.5 | 0.47 | 0.47 | 0.47 | 0.47 | 0.47 |
| 2.0 | 0.36 | 0.36 | 0.36 | 0.36 | 0.36 |
| 2.5 | 0.30 | 0.30 | 0.30 | 0.30 | 0.30 |
| 3.0 | 0.25 | 0.25 | 0.25 | 0.25 | 0.25 |

The grid agrees with the linear model on each point. The dip follows
Kp and hardly Kd. The soft overshoot does not move with Kd at all; it
falls with Kp, but Kp 3 is unstable in the stiffest chamber in the
linear model, Kp 2.5 has 0.1 dB of gain margin there and 33° at the
design chamber, and Kp 2 has 42°. The stiff step is where Kd shows,
0.29 to 0.03 MPa at Kp 1.5, because that step is the stop law running
into the measurement lag, which a rate term anticipates. The bench has
not shown that overshoot (see the anti-windup study), and the command
activity at rest rises from 0.0002 to 0.0005 mm/s per scan.

**The candidate gain sets** the linear model selects, in the same
cases (the cycle with the hose opened before the compression; "clear"
adds eq. (52) at 0.02 MPa):

| Set | Kp / Ki / Kd | Gain margin, stiffest | 5 atm step: overshoot / in band | 20 atm overshoot | Dip / back in band | Closing peak | Stiff step overshoot |
|---|---|---|---|---|---|---|---|
| P | 1.94 / 0 / 0 | 2.5 dB | does not arrive | does not arrive | 0.70 / never | 10.00 | 0.39 |
| PI, GVL | 1.5 / 0.6 / 0 | 4.6 dB | 0.47 / 72 s | 0.53 | 0.81 / 5.6 s | 10.84 | 0.29 |
| PI, GVL, clear | 1.5 / 0.6 / 0 | 4.6 dB | 0.02 / 34 s | 0.05 | 0.81 / 5.6 s | 10.24 | 0.29 |
| PI at the limit | 1.85 / 0.74 / 0 | 2.7 dB | 0.39 / 66 s | 0.37 | 0.68 / 5.0 s | 10.70 | 0.38 |
| PI at the limit, clear | 1.85 / 0.74 / 0 | 2.7 dB | 0.02 / 34 s | 0.05 | 0.68 / 5.1 s | 10.22 | 0.38 |
| PI, Ki/Kp 0.1 | 1.92 / 0.19 / 0 | 2.5 dB | 0.56 / 70 s | 0.11 | 0.69 / 17.1 s | 10.50 | 0.39 |
| PI, Ki/Kp 1.6 | 1.55 / 2.48 / 0 | 3.9 dB | 0.24 / 64 s | 0.54 | 0.72 / 1.3 s | 10.74 | 0.35 |
| PI, damped at 0.03 MPa/mm | 1.5 / 0.034 / 0 | 4.7 dB | 0.38 / not in 100 s | 0.34 | 0.87 / never | 10.23 | 0.27 |
| PID, 20 ms filter | 2.24 / 0.90 / 0.034 | 1.0 dB | 0.33 / 48 s | 0.25 | 0.57 / 4.6 s | 10.58 | 0.15 |
| PID, 20 ms filter, clear | 2.24 / 0.90 / 0.034 | 1.0 dB | 0.02 / 33 s | 0.04 | 0.57 / 4.6 s | 10.22 | 0.15 |
| PID, 4 ms filter | 2.66 / 1.06 / 0.053 | 1.0 dB | 0.28 / 47 s | 0.17 | 0.49 / 4.1 s | 10.49 | 0.12 |

**A smaller integral gain does not cure the soft overshoot either.**
The linear step of the first table is not what the machine does: the
approach is long, the output is at the stop-law floor for most of it,
and the integral arrives holding that floor, 2 mm/s, whatever the
gains. From there eq. (61) gives the overshoot, 0.35 to 0.54 MPa at the
GVL gains for 0.03 to 0.1 MPa/mm (the simulation gives 0.47 and the
bench 0.35 from a crossing at 1.43 mm/s, for which the equation gives
0.25 to 0.39). The overshoot grows as Ki falls, as sqrt(k / Ki): the
loop with Ki/Kp of 0.1 overshoots by 0.56 MPa, and the one damped to
0.7 by 0.38 MPa without coming into band in 100 s.

**Why clearing the integral is the right remedy.** In the soft chamber
the loop has two states, the error and the velocity the integral holds,
eq. (59). When the error crosses zero the whole state is in the
integral, and the chamber is sealed, so its equilibrium is zero error
with zero integral: clearing the integral there puts the loop at rest,
and nothing is left to ring. With the outlet open the equilibrium is
the outflow velocity, held by the integral at zero error, which is why
the clear must act only once the pressure is past the target (the
piston is then moving faster than the outflow), and never at the
target itself.

**What this means for the machine.**
- The loop is a PI controller. The delay allows a proportional gain of
  1.85 in the hose-shut chamber, and Ki/Kp of 0.4 is the trade between
  the delivery's recovery (5 s) and the margins. The GVL's Kp 1.5 and
  Ki 0.6 are 81 % of that: 52° and 9.6 dB at the design chamber and
  4.6 dB in the stiffest. The set at the limit takes the delivery's
  dip from 0.81 to 0.68 MPa and the closing peak from 10.84 to
  10.70 MPa, for a gain margin of 2.7 dB in the stiffest chamber and a
  stiff step that overshoots by 0.38 MPa in place of 0.29.
- It has no derivative term. The term is worth 21 % of gain for most of
  the stiffest chamber's gain margin and 1.7 times the command noise,
  2 % at an equal gain margin, and nothing in the soft chamber.
- No fixed linear compensator covers the chamber: a gain set keeps its
  phase margin over a stiffness range of about 18 times its integral
  time, 46 to 1 at the limit, and the chamber spans 4500 to 1. The
  soft end needs either a gain that follows the stiffness, which the
  slave cannot measure ahead of the approach, or the integral cleared
  past the target, eq. (52), which needs no stiffness and takes the
  5 atm overshoot from 0.47 to 0.02 MPa. The block has it, behind
  `G_cfgPressureIntegClearEnable`.

## Outflow feedforward study

`study_feedforward.py` tests a velocity feedforward for the outflow
that is switched by the measured pressure, eqs. (64) to (66), against
the block alone, the block with its integral clear, and the same
feedforward told of the valve by the simulation (the bound of the
nominal results above). The feedforward is sized once, for the
measured orifice bank: 0.440 mm/s per sqrt(MPa), 1.39 mm/s at 10 MPa.
The detector's defaults are a 20 ms window, d_on 0.03 MPa, the error
between 0.03 and 1 MPa, held for 8 ms; and for the release 0.02 MPa
past the target with a rise of 0.05 MPa over the window, or 0.3 MPa
past it.

**Nominal cases**, hose opened before the compression unless stated
(the opening dip; the time from the first opening to the pressure back
in band; the peak after the purge closes the valve; the margin over
saturation):

| Case | Arm | Dip MPa | Back in band s | Closing peak MPa | Margin MPa |
|---|---|---|---|---|---|
| 1:1 at 10 MPa | GVL | 0.81 | 5.6 | 10.84 | 5.6 |
| | GVL + clear | 0.81 | 5.6 | 10.23 | 5.6 |
| | valve signal + clear (bound) | 0.35 | 0.25 | 10.14 | 6.1 |
| | detector + clear | 0.42 | 0.26 | 10.28 | 6.0 |
| | detector + clear, short filter | 0.41 | 0.27 | 10.21 | 6.0 |
| 2.5:1 at 10 MPa | GVL + clear | 0.81 | 5.6 | 10.23 | 0.15 (degasses) |
| | valve signal + clear (bound) | 0.35 | 0.25 | 10.14 | 0.61 |
| | detector + clear | 0.42 | 0.26 | 10.28 | 0.54 |
| 2.5:1 at 11 MPa | GVL + clear | 0.85 | 5.7 | 11.25 | 1.1 |
| | detector + clear | 0.43 | 0.27 | 11.30 | 1.5 |
| 1:1 at 5 MPa | GVL + clear | 0.57 | 4.8 | 5.17 | 0.9 |
| | detector + clear | 0.35 | 0.24 | 5.24 | 1.1 |
| 1:1, valve opened over 0.5 s and shut over 1 s | GVL + clear | 0.79 | 5.8 | 10.05 | 5.7 |
| | valve signal + clear (bound) | 0.23 | 0.26 | 10.03 | 6.2 |
| | detector + clear | 0.35 | 0.08 | 10.34 | 6.1 |

**The pressure is enough to switch it.** The detector is on 20 ms
after the first opening, where the tube takes its volume as a step, and
36 ms after the reopening, where there is only the outflow's 4.5 MPa/s,
and off 36 ms after the closing. The dip is 0.42 MPa against the
bound's 0.35 and the block's 0.81, and the pressure is back in band in
a quarter of a second in place of five and a half. That is 0.4 MPa of
margin over saturation, which is what 2.5:1 at 10 MPa lacks without it.
The closing peak is the integral clear's, 0.05 MPa higher: with the
feedforward carrying the outflow the integral holds little, and the arm
without the clear gives the same figures. With the hose opened after
mixing the fall of 5.7 MPa as the hose joins the chamber does not
switch it on: the error leaves the range of an outflow within a scan,
before t_on has run. In a compression, in the soft chamber stepped to
5, 20 and 100 atm, and with the target taken down 2 MPa and back, it
never switches on.

**Settings** (one at a time, on the 1:1 delivery with both hose
procedures and with the slow valve):
- The trigger must reach the reopening, which has no tube step:
  d_on of 0.05 MPa finds the abrupt reopening 8 ms later and misses
  the slow one, and 0.1 MPa misses both (a dip of 0.8 MPa again);
  0.02 MPa is the same as 0.03 on the abrupt valve and finds the slow
  one sooner (0.26 MPa of dip against 0.35).
- t_on is what keeps the hose's joining out: with t_on of 0 it
  switches on there once, falsely. 16 ms costs 0.02 MPa of dip.
- The release needs the rise. On the error alone (d_off of 0) a
  feedforward sized 30 % high leaves the pressure above the target
  and the detector switches off and on some 200 times in a delivery;
  with the rise it stays on. A feedforward 30 % low or high gives 0.43
  and 0.41 MPa of dip, and 30 % high one false switching after the
  closing (a peak of 10.36 MPa). At 50 % high the error passes e_hi
  and it switches as often again: the sizing must be within
  e_hi Kp / v_ff, a third.
- A window of 12 ms releases late (a peak of 10.41 MPa) and misses the
  slow reopening; 40 ms is the same as 20 ms on the abrupt valve.
- Sensor noise of 4.6, 10 and 30 kPa rms changes nothing: the trigger
  is fifteen times the measured 1.9 kPa.
- A slow hand on the closing is the one case the feedforward makes
  worse: it stays on while the outflow fades, and the pressure peaks
  at 10.34 MPa against the block's 10.05 before e_hi releases it.
  e_hi of 0.15 MPa brings that to 10.24 MPa and halves the sizing
  error the detector tolerates.

**The shorter filter.** The pressure input's noise is 1.87 kPa rms per
sample, white and without spikes (`bench/pressure/README.md`,
section 5), and none of the readers of the filtered pressure needs the
20 ms low-pass at that level. A median of 3 with a 5 ms low-pass
leaves 0.95 kPa rms in place of 0.59 and takes 19 ms out of the loop:
the linear model's phase margin at the GVL gains goes from 52° to 63°
in the hose-shut chamber, and the largest Kp with 45° and 6 dB from
1.85 to 2.71. At the GVL gains the nominal results hardly move (a dip
of 0.79 MPa and a closing peak of 10.19 MPa with the clear; 0.41 and
10.21 with the detector, which is on 24 ms after the reopening): the
shorter filter is margin, and room for gain, and it is not needed for
the detector.

**Monte Carlo** (`--mc 100`: 100 Latin hypercube samples over the
ranges of `robust.py`, hose opened before the compression unless
stated, the feedforward sized for the nominal orifice while Cd runs
from 0.8 to 0.9; pass rate for every requirement in percent, the
opening dip at p50 / max and the margin over saturation at p50 / min,
in MPa):

| Case | GVL + clear | valve signal + clear (bound) | detector + clear | GVL + clear, short filter | detector + clear, short filter |
|---|---|---|---|---|---|
| 1:1 at 10 MPa | **100** | **100** | **100** | **100** | **100** |
| 2:1 at 10 MPa | **100** | **100** | **100** | **100** | **100** |
| 2.5:1 at 10 MPa | 0 | 73 | 50 | 0 | 64 |
| 2.5:1 at 11 MPa | **100** | **100** | **100** | **100** | **100** |
| 1:1 at 10 MPa, hose after mixing | 69 | 69 | 69 | 69 | 69 |
| dip, 1:1 | 0.84 / 0.93 | 0.37 / 0.48 | 0.44 / 0.57 | 0.81 / 0.88 | 0.41 / 0.53 |
| margin, 2:1 | 1.95 / 1.86 | 2.42 / 2.31 | 2.34 / 2.22 | 1.98 / 1.91 | 2.37 / 2.26 |
| margin, 2.5:1 at 10 MPa | 0.12 / 0.03 | 0.59 / 0.48 | 0.51 / 0.39 | 0.15 / 0.08 | 0.55 / 0.43 |
| margin, 2.5:1 at 11 MPa | 1.08 / 0.99 | 1.58 / 1.47 | 1.50 / 1.38 | 1.11 / 1.03 | 1.53 / 1.41 |

In the thousand runs of each detector arm (a compression and a delivery
per sample and case) the feedforward never switched on without flow
through the outlet and never missed an opening: it was on within 40 ms
of each (within 30 ms with the shorter filter). The dip is about half
the block's (0.57 MPa against 0.93 at worst) and the margin over
saturation 0.4 MPa wider.
That margin does not make 2.5:1 deliverable at 10 MPa: it passes half
the samples, and the feedforward told of the valve itself passes 73 %,
because the tube's step and the loop's delay leave too little above a
saturation pressure of 9.0 MPa. At 11 MPa every arm passes and the
feedforward widens the smallest margin from 1.0 to 1.4 MPa. The
closing peak is that of the integral clear, within 0.05 MPa. With the
hose opened after mixing the failures are the opening drop's, which no
arm touches.

**What this means for the machine.**
- A feedforward for the outflow needs no valve signal: the fall of
  the measured pressure switches it on within five to ten scans and
  its rise past the target switches it off, with no false switching
  over the measured ranges. It halves the opening dip (0.44 MPa
  against 0.84 at p50) and returns the pressure to the band in a
  quarter of a second.
- What it buys is margin over saturation at an opening, 0.4 MPa, and
  with it a higher charge at a given target or a lower target at a
  given charge. It does not replace the 11 MPa target for 2.5:1.
- It is sized for one orifice bank, within a third; a bank that
  passes less than a third of the present flow falls under the
  trigger. A slow hand on the closing is the case it makes worse, by
  0.3 MPa of peak.
- The 20 ms pressure filter is not needed for the noise: a median of
  3 and 5 ms give 11° more phase margin at the GVL gains and room to
  raise Kp from 1.85 to 2.7 at the same margins, with every result
  above unchanged or slightly better. The gains have not been raised
  in any run here.
- The detector and the feedforward are not in the slave.

## The loop as configured

The GVL carries what the bench settled on, which is not the loop the
studies above were computed with:

| Setting | Studies (`STUDY_CFG`) | GVL |
|---|---|---|
| `G_cfgPressureKp` | 1.5 | 2.0 |
| `G_cfgPressureKi` | 0.6 | 0.8 |
| `G_cfgPressureIntegClearEnable` | off | on |
| `G_cfgAnalogMedianSize` | 5 | 3 |
| `G_cfgAnalogFilterTimeConst` | 20 ms | 5 ms |

`--set as_configured=True` runs a study with the GVL's values, and the
`configured` arm of `study_robust.py` is the same loop. The model's
drive is the one measured with `Y_DirectControl`'s command filter at
0.02 s (18.2 ms of dead time and a 7.5 ms lag); the GVL has that filter
at zero, where the drive has not been measured, so the delay in these
figures is at least the machine's.

**Margins** (`study_linear.py --set as_configured=True`):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL + hose, hose open | 3.2 | 16.8 dB | 72° | 6.4 | 14.0 |
| liquid, 115 mL dead volume (limit switch), hose open | 3.8 | 15.4 dB | 70° | 7.6 | 11.8 |
| liquid, 343 mL, hose shut | 7.5 | 9.5 dB | 56° | 14.8 | 6.0 |

The largest Kp with 45° and 6 dB in the hose-shut chamber is 2.7
(1.85 with the studies' filter), so Kp 2.0 is 74 % of the limit, where
the studies' 1.5 was 81 % of theirs.

**Nominal delivery** (`study_delivery.py --set as_configured=True`,
hose opened before the compression): the opening dip is 0.62 MPa, the
peak after the purge closing 10.19 MPa, and the margin over saturation
5.8 MPa at 1:1, 2.2 MPa at 2:1 and 0.3 MPa at 2.5:1, against 0.81 MPa,
10.84 MPa and 5.6, 2.0 and 0.15 MPa for the studies' loop. With the
clear on, the floor that follows the outflow adds nothing: its arm
gives the same figures.

A compression with the hose shut overshoots by 0.02 MPa, and the stiff
step from 10 to 12 MPa by 0.12 MPa (`study_compare.py --set
as_configured=True`; 0.04 and 0.29 MPa for the studies' loop).

**Monte Carlo** (`study_robust.py --configs GVL,configured --n 100`
with the charge, target and procedure of each row; pass rate for every
requirement in percent, the closing peak at p50 / max and the margin
over saturation through the delivery at p50 / min, in MPa):

| Case | Studies' loop: pass, peak, margin | As configured: pass, peak, margin |
|---|---|---|
| 1:1 at 10 MPa, hose after mixing | 69, 10.9 / 11.0, 0.69 / −0.20 | 69, 10.2 / 10.3, 0.69 / −0.20 |
| 1:1 at 10 MPa, hose before | 100, 10.9 / 11.0, 5.61 / 5.52 | 100, 10.2 / 10.3, 5.80 / 5.73 |
| 2:1 at 10 MPa, hose before | 100, 10.9 / 11.0, 1.95 / 1.86 | 100, 10.2 / 10.3, 2.15 / 2.07 |
| 2.5:1 at 11 MPa, hose before | 100, 11.9 / 12.0, 1.08 / 0.99 | 100, 11.2 / 11.3, 1.29 / 1.21 |

The configured loop passes wherever the studies' loop does, with the
closing peak 0.7 MPa lower and 0.2 MPa more margin over saturation at
the openings; at 2.5:1 and 11 MPa the peak is 0.7 MPa under the
ceiling in place of at it. With the hose opened after mixing the
failures are the opening drop's, as before.

The bench's traces at these settings are under "On the machine" in
"The integral clear in the slave".
