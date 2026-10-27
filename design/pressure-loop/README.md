# Pressure loop design study

This directory holds a Python model of the slave's pressure loop and the
studies run on it. The studies were used to choose between closing the
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
| `--set key=value` | Overrides a `Params` field (`K=2.2e9`, `solver=heun`) or a slave setting by its GVL name (`G_cfgPressureKp=1`). The option can be repeated. |
| `--out DIR` | Sets where the CSV traces and PNG plots go. The default is `out/`, which git ignores. |
| `--legacy` | Uses the earlier plant and sensor: a fixed liquid volume, a single slew limit, and the median and first-order filter applied directly to pressure. |
| `--workers N`, `--seed N` | Set the number of processes (default one per CPU) and the sensor noise seed. |

`design/generated/` holds the slave blocks translated by `tools/st2py.py`.
`slave.py` regenerates it when it is missing or older than `slave-src/FB/`.

## Files

| File | Role |
|---|---|
| `params.py` | Defines `Params`: the chamber, oxygen charge, outlet, actuator, sensor and simulation parameters and the requirement limits. The values the slave configures are read from the GVL and can be overridden through `slave_cfg`. |
| `plant.py` | Models the chamber, the outlet and the actuator, equations (1) to (14) and (36) to (37). `Plant.derivatives` is the continuous-time model and `Plant.scan` advances one slave scan. |
| `solver.py` | Holds the fixed-step integration methods, equations (34) and (35). |
| `study_convergence.py` | Measures the integration error against the time step for both methods. |
| `sensor.py` | Models the measurement chain: `SlaveSensor` (transducer, shunt, noise and the slave's filter and scaling blocks) and `LegacySensor`. |
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound), `ScheduledSlaveController` (the same block with its gains and stop law scheduled on an estimate of the chamber stiffness, equations (39) to (41)) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). All share the `reset` and `step` interface. |
| `scenarios.py` | Defines the operating cases `compress`, `deliver`, `cycle` and the `stiffstep` probe. |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces and metrics. |
| `metrics.py` | Computes the loop metrics (overshoot, settling time, ripple, stroke) and the process metrics (pressure ceiling, saturation margin, opening dip, deliverable volume). |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `slave.py` | Puts the translated slave blocks on the import path, regenerating them if stale, and reads the `G_cfg*` defaults from the GVL. |
| `study_compare.py` | Compares velocity and torque command and the slave controller over three scenarios. |
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

A comment such as `eq. (5)` refers to the numbered equations below. The
symbols in the equations match the variable names in the code.

## Model

**Chamber.** A piston of area A compresses a volume that holds saline of
effective bulk modulus K and the oxygen charge, admitted at the supply
pressure. The gas is isothermal and dissolves into the saline according
to Henry's law with a first-order time constant. The chamber's compliance is the
gas term plus the liquid term, so its stiffness dP/dx rises from about
0.002 MPa/mm when gas-rich to 10 to 25 MPa/mm once the gas has dissolved.
The piston position is integrated, and the chamber volume is the initial
volume minus A times the position. The contents fill the chamber, so the
stressed liquid plus the free gas equals that volume. Every run reports
the stroke used, so it can be checked against the axis travel.

The outflow Q removes saline. The saline volume falls with the outflow,
the compliance uses the present saline volume, and the gas dissolved in
the saline leaves in proportion, so the chamber stiffens as it empties:
the liquid-only stiffness A·K/V_l is 11 MPa/mm with 300 mL left and
140 MPa/mm with 25 mL left (K = 1 GPa). The piston cannot sweep the
dead volume of 20 to 30 mL in the passages and fittings; it reaches the
limit switch there, the machine halts, and the delivery ends. The
deliverable volume is therefore the saline volume less the dead volume,
about 275 mL.

**Oxygen charge and saturation.** The charge is given as the ratio of
its STP volume to the saline volume (1:1, 2:1, ...) and is admitted at
the supply pressure, 0.5 to 1.4 MPa, where every run starts. Saline
holds n_sat = kH·(P + Patm)·V_l of oxygen at pressure P, and the
saturation pressure of what is dissolved is P_sat = n/(kH·V_l) − Patm,
eq. (38), about 3.5 MPa per unit of ratio. If the pressure falls below
P_sat the solution degasses, so the pressure must stay above P_sat plus
a margin from the moment the target is reached until the delivery ends.
At 10 MPa the saline holds a 2.9:1 charge; a 3:1 charge leaves free gas
in the chamber and has no margin at all.

**Outlet.** The outlet is a ¼" ball valve, a 1 mm bore tube 300 mm long
and a plate of 32 orifices of 40 µm discharging to atmosphere. The
valve's bore is 450 times the orifice area, so once it cracks the
orifices set the flow, Q = Cd·A_o·√(2P/ρ), eq. (36): 4.0 mL/s at 10 MPa
with Cd 0.7, falling with the square root of the pressure. The tube's
friction loss (about 1 % of the chamber pressure) and inertance (a flow
build-up time under 0.1 ms) are neglected. The tube holds air at
atmosphere before the valve opens; the chamber liquid compresses and
expels it in milliseconds, so the chamber loses the tube's 0.24 mL in
the opening scan, eq. (37). On a liquid-only chamber that is a pressure
step of K·V_tube/V_l, 0.8 MPa at K = 1 GPa, before the controller can
act.

**Velocity actuator** (Y_DirectControl in VelocityTLMode). The bench
measured the drive's response to a velocity command on 2026-10-26
(`bench/friction`, `bench/characterisation`): the motor velocity follows
the command after a dead time of 12 ms with a first-order lag of 8.7 ms,
the same for 0.3 to 2.5 mm/s steps in both directions, and **the command
is not slewed at the configured accel**; the drive accelerates at
whatever rate the step needs. The model therefore applies the dead time
in whole scans and the lag, and has the slew limit of eq. (11) only as an
option (`slew_limited`, off by default). The lag at the piston, behind
the gearbox and screw, was not resolved by the bench (below 25 ms); the
robustness study varies the lag from 5 to 25 ms to cover it. The piston
stalls when the force the drive must supply, the pressure force plus the
load-dependent, Coulomb and viscous friction, reaches the torque limit,
eq. (13): about 20 MPa at 100 % rated torque.

**Friction.** The friction bench measured, at the piston: Coulomb
friction 6.1 kN, viscous friction 2.3 MN s/m (11 kN at 5 mm/s), a
load-dependent term of 0.45 times the pressure force (a drivetrain
efficiency of 0.69 pushing), and a breakaway of about 24 kN. In velocity
mode the drive's own loop absorbs these and they enter the model only
through the stall; in torque mode they act on the reflected mass.

**Torque actuator** (TorqueVLMode). The commanded force drives the
reflected mass of the motor and gearbox against the chamber pressure, the
load-dependent friction and the viscous and Coulomb friction, and the
drive clamps the velocity. The controller always adds the feedforward
force A times the target.

**Sensor and timing.** The chamber gauge pressure becomes a 4-20 mA loop
current over the configured transducer range and a voltage across the
shunt, plus the transducer's zero error (measured −0.11 MPa with the
chamber open) and white noise specified as an RMS pressure (measured
2.0 mV rms on the 434 Ω shunt, 4.6 kPa, broadband; 0.8 mV with a 4.7 µF
capacitor across the shunt). The slave's median
and low-pass block filters the voltage, with its coefficient computed as
`PRG_Main` computes it, and the slave's scaling block converts it to MPa.
`PRG_Main` reads the filter and scaling outputs at the top of the scan
and calls the blocks at the end, so the controller sees the filter output
of two scans earlier. The controller's command takes effect one scan
later, which matches the order in which `PRG_Main` calls its blocks.

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

## Model equations

The equations below are numbered for reference from the code. P is the
chamber gauge pressure [Pa], x the piston position [m] (positive into
the cylinder), v the piston velocity [m/s] and Q the liquid outflow
[m^3/s]. The other symbols are the `Params` fields of the same name.

### Chamber

Absolute pressure:

    (1)  P_abs = P + Patm

Henry's-law equilibrium of dissolved gas, limited to the gas present, and
the dissolution rate:

    (2)  n_eq = min(kH P_abs V_l, n_tot)
    (3)  r_dis = (n_eq - n_dis) / tauDiss

Chamber capacitance, the volume change of the contents per unit pressure
change, from the isothermal free gas and the liquid:

    (4)  C = (n_tot - n_dis) R T / P_abs^2 + V_l / K

Chamber pressure from continuity. The piston adds the volume flow A v;
the outflow and the gas leaving the free phase remove volume:

    (5)  dP/dt = (A v - Q - (R T / P_abs) r_dis) / C

Piston kinematics:

    (6)  dx/dt = v

Liquid removed by the outflow. V_l is the liquid volume at zero gauge
pressure, so the stressed outflow Q removes Q / (1 - P/K) of it:

    (7)  dV_l/dt = -Q / (1 - P/K)

Dissolved gas carried out with the liquid, and the resulting gas
balances:

    (8)  r_out = -(n_dis / V_l) dV_l/dt
         dn_dis/dt = r_dis - r_out,   dn_tot/dt = -r_out,
         dn_out/dt = r_out

Volume closure. The chamber volume equals the stressed liquid plus the
free gas; dissolved gas takes no volume. The model does not impose this
identity, so it serves as a check on the integration:

    (9)  V_ch0 - A x = V_l (1 - P/K) + (n_tot - n_dis) R T / P_abs

Chamber stiffness:

    (10) k = dP/dx = A / C

### Actuator

Velocity command slew limit (option, off by default: the drive does not
slew), applied once per integration step of length dt. The accel limit
applies while the reference grows in magnitude and the decel limit while
it shrinks or reverses:

    (11) v_ref <- v_ref + clamp(u - v_ref, -a dt, a dt),
         a = accel if (u - v_ref) v_ref >= 0, else decel

Drive velocity loop, with the command u delayed by the dead time t_dead
in whole scans before it becomes the reference:

    (12) v_ref(t) = u(t - t_dead),   dv/dt = (v_ref - v) / tauV

Stall at the torque limit, applied after each step. The drive must
supply the pressure force, the load-dependent friction mu P A, the
Coulomb friction and the viscous friction:

    (13) if P A (1 + mu) + Fc + b v > Fcap and v > 0:  v = 0, v_ref = 0

Torque mode, with the Coulomb and load-dependent friction smoothed over
|v| < veps and the velocity clamped to |v| <= vmax after each step:

    (14) mr dv/dt = F - P A - b v - (Fc + mu P A) clamp(v / veps, -1, 1)

### Sensor

Shunt voltage sampled each scan, with z a standard normal draw and
P_offset the transducer's zero error. The range P_min to P_max is in
MPa, the currents I_zero and I_span in mA and the shunt R_shunt in ohms:

    (15) V_in = R_shunt / 1000 (I_zero + I_span ((P + P_offset)/1e6 - P_min)
                                / (P_max - P_min)) + V_noise z,
         V_noise = noise (dV_in/dP)

First-order filter coefficient, with the time constant tau rounded to
whole milliseconds as `PRG_Main` rounds it:

    (16) alpha = Ts / (tau + Ts)

Median of N samples followed by the first-order filter:

    (17) y[n] = y[n-1] + alpha (median(V_in[n-N+1 .. n]) - y[n-1])

Pipeline delay. The controller in scan n reads the scaled filter output
of scan n - 2:

    (18) P_meas[n] = scale(y[n-2])

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

### Outlet and charge

Orifice flow with the valve area fraction a(t), discharging to
atmosphere; A_o is the total orifice area:

    (36) Q = a(t) Cd A_o sqrt(2 max(P, 0) / rho)

Tube volume lost in the scan the valve opens, applied as an extra
outflow over that scan:

    (37) Q_tube = V_tube / Ts

Saturation pressure of n_dis moles of oxygen dissolved in V_l of saline
(gauge); the charge for an STP ratio r is n = r V_l Patm / (R 273.15 K):

    (38) P_sat = n_dis / (kH V_l) - Patm

### Scheduled slave controller

Stiffness bound from the measured pressure P and piston position x,
with V_ch = A (x_limit − x) + V_dead_cfg the chamber volume (x_limit the
limit-switch position and V_dead_cfg the configured dead volume, a lower
bound), V_l_cfg the configured saline volume (a lower bound), n_cfg the
configured charge, K_cfg the largest plausible bulk modulus and P_ref
the highest absolute pressure seen in the run:

    (39) V_gas_est = max(n_cfg - kH P_ref V_l_cfg, 0) R T / P_abs
         C_est = V_gas_est / P_abs + min(V_l_cfg, V_ch) / K_cfg
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

## Numerical solution

`sim.py` steps the closed loop one slave scan Ts at a time. In each scan
the sensor samples the pressure, the controller computes a command, and
the plant advances with the command computed in the previous scan held
constant. The plant splits the scan into `nsub` integration steps of
length dt = Ts / nsub (5 steps of 0.4 ms by default). Before each step
it applies the slew limit, eq. (11); after each step it applies the stall
or velocity clamp and the physical bounds on the state: the dissolved
gas cannot exceed the gas present, and the absolute pressure cannot fall
below 1 Pa.

The state is y = [x, v, P, n_dis, n_tot, V_l, n_out], and
`Plant.derivatives` returns f(y) = dy/dt from equations (1) to (14).
Forward Euler (`--set solver=euler`) is the same update as `ode1.m`:

    (34) y[i+1] = y[i] + dt f(y[i])

The default method is Heun's, which averages the slope at the start of
the step and at the Euler prediction:

    (35) y_p = y[i] + dt f(y[i]),
         y[i+1] = y[i] + (dt/2) (f(y[i]) + f(y_p))

**Convergence.** `study_convergence.py` compares both methods at 1 to 50
steps per scan with a reference of Heun at 100 steps per scan
(dt = 0.02 ms). The largest pressure difference from the reference, with
the slave controller:

| Scenario | Euler, nsub 1 / 5 / 50 | Heun, nsub 1 / 5 / 50 |
|---|---|---|
| compress | 65 / 13 / 1.4 kPa | 9.0 / 1.7 / 0.07 kPa |
| stiffstep | 10.5 / 2.0 / 0.10 kPa | 10.5 / 2.0 / 0.11 kPa |
| discharge | 0.29 / 0.05 / 0.02 kPa | 0.04 / 0.04 / 0.02 kPa |
| valve | 0.16 / 0.03 / 0.01 kPa | 0.02 / 0.01 / 0.01 kPa |

(The table was made with the earlier gas-rich compress from atmosphere
and prescribed-flow discharge and valve cases; the present cases are
less demanding, and `test_model.py` checks that the two methods agree to
5 kPa on them.) Euler converges at first order, halving its error with
each halving of the step. At the default of five Heun steps per scan
(dt = 0.4 ms) the largest error is 1.7 kPa, during the gas-rich
compression, and the metrics change by less than 0.001 MPa of overshoot
and 0.002 s of settling time between the default and the reference. On stiffstep both
methods give the same error, because the slew limit, eq. (11), is
applied once per step and converges at first order whatever the
integrator; this is the behaviour of the drive's own discrete profile
generator. Heun at five steps costs about 15 % more than Euler at five
and is eight times more accurate on compress, which is why it is the
default. The volume closure, eq. (9), holds to within 2e-9 m^3 over 6 s
of compression.

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
| dead volume | 45–55 mL | linear | measured 50 mL |
| tauDiss | 10–100 s | log | not measured |
| tauV | 5–25 ms | log | 8.7 ms at the motor; below 25 ms at the piston |
| sensor noise | 1.5–5 kPa rms | log | measured 1.8 (with capacitor) to 4.6 kPa |
| orifice Cd | 0.8–0.9 | linear | measured 0.84–0.87 |
| supply pressure | 0.52–1.38 MPa | linear | 75–200 psi with the cylinder's charge |

The ranges are the bench measurements of 2026-10-26 with their spread
(`bench/characterisation/README.md` section 7, `bench/friction`,
`bench/pressure`). `mr`, `b`, `Fc` and `mu` act only in torque mode and
the stall, so they are not varied; the dead time is fixed at its
measured 12 ms. Each
sample runs compress and deliver and checks these requirements:

- The pressure never exceeds the ceiling, 12 MPa (`Params.P_ceiling`).
  The machine's mechanical limit is to be confirmed; 15 MPa may break
  the piston.
- The pressure never falls below the saturation pressure plus the
  margin, 0.5 MPa (`Params.degas_margin`), from the moment the target is
  reached until the delivery ends. This is the no-degassing requirement,
  and the margin sets where the trade-off between charge and deliverable
  volume is read.
- Control is kept to the end of the delivery: once the opening dip has
  had 5 s to recover, the error never exceeds 1 MPa (`Params.e_crash`),
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
are the GVL (which now carries the 20 ms filter); the GVL with
`StopVelMin` 2 mm/s so the stop law passes the 1.4 mm/s the measured
outflow needs; the scheduled controller at wc = 6, 11 and 16 rad/s with
that setting and the measured bounds (K_cfg 1.0 GPa, dead volume
45 mL); and wc = 11 with VelMax 7 mm/s. Every configuration sees the
same samples and noise seeds.
`--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine cannot
supply the flow, so no built-in configuration uses it.

## Measured values

The bench tests of 2026-10-26 (`bench/`, branch `bench-tests`) replaced
the guessed parameters. The values the model now carries, with what was
assumed before:

| Parameter | Measured | Assumed before | Where |
|---|---|---|---|
| Effective bulk modulus K | 0.72 GPa (0.67–0.78; 0.81 gas-corrected) | 1 GPa | `charBulkModulus`, 3 runs at a 343 mL fill |
| Saline volume Vl0 | 343 mL at the 280 mm start, including the dead volume | 300 mL | geometry |
| Dead volume | 50 mL | 20–30 mL (then 10 mL in the master) | 50 g of water fills it |
| Orifice Cd | 0.85 (0.84–0.87), exponent 0.48; 4.8 mL/s at 10 MPa | 0.7; 4.0 mL/s | `charOrificeFlow`, deliveries at 2, 5 and 10 MPa |
| Drive response | 12 ms dead time, 8.7 ms lag at the motor, no slew | 20 ms lag, slewed at the configured accel | `stepResponseFit`, 74 step captures |
| Lag at the piston | below 25 ms (not resolved) | — | repeat with the pressure probe |
| Coulomb friction Fc | 6.1 kN | 0.5 kN | `frictionFit` |
| Viscous friction b | 2.3 MN s/m | 5 kN s/m | `frictionFit` |
| Load-dependent friction mu | 0.45 (efficiency 0.69 pushing) | 0.85 efficiency | `frictionFit` |
| Breakaway | about 24 kN | — | `frictionFit` |
| Sensor noise | 2.0 mV rms (4.6 kPa) bare, 0.8 mV (1.8 kPa) with 4.7 µF; broadband | 5 kPa | `pressureNoiseProbe` |
| Transducer zero | −0.11 MPa with the chamber open | 0 | every log |

Still to measure: the dissolution time constant tauDiss (the hold time
before delivery depends on it) and the lag at the piston. The GVL on the
machine now has the 20 ms filter, a median of 5, `G_cfgVelLimitMax`
7 mm/s and `G_cfgDirectCtrlAccel` 10 mm/s² (which the drive does not
apply).

## Results with the measured parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa·s (Ki). "GVL" is the
slave's `FB_PressureController` as configured on the machine after the
bench tests: Kp 0.5, Ki 0.2, VelMax 5 mm/s, accel and decel 50 mm/s²
(which the drive does not apply), stop-law stiffness 100 MPa/mm,
StopVelMin 0.5 mm/s and the 20 ms filter. The time-domain results use
Heun's method at five steps per scan and the measured plant: K 0.72 GPa,
343 mL of saline with 50 mL of dead volume, Cd 0.85, a 12 ms dead time
and 8.7 ms lag in the drive, and the −0.11 MPa transducer offset, which
puts the true pressure 0.11 MPa above every target.

**Velocity versus torque command.** Velocity command wins, now more
clearly. The drive's own velocity loop absorbs the 6 kN of Coulomb and
2.3 MN s/m of viscous friction and the mass-spring resonance, so the
pressure loop sees an integrator behind a short lag. In torque command
the measured friction is a dead band of 6 MPa and more at pressure, so a
force command cannot place the pressure at all. Torque remains only as
the stall cap, about 20 MPa at 100 % rated torque.

**Compression** (1:1 charge from a 1 MPa supply): the piston sweeps
11 mm, and the GVL loop arrives in band after 4.4 s with 0.56 MPa of
overshoot; the scheduled loop at wc = 11 rad/s arrives in 2.4 s with
0.11 MPa, which is the transducer offset, not an overshoot of the
reading. The chamber stiffness rises from 0.11 to 6.5 MPa/mm. The legacy
controllers without the configured stop law overshoot to 13–20 MPa; the
stop law is what makes the approach safe.

**Delivery with the gains on the machine.** The orifice bank passes
4.8 mL/s at 10 MPa (the bench measured it; the deliveries ran 20 %
faster than the earlier model), which needs the piston at 1.4 mm/s, and a
delivery of the 288 mL above the dead volume takes 60 s. With the GVL
settings the loop sits low for the whole delivery: the tube's air drops
the pressure 0.5 MPa in the opening scan, the outflow pulls it to
7.9 MPa, and the stop law, which with 100 MPa/mm and 50 mm/s² admits the
1.4 mm/s the outflow needs only at an error of about 2 MPa, holds the
integrator through its anti-windup. The bench saw this: the loop held
8.1 MPa for a 10 MPa target. Raising `StopVelMin` to 2 mm/s removes it,
and because the dead volume caps the end stiffness at 49 MPa/mm
(K 0.72 GPa, 50 mL) the GVL gain keeps 26° of phase margin and 4 dB of
gain margin at the limit switch, so the whole 288 mL is delivered with
the pressure in band at nominal parameters.

**Scheduled controller.** `ScheduledSlaveController` (equations (39) to
(41)) with the measured bounds (K_cfg 1.0 GPa, dead volume 45 mL) holds
its margins constant through the run, 69° / 16 dB at wc = 6 rad/s,
56° / 11 dB at wc = 11 and 44° / 8 dB at wc = 16, and because the
measured chamber is soft (7.2 MPa/mm against the 11–25 assumed) it can
use a higher gain at the valve opening than before. Nominal results,
1:1, all with StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / V_ok mL |
|---|---|---|
| GVL as configured | 4.4 / 0.56 | 2.07 / 4.4 / 22 (steady error) |
| GVL, StopVelMin 2 | 4.3 / 0.63 | 2.07 / 4.4 / 288 (all) |
| scheduled, wc 6 | 3.2 / 0.13 | 1.84 / 4.6 / 288 (all) |
| scheduled, wc 11 | 2.4 / 0.11 | 1.14 / 5.3 / 288 (all) |
| scheduled, wc 11, Ki = Kp, VelMax 7 | 2.0 / 0.11 | 1.11 / 5.3 / 288 (all) |
| scheduled, wc 16 | 2.2 / 0.11 | 0.94 / 5.5 / 288 (all) |
| flow feedforward (bound, not implementable) | — | 0.59 / 5.9 / 288 |

(The 0.11 MPa "overshoot" of the scheduled arms is the transducer
offset.) The scheduled arms compress twice as fast as the GVL with no
overshoot of the reading, because the stop law brakes for the estimated
stiffness and the whole lag (measurement, dead time and drive) and the
integrator is held until arrival, and they halve the opening dip,
because the gain at the opening is set by the measured stiffness rather
than a fixed worst case. VelMax 7 mm/s shortens the compression by
0.4 s and does not change the dip; the drive does not slew, so accel
and decel settings do nothing.

**The opening dip is what limits the charge.** The dip is the 0.5 MPa
tube step plus the quasi-static Q/(A·Kp) of eq. (33), recovered over the
integral time: 2.07 MPa at Kp 0.5, 1.14 at wc 11 (Kp 1.5 at 7.2 MPa/mm),
0.94 at wc 16. The saturation pressure is 3.7 MPa per unit of ratio in
saline, so each megapascal of dip costs about 0.27 in the ratio, and a
higher target buys 0.27 per megapascal the other way. Volume delivered
before a violation at nominal parameters, 0.5 MPa margin:

| Charge | P_sat MPa | GVL | GVL, StopVelMin 2 | sched wc 6 | sched wc 11 | sched wc 16 |
|---|---|---|---|---|---|---|
| 1:1 | 3.6 | 22 mL | 288 | 288 | 288 | 288 |
| 2:1 | 7.2 | 22 | 288 (margin 0.7) | 288 (0.9) | 288 (1.6) | 288 (1.8) |
| 2.5:1 at 10 MPa | 9.0 | 0 | 0 (−1.1) | 0 (−0.9) | 0 (−0.2) | 0 (0.0) |
| 2.5:1 at 11 MPa | 9.0 | 0 | 0 (−0.2) | 0 (0.0) | 288 (0.8) | 288 (1.0) |
| 3:1 at 11 MPa | 10.9 | 0 | 0 | 0 | 0 (−1.0) | 0 (−0.9) |

So 2:1 is deliverable at 10 MPa by every arm that clears the stop law,
2.5:1 needs the scheduled controller and an 11 MPa target (the true
pressure then peaks at 11.1 MPa, under the 12 MPa ceiling), and 3:1 is
saturated at 10.9 MPa and out of reach below the ceiling. The target
is a stronger lever than the gains: `--set P_target=11e6` runs any
study at another target.

**Robustness** (100 Latin hypercube samples over the measured ranges;
pass rate for every requirement, in percent):

| Charge, target | GVL | GVL, StopVelMin 2 | sched wc 6 | sched wc 11 | sched wc 11, VelMax 7 | sched wc 16 |
|---|---|---|---|---|---|---|
| 1:1 at 10 MPa | 0 | **100** | **100** | **100** | **100** | **100** |
| 2:1 at 10 MPa | 0 | **100** | **100** | **100** | **100** | **100** |
| 2.5:1 at 10 MPa | 0 | 0 | 0 | 0 | 0 | 0 |
| 2.5:1 at 11 MPa | 0 | 0 | 0 | 93 | 93 | **100** |

The GVL as configured fails every sample on control (the steady error).
Every other arm passes every sample at 1:1 and 2:1: with the measured
chamber the range of plausible parameters is narrow enough that nothing
in it threatens the ceiling, the saturation margin or the end of the
delivery, and the uncertain parameters rank tauDiss > Vl0 > P_supply >
V_dead > Cd > K > tauV on the key metrics, all with swings well inside
the limits. At 2.5:1 the 10 MPa target leaves 1.0 MPa to saturation, less
than the dip of any arm, and fast dissolution during the hold also takes
a fifth of the samples inside the 0.5 MPa margin; at 11 MPa the scheduled
controller at wc = 16 passes every sample and wc = 11 passes 93 %, the
failures being the stiffest chambers with the noisiest transducer. The
true pressure peaks at 11.1 MPa, under the ceiling.

**What this means for the machine.**
- Raise `G_cfgPressureStopVelMin` to 2 mm/s. That alone removes the
  steady error the bench saw (8.1 MPa for a 10 MPa target) and delivers
  1:1 and 2:1 at 10 MPa in every plausible case with the gains as they
  are.
- The scheduled controller halves the opening dip (1.1 MPa at wc = 11
  against 2.1) and compresses twice as fast with no overshoot; it is
  what makes 2.5:1 reachable, at an 11 MPa target. Its margin is the
  same at every point of the run, where the fixed gain's falls to 26°
  at the limit switch.
- 3:1 is saturated at 10.9 MPa and cannot be delivered without bubbles
  below the 12 MPa ceiling by any controller.
- The drive does not apply the accel and decel settings, so they are not
  tuning levers; VelMax 7 mm/s shortens the compression only.

**Margins against stiffness** (linear model, GVL gains, 20 ms filter,
12 ms dead time):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL, K = 0.72 GPa | 7.2 | 20.8 dB | 73° | 3.6 | 5.6 |
| liquid, 120 mL | 20 | 12.0 dB | 59° | 9.8 | 2.0 |
| liquid, 50 mL dead volume (limit switch) | 49 | 4.2 dB | 26° | 22 | 0.81 |
| stop-law stiffness | 100 | −2.0 dB | −14° | 37 | 0.40 |

The GVL gain is stable to the limit switch but with a quarter of its
starting margin; a dead volume of 35 mL or a bulk modulus of 1 GPa would
take it to the edge. The scheduled gain Kp = wc/k holds 56° at wc = 11
everywhere.

**Open questions:**
- The mechanical pressure ceiling: 12 MPa is assumed; 15 MPa may break
  the piston. With the true pressure 0.11 MPa above the reading, an
  11 MPa target peaks at 11.1 MPa.
- The transducer zero: correcting the −0.11 MPa in the slave's scaling
  (`G_cfgPressureRangeMin`) would make the reading true; until then
  every target is 0.11 MPa conservative for degassing and the same
  amount optimistic for the ceiling.
- The lag at the piston (below 25 ms) and the dissolution time constant
  are the two unmeasured plant parameters; the robustness study covers
  the first, and the second sets only the hold time.
- Whether the saline volume and the charge can be given to the slave
  (as configuration or over the protocol), which the schedule's
  stiffness bound needs.
