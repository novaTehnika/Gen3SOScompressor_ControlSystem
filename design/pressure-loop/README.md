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
110 MPa/mm with 30 mL left (K = 1 GPa).

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

**Velocity actuator** (Y_DirectControl in VelocityTLMode). The drive
limits the rate of change of the commanded velocity. The accel limit
applies while the command grows in magnitude and the decel limit while it
shrinks or reverses; the controller supplies both, as
`G_cfgPressureAccel` and `G_cfgPressureDecel` do on the slave. The
drive's velocity loop follows the limited command with a first-order lag,
and the piston stalls when the chamber force reaches the torque limit.

**Torque actuator** (TorqueVLMode). The commanded force drives the
reflected mass of the motor and gearbox against the chamber pressure,
viscous friction and Coulomb friction, and the drive clamps the velocity.
The controller always adds the feedforward force A times the target.

**Sensor and timing.** The chamber gauge pressure becomes a 4-20 mA loop
current over the configured transducer range and a voltage across the
shunt, plus white noise specified as an RMS pressure. The slave's median
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
has: the measured pressure, the piston position against a configured
contact position, and the configured saline volume, charge and bulk
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

Velocity command slew limit, applied once per integration step of length
dt. The accel limit applies while the reference grows in magnitude and
the decel limit while it shrinks or reverses:

    (11) v_ref <- v_ref + clamp(u - v_ref, -a dt, a dt),
         a = accel if (u - v_ref) v_ref >= 0, else decel

Drive velocity loop:

    (12) dv/dt = (v_ref - v) / tauV

Stall at the torque limit, applied after each step:

    (13) if P > (Fcap - Fc) / A and v > 0:  v = 0, v_ref = 0

Torque mode, with the Coulomb friction smoothed over |v| < veps and the
velocity clamped to |v| <= vmax after each step:

    (14) mr dv/dt = F - P A - b v - Fc clamp(v / veps, -1, 1)

### Sensor

Shunt voltage sampled each scan, with z a standard normal draw. The range
P_min to P_max is in MPa, the currents I_zero and I_span in mA and the
shunt R_shunt in ohms:

    (15) V_in = R_shunt / 1000 (I_zero + I_span (P/1e6 - P_min)
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
with V_ch = A (x_contact − x) the chamber volume, V_l_cfg the configured
saline volume (a lower bound), n_cfg the configured charge, K_cfg the
largest plausible bulk modulus and P_ref the highest absolute pressure
seen in the run:

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
(four for N = 5) and one scan for the command:

    (29) L(z) = C(z) z^-1 Gd(z) z^-d_s F(z)

Sensitivity and complementary sensitivity:

    (30) S = 1 / (1 + L),   T = L / (1 + L)

Target to pressure. The target enters at the controller, so the sensor
path does not act on it:

    (31) R = C z^-1 Gd S

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
- `z^-1` is the scan between computing a command and applying it.
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

| Parameter | Range | Sampling |
|---|---|---|
| K | 0.5–2.5 GPa | log |
| Vl0 | 250–350 mL | linear |
| tauDiss | 10–100 s | log |
| tauV | 5–50 ms | log |
| sensor noise | 1–20 kPa rms | log |
| orifice Cd | 0.6–0.8 | linear |
| supply pressure | 0.52–1.38 MPa | linear |

`mr`, `b` and `Fc` act only in torque mode, so they are not varied. Each
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
are the GVL; the GVL with the stop law relaxed (20 ms filter,
`StopVelMin` 2 mm/s); the scheduled controller at wc = 6 and 11 rad/s
with those settings; the scheduled controller at wc = 11 with the
drive's limits (VelMax 7 mm/s, accel and decel 100 mm/s²); and the same
assuming a measured bulk modulus (K_cfg = 1.2 GPa). Every configuration
sees the same samples and noise seeds.
`--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine cannot
supply the flow, so no built-in configuration uses it.

## Assumptions to replace with bench measurements

| Parameter | Value used | How to measure |
|---|---|---|
| Effective liquid bulk modulus K | 1 GPa | Run at 0.1 mm/s in the liquid-only chamber and read dP/dt. |
| Orifice discharge coefficient Cd | 0.7 (4.0 mL/s at 10 MPa) | Collect the volume discharged per unit time at a known chamber pressure. |
| Oxygen solubility in the saline | 5 % below water | Hold a known charge at pressure and read the free gas volume from the piston position. |
| Friction | 5 kN s/m viscous, 500 N Coulomb | Record the torque feedback at 0.1, 0.3, 1, 3 and 7 mm/s in both directions, unloaded and during a delivery. The friction is known to be high and stick-slip at the 1.2 mm/s delivery speed would need a richer drive model. |
| Sensor noise | 5 kPa rms per sample | Log the raw analog input at rest. |
| Drive velocity loop lag | 20 ms | Record a step response in velocity mode. |

## Results with these parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa·s (Ki). "Slave (GVL)" is
the slave's `FB_PressureController` with the GVL configuration: Kp 0.5,
Ki 0.2, VelMax 5 mm/s, accel and decel 50 mm/s², stop-law stiffness
100 MPa/mm, StopVelMin 0.5 mm/s and a 50 ms filter. These are the gains
on the machine. The time-domain results use Heun's method at five steps
per scan.

**Velocity versus torque command.** Velocity command wins. The drive's
own velocity loop absorbs the friction and the mass-spring resonance, so
the pressure loop sees an integrator. In torque command the pressure loop
sees the reflected mass on the stiff chamber, a lightly damped resonance
near 5 Hz behind the sensor filter, and a feedback gain above about
5 percent of A (N per MPa) is unstable. Pure force feedforward holds the
pressure only within the friction dead band of about 0.25 MPa, and only
if the force calibration is exact. Torque therefore remains only as a
force cap inside velocity mode.

**Compression** (1:1 charge from a 1 MPa supply, slave GVL): the piston
sweeps 9 mm, arrives in band after 7.5 s with 0.35 MPa of overshoot, and
the chamber stiffness rises from 0.13 to 10 MPa/mm. The legacy
controllers without the configured stop law overshoot to 13–23 MPa, past
the ceiling; the configured stop law is what makes the approach safe.

**Delivery with the gains on the machine.** The orifice bank passes
4.0 mL/s at 10 MPa, which needs the piston at 1.16 mm/s. The delivery of
300 mL takes 80 s. With the GVL settings a 1:1 delivery fails in two
ways:

1. *A steady error of 1.2 MPa.* The tube's air volume drops the pressure
   0.8 MPa in the opening scan, and the outflow then pulls it down to
   7.8 MPa at 1.2 s. The loop never recovers: the stop law clamps the
   velocity to √(2·Decel·|e|/Stiffness), which with 100 MPa/mm and
   50 mm/s² reaches the 1.16 mm/s the outflow needs only at an error of
   about 1.2 MPa, and the anti-windup holds the integrator while the
   output is clamped. The loop therefore sits at 8.8 MPa for the whole
   delivery. At 1:1 the solution stays 5 MPa above saturation, so it
   does not degas, but the pressure is not what was asked for.
2. *Loss of control as the chamber empties.* The liquid-only stiffness is
   A·K/V_l, 11 MPa/mm with 300 mL and over 100 MPa/mm below 30 mL. The
   GVL loop is unstable above about 80 MPa/mm, so the last 10 mL of
   every delivery end in a growing oscillation that stalls the drive at
   the torque limit, 27 MPa, far above the ceiling.

The stop-law settings alone do not fix either. A 2 mm/s StopVelMin
removes the steady error (the GVL arm delivers 19 mL before the error
exceeds 1 MPa; with StopVelMin 2 it delivers 281 mL) but not the
end-of-delivery instability, and a 20 MPa/mm stop stiffness removes the
steady error but overshoots every compression past the 12 MPa ceiling,
because the filtered reading lags the stiffening chamber on the
approach. With the gains on the machine no configuration passes every
requirement at 1:1 in the robustness study.

**Scheduled controller.** `ScheduledSlaveController` (equations (39) to
(41)) fixes both. Because the loop's crossover is held at or below wc
whatever the stiffness, the linear margins are the same at every point
of a run: with the 20 ms filter, 69° of phase margin and 18 dB of gain
margin at wc = 6 rad/s, and 57° and 13 dB at wc = 11 rad/s, from
0.1 MPa/mm to 1000 MPa/mm. When the estimate overstates the stiffness
the crossover falls and the margin grows, so the bound errs safe; when
the configured bulk modulus is below the true one the crossover rises
in proportion (K_cfg 1.2 GPa against a true 2.5 GPa gives 23 rad/s at
wc = 11, still stable with 34° of phase margin). Nominal results, 1:1,
with the 20 ms filter and StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / V_ok mL |
|---|---|---|
| GVL (for reference) | 3.7 / 0.35 | 2.16 / −3.5 (crash) / 19 |
| scheduled, wc 6 | 5.5 / 0.03 | 3.38 / 3.1 / 292 (all) |
| scheduled, wc 11 | 3.5 / 0.01 | 2.18 / 4.3 / 292 (all) |
| scheduled, wc 11, Ki = Kp | 3.3 / 0.06 | 2.07 / 4.4 / 292 (all) |
| scheduled, wc 11, VelMax 7, accel/decel 100 | 3.1 / 0.01 | 2.18 / 4.3 / 292 (all) |
| scheduled, wc 11, K_cfg 1.2 GPa, VelMax 7, accel/decel 100 | 2.0 / 0.00 | 1.51 / 4.9 / 292 (all) |

The scheduled arms compress with no measurable overshoot, because the
stop law brakes for the estimated stiffness and the measurement lag and
the integrator is held until arrival, and they deliver the whole
chamber with the pressure back in band, because the gain falls as the
chamber empties. The approach is also faster than the GVL's once wc is
11 rad/s. Raising the drive limits to 7 mm/s and 100 mm/s² shortens the
compression (3.5 → 3.1 s) but does not change the opening dip, which is
set by the gain at the moment the valve opens and the integral time, not
by how fast the piston can accelerate.

**The opening dip is what limits the charge.** With feedback alone the
dip is close to the quasi-static Q/(A·Kp) of eq. (33) plus part of the
0.8 MPa tube step: 3.4 MPa at Kp 0.24 (wc 6 against a 2.2 GPa bound),
2.2 MPa at Kp 0.44 (wc 11), 1.5 MPa at Kp 0.8 (wc 11 against a 1.2 GPa
bound). A faster integrator (Ki = Kp instead of 0.4 Kp) trims 0.1 MPa.
The flow-feedforward bound (1.1 MPa) shows what a flow measurement
would buy and is not available. The charge that can be delivered without
degassing follows directly: the saturation pressure is about 3.5 MPa
per unit of ratio, so each megapascal of dip costs about 0.3 in the
ratio.

| Charge | P_sat MPa | GVL | sched wc 6 | sched wc 11 | sched wc 11, K_cfg 1.2 |
|---|---|---|---|---|---|
| 1:1 | 3.6 | 19 mL | 292 | 292 | 292 |
| 1.5:1 | 5.4 | 19 | 292 | 292 | 292 |
| 2:1 | 7.2 | 19 | 0 (margin −0.6) | 292 (margin 0.6) | 292 (margin 1.3) |
| 2.5:1 | 9.0 | 0 | 0 | 0 (margin −1.2) | 0 (margin −0.6) |
| 3:1 | 10.0 | 0 | 0 | 0 | 0 |

(Volume delivered before a violation, nominal parameters, 0.5 MPa
margin. At 3:1 the saline is saturated at 10 MPa and the chamber holds
free gas; no controller can deliver it without bubbles at this target.)

**Robustness** (100 Latin hypercube samples over the table above;
pass rate for every requirement, in percent):

| Charge | GVL | stop law relaxed | sched wc 6 | sched wc 11 | sched wc 11, limits 7 / 100 | sched wc 11, K_cfg 1.2 |
|---|---|---|---|---|---|---|
| 1:1 | 0 | 3 | 78 | **100** | **100** | **100** |
| 2:1 | 0 | 3 | 0 | 45 | 45 | 78 |

At 1:1 the wc = 11 arms pass every sample: compression in band and
under the ceiling, no degassing, control to the end of every delivery,
stroke within limits. The wc = 6 arm loses control in 22 % of samples,
all at low K with high Cd, where its low gain lets the dip linger above
1 MPa for more than 5 s; this is slowness, not instability. At 2:1 the
margin is thin and the failures are driven by K (0 % of the lowest third
fail, 65 to 88 % of the highest third) and Cd: a stiff chamber with a
large outflow gives a deep dip. Assuming a measured K ≤ 1.2 GPa raises
the 2:1 pass rate from 45 % to 78 %. The uncertain parameters rank
K ≈ tauV ≈ Cd > noise > Vl0 > tauDiss on the key metrics.

**What this means for the machine.**
- 1:1 is deliverable with margin to spare under every plausible
  parameter, with the scheduled controller at wc = 11 rad/s, the 20 ms
  filter and StopVelMin 2 mm/s.
- 2:1 is deliverable at nominal parameters and in about half the
  plausible ones; measuring K (and Cd) decides it, and a K near 1 GPa
  makes it reliable.
- 2.5:1 and above are not deliverable at a 10 MPa target by feedback
  alone; a higher target (the ceiling permits about 11.5 MPa with the
  scheduled controller's negligible overshoot) or a smaller tube volume
  (less of the 0.8 MPa step) are the levers.
- The drive's limits (7 mm/s, 100 mm/s²) are worth raising for a faster
  compression but do not change the deliverable charge.

**Porting the schedule to the slave** needs four things in
`FB_PressureController` or `PRG_Main`: configured constants for the
contact position, the saline volume (a lower bound), the charge and the
bulk modulus (an upper bound), and the target crossover; the stiffness
bound of eq. (39) computed each scan from `G_sysActualPressure` and the
actual position; a `Lag` input to the stop law (eq. (41)) in place of
the fixed one-scan step; and an integrator that accumulates Ki·e (or is
rescaled when Ki changes) and is held at zero until the first arrival
within 0.5 MPa of the target.

**Margins against stiffness** (linear model, GVL gains):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| gas-rich | 0.002 | 92 dB | 2.8° | 0.02 | 2.1e4 |
| liquid, 300 mL, K = 1 GPa | 11.4 | 17.1 dB | 61° | 5.5 | 3.65 |
| liquid, 300 mL, K = 2.2 GPa | 25.2 | 10.2 dB | 41° | 10.8 | 1.65 |
| liquid, 30 mL, or the stop-law stiffness | 100 | −1.8 dB | −7° | 26 | 0.405 |

Expressed as a crossover wc = Kp·k (k in MPa/mm, Ki = 0.4 Kp):
- the stability limit is about 41 rad/s;
- 45° of phase margin needs about 11 rad/s;
- 60° of phase margin needs about 6 rad/s.

So a stiffness-scheduled gain is Kp = wc/k with wc of 6 rad/s, and k is
A·K/V_l during the delivery: Kp 0.24 at 300 mL and 0.024 at 30 mL for
K = 2.2 GPa.

At liquid-only stiffness, the 50 ms filter and the 20 ms drive lag cost
about the same. The largest stable Kp rises from 3.65 to 8.6 without the
filter, to 8.2 without the drive lag, and to 11.6 without both.

**Open questions:**
- The mechanical pressure ceiling: 12 MPa is assumed; 15 MPa may break
  the piston. A higher target is the main lever for a larger charge.
- The degas margin to design to. It sets where the charge against
  volume trade-off is read, not whether a configuration works.
- The measured bulk modulus K, orifice flow (Cd) and drive lag (tauV),
  which decide whether 2:1 is reliable, and the friction (see the
  assumptions table).
- Whether the saline volume and the charge can be given to the slave
  (as configuration or over the protocol), which the schedule's
  stiffness bound needs.
