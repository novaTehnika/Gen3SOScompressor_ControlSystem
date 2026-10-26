# Pressure loop design study

This directory holds a Python model of the slave's pressure loop and the
studies run on it. The studies were used to choose between closing the
loop through the drive's velocity loop or its torque loop, to choose the
controller structure and gains, and to size the opening rate of the
discharge valve.

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
python3 design/pressure-loop/study_valve.py      # dip versus valve opening time
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
| `params.py` | Defines `Params`, the chamber, actuator, sensor and simulation parameters. The values the slave configures are read from the GVL and can be overridden through `slave_cfg`. |
| `plant.py` | Models the chamber and the actuator, equations (1) to (14). `Plant.derivatives` is the continuous-time model and `Plant.scan` advances one slave scan. |
| `solver.py` | Holds the fixed-step integration methods, equations (34) and (35). |
| `study_convergence.py` | Measures the integration error against the time step for both methods. |
| `sensor.py` | Models the measurement chain: `SlaveSensor` (transducer, shunt, noise and the slave's filter and scaling blocks) and `LegacySensor`. |
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). Both share the `reset` and `step` interface. |
| `scenarios.py` | Defines the operating cases `compress`, `stiffstep`, `discharge` and `valve`. |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces and metrics. |
| `metrics.py` | Computes overshoot, peak error, settling time, RMS error, ripple, command activity and stroke used. |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `slave.py` | Puts the translated slave blocks on the import path, regenerating them if stale, and reads the `G_cfg*` defaults from the GVL. |
| `study_compare.py` | Compares velocity and torque command and the slave controller over three scenarios. |
| `study_valve.py` | Finds the pressure dip against the valve opening time. |
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

**Chamber.** A piston of area A compresses a volume that holds liquid of
effective bulk modulus K and oxygen gas drawn in at atmospheric pressure.
The gas is isothermal and dissolves into the liquid according to Henry's
law with a first-order time constant. The chamber's compliance is the
gas term plus the liquid term, so its stiffness dP/dx rises from about
0.002 MPa/mm when gas-rich to 10 to 25 MPa/mm once the gas has dissolved.
The piston position is integrated, and the chamber volume is the initial
volume minus A times the position. The contents fill the chamber, so the
stressed liquid plus the free gas equals that volume. Every run reports
the stroke used, so it can be checked against the axis travel.

A discharge outflow Q removes liquid. The liquid volume falls with the
outflow, the compliance uses the present liquid volume, and the gas
dissolved in the liquid leaves in proportion, so the chamber stiffens as
it empties.

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

Euler converges at first order, halving its error with each halving of
the step. At the default of five Heun steps per scan (dt = 0.4 ms) the
largest error is 1.7 kPa, during the gas-rich compression, and the
metrics change by less than 0.001 MPa of overshoot and 0.002 s of
settling time between the default and the reference. On stiffstep both
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

`study_robust.py` checks a slave configuration against its requirements
over the plausible ranges of the uncertain parameters, and shows which
parameters drive the failures. `robust.py` holds the parameter table
(`UNCERTAIN`), the limits (`THRESH`) and the sampler.

| Parameter | Range | Sampling |
|---|---|---|
| K | 0.5–2.5 GPa | log |
| Vl0 | 250–350 mL | linear |
| Vg0 | 100–300 mL | linear |
| tauDiss | 10–100 s | log |
| tauV | 5–50 ms | log |
| sensor noise | 1–20 kPa rms | log |
| discharge flow Qf | 0.5–2 mL/s | linear |

`mr`, `b` and `Fc` act only in torque mode, so they are not varied. Each
sample runs compress, stiffstep, discharge and valve (with a 0.3 s
opening) and checks these requirements:

- Compress and stiffstep end within the status band
  (`G_cfgPressureInBand`).
- The overshoot is at most 0.5 MPa on compress and 0.6 MPa on stiffstep,
  the limits in `tests/slavePressureMode.m`.
- The ripple over the last 2 s is at most a fifth of the status band.
- The discharge and valve dips are at most 1 MPa, and both cases end
  within the status band.
- The stroke is at most 105 mm, which is `G_cfgPosSoftLimitMax` minus the
  chamber contact position of the slave simulator.

The dip and stroke limits are placeholders until there is a stated
requirement.

The study runs a one-at-a-time sensitivity (tornado), then all corners of
the four most influential parameters, then a Latin hypercube sample
(`--n`, default 200) for each configuration. The default configurations
are the GVL, the GVL with a 20 ms filter, and the 20 ms filter with
`StopVelMin` 1 mm/s. Every configuration sees the same samples and noise
seeds. `--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine cannot
supply the flow, so no built-in configuration uses it.

## Assumptions to replace with bench measurements

| Parameter | Value used | How to measure |
|---|---|---|
| Effective liquid bulk modulus K | 1 GPa | Run at 0.1 mm/s in the liquid-only chamber and read dP/dt. |
| Discharge flow Q | 1 mL/s | Collect the volume discharged per unit time at operating pressure. |
| Reflected mass, friction | 50 t, 5 kN s/m, 500 N | Record the torque at constant velocity in both directions. |
| Sensor noise | 5 kPa rms per sample | Log the raw analog input at rest. |
| Drive velocity loop lag | 20 ms | Record a step response in velocity mode. |

## Results with these parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa s (Ki). "Slave (GVL)" is
the slave's `FB_PressureController` with the GVL configuration: Kp 0.5,
Ki 0.2, VelMax 5 mm/s, accel and decel 50 mm/s², stop-law stiffness
100 MPa/mm, StopVelMin 0.5 mm/s and a 50 ms filter. The time-domain
results use Heun's method at five steps per scan.

**Velocity versus torque command.** Velocity command wins. The drive's
own velocity loop absorbs the friction and the mass-spring resonance, so
the pressure loop sees an integrator. In torque command the pressure loop
sees the reflected mass on the stiff chamber, a lightly damped resonance
near 5 Hz behind the sensor filter, and a feedback gain above about
5 percent of A (N per MPa) is unstable. Through the slave's sensor chain,
which adds two scans of delay, the derivative-damped torque controller
(Kd 100) also oscillates, with 1.27 MPa of ripple on stiffstep. Pure
force feedforward holds the pressure only within the friction dead band
of about 0.25 MPa, and only if the force calibration is exact. Torque
therefore remains only as a force cap inside velocity mode.

**Slave controller, nominal parameters:**

| Scenario | settle s | overshoot MPa | rms error MPa | ripple MPa | stroke mm |
|---|---|---|---|---|---|
| compress to 10 MPa | 22.9 | 0.27 | 0.001 | 0.0002 | 59 |
| stiffstep 10 → 12 MPa | 1.31 | 0.22 | 0.014 | 0.003 | 0.2 |
| discharge 1 mL/s for 20 s | 24.4 | 0.60 | 0.022 | 0.005 | 5.9 |

On the gas-rich approach the configured stop law limits the overshoot to
0.27 MPa. The legacy estimating stop law overshoots by 3.1 MPa at the
same gains, because the filtered reading lags the stiffening chamber. The
cost is a slower approach: the clamp stays below VelMax from the start
and holds the velocity near 3 mm/s. The discharge overshoot of 0.6 MPa
comes after the flow stops, from integrator wind-up during the outflow.

**Margins against stiffness** (linear model, GVL gains):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| gas-rich | 0.002 | 92 dB | 2.8° | 0.02 | 2.1e4 |
| liquid, K = 1 GPa | 11.4 | 17.1 dB | 61° | 5.5 | 3.65 |
| liquid, K = 2.2 GPa | 25.2 | 10.2 dB | 41° | 10.8 | 1.65 |
| stop-law stiffness | 100 | −1.8 dB | −7° | 26 | 0.405 |

Expressed as a crossover wc = Kp·k (k in MPa/mm, Ki = 0.4 Kp):
- the stability limit is about 41 rad/s;
- 45° of phase margin needs about 11 rad/s;
- 60° of phase margin needs about 6 rad/s.

With the GVL Kp, the phase margin at liquid-only stiffness falls below
45° above K ≈ 1.9 GPa, and the loop goes unstable above K ≈ 7 GPa. It is
also unstable at the 100 MPa/mm the stop law assumes, so the stop law's
stiffness is a conservative braking assumption, not a stiffness the loop
could regulate at.

At liquid-only stiffness, the 50 ms filter and the 20 ms drive lag cost
about the same. The largest stable Kp rises from 3.65 to 8.6 without the
filter, to 8.2 without the drive lag, and to 11.6 without both.

**Derivative action** trims the dip for a valve that opens in one step
from 0.57 to 0.49 MPa, at three to five times the command activity from
sensor noise. It is not used.

**Accel and decel limit.** At the direct-control limit of 5 mm/s², the
approach into the stiff regime overshoots by 15 MPa. Pressure mode
therefore needs its own accel and decel of 50 mm/s² together with the
stop law.

**Discharge valve opening.** The table gives the dip in MPa for a 1 mL/s
final flow at 10 MPa in the liquid-only chamber, against the valve
opening time:

| Controller | step | 0.3 s | 1 s | 3 s | 10 s |
|---|---|---|---|---|---|
| PI, Kp 0.25, Ki 0.2 | 0.91 | 0.90 | 0.81 | 0.47 | 0.15 |
| Slave (GVL), Kp 0.5, Ki 0.2 | 0.57 | 0.56 | 0.49 | 0.35 | 0.14 |
| PI, Kp 1, Ki 0.5, filter 20 ms | 0.31 | 0.28 | 0.23 | 0.15 | 0.06 |
| Slave (GVL) plus flow feedforward (bound, not implementable) | 0.08 | 0.04 | 0.01 | 0.004 | 0.002 |
| Torque feedforward (reference) | 0.25 | 0.25 | 0.25 | 0.25 | 0.25 |

The dip is quasi-static, about Q/(A Kp) by eq. (33), until the opening
time exceeds the loop's integral time of about one second. Opening the
valve slowly helps only beyond that. Feedforward of the expected flow
would cut the dip below 0.1 MPa even for a step, but the machine cannot
supply the flow to the slave, so the feedforward rows are a bound on
what a flow measurement would buy, not a candidate.

**Robustness** (200 Latin hypercube samples; pass rate in percent, about
±4 points at 90 %):

| Requirement | GVL | filter 20 ms | filter 20 ms, StopVelMin 1 |
|---|---|---|---|
| compress overshoot ≤ 0.5 MPa | 98 | 100 | 100 |
| stiffstep overshoot ≤ 0.6 MPa | 80 | 98 | 98 |
| discharge dip ≤ 1 MPa | 76 | 88 | 88 |
| valve dip ≤ 1 MPa | 82 | 94 | 94 |
| valve ends in band | 81 | 81 | 100 |
| all requirements | 61 | 76 | 86 |

The other requirements pass in every sample: compress in band and
ripple, stiffstep in band, discharge in band, and stroke (at most
88 mm). A second seed gives 64 / 78 / 89 for "all". The shorter filter
buys the stiffstep margin and part of the dip; the higher StopVelMin
lets the loop end in band under the larger outflows.

The tornado ranks the parameters by influence:
Qf ≫ K > Vg0 > tauV > Vl0 > tauDiss > noise.

- **Discharge flow Qf** drives the dip and band failures. The stop law
  never commands less than StopVelMin, but above A·StopVelMin (1.7 mL/s
  at 0.5 mm/s) it can hold a sustained outflow only with a steady error
  of about StopStiffness·v²/(2·Decel), 0.3 to 0.45 MPa.
- **Bulk modulus K, with tauV,** drives the stiffstep overshoot through
  loss of margin. No sample in the lowest third of K fails, and 58 % in
  the highest third do. High K with a slow drive is the worst corner for
  every configuration.
- **Gas volume Vg0** sets only the stroke.
- **Sensor noise** up to 20 kPa has no effect on any requirement.

**Single changes that made things worse** (from an 80-sample
exploration):
- Kp 0.25, accel and decel of 100 mm/s², or a stop-law stiffness of
  40 MPa/mm each push the compress overshoot from 0.27 MPa to between 1
  and 2.6 MPa. The GVL gains sit in a narrow optimum for the gas-rich
  approach.
- Kp 1.0 with the 50 ms filter loses margin at high K.

**Open questions:**
- What is the range of the discharge flow? A flow feedforward is not
  practical on this machine, so the loop must hold the outflow by
  feedback alone.
- What is the dip limit?
- What is the stroke budget?
- Should `G_cfgPressureStopVelMin` rise above the largest Qf/A, and
  `G_cfgAnalogFilterTimeConst` fall to 20 ms?

Measure Qf first, then K, then tauV. Together they answer the questions
above.
