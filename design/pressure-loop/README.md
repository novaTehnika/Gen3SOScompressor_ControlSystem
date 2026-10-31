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
| `plant.py` | Models the chamber, the hose, the outlet and the actuator, equations (1) to (14), (36) to (37) and (42) to (45). `Plant.derivatives` is the continuous-time model and `Plant.scan` advances one slave scan. |
| `solver.py` | Holds the fixed-step integration methods, equations (34) and (35). |
| `study_convergence.py` | Measures the integration error against the time step for both methods. |
| `sensor.py` | Models the measurement chain: `SlaveSensor` (transducer, shunt, noise and the slave's filter and scaling blocks) and `LegacySensor`. |
| `controllers.py` | Defines `SlaveController` (the slave's block, with a flow feedforward option kept only as a bound), `ScheduledSlaveController` (the same block with its gains and stop law scheduled on an estimate of the chamber stiffness, equations (39) to (41)) and `LegacyController` (PID with a stiffness-estimating stop law and a torque mode). All share the `reset` and `step` interface. |
| `scenarios.py` | Defines the operating cases `compress`, `deliver`, `cycle` and the `stiffstep` probe, with the hose procedure and the valve timings (proximal opening, purge, pause, delivery). |
| `sim.py` | Runs one closed-loop simulation: `run(params, scenario, controller)` returns a `Result` with traces, the valve event times and metrics. It sequences the valves: the hose opening, the purge, its closing and the reopening. |
| `metrics.py` | Computes the loop metrics (overshoot, settling time, ripple, stroke) and the process metrics (pressure ceiling, saturation margin, the deepest dip at a valve event, volume delivered before a violation, solution delivered). |
| `runner.py` | Runs many simulation cases in parallel worker processes. |
| `slave.py` | Puts the translated slave blocks on the import path, regenerating them if stale, and reads the `G_cfg*` defaults from the GVL. |
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
passages of the cylinder head, 50 mL; it reaches the limit switch
there, the machine halts, and the delivery ends. With the hose open
the dead volume is the passages, the hose and the tube, 79 mL, so the
deliverable solution is the saline volume less 79 mL (and the hose's
own 29 mL of virgin saline, which is purged, is not solution): about
256 mL.

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

**Hose and procedure.** A 3 ft hose of ¼" bore (29 mL) joins the
cylinder head to the outlet, with a ball valve at each end. In the
present procedure the proximal valve is shut while the charge is
compressed and mixed, and the hose holds virgin saline at atmosphere.
After mixing the proximal valve is opened; the distal valve is then
opened to purge the virgin saline through the orifices, shut while the
tube is moved to the delivery vessel, and opened again to deliver the
solution. The model carries the hose as a volume of virgin saline, a
wall compliance C_hose (0.24 mL/MPa, measured by difference between
bulk-modulus runs with the hose shut and open) and an open/shut state.
When the proximal valve opens, the chamber's stored compression must
also stress the hose's liquid and dilate its wall, and the pressure
equalises in one step, eq. (43): on the measured chamber that is a drop
from 10 to 6.4 MPa, which degasses any charge above 1.8:1 and which no
controller can prevent, because it happens inside one scan. The
proximal valve is the one valve event whose rate the operator sets, so
the model also opens it through a restriction: the valve area rises
over `Params.Topen_prox` to the bore `Params.d_prox` (the ¼" ball
valve, or a needle valve in its place), the flow into the hose follows
the orifice law at the pressure difference, eq. (45), and the hose
pressure rises with the volume admitted, eq. (44), until the two meet
and the hose joins the chamber. The solution admitted is tracked as
parcels in the order it entered, each with the hose pressure it met.
A parcel that met a pressure below the saturation pressure of the
solution degassed; by plug flow it lies right behind the virgin
saline, and the purge is sized to carry both out (the scenario's
`V_purge` of None). Degassing is judged at the hose pressure; the
lower pressure in the valve throat, which the solution crosses in
microseconds, is neglected. The proposed alternative opens the
proximal valve before the compression, so the hose is part of the
chamber throughout (`Params.hose_before`); the model runs either. With the hose open the liquid part of the compliance
is the saline, the virgin saline and the wall, eq. (42), and the
chamber at the start of a delivery is 4.8 MPa/mm instead of 7.2. The
purge is sequenced by the simulation: the distal valve closes once the
hose's volume has left, over `Params.Tclose` (0 by default), and opens
again after a pause. The closing is a disturbance of its own: the
piston is running at the delivery speed when the flow stops, and the
pressure rises at the chamber stiffness times that speed until the
loop has stopped it.

**Outlet.** The outlet is a ¼" ball valve, a 1 mm bore tube 300 mm long
and a plate of 32 orifices of 40 µm discharging to atmosphere. The
valve's bore is 450 times the orifice area, so once it cracks the
orifices set the flow, Q = Cd·A_o·√(2P/ρ), eq. (36): 4.8 mL/s at 10 MPa
with the measured Cd 0.85, falling with the square root of the
pressure. The tube's friction loss (about 1 % of the chamber pressure)
and inertance (a flow build-up time under 0.1 ms) are neglected. The
tube holds air at atmosphere before the valve first opens; the chamber
liquid compresses and expels it in milliseconds, so the chamber loses
the tube's 0.24 mL in the opening scan, eq. (37). On a liquid-only
chamber that is a pressure step of V_tube divided by the chamber
compliance, 0.3 MPa with the hose open, before the controller can act.

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
shunt, plus the transducer's zero error (zero once the input is
calibrated: the slave's `G_cfgPressureShuntOhms` is the effective
421.4 Ω the input reads, `bench/pressure/README.md` section 6) and white
noise specified as an RMS pressure (measured 2.0 mV rms across the shunt, 4.6 kPa, broadband; 0.8 mV with a 4.7 µF
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

### Hose

Liquid part of the chamber compliance with the hose open: the saline
V_l, the virgin saline V_vir still in the system, and the hose wall. It
replaces the liquid term V_l / K of eq. (4). With the hose shut the
virgin saline is not in the system and C_hose does not apply:

    (42) C_liq = (V_l + V_vir) / K + C_hose

Pressure P1 after the proximal valve opens on a cylinder of volume
V_cyl at pressure P, holding n_g moles of free gas: the contents (the
liquid now including the hose's V_hose of virgin saline at atmosphere)
fill the cylinder, the hose and the hose's dilation. The model solves
it by bisection and applies it within the scan the valve opens:

    (43) (V_l + V_vir + V_hose)(1 - P1/K) + n_g R T / (P1 + Patm)
           = V_cyl + V_hose + C_hose P1

The virgin saline leaves through the outlet before the solution and
carries no dissolved gas, so eqs. (7) and (8) act on V_vir, with
r_out = 0, while V_vir > 0.

Hose pressure while it fills through the proximal valve, with V_in of
solution admitted (unstressed volume): the hose's saline and the
admitted solution, stressed, fill the bore and its dilation:

    (44) P_h = V_in / (C_hose + (V_hose + V_in) / K)

Flow through the proximal valve at area fraction a_p(t) into the
filling hose, the orifice law at the pressure difference, capped at
the flow that would equalise the two pressures within one scan (so
that a wide-open valve reproduces eq. (43) without overshoot); the
admitted volume grows by Q_prox / (1 - P/K) and the chamber loses
Q_prox from its volume balance, eq. (5). C_cyl is the cylinder's
capacitance with V_in removed from its liquid and C_h the hose's,
C_hose + (V_hose + V_in) / K:

    (45) Q_prox = min( a_p(t) Cd_prox A_prox sqrt(2 (P - P_h) / rho),
                       (P - P_h) / (1/C_cyl + 1/C_h) / Ts )

The solution that enters while P_h is below the saturation pressure
of eq. (38) degasses; its volume is P_sat C_h to first order,
whatever the bore.

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
| passages | 45–55 mL | linear | measured 50 mL with the hose shut |
| hose compliance | 0.12–0.36 mL/MPa | log | 0.24 mL/MPa by difference of two bulk-modulus runs |
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
are the GVL (the 20 ms filter and StopVelMin 2 mm/s, which passes the
1.4 mm/s the measured outflow needs); the earlier StopVelMin of 0.5 mm/s
as a reference; the scheduled controller at wc = 6, 11 and 16 rad/s with
the measured bounds (K_cfg 1.0 GPa, dead volume 74 mL, hose compliance
0.12 mL/MPa); and wc = 11 with VelMax 7 mm/s. `--hose-before` runs the
proposed procedure (the hose opened before the compression). Every
configuration sees the same samples and noise seeds.
`--cfg key=value` runs a single custom configuration instead, and
`--flow-ff` adds a flow feedforward to it as a bound; the machine cannot
supply the flow, so no built-in configuration uses it.

## Measured values

The bench tests of 2026-10-26 (`bench/`) replaced
the guessed parameters. The values the model now carries, with what was
assumed before:

| Parameter | Measured | Assumed before | Where |
|---|---|---|---|
| Effective bulk modulus K | 0.72 GPa (0.67–0.78; 0.81 gas-corrected) | 1 GPa | `charBulkModulus`, 3 runs at a 343 mL fill |
| Saline volume Vl0 | 343 mL at the 280 mm start, including the dead volume | 300 mL | geometry |
| Dead volume | 50 mL of passages (hose shut); 79 mL with the 29 mL hose and the tube (delivery) | 20–30 mL (then 10 mL in the master) | 50 g of water fills it |
| Hose compliance | 0.24 mL/MPa | not modelled | difference between bulk-modulus runs with the hose shut and open |
| Orifice Cd | 0.85 (0.84–0.87), exponent 0.48; 4.8 mL/s at 10 MPa | 0.7; 4.0 mL/s | `charOrificeFlow`, deliveries at 2, 5 and 10 MPa |
| Drive response | 12 ms dead time, 8.7 ms lag at the motor, no slew | 20 ms lag, slewed at the configured accel | `stepResponseFit`, 74 step captures |
| Lag at the piston | below 25 ms (not resolved) | — | repeat with the pressure probe |
| Coulomb friction Fc | 6.1 kN | 0.5 kN | `frictionFit` |
| Viscous friction b | 2.3 MN s/m | 5 kN s/m | `frictionFit` |
| Load-dependent friction mu | 0.45 (efficiency 0.69 pushing) | 0.85 efficiency | `frictionFit` |
| Breakaway | about 24 kN | — | `frictionFit` |
| Sensor noise | 2.0 mV rms (4.6 kPa) bare, 0.8 mV (1.8 kPa) with 4.7 µF; broadband | 5 kPa | `pressureNoiseProbe` |
| Transducer zero | +0.002 MPa vented with the input calibrated (421.4 Ω effective shunt); with the 434 Ω setting the reading was 0.971 P − 0.104 MPa | 0 | current source, `bench/pressure` section 6 |

Still to measure: the dissolution time constant tauDiss (the hold time
before delivery depends on it) and the lag at the piston. The GVL on the
machine now has the 20 ms filter, a median of 5, `G_cfgVelLimitMax`
7 mm/s and `G_cfgDirectCtrlAccel` 10 mm/s² (which the drive does not
apply).

## Results with the measured parameters

Gains are in mm/s per MPa (Kp) and mm/s per MPa·s (Ki). "GVL" is the
slave's `FB_PressureController` as now configured: Kp 0.5, Ki 0.2,
VelMax 5 mm/s, accel and decel 50 mm/s² (which the drive does not
apply), stop-law stiffness 100 MPa/mm, StopVelMin 2 mm/s and the 20 ms
filter. StopVelMin was 0.5 mm/s until this study; that setting appears
below as "StopVelMin 0.5". The time-domain results use
Heun's method at five steps per scan and the measured plant: K 0.72 GPa,
343 mL of saline with 50 mL of passages and a 29 mL hose, Cd 0.85, a
12 ms dead time
and 8.7 ms lag in the drive, and a −0.11 MPa transducer offset, which
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
sweeps 12 mm, and the GVL loop arrives in band after 4.7 s with
0.73 MPa of overshoot; the scheduled loop at wc = 11 rad/s arrives in
2.5 s with 0.12 MPa, which is the transducer offset, not an overshoot
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
the hose's saline is compressed with the charge (`Params.hose_before`).
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
| 6.35 mm (ball valve) | 0.5 s | 6.4 / 6.4 / 6.4 | 7.0 / 7.0 / 7.0 |
| 1.0 mm | 0.5–0.8 s | 6.9 / 7.8 / 8.0 | 7.6 / 8.6 / 8.8 |
| 0.5 mm | 0.7–1.1 s | 7.7 / 8.8 / 9.2 | 8.5 / 9.7 / 10.0 |
| 0.3 mm | 1.3–1.8 s | 8.1 / 9.2 / 9.4 | 8.9 / 10.2 / 10.4 |
| 0.2 mm | 2.4–2.9 s | 8.1 / 9.2 / 9.4 | 8.9 / 10.2 / 10.4 |
| 0.1 mm | 8.5–9.1 s | 8.1 / 9.2 / 9.4 | 8.9 / 10.2 / 10.4 |

Below a 0.3 mm bore (an effective area of 0.04 mm², a Cv of about
0.003) the fill is a flow the loop follows, the chamber floor is set
by the loop's own dip (the same as at the distal opening: 1.9 MPa for
the GVL gains, 0.8 at wc = 11, 0.6 at wc = 16), and the fill takes one
to nine seconds. At 2:1 that keeps every arm above saturation with the
required margin (0.8 MPa for the GVL, 2.0 and 2.2 for the scheduled
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
that carries it out is 31 mL at 2:1 and 32 mL at 2.5:1 instead of 29,
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
in rigid saline) and crosses the 0.92 m hose in 2.8 ms, under two
scans. The loop's own delay from a reading to piston motion is about
50 ms: the 20 ms filter, the median and pipeline scans, the 12 ms
drive dead time and the 8.7 ms velocity-loop lag. A lead of 2.8 ms
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
29 mL purge, the pause and 256 mL of solution. With StopVelMin at its
earlier 0.5 mm/s the loop sat low for the whole delivery: the stop law,
which with 100 MPa/mm and 50 mm/s² admits the 1.4 mm/s the outflow
needs only at an error of about 2 MPa, holds the integrator through its
anti-windup. The bench saw this: the loop held 8.1 MPa for a 10 MPa
target. `StopVelMin` of 2 mm/s, now the GVL value, removes it. With the
hose open its compliance caps the end stiffness at 9.8 MPa/mm (K
0.72 GPa, 79 mL, 0.24 mL/MPa), where the GVL gain keeps 71° of phase
margin and 18 dB of gain margin, so the emptying chamber no longer
threatens the loop.

**Closing the purge valve is the second limit.** When the distal valve
is shut at the end of the purge the piston is running at 1.4 mm/s into
a chamber of 4.8 MPa/mm, and the pressure rises at 7 MPa/s until the
loop has stopped it. The GVL loop, with Kp 0.5 mm/s per MPa and an
integral time of 2.5 s, lets it rise 2.2 MPa, to 12.2 MPa from a 10 MPa
target: through the ceiling. The scheduled loop at wc = 11 stops it in
1.0 MPa, at wc = 16 in 0.75 MPa. A slow hand on the valve helps little
(a 1 s closing takes 0.1 MPa off the peak), because the loop, not the
valve, is the slow element. A gain the chamber allows is the remedy,
or a procedure that stops the piston before the valve is shut.

**Scheduled controller.** `ScheduledSlaveController` (equations (39) to
(41)) with the measured bounds (K_cfg 1.0 GPa, dead volume 74 mL, hose
compliance 0.12 mL/MPa) holds its margins constant through the run,
69° / 16 dB at wc = 6 rad/s, 56° / 11 dB at wc = 11 and 44° / 8 dB at
wc = 16. Nominal results, 1:1 at 10 MPa, hose opened before the
compression, all with StopVelMin 2 mm/s:

| Arm | Compress: arrive s / overshoot MPa | Delivery: dip MPa / margin MPa / peak MPa / V_ok mL |
|---|---|---|
| StopVelMin 0.5 (earlier) | 4.8 / 0.63 | 1.96 / 4.5 / 10.8 / 22 (steady error) |
| GVL | 4.7 / 0.73 | 1.95 / 4.5 / 12.2 / 29 (ceiling at the purge closing) |
| scheduled, wc 6 | 3.1 / 0.13 | 1.32 / 5.1 / 11.5 / 256 (all) |
| scheduled, wc 11 | 2.5 / 0.12 | 0.77 / 5.7 / 11.0 / 256 (all) |
| scheduled, wc 11, Ki = Kp, VelMax 7 | 1.9 / 0.15 | 0.75 / 5.7 / 11.0 / 256 (all) |
| scheduled, wc 16 | 2.3 / 0.11 | 0.60 / 5.9 / 10.8 / 256 (all) |
| flow feedforward (bound, not implementable) | — | 2.18 / 4.3 / 12.7 / 29 |

(The 0.11 MPa "overshoot" of the scheduled arms is the transducer
offset. V_ok is given here as solution: the metric itself counts the
29 mL purge too, so 256 mL of solution reads as 285 mL delivered.) The
scheduled arms compress twice as fast as the GVL with
no overshoot of the reading, because the stop law brakes for the
estimated stiffness and the whole lag (measurement, dead time and
drive) and the integrator is held until arrival; they cut the opening
dip to a third, because the gain at the opening is set by the measured
stiffness, hose included, rather than a fixed worst case; and they are
the only arms that keep the purge closing under the ceiling. VelMax
7 mm/s shortens the compression by 0.6 s and does not change the dip;
the drive does not slew, so accel and decel settings do nothing.

**The dip and the closing peak set the charge.** The dip at an opening
is the 0.3 MPa tube step plus the quasi-static Q/(A·Kp) of eq. (33),
recovered over the integral time: 1.95 MPa at Kp 0.5, 0.77 at wc 11
(Kp 2.3 at 4.8 MPa/mm), 0.60 at wc 16. The saturation pressure is
3.7 MPa per unit of ratio in saline, so each megapascal of dip costs
about 0.27 in the ratio, and a higher target buys 0.27 per megapascal
the other way, until the closing peak meets the ceiling. Solution
delivered before a violation at nominal parameters, 0.5 MPa margin
(the margin or the peak in parentheses where it fails):

| Charge | P_sat MPa | StopVelMin 0.5 | GVL | sched wc 6 | sched wc 11 | sched wc 16 |
|---|---|---|---|---|---|---|
| 1:1 | 3.6 | 0 mL (steady error) | 0 (peak 12.2) | 256 | 256 | 256 |
| 2:1 | 7.2 | 0 | 0 (peak 12.2) | 256 (margin 1.5) | 256 (2.0) | 256 (2.2) |
| 2.5:1 at 10 MPa | 9.0 | 0 | 0 | 0 (−0.4) | 0 (0.2) | 0 (0.4) |
| 2.5:1 at 11 MPa | 9.0 | 0 | 0 (peak 13.3) | 0 (peak 12.6) | 256 (1.2, peak 12.0) | 256 (1.3, peak 11.8) |
| 3:1 at 11 MPa | 10.9 | 0 | 0 | 0 | 0 (−0.7) | 0 (−0.5) |
| 2:1, hose opened after mixing | 7.2 | 0 | 0 | 0 (−0.8) | 0 (−0.8) | 0 (−0.8) |

So 2:1 is deliverable at 10 MPa by the scheduled controller at any of
the three crossovers, 2.5:1 needs wc = 16 and an 11 MPa target (where
the closing peak reaches 11.8 MPa, at the ceiling; wc = 11 peaks at
12.0 and fails most plausible chambers), and 3:1 is saturated at
10.9 MPa and out of reach below the ceiling. The
GVL gains deliver nothing without a violation, because the purge
closing takes the pressure to 12.2 MPa; were the ceiling 12.5 MPa they
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
| 2.5:1 at 11 MPa, hose before | 0 | 29 | 98 |
| 1:1 at 10 MPa, hose after mixing | 0 | **100** | **100** |
| 2:1 at 10 MPa, hose after mixing | 0 | 0 | 0 |

The GVL fails every sample on the ceiling at the purge closing (its
peak is 12.2 MPa at nominal and rises with K and the hose compliance's
lower end); it passes every sample on degassing at 1:1 and 2:1 with the
hose opened before the compression. The scheduled arms pass every
sample at 1:1 and 2:1 with that procedure: the measured ranges are
narrow enough that nothing in them threatens the saturation margin,
the ceiling or the end of the delivery. With the hose opened after
mixing, every arm degasses every sample at 2:1, by the equalisation
drop, and 1:1 passes. At 2.5:1 the 10 MPa target leaves 1.0 MPa to
saturation, less than any arm's dip plus the margin; at 11 MPa the
closing peak sits at the ceiling, so wc = 16 passes 98 % and wc = 11
only 29 %, the failures being the stiffest chambers (high K, low hose
compliance), where the peak crosses 12 MPa.

**What this means for the machine.**
- Open the proximal valve before the compression. Opened at pressure,
  the hose drops the chamber 3.6 MPa in one scan, which no controller
  can prevent and which degasses any charge above about 1.8:1.
- `G_cfgPressureStopVelMin` is now 2 mm/s. That alone removes the
  steady error the bench saw (8.1 MPa for a 10 MPa target).
- The GVL gains let the purge closing carry the pressure to 12.2 MPa,
  through the 12 MPa ceiling, in every case; until the gain is raised,
  stop the piston before shutting the distal valve, or confirm that
  the mechanical ceiling allows 12.5 MPa.
- The scheduled controller cuts the opening dip to a third (0.8 MPa at
  wc = 11 against 2.0), holds the closing peak to 11.0 MPa and
  compresses twice as fast with no overshoot; it delivers 2:1 at 10 MPa
  in every plausible case and, at wc = 16 and an 11 MPa target, 2.5:1
  in 98 % of them, with the closing peak at the ceiling. Its margin is
  the same at every point of the run.
- 3:1 is saturated at 10.9 MPa and cannot be delivered without bubbles
  below the 12 MPa ceiling by any controller.
- The drive does not apply the accel and decel settings, so they are not
  tuning levers; VelMax 7 mm/s shortens the compression only.

**Margins against stiffness** (linear model, GVL gains, 20 ms filter,
12 ms dead time):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| liquid, 343 mL + hose, K = 0.72 GPa, hose open | 4.5 | 24.9 dB | 73° | 2.3 | 8.9 |
| liquid, 343 mL, hose shut | 7.2 | 20.8 dB | 73° | 3.6 | 5.6 |
| liquid, 120 mL, hose open | 8.4 | 19.5 dB | 72° | 4.2 | 4.8 |
| liquid, 79 mL dead volume (limit switch), hose open | 9.8 | 18.2 dB | 71° | 4.9 | 4.1 |
| stop-law stiffness | 100 | −2.0 dB | −14° | 37 | 0.40 |

With the hose open its compliance caps the stiffness at 9.8 MPa/mm, so
the GVL gain keeps most of its margin to the limit switch; the price is
a soft chamber at the openings and the closing, where the fixed gain is
four times too low for the dip and the peak. The scheduled gain
Kp = wc/k holds 56° at wc = 11 everywhere.

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
  mixing (`Params.hose_before` False). A switched bypass orifice is not
  fitted.

**Open questions:**
- The calibration's reference: a hand-held meter's mA range. A loop
  calibrator would tighten it.
- The lag at the piston (below 25 ms) and the dissolution time constant
  are the two unmeasured plant parameters; the robustness study covers
  the first, and the second sets only the hold time.
- The hose: how much of its 0.24 mL/MPa is wall and how much trapped
  air (air would make the proximal opening worse than eq. (43) at low
  pressure); whether the whole 29 mL is purged; how long the pause
  between the purge and the delivery is, and whether the tube drains
  during it (`tube_refills` in the scenario, off by default); and how
  fast the distal valve is shut (`Params.Tclose`, 0 by default, which
  is the worst case and changes the peak by 0.1 MPa).
- Whether a bypass orifice of 0.3 mm or less (Cv 0.003), opened ahead
  of the ball valve at pressure, is worth fitting later
  (`study_hose_opening.py`).
