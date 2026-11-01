# Models

This directory holds the physical models of the machine: the chamber
with its saline and oxygen charge, the outlet (hose, valves, orifice
bank), the drive and the pressure measurement chain. They are the plant
of the design studies (`design/pressure-loop/`) and of the simulated
slave (`slave-sim/`), so the studies and the closed-loop tests run on
one model.

- `plant.py` is the reference model. `c/plant.c` is its port to C, which
  the simulated slave compiles; `tests/plantParity.py` runs both on the
  same inputs and requires every state to agree, so a change to
  `plant.py` needs the same change in `c/plant.c`.
- `params.py` holds the model's parameters (`Params`), with the values
  measured on the benches. The design studies extend them with their own
  settings (`design/pressure-loop/studyparams.py`); the simulated slave
  takes the defaults through `plant_export.py`, which writes them, with
  the contents at a run's start, for the C plant at build time.
- `slave.py` gives the models and the studies the slave's own function
  blocks, translated into the gitignored `generated/` by
  `tools/st_translate.py --target py` (regenerated when stale), and the
  `G_cfg*` defaults of the GVL.

The code follows the conventions of `design/`: each file opens with a
header giving its purpose and dependencies, every quantity carries its
unit in square brackets, and a comment such as `eq. (5)` cites the
numbered equations below. The numbering is shared with
`design/pressure-loop/README.md`, which holds the controller and
linear-model equations (19) to (33) and (39) to (41).

## Files

| File | Role |
|---|---|
| `params.py` | Defines `Params`: the chamber, oxygen charge, outlet, actuator, sensor and simulation parameters. The values the slave configures are read from the GVL and can be overridden through `slave_cfg`. `Params.contents()` gives the chamber's contents at the start of a run. |
| `plant.py` | Models the chamber, the hose, the outlet and the actuator, equations (1) to (14), (36) to (38) and (42) to (46). `Plant.derivatives` is the continuous-time model and `Plant.scan` advances one slave scan. |
| `solver.py` | Holds the fixed-step integration methods, equations (34) and (35). |
| `sensor.py` | Models the measurement chain: `SlaveSensor` (transducer, shunt, noise and the slave's filter and scaling blocks) and `LegacySensor`. |
| `slave.py` | Puts the translated slave blocks on the import path, regenerating them if stale, and reads the `G_cfg*` defaults from the GVL. |
| `plant_export.py` | Writes the parameters and a run's starting state of the default `Params` for the C plant (`slave-sim/buildSlaveSim.m` runs it). |
| `c/plant.c`, `c/plant.h` | The plant in C, for the simulated slave, with one addition the simulator needs: a scan with the piston velocity prescribed. |

## Model


**Chamber.** A piston of area A compresses a volume that holds saline of
effective bulk modulus K and the oxygen charge, admitted at the supply
pressure. The gas is isothermal and dissolves into the saline according
to Henry's law with a first-order time constant. The chamber's compliance is the
gas term plus the liquid term, so its stiffness dP/dx rises from about
0.002 MPa/mm when gas-rich to 10 to 25 MPa/mm once the gas has dissolved.
The piston position is integrated, and the chamber volume is the initial
volume minus A times the position. The contents fill the chamber, so the
stressed liquid plus the free gas and the air pocket equals that volume.
Every run reports the stroke used, so it can be checked against the axis
travel.

**Air pocket at the transducer.** A passage about 2 cm long leads from
the chamber to the pressure transducer, and it and the transducer's
cavity hold air that cannot be purged. It is present in every run, with
or without an oxygen charge. Its air dissolves only very slowly through
the narrow passage, so the model holds it as a fixed amount of isothermal
gas, `V_air` at atmosphere, eq. (46): it adds its own gas term to the
capacitance, eq. (4), and its volume to the volume closure, eq. (9), and
to the hose-opening equalisation, eq. (43), and it never dissolves or
leaves with the outflow. Its compliance falls with the square of the
pressure: with the 0.65 mL default and the full 343 mL fill it is 3.4
times the liquid's at 0.1 MPa, 11 % of it at 1 MPa and 0.14 % at 10 MPa,
and in a liquid-only compression it is the whole gas term. The default
is the upper bound given by the gas-like term of the closed-chamber
bulk-modulus fits; the passage's geometry alone suggests 0.02 to 0.06 mL.
K is the bulk modulus of the liquid alone, which is what the fits give
once their gas term is read as this pocket.

The outflow Q removes saline. The saline volume falls with the outflow,
the compliance uses the present saline volume, and the gas dissolved in
the saline leaves in proportion, so the chamber stiffens as it empties:
the liquid-only stiffness A·K/V_l is 11 MPa/mm with 300 mL left and
140 MPa/mm with 25 mL left (K = 1 GPa). The piston cannot sweep the
passages of the cylinder head, 57 mL; it reaches the limit switch
there, the machine halts, and the delivery ends. With the hose open
the dead volume is the passages, the hose and the tube, 86 mL, so the
deliverable solution is the saline volume less 86 mL (and the hose's
own 29 mL of virgin saline, which is purged, is not solution): about
249 mL.

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
over the study's opening time (`StudyParams.Topen_prox`) to the bore `Params.d_prox` (the ¼" ball
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
chamber throughout (`StudyParams.hose_before` in the studies); the model
runs either. With the hose open the liquid part of the compliance
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
the command after a dead time of 22.8 ms with a first-order lag of
18.2 ms (fitted at the 4 ms spacing of the slave's scan rows),
the same for 0.3 to 2.5 mm/s steps in both directions, and **the command
is not slewed at the configured accel**; the drive accelerates at
whatever rate the step needs. The model therefore applies the dead time
in whole scans and the lag, and has the slew limit of eq. (11) only as an
option (`slew_limited`, off by default). The lag at the piston, behind
the gearbox and screw, was not resolved by the bench; the robustness
study varies the lag from 10 to 50 ms to cover it. The piston
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
change, from the isothermal free gas, the air pocket of eq. (46) and the
liquid:

    (4)  C = (n_tot - n_dis) R T / P_abs^2 + Patm V_air / P_abs^2
             + V_l / K

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
free gas and the air pocket; dissolved gas takes no volume. The model
does not impose this identity, so it serves as a check on the
integration:

    (9)  V_ch0 - A x = V_l (1 - P/K) + (n_tot - n_dis) R T / P_abs
                       + Patm V_air / P_abs

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

The filter runs once per scan, so with the 4 ms scan the 20 ms
configured is a 20 ms time constant (alpha = 1/6).

Median of N samples followed by the first-order filter:

    (17) y[n] = y[n-1] + alpha (median(V_in[n-N+1 .. n]) - y[n-1])

Pipeline delay. The controller in scan n reads the scaled filter output
of scan n - 2:

    (18) P_meas[n] = scale(y[n-2])

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
V_cyl at pressure P, holding n_g moles of free gas and the air pocket of
eq. (46): the contents (the liquid now including the hose's V_hose of
virgin saline at atmosphere) fill the cylinder, the hose and the hose's
dilation. The model solves
it by bisection and applies it within the scan the valve opens:

    (43) (V_l + V_vir + V_hose)(1 - P1/K) + n_g R T / (P1 + Patm)
           + Patm V_air / (P1 + Patm) = V_cyl + V_hose + C_hose P1

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


### Air pocket

The passage to the pressure transducer and the transducer's cavity hold
`V_air` of air at atmospheric pressure, a fixed amount of isothermal gas
that never dissolves and never leaves. Its amount, its volume at the
absolute pressure P_abs and its share of the capacitance of eq. (4) are

    (46) n_air R T = Patm V_air,   V_pocket = Patm V_air / P_abs,
         dV_pocket/dP = -Patm V_air / P_abs^2

It is the term added to eqs. (4), (9) and (43), and it is part of the
initial cylinder volume V_ch0 of eq. (9): at a run's starting pressure P0
the cylinder holds Patm V_air / (P0 + Patm) more than the liquid and the
oxygen alone. The stiffness, eq. (10), uses the capacitance that
contains it.


## Numerical solution

`sim.py` steps the closed loop one slave scan Ts at a time. In each scan
the sensor samples the pressure, the controller computes a command, and
the plant advances with the command computed in the previous scan held
constant. The plant splits the scan into `nsub` integration steps of
length dt = Ts / nsub (5 steps of 0.8 ms by default). Before each step
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
(dt = 0.04 ms). The largest pressure difference from the reference, with
the slave controller:

| Scenario | Euler, nsub 1 / 5 / 50 | Heun, nsub 1 / 5 / 50 |
|---|---|---|
| compress | 34 / 6.6 / 0.66 kPa | 0.053 / 0.002 / 0.000 kPa |
| stiffstep | 5.9 / 1.1 / 0.11 kPa | 0.46 / 0.016 / 0.000 kPa |
| deliver | 4.7 / 0.88 / 0.087 kPa | 0.38 / 0.013 / 0.000 kPa |

Euler converges at first order, halving its error with each halving of
the step; Heun at second order. At the default of five Heun steps per
scan (dt = 0.8 ms) the largest error is 0.016 kPa, and the metrics
change by less than 0.001 MPa of overshoot and 0.001 s of settling time
between the default and the reference. Heun at five steps costs about
50 % more than Euler at five and is 70 to 3000 times more accurate on
these cases, which is why it is the default. The volume closure,
eq. (9), holds to within 2e-9 m^3 over 6 s of compression.

## Measured values

The bench tests of 2026-10-26 (`bench/`) replaced
the guessed parameters. The values the model now carries, with what was
assumed before:

| Parameter | Measured | Assumed before | Where |
|---|---|---|---|
| Bulk modulus K | 0.72 GPa effective (0.67–0.78; 0.81 gas-corrected, the liquid value once the pocket is explicit) | 1 GPa | `charBulkModulus`, 3 runs at a 343 mL fill |
| Trapped air V_air at the transducer | upper bound 0.65 mL (the gas term of the closed-chamber K runs); the 2 cm passage of 1–2 mm bore holds 0.02–0.06 mL; to be settled by the fits at two fill levels | not modelled | `charBulkModulus`, `Vg_mL` of the closed-chamber runs |
| Saline volume Vl0 | 343 mL at the 280 mm start, including the dead volume | 300 mL | geometry |
| Dead volume | 57 mL of passages (hose shut, with the present fittings); 86 mL with the 29 mL hose and the tube (delivery) | 20–30 mL (then 10 mL in the master) | 57 g of water fills it |
| Hose compliance | 0.24 mL/MPa | not modelled | difference between bulk-modulus runs with the hose shut and open |
| Orifice Cd | 0.85 (0.84–0.87), exponent 0.48; 4.8 mL/s at 10 MPa | 0.7; 4.0 mL/s | `charOrificeFlow`, deliveries at 2, 5 and 10 MPa |
| Drive response | 22.8 ms dead time, 18.2 ms lag at the motor, no slew | 20 ms lag, slewed at the configured accel | `charDriveLag`, 74 step captures at the 4 ms scan |
| Lag at the piston | not resolved | — | repeat with the pressure probe |
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
