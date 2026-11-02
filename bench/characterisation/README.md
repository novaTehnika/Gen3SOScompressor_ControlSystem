# Pressure Loop Characterisation Bench

Measures four parameters of the pressure loop that the design model in
`design/pressure-loop/` (`params.py`) and the slave's stop law carry as
assumptions, so that the measured values replace them: the effective bulk
modulus of the saline-filled chamber (`K`), the discharge coefficient of the
outlet orifice bank (`Cd`, the outflow at a known pressure), the lag of the
drive's velocity loop as seen at the piston (`tauV`) and the distance the
piston needs to stop under load (the stop law's deceleration
`G_cfgPressureDecel` and its delay). The friction test is a separate bench,
but test 4 below gives friction points under load from the same run.
Test 5 (section 10) takes the drive path between the motor and the
piston: the reflected inertia, the friction reversal under load, the
path's stiffness (or a bound on it), its backlash and the motor-side
friction.

Everything is measured through the protocol the master already uses: the
slave reports position, velocity, torque and pressure in IR11-20 every 50 ms
(`docs/master/ModbusRegisterMap.md`). The drive lag alone needs the 4 ms
scan, which the slave's step probe (`slave-src/FB/FB_Probe.st`, read by
`bench/common/benchProbe.m`) provides. Nothing in `slave-src/` changes for
the bench: the probe is part of the main program.

This bench shares the friction bench's conventions: the same Modbus master
pattern (`bench/common/`), the same log columns and capture format, the
same `bench/common/captures/` directory and the same slave program. Its run
script differs in what the steps can do: ramps that end on a pressure,
command changes within one mode session without a stop, and pressure and
position guards.

| File | Contents |
|------|----------|
| `charRun.m` | The master for a run: drives a path of steps through `masterStep` as the shell does, logs every exchange in `frictionRun`'s format, stops the axis when the pressure or the position leaves the envelope, and arms and reads the step probe between exchanges (`benchProbe`) |
| `charConfig.m` | Chamber and outlet geometry, the pressure limits and the slave settings read from `slave-src/GVL/GlobalVariables_Reference.st` |
| `charBulkModulus.m` | Test 1: K from slow ramps, with a bound on residual gas and, from two fills, the split between liquid and series compliance |
| `charOrificeFlow.m` | Test 2: Cd and the square-root law from deliveries at held pressure, with the collected-volume check |
| `charDriveLag.m` | Test 3: tauV and dead time from the step probe captures at the motor and, through their pressure channel, at the piston; the 50 ms log as a coarse check |
| `charStopPath.m` | Test 4: builds the one-pass path, a staircase of pressure levels with an excursion set at each, sized to a pressure budget from the fill and the gas charge |
| `charStopDistance.m` | Test 4: the stopping distance of every stop in the run, from the captures and the log, fitted to the stop law's two terms per direction and compared with the slave's allowance |
| `charDrivePath.m` | Test 5: the drive path from the motor's position, velocity and torque against the pressure: the inertia of velocity steps, the friction reversal and the stiffness bound of the K ramps, and the backlash, windup and motor-side friction of the reversal and dither captures |
| `charDriveRecipe.m` | Test 5: the `velstep` paths of the two reversal runs (direct reversals in the open chamber, a dither about a pressure in the closed one) |
| `charScheduleCheck.m`, `charScheduleBound.m` | Section 9: the stiffness bound of the slave's gain schedule (`FB_PressureSchedule`) computed from the GVL at the pressures and positions of a log and compared with the stiffness the log measured |

`tests/charRunSim.m` runs `charRun` and the fits against the simulated slave
(through the friction bench's `fakeProbeLink`), the one-pass path included,
`tests/charFitsSynthetic.m` the fits on synthetic logs with known
parameters (the schedule check included, on the committed K300 logs too, and
`charDrivePath` on a synthetic two-mass drive and on the October logs),
and `tests/slavePressureMode.m` holds `charScheduleBound` equal to the
compiled block; `tests/run_all.sh` runs them.

## 0. What the friction bench's runs already give

The friction bench's logs are in the same format, so two of the three
parameters come out of runs made for it:

| Friction bench run | Gives | Command |
|---|---|---|
| `'delivery'` at 10, 5 and 2.5 MPa | Cd and the square-root law (test 2) | `charOrificeFlow({[logs '/d10_delivery_*.csv'], [logs '/d5_delivery_*.csv'], [logs '/d2_delivery_*.csv']}, struct('K_Pa', ..., 'collected_mL', [...]))`; leave `collected_mL` out if the discharge was not weighed |
| `'steps'` captures | tauV at the motor (test 3, open chamber) | `charDriveLag([logs '/s1_cap_steps_*.csv'], struct('accel', 1e4, 'decel', 1e4))` or `stepResponseFit` with the same options |

`logs` is where the files are (section 1, "Where the logs are"). The fits
take a name, a pattern or a cell array of either.

What needs a new run: K (section 3, slow ramps in a closed liquid-filled
chamber, which no friction set does) and the lag at the piston (section 5,
velocity steps in a closed liquid-filled chamber at 3 to 5 MPa with the
probe recording the pressure; the gas-charge set's chamber is too soft for
the pressure to move). The friction bench's deliveries also serve test 1's purge, section
2 step 4.

## 1. Setup

### Lab PC

As in `bench/common/README.md`: the shell model must be stopped,
`cfg.address` in `bench/common/benchConfig.m` set to the controller IP, and
`SafetyHold.slx` built once with `bench/common/buildSafetyHold` and set to
Connected IO. `charRun` starts it before the first command and stops it at
the end, which releases the safety relay.

```matlab
cd bench/characterisation
addpath ../common
r = charRun('label', path);              % see sections 3 to 6 for the paths
logs = '../common/captures';             % where charRun wrote it (below)
```

**Where the logs are.** A run writes to `bench/common/captures/`
(gitignored, the directory every bench logs to; `opts.saveDir` changes it).
The logs of the 26 October characterisation and friction runs and of the
30 and 31 October one-pass stops runs are committed in `bench/captures/`,
the reference set the results in section 8 were fitted from. The fit
examples below use `logs`: leave it at `'../common/captures'` for a new
run, or set `logs = '../captures'` to run the same fit on the committed
logs (for example `charBulkModulus([logs '/K300_char_20261026_19*.csv'])`).
Files are not moved between the two.

A fault still latched from an earlier run is cleared by `charRun` with the
operator reset before its first step. The controller answers a Modbus
request in 1 to 8 ms (`bench/modbus`), so the protocol exchange, one write
and one read, should take under 20 ms; the run prints any exchange over
150 ms, skips the probe transactions in that cycle, and at the end gives
the write and read times separately. The slave's watchdog faults the run
with `FAULT_COMM` (code 4) if the heartbeat stops for 300 ms
(`G_cfgCommTimeout`), so exchanges of 200 ms end it. `bench/modbus/modbusRawProbe`
times the transactions on their own, without the master, and is the
first check: 20 ms median and 30 ms p99 for the sequential exchange is
the controller's normal. A large register group in Hardware
Configuration slows it whether or not a program uses the group; the step
probe's is IR401-524 and HR111-114, which does not
(`bench/common/README.md` step 3). A second Modbus client (the shell
model or the operator app still connected, another MATLAB session, an
OPC client) and the network path are the other causes.

The run ends with the axis stopped, prints a table of the steps and saves
`<logs>/<label>_char_<timestamp>.csv` (columns as in
`bench/common/README.md`) and one
`<label>_cap_<n>_<step>_<timestamp>.csv` per probe capture (columns `t`,
`vCmd`, `velocity`, `torque`, `position`, `pressure`, `volts` and `status`;
the first five are the friction bench's captures').
Ctrl-C stops the axis the same way. The guards are `charConfig.pressureStop`
(11.0 MPa, below the 12 MPa ceiling) and the position band inside the
slave's soft limits; `opts.pressureStop`, `opts.posStop` and `opts.posMin`
override them for a run and cannot be set above the ceiling.

### Slave

No change for tests 1 and 2. Test 3 uses the step probe (`FB_Probe`), set up as in
`bench/friction/README.md` section 4.5; `charRun` arms it before every
command change in a velocity session when `opts.probe` is true (only for
commands to zero when it is `'stops'`), and applies the command once the
slave has acknowledged the arm. Without it,
run test 3 anyway: the 50 ms log gives the lag to about 25 ms.

### Settings that matter

`charConfig` reads the GVL reference, which must match the controller:
velocity-mode acceleration (`G_cfgDirectCtrlAccel`, in both directions; the
drive was found not to slew at it, section 8), the velocity clamp, the soft limits, the pressure filter (`G_cfgAnalogFilterTimeConst`,
`G_cfgAnalogMedianSize`) and the scan time. The ramps and steps below stay
within `G_cfgVelLimitMax`, the torque limits (34 kN at 10 MPa is about 37 %
of rated torque through the 70:1 gearbox) and the soft limits; the position
guard stops a delivery 3 mm before the positive soft limit, so the soft limit
must allow the piston to approach the end of travel if a delivery is to run
to the end. The pressure target in the operator app is in atm absolute; this
bench works in MPa gauge throughout, as the slave does.

## 2. Volumes: dead volume and fill

Both fits need the liquid volume. The dead volume, what the piston cannot
sweep, is 57 mL (`masterConfig.deadVolume`, with the present fittings),
the cylinder passages alone, without the discharge hose. Filling the
passages from a burette is not practical (they are small and
convoluted); it was measured by purging the air with liquid, running the
piston to the soft limit, opening the valves
and drawing the liquid out of the passages into the chamber, then removing
the liner and piston from the bottom of the cylinder, catching the liquid,
and weighing it: 57 g of DI water. The discharge hose (3 ft of 1/4 in,
29 mL) and the orifice tube (0.24 mL) are on top of this when the proximal
valve is open.

Then, for every liquid-only run:

3. Retract to about 285 mm (300 mL fill), fill through the port with the
   valve open until liquid runs from the orifices with no bubbles, close
   the port and the valve. Weigh the saline added (1.005 g/mL at 20 degC)
   as `V_fill`, which should agree with `A (x_EOT - x) + V_dead` to within
   a few mL; the difference is air or an error in `V_dead`.
4. Purge dissolved and trapped gas: compress to 10 MPa in pressure mode,
   hold 5 min, and watch the position in a `hold` step (position mode): a
   pressure that falls at constant position is gas dissolving or a leak.
   Repeat until the pressure holds to within 0.05 MPa over a minute.
   The passage to the pressure transducer (about 2 cm) and the
   transducer's cavity hold air that no purge reaches and that dissolves
   only slowly through the passage; it stays in every run as a fixed gas
   pocket (`V_air` in `models/params.py`, about 0.1 mL by the geometry)
   and appears in the K fit as the floor of its gas term, the same
   volume at every fill. Air beyond it shows on the scale as a shortfall
   of the fill against the geometry, and in the hold as a pressure that
   decays at constant position while it dissolves.

`A` is 3431 mm^2 (3.431e-3 m^2, 2.602 in bore), so the piston sweeps
3.43 mL per mm.

## 3. Test 1: bulk modulus K

The chamber is liquid-only and purged (section 2). The piston moves at a
slow constant velocity while the pressure is logged against the position;
K = (dP/dx) V / A. At 300 mL and K = 1 GPa the stiffness is 11.4 MPa/mm, so
0.1 mm/s raises the pressure by 1.1 MPa/s and 3 to 10 MPa takes 0.6 mm and
6 s. The ramps end on the pressure, with the guard as the backstop.

```matlab
path = {{'ramp',  0.1, 10.0, 90}, {'dwell', 2}, ...     % up to 10 MPa at 0.1 mm/s
        {'ramp', -0.1,  3.0, 90}, {'dwell', 2}, ...     % down to 3 MPa
        {'ramp',  0.1,  6.5, 90}, {'dwell', 2}, ...     % two more pairs, so the
        {'ramp', -0.1,  3.0, 90}, {'dwell', 2}, ...     % bands 3-6.5 and 6.5-10
        {'ramp',  0.1, 10.0, 90}, {'dwell', 2}, ...     % are each covered up and
        {'ramp', -0.1,  6.5, 90}, {'dwell', 2}, ...     % down
        {'ramp',  0.1, 10.0, 90}, {'dwell', 2}, {'ramp', -0.1, 3.0, 90}};
opts.probe = true;                          % the probe records the pressure beside the velocity
r1 = charRun('K300', path, opts);           % proximal valve shut; fill 1, about 343 mL (piston at about 283 mm)
r2 = charRun('K300', path, opts);           % the same twice more: the repeatability
r3 = charRun('K300', path, opts);
V1 = 343;                                   % mL, the weighed fill (section 2 step 3), also the liquid held
rk1 = charBulkModulus({r1.file, r2.file, r3.file}, struct('fillVolume_mL', V1));
```

The dead volume is the 57 mL of `masterConfig` unless
`opts.deadVolume_mL` says otherwise. Logs recorded while
`G_cfgPressureShuntOhms` was not the present 421.4 ohm (the 26 October logs
were at 434) take `opts.logShuntOhms` with the setting they were recorded
at, which rescales their pressure; the new runs need nothing.

**Second fill.** The liquid and the series compliance separate only with
two liquid volumes more than 20 % apart. Drain to about 200 mL (the piston
at about 325 mm), purge again as in section 2 and weigh the fill, then
run the same path twice and fit all the files together with each file's
fill:

```matlab
r4 = charRun('K200', path, opts);
r5 = charRun('K200', path, opts);
V2 = 200;                                   % mL, weighed
rk = charBulkModulus({r1.file, r2.file, r3.file, r4.file, r5.file}, ...
    struct('fillVolume_mL', [V1 V1 V1 V2 V2]));
rk.split                                    % liquid K and series stiffness (printed under "Two fills")
```

**Hose stiffness.** Fill and purge the hose (the proximal valve open, the
cylinder holding the same liquid volume as in `r1`, so the piston is at
the same position, the hose and tube liquid, 29.24 mL, on top), run the
path once, then again after a thorough hose purge (it carries the slug
that degassed, 2 to 3 mL behind the 29 mL; trapped air is what the second
run removes). The compliance the hose adds is the difference of the
ramps' compliances; take the liquid in the hose out to leave the wall and
the air:

```matlab
ro1 = charRun('K300_hose', path, opts);              % valve open, hose filled
ro2 = charRun('K300_hose_purged', path, opts);       % after the hose purge
A = 3431;                                            % mm^2
C = @(f, V) A / mean([getfield(charBulkModulus(f, struct('quiet', true, 'fillVolume_mL', V)), 'dPdx_MPa_mm')]) / 1000;   % mL/MPa
Ch1 = C(ro1.file, V1 + 29.24) - C(r1.file, V1);      % mL/MPa the hose adds
Ch2 = C(ro2.file, V1 + 29.24) - C(r1.file, V1);
wall1 = Ch1 - 29.24 / (rk1.K_Pa / 1e6)               % mL/MPa beyond the hose liquid, before the purge
wall2 = Ch2 - 29.24 / (rk1.K_Pa / 1e6)               % and after
```

A mL/MPa is 1e-12 m^3/Pa: the compliance beyond the liquid goes to
`G_cfgPressureSchedHoseCompl` (a lower bound, so the smaller of the
two) and to the model's `C_hose`; 0.24 mL/MPa was the 26 October value
with the hose unpurged.

Start the path from about 2.5 MPa (reach it with a `pressure` step or a
first short ramp) so the first ramp begins in the fitted range. Close the
ball valve between the chamber and the discharge hose for the K runs and
open it for the hose runs: the difference is the hose's compliance
(section 8), which the delivery sees and the model needs as a constant
term. Record the valve's state in the label. The ramps
run in one velocity session; the drive follows a velocity step in about
60 ms whatever its size (section 8), so the 0.1 mm/s is reached within a
few scans.

The fit (`charBulkModulus`) regresses the position on the pressure in each
ramp with a liquid term and an isothermal gas term. It reports per ramp the
slope dP/dx, the effective K (liquid plus the gas, which is what the loop
sees), the liquid-only K, the gas volume the curvature implies (at the
ramp's mean pressure), the pocket's share of it and the residual. Then up
against down (hysteresis: friction, backlash or a leak), the spread, and
the `params.py` line. The liquid-only K (`K_liq_GPa`, from the linear
term; `r.K_Pa` is its mean) is the value for `params.py`'s `K`: the gas
term is the air pocket and any free gas, which the model carries
separately.

**Expected values** (300 mL, 0.1 mm/s; the script prints this table):

| K | dP/dx | dP/dt | travel 3 -> 10 MPa |
|---|---|---|---|
| 0.5 GPa | 5.7 MPa/mm | 0.57 MPa/s | 1.22 mm |
| 1.0 GPa | 11.4 MPa/mm | 1.14 MPa/s | 0.61 mm |
| 1.5 GPa | 17.2 MPa/mm | 1.72 MPa/s | 0.41 mm |
| 2.0 GPa | 22.9 MPa/mm | 2.29 MPa/s | 0.31 mm |
| 2.5 GPa | 28.6 MPa/mm | 2.86 MPa/s | 0.24 mm |

Water alone is 2.2 GPa. A good run shows a nearly straight P(x) with a
slope in this table, the same within 10 % up and down and in both pressure
bands, and a gas volume no larger than the pocket described below. A
pressure that rises more slowly at 3 MPa than at 10 MPa (upward
curvature) is gas: a bubble of 0.1 mL at 5 MPa adds 7 % to the compliance
of 300 mL at 1 GPa, which is why the fit reports it. A K below 0.5 GPa
with no gas term is the actuator, a hose or a seal flexing; see below.

**Air pocket.** The chamber always holds the air in the passage to the
pressure transducer (about 2 cm) and in the transducer's cavity, which no
purge reaches (`cfg.airVolume_mL` in `charConfig.m`, the `V_air` of
`models/params.py`). It dissolves only very slowly, so for the fit it is a
fixed isothermal gas, and the gas term has it as a floor: a pocket of
V_air at atmosphere is Patm V_air / (P0 + Patm) at a ramp's mean pressure
P0, so the fit finds its volume at P0 and not at atmosphere (`Vair_fit_mL`
brings it back). The acceptance compares the fitted gas volume at P0 with
the pocket there and passes when it is no more than 0.3 mL above it
(`opts.airTolerance_mL`); a gas term beyond that is free gas the purge
left, or a compliance that falls with pressure, which the fit cannot tell
from gas (the hold of section 2 can: gas dissolves, a compliance does
not). The pocket is the same volume at every fill, so it does not change
the slope of the compliance against V in the two-fill split; its
compliance is removed from the intercept before the series stiffness is
taken.

**Residual gas.** Dissolved gas does not change the compressibility; only
free gas does. The curvature fit bounds the free gas; the hold in section 2
shows whether it is still dissolving. Both bands should give the same K
once the gas is gone.

**What the encoder measures.** The position is the motor encoder scaled
through the gearbox and screw, so any compliance between motor and piston
(screw, nut, thrust bearing, frame) is in series with the liquid and lowers
the measured K. That is the right value for the loop, which also sees it
through the encoder, but it does not scale with the liquid volume as the
liquid term does. To separate them, repeat the ramps at a second fill
(200 mL, piston about 40 mm further in) and pass both files: the compliance
dx/dP against V gives the liquid K from the slope and the series stiffness
from the intercept (`r.split`). The series term matters at the end of a
delivery, when the liquid term is small.

**Oxygen charge (if time allows).** Admit a known charge (for example 1:1,
300 mL STP, from the supply at 0.5 to 1.4 MPa), compress to 10 MPa, wait
for the dissolution to settle (minutes), and run the same path with
`opts.chargeRatio = 1`. The script prints the model's free gas volume at
each pressure against the fitted gas term; agreement within a factor of two
supports the model's Henry's-law term with its 5 % salting-out. At 3 MPa a
1:1 charge still has free gas and the curvature is large, so fit the bands
separately.

## 4. Test 2: orifice flow and Cd

Liquid-only chamber, about 343 mL (section 2). In pressure mode at a held target the
piston stands still; when the ball valve is opened the pressure dips and
the loop drives the piston at the velocity that balances the outflow, so
Q = A v at steady state without a flow meter, and the collected volume is
the check. The first 0.24 mL after the valve opens fill the 1 mm tube and
are excluded, as is the settling of the loop.

```matlab
% Piston at about 283 mm at 10 MPa; 30 mL is 8.7 mm further in, so posStop = x at the target + 8.7
rq10 = charRun('Q10', {{'pressure', 4, 15}, {'pressure', 10, 100}}, struct('posStop', 292));
```

Approach in two stages: the loop's stop law makes the approach to 10 MPa
safe, but a first stage at 4 MPa shows the chamber is liquid-full before
the pressure gets high. Once the second target is reached and steady, open
the valve quickly and collect the discharge in a tared beaker; the run ends
when the position guard stops it (set `posStop` for the volume to deliver,
3.43 mL per mm) or after the step's seconds. Note the beaker mass, and the
run clock at opening if a watch is handy (the script otherwise finds the
opening from the velocity and the pressure dip). Refill and repeat at 7 and
4 MPa, which is also the Cd check at held pressures when K is not being
measured:

```matlab
rq7 = charRun('Q7', {{'pressure', 4, 15}, {'pressure', 7, 100}}, struct('posStop', 292));
rq4 = charRun('Q4', {{'pressure', 4, 15}, {'pressure', 4, 100}}, struct('posStop', 292));
m4 = ...; m7 = ...; m10 = ...;                  % g in the beakers, by hand
rq = charOrificeFlow({rq4.file, rq7.file, rq10.file}, ...
    struct('K_Pa', rk.K_Pa, 'fillVolume_mL', V1, 'collected_mL', [m4 m7 m10] / 1.005));
```

`K_Pa` is the effective bulk modulus of the closed chamber (section 3); the
dead volume defaults to the 57 mL of `masterConfig`.

The fit reports per steady window the pressure, Q from the piston velocity
with the compressibility correction, Cd, and the drift; sub-windows of 10 s
show whether Q holds along the stroke; the exponent of Q = c P^n over the
three pressures; the collected volume against the swept volume (with the
decompression after the stop and the tube volume accounted for); and the
`params.py` line.

**Expected values** (32 orifices of 40 um, A_o = 4.02e-8 m^2):

| P | Q, Cd 0.6 | Q, Cd 0.7 | Q, Cd 0.8 | piston at Cd 0.7 |
|---|---|---|---|---|
| 4 MPa | 2.16 mL/s | 2.52 mL/s | 2.88 mL/s | 0.73 mm/s |
| 7 MPa | 2.85 mL/s | 3.33 mL/s | 3.81 mL/s | 0.97 mm/s |
| 10 MPa | 3.41 mL/s | 3.98 mL/s | 4.55 mL/s | 1.16 mm/s |

Accept Cd between 0.5 and 0.9, an exponent of 0.4 to 0.6, a spread of Cd
over windows below 10 % and a collected volume within 5 % of the swept
volume. A Cd that falls with time or pressure points to orifices clogging;
a Cd well above 0.8 to a leak past the valve or the seals; an exponent
far from 0.5 to a flow that is not orifice-limited (a partly open valve,
or air in the tube).

The steady pressure need not equal the target: with the gains on the
machine the loop settles below the target during a delivery (design
README, "Delivery with the gains on the machine"). The fit uses the
measured pressure, so that does not matter, but the pressure must be
steady; the `settleRate` option (0.05 MPa/s) defines steady.

## 5. Test 3: drive lag tauV

The design model represents the drive's velocity loop, the gearbox and the
ball screw together as one first-order lag, after a dead time, between the
velocity command and the piston velocity (eqs. 11 and 12 of the design
README; the slew limit of eq. 11 is not in evidence on the machine, see
section 8).
The slave's velocity feedback (`G_sysActualVelocity`, from
`MC_ReadActualVelocity` in the LD POU) is the controller's axis velocity:
the motor encoder, differentiated by the drive and scaled through the
70:1 gearbox and the 5.08 mm lead. It measures the motor, so gearbox lost
motion and screw compliance are invisible to it. The test therefore has
two parts. With the chamber open (drained, or the fill port open) the
velocity feedback at the scan rate gives the lag at the motor. With the
chamber closed and full of liquid, only the piston can change the
pressure, so the pressure gives the lag at the piston.

**Open chamber: the lag at the motor.** Run it with the piston in the
bore, the service condition; a run with the piston withdrawn is only
needed if the steps from rest show far more lag than the steps taken
while moving, which would be the seals' stiction rather than the loop
(the friction bench measures that). The friction bench measures this lag
too (`bench/friction/README.md` section 4.5): `frictionRun(label, 'steps')`
with the probe enabled and `stepResponseFit` on the captures.
`charDriveLag` reads the same captures and fits the same model by least
squares, so either script gives tauV at the motor; they should agree
within a millisecond or two. The drive follows a step in about 60 ms
whatever its size (section 8), so any step from `minStep` (0.2 mm/s) to a
few mm/s is a clean sample; hold each command at least 3 s, as the friction
bench's `stepHold` does: a capture is 500 scans (2 s) and reading it takes
39 gaps (13 records a gap, 2 s), so shorter holds queue captures that are
read in the later gaps and after the last step.

Both fits model the command as slewed at `G_cfgDirectCtrlAccel` unless told
otherwise, and the machine does not slew at that value, so give them a slew
that does not limit the step (`opts.accel` and `opts.decel`, mm/s^2):
with the GVL's 10 mm/s^2 the model cannot follow the 60 ms step and
reports a lag near zero.

```matlab
rl = charDriveLag(files, struct('accel', 1e4, 'decel', 1e4));
rs = stepResponseFit(files, struct('accel', 1e4, 'decel', 1e4));
```

Repeat the open-chamber steps now that `Y_DirectControl`'s
`FilterTimeConstant` is wired from `G_cfgDirectCtrlFilterTime` (`PRG_Main`;
0.05 s in the reference: record the loaded value with the results). With
the piston in the bore and the chamber open, the probe armed (the `'steps'`
set turns it on), at two piston positions as on 26 October:

```matlab
cd ../friction
r1 = frictionRun('s1', 'steps');             % 0.3, 1 and 2.5 mm/s steps, each up and down
% move the piston, then
r2 = frictionRun('s2', 'steps');
cd ../characterisation
rl = charDriveLag([r1.capFiles r2.capFiles], struct('accel', 1e4, 'decel', 1e4));
rf = stepResponseFit([r1.capFiles r2.capFiles], struct('accel', 1e4, 'decel', 1e4));
```

For the committed 26 October captures use
`[logs '/s1_cap_steps_*.csv']` and `[logs '/s2_cap_steps_*.csv']` in place of
the file lists.

**Closed, liquid-filled chamber: the lag at the piston** (purged, at 3 to
5 MPa, from section 2): small steps, because at 11 MPa/mm a 0.2 mm/s step
for 0.3 s moves the pressure by 0.7 MPa:

```matlab
opts.probe = true;                                   % step probe enabled
path = {{'ramp', 0.1, 4.0, 60}, {'dwell', 3}, ...
        {'velstep', [0.2 0 -0.2 0 0.2 0 -0.2 0], [0.3 3 0.3 3 0.3 3 0.3 3]}};
r = charRun('lag_wet', path, opts);                  % proximal valve shut, liquid-full, purged
rl = charDriveLag([r.capFiles {r.file}], struct('accel', 1e4, 'decel', 1e4));
```

Record the valve state in the label (`lag_wet` for shut, `lag_wet_open`
for open); `rl.tauP_s` is the lag at the piston plus its dead time.

The step probe records the pressure beside the velocity, so for every
capture whose pressure moves by more than 0.1 MPa the script also fits
the pressure channel: the lag model for the piston velocity, integrated
at the capture's own stiffness (its pressure change over its position
change) and passed through the slave's median filter (two scans for a
smooth signal), the low-pass (`G_cfgAnalogFilterTimeConst`, 20 ms on the
machine, 50 ms earlier) and the two-scan pipeline, eqs. 16 to 18. The
filter constant must be the one loaded. The result is the lag at the
piston plus its dead time, at the 4 ms scan: behind the sensor chain the
two trade against each other, so the fit determines their sum, which is
also the value a model without a dead time needs. The 50 ms log is fitted
the same way as a check, to about 25 ms; the `ramp` starts of test 1 are
pressurised steps too, so the K runs with `opts.probe` already give these
points.

The fit slews the command at `opts.accel` and `opts.decel` (default the GVL's
`G_cfgDirectCtrlAccel`, which the drive does not follow: pass 1e4 as above
unless checking that), delays it and passes it through a first-order lag, and
adjusts the lag and the dead time to the data (`opts.fitAccel` also scales
the slew, as a check of the limits it was given). It prints per capture tauV, the dead time, the
residual, and the model-free times at which the feedback reaches 10 % and
63 % of the step; per log the velocity and pressure ensembles.

**Expected values.** The design study assumed 20 ms and studied 5 to
50 ms. A Sigma-7 velocity loop tuned for this load is typically 10 to
30 ms; the dead time between `G_sysCommandedVelocity` and the feedback is
two to eight scans of 4 ms (Mechatrolink out and back, plus the task order)
and should be below 30 ms. The closed-chamber value (at the piston) can exceed
the open-chamber lag plus dead time when the chamber is stiff; a
difference of more than a few milliseconds is worth carrying into the
model as tauV. Accept a capture
residual below 5 % of the step, a spread of tauV across captures below
30 %, up and down steps within 20 % of each other.

**Without the probe** the 50 ms log still has the command (the reference
as sent) and the feedback. `charDriveLag` on the log alone fits every
step as an ensemble and returns the lag plus dead time to about 25 ms,
enough to say whether 20 ms is the right order but not to separate the
two.

## 6. Test 4: stopping distance under load, in one pass

The slave's stop law caps the piston speed so that the travel it cannot
avoid once it decides to stop, the delay before the drive acts plus the
ramp down at the deceleration, never carries the pressure past the target:
it allows the speed v for which v·lag + v²/(2·D) fits in the remaining
error at the chamber stiffness (design README, eq. (41)). The lag is
measured (test 3). D is `G_cfgPressureDecel`, 50 mm/s², a value chosen
to stay clear of drive alarms, not a measurement; and test 3 found that
the drive does not slew the command at all, so the real stop is the
velocity loop pulling the piston to rest against, or with, the pressure
force. This test measures that travel against speed, load and direction,
and gets the stiffness, the drive lag and the friction under load from the
same pass.

**The pass.** A staircase of pressure levels joined by ramps at a constant
speed, with an excursion set at each level:

- the ramps between levels are stiffness measurements (test 1's fit) and
  give the torque against pressure at one speed;
- at each level a dwell of 2 s gives the transducer noise and any creep;
- then, for each speed v that fits the budget, a leg of d mm at +v, a
  stop, a leg of d mm at −v and a stop. The step from rest gives the drive
  lag under load (test 3's fit), each stop gives a stopping distance in
  compression or on the return, the steady part of each leg gives the
  torque at speed in both directions, and the pair returns the piston to
  where it started. A slow `settle` step then brings the pressure back to
  the level, since the bench's latency at the end of each leg and the gas
  dissolving leave the pair a little off it, so the level stands for the
  next speed.

`charStopPath` builds the path from the fill and the gas charge. The
chamber is saline with an air charge (2:1 by STP volume by default, from a
cylinder or a pump, not oxygen), which keeps the chamber soft enough at
pressure for the fast legs; the stopping distance depends on the load and
not on the stiffness, so the charge is free to choose. Each leg plus the
stop allowance must stay within the pressure budget at its level, 2 MPa
up to 8 MPa and 1 MPa at 10 MPa, the compliance integrated across the
excursion with the gas the liquid holds in solution at the level taken
out; a speed is left out where the steady part of its leg would be too
short. The allowance is v·(T0 + latency) + v²/(2·D): the drive's delay,
the bench's own latency (a leg ends when the master sees it in one
exchange and the stop goes out in the next, so two cycles, 50 ms on a
clean link) and the
configured D. The ramps between levels are slowed where the chamber is
stiff so that the same latency cannot overshoot a level by more than
0.5 MPa; on the first run a ramp at 2 mm/s into a 9 MPa/mm chamber with
100 ms cycles ran 0.5 mm past 8 MPa and reached 12.6 MPa. The run's guard
stays at 11 MPa. With the defaults the full speed sweep (1, 2, 5, 7 mm/s)
runs at 2 and 4 MPa, 1 and 2 mm/s at 6 and 8, and 1 mm/s at 10; 8 MPa
carries 80 % of the load, which is enough for the fit.

```matlab
opts.probe = 'stops';                                % step probe enabled: capture the stops
[path, plan] = charStopPath(struct('pause_s', 3));   % prints the budget table
r = charRun('stops', path, opts);
rs = charStopDistance([r.capFiles {r.file}]);        % D and T0 per direction
rl = charDriveLag([r.capFiles {r.file}], struct('accel', 1e4, 'decel', 1e4));   % the lag under load
rk = charBulkModulus(r.file, struct('chargeRatio', 2, 'fillVolume_mL', 343, ...
    'pressureRange', [0.5 11], 'minRows', 8, 'skip', 0.2));   % stiffness of the legs and ramps against pressure
rr = frictionFit({}, r.file, struct('settle', 0.15, 'trim', 0.05, 'accel', 1e4));   % friction at speed under load
```

The chamber is gas-charged, so `rk` prints its liquid-bulk-modulus checks as
failed by design: read its table of dP/dx against the pressure, and the
model's stiffness for the same fill and charge from `charStopPath`'s plan
(`plan(i).k_MPa_mm`). `frictionFit` takes each leg as one steady point (the
short legs need the short `settle` and `trim`) and fits the loaded law
F - P A = F0 + sign(v) (Fc + mu P A) + b v.

`charStopPath(struct('chargeRatio', 1))` plans for a 1:1 charge, which is
stiffer and drops the 5 and 7 mm/s legs above 2 MPa; `'levels_MPa'`,
`'speeds'` and `'budget_MPa'` set the staircase. The plan prints, per level,
the model's stiffness, the budget, the leg distance and the stop allowance
for each speed; about 90 s of motion for the default 26 stops.

**The fit** (`charStopDistance`). Every reference step to zero in a
velocity session is a stop: from the 50 ms log, the travel from the row
that sent the command to the first rows at rest, to about half an
exchange; from a capture, the same at the scan rate. Each stop is
reported with its speed, pressure, direction, travel, the pressure's own
excursion during the stop, the creep (further travel in the direction of
the motion over the second after the velocity feedback first reads zero,
with the pressure change it causes; the drive still moving under load
after it reports rest), the peak torque and whether a fault ended it.
Per direction, the travel is fitted to x = T0·v + v²/(2·D) with both
terms non-negative; with two or more stops at a level, D is also fitted
per level with T0 held, which shows the load dependence. Every stop is
compared with the slave's allowance at the configured D and lag; the law
is conservative where the measured travel is below it.

**Expected values.** With the drive following the command in about 60 ms
(test 3), a stop from 5 mm/s should take 0.1 to 0.2 mm in compression,
where the pressure force helps, and more on the return, where the drive
brakes against it with the torque left after holding the load (34 kN at
10 MPa is 60 % of the stall cap). The configured law allows 0.5 mm at
5 mm/s. The log resolves a stop only on a clean link (25 ms cycles); with
90 ms cycles the command's position cannot be placed to better than the
stop itself, and the fit leaves such stops out when it has better ones. Three outcomes matter: measured travel below the allowance at
every level, in which case D can be raised toward the fitted value and
every approach gets shorter; travel above it on the return at high
pressure, in which case D must come down there, or become a value per
direction; or a drive alarm before the physical limit, in which case the
alarm is the real D and the slave should carry it. Accept a fit residual
below 20 % of the mean travel, both directions fitted, and no alarm.

**Carrying it over.** `r.D_mm_s2` and `r.T0_s` are the values the stop
law needs: the smaller D and the larger T0 of the two directions. The
design model's compression and delivery studies then run with the
measured allowance in place of the configured one, since the closing of
the purge valve at the end of a delivery is also a stop against a stiff
chamber.

**At 8 and 10 MPa, 0.25 and 0.5 mm/s, with scan-rate captures.** The
slow speeds are the ones the pressure budget leaves at these loads: the
chamber is 5 to 7 MPa/mm, so 1 MPa is 0.15 to 0.2 mm, of which the bench's
own latency (the exchange that sees the leg's end, the one that sends the
stop, and the wait for the probe's arm acknowledgement: `latency_s`, 0.2 s)
takes the run-on. Fill, purge and pressurise to about 7 MPa as in section 2
(the proximal valve in the state of the delivery you want the figure for:
shut for the bare chamber, open for the chamber the delivery sees), set
`P_start_MPa` to the pressure the chamber is at when the run starts, and
run, from `bench/characterisation` in MATLAB with the controller up and the
shell stopped:

```matlab
cd bench/characterisation
addpath ../common
opts.probe = 'stops';                                % arm the probe for every command to zero
[path, plan] = charStopPath(struct('levels_MPa', [8 10], 'speeds', [0.25 0.5], ...
    'budget_MPa', [2 0.7], 'rampOvershoot_MPa', 0.2, 'minRun_mm', 0.03, ...
    'minSteady_s', 0, 'pause_s', 3, 'P_start_MPa', 7));   % prints the plan: read it before running
r = charRun('stops_hp', path, opts);
rs = charStopDistance([r.capFiles {r.file}]);        % r.T0_s and r.D_mm_s2 per direction
rl = charDriveLag([r.capFiles {r.file}], struct('accel', 1e4, 'decel', 1e4));
rr = frictionFit({}, r.file, struct('settle', 0.15, 'trim', 0.05, 'accel', 1e4));
rs.T0_s, rs.D_mm_s2, rs.fit.comp, rs.fit.ret
```

Check the plan before the run. Each level lists the leg for each speed
(`d mm`), the allowance for its stop and the predicted rise; `skipped`
means the leg does not fit the budget with the stiffness the plan assumes.
The plan assumes the 2:1 air charge in 343 mL (`chargeRatio`,
`fillVolume_mL`, the proximal valve shut). For a liquid-only chamber add
`'chargeRatio', 0` (7.2 MPa/mm with the valve shut) and, with the hose
open, `'C_hose_m3_Pa', 2.4e-13` as well (4.8 MPa/mm). Whatever the chamber,
the 0.5 mm/s leg at 10 MPa is skipped: its stop alone, over the bench's
latency, takes more than the 0.7 MPa left under the guard, and the
0.25 mm/s leg there is 0.03 to 0.1 mm. The budget at 10 MPa is 0.7 MPa
and the ramps stop within 0.2 MPa of their level because `charRun` ends
the run at 11 MPa (10 + 0.2 + 0.7 stays under it). A stop that reaches the
guard ends the run safely; rerun with the budget lowered. Six stops, about
40 s of motion.

What to look for in the outputs:

- `charRun` prints a capture for every command to zero (the six stops
  and the three ramp ends), no probe failures and an
  exchange p99 under 50 ms; `rs.stops` lists a `capture` stop for each.
  Stops from the 50 ms log are listed too and are coarse.
- `rs.fit.comp` and `rs.fit.ret` give `T0_s` (the delay before the drive
  acts, which these speeds resolve: the travel is T0 v to within a few
  micrometres) per direction. The v² term at 0.5 mm/s is under 3 µm at the
  configured D, so these speeds do not determine `D_mm_s2`: it comes
  out as whatever the micrometres of scatter in the travels make it (in
  the simulator, 81 mm/s² where the faster legs give 620), and
  `r.D_mm_s2` is not a measurement here. Take `r.T0_s`, and keep the D of
  the faster stops (the 25 ms run of section 8, no deceleration term
  resolvable) or `G_cfgPressureDecel` for the v² term. The legs are short
  (0.03 to 0.35 mm), so the drive may not have reached speed when a stop
  begins and the delay can fall a few milliseconds short.
- The delay carries the capture's one-scan offset (below): read
  `r.T0_s` as including 4 ms.

**What the capture rows are.** The probe records, in each scan, the
command of that scan and the velocity and position feedback `PRG_Main`
sees, which the axis had at the end of the previous scan. A record's
feedback is therefore one scan (`G_cfgScanTime`, 4 ms) older than the
command beside it (`bench/common/README.md`, "The step probe"). A delay
or a stopping travel read from a capture's rows, from the command row to
the row where the feedback responds, includes that scan: allow one scan when comparing it with the drive's own response, with
a model's dead time or with a figure from the 50 ms log. The fits
(`charDriveLag`, `stepResponseFit`, `charStopDistance`) report what the
rows show and do not remove it.

## 7. Carrying the results into the model

`params.py` fields and the units the scripts print them in:

| Field | Script | Unit |
|---|---|---|
| `K` | `charBulkModulus` (`r.K_Pa`) | Pa |
| `Vl0` | the fill of the runs (`r` prints it) | m^3 |
| `Cd` | `charOrificeFlow` (`r.Cd`) | - |
| `tauV` | `charDriveLag` (`r.tauV_s` at the motor; `r.tauP_s` at the piston, the value the model stands for, when it exceeds it) or `stepResponseFit` | s |
| `mr` | `charDrivePath` (`r.inertia.mass_kg`, lossless) | kg |
| `G_cfgPressureDecel` (GVL, the stop law) and the model's stop allowance | `charStopDistance` (`r.D_mm_s2`, `r.T0_s`) | mm/s², s |

`V_dead` is not a `Params` field; the delivery scenario stops at `V_empty`.
Record the measured dead volume with the results so that the stroke checks
and the master's `deadVolume` can be corrected.

## 8. Results

**The scan was 4 ms, not 2**, on the 26th and since: the task was set to 2 ms but
the program takes about 2.5 ms a scan, so it ran on every second motion cycle
(the probe captures show 4.00 ms per row from the position advance against the
velocity, and the slave's scan counter advances 250 times a second). The capture
files of the 26th carry a 2 ms time column, which the fits replace by the scan
(`opts.rowPeriod_s`, default `G_cfgScanTime`). The task and `G_cfgScanTime` are
now 4 ms on the controller and in the reference.

Logs of 2026-10-26 (`bench/captures/`), fitted with the scripts of this
bench, with a dead volume of 50 mL (the passages with the fittings of
those runs) and the fill from the geometry. The logs were recorded with `G_cfgPressureShuntOhms`
at 434 ohm, which reads the pressure as 0.971 P - 0.104 MPa
(`bench/pressure/README.md` section 6); the table gives the values as
fitted from them. Refitted with `opts.logShuntOhms = 434`, which rescales
every logged pressure to the calibrated 421.4 ohm, K (chamber closed)
rises 3.0 % (0.736 to 0.758 GPa with the default options; 0.72 becomes
0.74, the gas-corrected 0.81 becomes 0.83), the hose-open stiffness
rises from 5.2 to 5.4 MPa/mm, and Cd falls from 0.85 to 0.83 (exponent
unchanged at 0.48, with no `pressureOffset`). `params.py` carries the
values as fitted until the tests are repeated with the calibrated input.

| Parameter | Result | Runs | Notes |
|---|---|---|---|
| K, chamber closed off from the hose (proximal valve shut) | 0.72 GPa, segments 0.67 to 0.78; stiffness 7.2 MPa/mm at 343 mL | `K300_char` x3 at 280 mm, 19:26 to 19:30 | Below the 1 GPa assumed. Up and down agree to 0.2 %, but the curvature gives a gas-like term of up to 0.65 mL, 23 % of the compliance, so the liquid-only K is not pinned: gas-corrected, 0.81 GPa. The gas term is a volume at the ramps' mean pressure; as gas it would be about 40 mL at atmosphere, more than the machine holds, and far more than the transducer passage's pocket (section 2, about 0.1 mL), so the curvature it fits may not be gas at all: a compliance that falls with pressure looks the same. The weighed fill and the hold at 10 MPa say whether there is gas; the K at two fill levels gives the series term and pins the liquid value. |
| Discharge hose open (3 ft of 1/4 in R7 hose, 29 mL, to the orifice tube) | stiffness 5.2 MPa/mm with 305 mL of liquid; as an effective K, 0.46 GPa | `K300_char` at 299 mm, 19:36 | The hose adds about 0.24 mL/MPa of compliance beyond its own liquid (2.4 mL per 10 MPa): its wall stretching and any air it holds, the latter being what the curvature's gas term of up to 1.1 mL in this run points at. This is the chamber the delivery sees. |
| Cd | 0.85 (0.84 to 0.87 over 1.6 to 8.3 MPa), exponent 0.48; Q = 4.8 mL/s at 10 MPa, piston 1.42 mm/s | `d2`, `d5`, `d10` deliveries and `Q10_char` x2 | With the transducer zero of -0.11 MPa corrected (`pressureOffset`). Above the 0.7 assumed: deliveries run 20 % faster than the model. The loop held 8.1 MPa for a 10 MPa target, the steady error the design study predicted. |
| tauV at the motor | 18.2 +/- 0.03 ms lag after 22.8 +/- 0.02 ms dead time (sum 41 ms), the same for 0.3, 1 and 2.5 mm/s steps, up and down; `stepResponseFit`'s grid gives 16 ms after 28 ms | `s1`, `s2` step captures, 74 | **The command is not slewed at `G_cfgDirectCtrlAccel`**: a 2.5 mm/s step is complete in 60 ms, as is a 0.3 mm/s step, so the drive follows the command at whatever rate it needs (18 mm/s^2 for a 1 mm/s step, 45 for 2.5) with a 2 to 5 % overshoot. Fit with `opts.accel` large (1e4) and the 4 ms row spacing, from several starting dead times; with the GVL slew the model cannot follow and reports a lag near zero. |
| tauV at the piston | not resolved (below 25 ms) | `lag_wet_char` 50 ms logs, 5 steps at 8.6 to 10 MPa/mm | No captures were taken in these runs (`opts.probe` off, probe without the pressure array). Repeat with the reloaded probe. |
| Stopping distance under load (test 4) | 0.031 s × v in both directions, 2 to 10 MPa, 1 to 6 mm/s, residual 4 µm; no deceleration term resolvable (the drive stops within a scan or two once it acts) and no travel after rest | `stops_char_20261030_232726` (16 stops, 25 ms cycles) | The slave's allowance v·0.05 s + v²/(2·50 mm/s²) is above every stop, by 3 to 4 times at 5 mm/s. The 31 ms is 10 ms less than the drive's 22.8 ms dead time and 18.2 ms lag of the velocity steps, so the stop is not explained by the step response plus the command pipeline and Modbus tick; the stop reaches rest earlier than a first-order lag after a dead time would, and the difference is not resolved. Two later runs with 90 ms cycles (the probe's register group mapped) do not resolve the stops. |
| Friction at speed under load (`frictionFit` on the legs of the one-pass runs) | Fc 4.4 +/- 0.5 kN, b 2.7e6 +/- 0.2e6 N s/m, mu 0.41 +/- 0.02 (efficiency 0.71 pushing, 0.59 pushed), F0 -1.0 kN, 60 legs | `stops_char_20261030_232726` (13 legs, 25 ms cycles; 6.2 +/- 1.6 kN, 2.4e6, mu 0.36 +/- 0.07), `stops_char_20261031_001105` (21; 4.3 kN, 2.8e6, 0.43), `stops_char_20261031_001956` (26; 4.1 kN, 2.5e6, 0.40) | Each leg of the path (1 to 5 mm/s on the 25 ms run, 0.25 to 2 mm/s on the others, 0.3 to 10 MPa, both directions) is one steady point; `frictionFit({}, logs, struct('logShuntOhms', 434, 'settle', 0.15, 'trim', 0.05, 'accel', 1e4))`. The fit is the loaded law F - P A = F0 + sign(v) (Fc + mu P A) + b v. The pooled values agree with the friction bench's own fit of the same day's runs (Coulomb 6.1 kN, viscous 2.3e6 N s/m, mu 0.45) on b and mu. The pressure rises 1 to 2 MPa through a leg, so the force drifts by 3 to 20 kN across it and `frictionFit` flags the legs `drift` and `0.1 lead` (a leg is a fraction of a screw lead); the residual of the fit is 1 to 1.5 kN and Fc scatters from 4.1 to 6.2 kN between the runs. The two `stops_hp_char` runs (20261031_004938, _012956; 10 legs each, 8 to 10 MPa, 90 ms cycles, the legs 0.2 to 0.5 s) give Fc 0.9 and 1.9 kN, b 5.7e6 and 4.3e6, mu 0.50 and 0.48 and are not used: at 90 ms the legs hold two to five samples and the velocity does not reach the command. Breakaway 15 to 22 kN from rest. |
| Stiffness against pressure (`charBulkModulus` with `chargeRatio` 2 on the legs and ramps of the one-pass runs) | dP/dx 0.9 MPa/mm at 3.3 MPa, 2.0 at 4.7, 2.9 at 6.3, 3.6 at 7.2, 5.2 at 9.5 | `stops_char_20261030_232726` (9 ramps), `stops_char_20261031_001105`, `_001956`, `stops_hp_char_20261031_004938`, `_012956` | `charBulkModulus(file, struct('logShuntOhms', 434, 'chargeRatio', 2, 'fillVolume_mL', 193, 'pressureRange', [0.5 11], 'minRows', 8, 'skip', 0.2))`; the five runs give the same curve to 15 % (up and down, 2.2 to 11 MPa). The piston stood at 325 to 328 mm from 3 to 10 MPa, which puts the liquid at about 193 mL, not the 343 mL the path assumes (`fillVolume_mL`); the model's stiffness for 193 mL and a 2:1 charge is 1.2, 2.3, 3.7, 4.5 and 6.0 MPa/mm at those pressures with the hose open (0.24 mL/MPa) and 1.3, 2.8, 5.1, 6.5 and 10.4 with it shut, so the chamber is 0.75 to 0.87 of the hose-open model (the valve state is not logged). The curvature fit gives 8 to 13 mL of free gas at 3.3 MPa against the model's 7.8, and 4 to 5 mL at 6.3 MPa and 1.9 mL at 9.6 MPa against the model's 0.8 and 0: the gas dissolves more slowly than the saturation model assumes. The liquid-bulk-modulus checks fail by design for a gas-charged chamber (K lin 0.05 to 0.5 GPa). |
| Reflected inertia (test 5) | 0.418 % of rated torque per mm/s^2 overall, 0.38, 0.41 and 0.44 at the steps of 0.3, 1 and 2.5 mm/s: 4.6e5 kg at the piston with the lossless constant | `s1`, `s2` step captures, 74 | `charDrivePath` (section 10.1): the torque excess over the steady torque after a step against the motor acceleration. The 0.3 mm/s steps scatter (acceleration 7 mm/s^2 against a torque noise of 1 %: interquartile range 0.12 to 0.51), the 1 and 2.5 mm/s ones do not (0.34 to 0.45, 0.42 to 0.46). An efficiency below 1 lowers the mass at the piston by that factor. `mr` in `params.py`. |
| Friction reversal under load (test 5) | 17 to 18 % of rated torque at 4 MPa rising to 29 to 32 % at 9 MPa (19 to 20 kN, then 32 to 35 kN lossless); dT = 6.9 + 2.54 P % gives Fc 3.8 kN, mu 0.41 | `K300_char_20261026_192617`, `_192751`, `_193002` | The torque of an up ramp minus that of the next down ramp at the same pressure (0.1 mm/s, 7 pairs a log); medians of the logs 25.1, 21.6 and 22.5 %. Agrees with `frictionFit` on mu (0.41 to 0.45); its Fc includes the viscous term at 0.1 mm/s. |
| Drive path stiffness (test 5) | k_s > 4.7 kN/um (4.3 % of rated torque per um): the motor position at the same pressure differs by 1.1, 2.3 and 2.6 um (median) between an up and a down ramp, through a 22 to 25 % force reversal; over 200 times the chamber's 24 N/um | the three K300 runs | A bound and not a value: the offsets are within the 5 um the logs resolve, and the pressure sensor's lag adds to them. The hose-open run `K300_char_20261026_193603` has offsets of -25 to -48 um (the hose, not the drive: the contrast). The model carries no compliance between the motor and the piston. |
| Backlash, motor-side friction, windup (test 5) | not measured | - | The October logs have no direct reversal. Sections 10.2 (open chamber) and 10.3 (dither) are the runs. |

**Two dead volumes.** The proximal valve at the cylinder head is shut
during compression and mixing and open during the delivery, so the
chamber has two dead volumes: the cylinder passages alone, the measured
57 mL (`masterConfig.deadVolume`), and the passages plus the hose (29 mL)
and the tube (0.24 mL), 86 mL, during the delivery. The hose is prefilled
with virgin saline, and the delivery's first 29 mL is that purge, which
does not change Cd but is not solution.

**What the hose means for the model.** A delivery runs with the hose
open, so the compliance the loop sees is the liquid's plus a constant
0.24 mL/MPa from the hose that does not shrink as the saline leaves. With
the closed chamber's K, the stiffness during a delivery is about 5 MPa/mm
with 300 mL left, 8 with 100 mL and 11 with 30 mL, against 8, 25 and 82
for the liquid alone. The model's chamber capacitance, eq. (4), needs that
constant term; with it the stiffening at the end of a delivery that the
design study found destabilising is an eighth of what the liquid-only
model predicts. The `lag_wet` runs (hose state not recorded, 283 to
285 mm) show 8.6 to 10 MPa/mm over their short steps, closer to the
closed value.

The friction bench's own fits on the same day's runs (`frictionFit`,
`stepResponseFit`): Coulomb 6.1 kN, viscous 2.3e6 N s/m (11 kN at
5 mm/s), load term mu 0.45 (efficiency 0.69 pushing), breakaway about
24 kN; all far above the model's 500 N and 5e3 N s/m. The transducer reads
-0.11 MPa with the chamber open in every log.

## 9. Checking the pressure schedule's stiffness bound

The slave's gain schedule (`FB_PressureSchedule`, off by default behind
`G_cfgPressureScheduleEnable`) sets Kp, Ki and the stop-law limit from an
upper bound on the chamber stiffness computed from the configured
settings and the position and pressure (design README, eqs. (39) to
(41)). The bound is an FB output that no register publishes, so the bench
cannot read it from the machine; `charScheduleCheck` computes it from the
same GVL settings at the pressure and position of every ramp of a log
(`charScheduleBound` is that formula, held equal to the compiled block by
`tests/slavePressureMode.m`) and compares it with the ramp's measured
dP/dx. It is the first step of commissioning the schedule: the bound must
not be below the stiffness, or the scheduled gain crosses over above its
`G_cfgPressureSchedCrossover` (11 rad/s).

Run it on the K runs of section 3 (the proximal valve shut, `hoseOpen`
false, the saline volume the master would send in HR9-10 for the fill, no
oxygen), then on the hose-open runs (`hoseOpen` true), and on a charged
run with the charge the master would send in HR11-12:

```matlab
cd bench/characterisation
addpath ../common
V1 = 343;                                   % mL, the weighed fill: HR9-10 (a lower bound on the true volume)
sc  = charScheduleCheck({r1.file, r2.file, r3.file}, struct('salineVolume_mL', V1));
sch = charScheduleCheck(ro1.file, struct('salineVolume_mL', V1 + 29.24, 'hoseOpen', true));
scg = charScheduleCheck(rc.file, struct('salineVolume_mL', V1, 'oxygenRatio', 1, 'bulk', ...
    struct('pressureRange', [0.5 11], 'minRows', 8)));          % rc: a charged run (section 3, oxygen charge)
```

For the committed 26 October logs (recorded at 434 ohm, with the 50 mL
dead volume they were fitted with; the shut-valve runs and the hose-open
run separately, each with its own `hoseOpen`):

```matlab
logs = '../captures';
bulk = struct('logShuntOhms', 434, 'deadVolume_mL', 50);
sc  = charScheduleCheck({[logs '/K300_char_20261026_192617.csv'], ...
    [logs '/K300_char_20261026_192751.csv'], [logs '/K300_char_20261026_193002.csv']}, ...
    struct('bulk', bulk));
sch = charScheduleCheck([logs '/K300_char_20261026_193603.csv'], struct('hoseOpen', true, 'bulk', bulk));
```

The table has one row per ramp: the measured dP/dx,
the bound at the ramp's top pressure and position, their ratio, the
scheduled Kp and the loop crossover Kp times the measured stiffness
(rad/s). `r.pass` is true when the ratio is at least `opts.margin`
(1) on every ramp; `r.rows` and `r.minRatio` hold the numbers, `r.lag_s`
the delay the stop law assumes (compare `charStopDistance`'s `r.T0_s`).
`opts.settings` replaces a GVL value by name (for example
`struct('G_cfgPressureSchedBulkModulus', 2e9)`) to see what a change
would do before it is loaded.

**Reading it.** A ratio above 1 is a bound above the stiffness: the
schedule's gain is below the one that would cross over at 11 rad/s, by
that ratio. A ratio below 1 on any ramp fails the check: the configured
bulk modulus is below the true one, or the saline volume or the end of
travel is set above the machine's. The result on the 26 October K300 runs
(valve shut, 343 mL, `G_cfgPressureSchedBulkModulus` 1 GPa) is a bound of
10.0 MPa/mm against 6.9 to 8.0 measured (ratio 1.24 to 1.45), loop
crossover 7.6 to 8.8 rad/s against 11; with the hose open at 299 mm a
bound of 7.9 MPa/mm against 4.8 to 5.9 (ratio 1.32 to 1.62). The schedule
is conservative on both by 25 to 60 %, the measured bulk modulus being
0.72 GPa against the 1 GPa configured.

## 10. Test 5: drive path

The slave's position, velocity and torque are the motor's: the encoder
and the drive current, scaled through the 70:1 gearbox and the 5.08 mm
lead to piston units (`G_cfgGearRatio`, `G_cfgLeadScrewPitch`). The
whole drive path lies between that measurement and the chamber: the
gearbox, the ball screw and nut, the rod and the piston seals. The only
measurement on the piston's side is the chamber pressure. With the
chamber closed and liquid-filled the pressure is a force sensor on the
piston (P A, A = 3431 mm^2), so the motor's position and torque against
the pressure identify the path:

| Quantity | What it is | Where it shows |
|---|---|---|
| Reflected inertia `mr` | motor and gearbox inertia at the piston, as the torque the motor needs per mm/s^2 | the torque excess over its steady value during a velocity step |
| Friction reversal | the torque difference between motion up and motion down at the same pressure, 2 (Fc + mu P A) | the K ramps, torque at the same pressure on an up and the next down ramp |
| Stiffness `k_s` | the motor-to-piston deflection per force | the motor position at the same pressure on the up and the down ramp (through the friction reversal), and the motor travel while the torque changes at a reversal in the dither |
| Backlash `b` | the free motor travel when the force through the path changes sign | a travel at near-constant torque at a reversal, hidden while the pressure force exceeds the friction |
| Motor-side friction `f_m` | the part of the friction that never reaches the path | the torque on the flat segment of the backlash traverse, where the path carries no force |

Every force is the torque (percent of rated) times the lossless constant
of `frictionBenchConfig` (1100 N per percent: the force at the piston
through a drivetrain with no loss), which the fit prints; so that an
efficiency can be applied later, the inertia is also given in percent per
mm/s^2, the friction reversal in percent and the stiffness in percent per
um. A force at the piston is the efficiency times the lossless one for the
motor pushing the piston and the lossless one divided by it for the motor
being pushed.

`charDrivePath` takes any mix of the files below (names, patterns or
cells, as the other fits do) and fits what each gives:

| Files | Gives |
|---|---|
| open-chamber step captures (`frictionRun('s1', 'steps')`, the first captures of the reversal run) | the inertia, per step size and overall |
| 50 ms logs of the K ramps (section 3), liquid-filled, proximal valve shut | the friction reversal against pressure and a bound on the stiffness |
| reversal captures, open chamber (below) | backlash, motor-side friction, and the stiffness if the windup is resolved |
| dither captures, closed liquid-filled chamber (below) | backlash, windup and stiffness (or a bound) against the pressure at the reversal |

### 10.1 What the October logs already give

```matlab
cd bench/characterisation
addpath ../common
logs = '../captures';
steps = {[logs '/s1_cap_steps_*.csv'], [logs '/s2_cap_steps_*.csv']};
rp = charDrivePath(steps);                                   % inertia
rk = charDrivePath({[logs '/K300_char_20261026_192617.csv'], ...
    [logs '/K300_char_20261026_192751.csv'], [logs '/K300_char_20261026_193002.csv']});   % friction reversal, stiffness bound
rh = charDrivePath([logs '/K300_char_20261026_193603.csv']); % the hose-open run, as the contrast
```

1. **Inertia.** For each step of at least 0.25 mm/s the torque excess over
   the steady torque 100 to 240 ms after the command is regressed through
   the origin on the motor acceleration (the gradient of the velocity
   feedback, one row per 4 ms scan whatever the `t` column says) over the
   160 ms after the command, on the rows with an acceleration above
   5 mm/s^2. The 74 step captures give 0.418 % of rated torque per mm/s^2
   overall: 0.38 at the steps of 0.3 mm/s (the acceleration is 7 mm/s^2
   and the torque noise 1 %, the interquartile range 0.12 to 0.51), 0.41
   at 1 mm/s and 0.44 at 2.5 mm/s (interquartile range 0.42 to 0.46).
   That is a reflected mass of 4.6e5 kg at the piston with the lossless
   constant (`models/params.py` `mr`).
2. **Friction reversal under load.** The segments of constant velocity
   sign of a K log (more than 20 rows, 50 ms each) are paired with the
   next of opposite direction; the torque of the up ramp minus that of the
   down ramp is interpolated at the same pressure on 4 to 9 MPa in steps
   of 0.5 MPa, and the median taken over the pairs. In all three K300 runs
   it rises from 17 to 18 % of rated torque at 4 MPa (19 to 20 kN
   lossless) to 29 to 32 % at 9 MPa (32 to 35 kN), the median of each log
   being 25.1, 21.6 and 22.5 % (27.6, 23.7 and 24.7 kN). The pooled fit
   dT = 6.9 + 2.54 P % (P in MPa) is 2 Fc + 2 mu P A with Fc 3.8 kN and mu
   0.41 on the lossless constant, against `frictionFit`'s 4.4 to 6.1 kN
   and 0.41 to 0.45 (the ramp speed is 0.1 mm/s, so the viscous term adds
   little here).
3. **Stiffness bound.** The same pairs give the motor position of the up
   ramp minus that of the down ramp at the same pressure: 0 to 10 um,
   median 1.1, 2.3 and 2.6 um (the +10 um at 6.5 MPa is the boundary of
   the ramp bands of the K path). The force reversal passes the path, so
   it deflects less than the offset, k_s > dT / max(|dx|, 5 um): 5.5, 4.7
   and 4.9 kN/um (5.0, 4.3 and 4.5 % of rated torque per um), over 200
   times the chamber's 24 N/um. The 5 um floor (`opts.floor_um`) is what
   the logs resolve: the pair-to-pair scatter, and the pressure sensor's
   lag (36 ms: median 5, low-pass 20 ms, two scans of pipeline), which at
   0.1 mm/s moves the up and down ramps 7 um apart in the same direction a
   stiff path would, so it raises the offset and the bound stays valid.
   The bound takes the whole torque reversal as passing the path, which
   holds if the friction is at the piston's side of it; the motor's own
   friction `f_m` (10.2) is subtracted twice from the reversal in
   `r.stiffness.boundPath_N_per_um` once the reversal run has measured it.
   The hose-open run (`K300_char_20261026_193603`) has offsets of -25 to
   -48 um, in the opposite sense (apart from -9 um at the band boundary,
   6.5 MPa): the hose and the air in it, not the drive. The fit reports them as above the floor and fails its
   `offset below floor` check; it is the contrast and not a result for the
   path.

The October data has no direct reversal at a speed, so it has no backlash
or motor-side friction: the steps from rest show the torque rising before
the motor moves (the stiction), and the 8 to 9 um of motor travel the K
logs show between a reversal and the first 0.05 MPa of pressure change is
what the pressure sensor's lag (36 ms) and the 50 ms rows give at 0.1 mm/s.

### 10.2 Reversal, open chamber (session 0)

Direct reversals from +v to -v with no stop, at 0.3, 1 and 2.5 mm/s, each
command held 2 s, the probe armed at every command change. The piston is
in the bore with the chamber open (fill port open or drained), 5 mm clear
of the position guard in the direction of the first command, after the
drive steps of section 5 (their captures give the inertia in the same
call).

```matlab
cd bench/characterisation
addpath ../common
opts.probe = true;                               % armed at every command change
[path, info] = charDriveRecipe('reversal');      % 27 captures, 54 s, 5 mm of travel
r = charRun('rev', path, opts);
rd = charDrivePath([r1.capFiles r2.capFiles r.capFiles]);    % r1, r2: the steps of section 5
rd.reversal.backlash_um, rd.reversal.motorFriction_kN        % NaN and a bound when none is resolved
% the K ramps' stiffness bound with the motor's own friction out of the reversal:
rk = charDrivePath({r1.file, r2.file, r3.file}, struct('motorFriction_pct', rd.reversal.motorFriction_pct));
rk.stiffness.boundPath_N_per_um
```

`charDriveRecipe('reversal')` is one `velstep` per speed, joined: for each
speed `[+v -v +v -v +v -v +v -v 0]` held 2 s each (seven reversals, then
the stop), so the explicit path is

```matlab
v = [0.3 1 2.5];  cmd = [];
for k = 1:3, cmd = [cmd v(k) * [1 -1 1 -1 1 -1 1 -1] 0]; end
path = {{'velstep', cmd, 2 * ones(size(cmd))}};
```

The commands alternate, so the piston ends where it began. The first
capture of each speed is a step from rest and the last a stop: both go
into the inertia.

**The fit's method.** The first command change of each capture is the
reversal (the capture starts at the arm, about 100 ms before the
command, and lasts 2 s). With J from the steps (or `opts.inertia_pct`;
`charConfig` has the October value), the torque is corrected for the
inertia, T - J a, with a the slope over five rows, and oriented along the
old direction. Its level before the command, L1, is the median of the
rows before it, and the level after, L2, the median from 0.3 s on. phi =
(L1 - T) / (L1 - L2) goes from 0 at the old level to 1 at the new. The
motor's turn is the extremum of its position, and the travel xi since it
(the abscissa, so that the speed profile of the lag drops out; at 0.3 mm/s
a row is 1 um) is where phi is analysed. A path with backlash b and a
stiff spring carries, in order: the friction of the motor flips at the
turn (phi jumps to f_m / (f_m + f_p)), the path unwinds (a ramp of f_p /
k_s of travel), the motor crosses the gap on its own friction alone (a
flat segment of length b, at the level where the path carries nothing:
phi = (2 f_m + f_p) / (2 f_m + 2 f_p), between 0.5 and 1), and the path
winds up on the other side (a ramp to 1). The fit takes the rows to
where phi last sits below 0.85 (or three times the noise below 1,
`opts.beta`), within 80 um, and fits that transition by least squares: a
ramp, a flat segment, a ramp and 1 after, with the three breakpoints
searched on a grid and refined, against the same without a flat segment
(one ramp). A flat segment counts as backlash when it is at least 4 um
long, holds three rows and the model without it fits twice as badly. The
level of the flat segment is the torque of the motor alone, f_m = -s T
there. The windup is the rest of the transition's travel, and the
stiffness the torque change over it, (1 - phi(0)) (L1 - L2), per
travel; below 3 um (`opts.res_um`) of windup the stiffness is a bound at
3 um. The backlash is reported when more than half the reversals show it,
else the travel of the whole transition bounds it
(`r.reversal.backlashBound_um`).

What limits it: the torque ripple, 1 % at the motor turn and its
harmonics (the lead is 72.6 um per motor turn), the noise of the inertia
correction (J known to 10 % leaves a tenth of the peak inertial torque,
0.5 % at 0.3 mm/s and 5 % at 2.5 mm/s) and the row spacing at the speed. The
0.3 mm/s reversals are the clean ones; the table prints the groups by
speed, with the share that showed a plateau, and the flat segment is
trusted where the speeds agree.

### 10.3 Dither, closed liquid chamber (session 1)

Triangles of about +-0.15 mm at 0.2 mm/s (legs of 1.5 s), ten cycles,
about mean pressures of 1, 5 and 9 MPa: a `ramp` to the pressure, then the
`velstep` train, the probe armed at every command change. At each
reversal the torque changes by the friction reversal while the pressure
stands still (the piston sticks); the motor travel during it over the
torque change is 1/k_s, at about 1 um resolution. At 1 MPa the pressure
force (3.4 kN) is below the friction, so the force through the path
passes zero and backlash appears; at 5 and 9 MPa it does not.

```matlab
opts.probe = true;
[path, info] = charDriveRecipe('dither');        % levels 1, 5 and 9 MPa, 75 captures, 105 s
% the pressure swings by +-0.15 mm x 7.2 MPa/mm = 1.1 MPa about each level; the call errors if
% the top comes within 0.5 MPa of the 11 MPa guard (opts.stiffness_MPa_mm, opts.levels_MPa)
r = charRun('dither', path, opts);               % the liquid-full, purged chamber of the K runs, proximal valve shut
rd = charDrivePath(r.capFiles);
rd.dither.groups                                 % per pressure and direction: backlash, windup, stiffness
```

`charDriveRecipe('dither')` is, for each level P, `{'ramp', 0.1, P, 240}`,
`{'dwell', 2}` and `{'velstep', [-0.2 repmat([0.2 -0.2], 1, 10) 0.2 0],
[0.75 1.5 * ones(1, 20) 0.75 2]}` (half a leg down, ten up-and-down
cycles, half a leg back, the stop). The ramp goes up or down to reach each
level from the one before (`opts.P_start_MPa`, 0 by default).

**The fit** is the same transition analysis, with the chamber closed. A
capture lasts 2 s and the legs 1.5 s, so each holds the next reversal too;
only the first is analysed. The late torque is regressed on the pressure
and evaluated at the reversal's pressure (the extremum of the pressure
about the reversal), because the load-dependent friction changes with it
along the leg. The reversals are grouped by their pressure (bins of 1 MPa)
and direction, so the top and the bottom turn of each triangle are
separate rows: the table has the torque reversal, the travel of the whole
transition, the backlash (the flat segment, only where the majority of
the group shows it), the windup, the stiffness or its bound and the
pressure change over the transition one sensor lag later, which says the
chamber stood still. The windup over the torque change gives
k_s = (1 - phi(0)) (L1 - L2) newtonPerPct / windup, the part of the
reversal outside the motor's own friction flip; a windup within 3 um is
reported as the bound at 3 um, which is below the stiffness when the true
windup is within it too.

### 10.4 What the fit returns

| Field | Contents |
|---|---|
| `r.inertia` | `.sizes` (per step size: n, J in % per mm/s^2 with its quartiles, mass in kg), `.J_pct_per_mm_s2`, `.mass_kg`, `.steps` |
| `r.friction` | `.logs` (per K log: pairs, torque reversal and up/down offset on the grid, medians, bound), `.P_MPa`, `.dT_pct`, `.dF_kN`, `.dX_um` (median over the logs), `.a_pct` and `.b_pct_per_MPa` (dT = a + b P), `.Fc_N`, `.mu` |
| `r.stiffness` | `.bound_N_per_um`, `.bound_pct_per_um` (the weakest log), `.boundPath_N_per_um` (the motor-side friction out), `.resolved` (an offset above the floor) |
| `r.reversal` | `.rec` (per capture), `.groups` (per speed), `.backlash_um` (NaN when none), `.backlashBound_um`, `.motorFriction_pct`, `.motorFriction_kN` |
| `r.dither` | `.rec`, `.groups` (per pressure and direction), `.backlash_um`, `.backlashBound_um` |
| `r.checks`, `r.pass` | the named checks: the inertia's spread (interquartile range over the median at 1 mm/s and more, below 0.35) and sign, the repeat of the torque reversal across the K logs (spread over mean below 0.3), its rise with the pressure, the up/down offset within the floor (the hose-open log fails this one), and the share of captures whose reversal the noise lets through (0.7) |

`opts` has the thresholds (`floor_um`, `res_um`, `minBacklash_um`,
`hingeRatio`, `inertia_pct`, `motorFriction_pct`, `logShuntOhms`, ...;
`help charDrivePath`). The logs of the 26 October runs were recorded at
434 ohm, which the pressure grid of the K logs hardly notices (3 %); pass
`opts.logShuntOhms = 434` to rescale them.

**Carrying it over.** `r.inertia.mass_kg` is `mr` in `models/params.py`
(lossless; with an efficiency e the mass at the piston is e times it for
the motor pushing the piston); the stiffness bound justifies the model
having no compliance between the motor and the piston (`models/README.md`,
"Measured values"). A backlash or a windup that the dither resolves
would add a dead zone and a spring to the plant's drive and to the
chamber's effective stiffness at the end of a delivery, where the
chamber is stiffest.
