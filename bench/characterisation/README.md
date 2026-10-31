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

Everything is measured through the protocol the master already uses: the
slave reports position, velocity, torque and pressure in IR11-20 every 50 ms
(`docs/master/ModbusRegisterMap.md`). The drive lag alone needs the 2 ms
scan, which the shared step probe (`bench/common/slave/PRG_StepProbe.st`,
already on the controller) provides. Nothing in `slave-src/` changes.

This bench shares the friction bench's conventions: the same Modbus master
pattern (`bench/common/`), the same log columns and capture format, the
same `bench/common/captures/` directory and the same slave program. Its run
script differs in what the steps can do: ramps that end on a pressure,
command changes within one mode session without a stop, and pressure and
position guards.

| File | Contents |
|------|----------|
| `charRun.m` | The master for a run: drives a path of steps through `masterStep` as the shell does, logs every exchange in `frictionRun`'s format, stops the axis when the pressure or the position leaves the envelope, and reads the step probe between exchanges |
| `charConfig.m` | Chamber and outlet geometry, the pressure limits and the slave settings read from `slave-src/GVL/GlobalVariables_Reference.st` |
| `charBulkModulus.m` | Test 1: K from slow ramps, with a bound on residual gas and, from two fills, the split between liquid and series compliance |
| `charOrificeFlow.m` | Test 2: Cd and the square-root law from deliveries at held pressure, with the collected-volume check |
| `charDriveLag.m` | Test 3: tauV and dead time from the step probe captures at the motor and, through their pressure channel, at the piston; the 50 ms log as a coarse check |
| `charStopPath.m` | Test 4: builds the one-pass path, a staircase of pressure levels with an excursion set at each, sized to a pressure budget from the fill and the gas charge |
| `charStopDistance.m` | Test 4: the stopping distance of every stop in the run, from the captures and the log, fitted to the stop law's two terms per direction and compared with the slave's allowance |

`tests/charRunSim.m` runs `charRun` and the fits against the simulated slave
(through the friction bench's `fakeProbeLink`), the one-pass path included,
`tests/charFitsSynthetic.m` the fits on synthetic logs with known
parameters; `tests/run_all.sh` runs both.

## 0. What the friction bench's runs already give

The friction bench's logs are in the same format, so two of the three
parameters come out of runs made for it:

| Friction bench run | Gives | Command |
|---|---|---|
| `'delivery'` at 10, 5 and 2.5 MPa | Cd and the square-root law (test 2) | `charOrificeFlow({'../common/captures/d10_delivery_*.csv', ...}, struct('K_Pa', ..., 'collected_mL', [...]))`; leave `collected_mL` out if the discharge was not weighed |
| `'steps'` captures | tauV at the motor (test 3, open chamber) | `stepResponseFit` or `charDriveLag` on the capture files |

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
r = charRun('label', path);              % see sections 3 to 5 for the paths
```

A fault still latched from an earlier run is cleared by `charRun` with the
operator reset before its first step. The controller answers a Modbus
request in 1 to 8 ms (`bench/modbus`), so the protocol exchange, one write
and one read, should take under 20 ms; the run prints any exchange over
150 ms, skips the probe transaction in that cycle, and at the end gives
the write and read times separately. The slave's watchdog faults the run
with `FAULT_COMM` (code 4) if the heartbeat stops for 300 ms
(`G_cfgCommTimeout`), so exchanges of 200 ms end it. `bench/modbus/modbusRawProbe`
times the transactions on their own, without the master, and is the
first check: 20 ms median and 30 ms p99 for the sequential exchange is
the controller's normal. The step probe's register group in Hardware
Configuration more than doubles that whether or not the probe program
runs (`bench/common/README.md` step 3); a second Modbus client (the shell
model or the operator app still connected, another MATLAB session, an
OPC client) and the network path are the other causes.

The run ends with the axis stopped, prints a table of the steps and saves
`../common/captures/<label>_char_<timestamp>.csv` (columns as in
`bench/common/README.md`) and one
`<label>_cap_<n>_<step>_<timestamp>.csv` per probe capture (columns `t`,
`vCmd`, `velocity`, `torque`, `position`, as the friction bench's captures).
Ctrl-C stops the axis the same way. The guards are `charConfig.pressureStop`
(11.0 MPa, below the 12 MPa ceiling) and the position band inside the
slave's soft limits; `opts.pressureStop`, `opts.posStop` and `opts.posMin`
override them for a run and cannot be set above the ceiling.

### Slave

No change for tests 1 and 2. Test 3 uses `PRG_StepProbe`, set up as in
`bench/friction/README.md` section 4.5; `charRun` arms it before every
command change in a velocity session when `opts.probe` is true. Without it,
run test 3 anyway: the 50 ms log gives the lag to about 25 ms.

### Settings that matter

`charConfig` reads the GVL reference, which must match the controller:
velocity-mode slew (`G_cfgDirectCtrlAccel`, `G_cfgDirectCtrlDecel`), the
velocity clamp, the soft limits, the pressure filter (`G_cfgAnalogFilterTimeConst`,
`G_cfgAnalogMedianSize`) and the scan time. The ramps and steps below stay
within `G_cfgVelLimitMax`, the torque limits (34 kN at 10 MPa is about 37 %
of rated torque through the 70:1 gearbox) and the soft limits; the position
guard stops a delivery 3 mm before the positive soft limit, so the soft limit
must allow the piston to approach the end of travel if a delivery is to run
to the end. The pressure target in the operator app is in atm absolute; this
bench works in MPa gauge throughout, as the slave does.

## 2. Volumes: dead volume and fill

Both fits need the liquid volume. The dead volume, what the piston cannot
sweep, was measured at 57 mL (57 g of water, with the present fittings) and is
`masterConfig.deadVolume`. To measure it again:

1. Jog the piston to the end of travel (the position guard allows up to
   3 mm inside the soft limit; note the position `x_EOT` reached and compare
   it with `masterConfig.posEOT`).
2. With the outlet valve open and the fill port open, fill the chamber from
   a burette or syringe until liquid appears at the orifices. The volume
   added is the dead volume `V_dead`.

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
r = charRun('K300', path);                  % add opts.probe = true with the step probe loaded
rk = charBulkModulus(r.file, struct('deadVolume_mL', 25, 'fillVolume_mL', 300));
```

Start the path from about 2.5 MPa (reach it with a `pressure` step or a
first short ramp) so the first ramp begins in the fitted range. The ramps
run in one velocity session; the slave slews the command at
`G_cfgDirectCtrlAccel`, so the 0.1 mm/s is reached within a few scans.

The fit (`charBulkModulus`) regresses the position on the pressure in each
ramp with a liquid term and an isothermal gas term. It reports per ramp the
slope dP/dx, the effective K (liquid plus whatever gas is left, which is
what the loop sees), the liquid-only K, the gas volume the curvature
implies, and the residual. Then up against down (hysteresis: friction,
backlash or a leak), the spread, and the `params.py` line.

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
bands, and a gas volume below 0.05 mL. A pressure that rises more slowly at
3 MPa than at 10 MPa (upward curvature) is residual gas: a bubble of 0.1 mL
at 5 MPa adds 7 % to the compliance of 300 mL at 1 GPa, which is why the
fit bounds it and the acceptance limits its share to 5 %. A K below 0.5 GPa
with no gas term is the actuator, a hose or a seal flexing; see below.

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
(150 mL, piston about 40 mm further in) and pass both files: the compliance
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

Liquid-only chamber, about 300 mL. In pressure mode at a held target the
piston stands still; when the ball valve is opened the pressure dips and
the loop drives the piston at the velocity that balances the outflow, so
Q = A v at steady state without a flow meter, and the collected volume is
the check. The first 0.24 mL after the valve opens fill the 1 mm tube and
are excluded, as is the settling of the loop.

```matlab
r = charRun('Q10', {{'pressure', 4, 15}, {'pressure', 10, 100}}, struct('posStop', 280));
```

Approach in two stages: the loop's stop law makes the approach to 10 MPa
safe, but a first stage at 4 MPa shows the chamber is liquid-full before
the pressure gets high. Once the second target is reached and steady, open
the valve quickly and collect the discharge in a tared beaker; the run ends
when the position guard stops it (set `posStop` for the volume to deliver,
3.43 mL per mm) or after the step's seconds. Note the beaker mass, and the
run clock at opening if a watch is handy (the script otherwise finds the
opening from the velocity and the pressure dip). Refill and repeat at 7 and
4 MPa.

```matlab
rq = charOrificeFlow({'../common/captures/Q4_char_...csv', '../common/captures/Q7_char_...csv', ...
    '../common/captures/Q10_char_...csv'}, ...
    struct('K_Pa', rk.K_Pa, 'deadVolume_mL', 25, 'fillVolume_mL', 300, 'collected_mL', [m4 m7 m10] / 1.005));
```

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
ball screw together as one first-order lag between the slewed velocity
command and the piston velocity (eqs. 11 and 12 of the design README).
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
with `PRG_StepProbe` loaded and `stepResponseFit` on the captures.
`charDriveLag` reads the same captures and fits the same model by least
squares, so either script gives tauV at the motor; they should agree
within a millisecond or two. Keep the step
size at or below 0.2 s times the slew (`G_cfgDirectCtrlAccel`: 1 mm/s at
5 mm/s^2) so the ramp fits in the capture, and hold each command at least
3 s so the capture (1 s) and its readout (1 s) finish before the next
change. Raising the slew to the pressure-mode values (50 mm/s^2) for the
test separates the lag from the ramp more sharply.

**Closed, liquid-filled chamber: the lag at the piston** (purged, at 3 to
5 MPa, from section 2): small steps, because at 11 MPa/mm a 0.2 mm/s step
for 0.3 s moves the pressure by 0.7 MPa:

```matlab
opts.probe = true;                                   % PRG_StepProbe loaded
path = {{'ramp', 0.1, 4.0, 60}, {'dwell', 3}, ...
        {'velstep', [0.2 0 -0.2 0 0.2 0 -0.2 0], [0.3 3 0.3 3 0.3 3 0.3 3]}};
r = charRun('lag_wet', path, opts);
rl = charDriveLag([r.capFiles {r.file}]);
```

`PRG_StepProbe` records the pressure beside the velocity, so for every
capture whose pressure moves by more than 0.1 MPa the script also fits
the pressure channel: the lag model for the piston velocity, integrated
at the capture's own stiffness (its pressure change over its position
change) and passed through the slave's median filter (two scans for a
smooth signal), the low-pass (`G_cfgAnalogFilterTimeConst`, 20 ms on the
machine, 50 ms earlier) and the two-scan pipeline, eqs. 16 to 18. The
filter constant must be the one loaded. The result is the lag at the
piston plus its dead time, at the 2 ms scan: behind the sensor chain the
two trade against each other, so the fit determines their sum, which is
also the value a model without a dead time needs. The 50 ms log is fitted
the same way as a check, to about 25 ms; the `ramp` starts of test 1 are
pressurised steps too, so the K runs with `opts.probe` already give these
points.

The fit slews the command at the GVL limits, delays it and passes it
through a first-order lag, and adjusts the lag and the dead time to the
data (`opts.fitAccel` also scales the slew, as a check that the drive uses
the configured limits). It prints per capture tauV, the dead time, the
residual, and the model-free times at which the feedback reaches 10 % and
63 % of the step; per log the velocity and pressure ensembles.

**Expected values.** The design study assumed 20 ms and studied 5 to
50 ms. A Sigma-7 velocity loop tuned for this load is typically 10 to
30 ms; the dead time between `G_sysCommandedVelocity` and the feedback is
two to four scans (Mechatrolink out and back, plus the task order) and
should be below 20 ms. The closed-chamber value (at the piston) can exceed
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
clean link and 200 ms with the probe's register group mapped) and the
configured D. The ramps between levels are slowed where the chamber is
stiff so that the same latency cannot overshoot a level by more than
0.5 MPa; on the first run a ramp at 2 mm/s into a 9 MPa/mm chamber with
100 ms cycles ran 0.5 mm past 8 MPa and reached 12.6 MPa. The run's guard
stays at 11 MPa. With the defaults the full speed sweep (1, 2, 5, 7 mm/s)
runs at 2 and 4 MPa, 1 and 2 mm/s at 6 and 8, and 1 mm/s at 10; 8 MPa
carries 80 % of the load, which is enough for the fit.

```matlab
opts.probe = 'stops';                                % PRG_StepProbe loaded: capture the stops
[path, plan] = charStopPath(struct('pause_s', 3));   % prints the budget table
r = charRun('stops', path, opts);
rs = charStopDistance([r.capFiles {r.file}]);        % D and T0 per direction
rl = charDriveLag([r.capFiles {r.file}]);            % the lag under load
rk = charBulkModulus(r.file, struct('chargeRatio', 2));   % the ramps
```

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

**Expected values.** With the drive following the command in about 30 ms
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

## 7. Carrying the results into the model

`params.py` fields and the units the scripts print them in:

| Field | Script | Unit |
|---|---|---|
| `K` | `charBulkModulus` (`r.K_Pa`) | Pa |
| `Vl0` | the fill of the runs (`r` prints it) | m^3 |
| `Cd` | `charOrificeFlow` (`r.Cd`) | - |
| `tauV` | `charDriveLag` (`r.tauV_s` at the motor; `r.tauP_s` at the piston, the value the model stands for, when it exceeds it) or `stepResponseFit` | s |
| `G_cfgPressureDecel` (GVL, the stop law) and the model's stop allowance | `charStopDistance` (`r.D_mm_s2`, `r.T0_s`) | mm/s², s |

`V_dead` is not a `Params` field; the delivery scenario stops at `V_empty`.
Record the measured dead volume with the results so that the stroke checks
and the master's `deadVolume` can be corrected.

## 8. Results

Logs of 2026-10-26 (`bench/captures/` on the `bench-tests` branch), fitted
with the scripts of this bench. Dead volume 50 mL (measured) and the fill
from the geometry.

| Parameter | Result | Runs | Notes |
|---|---|---|---|
| K (effective, at the piston) | 0.72 GPa, segments 0.67 to 0.78; stiffness 7.2 MPa/mm at 343 mL | `K300_char` x3 at 280 mm | Below the 1 GPa assumed. Up and down agree to 0.2 %, but the curvature gives a gas-like term of up to 0.65 mL, 23 % of the compliance, so the liquid-only K is not pinned: the gas-corrected value is 0.81 GPa. A fourth run at 299 mm (276 mL) came out softer, 0.42 GPa, which no liquid chamber does; that fill held more gas. Purge longer (section 2 step 4) and weigh the fill. |
| Cd | 0.85 (0.84 to 0.87 over 1.6 to 8.3 MPa), exponent 0.48; Q = 4.8 mL/s at 10 MPa, piston 1.42 mm/s | `d2`, `d5`, `d10` deliveries and `Q10_char` x2 | With the transducer zero of -0.11 MPa corrected (`pressureOffset`). Above the 0.7 assumed: deliveries run 20 % faster than the model. The loop held 8.1 MPa for a 10 MPa target, the steady error the design study predicted. |
| tauV at the motor | 8.7 +/- 0.5 ms lag after 12 +/- 0.6 ms dead time, the same for 0.3, 1 and 2.5 mm/s steps, up and down | `s1`, `s2` step captures, 74 | **The command is not slewed at `G_cfgDirectCtrlAccel`**: a 2.5 mm/s step is complete in 30 ms, as is a 0.3 mm/s step, so the drive follows the command at whatever rate it needs (36 mm/s^2 for a 1 mm/s step, 90 for 2.5) with a 2 to 5 % overshoot. Fit with `opts.accel` large (1e4); with the GVL slew the model cannot follow and reports a lag near zero. |
| tauV at the piston | not resolved (below 25 ms) | `lag_wet_char` 50 ms logs, 5 steps at 8.6 to 10 MPa/mm | No captures were taken in these runs (`opts.probe` off, probe without the pressure array). Repeat with the reloaded probe. |

| Stopping distance under load (test 4) | 0.031 s × v in both directions, 2 to 10 MPa, 1 to 6 mm/s, residual 4 µm; no deceleration term resolvable (the drive stops within a scan or two once it acts) and no travel after rest | `stops_char_20261030_232726` (16 stops, 25 ms cycles) | The slave's allowance v·0.05 s + v²/(2·50 mm/s²) is above every stop, by 3 to 4 times at 5 mm/s. The 31 ms is the drive's 12 ms dead time and 8.7 ms lag plus the slave's command pipeline and Modbus tick, the components the design model's lag already carries. Two later runs with 90 ms cycles (the probe's register group mapped) do not resolve the stops. |

The friction bench's own fits on the same day's runs (`frictionFit`,
`stepResponseFit`): Coulomb 6.1 kN, viscous 2.3e6 N s/m (11 kN at
5 mm/s), load term mu 0.45 (efficiency 0.69 pushing), breakaway about
24 kN; all far above the model's 500 N and 5e3 N s/m. The transducer reads
-0.11 MPa with the chamber open in every log.
