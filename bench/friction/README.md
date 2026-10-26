# Servo Axis Friction Bench

Measures the friction of the compressor's servo axis (Tolomatic RSA64
driven by the Sigma-7 through the MP2600iec) from the torque feedback the
slave reports over Modbus, unloaded and under chamber pressure, so that the
pressure loop design model (`design/pressure-loop/params.py`: `b`, `Fc`,
`Fcap`, `veps`) can use measured values in place of its guesses. The
operator app and the shell do not show torque; the run script below takes
the shell's place as Modbus master, as the pressure noise bench does, and
logs every exchange.

| File | Contents |
|------|----------|
| `frictionBenchConfig.m` | Torque to force conversion, speed points and durations, limits the run enforces, fit windows |
| `frictionRun.m` | The master for a run: drives a set of jog steps or a pressure-mode delivery through `masterStep` at the 50 ms exchange and logs position, velocity, torque and pressure to `captures/` |
| `frictionFit.m` | Reads the logs, checks each step, fits the friction unloaded and loaded, prints the model values |
| `slave/PRG_StepProbe.st` | Optional slave program: on request captures 500 scans of commanded velocity, velocity, torque, position and pressure in the probe registers |
| `stepResponseFit.m` | Reads the captures: the drive's velocity-loop lag from steps taken while moving, the breakaway and stick time from steps from rest, the torque ripple at the lead and motor periods |
| `captures/` | Run logs and captures (gitignored) |

The pressure loop's other parameters (bulk modulus, orifice flow, drive lag
at the piston) have their own bench in `bench/characterisation/`, which
runs through the same protocol, writes the same log and capture formats to
this `captures/` directory and uses the same step probe.

The friction sets need nothing on the slave beyond `PRG_Main` and nothing
in `master-src`: the run uses the protocol registers
(`docs/master/ModbusRegisterMap.md`) in the modes the slave already has,
velocity mode (3) and pressure mode (7). The step probe (section 4.5) adds
a bench program beside `PRG_Main`, as the pressure noise bench does.
`tests/frictionRunSim` runs the scripts and the fits against the simulated
slave with a synthetic friction model.

## 1. Conversion and assumptions

The slave reports the motor torque in percent of rated (`G_sysActualTorque`,
IR15-16). The bench converts every sample to the force the motor would
exert at the piston through a lossless gearbox and screw:

    F [N] = T [%] / 100 * 1.27 N m * 2 pi * 70 / 5.08e-3 m = T * 1099.6 N

with the rated torque of the SGM7J-04A (1.27 N m), the gear ratio
(`G_cfgGearRatio`, 70) and the screw lead (`G_cfgLeadScrewPitch`, 5.08 mm).
So 100 % is 110 kN. The design model's `Fcap` is this at an assumed
drivetrain efficiency of 0.85, 93.5 kN.

No efficiency is assumed in the fits. Losses in the gearbox and screw show
up as friction that grows with the load, and the loaded set measures that
as a term `mu * P * A` (section 5). The efficiency pushing against the
load is then `1 / (1 + mu)` and `Fcap` becomes `110 kN / (1 + mu)`.

The pressure force on the piston is `P * A` with `A = 3.431e-3 m^2` (bore
2.602 in): 3431 N per MPa, 3.1 % of rated. At 10 MPa the pressure alone
takes 31 % of rated torque, at 12 MPa 37 %, before friction.

Positive position, velocity and torque point into the cylinder
(compression). The pressure pushes the piston out, so during a delivery the
motor torque is `P * A` plus friction, and when the piston retracts under
a gas charge the pressure helps and the torque is `P * A` minus friction.
That reversal is what separates the Coulomb term from the load-dependent
one.

## 2. What the slave allows

The run stays inside the slave's configuration
(`slave-src/GVL/GlobalVariables_Reference.st`); check the values on the
machine before the first run.

| Setting | Value | Effect on the bench |
|---------|-------|---------------------|
| `G_cfgVelLimitMax`, `masterConfig.jogVelMax` | 5 mm/s | The highest speed point. 7 mm/s needs both raised, which this bench does not do |
| `G_cfgDirectCtrlAccel` / `Decel` | 5 mm/s² | Velocity-mode slew: the 5 mm/s step spends 1 s and 2.5 mm reaching speed |
| `G_cfgTorqueLimitNormal`, `G_cfgPressureTorqueLimit` | 100 % | The axis stalls there, which is 29 MPa on a sealed chamber; the run stops the axis at 90 % (`torqueAbort`) |
| `G_cfgPosSoftLimitMax` / `Min` | 305 / -2.5 mm (reference file) | `FAULT_POSITION` beyond them. `masterConfig.posMax` is 367 mm, so the machine's GVL may differ: read it and keep every step, with its slew, inside |
| `G_cfgPosCylinderEntry` + `G_cfgPosExitGuardMargin` | 220 mm | Below this, positive torque above 60 % while moving out is `FAULT_PISTON_EXIT`. Loaded steps stay above it |
| `G_cfgHoldPositionTimeout` | 2 min | The slave returns to idle between steps only after this; the run's stops are seconds, so it does not apply |

The slave has no pressure ceiling of its own in velocity mode. The run
enforces one (`pMax`, by set: 0.3 MPa unloaded, 3.5 MPa under gas,
11.5 MPa for a delivery, under the 12 MPa mechanical ceiling) and sends
Stop the exchange it is passed.

## 3. Setup

Slave: `PRG_Main` loaded and running as in service, Modbus server configured
as in `docs/master/ModbusRegisterMap.md`. No bench program is needed.

Lab PC: `cfg.address` in `bench/modbus/modbusBenchConfig.m` set to the
controller IP; `SafetyHold.slx` built once with `bench/pressure/buildSafetyHold`
and set to Connected IO (see `bench/pressure/README.md` section 2). **The
shell model must be stopped** for every run: a second Modbus client beside
it delays the shell's exchange and the slave raises `FAULT_COMM` while
motion is commanded. Use the operator app to place the axis, then
Disconnect.

```matlab
cd bench/friction
r = frictionRun('u1');                         % unloaded set
r = frictionRun('g1', 'gas');                  % under a gas charge
r = frictionRun('d10', 'delivery', struct('target', 10, 'xStop', 275));
```

The run prints each step as it starts, the mean velocity, torque and
pressure of every step at the end, and saves
`captures/<label>_<set>_<yyyymmdd_HHMMSS>.csv` with one row per exchange
(every 50 ms):

| Column | Unit | Content |
|--------|------|---------|
| `t` | s | Time since the run started |
| `step` | | Step number in the set, 0 before the first |
| `status` | | `masterStateMachine` status code: 4 jogging, 5 in pressure mode, 10 fault |
| `mode` | | Slave mode of the step: 3 velocity, 7 pressure, 2 position |
| `ref` | mm/s, MPa, mm | HR5-6 as sent that exchange: the velocity command, the pressure target or the position target |
| `position` | mm | IR11-12 |
| `velocity` | mm/s | IR13-14 |
| `torque` | % rated | IR15-16 |
| `pressure` | MPa gauge | IR19-20 |
| `inMotion`, `inBand` | | IR6 and IR7 |
| `exchangeMs` | ms | Duration of the write plus read |

The run ends early, with the axis stopped, on a slave fault, link loss, an
unconfirmed mode, a pressure above `pMax` or a torque above `torqueAbort`;
the reason is printed and returned in `r.abort`. Ctrl-C stops the axis
too. A log that ends early is still usable: the fit takes the steps it
holds.

## 4. Runs

### 4.1 Unloaded

Piston in the bore, chamber vented to atmosphere (outlet open, no liquid)
so that `pressure` stays near zero; the run stops if it exceeds 0.3 MPa.
Place the axis with at least 60 mm of travel toward the soft limit and
above 220 mm, for example at 235 mm with the reference limits. The set
jogs at 5, 3, 1, 0.3 and 0.1 mm/s, each speed into the cylinder and then
out for the same time, then the same speeds ascending, so every speed is
measured twice in each direction at different times:

| Speed (mm/s) | At speed (s) | Travel per step (mm) | Screw leads |
|---|---|---|---|
| 0.1 | 60 | 6 | 1.2 |
| 0.3 | 40 | 12 | 2.4 |
| 1 | 20 | 20 | 4 |
| 3 | 10 | 32 | 6 |
| 5 | 10 | 55 | 11 |

Each step adds the slew to speed (`v / 5 mm/s²`) and is followed by a
stop of a few seconds before the reverse step, so a pair returns to its
start. Both passes take about 11 minutes. Every step starts from rest, so
the breakaway is recorded 20 times.

```matlab
r = frictionRun('u1');
r = frictionRun('u1', 'unloaded', struct('startPosition', 235));   % position step first
```

Run it a second time with the piston withdrawn below the cylinder entry
(below 200 mm, for example from 60 mm) with the label `bare`: that log is
the gearbox, screw and brake drag without the seals, and the difference
between the two unloaded fits is the seal friction.

### 4.2 Under a gas charge

A sealed liquid-only chamber cannot move at steady speed: at 11 MPa/mm
one scan of travel at 1 mm/s is 0.02 MPa, and a millimetre is the whole
ceiling. A gas charge makes the chamber soft enough to move through:
with `V` mL of free gas at absolute pressure `P0`, a stroke `dx` takes the
pressure to `P0 * V / (V - 3.431 * dx)`. Charge the chamber as for a run
(step 3 of the process, stir bar off), to 0.5 to 1 MPa gauge, with as
little liquid as the procedure allows so that the free gas volume is
large; with 300 mL of gas at 0.5 MPa gauge a 20 mm stroke raises the
pressure by 0.2 MPa, and the full 0.5 to 3 MPa window allows 70 mm.
Position the piston in the bore above 220 mm with 25 mm free toward the
limit.

```matlab
r = frictionRun('g1', 'gas');
```

The set is the unloaded one with every step shortened to 20 mm of travel
(`gasStrokeMax`): the 3 and 5 mm/s steps last 6.7 and 4 s at speed. The
pressure is logged with every sample, so it need not be constant; the fit
uses `P * A` sample by sample. The run stops the axis above 3.5 MPa. If
the charge dissolves into liquid during the run the pressure falls; that
is fine for the fit, but keep the gas window by recharging between
passes if it drops below 0.5 MPa. About 10 minutes for both passes.

Points at the same speed in both directions at the same pressure are
what separates the Coulomb term from the load term, so do not skip the
return steps.

### 4.3 During a delivery

With the outlet open the orifice plate sets the flow: 32 orifices of
40 µm pass about 4.0 mL/s at 10 MPa (`Cd` 0.7), which moves the piston at
1.16 mm/s; the flow falls with the square root of the pressure, 0.82 mm/s
at 5 MPa and 0.58 mm/s at 2.5 MPa. The slave's pressure loop holds the
pressure, so the run only asks for the mode and logs. One direction only:
the load term and the Coulomb term separate across targets, so run three
deliveries, at 10, 5 and 2.5 MPa, each from a fresh fill. Saline alone,
without an oxygen charge, is enough for the friction and avoids the
degassing question; a charged chamber gives the same points.

Fill as for a run and note the piston position at which the chamber is
full of liquid: 300 mL is 87 mm of stroke. Set `xStop` so that the
delivery ends with at least 100 mL (30 mm) left: with the present gains
the loop loses control in the last 10 mL as the chamber stiffens past
80 MPa/mm (`design/pressure-loop/README.md`), and the run must not reach
that. The run also stops the axis above 11.5 MPa or 90 % torque.

```matlab
r = frictionRun('d10', 'delivery', struct('target', 10, 'xStop', 275));
r = frictionRun('d5',  'delivery', struct('target', 5,  'xStop', 275));
r = frictionRun('d2',  'delivery', struct('target', 2.5, 'xStop', 275));
```

The run commands pressure mode at the target and prints `in band: open the
outlet valve` when the slave reports the pressure within 0.1 MPa of it.
Open the valve then. The piston runs until `xStop`, the step's `seconds`
(default 120) or a limit; close the valve after the stop. The compression
before the valve opens is logged too and gives loaded points at the
charge pressure when the chamber holds gas.

### 4.4 Custom paths

`frictionRun(label, path)` takes the step grammar of `pressureNoiseRun`
plus the velocity-step type: `{'jog', mm_per_s, s}`,
`{'stepv', mm_per_s, hold}`, `{'position', mm, s}`, `{'pressure', MPa, s}`,
`{'idle', s}`. Use it to repeat one speed, or to add 7 mm/s once the
limits allow it. Velocity modes are entered at rest and the velocity is
sent once the mode runs, so every jog step starts with a command step
from standing.

### 4.5 Velocity steps at the scan rate

The 50 ms log cannot show the drive's velocity-loop lag (`tauV`, 20 ms in
the model), a breakaway shorter than one exchange, or the torque ripple
at the motor turn. `PRG_StepProbe` records 500 consecutive scans (1 s at
2 ms) of the commanded velocity, the actual velocity, the torque, the
position and the chamber pressure into input registers when the run
writes its arm register, and the run reads the block out in the gaps
between exchanges, as the pressure noise bench does. The pressure channel
serves the characterisation bench, which reads the lag at the piston from
it (`bench/characterisation/README.md` section 5).

Slave setup, once:

1. Hardware Configuration, online: the Modbus server as in service, with
   the **FC#04 input register group enlarged to at least 3410 registers**
   (byte range `%QB73792` to `%QB80611`). Save and cycle power. If the
   group cannot be made that large, shorten the capture: change the
   bounds and the count in the program header together, and
   `cfg.probe.nScan` in `frictionBenchConfig.m`.
2. Declare the globals in the header of `slave/PRG_StepProbe.st` in the
   matching groups. The five arrays are located; if the editor refuses an
   array at a direct address, declare it with a named type
   (`ARRAY[0..499] OF INT`, `ARRAY[0..499] OF DINT`) from the Data Types
   worksheet.
3. Add `PRG_StepProbe` to the project and assign it to the 2 ms cyclic
   task. It reads `G_sys*` globals only and runs beside `PRG_Main`. Do not
   load it together with `PRG_PressureProbe` or `PRG_ModbusBench`.
4. Compile, download, start.

Then, with the piston placed as for the unloaded set:

```matlab
r = frictionRun('s1', 'steps');                 % stepv at 0.3, 1 and 2.5 mm/s
r = frictionRun('u1', 'unloaded', struct('probe', true));   % captures in the jog set too
```

A `stepv` step enters velocity mode at rest and steps the reference
through 0, v, 2v, v, 0, −v, −2v, −v, 0, holding each level `stepHold`
seconds (3 s; the capture and its readout take 2 s), with a capture armed
in the exchange before every change. The steps between v and 2v are
taken while moving and give the lag; the steps from 0 give the breakaway
and the stick time. With `opts.probe` on a jog set, every step's command
from rest is captured, plus one capture a second after it reaches speed
for the ripple. 2v must stay within the 5 mm/s limit, so v is at most
2.5 mm/s.

Captures are saved as `captures/<label>_cap_<set>_<n>_<step>_<tag>_<timestamp>.csv`
with `t` (s from the first scan), `vCmd` and `velocity` (mm/s), `torque`
(% rated), `position` (mm) and `pressure` (MPa gauge); the tag is
`t<k>_<v>` for the k-th transition to v, or `steady`.

```matlab
f = stepResponseFit('captures/s1_cap_*.csv');
```

For each capture taken while moving it fits the response as the command
slewed at `G_cfgDirectCtrlAccel` (the profile Y_DirectControl generates),
through a first-order lag `tauV` and a delay, by a grid search on the rms
velocity error, and also reports the model-free tracking lag at mid-step
and the overshoot. For each capture from rest it reports the breakaway
force and the stick time. For each steady capture it reports the ripple
and its amplitude at the lead period and the motor turn. It ends with
`tauV` for `params.py`, the median over the moving steps.

Acceptance: the moving steps should give the same `tauV` within a few
milliseconds up and down and at both speeds; the rms error should be a
few hundredths of a millimetre per second; a delay above about 10 ms
or an overshoot above a few percent means the drive's velocity loop is
not the first-order lag the model assumes, and the captures should go
to the design model as they are. The breakaway should agree with the
50 ms log's lower bound and the stick time with the ramp: at 5 mm/s² the
command reaches 0.1 mm/s in 20 ms, so a stick of several tenths of a
second is the friction, not the profile. If the measured lag changes
with the step size, the loop is saturating; repeat at the smaller step.

## 5. Post-processing

```matlab
f = frictionFit('captures/u1_unloaded_*.csv');
f = frictionFit('captures/u1_unloaded_*.csv', ...
                {'captures/g1_gas_*.csv', 'captures/d*_delivery_*.csv'});
```

The first argument is the unloaded log or logs, the second the loaded
ones (gas sets and deliveries); either may be a pattern or a cell array.
The fit prints, in order:

1. **One line per jog step**: the commanded and measured speed (from the
   position slope), the mean pressure, the screw leads covered, the mean
   force, its ripple (sample standard deviation), its drift (slope times
   duration), the fraction of samples at rest, and the checks. The steady
   part of a step starts 1 s after the command has reached speed and
   ends 0.2 s before the stop.
2. **Breakaway** per step: the largest friction force (the pressure force
   at the sample removed) between the command and the first sample moving
   faster than 0.05 mm/s, and its ratio to the sliding friction of that
   step. The log is sampled every 50 ms, so a breakaway
   shorter than that shows as fewer than three samples and is a lower
   bound; the 0.1 mm/s steps hold the axis still longest and are the ones
   to trust.
3. **Reversal pairs**: for each in-then-out pair at the same speed, half
   the force step `(F+ - F-) / 2`, which is the Coulomb friction plus the
   load term, and the offset `(F+ + F-) / 2 - P A`, which should be the
   same constant `F0` at every speed.
4. **The unloaded fit** `F = F0 + Fc sign(v) + b v` over the step means,
   and the Stribeck form `F = F0 + sign(v) (Fc + (Fs - Fc) exp(-(|v|/vs)^2)) + b v`
   for a grid of knee speeds `vs`. The Stribeck form is kept when it cuts
   the residual by a fifth with a positive falling term.
5. **The loaded fit** `F - P A = F0 + sign(v) (Fc + mu P A) + b v` over
   the gas step means and the delivery bins (1 s of samples moving faster
   than 0.05 mm/s and accelerating slower than 0.5 mm/s²). When the
   points cannot separate `mu`, `b` is fixed to the unloaded value; with
   one direction at one pressure `mu` is fixed to zero and the printed
   `Fc` includes the load term at that pressure, which the output says.
6. **The values for `params.py`**: `b` in N s/m, `Fc` in N, `Fcap` at
   100 % through the measured efficiency, a note on `veps`, the loaded
   values and the breakaway. When the Stribeck form is kept its `Fc` and
   `b` are the ones printed, since the plain form's slope is then biased
   by the falling region.

Standard errors come from the residuals; the per-point residuals are
printed so that an outlying step can be found and the run repeated.

### Acceptance checks

A step is good when its line reads `ok`:

- **stationary**: the drift over the step is within the ripple or 3 % of
  the force. A drifting step was warming up or the pressure was moving;
  repeat it, or for the gas set accept it if the pressure column explains
  it.
- **at least one lead**: the step covered one screw lead (5.08 mm), so
  the ripple at the lead period averages out. Shorter steps are reported
  with their lead count.
- **at speed**: the measured velocity is within 0.05 mm/s or 10 % of the
  command. Otherwise the axis was limited or stalled.
- **stick-slip**: more than a tenth of the samples at rest while
  commanded. Expected only, if at all, at 0.1 mm/s; it means the
  friction at that speed is not a single value and the model needs the
  Stribeck term and a higher `veps`.
- **symmetric reversal**: each pair's offset lies within a tenth of its
  step from the fit's `F0`. A pair that fails has a step that was not
  steady or a pressure that changed between the two.

For the whole set: the two passes should agree at every speed within the
ripple; the unloaded fit's residual rms should be a few hundred newtons
(a few tenths of a percent of rated); `mu` should be positive and small
(0.1 to 0.3 for a ball screw and planetary gearbox); the three
deliveries should lie on one line in `F - P A` against `P A`.

## 6. Reading the results

- **`Fc` and `b`** go straight into `params.py`. The model's `Fc` of 500 N
  and `b` of 5 kN s/m were guesses; the drive's own unloaded baseline
  recorded during homing trials was about 8 % of rated, 9 kN, so expect
  `Fc` well above the guess.
- **`mu`** sets the efficiency and therefore `Fcap`. It also says how much
  of the 10 MPa load the motor carries as loss: at `mu` 0.18 the delivery
  takes 37 % of rated instead of 31 %.
- **A Stribeck region** (force falling from the breakaway to the sliding
  value over the first few tenths of a millimetre per second) means the
  loop's low-speed behaviour at the 1.2 mm/s delivery speed depends on
  where that knee is. The model has no falling term; the fit prints the
  knee speed so that one can be added if it lies near the delivery
  speed.
- **A large `F0`** is an offset the model does not have: the piston's
  weight if the axis is not horizontal, or a torque zero error. It
  cancels in the reversal step and does not enter `Fc`.
- **Different unloaded values with the piston in and out of the bore**:
  the difference is the seal friction, the part that may change with
  pressure, wear and lubrication.

## 7. Not covered

- 7 mm/s: `G_cfgVelLimitMax` and `masterConfig.jogVelMax` are 5 mm/s. The
  pressure loop's own clamp is 5 mm/s, so the model's operating range is
  covered.
- A velocity step without the 5 mm/s² slew. The lag is identified from
  the slewed response, which is what the model also sees; a sharper step
  needs `G_cfgDirectCtrlAccel` raised on the machine for the run, or the
  SERVOPACK's own trace in SigmaWin+.
- The pressure force in the captures: `stepResponseFit` reports the
  breakaway from the torque alone, so from a capture under load it
  includes `P A`. The capture carries the pressure; use the 50 ms log's
  breakaway, which removes it, for loaded starts.

## 8. Results

| Label | Condition | Fc (N) | b (N s/m) | Fs, vs | mu | Breakaway (N) | tauV (ms) | Notes |
|-------|-----------|--------|-----------|--------|----|---------------|-----------|-------|
| | | | | | | | | |
