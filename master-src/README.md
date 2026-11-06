# Master

The master runs on the lab PC under Simulink Desktop Real-Time and talks to the
MP2600iec slave over Modbus TCP (`docs/master/ModbusRegisterMap.md`), with the
safety relay and the stir bar motor on the NI PCI-6221 DAQ. The pressure
transducer is on the slave, which closes the pressure loop and reports the
reading over the link.

| Path | Contents |
|------|----------|
| `logic/` | All master logic, as plain `.m` functions that run under MATLAB and GNU Octave. `masterStep.m` is the entry point called once per model step; `masterStateMachine.m` sequences the handshake, motion and fault handling; `masterConfig.m` holds every constant. `proc*.m` is the operator app's procedure engine (`procTrackSeries` and `procTrackCheck` sample and check a track step's series) and `opTrack.m` the rule by which the app and the engine follow an operation through `StatusCode`. `trackStep.m` plays the track table inside the model (the `TrackStep` block). |
| `procedure/` | The procedure the operator app walks through in non-manual mode, as data: `sopProcedure.m`. The SOP replaces this file; no code changes. `experimentTemplate.m` is a procedure to copy from: every kind of step once, with a comment on its fields, and a track step and the record intervals. |
| `trace/` | The trace recorder's host-side functions, plain `.m` files that run under MATLAB and GNU Octave: `traceStart`, `traceDrain` and `traceAppend` (collect records from the ring), `traceAlignNew`/`traceAlign` (give each row the slave summary of its own window), `traceDecimateNew`/`traceDecimate` and `traceIntervalExchanges` (combine exchanges into the record interval), `traceLiveBuffer`, `traceLiveAppend`, `traceLiveNew` and `traceLiveUpdate` (the live plot), `traceMeta` and `traceWriteCsv` (write a trace file), `traceReadCsv` and `traceDerive` (read it back and add the displayed quantities), `tracePlot` (three figures) and `traceColumnNames`. The block that fills the ring, `traceStep`, and the ring's layout, `traceLayout`, are in `logic/`. |
| `captures/` | Trace files the app saves (gitignored; created on first use). |
| `shell/buildShell.m` | Generates `shell/SOS_Gen3_Master_Shell.slx`: Modbus Client blocks at the 50 ms exchange, DAQ blocks, one MATLAB Function block calling `masterStep`, a second one (`TraceStep`) calling `traceStep`, a third (`TrackStep`) calling `trackStep`, the Constant blocks the app writes and the Gain blocks it reads. The model holds no logic and is rebuilt from this script, not edited by hand. |
| `shell/modbusShellDiscover.m` | Prints the Modbus block and client settings of a hand-configured shell so they can be pasted into `buildShell.m`. |
| `shell/addDaq.m` | Creates one Simulink Desktop Real-Time DAQ block. Holds the board setup (PCI-6221, driver options, missed-tick limit) and the digital channel numbering. |
| `SOS_Gen3_Compressor_Interface.m` | Operator app (programmatic App Designer export). Starts the shell model, writes operator requests to its Constant blocks and polls its outputs. Takes its position target range, speed presets and cylinder geometry from `masterConfig.m` at startup. |
| `legacy/`, `SOS_Gen3_Compressor_Master.slx` | Earlier Stateflow-based app model and its `.mlapp`, and the hand-built `SimulinkMaster.slx` bench model with manual mode control, kept for reference. |

## Running on the lab PC

```matlab
cd master-src
run('shell/buildShell.m')          % once, and after any change to the block interface
addpath logic shell trace
SOS_Gen3_Compressor_Interface      % then press "Connect to Compressor"
```

### Modbus settings in the shell

`buildShell` sets everything: the Modbus blocks' register rows (the Write
block writes as many registers as its input is wide, so its row has no
count; the Read row reads 34) and the Modbus client (`ModbusClient`, TCP/IP,
controller IP and port, 0.5 s timeout, byte order big-endian, word order
little-endian) in Configuration Parameters > Modbus. The controller IP and
the timeout are `MODBUS_SETUP` at the top of the script. After a build,
open the Modbus pane once to confirm the client is listed; if the builder
printed a manual step, do as printed. `modbusShellDiscover` prints the
settings of a built model.

Timing: model step 25 ms, Modbus exchange 50 ms. When the cable is pulled
the slave stops itself within 300 ms (`FAULT_COMM`); the model keeps
retrying the connection and stops with an error from the Read block once
Windows TCP gives up, about 20 s later, which sets the DAQ outputs to their
final values (safety relay off) and makes the app report the stopped model.
Press **Connect** again once the slave is reachable, then Stop to clear the
link-lost status.

## App to model interface

The app writes `RequestedMode` (0 stop, 1 home to the limit switch, 2 position,
3 jog, 4 pressure, 5 reset fault, 6 go home) and then increments `CommandSeq`; the model acts on an operation
only when `CommandSeq` changes, except that `RequestedMode = 0` always stops.
`TargetPosition`, `TargetVelocity`, `JogDirection` and `TargetPressure` are read
continuously while their operation is active. `PositionSpeed` and `TargetVelocity`
both carry the one operator speed, set from the app's speed control in percent of
`cfg.speedMax` (presets `cfg.speedPresets`, default `cfg.speedDefault`, converted by
`speedFromPercent`). It is sent in HR7-8 every exchange, acts mid-move, and 0 holds. `TargetPressure` is in atm absolute;
the model sends it to the slave in MPa gauge, and the slave closes the loop on its
own transducer. `ESTOP` and `Mixer` act at once.

The `TrackStep` block sits between the `TargetPosition` and `TargetPressure`
Constant blocks and `masterStep`'s inputs 4 and 7; those two Constants are no
longer wired to `masterStep` directly. It takes the two targets, the
`TrackTable` Constant, `TrackMode` (0 none, 2 position, 4 pressure), `TrackSeq`
and the status code one step late (the `StatusDelay` Unit Delay on
`masterStep`'s output 7). While `TrackMode` is not 0 it replaces the target of
that mode by `TrackTable(index)`, and otherwise passes both through, so manual
control and the other steps are unchanged. The index is
`floor(count / 2) + 1`, one table value per exchange, where `count` is the
number of model steps since `TrackSeq` last changed in which the delayed status
showed the mode's operation running (5 for pressure, 2 or 3 for position); the
series therefore starts when the slave is in mode and not during the handshake,
and the last value holds when the table ends. `TrackIndex` (a Gain block with a
Display) exposes the index. `TrackTable` is a Constant whose value is the
model-workspace variable `trackTable` (`cfg.trackTableLength` values, one hour
at the exchange period), and the model is built with
`DefaultParameterBehavior` Tunable so the app can replace the variable while the
model runs (`setTrackTable`). `ProcStep` is a Constant holding the running
procedure's step number, 0 outside a run, written only when it changes.

The `TraceStep` block records the link for the Record button. It takes the raw
read vector, the write vector as `masterStep` produced it, `masterStep`'s
`doSafety`, `StatusCode`, `FaultCode`, `State` and `LinkOk`, the app's
`RequestedMode` and `CommandSeq`, the targets `masterStep` received (the outputs
of `TrackStep`, so a playing track shows in the trace), and the app's `ESTOP`,
`Mixer` and `ProcStep` blocks (14 inputs). Once per Modbus exchange it writes
one record into a ring of 512 records (25.6 s, 45 columns) and outputs the
whole ring each step to the `TraceRing` Gain block, a test point the app reads
like the other Gain blocks. Each record
carries its sequence number in the first column and again in the last (the
guard), so the app takes only rows whose two copies agree and sees from the
sequence numbers whether the ring overwrote records it had not read yet.
The raw read vector carries IR1-34, so the trace records the slave's per-write summary (IR21-34) with the rest. `masterStep`'s logic is unchanged; its read grows to IR34.

The app reads `StatusCode`, `FaultCode`, `State`, `Position`,
`Velocity`, `Pressure` (the slave's gauge transducer reading plus `atmAmbient`, atm absolute),
`LinkOk`, `PressureBlock` and `AtEndOfTravel`. The last two are `masterStep`'s
outputs 11 and 12, the slave's IR9 (chamber pressure above its ceiling, so it
refuses pressurising motion; not a fault) and IR10 (the piston is within its end
margin of the soft limit), each 0 or 1. A shell built before they existed lacks
the two Gain blocks, so rebuild it with `buildShell` after updating. Status codes are listed in
`masterStateMachine.m`; a fault shows as status 10 with the code in
`FaultCode`, status 35 is shown until the slave's counter has first been
seen, and a lost slave link as status 34 until the operator presses Stop.

## Connecting and closing

"Connect to Compressor" starts the model; the same button then reads
"Disconnect". Disconnecting, or closing the app while connected (after a
confirmation), requests a stop, waits for the master to report the axis
stopped (at most `tStopMax`) and then stops the model. The DAQ outputs fall to
0, which drops the safety relay, and MotionEnable is already low, so the slave
stays stopped once the Modbus exchange ends. Pressing STOP sends a Stop command as well as
the E-stop, which also clears a lost-link status once the link is back. The
mode buttons are enabled only in manual control, while connected with the
slave link up and out of E-STOP.

## Operator app readouts

The app shows volume (mL), absolute pressure (psi, atm or MPa), position (mm
from the end of travel) and flow (mL/s, positive toward the end of travel).
Position targets in manual control are also mm from the end of travel, and
pressure targets are absolute in the pressure unit. `masterConfig` holds the end of travel (`posEOT`), bore and dead
volume, and the atmosphere added to the slave's gauge pressure (`atmAmbient`).
`mLPerMm` converts position and velocity to volume and flow; `sweptVolume`,
`mixingVolume` and `readoutText` give the volume views and the readout text.
The transducer's 4-20 mA range and shunt resistance are slave settings
(`G_cfgPressure*`).

The volume panel has three views, chosen with the buttons under the value:

- Swept: the bore area times the travel left to the end of travel, without
  the dead volume (the saline still deliverable).
- Mix: the swept volume plus `mixingDeadVolume`, the part of `deadVolume` that
  takes part in mixing (at most `deadVolume`).
- Incr: the swept volume delivered since the operator pressed Zero, positive
  as the piston advances toward the end of travel, like the flow. It reads
  `--` until zeroed, and starting Home clears the zero because homing changes
  the position frame.

The pressure panel has a unit switch (psi, atm, MPa) under the value. All
three units are absolute (the slave's gauge reading plus `atmAmbient`), and the
panel title and the target label say so ("Pressure (psi abs)"). `pressureConvert`
and `pressureUnitInfo` hold the conversion (`psiPerAtm`, `atmPerMPa`) and each
unit's decimals, format and spinner step (psi 1 decimal and step 5, atm 2 and
0.5, MPa 3 and 0.05). The switch converts the target spinner's value and
limits (ambient up to the lower of `pressureMax` and `pressureCeiling` minus
`pressureCeilingHyst`), changes the procedure panel's pressure fields and
texts, and is kept for the next session with `setpref`/`getpref`
(`SOS_Gen3_Compressor`, `pressureUnit`). The model, the target of a running
operation and the procedure engine stay in atm absolute. The pressure value
turns red while `PressureBlock` is set, and the status line gets a suffix:
"(pressure ceiling: pressurising blocked)" while it is set and "(end of
travel)" when `AtEndOfTravel` is set during pressure control.

Readouts are rounded to their display resolution before the sign is chosen, so
a zero never reads `-0.00`, and a flow smaller than `flowDeadband` reads 0.00.

Display rate: the display timer runs at `displayPeriod` (`fixedRate`, a tick
that is still busy when the next is due is dropped). Each poll reads the
status, position, velocity and pressure blocks once through cached
`RuntimeObject` handles, and the fault code only while faulted; a label is
rewritten only when its text changed. The app measures its own poll: in a
MATLAB session, `app = SOS_Gen3_Compressor_Interface;`, connect, run for a
while and read `app.PollStats` (`n` polls, `mean`, `max` and `last` in
seconds, `overruns` the polls longer than `displayPeriod`).

## Operator app buttons

The mode buttons are enabled only in manual control, connected with the slave
link up and out of E-STOP. Start Procedure is enabled only out of manual
control, connected with the link up, out of E-STOP, with no operation active
and the status idle; Manual Control is disabled while a procedure is active or
paused (see Procedure below). Home, Go Home, the two Go buttons and Jog Up/Down start their operation and,
while it runs, stop it again (the model's `RequestedMode = 0`). The app follows
the operation through `StatusCode`: the button turns amber while the mode is
being entered and green once its running status appears, and returns to normal
when that status goes (stopped, finished, faulted, E-STOP or not confirmed by
the slave). While position or pressure control runs, changing the target turns
its button back to Go, which sends the new target without leaving the mode; the
other jog button reverses a running jog. The speed control (a percentage
field and a button per `masterConfig` `speedPresets` entry, `speedDefault` at start)
sets the one speed of jog, position moves and Go Home, and a change applies
to a running jog or position move (a Go Home keeps the speed it started at, and
homing runs at its configured speeds); 0 holds a jog or position move, a Go Home does not
start, and the status line then reads "held: speed 0". STOP turns dark red while E-STOP is
requested but not yet reported by the model, and yellow while the model reports
E-STOP (`StatusCode` 20); pressing it then releases E-STOP.

## Procedure (non-manual mode)

The engine is `procNew`, `procAction`, `procStep` and `procView` in `logic/`:
pure functions on one state struct `p`, with no persistent state, no timers and
no app handle. The app calls `procStep(p, in, dt)` on every poll with what it
read from the model (`connected`, `statusCode`, `faultCode`, `estop`, `pressure`,
`volume`, `pressureBlock`, `atEndOfTravel`, and for track steps
`positionFromEOT`, the position in mm from the end of travel, and
`positionSpeed`, the operator's effective speed in mm/s) and executes the
returned requests in this order (`params`, `mixer`, `stop`, `track`, `op` with
`target`, `unit` and `speed`, `zeroVolume`, `record`, `recordEnd`) through its
own operation requests, so the interlocks stay in one place. `procAction` only records the operator's
intent (start, confirm, repeat, check, setParam, pause, resume, abort, reset)
and refuses an action the state does not allow; `procView` gives what the panel
shows. `tests/smokeProcedureEngine.m` runs all of it against a stand-in for the
model's status. `procView(p, opts)` takes the display options of
`pressureUnitInfo` and shows pressure parameters, texts and the held pressure
in that unit (atm absolute without them).

**Start.** Start Procedure begins at step 1 when the model is running, the
slave link is up, E-STOP is not active and the status is 0 (idle); otherwise it
refuses and says why. Parameter values entered in an earlier run are kept.
Manual control and the procedure exclude each other: the mode buttons work only
in manual control and Start Procedure only out of it, and manual control is not
available while a procedure is active, paused included.

**Panel.** The Procedure panel (`ProcedurePanel`, right of the readouts) holds
the step list (display only: done steps marked `[x]`, the current one `>`), the
step's heading, title and instruction, its checklist (a pool of
`cfg.procMaxChecks` checkboxes) or, in a param step, its parameter fields (a
pool of `cfg.procMaxParams` fields in two columns, editable only while the step
waits for the operator), Confirm (with the step's label), Repeat where the step
has one, Pause/Resume and Abort, and the readouts: the procedure timer, the
volume discharged, the held pressure target and a message line with the engine's
state and the reason for a pause or the last refused action. The panel only
copies what `procView` returns. Every operator action and every poll steps the
engine at once, so Abort sends its stop immediately; the engine's operations go
through the app's `requestOp`/`stopOp` and the same `motionAllowed` interlock as
the manual buttons (the model running, the slave link up and no E-STOP). The
procedure's volume (`in.volume`) is the swept volume of the readouts, and
`Disconnect`
during a procedure is left to the engine, which pauses with "Model stopped".
The volume-zero request also zeroes the Incr view. A procedure that does not
validate leaves the panel empty, Start Procedure disabled and the reason on the
status line.

**Selection.** At start the app loads the procedure named by `cfg.procedure`
(the SOP). The **Procedure** menu lists, after **Open...** and **Reload**, every
`.m` file of `procedure/`, sorted by name, with a check mark on the loaded one;
choosing an item loads that function. **Open...** takes any `.m` file (its folder is added to the path and
the function is loaded by name, listed and checked in the menu while it is
the loaded one) and **Reload** loads the current choice again, for a file just
edited. A choice is validated through `procNew` at once: on success the status
line reads `Procedure loaded: <name>` and the panel shows the new procedure; on
a failure the previously loaded procedure and its check mark stay, and the
status line gives the reason. Nothing is remembered between sessions. The whole
menu is disabled while a procedure is active (running, waiting for the
operator, paused or aborting).

### Parameters

A parameter is a struct in the procedure's `params` cell array.

| field | default | meaning |
|---|---|---|
| `key` | required | valid MATLAB identifier |
| `label` | key | text in the panel |
| `unit` | `''` | shown after the label |
| `value` | required | default value, a numeric scalar |
| `min`, `max` | `-Inf`, `Inf` | range, checked when entered and on confirm |
| `format` | `'%g'` | display and `{key}` substitution in titles, texts and checks |
| `sink` | `''` | `''`, `'SalineVolume'` or `'OxygenRatio'`: the shell Constant block that receives the value when the param step is confirmed |
| `kind` | `''` | `''` or `'pressure'`: the value, `min` and `max` are atm absolute and shown and entered in the pressure unit; the parameter takes no `unit` of its own |

A pressure parameter's `{key}` renders converted and followed by the unit label
("114.7 psi abs"), so a text must not add a unit after it. The panel checks the
range in the display unit; the engine's own refusal reads "<label> is out of
range".

### Steps

A step is a struct in the procedure's `steps` cell array; it gives only the
fields it uses (`procNew` fills the defaults).

| field | applies to | default | meaning |
|---|---|---|---|
| `id` | all | `s<k>` | unique; the target of `repeatFrom` |
| `section` | all | previous step's | group heading |
| `title` | all | required | line in the list and the panel heading |
| `text` | all | `''` | instruction; `{key}` is replaced by the parameter value |
| `type` | all | required | `confirm`, `param`, `machine` or `wait` |
| `checks` | confirm | `{}` | checklist; Confirm is enabled when every item is ticked (at most `cfg.procMaxChecks`) |
| `confirmLabel` | confirm, param | `'Confirm'` | label of the Confirm button |
| `repeatFrom` | confirm | `''` | id of an earlier step; shows a second button that jumps back to it |
| `repeatLabel` | confirm | `'Repeat'` | label of that button |
| `params` | param | required | keys entered and confirmed in this step |
| `op` | machine | required | `home`, `goHome`, `position`, `pressure`, `track` or `stop` |
| `mode` | track | required | `pressure` (atm absolute) or `position` (mm from the end of travel) |
| `signal` | track | required | a function `@(t, p)` of the time column `t` (s) and the parameter values `p` (a struct, key to value), or a two-column matrix `[t value]` |
| `relative` | track | `false` | the series is added to the reading at the step's first issue |
| `record` | all | the procedure's | this step's record interval in s, or `'off'` (see Recording from a procedure) |
| `target` | position, pressure | required | a number, a parameter key, or a function handle `@(values, cfg)` returning a finite scalar, evaluated whenever the step issues its operation (on entry and again on resume); `procNew` evaluates it once on the parameter defaults and rejects a failure |
| `unit` | position | `'mm'` | `mm` from the end of travel, or `mL` chamber volume (`deadVolume` plus the swept volume); a pressure is atm absolute |
| `speed` | position | NaN (none) | mm/s the move is capped at; the operator's speed still wins when it is lower (0 holds) |
| `pressureMax` | machine | `Inf` | atm absolute; above it the step pauses (below) |
| `alsoWhen` | confirm, pressure | `''` | `'endOfTravel'`: the step also completes at the end of travel (below) |
| `until` | machine, wait | by op | the completion condition (below); a track has only `elapsed` |
| `keep` | position, pressure (not track) | `false` | leave the operation running as the background operation; otherwise the engine stops it once the step completes |
| `tolerance` | settled | `cfg.procPressureTol` | atm |
| `duration` | timed wait; settled; track | required for a timed wait and a track; `cfg.procSettleTime` | s: a number, or for a timed wait and a track a parameter key |
| `timeout` | machine, wait | `Inf` | when exceeded the procedure pauses (and stops a non-kept operation) |
| `timerStart`, `timerStop` | all | `false` | start the procedure timer when the step completes; stop it on entering the step |
| `zeroVolume` | all | `false` | on entering the step, take the reference of the volume-discharged readout |
| `mixer` | all | `''` | `on`, `off` or a parameter key (0/1), applied on entering the step |

Completion: `position` `arrive` is status 3; `home`/`goHome` `ended` is the
operation confirmed, its status gone and the master back at status 0;
`pressure` `running` is status 5 seen, and `settled` also needs the pressure
within `tolerance` of the target for `duration`; `stop` `stopped` is a status
outside 1 to 8. A wait is timed (`duration`) or `settled` about the background
pressure target. A step whose operation ends early (not confirmed by the slave,
not reported stopped, left by the master) pauses with a message naming the
status.

A target function that fails when the step issues pauses it with "Step target
could not be computed: " and the function's message, and nothing is sent; the
oxygen fill of the seed procedure does this when the charge does not fit the
chamber. While a machine step tracks or settles, `pressure` above its
`pressureMax` pauses it with "Pressure above the step's limit" and
`PressureBlock` pauses it with "Pressure ceiling reached" (every op but
`stop`), each sending a stop for a step that does not `keep`; resume issues the
step again. The messages carry no numbers, so the engine stays unit-free.
`alsoWhen = 'endOfTravel'` ends a confirm step (without the click) or a
pressure step (as its condition, then the usual stop or keep) once
`atEndOfTravel` is set or the swept volume is at or below `cfg.procEndVolume`
(above the slave's end margin, so it also works on a slave without the flag);
the panel shows "<title> ended at the end of travel" as the message. In the
app, a `speed` caps `PositionSpeed` for the step ("(step)" after the speed
readout) and the operator's speed is restored when the operation ends.
`procNew` rejects a procedure that cannot run (unknown type, op, unit,
`until`; missing title or target; a key that names no parameter; duplicate id;
a `repeatFrom` that is not an earlier step; too many checks or parameters; an
unknown sink or kind; a pressure parameter with a unit; `keep` on a step that
cannot hold; `speed` outside a position step or not above 0; `pressureMax` not
above 0; an unknown `alsoWhen` or one on another step; a target function that
fails on the defaults; a settled wait with no earlier pressure step with `keep`;
a `record` that is not a number above 0 or `'off'`; a `duration` key that names
no parameter; a track without `mode`, `signal` or `duration`, or with `target`,
`unit`, `speed`, `alsoWhen` or `keep`; `mode`, `signal` or `relative` on another
step; a track signal that fails on the defaults or leaves the limits below).

### Track steps

A track step (`op = 'track'`, `until = 'elapsed'`) makes the pressure or the
position follow a time series for `duration` seconds. The series is sampled
once per exchange: `N = floor(duration / exchangePeriod) + 1` values at
`t = (0 : N-1)' * exchangePeriod`. A function `signal` is called once with the
column `t`, and with each `t` in turn when the result is not N values; a
`[t value]` matrix needs strictly increasing `t >= 0` and finite values, is
linear between its points and holds its first value before `t(1)` and its last
after its end. Every sample must be finite, and N is bounded by
`cfg.trackTableLength` (`cfg.trackMaxSeconds`, one hour): a longer series is
split into consecutive track steps, or repeated with `repeatFrom`.

*Validation.* `procNew` samples the signal on the parameters' defaults and
checks it (`procTrackSeries`, `procTrackCheck`): an absolute pressure series
within ambient to `pressureMax` plus ambient, an absolute position series
within `max(posEOT - posMax, 0)` to `posEOT - posMin`; a relative series by its
span against the width of that range; a position slope above `cfg.speedMax`
(mm/s). A pressure slope is checked only when `cfg.trackPressureSlopeMax` is
finite (default `Inf`): the pressure a loop can follow depends on the chamber's
stiffness, which the procedure does not know, so a signal that is too fast
lags and the trace shows the tracking error. The step then checks again when
it issues, with the parameter values in force: the absolute series (the signal
plus the offset) against the limits and a position slope against
`in.positionSpeed`; a failure pauses with "Track signal ..." or "Track signal
could not be computed: ..." and sends nothing, and a relative position track
pauses with "Position reading unavailable" when `in.positionFromEOT` is NaN.

*Offset.* A relative series is added to the reading at the step's first issue
(`in.pressure`, or `in.positionFromEOT`), which is kept for a resume so the
planned levels stay the same; it is taken again when the step is entered again
(`repeatFrom`, or the return after a background operation was re-run).

*Issue and borrowing.* When a background operation of the track's mode is
running (an earlier pressure or position step with `keep`), the track
borrows it: no operation is requested, the series plays over the hold, and when
the step ends the target returns to the hold's own value and the hold carries
on. Otherwise the track requests its own operation with the first value as its
target and stops it when the step completes. `keep` is not allowed on a track,
because a lost background operation is re-established by running its step again,
which for a track would replay the series; a hold after a track is a pressure or
position step with `keep`. The step completes when `duration` seconds have
counted from the moment the operation's status showed it running, the same
condition under which the shell's `TrackStep` starts the series.

*Request.* `req.track` is `struct('action', 'start', 'mode', m, 'series', v,
'relative', tf, 'offset', o, 'duration', T)` with `v` the absolute series (atm
absolute, or mm from the end of travel; `o` is 0 when not relative) or
`struct('action', 'end')`. The app writes the table (position values as slave
axis positions, clamped to the soft limits, padded with the last value to
`cfg.trackTableLength`), then `TrackMode`, then `TrackSeq` incremented, then
the operation's `RequestedMode`/`CommandSeq`. It tries the simulation's
variable-tuning API for `trackTable` and falls back to assigning the model
workspace variable and updating the model; the first update's time is reported
on the status line.

*Pause and resume.* Any pause (operator, pressure guard, timeout, ended
operation, E-STOP, link, model or slave fault) ends the track (`TrackMode` 0);
an own operation is stopped as for any machine step, a borrowed one carries on.
Resume issues the step again: the series is sampled with the current
parameter values and restarts from its start with the original offset. Abort and
completion end the track too, and the app ends it whenever it clears its
operation.

*Progress.* While a timed wait runs or a track has been issued, the panel's
title line shows `(m:ss / m:ss)`, the elapsed time against the duration
(`stepProgress` of `procView`).

### Recording from a procedure

`record` on the procedure is a number above 0 (s) or `'off'`; the default is
`cfg.procRecordDefault` (1 s), not the app's interval, because a procedure is data
and its files must not depend on the operator's setting. A step's `record`
overrides it for that step, and a step without one inherits the procedure's. The
engine asks for the interval (`req.record`) on entering a step whose effective
value differs from the last one asked, and always on the first step, and asks
`req.recordEnd` when the run ends (completed or aborted). The app carries it out:

- a number starts a recording owned by the procedure at that interval, unless
  the operator stopped a recording earlier in this run; with a recording running
  it changes the interval; with one paused it resumes it;
- `'off'` pauses the recording: the open interval is emitted and no rows are
  taken until the next number, which shows as a jump of `seq` in the file;
- at the end of the run a procedure-owned recording is saved to `captures/`
  without a dialog, and an operator-owned one continues at the dropdown's
  interval;
- a slave fault or E-STOP pause keeps recording (the fault and the recovery are
  the data), a model stop ends the recording as it always does;
- **Stop recording** during a run saves with the dialog as usual and the
  procedure starts no other recording in that run (Start Procedure resets this);
- **Record** pressed during a run starts an operator-owned recording at the
  procedure's last requested interval; later requests still set its interval, and
  it outlives the run.

The Trace panel's dropdown is disabled during a run and shows the interval in
force; it returns to the operator's choice when the run ends.

### States

`idle`, `running`, `waitOperator` (confirm and param steps), `paused`,
`aborting` and `done` (outcome `completed` or `aborted`).

- **Pause** stops the current step's own operation and keeps a `keep` pressure
  hold running; a timed wait stops counting. Resume re-issues an interrupted
  move and continues a wait or an operator step where it was.
- **Abort** requests a stop, waits for the axis (at most `tStopMax` plus 0.5 s)
  and ends `done`/`aborted`; with the model not running it ends at once.
- **E-STOP, slave link lost, model stopped and a slave fault** (status 10,
  and 9 while resetting and 32 if the reset failed) pause the procedure with
  the reason shown ("Slave fault 7"). The master has already stopped
  everything, so no stop is sent;
  an interrupted move is issued again and a lost pressure hold re-run (and the
  step that was paused returned to, with its ticks) on resume. The timer keeps
  counting. Resume is refused while the condition holds and enabled once it
  has cleared, so after a slave fault the operator resets it (the app's
  dialog) and then resumes; Abort is always available (at a fault it ends
  `aborted`, the fault status not counting as busy). The mixer is left on until
  abort or completion.
- **A background operation that ends** (the pressure hold is left by the
  master) pauses the procedure; Resume re-runs the step that started it and
  returns to the paused step.
- The mixer is switched off at abort and completion only if the procedure
  switched it on.

### Readouts

The timer runs from the step with `timerStart` to the step with `timerStop`
(and through a pause). The volume discharged is the volume at the step with
`zeroVolume` minus the present volume. The message line shows the pause reason
or the last refusal, and when empty the note of a step that ended on its own at
the end of travel.

### Writing the SOP

Edit `procedure/sopProcedure.m` (or add a file there and set `cfg.procedure` in
`masterConfig.m`) with no code change, then run
`tests/run_all.sh smokeProcedureEngine`: it validates the shipped procedure
through `procNew` and walks it against the stand-in. Pressure parameters are
`kind = 'pressure'` in atm absolute; a pressure `{key}` renders with its unit,
so write `Holding {dischargePressure}.` and not `... atm.`. The seed procedure
computes the oxygen fill volume from its parameters in a function target
(`oxygenFillVolume`, with its equation in the comment) and writes the saline
volume and the oxygen ratio to the slave (sinks); its constants `FILL_EXCESS`,
`PUSH_SPEED` and `PUSH_PRESSURE_MAX` are at the top of the file; the seed
procedure records each run at 1 s (`record`).

`procedure/experimentTemplate.m` is the file to copy from for an experiment: it
sets the record interval for the run and per step, has one step of each kind
(including a relative pressure track after a kept pressurize, which borrows the
hold, and a timed wait that holds pressure through it) and comments the fields.
Copy it to a new name in `procedure/`, rename the function, delete what is not
needed and set `cfg.procedure`. `tests/run_all.sh smokeProcedureEngine`
validates the shipped procedure, and `experimentTemplate` through `procNew`, and
walks them against the stand-in.

## Recording a trace

The Trace panel (between the lamps and the Procedure panel) holds **Record**, an
interval dropdown, a status label, **Live plot** and **Plot trace...**.

The **Trace** menu has the same three actions: Record (Stop recording while
recording, Ctrl+R), Plot trace... (Ctrl+P) and Live plot (Ctrl+L), enabled as
their buttons are.

**Record** is available while connected. The dropdown chooses the record
interval: `cfg.traceIntervalPresets` (0.05 s (20 Hz), 0.1 s (10 Hz), 0.5 s
(2 Hz), 1 s, 10 s, 60 s), starting at `cfg.traceIntervalDefault` (0.05 s, every
exchange). A preset list cannot give an invalid value; a procedure may ask for
any value, which is rounded to whole exchanges (below). Changing the dropdown
while recording applies at once: the open interval is written as a row and a new
grid starts. Pressing Record starts a recording at the dropdown's interval; the
label shows the rows written, the elapsed time and the interval, `"N records,
S s, every X s"`, `", paused by step"` while a procedure step has recording off,
and the records missed in red. Pressing the button again (**Stop recording**)
ends the recording, writes the file at once to
`captures/trace_yyyymmdd_HHMMSS.csv` and then opens a Save dialog proposing that
path: choosing another name or folder moves the file there, and cancelling keeps
it in `captures/`. Recording also ends, without a dialog, when the model stops (a
lost Modbus connection, for example), when **Disconnect** is pressed and when the
app is closed; the file is then saved to `captures/` and the panel names it.
**Plot trace...** opens a file chooser in `captures/` and plots the file chosen.

During a procedure the interval belongs to the procedure (`record`, see
Recording from a procedure above): the dropdown is disabled and shows the
interval in force, a number starts a recording owned by the procedure that is
saved without a dialog when the run ends, and a recording the operator started
(Record during a run, or before it) takes the procedure's intervals while it
runs and returns to the dropdown's choice afterwards. Stopping a recording
during a run keeps the procedure from starting another in that run.

The app reads the ring on every display poll (`displayPeriod`, 0.1 s) whether or
not it is recording, so the live plot is always current; the ring holds 25.6 s,
so a stall of the app shorter than that loses nothing. Each poll takes the new
records (`traceDrain`, which keeps none), aligns them (`traceAlign`, below),
appends them to the live buffer and, while recording and not paused, passes
them through `traceDecimate` into the recording (`traceAppend`). Records
overwritten before they were read are counted as missed: the label turns red,
the file's `missed` entry gives the total and the sequence column jumps where
they were lost. The records between the last poll and a model stop (at most one
poll) are not captured; pressing **Disconnect** reads the ring once more before
stopping. The first rows of a recording start about two exchanges after the
button, because the alignment holds the newest two records back.

**Live plot** opens a figure with four stacked axes over the last
`cfg.livePlotSeconds` (600 s): the pressure (the snapshot, and the window mean
with its minimum and maximum), the position from the end of travel, the velocity
and the torque, against the time before now. The aligned 20 Hz stream goes into
a circular buffer of 12000 rows, and the figure is redrawn every
`cfg.livePlotPeriod` (0.5 s) while it is open, by setting the line data in place;
closing it stops the updates. The redraw is part of the poll, so a poll that
overruns `displayPeriod` shows in `app.PollStats`.

## Trace files

A trace file is a CSV with a `# key: value` header block, a line of column
names and one row per record interval (one Modbus exchange, 50 ms, by default).
Values are the raw engineering units of the link: position in slave axis
millimetres, pressure in MPa gauge, targets as the app writes them (the targets
`masterStep` received, so a playing track shows). The geometry in force at
capture time is stored in the header, so the displayed quantities are derived
from the file alone.

| Metadata key | Meaning |
|--------------|---------|
| `format` | File format version (3; formats 1 and 2 still read and plot, see below). |
| `source` | Writer of the file. |
| `started` | Wall-clock start, `yyyy-mm-ddTHH:MM:SS`. |
| `records`, `missed` | Rows written and records lost to ring overruns. |
| `dt_s`, `exchangePeriod_s` | Model step and Modbus exchange period. |
| `recordInterval_s` | The effective record interval when the recording started (whole exchanges). A procedure can change it during the file; a row's own interval is its `nExchanges` times `exchangePeriod_s`. |
| `posEOT_mm`, `boreDiameter_mm`, `deadVolume_mL`, `mixingDeadVolume_mL` | Cylinder geometry (`masterConfig`). |
| `atmPerMPa`, `atmAmbient_atm` | Pressure scaling and the atmosphere added to gauge pressure. |
| `referenceUnits` | Units of the `reference` column by mode. |

| Column | Source | Unit |
|--------|--------|------|
| `seq` | Record number since model start, from 1; of the last exchange in the row | - |
| `t_s` | Model time of the recording step; of the last exchange in the row | s |
| `heartbeat` | HR1 written in this exchange | count |
| `heartbeatEcho` | IR1 | count |
| `slaveCounter` | IR2 | count |
| `confCode` | IR3 | code |
| `slaveFaultCode` | IR4 | code |
| `faultActive`, `inMotion`, `performanceStatus`, `homingComplete` | IR5 to IR8 | 0/1 |
| `position_mm` | IR11-12, slave axis position | mm |
| `velocity_mm_s` | IR13-14 | mm/s |
| `torque_pct` | IR15-16 | % rated |
| `commandedPosition_mm` | IR17-18 | mm |
| `pressureGauge_MPa` | IR19-20, as the slave reports it | MPa gauge |
| `pressureMeanGauge_MPa`, `pressureMinGauge_MPa`, `pressureMaxGauge_MPa` | IR21-22, IR23-24, IR25-26: pressure over the scans of the row's window | MPa gauge |
| `velocityMean_mm_s` | IR27-28 | mm/s |
| `torqueMean_pct` | IR29-30 | % rated |
| `positionMean_mm` | IR31-32 | mm |
| `summaryScans` | IR33, scans in the window; 0 where there is none | count |
| `summarySeq` | IR34, window number, wraps at 65536; of the last window in the row | count |
| `modeCode`, `motionEnable`, `faultReset` | HR2 to HR4 as sent | code, 0/1, 0/1 |
| `reference` | HR5-6 as sent | mm (position), mm/s (jog), MPa gauge (pressure) |
| `positionSpeed_mm_s` | HR7-8 as sent | mm/s |
| `salineVolume_mL` | HR9-10 as sent | mL |
| `oxygenRatio` | HR11-12 as sent | STP volume per saline volume |
| `statusCode`, `faultCode`, `state` | `masterStep` outputs (`state` is `masterStateMachine`'s `ST_*`) | code |
| `linkOk`, `doSafety` | `masterStep` outputs | 0/1 |
| `op`, `cmdSeq` | `RequestedMode`, `CommandSeq` blocks | code, count |
| `targetPosition_mm` | the target `masterStep` received (`TrackStep` output 1) | mm, slave axis |
| `targetPressure_atm` | the target `masterStep` received (`TrackStep` output 2) | atm absolute |
| `estop`, `mixer` | `ESTOP`, `Mixer` blocks | 0/1 |
| `procStep` | `ProcStep` block: the running procedure's step number, 0 outside a run | count |
| `nExchanges` | Exchanges combined into the row (1 in the ring) | count |

**Summary alignment.** The slave closes a window on each change of the master's
heartbeat, so a window spans the scans of one master write, and publishes it with
a lag: the summary read in record n describes the window opened by the write of
record n-2. The ring is raw, and the app aligns while it records: `traceAlign`
holds two records back and gives row k the summary of record k+2 when
`seq(k+2) = seq(k) + 2`, `summaryScans(k+2) > 0` and its `summarySeq` is not the
window last assigned (a late exchange the link merged reads the same window
twice, and it is counted once). Otherwise the row has no window: the six values
0, `summaryScans` 0 and `summarySeq` 0. A format 3 file therefore has the summary
on the row of its window, whatever the interval, and a tool reading columns
`pressureMeanGauge_MPa` to `summarySeq` of a format 3 file must not shift them.
A format 2 file (raw, one row per exchange) has the lag in it:
`traceDerive` gives its row k the summary of record k+2, NaN where record k+2
is missing, is not exactly two records later or reports `summaryScans` 0, and a
format 1 file has none.

**Decimation.** An interval of `interval` seconds is `n = max(1, round(max(interval,
exchangePeriod) / exchangePeriod))` exchanges: a smaller interval is raised to
one exchange and a value that is not a multiple is rounded, not rejected, and
the file records the effective interval. `traceDecimate` combines the aligned rows
of each interval into one:

- over the windows with `summaryScans > 0`, the means of pressure, velocity,
  torque and position are weighted by the scans (`sum(scans * mean) / sum(scans)`),
  the minimum is the smallest window minimum and the maximum the largest window
  maximum, `summaryScans` is the sum and `summarySeq` the last window's number;
  with no window all are 0;
- every other column is the last exchange's, so `seq` and `t_s` identify the
  exchange the snapshot values come from, and `nExchanges` is the count;
- the row covers `t_s - (nExchanges - 1) * exchangePeriod_s` to
  `t_s + exchangePeriod_s`.

The grid is anchored at the first record taken after a start, resume or interval
change; an interval is emitted when its last record arrives, or early when a
record beyond it arrives (records were missed), when `procStep` changes (no row
spans two steps) and when the recording is flushed. A row with fewer than n
exchanges is a partial interval: at the end of a recording, at a step change, at
a pause or where records were missed.

`traceLayout.m` defines the columns (44 of them, 45 in the ring with the guard)
and `traceColumnNames` gives the header. Read and plot a file from MATLAB or
Octave (with `master-src/logic` and `master-src/trace` on the path):

```matlab
d = traceReadCsv(file);     % d.meta, d.names, d.data and one column vector per name
d = traceDerive(d);         % adds posFromEOT_mm, sweptVolume_mL, mixingVolume_mL, flow_mL_s, pressureAbs_atm, pressureMeanAbs_atm, pressureMinAbs_atm, pressureMaxAbs_atm, posMeanFromEOT_mm, interval_s, tStartRel_s, decimated, gap, tRel_s, ...
figs = tracePlot(file);     % a file name, the struct from traceReadCsv, or [] to choose a file
figs = tracePlot(d, 'visible', 'off', 'save', true, 'saveDir', folder);
```

`traceDerive` adds to the quantities above: `posMeanFromEOT_mm` (the end of
travel minus the window's mean position; NaN where `summaryScans` is 0 and in
files before format 3), `interval_s` (`nExchanges` times the exchange period),
`tStartRel_s` (`tRel_s` minus `(nExchanges - 1)` exchange periods, where the
row's interval begins) and the scalar `decimated` (some row combines more than
one exchange). A file without `procStep` and `nExchanges` gets 0 and 1. `gap`
is true at the first row after a jump of `seq` larger than the row's
`nExchanges`.

`tracePlot` returns three figures: `overview` (pressure with the window mean, maximum and
minimum when the file has them, position, velocity and torque against time), `delivery` (swept volume, flow, and pressure against
swept volume) and `status` (status and mode codes, the operation flags, link state
and link health). For a decimated file the overview is in interval mode: the
pressure row draws the interval's mean, minimum and maximum and the target as
steps from `tStartRel_s` without the snapshot, and the position, velocity and
torque rows draw the interval means as steps. Fault, link-down and E-STOP intervals are shaded and lines
break where records were missed. With `'save'` on, each figure is saved as
`<csv name>_<overview|delivery|status>.fig` in `saveDir` (the file's folder
by default).

## Testing without MATLAB

`tests/run_all.sh` (repository root) needs `python3` and `octave-cli`. It
translates `slave-src` into a simulated slave with `tools/st_translate.py` and runs the
master logic against it in closed loop. The trace recorder, the ring drain, the
alignment, decimation and live buffer, track playback, the procedure engine and
the CSV and plot functions are tested against a stand-in slave. The Simulink
shell and the app (including the `TraceRing` read, the Record button, the
interval dropdown, the live plot and the track table update) are not covered;
they can only be exercised on the lab PC.
