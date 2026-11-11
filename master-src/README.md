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
| `trace/` | The trace recorder's host-side functions, plain `.m` files that run under MATLAB and GNU Octave: `traceStart`, `traceDrain` and `traceAppend` (collect records from the ring), `traceAlignNew`/`traceAlign` (give each row the slave summary of its own window), `traceDecimateNew`/`traceDecimate` and `traceIntervalExchanges` (combine exchanges into the record interval), `traceLiveBuffer`, `traceLiveAppend`, `traceLiveNew` and `traceLiveUpdate` (the live plot), `traceWriteGap` (the longest gap between master writes, for the link stall check), `traceMeta` and `traceWriteCsv` (write a trace file), `traceReadCsv` and `traceDerive` (read it back and add the displayed quantities), `tracePlot` (three figures) and `traceColumnNames`. The block that fills the ring, `traceStep`, and the ring's layout, `traceLayout`, are in `logic/`. |
| `captures/` | Trace files the app saves (gitignored; created on first use). |
| `shell/buildShell.m` | Generates `shell/SOS_Gen3_Master_Shell.slx`: Modbus Client blocks at the 50 ms exchange, DAQ blocks, one MATLAB Function block calling `masterStep`, a second one (`TraceStep`) calling `traceStep`, a third (`TrackStep`) calling `trackStep`, the Constant blocks the app writes and the Gain blocks it reads. The model holds no logic and is rebuilt from this script, not edited by hand. |
| `shell/modbusShellDiscover.m` | Prints the Modbus block and client settings of a hand-configured shell so they can be pasted into `buildShell.m`. |
| `shell/addDaq.m` | Creates one Simulink Desktop Real-Time DAQ block. Holds the board setup (PCI-6221, driver options, missed-tick limit) and the digital channel numbering. |
| `controller/` | The controller process, which runs the shell model in a MATLAB of its own: `runController` (the process, MATLAB only), `controllerNew`/`controllerStep` (its decisions, with no I/O), `ctlEncode`/`ctlDecode` (one JSON message per line on the local TCP connection), `ctlStatusMessage` (what it sends the app), `ctlClientNew`/`ctlClientUpdate` (the app's view of it), `ctlBlocks` (the blocks the app writes and reads) and `ctlLaunchCommand` (how the app starts it). See Controller process below. |
| `SOS_Gen3_Compressor_Interface.m` | Operator app (programmatic App Designer export). Starts the controller, writes operator requests to the shell's Constant blocks through it and shows the outputs it reports. Takes its position target range, speed presets and cylinder geometry from `masterConfig.m` at startup. Laid out with nested `uigridlayout`s. |
| `legacy/`, `SOS_Gen3_Compressor_Master.slx` | Earlier Stateflow-based app model and its `.mlapp`, and the hand-built `SimulinkMaster.slx` bench model with manual mode control, kept for reference. |

## Running on the lab PC

```matlab
cd master-src
run('shell/buildShell.m')          % once, and after any change to the block interface
addpath logic shell trace controller
SOS_Gen3_Compressor_Interface      % then press "Connect to Compressor"
```

The app starts the controller, a second MATLAB that runs the shell model,
minimized; the status line reads "Starting the controller... N s" until it
answers and then "Controller ready: press Connect". Closing the app ends it.

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

The app does not touch the model itself: the controller (Controller process,
below) writes the app's values to the Constant blocks in the order the app sent
them and reports the Gain blocks' outputs and the trace records. The blocks and
their meaning are as follows.

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
`DefaultParameterBehavior` Tunable so the variable can be replaced while the
model runs: the app's `setTrackTable` writes the table to a file in `tempdir` and
the controller loads it (the simulation's `setVariable`, or the variable assigned
and the model updated) and notes how long the first load took. `ProcStep` is a Constant holding the running
procedure's step number, 0 outside a run, written only when it changes.
`SalineVolume` and `OxygenRatio` are Constants holding the fill (mL, and the
oxygen charge as STP volume per saline volume), written by a procedure's param
step through its sinks; `buildShell` sets them to `cfg.salineVolume` and
`cfg.oxygenRatio`.

The `TraceStep` block records the link for the Record button. It takes the raw
read vector, the write vector as `masterStep` produced it, `masterStep`'s
`doSafety`, `StatusCode`, `FaultCode`, `State` and `LinkOk`, the app's
`RequestedMode` and `CommandSeq`, the targets `masterStep` received (the outputs
of `TrackStep`, so a playing track shows in the trace), and the app's `ESTOP`,
`Mixer`, `ProcStep`, `SalineVolume` and `OxygenRatio` blocks (16 inputs). Once per Modbus exchange it writes
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

"Connect to Compressor" is enabled once the controller answers. It writes the
app's starting values (stop, E-STOP, mixer, speed, procedure step, track) and
asks the controller to start the model; the status line reads "Starting the
model..." and then "Connected" once the controller reports it running, and the
button then reads "Disconnect". Disconnecting, or closing the app while
connected (after a confirmation), requests a stop and asks the controller to
stop the model once the master reports the axis stopped (at most `tStopMax`);
the app waits for that. The DAQ outputs fall to 0, which drops the safety relay,
and MotionEnable is already low, so the slave stays stopped once the Modbus
exchange ends. Closing the app then ends the controller. Pressing STOP sends a Stop command as well as
the E-stop, which also clears a lost-link status once the link is back. The
mode buttons are enabled only in manual control, while connected with the
slave link up and out of E-STOP.

## Operator app layout

The window is a `MainGrid` of three bands, in nested `uigridlayout`s:

- The readout band (190 px) holds the four readouts (flow, volume,
  position, pressure) of equal width, the status line and the Fault and Mixer
  lamps under them, and the STOP button pinned at the top right,
  120 px wide over the band's height.
- The Controls band (128 px) holds three rows: connect, mode and target
  controls, the speed row (label, field, presets and the speed in mm/s) and, at
  the right, Record, the interval dropdown and the trace status.
- The Procedure panel takes the remaining height, so a taller window lengthens
  only its instruction text.

A new window is 940 x 900 px (drawable area); the minimum is 830 x 680. Below
the minimum the grid keeps its minimum extent and the window scrolls.

The **View** menu places the windows. The app's monitor is the one containing
the centre of its window. A monitor's work area is its area less a 48 px margin
along the bottom, for the taskbar.

- **Side by side** puts the app on the left half of its monitor's work area (at
  least its minimum width) and the live plot on the right half, opening the
  plot when it is closed; with under 200 px left for the plot it is not placed.
  The app is brought to the front.
- **Live plot on second monitor** maximises the live plot on the largest other
  monitor. It is disabled with one monitor.
- **Default size** restores 940 x 900 from maximised or side by side, keeping the
  window's top-left corner and clamping it into the monitor's work area.

The window decorations are measured from the figure's outer position and
position when visible, and otherwise taken as 8, 8, 8 and 90 px (left, bottom,
right, top).

## Operator app readouts

The app shows volume (mL), absolute pressure (psi, atm or MPa), position (mm
from the end of travel) and flow (mL/s, positive toward the end of travel).
Position targets in manual control are also mm from the end of travel, and
pressure targets are absolute in the pressure unit. The position target starts
at `targetFromEotDefault` and takes the piston's position whenever Manual
Control is engaged while connected, and when the slave link first comes up after
Connect, so Go moves nothing until the operator changes it. `masterConfig` holds the end of travel (`posEOT`), bore and dead
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
that is still busy when the next is due is dropped). Each poll takes the
controller's latest status (the status, position, velocity, pressure and fault
code blocks); a label is rewritten only when its text changed. The app measures its own poll: in a
MATLAB session, `app = SOS_Gen3_Compressor_Interface;`, connect, run for a
while and read `app.PollStats` (`n` polls, `mean`, `max` and `last` in
seconds, `overruns` the polls longer than `displayPeriod`, and `writeGapMax`
and `stalls` from the link stall check below) and `app.StallLog`.

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

**Panel.** The Procedure panel (`ProcedurePanel`, below the controls and taking the window's remaining height) holds
the step list (display only: done steps marked `[x]`, the current one `>`), the
step's heading, title and instruction, its checklist (a pool of
`cfg.procMaxChecks` checkboxes) or, in a param step, its parameter fields (a
pool of `cfg.procMaxParams` fields in two columns, editable only while the step
waits for the operator), Confirm (with the step's label), Repeat where the step
has one, Pause/Resume and Abort, and the readouts: the procedure timer, the
volume discharged, the held pressure target and a message line with the engine's
state and the reason for a pause or the last refused action. The checklist
and parameter rows collapse to no height when the step has none. The panel only
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
operator, paused or aborting), and while motion is commanded (see Link
stalls).

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
| `sink` | `''` | `''`, `'SalineVolume'` or `'OxygenRatio'`: the shell Constant block that receives the value when the param step is confirmed; the trace records it (`salineVolume_mL`, `oxygenRatio`) |
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

The interval dropdown is disabled during a run and shows the interval in
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
volume and the oxygen ratio to the shell's `SalineVolume` and
`OxygenRatio` blocks (sinks), which the trace records; its constants `FILL_EXCESS`,
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

**Record**, the interval dropdown and the trace status are at the right of the
Controls band. The **Trace** menu has Record (Stop recording while recording,
Ctrl+R; enabled as the button is), Plot trace... (Ctrl+P), Live plot (Ctrl+L)
and Open captures folder (Explorer on the folder every trace is saved in); the
last three are only in the menu. Plot trace... is disabled while motion is
commanded (see Link stalls below); the others are always enabled.

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
it in `captures/`. While motion is commanded no dialog opens: the file stays in
`captures/` and the label reads "Saved trace_... in captures (no Save dialog
while the axis is under command)". Motion is commanded from the request of an
operation until the master reports it stopped, so a position move that has
arrived and holds ("At position") or a held pressure counts, though nothing
moves. Recording also ends, without a dialog, when the model stops (a
lost Modbus connection, for example), when **Disconnect** is pressed and when the
app is closed; the file is then saved to `captures/` and the trace status names it.
**Plot trace...** opens a file chooser in `captures/` and plots the file chosen.

During a procedure the interval belongs to the procedure (`record`, see
Recording from a procedure above): the dropdown is disabled and shows the
interval in force, a number starts a recording owned by the procedure that is
saved without a dialog when the run ends, and a recording the operator started
(Record during a run, or before it) takes the procedure's intervals while it
runs and returns to the dropdown's choice afterwards. Stopping a recording
during a run keeps the procedure from starting another in that run.

The controller drains the ring every `controllerPeriod` (50 ms, `traceDrain`,
which keeps none) whether or not the app is recording, and sends the new records
with its status; the ring holds 25.6 s. While the app is quiet the controller
holds them back, up to one ring's worth, so a stall of the app shorter than
that loses nothing. Each poll takes the records received, aligns them
(`traceAlign`, below), appends them to the live buffer and, while recording and
not paused, passes them through `traceDecimate` into the recording
(`traceAppend`). Records overwritten before the controller read them, or
dropped while the app was quiet for longer, are counted as missed: the label
turns red, the file's `missed` entry gives the total and the sequence column
jumps where they were lost. The controller drains the ring once more before it
stops the model, so a Disconnect loses nothing; after a model stop it did not
ask for (a lost Modbus connection) the records of its last pass are lost. The first rows of a recording start about two exchanges after the
button, because the alignment holds the newest two records back.

**Live plot** (Trace > Live plot; View > Side by side places it) opens a figure with four stacked axes over the last
`cfg.livePlotSeconds` (120 s): the pressure (the snapshot, and the window mean
with its minimum and maximum), the position from the end of travel, the velocity
and the torque, against the time before now. The aligned 20 Hz stream goes into
a circular buffer of `livePlotSeconds / exchangePeriod` rows (2400), and the figure is redrawn every
`cfg.livePlotPeriod` (0.5 s) while it is open, by setting the line data in place;
closing it stops the updates. `cfg.livePlotPosition` ([x y width height] px,
empty for MATLAB's default) sets where the window opens. The redraw is part of
the poll, so a poll that overruns `displayPeriod` shows in `app.PollStats`.
The figure is built to be cheap to lay out again when its window is resized:
fixed normalized inner positions for the axes, time labels on the bottom axes
only, the pressure lines named in the top title instead of a legend, and
MATLAB's axes interactions, axes toolbar and line picking off.

## Link stalls

Under Connected IO the shell steps on its MATLAB's own thread. While anything
holds that thread the model does not step, so the master writes nothing; the
steps it missed run back to back afterwards (`MaxMissedTicks` is large), but
the slave has seen its heartbeat stand still for that long. With motion
commanded (an operation requested, running, starting or stopping, a held
pressure included) the slave faults at `G_cfgCommTimeout` (300 ms) with
`FAULT_COMM`, which the app shows as "Master link lost". With no motion the
slave ignores the gap.

Window resizes, file dialogs, plots, file writes and anything typed in a
MATLAB command window would hold that thread long enough. That is why the
model runs in the controller's MATLAB, which runs nothing else (Controller
process, below): nothing the app does holds it back.

The app still measures every gap between master writes as the slave sees it:
the exchange summary's scan count of each window (`summaryScans`, IR33) times
`cfg.slaveScanPeriod` (4 ms), through `traceWriteGap` on the records it
receives. A gap of at least `cfg.stallWarn` (0.15 s; the first second after
Connect is not checked) is a link stall: it is counted in `app.PollStats`
(`writeGapMax`, the longest gap seen, and `stalls`), added to `app.StallLog`
(`time` and `gap_s`), printed on one line in the command window, and shown
after the status line for 15 s, "(link stall 0.42 s)". A stall now comes from
the controller's MATLAB or the PC, not from the app.

While motion is commanded the app refuses what can hold its own thread past
`cfg.appTimeout`, since the controller would then stop the motion (below):
Trace > Plot trace... and the whole Procedure menu (Open..., Reload and the
file items) are disabled and refused with a reason on the status line, and a
recording stopped then is saved to `captures/` without the Save dialog
(Recording a trace, above). Writing the file itself is short of
`cfg.appTimeout` (about 1.2 s for an hour at 0.05 s).

## Controller process

`controller/runController` runs the shell model in a MATLAB of its own. The app
starts it at its own start with `ctlLaunchCommand`: on Windows
`start "SOS Gen3 controller" /min "<matlab>" -batch "cd('<master-src/controller>');
runController"`, with `<matlab>` the app's own MATLAB, or
`cfg.controllerMatlab` when set (a compiled app needs it). The `-batch` MATLAB
ends when `runController` returns.

**Connection.** The controller listens on `127.0.0.1:cfg.controllerPort` (50321;
local only, so no firewall rule is involved), one app at a time. The app tries
to connect every second, launching the controller the first time none answers,
and waits up to `cfg.controllerStartMax` (180 s) for it, the status line
counting the seconds; a controller that does not come up is reported with a
pointer to its log. A controller already running (from an app that closed or
failed within the last minute) is connected to instead, and the app continues
from its `CommandSeq`. Messages are JSON objects, one per line (`ctlEncode`,
`ctlDecode`):

| From | `t` | Fields | Meaning |
|---|---|---|---|
| app | `hello`, `ping` | | the app is there; it pings on every poll |
| app | `set` | `name`, `value` | write one Constant block (`ctlBlocks().write`, a finite number); others are refused with a note |
| app | `start` | | start the model when stopped |
| app | `stop` | | stop the model once the master reports the axis stopped (status outside 1 to 8), or after `tStopMax` + 0.5 s |
| app | `track` | `file`, `n` | load the `n` doubles of `file` into `trackTable` |
| app | `quit` | | stop motion, stop the model as for `stop`, then end |
| controller | `k` | | keepalive to a quiet app; ignored |
| controller | `status` | `running`, `sim`, `live`, `cmdSeq`, `values`, `rec`, `cols`, `lastSeq`, `missed`, `notes` | every pass: whether the model runs, the simulation's status, the `ctlBlocks().read` outputs (null when unknown), the CommandSeq it holds, the new trace records row after row, the reader's counts and texts for the app's status line |

**Loop.** Every `cfg.controllerPeriod` (50 ms) the controller reads the app's
messages and the model's outputs, decides with `controllerStep` (no I/O, tested
by `tests/controllerBridge.m`), writes the blocks in order, loads a track
table, starts or stops the model, drains the trace ring and sends the status,
then pauses, which lets the model step. An app write reaches the model within
one period. It sends only while the app's last message is at most
`cfg.controllerSendQuiet` (0.5 s) old, so a stalled app that reads nothing does
not fill the connection and block the controller; to a quiet app it writes only
a keepalive line (`{"t":"k"}`) every `cfg.controllerKeepalive` (0.25 s), which
the app ignores. A connection whose app has died (ended from the Task Manager)
is seen only when written to: a failed write, or a server that reports no
client after one, counts as the connection closed, and the controller opens its
server again so the next app can connect.

**Without the app.** The app is present while its connection is open and its
last message is at most `cfg.appTimeout` (5 s) old. When it is not and the model
runs, the controller stops motion once (RequestedMode 0, TrackMode 0, a new
CommandSeq): within a keepalive period or two when the connection closes (the
app crashed or was ended), after 5 s of silence when it hangs. The model keeps running, so an app
reopened within `cfg.controllerOrphanTime` (60 s) finds it connected. After
60 s without the app the controller stops the model (the safety relay drops)
and ends; it also ends when no app reaches it within 60 s of its start.

**Log.** The controller's events (start, app connected and gone, model started
and stopped, refused writes, track loads, errors) go to
`captures/controller.log` with a timestamp, and to its own window. It can be
run by hand from a MATLAB session (`cd master-src/controller; runController`)
to watch them; it then returns instead of ending that MATLAB, and the app
connects to it as usual.

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
| `salineVolume_mL` | `SalineVolume` block: the fill (a procedure's sink, else `cfg.salineVolume`) | mL |
| `oxygenRatio` | `OxygenRatio` block: the fill (a procedure's sink, else `cfg.oxygenRatio`) | STP volume per saline volume |
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
and link health). The overview always draws the snapshot
columns; when the file has window or interval statistics (`summaryScans` above
0) the pressure row adds their mean, minimum and maximum as steps and the
position, velocity and torque rows add the mean as steps, from `tStartRel_s` in
a decimated file. Fault, link-down and E-STOP intervals are shaded and lines
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
