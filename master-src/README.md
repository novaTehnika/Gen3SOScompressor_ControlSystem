# Master

The master runs on the lab PC under Simulink Desktop Real-Time and talks to the
MP2600iec slave over Modbus TCP (`docs/master/ModbusRegisterMap.md`), with the
safety relay and the stir bar motor on the NI PCI-6221 DAQ. The pressure
transducer is on the slave, which closes the pressure loop and reports the
reading over the link.

| Path | Contents |
|------|----------|
| `logic/` | All master logic, as plain `.m` functions that run under MATLAB and GNU Octave. `masterStep.m` is the entry point called once per model step; `masterStateMachine.m` sequences the handshake, motion and fault handling; `masterConfig.m` holds every constant. `proc*.m` is the operator app's procedure engine and `opTrack.m` the rule by which the app and the engine follow an operation through `StatusCode`. |
| `procedure/` | The procedure the operator app walks through in non-manual mode, as data: `sopProcedure.m`. The SOP replaces this file; no code changes. |
| `trace/` | The trace recorder's host-side functions, plain `.m` files that run under MATLAB and GNU Octave: `traceStart` and `traceDrain` (collect records from the ring), `traceMeta` and `traceWriteCsv` (write a trace file), `traceReadCsv` and `traceDerive` (read it back and add the displayed quantities), `tracePlot` (three figures) and `traceColumnNames`. The block that fills the ring, `traceStep`, and the ring's layout, `traceLayout`, are in `logic/`. |
| `captures/` | Trace files the app saves (gitignored; created on first use). |
| `shell/buildShell.m` | Generates `shell/SOS_Gen3_Master_Shell.slx`: Modbus Client blocks at the 50 ms exchange, DAQ blocks, one MATLAB Function block calling `masterStep`, a second one (`TraceStep`) calling `traceStep`, the Constant blocks the app writes and the Gain blocks it reads. The model holds no logic and is rebuilt from this script, not edited by hand. |
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
count; the Read row reads 20) and the Modbus client (`ModbusClient`, TCP/IP,
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

The `TraceStep` block records the link for the Record button. It takes the raw
read vector, the write vector as `masterStep` produced it, `masterStep`'s
`doSafety`, `StatusCode`, `FaultCode`, `State` and `LinkOk`, and the app's
`RequestedMode`, `CommandSeq`, `TargetPosition`, `TargetPressure`, `ESTOP` and
`Mixer` blocks. Once per Modbus exchange it writes one record into a ring of
512 records (25.6 s) and outputs the whole ring each step to the `TraceRing`
Gain block, a test point the app reads like the other Gain blocks. Each record
carries its sequence number in the first column and again in the last (the
guard), so the app takes only rows whose two copies agree and sees from the
sequence numbers whether the ring overwrote records it had not read yet.
`masterStep` and the register map are unchanged.

The app reads `StatusCode`, `FaultCode`, `State`, `Position`,
`Velocity`, `Pressure` (the slave's gauge transducer reading plus `atmAmbient`, atm absolute) and
`LinkOk`. Status codes are listed in
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

The app shows volume (mL), absolute pressure (atm), position (mm from the end
of travel) and flow (mL/s, positive toward the end of travel). Position targets
in manual control are also mm from the end of travel, and pressure targets are
atm absolute. `masterConfig` holds the end of travel (`posEOT`), bore and dead
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
sets the one speed of jog, position moves, Go Home and homing, and a change applies
to a running operation; 0 holds, and the status line then reads "held: speed 0". STOP turns dark red while E-STOP is
requested but not yet reported by the model, and yellow while the model reports
E-STOP (`StatusCode` 20); pressing it then releases E-STOP.

## Procedure (non-manual mode)

The engine is `procNew`, `procAction`, `procStep` and `procView` in `logic/`:
pure functions on one state struct `p`, with no persistent state, no timers and
no app handle. The app calls `procStep(p, in, dt)` on every poll with what it
read from the model (`connected`, `statusCode`, `faultCode`, `estop`, `pressure`,
`volume`) and executes the returned requests (`params`, `mixer`, `stop`, `op`
with `target` and `unit`, `zeroVolume`) through its own operation requests, so
the interlocks stay in one place. `procAction` only records the operator's
intent (start, confirm, repeat, check, setParam, pause, resume, abort, reset)
and refuses an action the state does not allow; `procView` gives what the panel
shows. `tests/smokeProcedureEngine.m` runs all of it against a stand-in for the
model's status.

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
procedure's volume is the swept volume of the readouts, and `Disconnect`
during a procedure is left to the engine, which pauses with "Model stopped".
The volume-zero request also zeroes the Incr view. A procedure that does not
validate leaves the panel empty, Start Procedure disabled and the reason on the
status line.

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
| `op` | machine | required | `home`, `goHome`, `position`, `pressure` or `stop` |
| `target` | position, pressure | required | a number, or a parameter key |
| `unit` | position | `'mm'` | `mm` from the end of travel, or `mL` chamber volume (`deadVolume` plus the swept volume); a pressure is atm absolute |
| `until` | machine, wait | by op | the completion condition (below) |
| `keep` | position, pressure | `false` | leave the operation running as the background operation; otherwise the engine stops it once the step completes |
| `tolerance` | settled | `cfg.procPressureTol` | atm |
| `duration` | wait; settled | required for a timed wait; `cfg.procSettleTime` | s |
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
status. `procNew` rejects a procedure that cannot run (unknown type, op, unit,
`until`; missing title or target; a key that names no parameter; duplicate id;
a `repeatFrom` that is not an earlier step; too many checks or parameters; an
unknown sink; `keep` on a step that cannot hold; a settled wait with no earlier
pressure step with `keep`).

### States

`idle`, `running`, `waitOperator` (confirm and param steps), `paused`,
`aborting`, `done` (outcome `completed` or `aborted`) and `faulted`.

- **Pause** stops the current step's own operation and keeps a `keep` pressure
  hold running; a timed wait stops counting. Resume re-issues an interrupted
  move and continues a wait or an operator step where it was.
- **Abort** requests a stop, waits for the axis (at most `tStopMax` plus 0.5 s)
  and ends `done`/`aborted`; with the model not running it ends at once.
- **Slave fault** (status 10) ends the procedure `faulted`, with the fault
  code. Nothing resumes: after the fault reset the operator presses Start
  Procedure to begin again at step 1.
- **E-STOP, slave link lost and model stopped** pause the procedure with the
  reason shown. The master has already stopped everything, so no stop is sent;
  an interrupted move is issued again and a lost pressure hold re-run (and the
  step that was paused returned to, with its ticks) on resume. The timer keeps
  counting. Resume is refused while the condition holds and enabled once it
  has cleared; Abort is always available.
- **A background operation that ends** (the pressure hold is left by the
  master) pauses the procedure; Resume re-runs the step that started it and
  returns to the paused step.
- The mixer is switched off at abort, fault and completion only if the
  procedure switched it on.

### Readouts

The timer runs from the step with `timerStart` to the step with `timerStop`
(and through a pause). The volume discharged is the volume at the step with
`zeroVolume` minus the present volume.

### Writing the SOP

Edit `procedure/sopProcedure.m` (or add a file there and set `cfg.procedure` in
`masterConfig.m`) with no code change, then run
`tests/run_all.sh smokeProcedureEngine`: it validates the shipped procedure
through `procNew`.
## Recording a trace

The Trace panel (between the lamps and the Procedure panel) records what
crosses the link. **Record** is available while connected. Pressing it starts a recording of the exchanges
from that moment, and the panel shows the samples taken, the elapsed time and,
in red, any records missed. Pressing it again (**Stop recording**) ends the
recording, writes the file at once to `captures/trace_yyyymmdd_HHMMSS.csv` and
then opens a Save dialog proposing that path: choosing another name or folder
moves the file there, and cancelling keeps it in `captures/`. Recording also
ends, without a dialog, when the model stops (a lost Modbus connection, for
example), when **Disconnect** is pressed and when the app is closed; the file
is then saved to `captures/` and the panel names it. **Plot trace...** opens
a file chooser in `captures/` and plots the file chosen.

The app reads the ring on its display poll (`displayPeriod`, 0.1 s) while recording. The ring holds 25.6 s,
so a stall of the app shorter than that loses nothing. Records overwritten
before they were read are counted as missed: the panel turns red, the file's
`missed` entry gives the total and the sequence column jumps where they were
lost. The records between the last poll and a model stop (at most one poll) are not
captured; pressing **Disconnect** reads the ring once more before stopping.

## Trace files

A trace file is a CSV with a `# key: value` header block, a line of column
names and one row per Modbus exchange (50 ms). Values are the raw engineering
units of the link: position in slave axis millimetres, pressure in MPa gauge,
targets as the app writes them. The geometry in force at capture time is
stored in the header, so the displayed quantities are derived from the file
alone.

| Metadata key | Meaning |
|--------------|---------|
| `format` | File format version (1). |
| `source` | Writer of the file. |
| `started` | Wall-clock start, `yyyy-mm-ddTHH:MM:SS`. |
| `records`, `missed` | Rows written and records lost to ring overruns. |
| `dt_s`, `exchangePeriod_s` | Model step and Modbus exchange period. |
| `posEOT_mm`, `boreDiameter_mm`, `deadVolume_mL`, `mixingDeadVolume_mL` | Cylinder geometry (`masterConfig`). |
| `atmPerMPa`, `atmAmbient_atm` | Pressure scaling and the atmosphere added to gauge pressure. |
| `referenceUnits` | Units of the `reference` column by mode. |

| Column | Source | Unit |
|--------|--------|------|
| `seq` | Record number since model start, from 1 | - |
| `t_s` | Model time of the recording step | s |
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
| `modeCode`, `motionEnable`, `faultReset` | HR2 to HR4 as sent | code, 0/1, 0/1 |
| `reference` | HR5-6 as sent | mm (position), mm/s (jog), MPa gauge (pressure) |
| `positionSpeed_mm_s` | HR7-8 as sent | mm/s |
| `salineVolume_mL` | HR9-10 as sent | mL |
| `oxygenRatio` | HR11-12 as sent | STP volume per saline volume |
| `statusCode`, `faultCode`, `state` | `masterStep` outputs (`state` is `masterStateMachine`'s `ST_*`) | code |
| `linkOk`, `doSafety` | `masterStep` outputs | 0/1 |
| `op`, `cmdSeq` | `RequestedMode`, `CommandSeq` blocks | code, count |
| `targetPosition_mm` | `TargetPosition` block | mm, slave axis |
| `targetPressure_atm` | `TargetPressure` block | atm absolute |
| `estop`, `mixer` | `ESTOP`, `Mixer` blocks | 0/1 |

`traceLayout.m` defines the columns and `traceColumnNames` gives the header.
Read and plot a file from MATLAB or Octave (with `master-src/logic` and
`master-src/trace` on the path):

```matlab
d = traceReadCsv(file);     % d.meta, d.names, d.data and one column vector per name
d = traceDerive(d);         % adds posFromEOT_mm, sweptVolume_mL, mixingVolume_mL, flow_mL_s, pressureAbs_atm, gap, tRel_s, ...
figs = tracePlot(file);     % a file name, the struct from traceReadCsv, or [] to choose a file
figs = tracePlot(d, 'visible', 'off', 'save', true, 'saveDir', folder);
```

`tracePlot` returns three figures: `overview` (pressure, position, velocity
and torque against time), `delivery` (swept volume, flow, and pressure against
swept volume) and `status` (status and mode codes, the operation flags, link state
and link health). Fault, link-down and E-STOP intervals are shaded and lines
break where records were missed. With `'save'` on, each figure is saved as
`<csv name>_<overview|delivery|status>.fig` in `saveDir` (the file's folder
by default).

## Testing without MATLAB

`tests/run_all.sh` (repository root) needs `python3` and `octave-cli`. It
translates `slave-src` into a simulated slave with `tools/st_translate.py` and runs the
master logic against it in closed loop. The trace recorder, the ring drain and
the CSV and plot functions are tested against a stand-in slave. The Simulink
shell and the app (including the `TraceRing` read and the Record button) are
not covered; they can only be exercised on the lab PC.
