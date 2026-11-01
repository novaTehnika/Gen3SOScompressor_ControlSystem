# PRG_LD specification

`PRG_LD` is the Ladder Diagram POU of the MotionWorks IEC project. It is
the only POU that owns the axis and the built-in motion function blocks:
the Structured Text never calls them. It runs on the one cyclic task with
`PRG_Input`, `PRG_Main` and `PRG_Output`, and every block in it executes
every scan. The POU is not in this repository; this file is its
specification, and the screenshots beside it (`<instance>.PNG`, taken
offline) are the record of what the project holds. The two are kept to
agree: a change to a field of `ST_Cmd*` or `ST_Sta*` in
`slave-src/DUT/DataTypes.st` is complete only when the pin here, the
wiring in the project and the screenshot all show it.

## Contract

- The axis is `AXIS1`, the axis variable Hardware Configuration creates for
  the Sigma-7 on the MP2600iec (the developer guides call it `G_sysAxis`).
  Every block's `Axis` input and output is wired to it.
- Inputs come from the `G_cmd*` structured globals, written by `PRG_Main`
  every scan: it defaults every command to idle at the top of its command
  arbitration and the owning state or function block (`FB_GoHome`,
  `FB_HomeLimit`, `FB_PositionPathShaper`) overwrites what it commands.
  Field names are the pin names, so each struct field is wired to the pin
  of the same name.
- Outputs go to the `G_sta*` structured globals and to the three
  `G_sysActual*` feedback globals, read by `PRG_Main` and the custom
  function blocks. Keep `PRG_LD` last in the task, after `PRG_Output`:
  the status `PRG_Main` reads is then one scan old, which is the order the
  simulator runs (`PRG_Input`, `PRG_Main`, `PRG_Output`, axis model) and
  the tests assume.
- Units are the axis's engineering units from Hardware Configuration:
  mm, mm/s, mm/s², and % of rated torque. Rates are always positive.
- Pins not listed as wired are left open. No block has `Jerk` or
  `BufferMode` wired; the blocks default them (no jerk limit, aborting).
  `ErrorID` outputs are not read by the ST; wire them to a local for
  diagnosis if wanted.
- Instance names are `<Block>_1`, as in the screenshots.

## Blocks

| Instance | Block | Role |
|---|---|---|
| `MC_Power_1` | `MC_Power` | Servo power on and off |
| `MC_Reset_1` | `MC_Reset` | Clears a drive alarm in `ST_DRIVE_RESET` and the fault recovery |
| `MC_Stop_1` | `MC_Stop` | The controlled stop of hold, fault and recovery; aborts `Y_DirectControl` to return the SERVOPACK to position control |
| `MC_MoveAbsolute_1` | `MC_MoveAbsolute` | Go Home move and the homing retract |
| `Jog_1` | `Jog` (Yaskawa toolbox) | Homing approach and back-off |
| `MC_SetPosition_1` | `MC_SetPosition` | Sets the homed zero |
| `Y_DirectControl_1` | `Y_DirectControl` | Position, velocity and torque command of the operating modes |
| `MC_ReadActualPosition_1` | `MC_ReadActualPosition` | Position feedback |
| `MC_ReadActualVelocity_1` | `MC_ReadActualVelocity` | Velocity feedback |
| `MC_ReadActualTorque_1` | `MC_ReadActualTorque` | Torque feedback |
| `MC_ReadStatus_1` | `MC_ReadStatus` | Drive alarm flag (not in the project yet, see below) |

`MC_MoveVelocity_1` is not part of the POU: nothing in the ST writes or
reads a velocity move, and the instance, its globals and its types are
deleted (`README.md`).

### MC_Power_1

Type `ST_CmdPower` in, `ST_StaPower` out. `PRG_Main` sets all three
enables TRUE together in every drive-active state and, in
`ST_FAULT_IDLE`, until the brake has engaged.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Enable | in | `G_cmdPower.Enable` | |
| Enable_Positive | in | `G_cmdPower.EnablePositive` | Shown open in the screenshot; wire it, the ST drives the field |
| Enable_Negative | in | `G_cmdPower.EnableNegative` | Shown open in the screenshot; wire it, the ST drives the field |
| BufferMode | in | open | |
| Axis | out | `AXIS1` | |
| Status | out | `G_staPower.Status` | `ST_DRIVE_ENABLE` waits for it; `ST_HOLD_POSITION` and `ST_FAULT_IDLE` treat it dropping as the drive going off |
| Busy | out | open | |
| Active | out | open | |
| Error | out | `G_staPower.Error` | `FAULT_DRIVE` in `ST_DRIVE_ENABLE` |
| ErrorID | out | open | |

### MC_Reset_1

Type `ST_CmdReset` in, `ST_StaReset` out. `Execute` is held TRUE for the
whole of `ST_DRIVE_RESET`.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Execute | in | `G_cmdReset.Execute` | |
| Axis | out | `AXIS1` | |
| Done | out | `G_staReset.Done` | Advances `ST_DRIVE_RESET` to `ST_DRIVE_ENABLE` |
| Busy | out | `G_staReset.Busy` | Not read by the ST |
| Error | out | `G_staReset.Error` | `FAULT_DRIVE` in `ST_DRIVE_RESET` |
| ErrorID | out | open | |

### MC_Stop_1

Type `ST_CmdStop` in, `ST_StaStop` out. `Execute` is TRUE in
`ST_HOLD_POSITION` until the axis is halted and in `ST_FAULT` and
`ST_RECOVERY` until `Done`; `FB_GoHome` also executes it to abort the Go
Home move. `Deceleration` is `G_cfgStopDecel` in every case.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Execute | in | `G_cmdStop.Execute` | |
| Deceleration | in | `G_cmdStop.Deceleration` | mm/s² |
| Jerk | in | open | |
| BufferMode | in | open | |
| Axis | out | `AXIS1` | |
| Done | out | `G_staStop.Done` | Ends the stop in `ST_HOLD_POSITION`, `ST_FAULT`, `ST_RECOVERY` and `FB_GoHome` |
| Busy | out | `G_staStop.Busy` | With `G_staDirectControl.Active` FALSE, releases the `Y_DirectControl` hand-off |
| Active | out | open | |
| CommandAborted | out | open | |
| Error | out | `G_staStop.Error` | Also releases the hand-off |
| ErrorID | out | open | |

### MC_MoveAbsolute_1

Type `ST_CmdMoveAbsolute` in, `ST_StaMoveAbsolute` out. Commanded by
`FB_GoHome` (`CmdMoveAbsolute`) and by `FB_HomeLimit` for the retract;
`PRG_Main` copies the owning block's command struct whole. Outside those
states `Execute` is FALSE and the rates keep the Go Home values so a move
interrupted by a hold or fault keeps real rates until `MC_Stop` takes the
axis.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Execute | in | `G_cmdMoveAbsolute.Execute` | |
| Position | in | `G_cmdMoveAbsolute.Position` | mm, absolute |
| Velocity | in | `G_cmdMoveAbsolute.Velocity` | mm/s |
| Acceleration | in | `G_cmdMoveAbsolute.Acceleration` | mm/s² |
| Deceleration | in | `G_cmdMoveAbsolute.Deceleration` | mm/s² |
| Jerk | in | open | |
| Direction | in | `G_cmdMoveAbsolute.Direction` | `MC_Direction`; the ST sends `Current_Direction` |
| BufferMode | in | open | |
| Axis | out | `AXIS1` | |
| Done | out | `G_staMoveAbsolute.Done` | |
| Busy | out | `G_staMoveAbsolute.Busy` | |
| Active | out | `G_staMoveAbsolute.Active` | |
| CommandAborted | out | `G_staMoveAbsolute.CommandAborted` | |
| Error | out | `G_staMoveAbsolute.Error` | |
| ErrorID | out | open | |

### Jog_1

Type `ST_CmdJog` in, `ST_StaJog` out. Homing (`FB_HomeLimit`) is the only
user: fast and slow approach to the negative overtravel switch and the
back-off. Outside homing `Forward` and `Reverse` are FALSE and the rates
keep the homing values, so an abort into `ST_HOLD_POSITION` lets the block
decelerate on its own `Deceleration`.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Forward | in | `G_cmdJog.Forward` | |
| Reverse | in | `G_cmdJog.Reverse` | |
| Velocity | in | `G_cmdJog.Velocity` | mm/s |
| Acceleration | in | `G_cmdJog.Acceleration` | mm/s² |
| Deceleration | in | `G_cmdJog.Deceleration` | mm/s² |
| Jerk | in | open | |
| Axis | out | `AXIS1` | |
| InVelocity | out | `G_staJog.InVelocity` | |
| Done | out | `G_staJog.Done` | One-scan pulse once stopped after both inputs FALSE |
| Busy | out | `G_staJog.Busy` | |
| Error | out | `G_staJog.Error` | |
| ErrorID | out | `G_staJog.ErrorID` | The one `ErrorID` the ST carries (UINT) |

### MC_SetPosition_1

Type `ST_CmdSetPosition` in, `ST_StaSetPosition` out. `FB_HomeLimit`
executes it once at the end of homing with `G_cfgHomeLimSetPosition`.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Execute | in | `G_cmdSetPosition.Execute` | |
| Position | in | `G_cmdSetPosition.Position` | mm |
| Mode | in | `G_cmdSetPosition.Mode` | FALSE absolute, TRUE relative; the ST sends FALSE |
| Axis | out | `AXIS1` | |
| Done | out | `G_staSetPosition.Done` | |
| Busy | out | `G_staSetPosition.Busy` | |
| Error | out | `G_staSetPosition.Error` | |
| ErrorID | out | open | |

### Y_DirectControl_1

Type `ST_CmdDirectControl` in, `ST_StaDirectControl` out. `Enable` is
TRUE in the position, velocity, torque and pressure states, and is held
TRUE with a zero command through the stop of `ST_HOLD_POSITION`,
`ST_FAULT` and `ST_RECOVERY` until `MC_Stop` has aborted the block
(`G_staStop.Busy` with `Active` FALSE), which is what returns the
SERVOPACK from velocity or torque control to position control. The
`README.md` table carries the notes on each pin.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Enable | in | `G_cmdDirectControl.Enable` | |
| ControlMode | in | `G_cmdDirectControl.ControlMode` | `Y_ControlMode`: `PositionMode`, `VelocityTLMode` or `TorqueVLMode` |
| Position | in | `G_cmdDirectControl.Position` | mm; the shaper's command in position mode |
| Velocity | in | `G_cmdDirectControl.Velocity` | mm/s; the command in velocity and pressure modes, the limit in torque mode, the shaper's speed in position mode (sets no limit there) |
| Acceleration | in | `G_cmdDirectControl.Acceleration` | mm/s²; `G_cfgDirectCtrlAccel`, or `G_cfgPressureAccel` in `ST_PRESSURE_CTRL` |
| Torque | in | `G_cmdDirectControl.Torque` | % of rated; the command in torque mode, the limit otherwise |
| FilterTimeConstant | in | `G_cmdDirectControl.FilterTimeConstant` | s, `G_cfgDirectCtrlFilterTime` |
| BufferMode | in | open | |
| Axis | out | `AXIS1` | |
| Busy | out | open | |
| Active | out | `G_staDirectControl.Active` | Must be wired: the hand-off waits for it to drop |
| CommandAborted | out | open | |
| Error | out | `G_staDirectControl.Error` | Releases the hand-off |
| ErrorID | out | open | |

### MC_ReadActualPosition_1, MC_ReadActualVelocity_1, MC_ReadActualTorque_1

The three feedback readers. `Enable` is the left power rail (always TRUE),
so each reads every scan. Their outputs are the `G_sysActual*` globals,
the source of every position, velocity and torque the ST uses: the limit
and exit-guard checks, the path shaper's seed and tether, the in-motion
output, homing, the step probe record and the Modbus feedback registers.

| Instance | Pin | Direction | Wired to | Note |
|---|---|---|---|---|
| all three | Axis | in | `AXIS1` | |
| all three | Enable | in | power rail (TRUE) | |
| all three | Axis | out | `AXIS1` | |
| all three | Valid, Busy, Error, ErrorID | out | open | |
| `MC_ReadActualPosition_1` | Position | out | `G_sysActualPosition` | mm |
| `MC_ReadActualVelocity_1` | ActualVelocity | out | `G_sysActualVelocity` | mm/s |
| `MC_ReadActualTorque_1` | ActualTorque | out | `G_sysActualTorque` | % of rated |

### MC_ReadStatus_1 (to add)

`G_sysDriveFault` is the drive alarm input of `FB_SafetyMonitor`
(`FAULT_DRIVE` outside the reset and enable states, the path the fault
code reference gives for the amplifier's own alarms, encoder alarms
included). Nothing in the ST writes it and no block in the screenshots
does, so it stays FALSE and a drive alarm during motion is only noticed
through the block that was commanding. Add a status reader wired as
below, enabled from the power rail like the feedback readers. The axis is
in `ErrorStop` whenever the SERVOPACK holds an alarm and leaves it on
`MC_Reset`, which is what `ST_DRIVE_RESET` and the recovery expect of the
flag.

| Pin | Direction | Wired to | Note |
|---|---|---|---|
| Axis | in | `AXIS1` | |
| Enable | in | power rail (TRUE) | |
| Axis | out | `AXIS1` | |
| ErrorStop | out | `G_sysDriveFault` | |
| Valid, Busy, Error, ErrorID, other states | out | open | |

The simulator sets `G_sysDriveFault` directly (`sl.hook.driveFault`), so
the ST's response to the flag is tested; only the wiring is missing.

## Checks

- Every `G_cmd*` field is wired to a pin and every wired `G_sta*` field
  has a pin: the structs hold nothing the POU does not use. The two
  `MC_Power` direction enables are the one known gap.
- The axis's units in Hardware Configuration are mm and the torque unit
  is % of rated: the ST and the protocol scale nothing.
- After a change, export the GVL and compare it against
  `slave-src/GVL/GlobalVariables_Reference.st`, and replace the
  screenshot of every block touched.
