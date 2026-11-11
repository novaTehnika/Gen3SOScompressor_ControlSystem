# System Architecture

## Gen3 SOS Compressor Servo Controller
### Software Architecture and Data Flow

---

## 1. Introduction

This document provides a high-level overview of the slave controller's software architecture. It describes the main functional blocks, their interactions, and the flow of data through the system during key operations.

---

## 2. Software Block Diagram

The application spans `PRG_Main` (Structured Text), which contains the state machine and all custom function blocks, and two Ladder Diagram POUs that hold every built-in motion FB on the axis: `PRG_LD_Read`, first in the scan, with the blocks that only read it (`MC_ReadActualPosition`, `MC_ReadActualVelocity`, `MC_ReadActualTorque`, `MC_ReadStatus`), and `PRG_LD`, last, with the blocks that command it (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_SetPosition`, Jog, `Y_DirectControl`). `PRG_LD_Read` writes the feedback globals (`G_sysActual*`, `G_sysDriveFault`); `PRG_Main` and `PRG_LD` exchange data each scan through a pair of structured global families:

- **`G_cmd*`** — written by `PRG_Main`, read by `PRG_LD`. Each struct mirrors the pin names of its target built-in FB (`G_cmdPower`, `G_cmdStop`, `G_cmdReset`, `G_cmdMoveAbsolute`, `G_cmdJog`, `G_cmdDirectControl`, `G_cmdSetPosition`). Command Arbitration fills every field each scan before the owning state or FB overwrites it. A target or velocity defaults to zero, as it cannot matter while the block's enable is FALSE; an acceleration or deceleration keeps the value of the block that may still be running after its owning state is left (`MC_MoveAbsolute` is not aborted by `Execute` falling, and Jog decelerates on its `Deceleration` after `Forward`/`Reverse` drop). Every `MC_Stop` uses `G_cfgStopDecel`. `Y_DirectControl` has no deceleration pin and `ST_CmdDirectControl` no `Deceleration` field. `G_cfgVelLimitMax` is the speed ceiling of every mode; in position mode `Y_DirectControl`'s `Velocity` input is the path shaper's speed (`rPosShaperVelMax`), so the two never disagree.
- **`G_sta*`** — written by `PRG_LD`, read by `PRG_Main` and by custom FBs that need built-in FB status (`G_staPower`, `G_staStop`, `G_staReset`, `G_staMoveAbsolute`, `G_staJog`, `G_staDirectControl`, `G_staSetPosition`).

`PRG_Main` runs in six phases on every 4 ms scan (the cyclic task interval, mirrored by `G_cfgScanTime`). A phase reads only what an earlier phase produced in the same scan, so a reading reaches the command it leads to in the scan it is sampled in.

```
+-----------------------------------------------------------------------------+
|                              One scan (4 ms)                                |
+-----------------------------------------------------------------------------+
| PRG_LD_Read    MC_ReadActual*, MC_ReadStatus: axis -> G_sysActual*,         |
|                G_sysDriveFault                                              |
|                                                                             |
| PRG_Input      raw inputs and holding registers -> G_di*, G_ai*, G_sys*     |
|                                                                             |
| PRG_Main                                                                    |
|   1 Sense      digital input filters, FB_AnalogInputFilter ->               |
|                FB_PressureInput, FB_ModeDecoder, FB_CommWatchdog,           |
|                limit, ceiling and end-of-travel flags, FB_PistonExitGuard   |
|        |                                                                    |
|        v                                                                    |
|   2 Supervise  FB_SafetyMonitor -> ST_FAULT                                 |
|        |                                                                    |
|        v                                                                    |
|   3 Sequence   state machine (IF/ELSIF), state properties, timers           |
|        |                 ^                                                  |
|        v                 | block status, one scan old                       |
|   4 Control    FB_HandshakeManager, FB_FaultResetHandler, FB_GoHome,        |
|                FB_HomeLimit, FB_PositionPathShaper, FB_PressureController   |
|        |                                                                    |
|        v                                                                    |
|   5 Command    G_cmd*: arbitration, block outputs, limit / ceiling /        |
|                end-stop blocking of the pressure loop's command             |
|        |                                                                    |
|        v                                                                    |
|   6 Report     status, FB_ExchangeSummary, FB_Probe                         |
|                                                                             |
| PRG_Output     G_mb* input registers, digital outputs                       |
|                                                                             |
| PRG_LD         MC_*, Y_DirectControl, Jog: G_cmd* -> drive                  |
|                G_sta* -> the next scan                                      |
+-----------------------------------------------------------------------------+

```

The phases, and what each may read:

| Phase | Contents | Reads |
|---|---|---|
| 1 Sense | The debounce filters, the analog filter and the pressure scaling on its output, the mode decoder, the link watchdog, the limit, ceiling and end-of-travel flags, the piston exit guard. None depends on the state. | This scan's inputs (`PRG_Input`) and axis feedback (`PRG_LD_Read`) |
| 2 Supervise | `FB_SafetyMonitor`. A fault moves `G_sysCurrentState` to `ST_FAULT` before the state machine runs. | Sense; the state and its property flags as the previous scan left them |
| 3 Sequence | The state machine, the state properties, the timers. | Sense, Supervise; the Control blocks' status and the timers of the previous scan |
| 4 Control | The blocks the state enables: handshake, fault reset, Go Home, homing, the position path shaper, the pressure loop. | Sense, Sequence |
| 5 Command | The `G_cmd*` structs: the defaults, each state's commands, the Control blocks' outputs. | Sense, Sequence, Control |
| 6 Report | The status and feedback for `PRG_Output`, the exchange summary, the step probe. | Everything above |

Three reads go back one scan. The state machine reads the status of the Control blocks (handshake complete, Go Home done, the shaper's command) and the timers as the previous scan left them, since those blocks are enabled by the state they report to. The pressure loop's integrator is reset by the block of its own command (ceiling, end stop), found in the Command phase of the previous scan. And `PRG_LD`, which runs after `PRG_Output` so that a command reaches the drive in the scan it is written, returns its blocks' status (`G_sta*`) in the next scan. The axis feedback (`G_sysActual*`, `G_sysDriveFault`) does not go back a scan: `PRG_LD_Read` reads it before `PRG_Input`, in the scan that uses it.

Scans added by the order of the program, beyond the filters' own time constants and the drive:

| Path | Scans |
|---|---|
| Transducer voltage to the pressure loop's velocity command | 0 |
| Transducer voltage to the pressure ceiling block | 0 |
| Overtravel switch to a zeroed command, and to the fault stop | 0 |
| Piston exit, link loss or transducer fault to the stop command | 0 |
| MotionEnable dropping to the hold | 0 |
| Mode code to the state machine seeing the handshake complete | 1 |
| Drive alarm, and axis feedback, to `PRG_Main` | 1 |

### Component Descriptions

-   **Input Filters:** A layer of digital (`FB_DigitalInputFilter`) and analog (`FB_AnalogInputFilter`) filters that debounce the MotionEnable and FaultReset registers (`G_mbMotionEnable`, `G_mbFaultReset`) and the overtravel switches, and remove noise from the pressure input, providing stable values for the rest of the program.
-   **Decoders/Processors:** Function blocks that convert raw, filtered inputs into meaningful data (e.g., `FB_ModeDecoder` debounces the mode code register `G_mbModeCode` and converts it into an `E_OperatingMode`; `FB_PressureInput` scales the pressure transducer voltage to MPa).
-   **Core Logic (State Machine):** The `IF/ELSIF` chain in `PRG_Main` that orchestrates all system behavior based on the current state (`G_sysCurrentState`). MotionWorks IEC Express does not support enum-valued `CASE` selectors, hence the `IF/ELSIF` pattern.
-   **Control Blocks (`Y_DirectControl`, `MC_*`):** PLCopen (`MC_`) and Yaskawa-specific (`Y_`) function blocks. These live in `PRG_LD` and are driven from ST through the `G_cmd*` / `G_sta*` globals.
-   **Homing FBs (`FB_HomeLimit`, `FB_GoHome`):** Emit built-in FB commands via `Cmd*` `VAR_OUTPUT` structs and receive built-in FB status via `Sta*` `VAR_INPUT` structs. `PRG_Main` bridges these to the matching globals.
-   **Safety Monitor:** A watchdog (`FB_SafetyMonitor`) that runs every scan, after the inputs are conditioned and before the state machine, so a fault it finds is acted on in the same scan.
-   **Exchange Summary (`FB_ExchangeSummary`):** Summarises the scans between two changes of the master's heartbeat (mean, minimum and maximum pressure, mean velocity, torque and position, the scan count and a window number) and `PRG_Output` publishes the last closed window in IR21-34. It is called in `PRG_Main` before the step probe and reads no globals.
-   **Step Probe (`FB_Probe`):** An optional recorder that runs beside the main state machine, called last in `PRG_Main`. Every scan it appends a record (commanded and actual velocity, torque, position, pressure, raw pressure input, status word) to a ring of 4096 records and serves them to the bench scripts through a 13-record window in IR401-524. It has two state machines of its own: the recording state (`E_ProbeState`: `PROBE_OFF`, `PROBE_RECORDING`, `PROBE_CAPTURING`, `PROBE_HELD`) follows the master's arms with a length, and the transfer state (`E_ProbeXferState`: `XFER_IDLE`, `XFER_FILLING`, `XFER_PENDING`, `XFER_OVERRUN`) copies the requested records into a staging array at most `G_cfgProbeFillPerScan` per scan and lets `PRG_Output` publish the window in the scan the copy completes. On the scan `PRG_Main` enters `ST_FAULT` the block latches that scan's record index as `FaultMark`. Nothing in the protocol depends on it (registers: ModbusRegisterMap.md, section 6).
-   **Output Processors:** `PRG_Output` publishes the confirmed mode code (`G_mbConfCode`, IR3), the fault code (`G_mbFaultCode`, IR4), the status bits and the feedback in the Modbus input registers, and drives the status and brake digital outputs.

---

## 3. Data Flow Analysis

This section describes the path of data during two primary scenarios.

### Scenario 1: Position Command Input

This flow describes how a position reference from the master controller becomes motion.

1.  **Modbus Input:** The master writes the reference in millimeters to the holding register `G_mbReference` (`AT %MD3.483336 : REAL`), together with the mode code, MotionEnable, FaultReset and its heartbeat, and the position-move speed in `G_mbPositionSpeed` (`AT %MD3.483340 : REAL`, mm/s), which `PRG_Input` converts to `G_sysPositionSpeed`.
2.  **Mapping:** `PRG_Input` copies it to `G_sysReference` (`LREAL`) and the operator speed to `G_sysPositionSpeed` (`LREAL`). `PRG_Main` passes the mode code register `G_mbModeCode` to `FB_ModeDecoder`, which debounces and decodes it.
3.  **State Machine Execution:** The `ST_POSITION_CTRL` state (and, during fault recovery, `ST_RECOVERY_POSITION`) is active and clamps `G_sysReference` to the active software limits (`stCurrentLimits.PosMin`, `stCurrentLimits.PosMax`), storing the result in a local `rPosTarget`.
4.  **Path Shaping:** `Y_DirectControl` does no path shaping in PositionMode (the command reaches the drive as given), so `fbPosShaper` (`FB_PositionPathShaper`) shapes `rPosTarget` into a per-scan command: it limits command velocity to `rPosShaperVelMax`, the operator speed `rOperatorSpeed = LIMIT(0, G_sysPositionSpeed, G_cfgVelLimitMax)` evaluated every scan (a speed of 0 brakes the command to rest and holds), and acceleration to `G_cfgPosShaperAccelMax`, seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosShaperMaxDeviation` of actual position so it cannot run away from the real axis if motion stalls. This stage runs in both `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`; velocity mode and torque mode are unchanged.
5.  **Route Active FB Command Outputs:** `fbPosShaper.CommandedPosition` becomes `G_sysCommandedPosition` and is written into `G_cmdDirectControl.Position`, enabling direct control.
6.  **Drive Command:** `PRG_LD`'s `Y_DirectControl` instance reads `G_cmdDirectControl` each scan, commands the Yaskawa drive, and the motor moves.

### Scenario 2: Position Feedback Output

This flow describes how the motor's physical position is reported back to the master.

1.  **Encoder Reading:** The Yaskawa drive continuously tracks the motor's absolute encoder position.
2.  **PLCopen Feedback:** `MC_ReadActualPosition` in `PRG_LD_Read` retrieves the current axis position at the start of each scan and writes it to `G_sysActualPosition`.
3.  **Modbus Output:** `PRG_Output` writes `G_sysActualPosition`, `G_sysActualVelocity`, `G_sysActualTorque` and `G_sysCommandedPosition` as `REAL` values to the input registers (`G_mbActualPosition` at `%QD73812` and following), alongside the heartbeat echo, scan counter, confirmation code, fault code and status bits. The master reads them every 50 ms.

---

## 4. Key Function Block Roles (`PRG_Main`)

The main program orchestrates a number of critical function blocks.

| Instance | FB Type | Role in Architecture |
|---|---|---|
| `fbFilter*` | `FB_Digital/AnalogInputFilter` | **Input Conditioning:** Provide clean, stable signals to the core logic. |
| `fbModeDecoder`| `FB_ModeDecoder` | **Command Interpretation:** Debounces the master's mode code (`G_mbModeCode`, HR2) and converts it into a meaningful `E_OperatingMode` enum. |
| `fbHandshake`| `FB_HandshakeManager`| **Mode Synchronization:** Manages the mode entry handshake protocol with the master (confirmation, timeout, verification). |
| `fbFaultReset`| `FB_FaultResetHandler`| **Fault Reset Validation:** Validates fault reset conditions — FaultReset HIGH, MotionEnable LOW, mode code stable, and master fault code acknowledgement matches active fault. |
| `fbPosShaper` | `FB_PositionPathShaper` | **Command Shaping:** In `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`, limits the position command's velocity (the operator speed, `G_sysPositionSpeed`, limited to `G_cfgVelLimitMax`) and acceleration (`G_cfgPosShaperAccelMax`), seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosShaperMaxDeviation` of actual position. Velocity and torque modes are unchanged. |
| `fbReadActual*`| `MC_ReadActual*` | **Feedback:** The primary source of feedback from the drive, providing real-time position, velocity, and torque. |
| `fbPower` | `MC_Power` | **Drive Control:** Enables and disables power to the servo drive. |
| `fbStop` | `MC_Stop` | **Safe Stop:** Used to execute a controlled stop when changing modes or entering a fault state. `Y_DirectControl` is kept enabled with a zero command until `MC_Stop` has aborted it, which is what returns the SERVOPACK from velocity/torque control to position control (Yaskawa AN.MPIEC.30 §4.1.3). |
| `fbDirectControl`| `Y_DirectControl`| **Motion Command:** The primary interface for sending real-time position, velocity, or torque commands to the drive during operational states. |
| `fbHome*` | `FB_HomeLimit`, `FB_GoHome` | **Homing Logic:** Encapsulates the multi-step homing sequence and the Go Home move. `PRG_Main` caps the Go Home velocity with the operator speed (HR7-8, `rOperatorSpeed`) when the move starts; homing keeps its configured speeds. |
| `fbSafetyMonitor`| `FB_SafetyMonitor` | **System Watchdog:** The central authority for detecting unsafe conditions and triggering a fault. |
| `fbCommWatchdog` | `FB_CommWatchdog` | **Link Supervision:** Raises `FAULT_COMM` through the safety monitor when the master heartbeat register stops changing. |
| `fbExchangeSummary` | `FB_ExchangeSummary` | **Exchange Statistics:** Accumulates the pressure, velocity, torque and position of the scans between two changes of the master's heartbeat and publishes the means, the pressure minimum and maximum, the scan count and the window number when a window closes. Called once, before `fbProbe`. |
| `fbProbe` | `FB_Probe` | **Data Capture:** Records every scan into the probe ring, serves the master's window requests and marks the scan of a fault. Called once, last in the file, so the scan's state, status bits and confirmed mode code are final. Optional; `G_cfgProbeEnable` FALSE stops it. |
| `fbIntToFaultCode` | `FB_IntToFaultCode` | **Fault Acknowledgement:** Decodes the full fault code the master mirrors in the mode register for `FB_FaultResetHandler`. |
