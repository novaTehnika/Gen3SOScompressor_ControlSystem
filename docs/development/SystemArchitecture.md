# System Architecture

## Gen3 SOS Compressor Servo Controller
### Software Architecture and Data Flow

---

## 1. Introduction

This document provides a high-level overview of the slave controller's software architecture. It describes the main functional blocks, their interactions, and the flow of data through the system during key operations.

---

## 2. Software Block Diagram

The application spans two POUs: `PRG_Main` (Structured Text) contains the state machine and all custom function blocks, and a Ladder Diagram POU owns `G_sysAxis` together with every built-in motion FB (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_MoveVelocity`, `MC_SetPosition`, `MC_ReadActual*`, `Y_DirectControl`). The two POUs exchange data each scan through a pair of structured global families:

- **`G_cmd*`** — written by `PRG_Main`, read by the LD POU. Each struct mirrors the pin names of its target built-in FB (`G_cmdPower`, `G_cmdStop`, `G_cmdReset`, `G_cmdMoveAbsolute`, `G_cmdMoveVelocity`, `G_cmdDirectControl`, `G_cmdSetPosition`).
- **`G_sta*`** — written by the LD POU, read by `PRG_Main` and by custom FBs that need built-in FB status (`G_staPower`, `G_staStop`, `G_staReset`, `G_staMoveAbsolute`, `G_staMoveVelocity`, `G_staDirectControl`, `G_staSetPosition`).

`PRG_Main` is structured as a series of processing stages that execute on every 2 ms scan cycle (the cyclic task interval, mirrored by `G_cfgScanTime`).

```
+-----------------------------------------------------------------------------+
|                               PRG_Main Scan Cycle                           |
+-----------------------------------------------------------------------------+
|                                                                             |
|  [Raw Inputs] ----->+-----------------+      +-----------------+             |
|   (di*, ai*)        |  Digital Input  |----->|  FB_ModeDecoder |-----+       |
|                     |     Filters     |      +-----------------+     |       |
|                     +-----------------+                            |       |
|                                                                    |       |
|                     +-----------------+      +-----------------+     |       |
|                     |  Analog Input   |----->| FB_AnalogProc   |-----+       |
|                     |     Filter      |      +-----------------+     |       |
|                     +-----------------+                            |       |
|                                                                    |       |
|  [Drive Feedback] ->+-----------------+                            |       |
|   (Position, Vel)   | MC_ReadActual*  |----------------------------+       |
|                     +-----------------+                            |       |
|                                                                    |       |
|  +-----------------------------------------------------------------+ V     |
|  |                            CORE LOGIC                           |       |
|  |                                                                 |       |
|  |  +-----------------+     +-----------------+     +-----------+  |       |
|  |  | FB_HandshakeMngr|<-+->|                 |<---+|  Homing   |  |       |
|  |  +-----------------+   | | IF-ELSIF States |    |   FBs     |  |       |
|  |                        +>| (State Machine) |<---+ (Go/Lim/EOT)|  |       |
|  |  +-----------------+   | |                 |    +-----------+  |       |
|  |  | Y_DirectControl |<--+ +-----------------+                   |       |
|  |  +-----------------+                                           |       |
|  |                                                                 |       |
|  +-----------------------------------------------------------------+       |
|                                     |                                      |
|  +----------------------------------+----------------------------------+   |
|  V                                                                    V   |
| +--------------------+                                     +-----------------+  |
| | FB_SafetyMonitor   |                                     |  FB_OutputMux   |  |
| +--------------------+                                     +-----------------+  |
|           |                                                          |       |
|           |                                                          |       |
|           +-----> [To State Machine for FAULT transition]             |       |
|                                                                      |       |
|                     +-----------------+      +-----------------+     |       |
|                     | FB_PositionOut  |<-----| Actual Position |     |       |
|                     +-----------------+      +-----------------+     |       |
|                           |                                          |       |
|  [Raw Outputs] <----------+------------------------------------------+       |
|   (do*, ao*)                                                                |
|                                                                             |
+-----------------------------------------------------------------------------+

```

### Component Descriptions

-   **Input Filters:** A layer of digital (`FB_DigitalInputFilter`) and analog (`FB_AnalogInputFilter`) filters that debounce signals and remove noise from the raw I/O, providing stable values for the rest of the program.
-   **Decoders/Processors:** Function blocks that convert raw, filtered inputs into meaningful data (e.g., `FB_ModeDecoder` converts three booleans into an `E_OperatingMode`).
-   **Core Logic (State Machine):** The `IF/ELSIF` chain in `PRG_Main` that orchestrates all system behavior based on the current state (`G_sysCurrentState`). MotionWorks IEC Express does not support enum-valued `CASE` selectors, hence the `IF/ELSIF` pattern.
-   **Control Blocks (`Y_DirectControl`, `MC_*`):** PLCopen (`MC_`) and Yaskawa-specific (`Y_`) function blocks. These live in the LD POU and are driven from ST through the `G_cmd*` / `G_sta*` globals.
-   **Homing FBs (`FB_HomeLimit`, `FB_HomeEOT`, `FB_GoHome`):** Emit built-in FB commands via `Cmd*` `VAR_OUTPUT` structs and receive built-in FB status via `Sta*` `VAR_INPUT` structs. `PRG_Main` bridges these to the matching globals.
-   **Safety Monitor:** A watchdog (`FB_SafetyMonitor`) that runs every cycle to check for fault conditions.
-   **Output Processors:** Function blocks (`FB_OutputMux`, `FB_PositionOutput`) that format system data into the correct signals for the physical outputs.

---

## 3. Data Flow Analysis

This section describes the path of data during two primary scenarios.

### Scenario 1: Position Command Input

This flow describes how a voltage from the master controller becomes motion.

1.  **Analog Input:** The master's analog voltage arrives at `G_aiReference`, typed as `LREAL` volts (`AT %IW0 : LREAL`).
2.  **Filtering:** `FB_AnalogInputFilter` applies a median filter to reject outliers and a low-pass filter to smooth the signal. The output is `nFilteredAnalog`.
3.  **Scaling:** `FB_AnalogProcessor` takes `nFilteredAnalog` and, based on the `CurrentMode` being `MODE_POSITION`, scales it using the two-stage mapping parameters into a physical position setpoint in millimeters.
4.  **State Machine Execution:** The `ST_POSITION_CTRL` state (and, during fault recovery, `ST_RECOVERY_POSITION`) is active and clamps the scaled setpoint to the active software limits (`stCurrentLimits.PosMin`, `stCurrentLimits.PosMax`), storing the result in a local `rPosTarget`.
5.  **Command Gate:** `fbPosGate` (`FB_PositionCommandGate`) shapes `rPosTarget` into a per-scan command: it limits command velocity to `stCurrentLimits.VelMax` and acceleration to `G_cfgPosGateAccelMax`, seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosGateMaxDeviation` of actual position so it cannot run away from the real axis if motion stalls. This stage runs in both `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`; velocity mode and torque mode are unchanged.
6.  **Route Active FB Command Outputs:** `fbPosGate.CommandedPosition` becomes `G_sysCommandedPosition` and is written into `G_cmdDirectControl.Position`, enabling direct control.
7.  **Drive Command:** The LD POU's `Y_DirectControl` instance reads `G_cmdDirectControl` each scan, commands the Yaskawa drive, and the motor moves.

### Scenario 2: Position Feedback Output

This flow describes how the motor's physical position is reported back to the master.

1.  **Encoder Reading:** The Yaskawa drive continuously tracks the motor's absolute encoder position.
2.  **PLCopen Feedback:** `MC_ReadActualPosition` in the LD POU retrieves the current axis position each scan and publishes it through the globals; `PRG_Main` stores it in `G_sysActualPosition`.
3.  **Offset Application:** `FB_PositionOutput` takes `G_sysActualPosition` and adds `G_posEOTOffset` (calculated during EOT homing). This translates the position from the slave's coordinate system (where 0 is the home limit switch) to the master's coordinate system (where the EOT corresponds to `G_cfgHomeEOTSetPosition`, typically 300 mm).
4.  **Analog Scaling:** `FB_PositionOutput` applies the two-stage position-to-voltage mapping, producing an `LREAL` voltage value.
5.  **Analog Output:** The voltage is written to `G_aoPositionOutput` (`AT %QW0 : LREAL`). MotionWorksIEC converts this to the physical -10V to +10V signal for the master controller to read.

---

## 4. Key Function Block Roles (`PRG_Main`)

The main program orchestrates a number of critical function blocks.

| Instance | FB Type | Role in Architecture |
|---|---|---|
| `fbFilter*` | `FB_Digital/AnalogInputFilter` | **Input Conditioning:** Provide clean, stable signals to the core logic. |
| `fbModeDecoder`| `FB_ModeDecoder` | **Command Interpretation:** Converts the master's 3-bit command into a meaningful `E_OperatingMode` enum. |
| `fbHandshake`| `FB_HandshakeManager`| **Mode Synchronization:** Manages the mode entry handshake protocol with the master (confirmation, timeout, verification). |
| `fbFaultReset`| `FB_FaultResetHandler`| **Fault Reset Validation:** Validates fault reset conditions — FaultReset HIGH, MotionEnable LOW, bits stable, and master fault code acknowledgement matches active fault. |
| `fbAnalogProc`| `FB_AnalogProcessor`| **Command Scaling:** Translates the filtered analog input voltage into a physical setpoint (position, velocity, or torque). |
| `fbPosGate` | `FB_PositionCommandGate` | **Command Shaping:** In `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`, limits the position command's velocity (`stCurrentLimits.VelMax`) and acceleration (`G_cfgPosGateAccelMax`), seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosGateMaxDeviation` of actual position. Velocity and torque modes are unchanged. |
| `fbReadActual*`| `MC_ReadActual*` | **Feedback:** The primary source of feedback from the drive, providing real-time position, velocity, and torque. |
| `fbPower` | `MC_Power` | **Drive Control:** Enables and disables power to the servo drive. |
| `fbStop` | `MC_Stop` | **Safe Stop:** Used to execute a controlled stop when changing modes or entering a fault state. `Y_DirectControl` is kept enabled with a zero command until `MC_Stop` has aborted it, which is what returns the SERVOPACK from velocity/torque control to position control (Yaskawa AN.MPIEC.30 §4.1.3). |
| `fbDirectControl`| `Y_DirectControl`| **Motion Command:** The primary interface for sending real-time position, velocity, or torque commands to the drive during operational states. |
| `fbHome*` | `FB_HomeLimit`, `FB_HomeEOT` | **Homing Logic:** Encapsulates the complex, multi-step homing sequences. |
| `fbSafetyMonitor`| `FB_SafetyMonitor` | **System Watchdog:** The central authority for detecting unsafe conditions and triggering a fault. |
| `fbOutputMux` | `FB_OutputMux` | **Output Routing:** Switches the meaning of the 3-bit digital outputs between "Mode Confirmation" and "Fault Code". |
| `fbPosOutput` | `FB_PositionOutput` | **Feedback Scaling:** Translates the internal axis position into the correct analog voltage for the master, applying the EOT offset and two-stage mapping. |
