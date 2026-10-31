# System Architecture

## Gen3 SOS Compressor Servo Controller
### Software Architecture and Data Flow

---

## 1. Introduction

This document provides a high-level overview of the slave controller's software architecture. It describes the main functional blocks, their interactions, and the flow of data through the system during key operations.

---

## 2. Software Block Diagram

The application spans two POUs: `PRG_Main` (Structured Text) contains the state machine and all custom function blocks, and a Ladder Diagram POU owns `G_sysAxis` together with every built-in motion FB (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_SetPosition`, `MC_ReadActual*`, Jog, `Y_DirectControl`). The two POUs exchange data each scan through a pair of structured global families:

- **`G_cmd*`** — written by `PRG_Main`, read by the LD POU. Each struct mirrors the pin names of its target built-in FB (`G_cmdPower`, `G_cmdStop`, `G_cmdReset`, `G_cmdMoveAbsolute`, `G_cmdJog`, `G_cmdDirectControl`, `G_cmdSetPosition`).
- **`G_sta*`** — written by the LD POU, read by `PRG_Main` and by custom FBs that need built-in FB status (`G_staPower`, `G_staStop`, `G_staReset`, `G_staMoveAbsolute`, `G_staJog`, `G_staDirectControl`, `G_staSetPosition`).

`PRG_Main` is structured as a series of processing stages that execute on every 2 ms scan cycle (the cyclic task interval, mirrored by `G_cfgScanTime`).

```
+-----------------------------------------------------------------------------+
|                               PRG_Main Scan Cycle                           |
+-----------------------------------------------------------------------------+
|                                                                             |
|  [Modbus HR] ------------------------------->+-----------------+             |
|   (G_mb*)  |                                 |  FB_ModeDecoder |-----+       |
|            |        +-----------------+      +-----------------+     |       |
|            +------->|  Digital Input  |                            |       |
|  [Raw Inputs] ----->|     Filters     |----------------------------+       |
|   (di*, ai*)        +-----------------+                            |       |
|                                                                    |       |
|                     +-----------------+      +-----------------+     |       |
|                     |  Analog Input   |----->| FB_PressureInput|-----+       |
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
|  |                        +>| (State Machine) |<---+ (Go/Lim)    |  |       |
|  |  +-----------------+   | |                 |    +-----------+  |       |
|  |  | Y_DirectControl |<--+ +-----------------+                   |       |
|  |  +-----------------+                                           |       |
|  |                                                                 |       |
|  +-----------------------------------------------------------------+       |
|                                     |                                      |
|  +----------------------------------+----------------------------------+   |
|  V                                                                    V   |
| +--------------------+                                     +-----------------+  |
| | FB_SafetyMonitor   |                                     |   PRG_Output    |  |
| +--------------------+                                     +-----------------+  |
|           |                                                          |       |
|           |                                                          |       |
|           +-----> [To State Machine for FAULT transition]             |       |
|                                                                      |       |
|  [Modbus IR, Raw Outputs] <------------------------------------------+       |
|   (G_mb*, do*)                                                              |
|                                                                             |
+-----------------------------------------------------------------------------+

```

### Component Descriptions

-   **Input Filters:** A layer of digital (`FB_DigitalInputFilter`) and analog (`FB_AnalogInputFilter`) filters that debounce the MotionEnable and FaultReset registers (`G_mbMotionEnable`, `G_mbFaultReset`) and the overtravel switches, and remove noise from the pressure input, providing stable values for the rest of the program.
-   **Decoders/Processors:** Function blocks that convert raw, filtered inputs into meaningful data (e.g., `FB_ModeDecoder` debounces the mode code register `G_mbModeCode` and converts it into an `E_OperatingMode`; `FB_PressureInput` scales the pressure transducer voltage to MPa).
-   **Core Logic (State Machine):** The `IF/ELSIF` chain in `PRG_Main` that orchestrates all system behavior based on the current state (`G_sysCurrentState`). MotionWorks IEC Express does not support enum-valued `CASE` selectors, hence the `IF/ELSIF` pattern.
-   **Control Blocks (`Y_DirectControl`, `MC_*`):** PLCopen (`MC_`) and Yaskawa-specific (`Y_`) function blocks. These live in the LD POU and are driven from ST through the `G_cmd*` / `G_sta*` globals.
-   **Homing FBs (`FB_HomeLimit`, `FB_GoHome`):** Emit built-in FB commands via `Cmd*` `VAR_OUTPUT` structs and receive built-in FB status via `Sta*` `VAR_INPUT` structs. `PRG_Main` bridges these to the matching globals.
-   **Safety Monitor:** A watchdog (`FB_SafetyMonitor`) that runs every cycle to check for fault conditions.
-   **Output Processors:** `PRG_Output` publishes the confirmed mode code (`G_mbConfCode`, IR3), the fault code (`G_mbFaultCode`, IR4), the status bits and the feedback in the Modbus input registers, and drives the status and brake digital outputs.

---

## 3. Data Flow Analysis

This section describes the path of data during two primary scenarios.

### Scenario 1: Position Command Input

This flow describes how a position reference from the master controller becomes motion.

1.  **Modbus Input:** The master writes the reference in millimeters to the holding register `G_mbReference` (`AT %MD3.483336 : REAL`), together with the mode code, MotionEnable, FaultReset and its heartbeat.
2.  **Mapping:** `PRG_Input` copies it to `G_sysReference` (`LREAL`). `PRG_Main` passes the mode code register `G_mbModeCode` to `FB_ModeDecoder`, which debounces and decodes it.
3.  **State Machine Execution:** The `ST_POSITION_CTRL` state (and, during fault recovery, `ST_RECOVERY_POSITION`) is active and clamps `G_sysReference` to the active software limits (`stCurrentLimits.PosMin`, `stCurrentLimits.PosMax`), storing the result in a local `rPosTarget`.
4.  **Command Gate:** `fbPosGate` (`FB_PositionCommandGate`) shapes `rPosTarget` into a per-scan command: it limits command velocity to `stCurrentLimits.VelMax` and acceleration to `G_cfgPosGateAccelMax`, seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosGateMaxDeviation` of actual position so it cannot run away from the real axis if motion stalls. This stage runs in both `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`; velocity mode and torque mode are unchanged.
5.  **Route Active FB Command Outputs:** `fbPosGate.CommandedPosition` becomes `G_sysCommandedPosition` and is written into `G_cmdDirectControl.Position`, enabling direct control.
6.  **Drive Command:** The LD POU's `Y_DirectControl` instance reads `G_cmdDirectControl` each scan, commands the Yaskawa drive, and the motor moves.

### Scenario 2: Position Feedback Output

This flow describes how the motor's physical position is reported back to the master.

1.  **Encoder Reading:** The Yaskawa drive continuously tracks the motor's absolute encoder position.
2.  **PLCopen Feedback:** `MC_ReadActualPosition` in the LD POU retrieves the current axis position each scan and publishes it through the globals; `PRG_Main` stores it in `G_sysActualPosition`.
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
| `fbPosGate` | `FB_PositionCommandGate` | **Command Shaping:** In `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`, limits the position command's velocity (`stCurrentLimits.VelMax`) and acceleration (`G_cfgPosGateAccelMax`), seeds from `G_sysActualPosition` on state entry, and tethers the command to within `G_cfgPosGateMaxDeviation` of actual position. Velocity and torque modes are unchanged. |
| `fbReadActual*`| `MC_ReadActual*` | **Feedback:** The primary source of feedback from the drive, providing real-time position, velocity, and torque. |
| `fbPower` | `MC_Power` | **Drive Control:** Enables and disables power to the servo drive. |
| `fbStop` | `MC_Stop` | **Safe Stop:** Used to execute a controlled stop when changing modes or entering a fault state. `Y_DirectControl` is kept enabled with a zero command until `MC_Stop` has aborted it, which is what returns the SERVOPACK from velocity/torque control to position control (Yaskawa AN.MPIEC.30 §4.1.3). |
| `fbDirectControl`| `Y_DirectControl`| **Motion Command:** The primary interface for sending real-time position, velocity, or torque commands to the drive during operational states. |
| `fbHome*` | `FB_HomeLimit`, `FB_GoHome` | **Homing Logic:** Encapsulates the multi-step homing sequence and the Go Home move. |
| `fbSafetyMonitor`| `FB_SafetyMonitor` | **System Watchdog:** The central authority for detecting unsafe conditions and triggering a fault. |
| `fbCommWatchdog` | `FB_CommWatchdog` | **Link Supervision:** Raises `FAULT_COMM` through the safety monitor when the master heartbeat register stops changing. |
| `fbIntToFaultCode` | `FB_IntToFaultCode` | **Fault Acknowledgement:** Decodes the full fault code the master mirrors in the mode register for `FB_FaultResetHandler`. |
