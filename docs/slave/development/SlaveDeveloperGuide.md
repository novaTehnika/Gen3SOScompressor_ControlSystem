# Slave-Side (PLC) Developer Guide

## Gen3 SOS Compressor Servo Controller
### MP2600iec Structured Text Development

---

## 1. Introduction

This guide is intended for developers who need to understand, maintain, or extend the Structured Text (ST) source code for the MP2600iec servo controller (the "slave"). It assumes a basic understanding of IEC 61131-3 programming languages and PLC concepts.

For information regarding the master controller interface, see the [Master Protocol Guide](../../master/MasterProtocolGuide.md).

---

## 2. Project Structure

The MotionWorks IEC project is organized into several key folders, following standard PLC programming practices.

| Folder | Name | Purpose |
|--------|------|---------|
| `slave-src/PRG/` | Programs | Contains the main program entry point, `PRG_Main.st` (Structured Text), which houses the core state machine. A separate Ladder Diagram POU owns `G_sysAxis` and every built-in motion FB (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_MoveVelocity`, `MC_SetPosition`, `MC_ReadActual*`, `Y_DirectControl`). |
| `slave-src/FB/` | Function Blocks | Contains reusable modules of encapsulated logic (e.g., `FB_HandshakeManager`, `FB_HomeEOT`) as well as the enum-conversion helpers (`FB_BitsToMode`, `FB_IsOperationalMode`, etc.). This is where most of the application logic resides. |
| `slave-src/GVL/` | Global Variable Lists | Contains a single comment-only reference file (`GlobalVariables_Reference.st`). Variables are defined in the MWiec GUI; this file documents their names, types, and defaults. It also documents the `G_cmd*` / `G_sta*` structured globals used to connect ST and LD POUs. |
| `slave-src/DUT/` | Data Unit Types | Contains all custom data structures (`STRUCT`) and enumerations (`ENUM`) in a single `DataTypes.st` file, including the `ST_Cmd*` / `ST_Sta*` structs that shape the ST↔LD command/status interface. |

### ST ↔ LD POU Interface

Because `G_sysAxis` and the built-in motion FBs live in the LD POU, `PRG_Main` never calls `MC_Power`, `MC_Stop`, `Y_DirectControl`, etc. directly. Instead it writes command fields into the `G_cmd*` structured globals and reads state from the matching `G_sta*` globals each scan:

| ST writes (command) | LD owns (FB instance) | LD writes back (status) |
|---------------------|------------------------|--------------------------|
| `G_cmdPower` | `MC_Power` | `G_staPower` |
| `G_cmdStop` | `MC_Stop` | `G_staStop` |
| `G_cmdReset` | `MC_Reset` | `G_staReset` |
| `G_cmdMoveAbsolute` | `MC_MoveAbsolute` | `G_staMoveAbsolute` |
| `G_cmdMoveVelocity` | `MC_MoveVelocity` | `G_staMoveVelocity` |
| `G_cmdDirectControl` | `Y_DirectControl` | `G_staDirectControl` |
| `G_cmdSetPosition` | `MC_SetPosition` | `G_staSetPosition` |

Custom FBs that need to command or observe a built-in motion FB (`FB_HomeLimit`, `FB_HomeEOT`, `FB_GoHome`) follow the same pattern at the FB level: they emit `Cmd*` `VAR_OUTPUT` structs and receive `Sta*` `VAR_INPUT` structs. `PRG_Main` wires those to the matching globals — see §4 below.

---

## 3. Coding Conventions

The codebase follows a set of naming conventions to improve readability and maintainability.

### Variable Prefixes

| Prefix | Type | Example | Description |
|--------|------|---------|-------------|
| `b` | `BOOL` | `bEnterState` | A boolean flag. |
| `n` | `INT` / `UINT` | `nValue` | An integer value. |
| `r` | `REAL` / `LREAL` | `rAnalogFilterAlpha` | A floating-point (real) number. |
| `t` | `TIME` / `TON` | `tStateTimer` | A timer instance or time value. |
| `e` | `ENUM` | `eTargetState` | An enumeration value. |
| `st` | `STRUCT` | `stCurrentLimits` | A structure instance. |
| `fb` | Function Block | `fbHandshake` | An instance of a Function Block. |
| `rTrig` | `R_TRIG` | `rTrigMotionEnable` | A rising-edge trigger. |
| `fTrig` | `F_TRIG` | `fTrigMotionEnable` | A falling-edge trigger. |

### Global Variable Prefixes

Global variables defined in a `GVL` use a prefix to indicate their scope.

| Prefix | Scope | Example | Description |
|--------|-------|---------|-------------|
| `cfg` | Configuration | `G_cfgDebounceTimeMode` | A static configuration parameter used for tuning. |
| `sys` | System | `G_sysCurrentState` | A dynamic system status variable. |
| `pos` | Position | `G_posEOTOffset` | A position-related value, often calculated during homing. |
| `di` / `do` | I/O | `G_diModeBit0` | A digital input (`di`) or output (`do`). |
| `ai` / `ao` | I/O | `G_aiReference` | An analog input (`ai`) or output (`ao`). |

> All global variables are documented in `slave-src/GVL/GlobalVariables_Reference.st` and defined in the MWiec GUI.

### Constant Prefixes

| Prefix | Type | Example | Description |
|--------|------|---------|-------------|
| `ST_` | `E_SystemState` ENUM | `ST_IDLE` | A state in the main state machine. |
| `MODE_` | `E_OperatingMode` ENUM | `MODE_POSITION` | An operating mode commanded by the master. |
| `FAULT_` | `E_FaultCode` ENUM | `FAULT_DRIVE` | A system fault code. |

### MotionWorks IEC Express Compatibility

This project uses **MotionWorks IEC Express**, which has more restrictions than MotionWorks IEC Pro.

**CASE Statement Restriction:**

MotionWorks IEC Express does not support enum types as CASE selectors. Always use IF-ELSIF-ELSE chains instead:

```structuredtext
(* ✓ CORRECT - Use IF-ELSIF with enums *)
IF G_sysCurrentState = ST_IDLE THEN
    (* logic *)
ELSIF G_sysCurrentState = ST_RUNNING THEN
    (* logic *)
END_IF;

(* ✗ WRONG - CASE with enum will not compile in MWiec Express *)
CASE G_sysCurrentState OF
    ST_IDLE:
        (* logic *)
    ST_RUNNING:
        (* logic *)
END_CASE;
```

**Note:** All state machines in this project (both `PRG_Main` with enums and internal FB state machines with INT constants) use the IF-ELSIF pattern for MWiec Express compatibility.

**Function Output Limitation:**

Functions can only have one output. For multiple outputs, use a function block instead.

**User-Enum Parameters on Functions:**

MotionWorks IEC Express fails native code generation with `E2001: circular assembly references` when a `FUNCTION` POU has a user-defined enum in its signature — as a `VAR_INPUT` type or as the return type. Always implement such conversions as `FUNCTION_BLOCK`s. Precedent: the six enum-helper FBs (`FB_IsOperationalMode`, `FB_IsHomingMode`, `FB_ModeToInt`, `FB_FaultToInt`, `FB_BitsToMode`, `FB_BitsToFaultCode`) exist as FBs specifically to work around this defect.

---

## 4. State Machine Deep Dive

The core logic resides in a state machine within `PRG_Main.st`. Understanding this flow is critical.

### Key State Groups

- **Initialization (`ST_INIT`)**: The default enum initializer. On the first scan the state machine forces both `G_flagAbsHomeRequired` and `G_flagEOTHomeRequired` TRUE and transitions directly to `ST_IDLE`.
- **Idle (`ST_IDLE`)**: The default safe state. The drive is disabled, and the brake is engaged. It waits here for a valid handshake from the master.
- **Activation (`ST_DRIVE_ENABLE`, `ST_BRAKE_RELEASE`)**: A transitional sequence to power on the drive and release the brake before entering an operational mode.
- **Operational States (`ST_POSITION_CTRL`, `ST_VELOCITY_CTRL`, etc.)**: The active motion or control states. Each state writes setpoints into `G_cmdDirectControl` (read by the LD POU's `Y_DirectControl` instance each scan) and checks for a `G_diMotionEnable` drop to exit. The position states (`ST_POSITION_CTRL`, `ST_RECOVERY_POSITION`) feed their target through `fbPosGate` (`FB_PositionCommandGate`), and on a `G_diMotionEnable` drop they first ramp the command to rest under `G_cfgPosGateAccelMax` (`bPosStopping`) before leaving the state.
- **Homing States (`ST_HOME_LIMIT`, `ST_HOME_EOT`)**: These states execute the encapsulated logic inside `FB_HomeLimit` and `FB_HomeEOT`. Each FB emits `Cmd*` `VAR_OUTPUT` structs (`CmdMoveVelocity`, `CmdStop`, `CmdSetPosition`, `CmdDirectControl`, `CmdMoveAbsolute`) which `PRG_Main` copies into the matching `G_cmd*` globals so the LD POU's motion FBs act on them; status comes back through `G_sta*` globals wired into the FBs' `Sta*` `VAR_INPUT` structs. Deprecated homing sub-state enum members (`ST_HOME_LIM_*`, `ST_HOME_EOT_*`) still exist in `E_SystemState` but are unused by the live state machine.
- **Transition (`ST_HOLD_POSITION`)**: A critical intermediate state entered when `G_diMotionEnable` drops. It brings the axis to a controlled halt (`MC_Stop`) and waits for a new mode handshake from the master. While `MC_Stop` executes, `Y_DirectControl` is held enabled in its last `ControlMode` with a zero command (`bKeepDirectControl` / `eKeepDirectMode`) until `MC_Stop` reports Busy and `Y_DirectControl.Active` drops; only an *aborted* `Y_DirectControl` returns the SERVOPACK to position control, and mode transitions wait for that hand-off. The same hand-off runs in `ST_FAULT` and `ST_RECOVERY`.
- **Deactivation (`ST_BRAKE_ENGAGE`, `ST_DRIVE_DISABLE`)**: A transitional sequence to engage the brake and disable the drive before returning to `ST_IDLE`.
- **Fault Handling (`ST_FAULT`, `ST_FAULT_IDLE`)**:
    - `ST_FAULT`: The active fault state. It performs a controlled `MC_Stop` and waits for a fault reset handshake. It includes logic for a "fast recovery" (to `ST_BRAKE_HOLD`) or a "slow recovery" timeout to `ST_FAULT_IDLE`.
    - `ST_FAULT_IDLE`: The safe, powered-down fault state. The brake is engaged and the drive is off. It waits for a fault reset handshake to return to `ST_IDLE`.

---

## 5. How-To Guides

### How to Add a New Operating Mode

Let's say you want to add `MODE_NEW_FEATURE` with a value of `101` (currently unused).

1.  **Define Enum:** In `slave-src/DUT/DataTypes.st`, add the new mode member in the correct ordinal position (the enum uses dense sequential numbering):
    ```structuredtext
    MODE_NEW_FEATURE, (* Binary 101 - new feature mode *)
    ```

2.  **Update Conversions:** Update the mode helpers in `slave-src/FB/` to handle the new mode: `FB_ModeToInt.st` (add an `ELSIF` branch for the new enum → int value) and `FB_BitsToMode.st` (add an `ELSIF` branch for the new int → enum value). `FB_ModeToBits` delegates to `FB_ModeToInt` and needs no direct change.

3.  **Update Master Docs:** In `docs/master/MasterProtocolGuide.md` and `docs/master/IOReference.md`, add the new mode to the mode tables so the master developer knows about it.

4.  **Create a State:** In `slave-src/DUT/DataTypes.st`, add a corresponding state in `E_SystemState`, e.g., `ST_NEW_FEATURE`.

5.  **Add State Logic:** In `PRG_Main.st`, add a new `ELSIF` branch in the main state machine for `ST_NEW_FEATURE`. Implement the logic for this state.
    ```structuredtext
    ELSIF G_sysCurrentState = ST_NEW_FEATURE THEN
        IF bEnterState THEN
            G_sysConfirmedMode := MODE_NEW_FEATURE;
            (* ... initialization logic ... *)
        END_IF;

        (* ... continuous logic for the new mode ... *)

        IF fTrigMotionEnable.Q THEN
            G_sysCurrentState := ST_HOLD_POSITION;
        END_IF;
    ```

6.  **Handle Transitions:** In `PRG_Main.st`, update the state transitions to allow entry into your new state.
    *   In `ST_BRAKE_RELEASE`, add: `ELSIF eTargetMode = MODE_NEW_FEATURE THEN G_sysCurrentState := ST_NEW_FEATURE;`
    *   In `ST_HOLD_POSITION`, add: `ELSIF eTargetMode = MODE_NEW_FEATURE THEN G_sysCurrentState := ST_NEW_FEATURE;`

### How to Add a New Fault Condition

Let's say you want to add `FAULT_NEW_CONDITION` with a value of `5` (currently `FAULT_PISTON_EXIT`).

1.  **Define Enum:** In `slave-src/DUT/DataTypes.st`, add the new fault to `E_FaultCode`. Note that ordinal position matters for bit encoding.

2.  **Update Conversions:** Update the fault helpers in `slave-src/FB/` to handle the new fault: `FB_FaultToInt.st` (add an `ELSIF` branch for the new enum → int value) and `FB_BitsToFaultCode.st` (add an `ELSIF` branch for the new int → enum value). `FB_FaultToBits` delegates to `FB_FaultToInt` and needs no direct change.

3.  **Update Master Docs:** Add the new fault to the tables in `docs/master/FaultCodeReference.md` and other relevant guides.

4.  **Add Detection Logic:** In `slave-src/FB/FB_SafetyMonitor.st`, add the logic that detects the new fault condition.
    ```structuredtext
    (* --------------------------------------------------------------------------
       NEW FAULT MONITORING
       -------------------------------------------------------------------------- *)
    IF (some_new_dangerous_condition) THEN
        FaultNewCondition := TRUE;
    ELSE
        FaultNewCondition := FALSE;
    END_IF;
    ```

5.  **Prioritize the Fault:** In the "FAULT PRIORITY AND OUTPUT" section of `FB_SafetyMonitor.st`, add your new fault into the `IF/ELSIF` chain according to its desired priority.
    ```structuredtext
    // Example: Giving it a higher priority than FAULT_POSITION
    ...
    ELSIF FaultLimitSwitch THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_LIMIT_SWITCH;

    ELSIF FaultNewCondition THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_NEW_CONDITION;

    ELSIF FaultPosition THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_POSITION;
    ...
    ```
    The main program (`PRG_Main.st`) will automatically handle the transition to `ST_FAULT` when `fbSafetyMonitor.FaultDetected` becomes true.

---

## 6. Build and Deployment

This section provides a high-level overview of deploying code to the controller.

1.  **Open Project:** Launch Yaskawa MotionWorksIEC and open the project file.
2.  **Connect to Controller:** Ensure you have a network connection to the MP2600iec controller. In the software, connect to the target device.
3.  **Compile Code:**
    *   Right-click on the "Application" in the project tree and select "Clean".
    *   Right-click again and select "Make". This compiles the ST code into a binary format for the controller.
    *   Check the "Messages" window for any compilation errors and resolve them.
4.  **Download to Controller:**
    *   Once compilation is successful, go to "Online" -> "Login". This will connect you to the controller for programming.
    *   The software will prompt you if the code on the PC is different from the code on the controller. Select "Yes" to download the new program.
5.  **Run Program:**
    *   After the download is complete, go to "Online" -> "Run". This will start the execution of the `PRG_Main` program on the controller.
    *   You can now enter "Debug" mode to monitor variable values in real-time.
6.  **Logout:** When finished, go to "Online" -> "Logout" to disconnect from the controller. The program will continue to run.
