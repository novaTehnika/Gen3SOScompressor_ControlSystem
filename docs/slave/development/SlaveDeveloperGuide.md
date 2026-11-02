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
| `slave-src/PRG/` | Programs | Contains the main program entry point, `PRG_Main.st` (Structured Text), which houses the core state machine. A separate Ladder Diagram POU owns `G_sysAxis` and every built-in motion FB (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_SetPosition`, `MC_ReadActual*`, Jog, `Y_DirectControl`). |
| `slave-src/FB/` | Function Blocks | Contains reusable modules of encapsulated logic (e.g., `FB_HandshakeManager`, `FB_HomeLimit`) as well as the enum-conversion helpers (`FB_IntToMode`, `FB_IsOperationalMode`, etc.). This is where most of the application logic resides. |
| `slave-src/GVL/` | Global Variable Lists | Contains a single comment-only reference file (`GlobalVariables_Reference.st`). Variables are defined in the MWiec GUI; this file documents their names, types, and defaults. It also documents the `G_cmd*` / `G_sta*` structured globals used to connect ST and LD POUs. |
| `slave-src/DUT/` | Data Unit Types | Contains all custom data structures (`STRUCT`) and enumerations (`ENUM`) in a single `DataTypes.st` file, including the `ST_Cmd*` / `ST_Sta*` structs that shape the ST↔LD command/status interface. |

### ST ↔ LD POU Interface

Because `G_sysAxis` and the built-in motion FBs live in the LD POU, `PRG_Main` never calls `MC_Power`, `MC_Stop`, `Y_DirectControl`, etc. directly. The LD POU (`PRG_LD`) has no text form; `slave-src/PRG/PRG_LD.st` specifies it in the form of the ST headers (the instances to declare and the wiring of each pin, network by network) and the screenshots in `docs/slave/ld/` record the project's wiring, and a change to a `G_cmd*` or `G_sta*` field is complete only when the specification, the screenshots and the LD agree. Instead it writes command fields into the `G_cmd*` structured globals and reads state from the matching `G_sta*` globals each scan:

| ST writes (command) | LD owns (FB instance) | LD writes back (status) |
|---------------------|------------------------|--------------------------|
| `G_cmdPower` | `MC_Power` | `G_staPower` |
| `G_cmdStop` | `MC_Stop` | `G_staStop` |
| `G_cmdReset` | `MC_Reset` | `G_staReset` |
| `G_cmdMoveAbsolute` | `MC_MoveAbsolute` | `G_staMoveAbsolute` |
| `G_cmdJog` | Jog | `G_staJog` |
| `G_cmdDirectControl` | `Y_DirectControl` (no deceleration pin, so no `Deceleration` field) | `G_staDirectControl` |
| `G_cmdSetPosition` | `MC_SetPosition` | `G_staSetPosition` |

Custom FBs that need to command or observe a built-in motion FB (`FB_HomeLimit`, `FB_GoHome`) follow the same pattern at the FB level: they emit `Cmd*` `VAR_OUTPUT` structs and receive `Sta*` `VAR_INPUT` structs. `PRG_Main` wires those to the matching globals — see §4 below.

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
| `fTrig` | `F_TRIG` | `fTrigSignal` | A falling-edge trigger. |

### Global Variable Prefixes

Global variables defined in a `GVL` use a prefix to indicate their scope.

| Prefix | Scope | Example | Description |
|--------|-------|---------|-------------|
| `cfg` | Configuration | `G_cfgDebounceTimeMode` | A static configuration parameter used for tuning. |
| `sys` | System | `G_sysCurrentState` | A dynamic system status variable. |
| `di` / `do` | I/O | `G_diOvertravelNeg` | A digital input (`di`) or output (`do`): the overtravel switches and the brake. |
| `ai` / `ao` | I/O | `G_aiPressure` | An analog input (`ai`) or output (`ao`). The pressure transducer is the only analog input; no `G_ao*` globals exist. |
| `mb` | Modbus | `G_mbModeCode` | A Modbus holding register (master → slave) or input register (slave → master), see `docs/master/ModbusRegisterMap.md`. |

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

**No Initial Value on Enum Variables:**

MotionWorks IEC Express does not accept an initial value on a variable of a user-defined enum type, in the global variable worksheet or in a POU's variables. Declare it without one; it starts at the enum's first member (the implicit 0), so order an enum's members with the starting state first (`ST_INIT`, `FAULT_NONE`, `PROBE_OFF`, `XFER_IDLE`). The translator rejects an enum declared with an initial value.

```structuredtext
eState : E_HomeLimitState;                          (* ✓ starts at the first member *)
eState : E_HomeLimitState := E_HomeLimitState#IDLE; (* ✗ will not compile *)
```

**Array Index Is a Variable or a Literal:**

The compiler takes only a variable or an integer literal as an array index (`Illegal variable location or multi-element variable` otherwise). Compute an index expression into an INT variable first:

```structuredtext
nIx := nBase + 1;            (* ✓ *)
aRing[nIx] := wValue;
aRing[nBase + 1] := wValue;  (* ✗ will not compile *)
```

**FOR Counters Are INT:**

The compiler types bare literals and a FOR loop's implicit step as INT, so the counter and its bounds must be INT; a DINT counter fails with type mismatches on the assignment, SUB and ADD.

**No VAR_IN_OUT:**

`VAR_IN_OUT` did not work on this controller. A block hands an array back as a `VAR_OUTPUT`, which the caller copies.

**Function Output Limitation:**

Functions can only have one output. For multiple outputs, use a function block instead.

**User-Enum Parameters on Functions:**

MotionWorks IEC Express fails native code generation with `E2001: circular assembly references` when a `FUNCTION` POU has a user-defined enum in its signature — as a `VAR_INPUT` type or as the return type. Always implement such conversions as `FUNCTION_BLOCK`s. Precedent: the six enum-helper FBs (`FB_IsOperationalMode`, `FB_IsHomingMode`, `FB_ModeToInt`, `FB_FaultToInt`, `FB_IntToMode`, `FB_IntToFaultCode`) exist as FBs specifically to work around this defect.

### Changing a block's gains between scans

A POU never writes another block's internal `VAR` (IEC 61131-3 forbids it), so a state that must change when its inputs change is rescaled by the block itself: `FB_PressureController` takes `IntegScale` (old Ki / new Ki on the scan Ki changes, otherwise 1) and `FB_PressureSchedule` computes it. Every input of a block is passed on its one call in `PRG_Main`, because an omitted input keeps its previous value.

---

## 4. State Machine Deep Dive

The core logic resides in a state machine within `PRG_Main.st`. Understanding this flow is critical.

### Key State Groups

- **Initialization (`ST_INIT`)**: The default enum initializer. On the first scan the state machine transitions directly to `ST_IDLE`; no homing is required before operational modes, since the absolute encoder and the controller's stored offset retain the coordinate frame.
- **Idle (`ST_IDLE`)**: The default safe state. The drive is disabled, and the brake is engaged. It waits here for a valid handshake from the master.
- **Activation (`ST_DRIVE_ENABLE`, `ST_BRAKE_RELEASE`)**: A transitional sequence to power on the drive and release the brake before entering an operational mode.
- **Operational States (`ST_POSITION_CTRL`, `ST_VELOCITY_CTRL`, etc.)**: The active motion or control states. Each state writes setpoints into `G_cmdDirectControl` (read by the LD POU's `Y_DirectControl` instance each scan) and checks for a MotionEnable drop (`G_mbMotionEnable`, debounced) to exit. The position states (`ST_POSITION_CTRL`, `ST_RECOVERY_POSITION`) feed their target through `fbPosShaper` (`FB_PositionPathShaper`), and on a MotionEnable drop they first ramp the command to rest under `G_cfgPosShaperAccelMax` (`bPosStopping`) before leaving the state.
- **Homing State (`ST_HOME_LIMIT`)**: This state executes the encapsulated logic inside `FB_HomeLimit`. The FB emits `Cmd*` `VAR_OUTPUT` structs (`CmdJog`, `CmdMoveAbsolute`, `CmdSetPosition`) which `PRG_Main` copies into the matching `G_cmd*` globals so the LD POU's motion FBs act on them; status comes back through `G_sta*` globals wired into the FB's `Sta*` `VAR_INPUT` structs.
- **State properties**: `G_sysCurrentState` is classified in one place, the State Properties section of `PRG_Main` after the state machine. It computes the BOOLs the rest of the scan reads (`bPowerOn`, `bHandshakeState`, `bCommandedMotion`, `bMotionWatched`, `bInHoming`, `bInRecovery`, `bInFault`, `bPositionMode`, `bVelocityMode`, and the brake and drive-wait flags), so a state list is written once. The flags hold the state the scan's state machine ended in; the safety block at the end of the file can still move the state to `ST_FAULT`, and the outputs that must follow that move (`G_sysFaultActive`) read `G_sysCurrentState` again. A new state is added to each list it belongs to there. `FB_SafetyMonitor` takes the flags it needs as inputs (`InCommandedMotion`, `InMotionWatched`, `InHoming`, `InRecovery`, `InFault`) and compares `CurrentState` itself only for `ST_HOME_LIMIT` and `ST_PRESSURE_CTRL`.
- **Transition (`ST_HOLD_POSITION`)**: A critical intermediate state entered when MotionEnable (`G_mbMotionEnable`) drops. It brings the axis to a controlled halt (`MC_Stop`) and waits for a new mode handshake from the master. While `MC_Stop` executes, `Y_DirectControl` is held enabled in its last `ControlMode` with a zero command (`bKeepDirectControl` / `eKeepDirectMode`) until `MC_Stop` reports Busy and `Y_DirectControl.Active` drops; only an *aborted* `Y_DirectControl` returns the SERVOPACK to position control, and mode transitions wait for that hand-off. The same hand-off runs in `ST_FAULT` and `ST_RECOVERY`.
- **Deactivation (`ST_BRAKE_ENGAGE`, `ST_DRIVE_DISABLE`)**: A transitional sequence to engage the brake and disable the drive before returning to `ST_IDLE`.
- **Fault Handling (`ST_FAULT`, `ST_FAULT_IDLE`)**:
    - `ST_FAULT`: The active fault state. It performs a controlled `MC_Stop` and waits for a fault reset handshake. It includes logic for a "fast recovery" (to `ST_BRAKE_HOLD`) or a "slow recovery" timeout to `ST_FAULT_IDLE`.
    - `ST_FAULT_IDLE`: The safe, powered-down fault state. The brake is engaged and the drive is off. It waits for a fault reset handshake to return to `ST_IDLE`.

---

### Input, Handshake and Safety Blocks

These blocks sit between the Modbus registers and the state machine. Each is called once per scan at the end of `PRG_Main`, and the states read their outputs on the next scan.

- **`FB_AnalogInputFilter`** filters the transducer voltage in two stages: a median of 3 or 5 samples (`G_cfgAnalogMedianSize`), then a first-order low pass whose coefficient the caller computes (`alpha = dt / (tau + dt)` from `G_cfgAnalogFilterTimeConst`). The first enabled scan fills the median buffer and the filter state with the input. While disabled it passes the raw input through and keeps its state, so a re-enable resumes from the stored samples rather than from a fresh start.
- **`FB_DigitalInputFilter`** debounces a BOOL: the first scan accepts the input as it is, and afterwards the output follows the input only once it has held the new value for `DebounceTime`. `PRG_Main` filters MotionEnable, FaultReset and the two overtravel inputs with it; the edge outputs are not read.
- **`FB_ModeDecoder`** debounces the mode register (`G_cfgDebounceTimeMode`) the same way and decodes it with `FB_IntToMode`. A code above 7, such as the fault code the master mirrors during a reset, decodes to `MODE_IDLE` with value 0. `ModeStable` is FALSE while a new code is being debounced.
- **`FB_HandshakeManager`** runs the mode handshake only (request, confirmation, MotionEnable, verification, timeout). Its `ConfirmedMode` is an `E_OperatingMode`; `PRG_Main` converts it to `G_sysConfirmedModeCode`, which `PRG_Output` publishes as `G_mbConfCode`.
- **`FB_FaultResetHandler`** is a combinational check with no state or timer. A reset is valid when FaultReset is high, MotionEnable is low, the system is in a fault state, the decoded mode code is stable and the fault code the master mirrored in `G_mbModeCode` equals the active fault. `PRG_Main` decodes the mirrored code with `FB_IntToFaultCode` and passes it in as `MasterFaultCode`, so it is compared as an `E_FaultCode` and not as a mode.
- **`FB_PistonExitGuard`** only detects; it never modifies a command. Positive is into the cylinder, so the pressure pushes the piston out and the motor holds it with positive torque. Within `G_cfgPosExitGuardMargin` of `G_cfgPosCylinderEntry` (the boundary is their sum), positive torque above `G_cfgTorqueStallThreshold` together with a velocity below minus `G_cfgInMotionVelThreshold` means the pressure is winning, and `FaultTriggered` goes to `FB_SafetyMonitor`. It does not depend on the mode.
- **`FB_SafetyMonitor`** holds every fault condition in one block and latches the highest-priority one (the order is in its header). It has no encoder input and no warning outputs: the Sigma-7 detects encoder alarms itself and they arrive as `DriveFault`, so they raise `FAULT_DRIVE`; there is no encoder-manager block.

---

### Step Probe and the Per-Scan Bound

MotionWorks IEC 3 Express has a single task, so every POU runs every scan and
anything that stretches a scan stretches the cycle. `FB_Probe` therefore
bounds its own cost:

- the record write is 8 word stores and some scaling;
- a transfer copies at most `G_cfgProbeFillPerScan` records (8 words each,
  default 6, limited to 1-13) from the ring into the staging array
  `G_sysProbeStage` per scan, and a request is served over several scans
  (`XFER_FILLING`) instead of in one;
- the mapped window is written only in the scan a fill completes
  (`G_sysProbePublish`), by two 104-word copies: `PRG_Main` copies the
  block's `Stage` to `G_sysProbeStage`, and `PRG_Output` copies that into
  the mapped window with StartTail made different from Start before the
  copy and set equal to it last;
- everything else is a few comparisons and assignments.

The worst scan therefore adds 8 + 8 x `G_cfgProbeFillPerScan` word moves from the
block plus, in the completing scan, the two 104-word copies. Do not replace the
fill loop by a copy of the whole window in one scan, and keep any new probe
channel inside the record, whose layout is defined only in `FB_Probe.st`
(the master decoder is `benchProbe.m`).

The block is written so that `tools/st_translate.py` translates it for the py and c targets
and the simulator runs the source: it reads no `G_*` variable (the caller
passes the channels and the probe's holding registers as inputs). It fills
the window in its own output array `Stage`, which `PRG_Main` copies to
`G_sysProbeStage` only in the scan a fill completes. `VAR_IN_OUT` is not used
in the slave: it did not work out on this controller. It uses `MOD`, `SHR`, the `*_TO_WORD` /
`DWORD_TO_WORD` / `DINT_TO_DWORD` conversions (which wrap modulo 2^16 and 2^32,
as on the controller) and IF/ELSIF chains, no CASE. MotionWorks setup for it:
the `FB_Probe` block, the `E_ProbeState` / `E_ProbeXferState` enumerations and
the `ARRAY104WORD` / `ARRAY32768WORD` types from `DataTypes.st`; the `G_cfgProbe*`,
`G_mbProbe*` and `G_sysProbe*` globals of `GlobalVariables_Reference.st` with the
Modbus holding group extended to HR114 and the input group to IR524; and an
`fbProbe : FB_Probe` instance in `PRG_Main`.

---

## 5. How-To Guides

### How to Add a New Operating Mode

Let's say you want to add `MODE_NEW_FEATURE` with a value of `101` (currently unused).

1.  **Define Enum:** In `slave-src/DUT/DataTypes.st`, add the new mode member in the correct ordinal position (the enum uses dense sequential numbering):
    ```structuredtext
    MODE_NEW_FEATURE, (* Binary 101 - new feature mode *)
    ```

2.  **Update Conversions:** Update the mode helpers in `slave-src/FB/` to handle the new mode: `FB_ModeToInt.st` (add an `ELSIF` branch for the new enum → int value) and `FB_IntToMode.st` (add an `ELSIF` branch for the new int → enum value).

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

        IF NOT bFilteredMotionEnable THEN   (* level, not edge: see below *)
            G_sysCurrentState := ST_HOLD_POSITION;
        END_IF;
    ```

    Leave the state on MotionEnable LOW rather than on its falling edge. The start-up states (drive reset, enable, brake release) do not watch MotionEnable, so a drop there would otherwise be missed and the state never left.

6.  **Handle Transitions:** In `PRG_Main.st`, update the state transitions to allow entry into your new state.
    *   In `ST_BRAKE_RELEASE`, add: `ELSIF eTargetMode = MODE_NEW_FEATURE THEN G_sysCurrentState := ST_NEW_FEATURE;`
    *   In `ST_HOLD_POSITION`, add: `ELSIF eTargetMode = MODE_NEW_FEATURE THEN G_sysCurrentState := ST_NEW_FEATURE;`

### How to Add a New Fault Condition

Let's say you want to add `FAULT_NEW_CONDITION` with a value of `5` (currently `FAULT_PISTON_EXIT`).

1.  **Define Enum:** In `slave-src/DUT/DataTypes.st`, add the new fault to `E_FaultCode`. Note that the ordinal position is the fault code published in `G_mbFaultCode` (IR4).

2.  **Update Conversions:** Update the fault helpers in `slave-src/FB/` to handle the new fault: `FB_FaultToInt.st` (add an `ELSIF` branch for the new enum → int value) and `FB_IntToFaultCode.st` (add an `ELSIF` branch for the new int → enum value).

3.  **Update Master Docs:** Add the new fault to the tables in `docs/master/FaultCodeReference.md` and other relevant guides.

4.  **Add Detection Logic:** In `slave-src/FB/FB_SafetyMonitor.st`, declare a local flag (`bFaultNewCondition : BOOL`, in the `VAR` section of the header comment) and add the logic that detects the new fault condition.
    ```structuredtext
    (* --------------------------------------------------------------------------
       NEW FAULT MONITORING
       -------------------------------------------------------------------------- *)
    IF (some_new_dangerous_condition) THEN
        bFaultNewCondition := TRUE;
    ELSE
        bFaultNewCondition := FALSE;
    END_IF;
    ```

5.  **Prioritize the Fault:** In the "FAULT PRIORITY AND OUTPUT" section of `FB_SafetyMonitor.st`, add your new fault into the `IF/ELSIF` chain according to its desired priority.
    ```structuredtext
    // Example: Giving it a higher priority than FAULT_POSITION
    ...
    ELSIF bFaultLimitSwitch THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_LIMIT_SWITCH;

    ELSIF bFaultNewCondition THEN
        FaultDetected := TRUE;
        FaultCode := FAULT_NEW_CONDITION;

    ELSIF bFaultPosition THEN
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
