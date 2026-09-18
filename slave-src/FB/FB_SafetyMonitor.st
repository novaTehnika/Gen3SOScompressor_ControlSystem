(* ============================================================================
   FB_SafetyMonitor - Centralized Safety Monitoring

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Monitors all safety-related conditions and generates appropriate fault codes.
   This function block provides a single point for all safety monitoring,
   making the system easier to audit and maintain.

   Monitored Conditions:
   - Drive faults (from servo amplifier; encoder alarms surface here via the drive)
   - Overtravel faults (unexpected switch triggers; positive switch never expected)
   - Position limit violations (beyond soft limits)
   - Piston exit guard faults (from FB_PistonExitGuard)

   Fault Priority (highest to lowest):
   1. Drive fault (FAULT_DRIVE)
   2. Limit switch fault (FAULT_LIMIT_SWITCH)
   3. Position limit fault (FAULT_POSITION)
   4. Piston exit fault (FAULT_PISTON_EXIT)

   Integration:
   - Called every scan after state machine, before output update
   - FaultDetected triggers transition to E_SystemState#ST_FAULT in PRG_Main

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable safety monitoring
       OvertravelNegActive : BOOL             (* TRUE = negative overtravel / home switch triggered
       OvertravelPosActive : BOOL             (* TRUE = positive overtravel switch triggered
       ActualPosition      : LREAL            (* Current axis position (mm)
       SoftLimitMin        : LREAL            (* Minimum allowed position (mm)
       SoftLimitMax        : LREAL            (* Maximum allowed position (mm)
       HomingRequired      : BOOL             (* TRUE while coordinate frame is not yet established
       DriveFault          : BOOL             (* Drive has reported a fault
       CurrentState        : E_SystemState    (* Current state machine state
       ExitGuardTriggered  : BOOL             (* Piston exit guard triggered fault
     VAR_OUTPUT:
       FaultDetected       : BOOL             (* A fault condition exists
       FaultCode           : E_FaultCode      (* Fault code to report
       FaultLimitSwitch    : BOOL             (* Limit switch fault detected
       FaultPosition       : BOOL             (* Position limit exceeded
       FaultDrive          : BOOL             (* Drive fault detected
       FaultPistonExit     : BOOL             (* Piston exit guard fault
     VAR:
       bLatchedFault       : BOOL
       eLatchedFaultCode   : E_FaultCode
       bInMotionState      : BOOL
       bInHomingState      : BOOL
       bInFaultState       : BOOL
       bInRecoveryState    : BOOL             (* TRUE in ST_RECOVERY / ST_RECOVERY_POSITION / ST_RECOVERY_VELOCITY
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    FaultDetected := FALSE;
    FaultCode := E_FaultCode#FAULT_NONE;
    FaultLimitSwitch := FALSE;
    FaultPosition := FALSE;
    FaultDrive := FALSE;
    FaultPistonExit := FALSE;
    bLatchedFault := FALSE;
    eLatchedFaultCode := E_FaultCode#FAULT_NONE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Determine operating context
   -------------------------------------------------------------------------- *)

(* Check if in a motion state *)
bInMotionState := (CurrentState = E_SystemState#ST_POSITION_CTRL) OR
                  (CurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
                  (CurrentState = E_SystemState#ST_TORQUE_CTRL) OR
                  (CurrentState = E_SystemState#ST_GO_HOME) OR
                  (CurrentState = E_SystemState#ST_HOLD_POSITION);

(* Check if in a homing state *)
bInHomingState := (CurrentState = E_SystemState#ST_HOME_LIMIT) OR
                  (CurrentState = E_SystemState#ST_HOME_COMPLETE);

(* Check if already in fault state *)
bInFaultState := (CurrentState = E_SystemState#ST_FAULT) OR
                 (CurrentState = E_SystemState#ST_FAULT_IDLE);

(* Check if in a limit-recovery state. While recovering, the operator is
   deliberately jogging off an active limit; the active overtravel switch and
   out-of-soft-limit position are expected and must not fault. Travel direction
   is enforced by command clamping in PRG_Main, not here. *)
bInRecoveryState := (CurrentState = E_SystemState#ST_RECOVERY) OR
                    (CurrentState = E_SystemState#ST_RECOVERY_POSITION) OR
                    (CurrentState = E_SystemState#ST_RECOVERY_VELOCITY);

(* --------------------------------------------------------------------------
   DRIVE FAULT MONITORING
   Most critical - servo amplifier has detected an issue
   -------------------------------------------------------------------------- *)

FaultDrive := DriveFault;

(* --------------------------------------------------------------------------
   OVERTRAVEL FAULT MONITORING
   The positive overtravel switch must never activate during any state.
   The negative overtravel switch is expected only as the homing target
   inside ST_HOME_LIMIT.
   -------------------------------------------------------------------------- *)

IF bInHomingState THEN
    (* During homing, negative overtravel is the expected target.
       Positive overtravel is never expected - always faults. *)
    IF OvertravelPosActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSIF bInRecoveryState THEN
    (* During recovery, the active switch is expected (we are jogging off it).
       Only a simultaneous both-switches reading is impossible mechanically and
       indicates a wiring/sensor fault. *)
    IF OvertravelNegActive AND OvertravelPosActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSIF bInMotionState THEN
    (* During normal motion, hitting either overtravel switch is a fault. *)
    IF OvertravelNegActive OR OvertravelPosActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSE
    (* Not in motion - ignore switch state. *)
    FaultLimitSwitch := FALSE;
END_IF;

(* --------------------------------------------------------------------------
   POSITION LIMIT MONITORING
   Position beyond software limits.

   Suppressed when EITHER:
   - HomingRequired is TRUE (the post-homing coordinate frame is not yet
     established, so soft limits are not meaningful). This covers boot
     with a stale absolute-encoder reading and the activation transition
     states leading into the first homing.
   - CurrentState = ST_HOME_LIMIT (the homing sequence is running, even
     if voluntarily re-commanded after a successful homing). The sequence
     legitimately drives the axis below SoftLimitMin to find the switch,
     and the coordinate frame is being re-established mid-flight.
   - In a recovery state (bInRecoveryState): the axis starts outside the
     soft-limit box and is being jogged back inside; faulting on the very
     position we are recovering from would defeat the recovery.
   -------------------------------------------------------------------------- *)

IF (NOT HomingRequired) AND
   (CurrentState <> E_SystemState#ST_HOME_LIMIT) AND
   (NOT bInRecoveryState) AND
   ((ActualPosition < SoftLimitMin) OR (ActualPosition > SoftLimitMax)) THEN
    FaultPosition := TRUE;
ELSE
    FaultPosition := FALSE;
END_IF;

(* --------------------------------------------------------------------------
   PISTON EXIT GUARD MONITORING
   Triggered by FB_PistonExitGuard
   -------------------------------------------------------------------------- *)

FaultPistonExit := ExitGuardTriggered;

(* --------------------------------------------------------------------------
   FAULT PRIORITY AND OUTPUT
   Determine highest priority RAW fault for this scan.
   Must run BEFORE latch clearing so the clear check sees the current scan's
   value, not a stale value from the previous scan's latch override below.
   -------------------------------------------------------------------------- *)

(* Priority order: Drive > LimitSwitch > Position > PistonExit *)
IF FaultDrive THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_DRIVE;

ELSIF FaultLimitSwitch THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_LIMIT_SWITCH;

ELSIF FaultPosition THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_POSITION;

ELSIF FaultPistonExit THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_PISTON_EXIT;

ELSE
    FaultDetected := FALSE;
    FaultCode := E_FaultCode#FAULT_NONE;
END_IF;

(* --------------------------------------------------------------------------
   FAULT LATCHING
   Latch fault for persistence across scans until the physical condition
   clears AND the system has left fault states. The latch clear gate uses
   the freshly computed FaultDetected above — checking the OUTPUT variable
   would see a stale latch-asserted TRUE and never clear.
   -------------------------------------------------------------------------- *)

(* Clear latch once we're out of fault states with no raw fault present. *)
IF NOT bInFaultState AND NOT FaultDetected THEN
    bLatchedFault := FALSE;
    eLatchedFaultCode := E_FaultCode#FAULT_NONE;
END_IF;

IF FaultDetected AND NOT bLatchedFault AND NOT bInFaultState THEN
    bLatchedFault := TRUE;
    eLatchedFaultCode := FaultCode;
END_IF;

(* Use latched fault if current scan doesn't detect new fault *)
IF bLatchedFault AND NOT FaultDetected THEN
    FaultDetected := TRUE;
    FaultCode := eLatchedFaultCode;
END_IF;
