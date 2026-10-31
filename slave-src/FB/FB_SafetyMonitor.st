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
   - Master link loss (heartbeat stale, from FB_CommWatchdog) while motion is commanded
   - Overtravel faults (unexpected switch triggers; positive switch never expected)
   - Position limit violations (beyond soft limits)
   - Pressure sensor faults (loop current out of range, from
     FB_PressureInput) while the pressure loop is closed on the sensor
   - Piston exit guard faults (from FB_PistonExitGuard)

   The caller classifies the state once per scan (PRG_Main, State Properties)
   and passes the results in as the In* inputs; the block compares
   CurrentState itself only for ST_HOME_LIMIT and ST_PRESSURE_CTRL.

   Fault Priority (highest to lowest):
   1. Drive fault (FAULT_DRIVE)
   2. Link loss (FAULT_COMM)
   3. Limit switch fault (FAULT_LIMIT_SWITCH)
   4. Position limit fault (FAULT_POSITION)
   5. Pressure sensor fault (FAULT_PRESSURE_SENSOR)
   6. Piston exit fault (FAULT_PISTON_EXIT)

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
       DriveFault          : BOOL             (* Drive has reported a fault
       CurrentState        : E_SystemState    (* Current state machine state
       InCommandedMotion   : BOOL             (* State commands motion from the master's reference (link loss faults only here)
       InMotionWatched     : BOOL             (* State in which either overtravel switch is a fault
       InHoming            : BOOL             (* ST_HOME_LIMIT / ST_HOME_COMPLETE
       InRecovery          : BOOL             (* ST_RECOVERY / ST_RECOVERY_POSITION / ST_RECOVERY_VELOCITY
       InFault             : BOOL             (* ST_FAULT / ST_FAULT_IDLE
       ExitGuardTriggered  : BOOL             (* Piston exit guard triggered fault
       CommLost            : BOOL             (* Master heartbeat stale
       PressureSensorFault : BOOL             (* Transducer loop current out of range (from FB_PressureInput)
     VAR_OUTPUT:
       FaultDetected       : BOOL             (* A fault condition exists
       FaultCode           : E_FaultCode      (* Fault code to report
       FaultLimitSwitch    : BOOL             (* Limit switch fault detected
       FaultPosition       : BOOL             (* Position limit exceeded
       FaultDrive          : BOOL             (* Drive fault detected
       FaultComm           : BOOL             (* Link loss detected
       FaultPistonExit     : BOOL             (* Piston exit guard fault
       FaultPressureSensor : BOOL             (* Pressure sensor fault while in pressure control
     VAR:
       bLatchedFault       : BOOL
       eLatchedFaultCode   : E_FaultCode
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
    FaultComm := FALSE;
    FaultPistonExit := FALSE;
    FaultPressureSensor := FALSE;
    bLatchedFault := FALSE;
    eLatchedFaultCode := E_FaultCode#FAULT_NONE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   DRIVE FAULT MONITORING
   Most critical - servo amplifier has detected an issue
   -------------------------------------------------------------------------- *)

FaultDrive := DriveFault;

(* --------------------------------------------------------------------------
   LINK LOSS MONITORING
   The master heartbeat has stopped; its commands can no longer be trusted.
   A fault only while motion is commanded: there the frozen registers would
   keep the axis moving. In every other state nothing moves, the master
   stopping is routine (the app closed, which also drops the safety relay
   and the drive), and the hold inactivity watchdog returns the slave to
   idle on its own.
   -------------------------------------------------------------------------- *)

FaultComm := CommLost AND InCommandedMotion;

(* --------------------------------------------------------------------------
   OVERTRAVEL FAULT MONITORING
   The positive overtravel switch must never activate during any state.
   The negative overtravel switch is expected only as the homing target
   inside ST_HOME_LIMIT.
   -------------------------------------------------------------------------- *)

IF InHoming THEN
    (* During homing, negative overtravel is the expected target.
       Positive overtravel is never expected - always faults. *)
    IF OvertravelPosActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSIF InRecovery THEN
    (* During recovery, the active switch is expected (we are jogging off it).
       Travel direction is enforced by command clamping in PRG_Main, not here.
       Only a simultaneous both-switches reading is impossible mechanically and
       indicates a wiring/sensor fault. *)
    IF OvertravelNegActive AND OvertravelPosActive THEN
        FaultLimitSwitch := TRUE;
    ELSE
        FaultLimitSwitch := FALSE;
    END_IF;
ELSIF InMotionWatched THEN
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

   Active from boot: the absolute encoder and the controller's stored offset
   keep the coordinate frame across power cycles.

   Suppressed when EITHER:
   - CurrentState = ST_HOME_LIMIT (the homing sequence is running). The
     sequence legitimately drives the axis below SoftLimitMin to find the
     switch, and the coordinate frame is being re-established mid-flight.
   - In a recovery state (InRecovery): the axis starts outside the
     soft-limit box and is being jogged back inside; faulting on the very
     position we are recovering from would defeat the recovery.
   -------------------------------------------------------------------------- *)

IF (CurrentState <> E_SystemState#ST_HOME_LIMIT) AND
   (NOT InRecovery) AND
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
   PRESSURE SENSOR MONITORING
   Only while the pressure loop is closed on the sensor. In every other
   state a bad reading is visible to the master in the pressure register and
   drives nothing.
   -------------------------------------------------------------------------- *)

FaultPressureSensor := PressureSensorFault AND
                       (CurrentState = E_SystemState#ST_PRESSURE_CTRL);

(* --------------------------------------------------------------------------
   FAULT PRIORITY AND OUTPUT
   Determine highest priority RAW fault for this scan.
   Must run BEFORE latch clearing so the clear check sees the current scan's
   value, not a stale value from the previous scan's latch override below.
   -------------------------------------------------------------------------- *)

(* Priority order: Drive > Comm > LimitSwitch > Position > PressureSensor > PistonExit *)
IF FaultDrive THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_DRIVE;

ELSIF FaultComm THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_COMM;

ELSIF FaultLimitSwitch THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_LIMIT_SWITCH;

ELSIF FaultPosition THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_POSITION;

ELSIF FaultPressureSensor THEN
    FaultDetected := TRUE;
    FaultCode := E_FaultCode#FAULT_PRESSURE_SENSOR;

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
IF NOT InFault AND NOT FaultDetected THEN
    bLatchedFault := FALSE;
    eLatchedFaultCode := E_FaultCode#FAULT_NONE;
END_IF;

IF FaultDetected AND NOT bLatchedFault AND NOT InFault THEN
    bLatchedFault := TRUE;
    eLatchedFaultCode := FaultCode;
END_IF;

(* Use latched fault if current scan doesn't detect new fault *)
IF bLatchedFault AND NOT FaultDetected THEN
    FaultDetected := TRUE;
    FaultCode := eLatchedFaultCode;
END_IF;
