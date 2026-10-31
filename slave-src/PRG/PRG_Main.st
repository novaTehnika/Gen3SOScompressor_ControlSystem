(* ============================================================================
   PRG_Main - Main Program with State Machine

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Main program containing the central state machine for servo control.
   Coordinates all function blocks, handles state transitions, and manages
   the handshake protocol with the master controller.

   All built-in motion FBs (MC_Power, MC_Stop, MC_Reset, MC_MoveAbsolute,
   MC_ReadActual*, Y_DirectControl, MC_SetPosition) and
   the PLCopen Toolbox Jog FB are instantiated in the LD POU and communicate
   via G_cmd*/G_sta* structured globals. Homing (ST_HOME_LIMIT) uses Jog.

   Every user-defined FB is called exactly once at the end of this file.
   The step probe (fbProbe) is called last, after the state of the scan is
   final, so its record and its fault mark see the scan's end state.

   State Machine Groups:
   - Idle: ST_IDLE (first scan goes directly here)
   - Transition (Activation): ST_DRIVE_RESET, ST_DRIVE_ENABLE, ST_BRAKE_RELEASE
   - Operational: ST_BRAKE_HOLD, ST_POSITION_CTRL, ST_VELOCITY_CTRL,
                  ST_TORQUE_CTRL, ST_PRESSURE_CTRL, ST_GO_HOME, ST_HOME_LIMIT,
                  ST_HOME_COMPLETE
   - Transition (Inter-Mode): ST_HOLD_POSITION
   - Transition (Deactivation): ST_BRAKE_ENGAGE, ST_DRIVE_DISABLE
   - Fault: ST_FAULT, ST_FAULT_IDLE
   - Recovery: ST_RECOVERY (hub), ST_RECOVERY_POSITION, ST_RECOVERY_VELOCITY
               (jog off a still-active limit; entered from ST_FAULT, drive stays on)

   Variables (define in MWiec GUI):
     VAR:
       (* Function block instances
       fbFilterMotionEnable: FB_DigitalInputFilter
       fbFilterOvertravelNeg : FB_DigitalInputFilter
       fbFilterOvertravelPos : FB_DigitalInputFilter
       fbFilterFaultReset  : FB_DigitalInputFilter
       fbFilterPressure    : FB_AnalogInputFilter
       fbPressureInput     : FB_PressureInput
       fbPressureCtrl      : FB_PressureController
       fbModeDecoder       : FB_ModeDecoder
       fbHandshake         : FB_HandshakeManager
       fbFaultReset        : FB_FaultResetHandler
       fbSafetyMonitor     : FB_SafetyMonitor
       fbPistonExitGuard   : FB_PistonExitGuard
       fbGoHome            : FB_GoHome
       fbHomeLimit         : FB_HomeLimit
       fbPosShaper         : FB_PositionPathShaper
       (* Enum-helper FBs (replacing FN_* with user-enum signatures — MWIEC bug)
       fbIsOperational     : FB_IsOperationalMode
       fbIsHoming          : FB_IsHomingMode
       fbIntToFaultCode    : FB_IntToFaultCode
       fbFaultToInt        : FB_FaultToInt
       fbConfModeToInt     : FB_ModeToInt
       fbCommWatchdog      : FB_CommWatchdog
       (* Step probe: scan-rate data capture, called last
       fbProbe             : FB_Probe
       (* Timers — each has exactly one call site (see end of body)
       tStateTimer         : TON
       tBrakeTimer         : TON
       tDriveReadyTimer    : TON
       tHoldPositionTimer  : TON
       (* Timer IN/PT latches (single-call-per-scan pattern)
       bStateTimerIN       : BOOL
       bBrakeTimerIN       : BOOL
       tBrakeTimerPT       : TIME
       bDriveReadyTimerIN  : BOOL
       bHoldPositionTimerIN : BOOL
       (* Edge triggers
       rTrigMotionEnable   : R_TRIG
       (* Local variables
       bFirstScan          : BOOL := TRUE
       bEnterState         : BOOL
       eTargetState        : E_SystemState
       eTargetMode         : E_OperatingMode
       stCurrentLimits     : ST_AxisLimits
       bFilteredMotionEnable : BOOL
       bFilteredOvertravelNeg : BOOL
       bFilteredOvertravelPos : BOOL
       bFilteredFaultReset : BOOL
       rFilteredPressureVolts : LREAL
       rAnalogFilterAlpha  : LREAL := 0.0
       bFaultResetInFaultState : BOOL
       eFaultResetActiveCode   : E_FaultCode
       (* Limit-recovery context (computed once per scan before the state machine)
       bBlockNeg           : BOOL             (* Negative travel blocked by neg switch or below SoftLimitMin
       bBlockPos           : BOOL             (* Positive travel blocked by pos switch or above SoftLimitMax
       bLimitConditionActive : BOOL           (* Either direction blocked - a limit is currently active
       bRecoveryPending    : BOOL             (* Latched at ST_FAULT_IDLE reset; routes activation to ST_RECOVERY at brake release
       (* Position path shaper (ST_POSITION_CTRL / ST_RECOVERY_POSITION)
       rPosShaperVelMax    : LREAL            (* Shaper speed limit (mm/s): the master's position speed clamped to [0, G_cfgVelLimitMax], set every scan by a position state body
       rPosTarget          : LREAL            (* Soft-limit-clamped position target from the master, input to fbPosShaper
       bPosShaperActive    : BOOL             (* Set by a position state body this scan; gates fbPosShaper.Enable
       bPosStopping        : BOOL             (* Motion enable dropped; braking under the shaper before ST_HOLD_POSITION
       (* Pressure loop (ST_PRESSURE_CTRL)
       rPressureTarget     : LREAL            (* Clamped target from the master, input to fbPressureCtrl
       bPressureCtrlActive : BOOL             (* Set by the pressure state body this scan; gates fbPressureCtrl.Enable
       (* State properties (computed once per scan after the state machine, see State Properties)
       bPowerOn            : BOOL             (* Drive energised: MC_Power enabled (ST_FAULT_IDLE handled separately)
       bDriveStartup       : BOOL             (* ST_DRIVE_RESET / ST_DRIVE_ENABLE: waiting on the drive
       bBrakeReleasing     : BOOL             (* ST_BRAKE_RELEASE: brake release delay
       bBrakeEngaging      : BOOL             (* ST_BRAKE_ENGAGE / ST_FAULT_IDLE: brake engage delay
       bHandshakeState     : BOOL             (* States that accept a new mode request from the master
       bCommandedMotion    : BOOL             (* States that command motion from the master's reference
       bMotionWatched      : BOOL             (* States where either overtravel switch is a fault
       bInHoming           : BOOL             (* ST_HOME_LIMIT / ST_HOME_COMPLETE
       bInGoHome           : BOOL             (* ST_GO_HOME
       bInHomeLimit        : BOOL             (* ST_HOME_LIMIT
       bInRecovery         : BOOL             (* ST_RECOVERY / ST_RECOVERY_POSITION / ST_RECOVERY_VELOCITY
       bInFault            : BOOL             (* ST_FAULT / ST_FAULT_IDLE
       bPositionMode       : BOOL             (* ST_POSITION_CTRL / ST_RECOVERY_POSITION: Y_DirectControl in position mode
       bVelocityMode       : BOOL             (* ST_VELOCITY_CTRL / ST_RECOVERY_VELOCITY: Y_DirectControl in velocity mode
       (* Y_DirectControl hand-off to MC_Stop (see Command Arbitration)
       bKeepDirectControl  : BOOL             (* Keep Y_DirectControl enabled (zero command) until MC_Stop aborts it
       eKeepDirectMode     : Y_ControlMode    (* ControlMode of the direct-control session being stopped
   ============================================================================ *)

(* ============================================================================
   MAIN PROGRAM BODY
   ============================================================================ *)

(* --------------------------------------------------------------------------
   First Scan Initialization
   -------------------------------------------------------------------------- *)
IF bFirstScan THEN
    bFirstScan := FALSE;

    G_sysCurrentState := E_SystemState#ST_IDLE;
    G_sysPreviousState := E_SystemState#ST_IDLE;

    G_doBrakeDisengage := FALSE;
    G_doPerformanceStatus := FALSE;
    G_doHomingComplete := FALSE;
    G_doInMotion := FALSE;

    bRecoveryPending := FALSE;

    IF G_cfgAnalogFilterTimeConst > T#0MS THEN
        rAnalogFilterAlpha := G_cfgScanTime / ((DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / LREAL#1000.0) + G_cfgScanTime);
    ELSE
        rAnalogFilterAlpha := LREAL#1.0;
    END_IF;
    rAnalogFilterAlpha := LIMIT(LREAL#0.0, rAnalogFilterAlpha, LREAL#1.0);
END_IF;


(* --------------------------------------------------------------------------
   Input Processing
   Read filtered values from FB outputs (FBs called at end of file).
   Values reflect the previous scan's filter outputs.
   -------------------------------------------------------------------------- *)

bFilteredMotionEnable := fbFilterMotionEnable.FilteredOutput;
bFilteredOvertravelNeg := fbFilterOvertravelNeg.FilteredOutput;
bFilteredOvertravelPos := fbFilterOvertravelPos.FilteredOutput;
bFilteredFaultReset := fbFilterFaultReset.FilteredOutput;
rFilteredPressureVolts := fbFilterPressure.FilteredOutput;
G_sysActualPressure := fbPressureInput.PressureMPa;

G_sysRequestedMode := fbModeDecoder.DecodedMode;

rTrigMotionEnable(CLK := bFilteredMotionEnable);

G_sysOvertravelNegActive := NOT bFilteredOvertravelNeg;
G_sysOvertravelPosActive := NOT bFilteredOvertravelPos;


(* --------------------------------------------------------------------------
   Limit Recovery Context
   Which travel directions are blocked by an active limit (hard overtravel
   switch or soft-limit overshoot). Used by the ST_FAULT recovery entry and
   the ST_RECOVERY_* sub-states. Positive = into cylinder, Negative = toward exit.
   -------------------------------------------------------------------------- *)

bBlockNeg := G_sysOvertravelNegActive OR (G_sysActualPosition < G_cfgPosSoftLimitMin);
bBlockPos := G_sysOvertravelPosActive OR (G_sysActualPosition > G_cfgPosSoftLimitMax);
bLimitConditionActive := bBlockNeg OR bBlockPos;


(* --------------------------------------------------------------------------
   State Machine
   No inline FB calls. Uses G_sta* for built-in FB status and
   previous-scan outputs for user FB status.
   -------------------------------------------------------------------------- *)

(* Cleared every scan; a position state body sets it when it has produced a
   fresh rPosTarget. On the scan a transition lands in a position state the
   body has not yet run, so the shaper stays disabled and commands actual
   position for that scan rather than a stale target. *)
bPosShaperActive := FALSE;

(* Likewise for the pressure loop. The in-band status output is only
   meaningful in pressure mode and is recomputed there. *)
bPressureCtrlActive := FALSE;
G_doPerformanceStatus := FALSE;

bEnterState := (G_sysCurrentState <> G_sysPreviousState);
IF bEnterState THEN
    IF G_sysCurrentState = E_SystemState#ST_FAULT THEN
        G_sysLastFaultState := G_sysPreviousState;
    END_IF;
    G_sysPreviousState := G_sysCurrentState;
END_IF;

IF G_sysCurrentState = E_SystemState#ST_IDLE THEN

    (* ======================================================================
       IDLE STATE
       ====================================================================== *)

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            G_sysDriveEnabled := FALSE;
            G_sysDriveReady := FALSE;
            G_sysConfirmedMode := E_OperatingMode#MODE_IDLE;
        END_IF;

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;
            fbIsOperational(Mode := eTargetMode);
            fbIsHoming(Mode := eTargetMode);

            IF fbIsOperational.Result OR fbIsHoming.Result THEN
                G_sysCurrentState := E_SystemState#ST_DRIVE_RESET;
            END_IF;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - ACTIVATION
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_RESET THEN

        IF G_staReset.Done THEN
            G_sysCurrentState := E_SystemState#ST_DRIVE_ENABLE;

        ELSIF G_staReset.Error OR tDriveReadyTimer.Q THEN
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE THEN

        IF G_staPower.Status THEN
            G_sysDriveEnabled := TRUE;
            G_sysDriveReady := TRUE;
            G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;

        ELSIF G_staPower.Error OR tDriveReadyTimer.Q THEN
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE THEN

        IF bEnterState THEN
            G_doBrakeDisengage := TRUE;
        END_IF;

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := FALSE;

            IF bRecoveryPending THEN
                (* Re-powered out of ST_FAULT_IDLE to jog off a still-active
                   limit. eTargetMode is stale on this path (the DI bits carried
                   the fault-ack code), so route by the flag, not the mode. *)
                bRecoveryPending := FALSE;
                G_sysCurrentState := E_SystemState#ST_RECOVERY;
            ELSIF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
            ELSIF eTargetMode = E_OperatingMode#MODE_POSITION THEN
                G_sysCurrentState := E_SystemState#ST_POSITION_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_VELOCITY THEN
                G_sysCurrentState := E_SystemState#ST_VELOCITY_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_TORQUE THEN
                G_sysCurrentState := E_SystemState#ST_TORQUE_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_PRESSURE THEN
                G_sysCurrentState := E_SystemState#ST_PRESSURE_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_GO_HOME THEN
                G_sysCurrentState := E_SystemState#ST_GO_HOME;
            ELSIF eTargetMode = E_OperatingMode#MODE_HOME_LIMIT THEN
                G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;
            ELSE
                G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
            END_IF;
        END_IF;


    (* ======================================================================
       OPERATIONAL STATES
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD THEN

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            G_sysConfirmedMode := E_OperatingMode#MODE_BRAKE_HOLD;
        END_IF;

        G_doInMotion := FALSE;

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF eTargetMode = E_OperatingMode#MODE_IDLE THEN
                G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;

            ELSIF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                ;

            ELSIF eTargetMode = E_OperatingMode#MODE_POSITION OR eTargetMode = E_OperatingMode#MODE_VELOCITY OR eTargetMode = E_OperatingMode#MODE_TORQUE OR
                  eTargetMode = E_OperatingMode#MODE_PRESSURE OR
                  eTargetMode = E_OperatingMode#MODE_GO_HOME OR eTargetMode = E_OperatingMode#MODE_HOME_LIMIT THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_RELEASE;
            END_IF;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_POSITION_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_POSITION;
            stCurrentLimits.PosMax := G_cfgPosSoftLimitMax;
            stCurrentLimits.PosMin := G_cfgPosSoftLimitMin;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
            bPosStopping := FALSE;
        END_IF;

        IF NOT bFilteredMotionEnable THEN
            bPosStopping := TRUE;
        END_IF;

        IF bPosStopping THEN
            (* Brake under the shaper's accel limit before handing off to
               MC_Stop: pinning the target to the current command makes the
               distance-to-go zero, so the rate ramps down at AccelMax. *)
            rPosTarget := fbPosShaper.CommandedPosition;
        ELSE
            (* Raw target only; fbPosShaper shapes it into G_sysCommandedPosition *)
            rPosTarget := LIMIT(stCurrentLimits.PosMin,
                                G_sysReference,
                                stCurrentLimits.PosMax);
        END_IF;
        (* Position-move speed from the master (HR7-8), re-read every scan so
           a change acts mid-move. 0 decelerates the command to rest at
           G_cfgPosShaperAccelMax and holds. *)
        rPosShaperVelMax := LIMIT(LREAL#0.0, G_sysPositionSpeed, G_cfgVelLimitMax);
        bPosShaperActive := TRUE;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF bPosStopping AND (ABS(fbPosShaper.CommandedRate) < LREAL#0.001) THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_VELOCITY;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                        G_sysReference,
                                        stCurrentLimits.VelMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF NOT bFilteredMotionEnable THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_TORQUE;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitMax;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
        END_IF;

        G_sysCommandedTorque := LIMIT(-stCurrentLimits.TorqueMax,
                                      G_sysReference,
                                      stCurrentLimits.TorqueMax);

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF NOT bFilteredMotionEnable THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_PRESSURE_CTRL THEN
        (* ----------------------------------------------------------------
           PRESSURE CONTROL - like ST_VELOCITY_CTRL, but the velocity
           command comes from the pressure loop (fbPressureCtrl) and the
           reference is the target pressure in MPa gauge. The pressure-mode
           velocity clamp and torque limit apply; a command toward an active
           limit is zeroed, as in recovery.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_PRESSURE;
            stCurrentLimits.VelMax := MIN(G_cfgPressureVelMax, G_cfgVelLimitMax);
            stCurrentLimits.TorqueMax := G_cfgPressureTorqueLimit;
        END_IF;

        rPressureTarget := LIMIT(LREAL#0.0, G_sysReference, G_cfgPressureTargetMax);
        bPressureCtrlActive := TRUE;

        (* fbPressureCtrl ran at the end of the previous scan. On the entry
           scan it was disabled, so the command starts from zero. *)
        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      fbPressureCtrl.Velocity,
                                      stCurrentLimits.VelMax);

        IF bBlockNeg AND (G_sysCommandedVelocity < LREAL#0.0) THEN
            G_sysCommandedVelocity := LREAL#0.0;
        END_IF;
        IF bBlockPos AND (G_sysCommandedVelocity > LREAL#0.0) THEN
            G_sysCommandedVelocity := LREAL#0.0;
        END_IF;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;
        G_doPerformanceStatus := ABS(G_sysActualPressure - rPressureTarget) <= G_cfgPressureInBand;

        IF NOT bFilteredMotionEnable THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_GO_HOME THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_GO_HOME;
        END_IF;

        G_doInMotion := fbGoHome.InMotion;

        IF fbGoHome.Error THEN
            G_sysActiveFault := E_FaultCode#FAULT_DRIVE;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF NOT bFilteredMotionEnable THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;

        ELSIF fbGoHome.Done THEN
            G_doInMotion := FALSE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIMIT THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_HOME_LIMIT;
            G_flagHomingInProgress := TRUE;
        END_IF;

        G_doInMotion := fbHomeLimit.InMotion;

        IF fbHomeLimit.Done THEN
            G_flagHomingComplete := TRUE;
            G_sysCurrentState := E_SystemState#ST_HOME_COMPLETE;

        ELSIF fbHomeLimit.Error THEN
            G_sysActiveFault := E_FaultCode#FAULT_LIMIT_SWITCH;
            G_sysCurrentState := E_SystemState#ST_FAULT;

        ELSIF NOT bFilteredMotionEnable THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_HOME_COMPLETE THEN

        IF bEnterState THEN
            G_flagHomingInProgress := FALSE;
            G_doHomingComplete := TRUE;
        END_IF;

        G_doInMotion := FALSE;

        IF NOT bFilteredMotionEnable THEN
            G_doHomingComplete := FALSE;
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


    (* ======================================================================
       TRANSITION STATE - INTER-MODE
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_HOLD_POSITION THEN

        IF bEnterState THEN
            G_sysHalted := FALSE;
        END_IF;

        IF NOT G_sysHalted THEN
            IF G_staStop.Done THEN
                G_sysHalted := TRUE;
            END_IF;
        END_IF;

        G_doInMotion := NOT G_sysHalted;

        (* Transitions wait for MC_Stop to have aborted Y_DirectControl
           (bKeepDirectControl cleared in Command Arbitration); otherwise the
           servo can be left in velocity/torque control - see arbitration. *)
        IF G_sysHalted AND NOT bKeepDirectControl AND
           (G_sysRequestedMode = E_OperatingMode#MODE_IDLE) THEN
            (* Master wants IDLE - no handshake required, shut down cleanly *)
            G_sysCurrentState := E_SystemState#ST_BRAKE_ENGAGE;

        ELSIF G_sysHalted AND NOT bKeepDirectControl AND fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
            ELSIF eTargetMode = E_OperatingMode#MODE_POSITION THEN
                G_sysCurrentState := E_SystemState#ST_POSITION_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_VELOCITY THEN
                G_sysCurrentState := E_SystemState#ST_VELOCITY_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_TORQUE THEN
                G_sysCurrentState := E_SystemState#ST_TORQUE_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_PRESSURE THEN
                G_sysCurrentState := E_SystemState#ST_PRESSURE_CTRL;
            ELSIF eTargetMode = E_OperatingMode#MODE_GO_HOME THEN
                G_sysCurrentState := E_SystemState#ST_GO_HOME;
            ELSIF eTargetMode = E_OperatingMode#MODE_HOME_LIMIT THEN
                G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;
            END_IF;
            (* MODE_IDLE handled by direct path above *)

        ELSIF tHoldPositionTimer.Q THEN
            (* Master inactivity watchdog - graceful shutdown to idle, no fault *)
            G_sysCurrentState := E_SystemState#ST_BRAKE_ENGAGE;
        END_IF;


    (* ======================================================================
       TRANSITION STATES - DEACTIVATION
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE THEN

        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
        END_IF;

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;
            G_sysCurrentState := E_SystemState#ST_DRIVE_DISABLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_DISABLE THEN

        IF NOT G_staPower.Status THEN
            G_sysDriveEnabled := FALSE;
            G_sysDriveReady := FALSE;
            G_sysCurrentState := E_SystemState#ST_IDLE;
        END_IF;


    (* ======================================================================
       FAULT STATES
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_FAULT THEN
        (* ----------------------------------------------------------------
           FAULT STATE - Active fault handling

           NO brake engagement here - drive holds position via MC_Stop.
           Two exit paths:
           - Fast recovery: Handshake before timeout -> ST_BRAKE_HOLD (drive stays on)
           - Slow recovery: Timeout -> ST_FAULT_IDLE (drive off, brake on)
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_doInMotion := FALSE;

            G_sysLastFaultCode := G_sysActiveFault;
            G_sysFaultCount := G_sysFaultCount + UDINT#1;

            (* Drop any pending recovery intent if a re-enable aborted to fault *)
            bRecoveryPending := FALSE;
        END_IF;

        IF fbFaultReset.FaultResetValid THEN
            G_sysActiveFault := E_FaultCode#FAULT_NONE;

            IF ((G_sysLastFaultCode = E_FaultCode#FAULT_LIMIT_SWITCH) OR
                (G_sysLastFaultCode = E_FaultCode#FAULT_POSITION)) AND
               bLimitConditionActive THEN
                (* Limit still active - enter recovery to jog off it.
                   Drive stays energized and the brake stays released. *)
                G_sysCurrentState := E_SystemState#ST_RECOVERY;
            ELSE
                G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
            END_IF;

        ELSIF tStateTimer.Q THEN
            G_sysCurrentState := E_SystemState#ST_FAULT_IDLE;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
        (* ----------------------------------------------------------------
           FAULT IDLE STATE - Drive disabled, brake engaged

           Entered after ST_FAULT timeout (slow recovery path).
           Awaits fault reset handshake to return to normal ST_IDLE.

           Drive power-down does not invalidate position — the absolute
           encoder and the controller's stored offset retain it.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_doBrakeDisengage := FALSE;
            G_doInMotion := FALSE;
        END_IF;

        IF tBrakeTimer.Q THEN
            G_sysBrakeEngaged := TRUE;
            IF NOT G_staPower.Status THEN
                G_sysDriveEnabled := FALSE;
            END_IF;
        END_IF;

        IF fbFaultReset.FaultResetValid THEN
            G_sysActiveFault := E_FaultCode#FAULT_NONE;

            IF ((G_sysLastFaultCode = E_FaultCode#FAULT_LIMIT_SWITCH) OR
                (G_sysLastFaultCode = E_FaultCode#FAULT_POSITION)) AND
               bLimitConditionActive THEN
                (* Limit still active - re-power through the activation sequence
                   and divert to recovery at brake release (bRecoveryPending). *)
                bRecoveryPending := TRUE;
                G_sysCurrentState := E_SystemState#ST_DRIVE_RESET;
            ELSE
                G_sysCurrentState := E_SystemState#ST_IDLE;
            END_IF;
        END_IF;


    (* ======================================================================
       RECOVERY STATES
       Jog off a still-active limit (overtravel switch or soft-limit
       overshoot) after a FAULT_LIMIT_SWITCH / FAULT_POSITION reset.
       Drive stays energized and the brake stays released throughout.
       ====================================================================== *)

ELSIF G_sysCurrentState = E_SystemState#ST_RECOVERY THEN
        (* ----------------------------------------------------------------
           RECOVERY HUB - armed, position held via MC_Stop.
           Mirrors ST_BRAKE_HOLD's handshake, but the brake stays released
           so a motion sub-mode can take over without a re-enable sequence.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_doBrakeDisengage := TRUE;
            G_sysConfirmedMode := E_OperatingMode#MODE_BRAKE_HOLD;
        END_IF;

        G_doInMotion := FALSE;

        IF fbHandshake.HandshakeComplete THEN
            eTargetMode := G_sysRequestedMode;

            IF eTargetMode = E_OperatingMode#MODE_POSITION THEN
                G_sysCurrentState := E_SystemState#ST_RECOVERY_POSITION;
            ELSIF eTargetMode = E_OperatingMode#MODE_VELOCITY THEN
                G_sysCurrentState := E_SystemState#ST_RECOVERY_VELOCITY;
            ELSIF eTargetMode = E_OperatingMode#MODE_IDLE THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_ENGAGE;
            ELSIF eTargetMode = E_OperatingMode#MODE_BRAKE_HOLD THEN
                G_sysCurrentState := E_SystemState#ST_BRAKE_HOLD;
            END_IF;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION THEN
        (* ----------------------------------------------------------------
           POSITION RETRACT - like ST_POSITION_CTRL, but the target is also
           blocked from moving further toward the active limit. The normal
           soft-limit clamp already pulls the target back inside the valid
           box; the explicit block covers the (defensive) case of a switch
           tripping inside the soft-limit range.
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_POSITION;
            stCurrentLimits.PosMax := G_cfgPosSoftLimitMax;
            stCurrentLimits.PosMin := G_cfgPosSoftLimitMin;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
            bPosStopping := FALSE;
        END_IF;

        IF NOT bFilteredMotionEnable THEN
            bPosStopping := TRUE;
        END_IF;

        IF bPosStopping THEN
            (* Brake under the shaper's accel limit before handing off to MC_Stop *)
            rPosTarget := fbPosShaper.CommandedPosition;
        ELSE
            (* Raw target only; fbPosShaper shapes it into G_sysCommandedPosition *)
            rPosTarget := LIMIT(stCurrentLimits.PosMin,
                                G_sysReference,
                                stCurrentLimits.PosMax);

            (* Block any target that would drive further toward the active limit *)
            IF bBlockNeg AND (rPosTarget < G_sysActualPosition) THEN
                rPosTarget := G_sysActualPosition;
            END_IF;
            IF bBlockPos AND (rPosTarget > G_sysActualPosition) THEN
                rPosTarget := G_sysActualPosition;
            END_IF;
        END_IF;
        (* Position-move speed from the master (HR7-8), re-read every scan so
           a change acts mid-move. 0 decelerates the command to rest at
           G_cfgPosShaperAccelMax and holds. *)
        rPosShaperVelMax := LIMIT(LREAL#0.0, G_sysPositionSpeed, G_cfgVelLimitMax);
        bPosShaperActive := TRUE;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF bPosStopping AND (ABS(fbPosShaper.CommandedRate) < LREAL#0.001) THEN
            IF bLimitConditionActive THEN
                G_sysCurrentState := E_SystemState#ST_RECOVERY;
            ELSE
                G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
            END_IF;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY THEN
        (* ----------------------------------------------------------------
           VELOCITY RETRACT - like ST_VELOCITY_CTRL, but a command that
           would drive further toward the active limit is zeroed (no fault).
           ---------------------------------------------------------------- *)
        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_VELOCITY;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                        G_sysReference,
                                        stCurrentLimits.VelMax);

        (* Block velocity that would drive further toward the active limit *)
        IF bBlockNeg AND (G_sysCommandedVelocity < LREAL#0.0) THEN
            G_sysCommandedVelocity := LREAL#0.0;
        END_IF;
        IF bBlockPos AND (G_sysCommandedVelocity > LREAL#0.0) THEN
            G_sysCommandedVelocity := LREAL#0.0;
        END_IF;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF NOT bFilteredMotionEnable THEN
            IF bLimitConditionActive THEN
                G_sysCurrentState := E_SystemState#ST_RECOVERY;
            ELSE
                G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
            END_IF;
        END_IF;

END_IF;


(* --------------------------------------------------------------------------
   State Properties
   The one place G_sysCurrentState is classified. Computed here, after the
   state machine has made this scan's transitions, so the timers, command
   arbitration, FB enables and fbSafetyMonitor below all see the state the
   scan ended the state machine in. The safety block at the end of the file
   can still move the state to ST_FAULT; these flags keep the state the
   scan ran in, and the outputs that must follow that move read
   G_sysCurrentState again.
   -------------------------------------------------------------------------- *)

(* Drive energised (MC_Power enabled). ST_FAULT_IDLE is not in the list: it
   keeps the drive on only until the brake has engaged (Command Arbitration). *)
bPowerOn := (G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE) OR
            (G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE) OR
            (G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD) OR
            (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
            (G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
            (G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL) OR
            (G_sysCurrentState = E_SystemState#ST_PRESSURE_CTRL) OR
            (G_sysCurrentState = E_SystemState#ST_GO_HOME) OR
            (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT) OR
            (G_sysCurrentState = E_SystemState#ST_HOME_COMPLETE) OR
            (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) OR
            (G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE) OR
            (G_sysCurrentState = E_SystemState#ST_RECOVERY) OR
            (G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION) OR
            (G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY) OR
            (G_sysCurrentState = E_SystemState#ST_FAULT);

(* Activation and deactivation waits *)
bDriveStartup := (G_sysCurrentState = E_SystemState#ST_DRIVE_RESET) OR
                 (G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE);
bBrakeReleasing := (G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE);
bBrakeEngaging := (G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE) OR
                  (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE);

(* States in which the master may request a new mode (fbHandshake enable) *)
bHandshakeState := (G_sysCurrentState = E_SystemState#ST_IDLE) OR
                   (G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD) OR
                   (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) OR
                   (G_sysCurrentState = E_SystemState#ST_RECOVERY);

(* States that move the axis from the master's reference: a stale heartbeat
   is a fault only here (fbSafetyMonitor, FaultComm) *)
bCommandedMotion := (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
                    (G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
                    (G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL) OR
                    (G_sysCurrentState = E_SystemState#ST_PRESSURE_CTRL) OR
                    (G_sysCurrentState = E_SystemState#ST_GO_HOME) OR
                    (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT) OR
                    (G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION) OR
                    (G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY);

(* States in which an active overtravel switch is a fault (fbSafetyMonitor).
   Homing and recovery have their own rules and are not in the list. *)
bMotionWatched := (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
                  (G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
                  (G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL) OR
                  (G_sysCurrentState = E_SystemState#ST_PRESSURE_CTRL) OR
                  (G_sysCurrentState = E_SystemState#ST_GO_HOME) OR
                  (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION);

bInGoHome := (G_sysCurrentState = E_SystemState#ST_GO_HOME);
bInHomeLimit := (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT);
bInHoming := bInHomeLimit OR (G_sysCurrentState = E_SystemState#ST_HOME_COMPLETE);

bInRecovery := (G_sysCurrentState = E_SystemState#ST_RECOVERY) OR
               (G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION) OR
               (G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY);

bInFault := (G_sysCurrentState = E_SystemState#ST_FAULT) OR
            (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE);

(* Y_DirectControl mode of the states that drive it from a position or a
   velocity command; the recovery states share the normal states' commands *)
bPositionMode := (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
                 (G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION);
bVelocityMode := (G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL) OR
                 (G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY);


(* --------------------------------------------------------------------------
   Timer calls — exactly one call site per timer per scan.
   bEnterState is used as a 1-scan reset gate so the TON sees IN=FALSE
   on the state-entry scan, then IN=TRUE on subsequent scans.
   -------------------------------------------------------------------------- *)

bDriveReadyTimerIN := bDriveStartup AND NOT bEnterState;

bBrakeTimerIN := (bBrakeReleasing OR bBrakeEngaging) AND NOT bEnterState;

IF bBrakeReleasing THEN
    tBrakeTimerPT := G_cfgBrakeDisengageDelay;
ELSIF bBrakeEngaging THEN
    tBrakeTimerPT := G_cfgBrakeEngageDelay;
ELSE
    tBrakeTimerPT := T#0MS;
END_IF;

bHoldPositionTimerIN := (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) AND G_sysHalted;

bStateTimerIN := (G_sysCurrentState = E_SystemState#ST_FAULT) AND NOT bEnterState;

tDriveReadyTimer(IN := bDriveReadyTimerIN, PT := G_cfgDriveReadyTimeout);
tBrakeTimer(IN := bBrakeTimerIN, PT := tBrakeTimerPT);
tHoldPositionTimer(IN := bHoldPositionTimerIN, PT := G_cfgHoldPositionTimeout);
tStateTimer(IN := bStateTimerIN, PT := G_cfgFaultIdleTimeout);


(* --------------------------------------------------------------------------
   Command Arbitration (ST -> LD POU via G_cmd* )
   Default all commands to idle, then override per state.
   -------------------------------------------------------------------------- *)

(* Zero all commands *)
G_cmdPower.Enable := FALSE;
G_cmdPower.EnablePositive := FALSE;
G_cmdPower.EnableNegative := FALSE;
G_cmdStop.Execute := FALSE;
(* Every command default below is the value of the block that may still be
   running after the owning state is left; the owning FB or state overwrites
   it while it commands. A Execute/Forward/Reverse that has dropped does not
   stop the block (MC_MoveAbsolute is not aborted by Execute falling; Jog
   decelerates on its Deceleration pin), so the rates keep real values.
   Targets and velocities cannot matter while the enable is FALSE and are
   zeroed. *)
G_cmdStop.Deceleration := G_cfgStopDecel;   (* the one MC_Stop deceleration: hold, fault, recovery *)
G_cmdReset.Execute := FALSE;
G_cmdMoveAbsolute.Execute := FALSE;
G_cmdMoveAbsolute.Position := LREAL#0.0;
G_cmdMoveAbsolute.Velocity := LREAL#0.0;
G_cmdMoveAbsolute.Acceleration := G_cfgGoHomeAccel;   (* Go Home's move, the one a hold or fault can interrupt before MC_Stop takes the axis *)
G_cmdMoveAbsolute.Deceleration := G_cfgGoHomeDecel;
G_cmdMoveAbsolute.Direction := MC_Direction#Current_Direction;
G_cmdJog.Forward := FALSE;
G_cmdJog.Reverse := FALSE;
G_cmdJog.Velocity := LREAL#0.0;
G_cmdJog.Acceleration := G_cfgHomeLimAccel;   (* homing is the only user of Jog; an abort into ST_HOLD_POSITION drops Forward/Reverse and *)
G_cmdJog.Deceleration := G_cfgHomeLimDecel;   (* the block then stops on this Deceleration, as it did under FB_HomeLimit *)
G_cmdDirectControl.Enable := FALSE;
G_cmdDirectControl.ControlMode := Y_ControlMode#VelocityTLMode;
G_cmdDirectControl.Position := LREAL#0.0;
G_cmdDirectControl.Velocity := LREAL#0.0;
G_cmdDirectControl.Torque := LREAL#0.0;
G_cmdDirectControl.Acceleration := G_cfgDirectCtrlAccel;   (* kept through the MC_Stop hand-off, where Enable stays TRUE *)
G_cmdDirectControl.FilterTimeConstant := G_cfgDirectCtrlFilterTime;
G_cmdSetPosition.Execute := FALSE;
G_cmdSetPosition.Position := LREAL#0.0;
G_cmdSetPosition.Mode := FALSE;

(* MC_Power: enable for all drive-active states. In ST_FAULT_IDLE the drive
   stays on only until the brake has engaged. *)
IF bPowerOn OR ((G_sysCurrentState = E_SystemState#ST_FAULT_IDLE) AND NOT tBrakeTimer.Q) THEN
    G_cmdPower.Enable := TRUE;
    G_cmdPower.EnablePositive := TRUE;
    G_cmdPower.EnableNegative := TRUE;
END_IF;

(* State-direct commands for built-in FBs.

   Y_DirectControl -> MC_Stop hand-off: in VelocityTLMode/TorqueVLMode the
   SERVOPACK itself is switched to velocity/torque control. It is switched
   back to position control only when MC_Stop aborts an *active*
   Y_DirectControl (Yaskawa AN.MPIEC.30 §4.1.3: Enable held TRUE while
   MC_Stop executes). If Enable drops before MC_Stop takes the axis, no
   abort occurs and the servo stays in velocity/torque control, and later
   position-mode blocks are refused. So each direct-control state records
   its mode in eKeepDirectMode and sets bKeepDirectControl; the stopping
   states keep Y_DirectControl enabled with a zero command until MC_Stop is
   Busy and Y_DirectControl.Active drops (Done/Error as fallbacks). *)
IF bPositionMode THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#PositionMode;
    G_cmdDirectControl.Position := G_sysCommandedPosition;   (* previous scan; overridden by fbPosShaper in Route Active FB Command Outputs *)
    (* Y_DirectControl does no path shaping in PositionMode: the position
       command reaches the drive as given. Its Velocity (and Acceleration)
       inputs therefore set no limits in this mode. fbPosShaper
       (FB_PositionPathShaper) does the path shaping, and its VelMax
       (the master's position speed, HR7-8) and AccelMax are what limit a
       position move. The block is given the shaper's speed, so the two
       never disagree. *)
    G_cmdDirectControl.Velocity := rPosShaperVelMax;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#PositionMode;

ELSIF bVelocityMode THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#VelocityTLMode;
    G_cmdDirectControl.Velocity := G_sysCommandedVelocity;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#VelocityTLMode;

ELSIF G_sysCurrentState = E_SystemState#ST_TORQUE_CTRL THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#TorqueVLMode;
    G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
    G_cmdDirectControl.Torque := G_sysCommandedTorque;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#TorqueVLMode;

ELSIF G_sysCurrentState = E_SystemState#ST_PRESSURE_CTRL THEN
    (* Velocity control under the pressure loop, with its own slew limits:
       the chamber stiffens by orders of magnitude as the gas dissolves, so
       the command must be able to reverse within a fraction of a scan's
       travel. *)
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#VelocityTLMode;
    G_cmdDirectControl.Velocity := G_sysCommandedVelocity;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
    G_cmdDirectControl.Acceleration := G_cfgPressureAccel;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#VelocityTLMode;

ELSIF (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) OR
      (G_sysCurrentState = E_SystemState#ST_FAULT) OR
      (G_sysCurrentState = E_SystemState#ST_RECOVERY) THEN

    IF G_sysCurrentState = E_SystemState#ST_HOLD_POSITION THEN
        G_cmdStop.Execute := NOT G_sysHalted;
    ELSE
        G_cmdStop.Execute := NOT G_staStop.Done;
    END_IF;

    (* Hand-off: hold Y_DirectControl enabled (zero command, last mode) so
       MC_Stop aborts it and restores servo position control. *)
    IF bKeepDirectControl THEN
        (* Release once MC_Stop has taken the axis (Busy) and Y_DirectControl
           reports aborted (Active low); Done/Error are fallbacks. Requiring
           Busy guards against an unwired Active reading FALSE forever. *)
        IF (G_staStop.Busy AND NOT G_staDirectControl.Active) OR
           G_staStop.Done OR G_staStop.Error OR G_staDirectControl.Error THEN
            bKeepDirectControl := FALSE;
        ELSE
            G_cmdDirectControl.Enable := TRUE;
            G_cmdDirectControl.ControlMode := eKeepDirectMode;
            G_cmdDirectControl.Position := G_sysCommandedPosition;
            G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
            IF eKeepDirectMode = Y_ControlMode#VelocityTLMode THEN
                G_cmdDirectControl.Velocity := LREAL#0.0;
            ELSIF eKeepDirectMode = Y_ControlMode#PositionMode THEN
                G_cmdDirectControl.Velocity := rPosShaperVelMax;
            ELSE
                G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
            END_IF;
            IF eKeepDirectMode = Y_ControlMode#TorqueVLMode THEN
                G_cmdDirectControl.Torque := LREAL#0.0;
            END_IF;
        END_IF;
    END_IF;

ELSIF G_sysCurrentState = E_SystemState#ST_DRIVE_RESET THEN
    G_cmdReset.Execute := TRUE;

ELSE
    (* Any other state: no direct-control session to hand off *)
    bKeepDirectControl := FALSE;

END_IF;


(* --------------------------------------------------------------------------
   Function Block Calls - Exactly one call per instance
   -------------------------------------------------------------------------- *)

(* Input filters *)
fbFilterMotionEnable(RawInput := G_mbMotionEnable <> UINT#0, DebounceTime := G_cfgDebounceTimeMotion);
fbFilterOvertravelNeg(RawInput := G_diOvertravelNeg, DebounceTime := G_cfgDebounceTimeOvertravelNeg);
fbFilterOvertravelPos(RawInput := G_diOvertravelPos, DebounceTime := G_cfgDebounceTimeOvertravelPos);
fbFilterFaultReset(RawInput := G_mbFaultReset <> UINT#0, DebounceTime := G_cfgDebounceTimeFault);

fbFilterPressure(
    Enable := TRUE,
    RawInput := G_aiPressure,
    Alpha := rAnalogFilterAlpha,
    MedianFilterSize := G_cfgAnalogMedianSize
);

(* Pressure transducer scaling and live-zero supervision *)
fbPressureInput(
    Enable := TRUE,
    Volts := rFilteredPressureVolts,
    ShuntOhms := G_cfgPressureShuntOhms,
    ZeroMa := G_cfgPressureZeroMa,
    SpanMa := G_cfgPressureSpanMa,
    RangeMin := G_cfgPressureRangeMin,
    RangeMax := G_cfgPressureRangeMax,
    MinMa := G_cfgPressureSensorMinMa,
    MaxMa := G_cfgPressureSensorMaxMa,
    FaultTime := G_cfgPressureSensorFaultTime
);

(* Mode decoder *)
fbModeDecoder(
    Enable := TRUE,
    ModeCode := G_mbModeCode,
    DebounceTime := G_cfgDebounceTimeMode
);

(* Handshake manager *)
fbHandshake(
    Enable := bHandshakeState,
    RequestedMode := G_sysRequestedMode,
    ModeStable := fbModeDecoder.ModeStable,
    MotionEnable := bFilteredMotionEnable,
    HandshakeTimeout := G_cfgHandshakeTimeout
);

(* Fault reset handler - parameters differ by state *)
IF G_sysCurrentState = E_SystemState#ST_FAULT THEN
    bFaultResetInFaultState := TRUE;
    eFaultResetActiveCode := G_sysActiveFault;
ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
    bFaultResetInFaultState := (G_sysActiveFault <> E_FaultCode#FAULT_NONE);
    eFaultResetActiveCode := G_sysLastFaultCode;
ELSE
    bFaultResetInFaultState := FALSE;
    eFaultResetActiveCode := E_FaultCode#FAULT_NONE;
END_IF;

(* Fault acknowledgement: the master mirrors the full fault code in the mode register *)
fbIntToFaultCode(Value := G_mbModeCode);

fbFaultReset(
    Enable := bInFault,
    FaultReset := bFilteredFaultReset,
    MotionEnable := bFilteredMotionEnable,
    InFaultState := bFaultResetInFaultState,
    CodeStable := fbModeDecoder.ModeStable,
    ActiveFaultCode := eFaultResetActiveCode,
    MasterFaultCode := fbIntToFaultCode.Fault
);

(* Go Home handler *)
fbGoHome(
    Enable := bInGoHome,
    Execute := bInGoHome,
    HomePosition := G_cfgGoHomePosition,
    HomeVelocity := MIN(G_cfgGoHomeVelocity, G_cfgVelLimitMax),
    Acceleration := G_cfgGoHomeAccel,
    Deceleration := G_cfgGoHomeDecel,
    StopDeceleration := G_cfgStopDecel,
    Abort := NOT bFilteredMotionEnable,
    StaMoveAbsolute := G_staMoveAbsolute,
    StaStop := G_staStop
);

(* Home to Negative Overtravel / Home Reference handler (two-stage + retract) *)
fbHomeLimit(
    Enable := bInHomeLimit,
    Execute := bInHomeLimit,
    FastApproachVelocity := MIN(G_cfgHomeLimFastApproachVel, G_cfgVelLimitMax),
    SlowApproachVelocity := MIN(G_cfgHomeLimSlowApproachVel, G_cfgVelLimitMax),
    BackoffVelocity := MIN(G_cfgHomeLimBackoffVel, G_cfgVelLimitMax),
    RetractDist := G_cfgHomeLimRetractDist,
    Acceleration := G_cfgHomeLimAccel,
    Deceleration := G_cfgHomeLimDecel,
    SetPosition := G_cfgHomeLimSetPosition,
    Timeout := G_cfgHomingTimeout,
    OvertravelNegActive := G_sysOvertravelNegActive,
    OvertravelPosActive := G_sysOvertravelPosActive,
    Abort := NOT bFilteredMotionEnable,
    ActualPosition := G_sysActualPosition,
    StaJog := G_staJog,
    StaMoveAbsolute := G_staMoveAbsolute,
    StaSetPosition := G_staSetPosition
);

(* Safety monitors *)
fbPistonExitGuard(
    Enable := TRUE,
    ActualPosition := G_sysActualPosition,
    ActualVelocity := G_sysActualVelocity,
    ActualTorque := G_sysActualTorque,
    CylinderEntry := G_cfgPosCylinderEntry,
    GuardMargin := G_cfgPosExitGuardMargin,
    VelocityThreshold := G_cfgInMotionVelThreshold,
    TorqueThreshold := G_cfgTorqueStallThreshold
);

(* Master link supervision *)
fbCommWatchdog(Heartbeat := G_mbHeartbeat, Timeout := G_cfgCommTimeout);

fbSafetyMonitor(
    Enable := TRUE,
    OvertravelNegActive := G_sysOvertravelNegActive,
    OvertravelPosActive := G_sysOvertravelPosActive,
    ActualPosition := G_sysActualPosition,
    SoftLimitMin := G_cfgPosSoftLimitMin,
    SoftLimitMax := G_cfgPosSoftLimitMax,
    DriveFault := G_sysDriveFault,
    CurrentState := G_sysCurrentState,
    InCommandedMotion := bCommandedMotion,
    InMotionWatched := bMotionWatched,
    InHoming := bInHoming,
    InRecovery := bInRecovery,
    InFault := bInFault,
    ExitGuardTriggered := fbPistonExitGuard.FaultTriggered,
    CommLost := fbCommWatchdog.Lost,
    PressureSensorFault := fbPressureInput.SensorFault
);

(* Position path shaper - shapes rPosTarget into the Y_DirectControl command.
   Enable is bPosShaperActive (set only by a position state body that ran this
   scan), not the state itself, so the transition scan commands actual position.
   While disabled it seeds from actual, so every entry is bumpless. *)
fbPosShaper(
    Enable := bPosShaperActive,
    Reset := bEnterState AND bPosShaperActive,
    Target := rPosTarget,
    ActualPosition := G_sysActualPosition,
    VelMax := rPosShaperVelMax,
    AccelMax := G_cfgPosShaperAccelMax,
    MaxDeviation := G_cfgPosShaperMaxDeviation,
    CycleTime := G_cfgScanTime
);

(* Pressure loop - enabled only on scans where the pressure state body ran,
   so the transition scan commands zero velocity. Reset on entry clears the
   integrator. *)
fbPressureCtrl(
    Enable := bPressureCtrlActive,
    Reset := bEnterState AND bPressureCtrlActive,
    Target := rPressureTarget,
    Actual := G_sysActualPressure,
    Kp := G_cfgPressureKp,
    Ki := G_cfgPressureKi,
    VelMax := stCurrentLimits.VelMax,
    DecelMax := G_cfgPressureDecel,
    Stiffness := G_cfgPressureStopStiffness,
    StopVelMin := G_cfgPressureStopVelMin,
    CycleTime := G_cfgScanTime
);


(* --------------------------------------------------------------------------
   Route Active FB Command Outputs -> G_cmd*
   Overrides the zeroed defaults from Command Arbitration for states
   where a user FB owns the motion command.
   -------------------------------------------------------------------------- *)

IF bInGoHome THEN
    G_cmdMoveAbsolute := fbGoHome.CmdMoveAbsolute;
    G_cmdStop := fbGoHome.CmdStop;

ELSIF bInHomeLimit THEN
    G_cmdJog := fbHomeLimit.CmdJog;
    G_cmdMoveAbsolute := fbHomeLimit.CmdMoveAbsolute;
    G_cmdSetPosition := fbHomeLimit.CmdSetPosition;

ELSIF bPositionMode THEN
    (* Shaped command (= actual position on the transition scan) *)
    G_sysCommandedPosition := fbPosShaper.CommandedPosition;
    G_cmdDirectControl.Position := fbPosShaper.CommandedPosition;

END_IF;


(* --------------------------------------------------------------------------
   Safety Monitoring
   Runs after FB calls so safety FBs have fresh outputs.
   -------------------------------------------------------------------------- *)

G_sysExitGuardActive := fbPistonExitGuard.GuardActive;
G_sysInCylinderZone := NOT fbPistonExitGuard.InDangerZone;

IF fbSafetyMonitor.FaultDetected AND NOT bInFault THEN
    IF NOT (G_sysCurrentState = E_SystemState#ST_DRIVE_RESET AND
            fbSafetyMonitor.FaultCode = E_FaultCode#FAULT_DRIVE) THEN
        G_sysActiveFault := fbSafetyMonitor.FaultCode;
        G_sysCurrentState := E_SystemState#ST_FAULT;
    END_IF;
END_IF;


(* --------------------------------------------------------------------------
   Update Outputs
   -------------------------------------------------------------------------- *)

(* Reads the state again: the safety block above may have just entered ST_FAULT *)
G_doFaultActive := (G_sysCurrentState = E_SystemState#ST_FAULT) OR
                   (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE);

(* Values published in the Modbus input registers by PRG_Output *)
fbConfModeToInt(Mode := fbHandshake.ConfirmedMode);
G_sysConfirmedModeCode := INT_TO_UINT(fbConfModeToInt.Value);
fbFaultToInt(Fault := G_sysActiveFault);
G_sysActiveFaultCode := INT_TO_UINT(fbFaultToInt.Value);
IF G_sysScanCounter >= UINT#65535 THEN
    G_sysScanCounter := UINT#0;
ELSE
    G_sysScanCounter := G_sysScanCounter + UINT#1;
END_IF;


(* --------------------------------------------------------------------------
   Step Probe
   Records this scan's values in the probe's ring and serves the master's
   window requests (see FB_Probe). Last in the file: the state, the status
   outputs and the confirmed mode code are final for this scan, and a fault
   entered by the safety monitor above marks this scan's record. The staging
   array is passed by reference; PRG_Output publishes it.
   -------------------------------------------------------------------------- *)
fbProbe(
    Enable := G_cfgProbeEnable,
    FillPerScan := G_cfgProbeFillPerScan,
    VelCmd := G_sysCommandedVelocity,
    Vel := G_sysActualVelocity,
    Torque := G_sysActualTorque,
    Position := G_sysActualPosition,
    Pressure := G_sysActualPressure,
    Volts := G_aiPressure,
    InMotion := G_doInMotion,
    Performance := G_doPerformanceStatus,
    FaultActive := G_doFaultActive,
    HomingComplete := G_doHomingComplete,
    ModeCode := G_sysConfirmedModeCode,
    FaultState := (G_sysCurrentState = E_SystemState#ST_FAULT),
    Arm := G_mbProbeArm,
    Length := G_mbProbeLength,
    ReqLo := G_mbProbeReqLo,
    ReqHi := G_mbProbeReqHi,
    Stage := G_sysProbeStage
);
G_sysProbePublish := fbProbe.Publish;
G_sysProbeWritten := fbProbe.Written;
G_sysProbeMark := fbProbe.Mark;
G_sysProbeFaultMark := fbProbe.FaultMark;
G_sysProbeStart := fbProbe.Start;
G_sysProbeCount := fbProbe.Count;
G_sysProbeArmEcho := fbProbe.ArmEcho;
G_sysProbeDepth := fbProbe.Depth;
G_sysProbeWords := fbProbe.Words;
G_sysProbeState := fbProbe.StateCode;
G_sysProbeXfer := fbProbe.XferCode;
