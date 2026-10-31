(* ============================================================================
   PRG_Main - Main Program with State Machine

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Main program containing the central state machine for servo control.
   Coordinates all function blocks, handles state transitions, and manages
   the handshake protocol with the master controller.

   All built-in motion FBs (MC_Power, MC_Stop, MC_Reset, MC_MoveAbsolute,
   MC_MoveVelocity, MC_ReadActual*, Y_DirectControl, MC_SetPosition) and
   the PLCopen Toolbox Jog FB are instantiated in the LD POU and communicate
   via G_cmd*/G_sta* structured globals.
   Note: MC_MoveVelocity is retained in the LD POU but currently unused;
   homing (ST_HOME_LIMIT) now uses Jog.

   Every user-defined FB is called exactly once at the end of this file.

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
       fbPressureSched     : FB_PressureSchedule
       fbModeDecoder       : FB_ModeDecoder
       fbHandshake         : FB_HandshakeManager
       fbFaultReset        : FB_FaultResetHandler
       fbSafetyMonitor     : FB_SafetyMonitor
       fbPistonExitGuard   : FB_PistonExitGuard
       fbGoHome            : FB_GoHome
       fbHomeLimit         : FB_HomeLimit
       fbPosGate           : FB_PositionCommandGate
       (* Enum-helper FBs (replacing FN_* with user-enum signatures — MWIEC bug)
       fbIsOperational     : FB_IsOperationalMode
       fbIsHoming          : FB_IsHomingMode
       fbIntToFaultCode    : FB_IntToFaultCode
       fbFaultToInt        : FB_FaultToInt
       fbConfModeToInt     : FB_ModeToInt
       fbCommWatchdog      : FB_CommWatchdog
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
       rAnalogFilterTime   : LREAL := 0.0     (* G_cfgAnalogFilterTimeConst in seconds, for fbPressureSched
       bFaultResetInFaultState : BOOL
       eFaultResetActiveCode   : E_FaultCode
       (* Limit-recovery context (computed once per scan before the state machine)
       bBlockNeg           : BOOL             (* Negative travel blocked by neg switch or below SoftLimitMin
       bBlockPos           : BOOL             (* Positive travel blocked by pos switch or above SoftLimitMax
       bLimitConditionActive : BOOL           (* Either direction blocked - a limit is currently active
       bRecoveryPending    : BOOL             (* Latched at ST_FAULT_IDLE reset; routes activation to ST_RECOVERY at brake release
       (* Position command gate (ST_POSITION_CTRL / ST_RECOVERY_POSITION)
       rPosTarget          : LREAL            (* Soft-limit-clamped position target from the master, input to fbPosGate
       bPosGateActive      : BOOL             (* Set by a position state body this scan; gates fbPosGate.Enable
       bPosStopping        : BOOL             (* Motion enable dropped; braking under the gate before ST_HOLD_POSITION
       (* Pressure loop (ST_PRESSURE_CTRL)
       rPressureTarget     : LREAL            (* Clamped target from the master, input to fbPressureCtrl
       bPressureCtrlActive : BOOL             (* Set by the pressure state body this scan; gates fbPressureCtrl.Enable
       rSchedSaline        : LREAL            (* G_sysSalineVolume latched on entry to pressure mode (mL)
       rSchedOxygenRatio   : LREAL            (* G_sysOxygenRatio latched on entry to pressure mode
       bPressureSchedActive : BOOL            (* G_cfgPressureScheduleEnable latched on entry to pressure mode
       bPressureSchedUsed  : BOOL             (* fbPressureSched drives fbPressureCtrl's gains this scan
       rPressureKp         : LREAL            (* Gains and limits passed to fbPressureCtrl this scan:
       rPressureKi         : LREAL            (*   scheduled, or the G_cfgPressure* values
       rPressureVelMax     : LREAL
       rPressureStiffness  : LREAL
       rPressureIntegScale : LREAL := 1.0
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

    rAnalogFilterTime := DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / LREAL#1000.0;
    IF G_cfgAnalogFilterTimeConst > T#0MS THEN
        rAnalogFilterAlpha := G_cfgScanTime / (rAnalogFilterTime + G_cfgScanTime);
    ELSE
        rAnalogFilterAlpha := LREAL#1.0;
    END_IF;
    rAnalogFilterAlpha := LIMIT(LREAL#0.0, rAnalogFilterAlpha, LREAL#1.0);
END_IF;


(* --------------------------------------------------------------------------
   Input Processing
   Read filtered values from FB outputs (FBs called at end of file).
   Values reflect previous scan's filter outputs — negligible at 1ms cycle.
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
   body has not yet run, so the gate stays disabled and commands actual
   position for that scan rather than a stale target. *)
bPosGateActive := FALSE;

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
            stCurrentLimits.VelMax := G_cfgVelLimitNormal;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
            bPosStopping := FALSE;
        END_IF;

        IF NOT bFilteredMotionEnable THEN
            bPosStopping := TRUE;
        END_IF;

        IF bPosStopping THEN
            (* Brake under the gate's accel limit before handing off to
               MC_Stop: pinning the target to the current command makes the
               distance-to-go zero, so the rate ramps down at AccelMax. *)
            rPosTarget := fbPosGate.CommandedPosition;
        ELSE
            (* Raw target only; fbPosGate shapes it into G_sysCommandedPosition *)
            rPosTarget := G_sysReference;
            rPosTarget := LIMIT(stCurrentLimits.PosMin,
                                rPosTarget,
                                stCurrentLimits.PosMax);
        END_IF;
        bPosGateActive := TRUE;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF bPosStopping AND (ABS(fbPosGate.CommandedRate) < LREAL#0.001) THEN
            G_sysCurrentState := E_SystemState#ST_HOLD_POSITION;
        END_IF;


ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN

        IF bEnterState THEN
            G_sysConfirmedMode := E_OperatingMode#MODE_VELOCITY;
            stCurrentLimits.VelMax := G_cfgVelLimitMax;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
        END_IF;

        G_sysCommandedVelocity := G_sysReference;
        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      G_sysCommandedVelocity,
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

        G_sysCommandedTorque := G_sysReference;
        G_sysCommandedTorque := LIMIT(-stCurrentLimits.TorqueMax,
                                    G_sysCommandedTorque,
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
            stCurrentLimits.VelMax := G_cfgPressureVelMax;
            stCurrentLimits.TorqueMax := G_cfgPressureTorqueLimit;
            (* The gain schedule and the fill it is computed for are chosen
               once per entry, so a change of the switch or of the fill
               registers never changes the gains mid-run. *)
            bPressureSchedActive := G_cfgPressureScheduleEnable;
            rSchedSaline := G_sysSalineVolume;
            rSchedOxygenRatio := G_sysOxygenRatio;
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


    (* Deprecated sub-states - kept for E_SystemState compatibility *)
ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIM_APPROACH OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_DETECT OR
      G_sysCurrentState = E_SystemState#ST_HOME_LIM_BACKOFF OR G_sysCurrentState = E_SystemState#ST_HOME_LIM_SETREF THEN
        G_sysCurrentState := E_SystemState#ST_HOME_LIMIT;


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
            stCurrentLimits.VelMax := G_cfgVelLimitNormal;
            stCurrentLimits.TorqueMax := G_cfgTorqueLimitNormal;
            bPosStopping := FALSE;
        END_IF;

        IF NOT bFilteredMotionEnable THEN
            bPosStopping := TRUE;
        END_IF;

        IF bPosStopping THEN
            (* Brake under the gate's accel limit before handing off to MC_Stop *)
            rPosTarget := fbPosGate.CommandedPosition;
        ELSE
            (* Raw target only; fbPosGate shapes it into G_sysCommandedPosition *)
            rPosTarget := G_sysReference;
            rPosTarget := LIMIT(stCurrentLimits.PosMin,
                                rPosTarget,
                                stCurrentLimits.PosMax);

            (* Block any target that would drive further toward the active limit *)
            IF bBlockNeg AND (rPosTarget < G_sysActualPosition) THEN
                rPosTarget := G_sysActualPosition;
            END_IF;
            IF bBlockPos AND (rPosTarget > G_sysActualPosition) THEN
                rPosTarget := G_sysActualPosition;
            END_IF;
        END_IF;
        bPosGateActive := TRUE;

        G_doInMotion := ABS(G_sysActualVelocity) > G_cfgInMotionVelThreshold;

        IF bPosStopping AND (ABS(fbPosGate.CommandedRate) < LREAL#0.001) THEN
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

        G_sysCommandedVelocity := G_sysReference;
        G_sysCommandedVelocity := LIMIT(-stCurrentLimits.VelMax,
                                      G_sysCommandedVelocity,
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
   Timer calls — exactly one call site per timer per scan.
   bEnterState is used as a 1-scan reset gate so the TON sees IN=FALSE
   on the state-entry scan, then IN=TRUE on subsequent scans.
   -------------------------------------------------------------------------- *)

bDriveReadyTimerIN := ((G_sysCurrentState = E_SystemState#ST_DRIVE_RESET) OR
                        (G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE)) AND NOT bEnterState;

bBrakeTimerIN := ((G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE) OR
                  (G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE) OR
                  (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE)) AND NOT bEnterState;

IF G_sysCurrentState = E_SystemState#ST_BRAKE_RELEASE THEN
    tBrakeTimerPT := G_cfgBrakeDisengageDelay;
ELSIF (G_sysCurrentState = E_SystemState#ST_BRAKE_ENGAGE) OR
      (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE) THEN
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
G_cmdStop.Deceleration := G_cfgGoHomeDecel;
G_cmdReset.Execute := FALSE;
G_cmdMoveAbsolute.Execute := FALSE;
G_cmdMoveAbsolute.Position := LREAL#0.0;
G_cmdMoveAbsolute.Velocity := LREAL#0.0;
G_cmdMoveAbsolute.Acceleration := G_cfgGoHomeAccel;
G_cmdMoveAbsolute.Deceleration := G_cfgGoHomeDecel;
G_cmdMoveAbsolute.Direction := MC_Direction#Current_Direction;
G_cmdMoveVelocity.Execute := FALSE;
G_cmdMoveVelocity.Velocity := LREAL#0.0;
G_cmdMoveVelocity.Acceleration := G_cfgHomeLimAccel;
G_cmdMoveVelocity.Deceleration := G_cfgHomeLimDecel;
G_cmdMoveVelocity.Direction := MC_Direction#Current_Direction;
G_cmdJog.Forward := FALSE;
G_cmdJog.Reverse := FALSE;
G_cmdJog.Velocity := LREAL#0.0;
G_cmdJog.Acceleration := G_cfgHomeLimAccel;
G_cmdJog.Deceleration := G_cfgHomeLimDecel;
G_cmdDirectControl.Enable := FALSE;
G_cmdDirectControl.ControlMode := Y_ControlMode#VelocityTLMode;
G_cmdDirectControl.Position := LREAL#0.0;
G_cmdDirectControl.Velocity := LREAL#0.0;
G_cmdDirectControl.Torque := LREAL#0.0;
G_cmdDirectControl.Acceleration := G_cfgDirectCtrlAccel;
G_cmdDirectControl.Deceleration := G_cfgDirectCtrlDecel;
G_cmdDirectControl.FilterTimeConstant := G_cfgDirectCtrlFilterTime;
G_cmdSetPosition.Execute := FALSE;
G_cmdSetPosition.Position := LREAL#0.0;
G_cmdSetPosition.Mode := FALSE;

(* MC_Power: enable for all drive-active states *)
IF (G_sysCurrentState = E_SystemState#ST_DRIVE_ENABLE) OR
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
   (G_sysCurrentState = E_SystemState#ST_FAULT) THEN
    G_cmdPower.Enable := TRUE;
    G_cmdPower.EnablePositive := TRUE;
    G_cmdPower.EnableNegative := TRUE;
ELSIF G_sysCurrentState = E_SystemState#ST_FAULT_IDLE THEN
    IF NOT tBrakeTimer.Q THEN
        G_cmdPower.Enable := TRUE;
        G_cmdPower.EnablePositive := TRUE;
        G_cmdPower.EnableNegative := TRUE;
    END_IF;
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
IF G_sysCurrentState = E_SystemState#ST_POSITION_CTRL THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#PositionMode;
    G_cmdDirectControl.Position := G_sysCommandedPosition;   (* previous scan; overridden by fbPosGate in Route Active FB Command Outputs *)
    G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#PositionMode;

ELSIF G_sysCurrentState = E_SystemState#ST_VELOCITY_CTRL THEN
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
    G_cmdDirectControl.Deceleration := G_cfgPressureDecel;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#VelocityTLMode;

ELSIF G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#PositionMode;
    G_cmdDirectControl.Position := G_sysCommandedPosition;   (* previous scan; overridden by fbPosGate in Route Active FB Command Outputs *)
    G_cmdDirectControl.Velocity := stCurrentLimits.VelMax;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
    bKeepDirectControl := TRUE;
    eKeepDirectMode := Y_ControlMode#PositionMode;

ELSIF G_sysCurrentState = E_SystemState#ST_RECOVERY_VELOCITY THEN
    G_cmdDirectControl.Enable := TRUE;
    G_cmdDirectControl.ControlMode := Y_ControlMode#VelocityTLMode;
    G_cmdDirectControl.Velocity := G_sysCommandedVelocity;
    G_cmdDirectControl.Torque := stCurrentLimits.TorqueMax;
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
    Enable := (G_sysCurrentState = E_SystemState#ST_IDLE) OR
              (G_sysCurrentState = E_SystemState#ST_BRAKE_HOLD) OR
              (G_sysCurrentState = E_SystemState#ST_HOLD_POSITION) OR
              (G_sysCurrentState = E_SystemState#ST_RECOVERY),
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
    Enable := (G_sysCurrentState = E_SystemState#ST_FAULT) OR
              (G_sysCurrentState = E_SystemState#ST_FAULT_IDLE),
    FaultReset := bFilteredFaultReset,
    MotionEnable := bFilteredMotionEnable,
    InFaultState := bFaultResetInFaultState,
    CodeStable := fbModeDecoder.ModeStable,
    ActiveFaultCode := eFaultResetActiveCode,
    MasterFaultCode := fbIntToFaultCode.Fault
);

(* Go Home handler *)
fbGoHome(
    Enable := (G_sysCurrentState = E_SystemState#ST_GO_HOME),
    Execute := (G_sysCurrentState = E_SystemState#ST_GO_HOME),
    HomePosition := G_cfgGoHomePosition,
    HomeVelocity := G_cfgGoHomeVelocity,
    Acceleration := G_cfgGoHomeAccel,
    Deceleration := G_cfgGoHomeDecel,
    Abort := NOT bFilteredMotionEnable,
    StaMoveAbsolute := G_staMoveAbsolute,
    StaStop := G_staStop
);

(* Home to Negative Overtravel / Home Reference handler (two-stage + retract) *)
fbHomeLimit(
    Enable := (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT),
    Execute := (G_sysCurrentState = E_SystemState#ST_HOME_LIMIT),
    FastApproachVelocity := G_cfgHomeLimFastApproachVel,
    SlowApproachVelocity := G_cfgHomeLimSlowApproachVel,
    BackoffVelocity := G_cfgHomeLimBackoffVel,
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
    ExitGuardTriggered := fbPistonExitGuard.FaultTriggered,
    CommLost := fbCommWatchdog.Lost,
    PressureSensorFault := fbPressureInput.SensorFault
);

(* Position command gate - shapes rPosTarget into the Y_DirectControl command.
   Enable is bPosGateActive (set only by a position state body that ran this
   scan), not the state itself, so the transition scan commands actual position.
   While disabled it seeds from actual, so every entry is bumpless. *)
fbPosGate(
    Enable := bPosGateActive,
    Reset := bEnterState AND bPosGateActive,
    Target := rPosTarget,
    ActualPosition := G_sysActualPosition,
    VelMax := stCurrentLimits.VelMax,
    AccelMax := G_cfgPosGateAccelMax,
    MaxDeviation := G_cfgPosGateMaxDeviation,
    CycleTime := G_cfgScanTime
);

(* Pressure-loop gain schedule - Kp, Ki and the stop-law limit from the
   stiffness bound, when selected on entry. A ParamError falls back to the
   configured gains. *)
fbPressureSched(
    Enable := bPressureCtrlActive AND bPressureSchedActive,
    Reset := bEnterState AND bPressureCtrlActive,
    Target := rPressureTarget,
    Actual := G_sysActualPressure,
    Position := G_sysActualPosition,
    EndOfTravel := G_cfgPressureSchedEndOfTravel,
    VelMax := stCurrentLimits.VelMax,
    DecelMax := G_cfgPressureDecel,
    StopVelMin := G_cfgPressureStopVelMin,
    CycleTime := G_cfgScanTime,
    FilterTime := rAnalogFilterTime,
    MedianSize := G_cfgAnalogMedianSize,
    DriveDeadTime := G_cfgPressureSchedDriveDeadTime,
    DriveLag := G_cfgPressureSchedDriveLag,
    Crossover := G_cfgPressureSchedCrossover,
    KiRatio := G_cfgPressureSchedKiRatio,
    KpMin := G_cfgPressureSchedKpMin,
    KpMax := G_cfgPressureSchedKpMax,
    PistonArea := G_cfgPressureSchedPistonArea,
    SalineVolume := rSchedSaline,
    PassagesVolume := G_cfgPressureSchedPassagesVolume,
    HoseVolume := G_cfgPressureSchedHoseVolume,
    HoseCompliance := G_cfgPressureSchedHoseCompliance,
    HoseOpen := G_cfgPressureSchedHoseOpen,
    BulkModulus := G_cfgPressureSchedBulkModulus,
    OxygenRatio := rSchedOxygenRatio,
    Henry := G_cfgPressureSchedHenry,
    GasTemp := G_cfgPressureSchedGasTemp,
    Patm := G_cfgPressureSchedPatm,
    ArriveBand := G_cfgPressureSchedArriveBand
);

bPressureSchedUsed := bPressureCtrlActive AND bPressureSchedActive AND
                      NOT fbPressureSched.ParamError;
IF bPressureSchedUsed THEN
    rPressureKp := fbPressureSched.Kp;
    rPressureKi := fbPressureSched.Ki;
    rPressureVelMax := fbPressureSched.VelLimit;
    rPressureStiffness := fbPressureSched.Stiffness;
    rPressureIntegScale := fbPressureSched.IntegScale;
ELSE
    rPressureKp := G_cfgPressureKp;
    rPressureKi := G_cfgPressureKi;
    rPressureVelMax := stCurrentLimits.VelMax;
    rPressureStiffness := G_cfgPressureStopStiffness;
    rPressureIntegScale := LREAL#1.0;
END_IF;

(* Pressure loop - enabled only on scans where the pressure state body ran,
   so the transition scan commands zero velocity. Reset on entry clears the
   integrator; under the schedule, so does the hold until the first
   arrival. *)
fbPressureCtrl(
    Enable := bPressureCtrlActive,
    Reset := (bEnterState AND bPressureCtrlActive) OR
             (bPressureSchedUsed AND fbPressureSched.IntegHold),
    Target := rPressureTarget,
    Actual := G_sysActualPressure,
    Kp := rPressureKp,
    Ki := rPressureKi,
    VelMax := rPressureVelMax,
    DecelMax := G_cfgPressureDecel,
    Stiffness := rPressureStiffness,
    StopVelMin := G_cfgPressureStopVelMin,
    CycleTime := G_cfgScanTime,
    IntegScale := rPressureIntegScale
);


(* --------------------------------------------------------------------------
   Route Active FB Command Outputs -> G_cmd*
   Overrides the zeroed defaults from Command Arbitration for states
   where a user FB owns the motion command.
   -------------------------------------------------------------------------- *)

IF G_sysCurrentState = E_SystemState#ST_GO_HOME THEN
    G_cmdMoveAbsolute := fbGoHome.CmdMoveAbsolute;
    G_cmdStop := fbGoHome.CmdStop;

ELSIF G_sysCurrentState = E_SystemState#ST_HOME_LIMIT THEN
    G_cmdJog := fbHomeLimit.CmdJog;
    G_cmdMoveAbsolute := fbHomeLimit.CmdMoveAbsolute;
    G_cmdSetPosition := fbHomeLimit.CmdSetPosition;

ELSIF (G_sysCurrentState = E_SystemState#ST_POSITION_CTRL) OR
      (G_sysCurrentState = E_SystemState#ST_RECOVERY_POSITION) THEN
    (* Gated command (= actual position on the transition scan) *)
    G_sysCommandedPosition := fbPosGate.CommandedPosition;
    G_cmdDirectControl.Position := fbPosGate.CommandedPosition;

END_IF;


(* --------------------------------------------------------------------------
   Safety Monitoring
   Runs after FB calls so safety FBs have fresh outputs.
   -------------------------------------------------------------------------- *)

G_sysExitGuardActive := fbPistonExitGuard.GuardActive;
G_sysInCylinderZone := NOT fbPistonExitGuard.InDangerZone;

IF fbSafetyMonitor.FaultDetected AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT) AND
   (G_sysCurrentState <> E_SystemState#ST_FAULT_IDLE) THEN
    IF NOT (G_sysCurrentState = E_SystemState#ST_DRIVE_RESET AND
            fbSafetyMonitor.FaultCode = E_FaultCode#FAULT_DRIVE) THEN
        G_sysActiveFault := fbSafetyMonitor.FaultCode;
        G_sysCurrentState := E_SystemState#ST_FAULT;
    END_IF;
END_IF;


(* --------------------------------------------------------------------------
   Update Outputs
   -------------------------------------------------------------------------- *)

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

G_sysCycleCount := G_sysCycleCount + UDINT#1;
