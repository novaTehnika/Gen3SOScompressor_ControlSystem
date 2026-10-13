(* ============================================================================
   GlobalVariables_Reference - Global Variable Reference Document

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   This file documents all global variables that need to be configured
   in the MWiec GUI. It is a REFERENCE ONLY - variables are defined
   in the MotionWorks IEC Express global variable editor.

   Organized by functional group:
   1. I/O Mappings (from GVL_IO)
   2. Configurable Parameters (from GVL_Config)
   3. Position References (from GVL_Position)
   4. Runtime State Variables (from GVL_System)

   Units:
   - Position: millimeters (mm)
   - Velocity: mm/s
   - Torque: % of rated (100 = 100%, up to 800% peak)
   - Time: TIME type (T#xxxMS, T#xxxS)
   ============================================================================ *)


(* ============================================================================
   1. I/O MAPPINGS
   ============================================================================

   Note: Actual I/O addresses (%IX, %QX, %IW, %QW) must be verified against
   the hardware configuration in MotionWorksIEC Express.

   VAR_GLOBAL
     (* ========================================================================
        DIGITAL INPUTS (8 Available)
        PNP limit switches: TRUE = not triggered, FALSE = triggered (fail-safe)
        ======================================================================== *)

     (* Mode Command from Master (3-bit) *)
     G_diModeBit0          AT %IX0.0 : BOOL   (* Mode bit 0 - LSB *)
     G_diModeBit1          AT %IX0.1 : BOOL   (* Mode bit 1 *)
     G_diModeBit2          AT %IX0.2 : BOOL   (* Mode bit 2 - MSB *)

     (* Motion Control *)
     G_diMotionEnable      AT %IX0.3 : BOOL   (* Motion enable from master *)

     (* Fault Control *)
     G_diFaultReset        AT %IX0.4 : BOOL   (* Fault reset command from master *)

     (* Reserved *)
     (*G_diReserved          AT %IX0.5 : BOOL   (* Reserved for future use *)

     (* Overtravel Switches - Tolomatic #8100-9092, Solid State, NC, PNP *)
     (* TRUE = switch NOT triggered (normal), FALSE = triggered or wire broken *)
     (* Negative overtravel doubles as the home reference switch (Mode 110). *)
     G_diOvertravelNeg     AT %IX0.6 : BOOL   (* Negative overtravel / home reference *)
     G_diOvertravelPos     AT %IX0.7 : BOOL   (* Positive overtravel *)


     (* ========================================================================
        DIGITAL OUTPUTS (8 Available)
        ======================================================================== *)

     (* Mode Confirmation to Master (3-bit) / Fault Code when G_doFaultActive=TRUE *)
     G_doModeConfBit0      AT %QX0.0 : BOOL   (* Mode confirm bit 0 / Fault code bit 0 *)
     G_doModeConfBit1      AT %QX0.1 : BOOL   (* Mode confirm bit 1 / Fault code bit 1 *)
     G_doModeConfBit2      AT %QX0.2 : BOOL   (* Mode confirm bit 2 / Fault code bit 2 *)

     (* Status Outputs *)
     G_doPerformanceStatus AT %QX0.3 : BOOL   (* Mode-dependent status indicator *)
     G_doFaultActive       AT %QX0.4 : BOOL   (* Fault indicator: HIGH = fault active *)
     G_doHomingComplete    AT %QX0.5 : BOOL   (* Homing sequence complete *)
     G_doInMotion          AT %QX0.6 : BOOL   (* Axis in motion indicator *)

     (* Brake Control *)
     (* Spring-engaged, electrically released: HIGH = brake disengaged (motion allowed) *)
     G_doBrakeDisengage    AT %QX0.7 : BOOL   (* Brake release command *)


     (* ========================================================================
        ANALOG INPUTS (1 Available)
        Range: -10V to +10V (LREAL, volts)
        ======================================================================== *)

     G_aiReference         AT %IW0   : LREAL  (* Not used by the protocol; the *)
                                             (* pressure channel takes this input *)


     (* ========================================================================
        MODBUS HOLDING REGISTERS (master -> slave), group FC#03,06,16
        With "Use %M for bidirectional Holding Registers" the group lives in
        %M memory: register k is at %MB3.(483328 + 2*(k-1)). Declared under
        the group that Hardware Configuration creates. See
        docs/master/ModbusRegisterMap.md.
        ======================================================================== *)

     G_mbHeartbeat         AT %MW3.483328 : UINT  (* HR1: master step counter *)
     G_mbModeCode          AT %MW3.483330 : UINT  (* HR2: mode code 0-7, or the full fault code mirrored during reset *)
     G_mbMotionEnable      AT %MW3.483332 : UINT  (* HR3: 1 = MotionEnable *)
     G_mbFaultReset        AT %MW3.483334 : UINT  (* HR4: 1 = FaultReset *)
     G_mbReference         AT %MD3.483336 : REAL  (* HR5-6: reference, mm / mm/s / % by mode *)


     (* ========================================================================
        MODBUS INPUT REGISTERS (slave -> master), group FC#04
        Register k is at byte offset 73792 + 2*(k-1).
        ======================================================================== *)

     G_mbHeartbeatEcho     AT %QW73792 : UINT  (* IR1: last G_mbHeartbeat received *)
     G_mbSlaveCounter      AT %QW73794 : UINT  (* IR2: scan counter *)
     G_mbConfCode          AT %QW73796 : UINT  (* IR3: DO0-2 value: mode confirmation or 3-bit fault code *)
     G_mbFaultCode         AT %QW73798 : UINT  (* IR4: G_sysActiveFault, full value *)
     G_mbFaultActive       AT %QW73800 : UINT  (* IR5: DO4 *)
     G_mbInMotion          AT %QW73802 : UINT  (* IR6: DO6 *)
     G_mbPerformanceStatus AT %QW73804 : UINT  (* IR7: DO3 *)
     G_mbHomingComplete    AT %QW73806 : UINT  (* IR8: DO5 *)
     G_mbActualPosition    AT %QD73812 : REAL  (* IR11-12: mm *)
     G_mbActualVelocity    AT %QD73816 : REAL  (* IR13-14: mm/s *)
     G_mbActualTorque      AT %QD73820 : REAL  (* IR15-16: % of rated *)
     G_mbCommandedPosition AT %QD73824 : REAL  (* IR17-18: mm *)

   END_VAR


   ============================================================================
   2. CONFIGURABLE PARAMETERS
   ============================================================================

   VAR_GLOBAL
     (* ========================================================================
        MECHANICAL CONFIGURATION
        Actuator: Tolomatic RSA64
        Motor: Yaskawa Sigma-7 400W (SGM7J-04A6A6C)
        ======================================================================== *)

     G_cfgGearRatio            : LREAL := 70.0        (* Motor turns per lead screw turn *)
     G_cfgLeadScrewPitch       : LREAL := 5.08        (* mm per lead screw turn (0.2 inch) *)


     (* ========================================================================
        POSITION LIMITS (mm)
        Positive direction: toward cylinder EOT (compression direction)
        ======================================================================== *)

     (* Soft limits - software enforced boundaries *)
     G_cfgPosSoftLimitMax      : LREAL := 305.0       (* Soft limit - extend direction *)
     G_cfgPosSoftLimitMin      : LREAL := -2.5        (* Soft limit - retract direction. Lives midway between
                                                          position 0 (the homed retract endpoint) and the
                                                          negative overtravel switch (at -G_cfgHomeLimRetractDist,
                                                          default -5.0 mm). Adjust if RetractDist changes so the
                                                          soft limit stays bracketed by 0 and the switch. *)

     (* Piston exit guard margin *)
     G_cfgPosExitGuardMargin   : LREAL := 20.0        (* Safety margin above cylinder entry (mm). Position
                                                          within this band of G_cfgPosCylinderEntry triggers
                                                          torque/velocity danger-zone monitoring. *)


     (* ========================================================================
        VELOCITY LIMITS (mm/s)
        ======================================================================== *)

     G_cfgVelLimitMax          : LREAL := 5.0         (* Maximum velocity limit *)
     G_cfgVelLimitNormal       : LREAL := 5.0         (* Normal operating velocity *)


     (* ========================================================================
        DIRECT CONTROL ACCEL/DECEL (mm/s^2)
        Slew limits applied by Y_DirectControl in Position/Velocity/Torque
        operating modes. Required nonzero for the FB to ramp to commanded
        Velocity/Position; zero values leave the servo unable to move.
        ======================================================================== *)

     G_cfgDirectCtrlAccel      : LREAL := 5.0         (* Accel for Y_DirectControl operating modes (mm/s^2) *)
     G_cfgDirectCtrlDecel      : LREAL := 5.0         (* Decel for Y_DirectControl operating modes (mm/s^2) *)


     (* ========================================================================
        POSITION COMMAND GATE (FB_PositionCommandGate, position modes only)
        Y_DirectControl adds no profiling in PositionMode, so the gate shapes
        the master's setpoint into a command the axis can follow: rate limited
        to stCurrentLimits.VelMax, accel limited to G_cfgPosGateAccelMax, and
        tethered to within G_cfgPosGateMaxDeviation of actual position.
        ======================================================================== *)

     G_cfgScanTime             : LREAL := 0.002       (* Task period (s). dt for the gate and the analog filter alpha.
                                                          Must match the cyclic task interval in the MWiec task
                                                          configuration (2 ms is the MP2600iec lower bound). *)
     G_cfgPosGateAccelMax      : LREAL := 5.0         (* Command accel limit (mm/s^2). Conservative start - raise on the
                                                          bench. A master ramp at v is followed with lag v^2/(2A):
                                                          2.5 mm at 5 mm/s with A=5, 0.25 mm with A=50. *)
     G_cfgPosGateMaxDeviation  : LREAL := 2.0         (* Tether window (mm). Command never leads actual by more than this
                                                          (stall / torque limit). Must be well below drive Pn520
                                                          (excessive deviation) and well above normal following error
                                                          (~VelMax/Kp). <= 0 disables the tether. *)


     (* ========================================================================
        TORQUE LIMITS (% of rated torque)
        100 = 100% rated, up to 800% peak
        ======================================================================== *)

     G_cfgTorqueLimitMax       : LREAL := 110.0       (* Maximum torque limit *)
     G_cfgTorqueLimitNormal    : LREAL := 100.0       (* Normal operating torque *)
     G_cfgTorqueStallThreshold : LREAL := 60.0        (* Torque threshold for piston exit guard *)


     (* ========================================================================
        TIMING PARAMETERS
        ======================================================================== *)

     G_cfgHandshakeTimeout     : TIME := T#500MS      (* Handshake timeout: the master must raise motion enable within this window after confirmation; it needs at least two Modbus exchanges *)
     G_cfgCommTimeout          : TIME := T#150MS      (* Master heartbeat unchanged this long raises FAULT_COMM *)
     G_cfgHoldPositionTimeout  : TIME := T#2M         (* Hold-position inactivity watchdog: if no handshake completes within this window after motion halts, gracefully shut down to idle (no fault) *)
     G_cfgBrakeEngageDelay     : TIME := T#500MS      (* Wait after brake engage command *)
     G_cfgBrakeDisengageDelay  : TIME := T#500MS      (* Wait after brake disengage command *)
     G_cfgHomingTimeout        : TIME := T#90S        (* Overall homing sequence timeout *)
     G_cfgDriveReadyTimeout    : TIME := T#5S         (* Timeout waiting for drive ready *)
     G_cfgFaultIdleTimeout     : TIME := T#30S        (* Timeout in fault before going to idle *)


     (* ========================================================================
        INPUT FILTERING PARAMETERS
        ======================================================================== *)

     (* Digital input debounce times *)
     G_cfgDebounceTimeMode         : TIME := T#50MS   (* Mode code (G_mbModeCode) debounce *)
     G_cfgDebounceTimeMotion       : TIME := T#20MS   (* Motion enable (G_mbMotionEnable) debounce *)
     G_cfgDebounceTimeFault        : TIME := T#20MS   (* Fault reset (G_mbFaultReset) debounce *)
     G_cfgDebounceTimeOvertravelNeg: TIME := T#10MS   (* Negative overtravel / home reference (short for homing precision) *)
     G_cfgDebounceTimeOvertravelPos: TIME := T#10MS   (* Positive overtravel standard debounce *)

     (* Analog input filter parameters (pressure channel) *)
     G_cfgAnalogFilterTimeConst: TIME := T#50MS       (* Low-pass filter time constant *)
     G_cfgAnalogMedianSize     : INT := 5             (* Median filter size (3 or 5) *)


     (* ========================================================================
        MOTION DETECTION THRESHOLDS
        ======================================================================== *)

     G_cfgInMotionVelThreshold : LREAL := 0.1         (* Velocity threshold for "in motion" (mm/s) *)


     (* ========================================================================
        HOMING PARAMETERS
        ======================================================================== *)

     (* Mode 110 - Home to Negative Overtravel Switch (two-stage approach + fixed retract) *)
     G_cfgHomeLimFastApproachVel : LREAL := 5.0       (* Coarse approach toward switch (mm/s) *)
     G_cfgHomeLimSlowApproachVel : LREAL := 1.0       (* Slow re-approach for precise switch detect (mm/s) *)
     G_cfgHomeLimBackoffVel      : LREAL := 5.0       (* Inter-approach backoff AND final retract velocity (mm/s) *)
     G_cfgHomeLimRetractDist     : LREAL := 5.0       (* Retract distance from precise switch to position 0 (mm).
                                                          Switch lands at -G_cfgHomeLimRetractDist in the homed frame. *)
     G_cfgHomeLimSetPosition     : LREAL := 0.0       (* Position value at the retract endpoint (becomes coord-frame zero) *)
     G_cfgHomeLimAccel           : LREAL := 5.0       (* Acceleration for all homing moves (mm/s²) *)
     G_cfgHomeLimDecel           : LREAL := 50.0       (* Deceleration for all homing moves (mm/s²) *)

     (* Mode 101 - Go Home *)
     G_cfgGoHomePosition       : LREAL := 0.0         (* Target position for Go Home *)
     G_cfgGoHomeVelocity       : LREAL := 5.0         (* Velocity for Go Home move *)
     G_cfgGoHomeAccel          : LREAL := 5.0         (* Acceleration for Go Home move (mm/s²) *)
     G_cfgGoHomeDecel          : LREAL := 5.0         (* Deceleration for Go Home move (mm/s²) *)
     G_cfgGoHomeStopDecel      : LREAL := 5.0         (* Deceleration for MC_Stop commands (mm/s²) *)
   END_VAR


   ============================================================================
   3. POSITION REFERENCES
   ============================================================================

   Coordinate System:
   - Single zero reference established by homing to the negative overtravel
     switch (Mode 110). Master and slave share this frame — every landmark
     below is defined relative to the switch position captured during homing.
   - Positive direction: toward cylinder EOT (compression direction, INTO cylinder)
   - Negative direction: toward exit (retract, OUT OF cylinder)
   - All positions in millimeters (mm)

   Key landmarks (positive = deeper into cylinder, all relative to the homed zero):
   - -G_cfgHomeLimRetractDist     : Negative overtravel switch position. Established
                                    by Mode 110: slow approach precisely re-detects
                                    the switch, then MC_SetPosition declares this
                                    point to be (SetPosition - RetractDist).
   - G_cfgPosSoftLimitMin (~-2.5) : Software floor; lives between zero and the
                                    switch, providing noise headroom while still
                                    catching real over-travel toward the switch.
   - 0 mm                         : Home / zero reference. Reached by MC_MoveAbsolute(
                                    SetPosition) at the end of Mode 110 — exactly
                                    G_cfgHomeLimRetractDist positive of the switch.
   - G_cfgPosCylinderEntry        : Physical cylinder-entry point. Below this the
                                    piston is leaving the cylinder bore; the exit
                                    guard monitors torque/velocity here.
   - G_cfgPosMapTransitionPos     : Analog-scaling stage boundary (200 mm).
   - G_cfgPosSoftLimitMax (305 mm): Software ceiling; end-of-stroke in compression
                                    direction.

   VAR_GLOBAL
     (* ========================================================================
        DEVICE DIMENSIONS (physical landmarks in the coordinate frame)
        ======================================================================== *)

     G_cfgPosCylinderEntry     : LREAL := 200.0       (* Physical cylinder-entry position (mm).
                                                          Below this the piston is leaving the
                                                          cylinder — exit guard fires regardless
                                                          of soft-limit value. Measure and set to
                                                          the actual mechanical entry point. *)


     (* ========================================================================
        DYNAMIC POSITION TRACKING
        ======================================================================== *)

     (* Position at mode entry (for reference) *)
     G_posModeEntryPosition    : LREAL := 0.0     (* Position when entering current mode *)

     (* Exit guard boundary *)
     G_posExitGuardBoundary    : LREAL := 0.0     (* Dynamic boundary for exit prevention *)
   END_VAR


   ============================================================================
   4. RUNTIME STATE VARIABLES
   ============================================================================

   VAR_GLOBAL
     (* ========================================================================
        STATE MACHINE
        ======================================================================== *)

     G_sysCurrentState         : E_SystemState := ST_INIT     (* Current state *)
     G_sysPreviousState        : E_SystemState := ST_INIT     (* Previous state *)
     G_sysRequestedMode        : E_OperatingMode := MODE_IDLE (* Mode from master *)
     G_sysReference            : LREAL := 0.0     (* Reference from master: mm, mm/s or % by mode *)
     G_sysScanCounter          : UINT := UINT#0   (* Incremented every scan, published in G_mbSlaveCounter *)
     G_sysActiveFaultCode      : UINT := UINT#0   (* G_sysActiveFault as an integer, published in G_mbFaultCode *)
     G_sysConfirmedMode        : E_OperatingMode := MODE_IDLE (* Mode confirmed to master *)
     G_sysActiveFault          : E_FaultCode := FAULT_NONE    (* Active fault code *)


     (* ========================================================================
        HOMING FLAGS
        ======================================================================== *)

     G_flagHomingInProgress    : BOOL := FALSE    (* TRUE while homing sequence active *)
     G_flagHomingComplete      : BOOL := FALSE    (* TRUE after successful homing *)


     (* ========================================================================
        MOTION FEEDBACK
        Current state of the servo motor/actuator
        ======================================================================== *)

     G_sysActualPosition       : LREAL := 0.0     (* Current position (mm) *)
     G_sysActualVelocity       : LREAL := 0.0     (* Current velocity (mm/s) *)
     G_sysActualTorque         : LREAL := 0.0     (* Current torque (% of rated) *)

     G_sysCommandedPosition    : LREAL := 0.0     (* Commanded position (mm) *)
     G_sysCommandedVelocity    : LREAL := 0.0     (* Commanded velocity (mm/s) *)
     G_sysCommandedTorque      : LREAL := 0.0     (* Commanded torque (%) *)


     (* ========================================================================
        STATUS FLAGS
        ======================================================================== *)

     (* Drive status *)
     G_sysDriveReady           : BOOL := FALSE    (* Drive is ready for motion *)
     G_sysDriveEnabled         : BOOL := FALSE    (* Drive power is enabled *)
     G_sysDriveFault           : BOOL := FALSE    (* Drive has reported a fault *)

     (* Motion status *)
     G_sysMotionActive         : BOOL := FALSE    (* Motion command is active *)
     G_sysInPosition           : BOOL := FALSE    (* At commanded position (position mode) *)
     G_sysAtVelocity           : BOOL := FALSE    (* At commanded velocity (velocity mode) *)
     G_sysHalted               : BOOL := FALSE    (* Halt is active (ST_HOLD_POSITION) *)

     (* Limit status *)
     G_sysAtPosLimitHigh       : BOOL := FALSE    (* At or beyond high position limit *)
     G_sysAtPosLimitLow        : BOOL := FALSE    (* At or beyond low position limit *)
     G_sysOvertravelNegActive  : BOOL := FALSE    (* Negative overtravel switch triggered *)
     G_sysOvertravelPosActive  : BOOL := FALSE    (* Positive overtravel switch triggered *)

     (* Safety status *)
     G_sysExitGuardActive      : BOOL := FALSE    (* Piston exit prevention active *)
     G_sysInCylinderZone       : BOOL := FALSE    (* Piston is inside cylinder *)

     (* Brake status *)
     G_sysBrakeEngaged         : BOOL := TRUE     (* Brake is engaged (spring-applied) *)


     (* ========================================================================
        DIAGNOSTICS
        ======================================================================== *)

     G_sysCycleCount           : UDINT := 0       (* Main program cycle counter *)
     G_sysLastFaultCode        : E_FaultCode := FAULT_NONE    (* Last fault that occurred *)
     G_sysLastFaultState       : E_SystemState := ST_INIT     (* State when last fault occurred *)
     G_sysFaultCount           : UDINT := 0       (* Total number of faults since power-up *)

     (* Timing diagnostics *)
     G_sysStateEntryTime       : TIME := T#0MS    (* Time when current state was entered *)
     G_sysTimeInState          : TIME := T#0MS    (* Duration in current state *)
   END_VAR


   ============================================================================
   5. COMMAND / STATUS INTERFACE (ST <-> LD POU)
   ============================================================================

   These structured globals connect the ST state machine (PRG_Main) to the
   Ladder Logic POU that owns G_sysAxis and all built-in motion FBs.

   Commands flow ST -> LD (written by PRG_Main, read by LD POU).
   Status flows LD -> ST (written by LD POU, read by PRG_Main and custom FBs).

   VAR_GLOBAL
     (* ========================================================================
        COMMAND STRUCTS (ST -> LD)
        ======================================================================== *)

     G_cmdPower            : ST_CmdPower
     G_cmdStop             : ST_CmdStop
     G_cmdReset            : ST_CmdReset
     G_cmdMoveAbsolute     : ST_CmdMoveAbsolute
     G_cmdMoveVelocity     : ST_CmdMoveVelocity     (* unused after Jog conversion; retained for future use *)
     G_cmdJog              : ST_CmdJog
     G_cmdDirectControl    : ST_CmdDirectControl
     G_cmdSetPosition      : ST_CmdSetPosition


     (* ========================================================================
        STATUS STRUCTS (LD -> ST)
        ======================================================================== *)

     G_staPower            : ST_StaPower
     G_staStop             : ST_StaStop
     G_staReset            : ST_StaReset
     G_staMoveAbsolute     : ST_StaMoveAbsolute
     G_staMoveVelocity     : ST_StaMoveVelocity     (* unused after Jog conversion; retained for future use *)
     G_staJog              : ST_StaJog
     G_staDirectControl    : ST_StaDirectControl
     G_staSetPosition      : ST_StaSetPosition
   END_VAR

*)
