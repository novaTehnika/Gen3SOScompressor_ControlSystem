(* ============================================================================
   DataTypes - All Data Type Definitions

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Combined data type definitions for the servo controller project.
   Contains all enumerations and structures used throughout the codebase.

   Contents:
   - E_OperatingMode: Operating mode enumeration (Modbus mode code)
   - E_FaultCode: Fault code enumeration (Modbus fault code)
   - E_SystemState: System state machine enumeration
   - E_ProbeState, E_ProbeXferState: step probe recording and transfer states
   - ST_AxisLimits: Position, velocity, torque, accel/decel limits
   - ST_HomingParams: Limit-switch homing parameters
   - ST_Cmd*/ST_Sta*: Command/status struct pairs bridging ST and LD POUs
   ============================================================================ *)

TYPE

   (* ============================================================================
      E_OperatingMode - Operating Mode Enumeration

      The integer value is the mode code in the Modbus holding register
      G_mbModeCode and the confirmation in the input register G_mbConfCode.
      ============================================================================ *)

   (* IMPORTANT: Member order defines implicit integer values (0-7).
      These values are the Modbus mode codes. Do not reorder.
      MODE_PRESSURE closes the slave's pressure loop on the transducer wired to
      the analog input; its reference is the target pressure in MPa gauge. *)
   E_OperatingMode :
   (
      MODE_IDLE,               (* implicit 0 - 000: Idle - Drive OFF, brake engaged *)
      MODE_BRAKE_HOLD,         (* implicit 1 - 001: Brake Hold - Drive ON, brake engaged *)
      MODE_POSITION,           (* implicit 2 - 010: Position Control - reference in mm *)
      MODE_VELOCITY,           (* implicit 3 - 011: Velocity Control - reference in mm/s *)
      MODE_TORQUE,             (* implicit 4 - 100: Torque Control - reference in % rated *)
      MODE_GO_HOME,            (* implicit 5 - 101: Go Home - Move to home position *)
      MODE_HOME_LIMIT,         (* implicit 6 - 110: Home to Negative Overtravel Switch (reference) *)
      MODE_PRESSURE            (* implicit 7 - 111: Pressure Control - reference is target MPa gauge *)
   );


   (* ============================================================================
      E_FaultCode - Fault Code Enumeration

      The full integer value is published in the Modbus input register
      G_mbFaultCode and mirrored back by the master in G_mbModeCode during
      the fault reset handshake.
      ============================================================================ *)

   (* IMPORTANT: Member order defines implicit integer values. Do not reorder.
      Values above 7 are carried by the Modbus registers only. *)
   E_FaultCode :
   (
      FAULT_NONE,              (* implicit 0 - 000: No fault / fault cleared *)
      FAULT_HANDSHAKE,         (* implicit 1 - 001: Handshake timeout or mismatch *)
      FAULT_DRIVE,             (* implicit 2 - 010: Drive fault (from servo amplifier) *)
      FAULT_POSITION,          (* implicit 3 - 011: Position limit exceeded *)
      FAULT_COMM,              (* implicit 4 - 100: Master heartbeat stopped (link lost) *)
      FAULT_PISTON_EXIT,       (* implicit 5 - 101: Piston exit prevention triggered *)
      FAULT_LIMIT_SWITCH,      (* implicit 6 - 110: Limit switch fault *)
      FAULT_ENCODER,           (* implicit 7 - 111: Encoder / absolute position fault *)
      FAULT_PRESSURE_SENSOR    (* implicit 8 - registers only: transducer loop current out of range in pressure mode *)
   );


   (* ============================================================================
      E_SystemState - System State Enumeration

      All possible states in the main state machine. States are grouped by
      function: Initialization, Idle, Transitions, Operational, and Fault states.
      ============================================================================ *)

   (* NOTE: Implicit integer values do not matter for this enum.
      All code references states by name, never by numeric value.
      States are grouped logically below for readability. *)
   E_SystemState :
   (
      (* Initialization State *)
      ST_INIT,                 (* Default initializer; first scan transitions directly to ST_IDLE *)

      (* Idle State *)
      ST_IDLE,                 (* Drive OFF, brake engaged, awaiting commands *)

      (* Transition States - Activation *)
      ST_DRIVE_RESET,          (* Issue MC_Reset to clear drive alarms before power-on *)
      ST_DRIVE_ENABLE,         (* Power on drive, wait for ready *)
      ST_BRAKE_RELEASE,        (* Release brake, wait for mechanical delay *)

      (* Operational States *)
      ST_BRAKE_HOLD,           (* Mode 001: Drive ON, brake engaged *)
      ST_POSITION_CTRL,        (* Mode 010: Position control active *)
      ST_VELOCITY_CTRL,        (* Mode 011: Velocity control active *)
      ST_TORQUE_CTRL,          (* Mode 100: Torque control active *)
      ST_PRESSURE_CTRL,        (* Mode 111: Pressure control active - velocity command from the pressure loop *)
      ST_GO_HOME,              (* Mode 101: Moving to home position *)

      (* Homing States *)
      ST_HOME_LIMIT,           (* Mode 110: Homing to negative overtravel switch - uses FB_HomeLimit *)
      ST_HOME_COMPLETE,        (* Homing finished, awaiting mode change *)

      (* Transition State - Inter-Mode *)
      ST_HOLD_POSITION,        (* MC_Stop active, brake disengaged, awaiting handshake *)

      (* Transition States - Deactivation *)
      ST_BRAKE_ENGAGE,         (* Engage brake, wait for mechanical delay *)
      ST_DRIVE_DISABLE,        (* Power off drive, return to idle *)

      (* Fault States *)
      ST_FAULT,                (* Fault active, no brake, MC_Stop, await handshake/timeout *)
      ST_FAULT_IDLE,           (* Drive disabled, brake engaged, await fault reset handshake *)

      (* Recovery States - jog off a still-active limit switch / soft-limit overshoot.
         Entered from ST_FAULT (drive stays energized, brake released). *)
      ST_RECOVERY,             (* Armed recovery hub: holds position via MC_Stop; select MODE_POSITION/MODE_VELOCITY to retract *)
      ST_RECOVERY_POSITION,    (* Position-control retract; target clamped away from the active limit *)
      ST_RECOVERY_VELOCITY     (* Velocity-control retract; command zeroed if it drives into the active limit *)
   );


   (* ============================================================================
      E_HomeLimitState - FB_HomeLimit Internal State Enumeration

      Sub-state machine inside FB_HomeLimit (Mode 110 homing sequence).
      ============================================================================ *)

   (* NOTE: Implicit integer values do not matter for this enum.
      All code references states by name, never by numeric value. *)
   E_HomeLimitState :
   (
      IDLE,                    (* Waiting for execute command *)
      FAST_APPROACH,           (* Jog Reverse @ FastApproachVel toward switch *)
      FAST_AWAIT,              (* Drop Reverse on switch trip; wait for Jog.Done *)
      INTER_BACKOFF,           (* Jog Forward @ BackoffVel until switch clears *)
      INTER_AWAIT,             (* Drop Forward; wait for Jog.Done *)
      SLOW_APPROACH,           (* Jog Reverse @ SlowApproachVel for precise re-detect; capture rSwitchPos *)
      SLOW_AWAIT,              (* Drop Reverse on switch trip; wait for Jog.Done *)
      RETRACT,                 (* MC_MoveAbsolute(rSwitchPos + RetractDist) — backs the axis off the switch *)
      RETRACT_AWAIT,           (* Wait for MC_MoveAbsolute.Done *)
      SETREF,                  (* MC_SetPosition(SetPosition) — declares retract endpoint as zero *)
      DONE,                    (* Homing sequence complete *)
      ABORTING,                (* Abort requested; wait for whichever motion FB is busy *)
      ERROR                    (* Error latched; await Execute clear *)
   );


   (* ============================================================================
      E_ProbeState - FB_Probe Recording State

      State of the step probe's recorder. The code published in the input
      register G_mbProbeState is the member's position below (0-3).
      ============================================================================ *)

   (* IMPORTANT: Member order defines the published code. Do not reorder. *)
   E_ProbeState :
   (
      PROBE_OFF,               (* implicit 0: disabled by G_cfgProbeEnable, nothing recorded *)
      PROBE_RECORDING,         (* implicit 1: ring running, no armed capture open *)
      PROBE_CAPTURING,         (* implicit 2: an armed capture whose last record is not written yet (or is open-ended) *)
      PROBE_HELD               (* implicit 3: the armed capture is fully recorded and still entirely in the ring *)
   );


   (* ============================================================================
      E_ProbeXferState - FB_Probe Transfer State

      State of the window the master reads. The code published in the input
      register G_mbProbeXfer is the member's position below (0-3).
      ============================================================================ *)

   (* IMPORTANT: Member order defines the published code. Do not reorder. *)
   E_ProbeXferState :
   (
      XFER_IDLE,               (* implicit 0: the window is complete for the request *)
      XFER_FILLING,            (* implicit 1: copying the requested records into the window, a few per scan *)
      XFER_PENDING,            (* implicit 2: the requested records are not recorded yet *)
      XFER_OVERRUN             (* implicit 3: the requested start has been overwritten in the ring *)
   );


   (* ============================================================================
      ST_AxisLimits - Axis Limits Structure

      Structure containing position, velocity, and torque limits.
      Used for mode-dependent limit configuration.
      ============================================================================ *)

   ST_AxisLimits :
   STRUCT
      (* Position limits (mm) *)
      PosMax          : LREAL;    (* Maximum position limit *)
      PosMin          : LREAL;    (* Minimum position limit *)

      (* Velocity limits (mm/s) *)
      VelMax          : LREAL;    (* Maximum velocity limit *)
      VelMin          : LREAL;    (* Minimum velocity limit (usually negative of VelMax) *)

      (* Torque limits (% of rated) *)
      TorqueMax       : LREAL;    (* Maximum torque limit (positive direction) *)
      TorqueMin       : LREAL;    (* Minimum torque limit (negative direction) *)

      (* Acceleration/Deceleration limits (mm/s^2) *)
      AccelMax        : LREAL;    (* Maximum acceleration *)
      DecelMax        : LREAL;    (* Maximum deceleration *)
   END_STRUCT;


   (* ============================================================================
      ST_HomingParams - Homing Parameters Structure

      Structure containing parameters for limit-switch homing (Mode 110).
      ============================================================================ *)

   ST_HomingParams :
   STRUCT
      (* Approach velocities (mm/s) *)
      VelFast         : LREAL;    (* Fast approach velocity *)
      VelSlow         : LREAL;    (* Slow backoff velocity *)

      (* Distance parameters (mm) *)
      BackoffDist     : LREAL;    (* Distance to back off after detection *)

      (* Position to set after homing (mm) *)
      SetPosition     : LREAL;    (* Position value to assign at home point *)

      (* Timing *)
      Timeout         : TIME;     (* Overall homing sequence timeout *)

      (* Direction *)
      Direction       : INT;      (* Homing direction: 1 = positive, -1 = negative *)
   END_STRUCT;

   (* ============================================================================
      Command Structs (ST -> LD POU)

      These structs carry commands from the Structured Text state machine to
      the Ladder Logic POU that owns G_sysAxis and all built-in motion FBs.
      Field names mirror the target built-in FB pin names for 1:1 mapping.
      ============================================================================ *)

   ST_CmdPower :
   STRUCT
      Enable          : BOOL;
      EnablePositive   : BOOL;
      EnableNegative   : BOOL;
   END_STRUCT;

   ST_CmdStop :
   STRUCT
      Execute         : BOOL;
      Deceleration    : LREAL;
   END_STRUCT;

   ST_CmdReset :
   STRUCT
      Execute         : BOOL;
   END_STRUCT;

   ST_CmdMoveAbsolute :
   STRUCT
      Execute         : BOOL;
      Position        : LREAL;
      Velocity        : LREAL;
      Acceleration    : LREAL;
      Deceleration    : LREAL;
      Direction       : MC_Direction;
   END_STRUCT;

   ST_CmdJog :
   STRUCT
      Forward         : BOOL;
      Reverse         : BOOL;
      Velocity        : LREAL;
      Acceleration    : LREAL;
      Deceleration    : LREAL;
   END_STRUCT;

   ST_CmdDirectControl :
   STRUCT
      Enable          : BOOL;
      ControlMode     : Y_ControlMode;
      Position        : LREAL;
      Velocity        : LREAL;
      Torque          : LREAL;
      Acceleration    : LREAL;
      FilterTimeConstant : LREAL;   (* s, the block's FilterTimeConstant input *)
   END_STRUCT;

   ST_CmdSetPosition :
   STRUCT
      Execute         : BOOL;
      Position        : LREAL;
      Mode            : BOOL;       (* FALSE = absolute (default), TRUE = relative *)
   END_STRUCT;


   (* ============================================================================
      Status Structs (LD POU -> ST)

      These structs carry status from the Ladder Logic POU back to the
      Structured Text state machine. Field names mirror the built-in FB
      output pin names.
      ============================================================================ *)

   ST_StaPower :
   STRUCT
      Status          : BOOL;
      Error           : BOOL;
   END_STRUCT;

   ST_StaStop :
   STRUCT
      Done            : BOOL;
      Busy            : BOOL;
      Error           : BOOL;
   END_STRUCT;

   ST_StaReset :
   STRUCT
      Done            : BOOL;
      Busy            : BOOL;
      Error           : BOOL;
   END_STRUCT;

   ST_StaMoveAbsolute :
   STRUCT
      Done            : BOOL;
      Busy            : BOOL;
      Active          : BOOL;
      CommandAborted  : BOOL;
      Error           : BOOL;
   END_STRUCT;

   ST_StaJog :
   STRUCT
      InVelocity      : BOOL;
      Done            : BOOL;      (* one-scan pulse when stopped after both inputs FALSE *)
      Busy            : BOOL;
      Error           : BOOL;
      ErrorID         : UINT;
   END_STRUCT;

   ST_StaDirectControl :
   STRUCT
      Active          : BOOL;
      Error           : BOOL;
   END_STRUCT;

   ST_StaSetPosition :
   STRUCT
      Done            : BOOL;
      Busy            : BOOL;
      Error           : BOOL;
   END_STRUCT;

    (* ============================================================================
       ARRAY5LREAL - Array of 5 LREALs
       ============================================================================ *)
   ARRAY5LREAL : ARRAY[0..4] OF LREAL;
   ARRAY250UINT : ARRAY[0..249] OF UINT;
   ARRAY500INT : ARRAY[0..499] OF INT;
   ARRAY500DINT : ARRAY[0..499] OF DINT;
   ARRAY104WORD : ARRAY[0..103] OF WORD;
   ARRAY32768WORD : ARRAY[0..32767] OF WORD;
END_TYPE
