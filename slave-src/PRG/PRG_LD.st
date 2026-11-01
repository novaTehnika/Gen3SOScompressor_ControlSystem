(* ============================================================================
   PRG_LD - Ladder Diagram POU: the axis and the built-in motion FBs

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   The one POU that owns the axis and instantiates the built-in motion
   function blocks (PLCopen MC_*, Yaskawa Y_DirectControl, the PLCopen
   Toolbox Jog). The Structured Text never calls them: PRG_Main writes the
   G_cmd* structured globals every scan and reads the G_sta* globals and
   the G_sysActual* feedback (types in DUT/DataTypes.st). Each network of
   this POU is one block instance whose pins are wired to the fields of
   the same name, so the structs are the pin lists.

   The POU is written in Ladder Diagram in MotionWorks IEC and has no text
   form. This file is its specification: the header declares its
   variables as the headers of the ST POUs do, and the body below lists
   every network with the wiring of each pin. The screenshots in
   docs/slave/ld/ record the wiring as it is in the project; a change to a
   field of ST_Cmd* or ST_Sta* is complete only when the struct, this
   file, the LD and the screenshot agree.

   Integration:
   - Runs on the one cyclic task, last, after PRG_Output: the status
     PRG_Main reads is one scan old, which is the order the simulator runs
     (PRG_Input, PRG_Main, PRG_Output, axis model) and the tests assume.
   - PRG_Main defaults every command to idle at the top of its command
     arbitration; the owning state, FB_GoHome, FB_HomeLimit or
     FB_PositionPathShaper overwrites what it commands. A dropped
     Execute/Forward/Reverse does not stop a block, so the rate fields
     keep real values while the enable is FALSE.
   - Units are the axis's engineering units from Hardware Configuration:
     mm, mm/s, mm/s^2 and % of rated torque. The ST and the protocol scale
     nothing. Rates are positive.
   - Jerk and BufferMode are left open on every block (no jerk limit,
     aborting). ErrorID outputs are left open except Jog's, the one the ST
     carries; wire the others to locals for diagnosis if wanted.
   - Every G_cmd* field has a pin and every wired G_sta* field has a pin:
     the structs hold nothing the POU does not use.
   - In the simulator, slave-sim/c/slave_sim.c stands in for this POU.

   Variables (define in MWiec GUI):
     VAR_EXTERNAL:
       G_sysAxis           : AXIS_REF         (* The axis, created by Hardware Configuration (AXIS1 in the project)
     VAR:
       (* Built-in motion FB instances, one network each
       fbPower             : MC_Power         (* Servo power on and off
       fbReset             : MC_Reset         (* Clears a drive alarm (ST_DRIVE_RESET)
       fbStop              : MC_Stop          (* Controlled stop of hold, fault and recovery; aborts fbDirectControl
       fbMoveAbsolute      : MC_MoveAbsolute  (* Go Home move and the homing retract
       fbJog               : Jog              (* Homing approach and back-off (PLCopen Toolbox)
       fbSetPosition       : MC_SetPosition   (* Sets the homed zero
       fbDirectControl     : Y_DirectControl  (* Position, velocity and torque command of the operating modes
       fbReadActualPosition : MC_ReadActualPosition  (* Position feedback
       fbReadActualVelocity : MC_ReadActualVelocity  (* Velocity feedback
       fbReadActualTorque  : MC_ReadActualTorque     (* Torque feedback
       fbReadStatus        : MC_ReadStatus    (* Drive alarm flag for FB_SafetyMonitor
   ============================================================================ *)

(* ============================================================================
   NETWORKS
   One block per network, in task order. "<-" is a pin wired from a
   variable, "->" a pin wired to one. A pin not listed is left open.
   ============================================================================ *)

(* --------------------------------------------------------------------------
   fbPower : MC_Power - Servo power on and off
   PRG_Main sets the three enables TRUE together in every drive-active
   state and, in ST_FAULT_IDLE, until the brake has engaged.
     Axis            <- G_sysAxis
     Enable          <- G_cmdPower.Enable
     Enable_Positive <- G_cmdPower.EnablePositive
     Enable_Negative <- G_cmdPower.EnableNegative
     Axis            -> G_sysAxis
     Status          -> G_staPower.Status    (* ST_DRIVE_ENABLE waits for it; ST_HOLD_POSITION and
                                                ST_FAULT_IDLE take it dropping as the drive going off
     Error           -> G_staPower.Error     (* FAULT_DRIVE in ST_DRIVE_ENABLE
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbReset : MC_Reset - Drive alarm reset
   Execute is held TRUE for the whole of ST_DRIVE_RESET.
     Axis            <- G_sysAxis
     Execute         <- G_cmdReset.Execute
     Axis            -> G_sysAxis
     Done            -> G_staReset.Done      (* ST_DRIVE_RESET -> ST_DRIVE_ENABLE
     Busy            -> G_staReset.Busy      (* Not read by the ST
     Error           -> G_staReset.Error     (* FAULT_DRIVE in ST_DRIVE_RESET
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbStop : MC_Stop - Controlled stop
   Execute is TRUE in ST_HOLD_POSITION until the axis is halted and in
   ST_FAULT and ST_RECOVERY until Done; FB_GoHome executes it to abort the
   Go Home move. Deceleration is G_cfgStopDecel in every case. While it is
   Busy and fbDirectControl.Active has dropped, PRG_Main releases the
   Y_DirectControl hand-off (see fbDirectControl).
     Axis            <- G_sysAxis
     Execute         <- G_cmdStop.Execute
     Deceleration    <- G_cmdStop.Deceleration   (* mm/s^2
     Axis            -> G_sysAxis
     Done            -> G_staStop.Done       (* Ends the stop in ST_HOLD_POSITION, ST_FAULT, ST_RECOVERY, FB_GoHome
     Busy            -> G_staStop.Busy       (* Hand-off release, with fbDirectControl Active FALSE
     Error           -> G_staStop.Error      (* Also releases the hand-off
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbMoveAbsolute : MC_MoveAbsolute - Go Home move and the homing retract
   Commanded by FB_GoHome (CmdMoveAbsolute) and FB_HomeLimit; PRG_Main
   copies the owning block's command struct whole. Outside those states
   Execute is FALSE and the rates keep the Go Home values, so a move
   interrupted by a hold or fault keeps real rates until fbStop takes the
   axis. The ST sends Direction = Current_Direction.
     Axis            <- G_sysAxis
     Execute         <- G_cmdMoveAbsolute.Execute
     Position        <- G_cmdMoveAbsolute.Position        (* mm, absolute
     Velocity        <- G_cmdMoveAbsolute.Velocity        (* mm/s
     Acceleration    <- G_cmdMoveAbsolute.Acceleration    (* mm/s^2
     Deceleration    <- G_cmdMoveAbsolute.Deceleration    (* mm/s^2
     Direction       <- G_cmdMoveAbsolute.Direction       (* MC_Direction
     Axis            -> G_sysAxis
     Done            -> G_staMoveAbsolute.Done
     Busy            -> G_staMoveAbsolute.Busy
     Active          -> G_staMoveAbsolute.Active
     CommandAborted  -> G_staMoveAbsolute.CommandAborted
     Error           -> G_staMoveAbsolute.Error
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbJog : Jog - Homing approach and back-off
   FB_HomeLimit is the only user: fast and slow approach to the negative
   overtravel switch and the back-off. Outside homing Forward and Reverse
   are FALSE and the rates keep the homing values, so an abort into
   ST_HOLD_POSITION lets the block decelerate on its own Deceleration.
     Axis            <- G_sysAxis
     Forward         <- G_cmdJog.Forward
     Reverse         <- G_cmdJog.Reverse
     Velocity        <- G_cmdJog.Velocity          (* mm/s
     Acceleration    <- G_cmdJog.Acceleration      (* mm/s^2
     Deceleration    <- G_cmdJog.Deceleration      (* mm/s^2
     Axis            -> G_sysAxis
     InVelocity      -> G_staJog.InVelocity
     Done            -> G_staJog.Done        (* One-scan pulse once stopped after both inputs FALSE
     Busy            -> G_staJog.Busy
     Error           -> G_staJog.Error
     ErrorID         -> G_staJog.ErrorID     (* UINT
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbSetPosition : MC_SetPosition - Sets the homed zero
   FB_HomeLimit executes it once at the end of homing with
   G_cfgHomeLimSetPosition. The ST sends Mode = FALSE (absolute).
     Axis            <- G_sysAxis
     Execute         <- G_cmdSetPosition.Execute
     Position        <- G_cmdSetPosition.Position    (* mm
     Mode            <- G_cmdSetPosition.Mode        (* FALSE absolute, TRUE relative
     Axis            -> G_sysAxis
     Done            -> G_staSetPosition.Done
     Busy            -> G_staSetPosition.Busy
     Error           -> G_staSetPosition.Error
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbDirectControl : Y_DirectControl - Command of the operating modes
   Enable is TRUE in the position, velocity, torque and pressure states,
   and is held TRUE with a zero command through the stop of
   ST_HOLD_POSITION, ST_FAULT and ST_RECOVERY until fbStop has aborted the
   block (G_staStop.Busy with Active FALSE; Done and Error are fallbacks),
   which is what returns the SERVOPACK from velocity or torque control to
   position control. In PositionMode the block does no path shaping, so
   Velocity and Acceleration set no limit there: FB_PositionPathShaper
   shapes the command and the block is given the shaper's speed. The
   block has no Deceleration pin.
     Axis            <- G_sysAxis
     Enable          <- G_cmdDirectControl.Enable
     ControlMode     <- G_cmdDirectControl.ControlMode   (* Y_ControlMode: PositionMode, VelocityTLMode, TorqueVLMode
     Position        <- G_cmdDirectControl.Position      (* mm; the shaper's command in position mode
     Velocity        <- G_cmdDirectControl.Velocity      (* mm/s; the command in velocity and pressure modes,
                                                            the limit in torque mode, the shaper's speed in position mode
     Acceleration    <- G_cmdDirectControl.Acceleration  (* mm/s^2; G_cfgDirectCtrlAccel, G_cfgPressureAccel in ST_PRESSURE_CTRL
     Torque          <- G_cmdDirectControl.Torque        (* % of rated; the command in torque mode, the limit otherwise
     FilterTimeConstant <- G_cmdDirectControl.FilterTimeConstant  (* s, G_cfgDirectCtrlFilterTime
     Axis            -> G_sysAxis
     Active          -> G_staDirectControl.Active   (* The hand-off waits for it to drop
     Error           -> G_staDirectControl.Error    (* Releases the hand-off
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbReadActualPosition : MC_ReadActualPosition - Position feedback
   fbReadActualVelocity : MC_ReadActualVelocity - Velocity feedback
   fbReadActualTorque   : MC_ReadActualTorque   - Torque feedback
   Enable is the left power rail, so each reads every scan. The outputs
   are the source of every position, velocity and torque the ST uses: the
   limit and exit-guard checks, the path shaper's seed and tether, the
   in-motion output, homing, the step probe record and the Modbus
   feedback registers.
     Axis            <- G_sysAxis
     Enable          <- TRUE (power rail)
     Axis            -> G_sysAxis
     Position        -> G_sysActualPosition   (* mm, fbReadActualPosition
     ActualVelocity  -> G_sysActualVelocity   (* mm/s, fbReadActualVelocity
     ActualTorque    -> G_sysActualTorque     (* % of rated, fbReadActualTorque
   -------------------------------------------------------------------------- *)

(* --------------------------------------------------------------------------
   fbReadStatus : MC_ReadStatus - Drive alarm flag
   G_sysDriveFault is the DriveFault input of FB_SafetyMonitor
   (FAULT_DRIVE outside the reset and enable states, the path the fault
   code reference gives for the amplifier's own alarms, encoder alarms
   included). Nothing else writes it. The axis is in ErrorStop whenever
   the SERVOPACK holds an alarm and leaves it on fbReset, which is what
   ST_DRIVE_RESET and the recovery expect of the flag. Enable is the left
   power rail, as for the feedback readers. The simulator sets the flag
   directly (sl.hook.driveFault).
     Axis            <- G_sysAxis
     Enable          <- TRUE (power rail)
     Axis            -> G_sysAxis
     ErrorStop       -> G_sysDriveFault
   -------------------------------------------------------------------------- *)
