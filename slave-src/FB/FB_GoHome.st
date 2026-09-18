(* ============================================================================
   FB_GoHome - Go Home Mode Handler (Mode 101)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Handles Mode 101 (Go Home) which moves the axis to the home position.
   Checks that homing has been performed before executing; if not, requests
   that PRG_Main redirect to Mode 110.

   Behavior:
   - If HomingRequired is TRUE: Set RedirectToHoming, PRG_Main switches to 110
   - If requirements met: Command MC_MoveAbsolute via CmdMoveAbsolute output
   - After reaching position: Hold there until mode change

   Integration:
   - Called once per scan at end of PRG_Main
   - CmdMoveAbsolute/CmdStop routed to G_cmd* by state machine when active
   - Receives built-in FB status via StaMoveAbsolute/StaStop inputs
   - PRG_Main routes G_staMoveAbsolute/G_staStop into these inputs

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL                  (* Enable the function block
       Execute             : BOOL                  (* Start go home sequence
       AbsHomeRequired     : BOOL                  (* Homing required (blocks Go Home, bind G_flagHomingRequired)
       HomePosition        : LREAL                 (* Target home position (mm)
       HomeVelocity        : LREAL                 (* Movement velocity (mm/s)
       Acceleration        : LREAL                 (* Move acceleration (mm/s²)
       Deceleration        : LREAL                 (* Move/stop deceleration (mm/s²)
       Abort               : BOOL                  (* Abort the move
       StaMoveAbsolute     : ST_StaMoveAbsolute    (* Status from MC_MoveAbsolute
       StaStop             : ST_StaStop            (* Status from MC_Stop
     VAR_OUTPUT:
       Active              : BOOL             (* Sequence is running
       Done                : BOOL             (* At home position
       AtPosition          : BOOL             (* Currently at home position
       HomingRequired      : BOOL             (* Homing must be done first
       RedirectToHoming    : BOOL             (* Caller should switch to homing mode
       InMotion            : BOOL             (* Currently moving
       Error               : BOOL             (* Error occurred
       CmdMoveAbsolute     : ST_CmdMoveAbsolute  (* Command output for MC_MoveAbsolute
       CmdStop             : ST_CmdStop          (* Command output for MC_Stop
     VAR:
       eState              : INT := 0         (* State machine
       rTrigExecute        : R_TRIG
       rPosTolerance       : LREAL := 0.1     (* mm
     VAR CONSTANT:
       GOH_IDLE            : INT := 0
       GOH_CHECK_REQ       : INT := 1
       GOH_REDIRECT        : INT := 2
       GOH_MOVING          : INT := 3
       GOH_AT_POSITION     : INT := 4
       GOH_ABORTING        : INT := 5
       GOH_ERROR           : INT := 6
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    Active := FALSE;
    Done := FALSE;
    AtPosition := FALSE;
    HomingRequired := FALSE;
    RedirectToHoming := FALSE;
    InMotion := FALSE;
    Error := FALSE;
    eState := GOH_IDLE;
    CmdMoveAbsolute.Execute := FALSE;
    CmdStop.Execute := FALSE;
    RETURN;
END_IF;

(* Detect rising edge on Execute *)
rTrigExecute(CLK := Execute);

(* Handle abort in any state *)
IF Abort AND (eState <> GOH_IDLE) AND (eState <> GOH_ERROR) AND (eState <> GOH_REDIRECT) THEN
    CmdMoveAbsolute.Execute := FALSE;
    eState := GOH_ABORTING;
END_IF;

(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

IF eState = GOH_IDLE THEN

    (* GOH_IDLE - Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        AtPosition := FALSE;
        HomingRequired := FALSE;
        RedirectToHoming := FALSE;
        InMotion := FALSE;
        Error := FALSE;

        CmdMoveAbsolute.Execute := FALSE;
        CmdStop.Execute := FALSE;

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := GOH_CHECK_REQ;
        END_IF;


ELSIF eState = GOH_CHECK_REQ THEN

    (* GOH_CHECK_REQ - Check homing requirements *)

        IF AbsHomeRequired THEN
            HomingRequired := TRUE;
            RedirectToHoming := TRUE;
            eState := GOH_REDIRECT;

        ELSE
            HomingRequired := FALSE;
            RedirectToHoming := FALSE;
            eState := GOH_MOVING;
        END_IF;


ELSIF eState = GOH_REDIRECT THEN

    (* GOH_REDIRECT - Waiting for PRG_Main to handle redirect *)
        Active := TRUE;
        HomingRequired := TRUE;
        RedirectToHoming := TRUE;


ELSIF eState = GOH_MOVING THEN

    (* GOH_MOVING - Command move to home position *)
        CmdMoveAbsolute.Execute := TRUE;
        CmdMoveAbsolute.Position := HomePosition;
        CmdMoveAbsolute.Velocity := HomeVelocity;
        CmdMoveAbsolute.Acceleration := Acceleration;
        CmdMoveAbsolute.Deceleration := Deceleration;
        CmdMoveAbsolute.Direction := MC_Direction#Current_Direction;

        InMotion := StaMoveAbsolute.Busy;

        IF StaMoveAbsolute.Done THEN
            CmdMoveAbsolute.Execute := FALSE;
            Done := TRUE;
            AtPosition := TRUE;
            InMotion := FALSE;
            eState := GOH_AT_POSITION;

        ELSIF StaMoveAbsolute.Error THEN
            Error := TRUE;
            CmdMoveAbsolute.Execute := FALSE;
            eState := GOH_ERROR;

        ELSIF StaMoveAbsolute.CommandAborted THEN
            CmdMoveAbsolute.Execute := FALSE;
            eState := GOH_ABORTING;
        END_IF;


ELSIF eState = GOH_AT_POSITION THEN

    (* GOH_AT_POSITION - At home position - hold here until mode change *)
        Done := TRUE;
        AtPosition := TRUE;
        InMotion := FALSE;
        Active := TRUE;

        IF NOT Execute THEN
            Done := FALSE;
            eState := GOH_IDLE;
        END_IF;


ELSIF eState = GOH_ABORTING THEN

    (* GOH_ABORTING - Stop motion *)
        CmdStop.Deceleration := Deceleration;
        CmdStop.Execute := TRUE;

        IF StaStop.Done THEN
            CmdStop.Execute := FALSE;
            Active := FALSE;
            InMotion := FALSE;
            eState := GOH_IDLE;

        ELSIF StaStop.Error THEN
            CmdStop.Execute := FALSE;
            eState := GOH_ERROR;
        END_IF;


ELSIF eState = GOH_ERROR THEN

    (* GOH_ERROR *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        IF NOT Execute THEN
            Error := FALSE;
            eState := GOH_IDLE;
        END_IF;

END_IF;
