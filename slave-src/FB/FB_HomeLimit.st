(* ============================================================================
   FB_HomeLimit - Home-to-Negative-Overtravel Sequence Handler (Mode 110)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Two-stage homing sequence with a fixed retract that establishes a
   deterministic, repeatable coordinate frame:
   - Fast Jog approach toward the negative overtravel switch.
   - Inter-approach backoff just far enough to clear the switch.
   - Slow Jog re-approach for a precise switch detect; the encoder
     reading at that instant is captured as rSwitchPos (in the pre-homed
     frame).
   - MC_MoveAbsolute(rSwitchPos + RetractDist) backs the axis off the
     switch by exactly RetractDist measured from the captured switch
     position. Backing off the switch before establishing the coordinate
     frame avoids any drive-side overtravel interlock interfering with
     the post-frame motion.
   - MC_SetPosition(SetPosition) then declares this retract endpoint to
     be coordinate-frame zero (= SetPosition by default). The switch
     ends up at SetPosition - RetractDist in the homed frame.

   This places position 0 at a deterministic, geometrically-defined point
   (one RetractDist positive of the switch) rather than at whatever spot
   the backoff happened to clear the switch.

   Uses the PLCopen Toolbox Jog FB (level-sensitive Forward/Reverse) for
   all approach and inter-approach moves; the explicit MC_Stop wait is
   absorbed by waiting on Jog.Done.

   Sequence (states qualified with E_HomeLimitState#<name>):
   1. FAST_APPROACH  : Jog Reverse @ FastApproachVelocity toward switch.
   2. FAST_AWAIT     : Reverse cleared on OvertravelNegActive; wait Jog.Done.
   3. INTER_BACKOFF  : Jog Forward @ BackoffVelocity until switch clears.
   4. INTER_AWAIT    : Forward cleared; wait Jog.Done.
   5. SLOW_APPROACH  : Jog Reverse @ SlowApproachVelocity for precise detect.
                       On switch trip, capture rSwitchPos := ActualPosition.
   6. SLOW_AWAIT     : Reverse cleared on OvertravelNegActive; wait Jog.Done.
   7. RETRACT        : MC_MoveAbsolute(rSwitchPos + RetractDist) — backs the
                       axis off the switch by exactly RetractDist measured from
                       the captured precise switch position.
   8. RETRACT_AWAIT  : Wait MC_MoveAbsolute.Done.
   9. SETREF         : MC_SetPosition(SetPosition) — declares the retract
                       endpoint to be SetPosition (=0). The switch then sits
                       at SetPosition - RetractDist (= -RetractDist).

   Error conditions:
   - OvertravelPosActive at any point during the sequence.
   - Jog or MC_MoveAbsolute reporting Error in any moving state.
   - MC_SetPosition reporting Error in SETREF.
   - Overall Timeout expiring.

   Integration:
   - Called once per scan at end of PRG_Main.
   - CmdJog routed to G_cmdJog by PRG_Main when ST_HOME_LIMIT is active.
   - CmdMoveAbsolute routed to G_cmdMoveAbsolute by PRG_Main when ST_HOME_LIMIT.
   - CmdSetPosition routed to G_cmdSetPosition by PRG_Main.
   - PRG_Main passes G_staJog into StaJog, G_staMoveAbsolute into
     StaMoveAbsolute, and G_staSetPosition into StaSetPosition.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable               : BOOL                   (* Enable the function block
       Execute              : BOOL                   (* Start homing sequence
       FastApproachVelocity : LREAL                  (* Coarse approach velocity (mm/s)
       SlowApproachVelocity : LREAL                  (* Precise re-approach velocity (mm/s)
       BackoffVelocity      : LREAL                  (* Inter-approach backoff AND retract velocity (mm/s)
       RetractDist          : LREAL                  (* Distance retracted from precise switch to position 0 (mm)
       Acceleration         : LREAL                  (* Move acceleration (mm/s²)
       Deceleration         : LREAL                  (* Move/stop deceleration (mm/s²)
       SetPosition          : LREAL                  (* Position value at the retract endpoint (mm)
       Timeout              : TIME                   (* Overall sequence timeout
       OvertravelNegActive  : BOOL                   (* Negative overtravel / home switch triggered
       OvertravelPosActive  : BOOL                   (* Positive overtravel switch triggered (fault)
       Abort                : BOOL                   (* Abort the sequence
       ActualPosition       : LREAL                  (* Current axis position (mm); sampled at slow detect
       StaJog               : ST_StaJog              (* Status from Jog FB
       StaMoveAbsolute      : ST_StaMoveAbsolute     (* Status from MC_MoveAbsolute
       StaSetPosition       : ST_StaSetPosition      (* Status from MC_SetPosition
     VAR_OUTPUT:
       Active               : BOOL                   (* Sequence is running
       Done                 : BOOL                   (* Homing complete
       InMotion             : BOOL                   (* Currently moving
       Error                : BOOL                   (* Error occurred
       CmdJog               : ST_CmdJog              (* Command output for Jog FB
       CmdMoveAbsolute      : ST_CmdMoveAbsolute     (* Command output for MC_MoveAbsolute
       CmdSetPosition       : ST_CmdSetPosition      (* Command output for MC_SetPosition
     VAR:
       eState               : E_HomeLimitState  (* State machine - default IDLE
       rTrigExecute         : R_TRIG
       tTimeout             : TON               (* Single call site - see end of body
       bTimeoutIN           : BOOL              (* Latched IN for tTimeout
       rSwitchPos           : LREAL             (* ActualPosition captured at SLOW_APPROACH switch trip
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    Active := FALSE;
    Done := FALSE;
    InMotion := FALSE;
    Error := FALSE;
    eState := E_HomeLimitState#IDLE;
    CmdJog.Forward := FALSE;
    CmdJog.Reverse := FALSE;
    CmdJog.Velocity := LREAL#0.0;
    CmdMoveAbsolute.Execute := FALSE;
    CmdSetPosition.Execute := FALSE;
    bTimeoutIN := FALSE;

ELSE
    (* Detect rising edge on Execute *)
    rTrigExecute(CLK := Execute);

    (* Handle abort in most states *)
    IF Abort AND (eState <> E_HomeLimitState#IDLE) AND (eState <> E_HomeLimitState#ERROR) AND (eState <> E_HomeLimitState#DONE) THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        eState := E_HomeLimitState#ABORTING;
    END_IF;

    (* Positive overtravel must never trigger during homing. *)
    IF OvertravelPosActive AND (eState <> E_HomeLimitState#IDLE) AND (eState <> E_HomeLimitState#ERROR) AND
       (eState <> E_HomeLimitState#DONE) AND (eState <> E_HomeLimitState#ABORTING) THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        Error := TRUE;
        eState := E_HomeLimitState#ERROR;
    END_IF;

    (* Default timer IN; overall timeout check uses last-scan Q *)
    bTimeoutIN := (eState <> E_HomeLimitState#IDLE) AND (eState <> E_HomeLimitState#DONE) AND
                  (eState <> E_HomeLimitState#ERROR) AND (eState <> E_HomeLimitState#ABORTING);

    IF tTimeout.Q THEN
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;
        Error := TRUE;
        bTimeoutIN := FALSE;
        eState := E_HomeLimitState#ERROR;
    END_IF;

(* --------------------------------------------------------------------------
   State Machine
   -------------------------------------------------------------------------- *)

IF eState = E_HomeLimitState#IDLE THEN

    (* IDLE - Waiting for execute command *)
        Active := FALSE;
        Done := FALSE;
        InMotion := FALSE;
        Error := FALSE;

        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;

        IF rTrigExecute.Q OR Execute THEN
            Active := TRUE;
            eState := E_HomeLimitState#FAST_APPROACH;
        END_IF;


ELSIF eState = E_HomeLimitState#FAST_APPROACH THEN

    (* FAST_APPROACH - Jog negative at fast velocity toward switch *)
        CmdJog.Reverse := TRUE;
        CmdJog.Forward := FALSE;
        CmdJog.Velocity := FastApproachVelocity;
        CmdJog.Acceleration := Acceleration;
        CmdJog.Deceleration := Deceleration;

        InMotion := StaJog.Busy;

        IF OvertravelNegActive THEN
            CmdJog.Reverse := FALSE;       (* Jog auto-decels *)
            eState := E_HomeLimitState#FAST_AWAIT;

        ELSIF StaJog.Error THEN
            CmdJog.Reverse := FALSE;
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#FAST_AWAIT THEN

    (* FAST_AWAIT - Wait for Jog to finish decelerating after fast detect *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done THEN
            eState := E_HomeLimitState#INTER_BACKOFF;

        ELSIF StaJog.Error THEN
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#INTER_BACKOFF THEN

    (* INTER_BACKOFF - Jog positive until switch clears, ready for slow re-approach *)
        CmdJog.Forward := TRUE;
        CmdJog.Reverse := FALSE;
        CmdJog.Velocity := BackoffVelocity;
        CmdJog.Acceleration := Acceleration;
        CmdJog.Deceleration := Deceleration;

        InMotion := StaJog.Busy;

        IF NOT OvertravelNegActive THEN
            CmdJog.Forward := FALSE;       (* Jog auto-decels *)
            eState := E_HomeLimitState#INTER_AWAIT;

        ELSIF StaJog.Error THEN
            CmdJog.Forward := FALSE;
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#INTER_AWAIT THEN

    (* INTER_AWAIT - Wait for Jog to finish decelerating after switch clears *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done THEN
            eState := E_HomeLimitState#SLOW_APPROACH;

        ELSIF StaJog.Error THEN
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#SLOW_APPROACH THEN

    (* SLOW_APPROACH - Jog negative at slow velocity for precise switch detect.
       Capture ActualPosition on the rising edge of OvertravelNegActive — this
       is the precise reference point for the subsequent retract. *)
        CmdJog.Reverse := TRUE;
        CmdJog.Forward := FALSE;
        CmdJog.Velocity := SlowApproachVelocity;
        CmdJog.Acceleration := Acceleration;
        CmdJog.Deceleration := Deceleration;

        InMotion := StaJog.Busy;

        IF OvertravelNegActive THEN
            rSwitchPos := ActualPosition;  (* precise switch position in pre-homed frame *)
            CmdJog.Reverse := FALSE;       (* Jog auto-decels *)
            eState := E_HomeLimitState#SLOW_AWAIT;

        ELSIF StaJog.Error THEN
            CmdJog.Reverse := FALSE;
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#SLOW_AWAIT THEN

    (* SLOW_AWAIT - Wait for Jog to finish decelerating after precise detect *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := StaJog.Busy;

        IF StaJog.Done THEN
            eState := E_HomeLimitState#RETRACT;

        ELSIF StaJog.Error THEN
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#RETRACT THEN

    (* RETRACT - MoveAbsolute to (rSwitchPos + RetractDist), backing the axis
       off the limit switch by exactly RetractDist measured from the captured
       precise switch position. This runs in the pre-homed frame; SETREF
       relabels the resulting endpoint as SetPosition (= 0) afterwards. *)
        CmdMoveAbsolute.Execute := TRUE;
        CmdMoveAbsolute.Position := rSwitchPos + RetractDist;
        CmdMoveAbsolute.Velocity := BackoffVelocity;
        CmdMoveAbsolute.Acceleration := Acceleration;
        CmdMoveAbsolute.Deceleration := Deceleration;
        CmdMoveAbsolute.Direction := MC_Direction#Positive_Direction;

        InMotion := StaMoveAbsolute.Busy;
        eState := E_HomeLimitState#RETRACT_AWAIT;


ELSIF eState = E_HomeLimitState#RETRACT_AWAIT THEN

    (* RETRACT_AWAIT - Wait for the retract MoveAbsolute to complete *)
        CmdMoveAbsolute.Execute := FALSE;
        InMotion := StaMoveAbsolute.Busy;

        IF StaMoveAbsolute.Done THEN
            eState := E_HomeLimitState#SETREF;

        ELSIF StaMoveAbsolute.Error OR StaMoveAbsolute.CommandAborted THEN
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#SETREF THEN

    (* SETREF - Declare the retract endpoint to be SetPosition (the
       coordinate-frame zero by default). The switch then sits at
       SetPosition - RetractDist in the resulting frame. *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        InMotion := FALSE;

        CmdSetPosition.Execute := TRUE;
        CmdSetPosition.Position := SetPosition;
        CmdSetPosition.Mode := FALSE;

        IF StaSetPosition.Done THEN
            CmdSetPosition.Execute := FALSE;
            Done := TRUE;
            eState := E_HomeLimitState#DONE;

        ELSIF StaSetPosition.Error THEN
            CmdSetPosition.Execute := FALSE;
            Error := TRUE;
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#DONE THEN

    (* DONE - Homing sequence complete *)
        Done := TRUE;
        Active := TRUE;
        InMotion := FALSE;

        IF NOT Execute THEN
            Done := FALSE;
            eState := E_HomeLimitState#IDLE;
        END_IF;


ELSIF eState = E_HomeLimitState#ABORTING THEN

    (* ABORTING - All command Execute lines already cleared; wait for whichever
       motion FB is still busy to finish decelerating before returning to IDLE. *)
        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        InMotion := StaJog.Busy OR StaMoveAbsolute.Busy;

        IF (StaJog.Done OR NOT StaJog.Busy) AND (NOT StaMoveAbsolute.Busy) THEN
            Active := FALSE;
            InMotion := FALSE;
            eState := E_HomeLimitState#IDLE;

        ELSIF StaJog.Error OR StaMoveAbsolute.Error THEN
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#ERROR THEN

    (* ERROR *)
        Active := FALSE;
        InMotion := FALSE;
        Done := FALSE;

        IF NOT Execute THEN
            Error := FALSE;
            eState := E_HomeLimitState#IDLE;
        END_IF;

    END_IF;  (* state machine *)
END_IF;  (* Enable gate *)

(* --------------------------------------------------------------------------
   Timer calls - exactly one site per instance per scan
   -------------------------------------------------------------------------- *)
tTimeout(IN := bTimeoutIN, PT := Timeout);
