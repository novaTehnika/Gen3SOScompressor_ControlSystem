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

   Speed hold: SpeedFraction <= 0 (the operator speed is 0) holds the
   sequence. The jog phases release their direction and the Jog FB
   decelerates the axis to rest; the retract is not started while held, and
   one that has started completes. The sequence timeout is a clock advanced
   by CycleTime * SpeedFraction, so it counts no time while held and, at a
   reduced speed, allows the same travel as at the ceiling.

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
       Timeout              : TIME                   (* Sequence timeout, counted at the speed fraction (time at the fast-approach ceiling)
       SpeedFraction        : LREAL                  (* Operator speed as a fraction of the fast-approach ceiling [0..1]; 0 holds the sequence; the timeout clock runs at this rate
       CycleTime            : LREAL                  (* Scan time (s)
       OvertravelNegActive  : BOOL                   (* Negative overtravel / home switch triggered
       OvertravelPosActive  : BOOL                   (* Positive overtravel switch triggered (fault)
       Abort                : BOOL                   (* Abort the sequence
       ActualPosition       : LREAL                  (* Current axis position (mm); sampled at slow detect
       StaJog               : ST_StaJog              (* Status from Jog FB
       StaMoveAbsolute      : ST_StaMoveAbsolute     (* Status from MC_MoveAbsolute
       StaSetPosition       : ST_StaSetPosition      (* Status from MC_SetPosition
     VAR_OUTPUT:
       Done                 : BOOL                   (* Homing complete
       InMotion             : BOOL                   (* Currently moving
       Error                : BOOL                   (* Error occurred
       CmdJog               : ST_CmdJog              (* Command output for Jog FB
       CmdMoveAbsolute      : ST_CmdMoveAbsolute     (* Command output for MC_MoveAbsolute
       CmdSetPosition       : ST_CmdSetPosition      (* Command output for MC_SetPosition
     VAR:
       eState               : E_HomeLimitState  (* State machine - default IDLE
       rTrigExecute         : R_TRIG
       rTimeoutElapsed      : LREAL             (* Timeout clock (s), advanced by CycleTime * SpeedFraction while the sequence runs
       bTimeoutIN           : BOOL              (* Sequence running: the timeout clock advances
       bHold                : BOOL              (* SpeedFraction <= 0
       rSwitchPos           : LREAL             (* ActualPosition captured at SLOW_APPROACH switch trip
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
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

    bHold := SpeedFraction <= LREAL#0.0;

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

    (* Timeout clock runs while the sequence does; the check uses last scan's clock *)
    bTimeoutIN := (eState <> E_HomeLimitState#IDLE) AND (eState <> E_HomeLimitState#DONE) AND
                  (eState <> E_HomeLimitState#ERROR) AND (eState <> E_HomeLimitState#ABORTING);

    IF rTimeoutElapsed >= DINT_TO_LREAL(TIME_TO_DINT(Timeout)) / LREAL#1000.0 THEN
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
        Done := FALSE;
        InMotion := FALSE;
        Error := FALSE;

        CmdJog.Forward := FALSE;
        CmdJog.Reverse := FALSE;
        CmdMoveAbsolute.Execute := FALSE;
        CmdSetPosition.Execute := FALSE;

        IF rTrigExecute.Q OR Execute THEN
            eState := E_HomeLimitState#FAST_APPROACH;
        END_IF;


ELSIF eState = E_HomeLimitState#FAST_APPROACH THEN

    (* FAST_APPROACH - Jog negative at fast velocity toward switch *)
        CmdJog.Reverse := NOT bHold;
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
        CmdJog.Forward := NOT bHold;
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
        CmdJog.Reverse := NOT bHold;
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
       relabels the resulting endpoint as SetPosition (= 0) afterwards.
       While held the move is not started; this state is held here, not in
       SLOW_AWAIT, because Jog.Done is a one-scan pulse. *)
        IF NOT bHold THEN
            CmdMoveAbsolute.Execute := TRUE;
            CmdMoveAbsolute.Position := rSwitchPos + RetractDist;
            CmdMoveAbsolute.Velocity := BackoffVelocity;
            CmdMoveAbsolute.Acceleration := Acceleration;
            CmdMoveAbsolute.Deceleration := Deceleration;
            CmdMoveAbsolute.Direction := MC_Direction#Positive_Direction;

            InMotion := StaMoveAbsolute.Busy;
            eState := E_HomeLimitState#RETRACT_AWAIT;
        ELSE
            CmdMoveAbsolute.Execute := FALSE;
            InMotion := FALSE;
        END_IF;


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
            InMotion := FALSE;
            eState := E_HomeLimitState#IDLE;

        ELSIF StaJog.Error OR StaMoveAbsolute.Error THEN
            eState := E_HomeLimitState#ERROR;
        END_IF;


ELSIF eState = E_HomeLimitState#ERROR THEN

    (* ERROR *)
        InMotion := FALSE;
        Done := FALSE;

        IF NOT Execute THEN
            Error := FALSE;
            eState := E_HomeLimitState#IDLE;
        END_IF;

    END_IF;  (* state machine *)
END_IF;  (* Enable gate *)

(* --------------------------------------------------------------------------
   Timeout clock - advances at the speed fraction while the sequence runs
   -------------------------------------------------------------------------- *)
IF bTimeoutIN THEN
    rTimeoutElapsed := rTimeoutElapsed + CycleTime * LIMIT(LREAL#0.0, SpeedFraction, LREAL#1.0);
ELSE
    rTimeoutElapsed := LREAL#0.0;
END_IF;
