(* ============================================================================
   FB_PositionCommandGate - Velocity/Acceleration Gate for Direct Position Control

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Sits between the analog-derived position target and Y_DirectControl in
   PositionMode. Y_DirectControl adds no profiling in that mode (Toolbox
   manual, Y_ControlMode#PositionMode), so a target step reaches the drive
   as a step and trips the SERVOPACK excessive-deviation alarm. This FB
   converts an arbitrary target into a per-scan command the axis can follow.

   It is a gate, not a move planner. State is the last command issued
   (rCmdPos) and its rate (rCmdRate) - nothing else. Each scan:
   1. Tether  - if the command is already MaxDeviation ahead of the actual
                axis and the target is further away still, hold. Bounds
                following error when the axis stalls (torque limit during
                compression, blocked piston). Releases when the axis catches
                up or the target reverses.
   2. Stop law - fastest rate from which the accel gate can still brake to
                rest at the target, exact in discrete time:
                  rateStop = -A*dt + SQRT((A*dt)^2 + 2*A*|dist|)
                Near the target this collapses to |dist|/dt, so the command
                lands exactly - no deadband, no chatter, no overshoot.
   3. Rate gate - target rate = +/-MIN(VelMax, rateStop).
   4. Accel gate - rate slews toward target rate by at most A*dt per scan,
                then the command integrates.

   A target step yields a trapezoidal velocity profile. Analog noise is
   attenuated, not hunted: a 1-count flicker can change the rate by at most
   A*dt per scan.

   Seeding: while NOT Enable, or on Reset, the command is seeded at
   ActualPosition with zero rate and passed straight to the output. Callers
   enable this FB only on scans where the owning state body has run, so the
   state-entry scan (whose target is stale) commands "stay put".

   Soft limits are the caller's job (clamp Target upstream). There is
   deliberately no output clamp: the command may legitimately seed outside
   the soft box in ST_RECOVERY_POSITION and must ramp inward.

   Parameter guard: a non-positive VelMax, AccelMax or CycleTime holds the
   command and sets ParamError. Fails to "not moving", never to passing
   Target through unshaped.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Gate active; FALSE seeds from ActualPosition
       Reset               : BOOL             (* One-scan re-seed (state entry)
       Target              : LREAL            (* Position target (mm), soft-limit clamped by caller
       ActualPosition      : LREAL            (* Axis actual position (mm)
       VelMax              : LREAL            (* Command rate limit (mm/s), > 0
       AccelMax            : LREAL            (* Command accel limit (mm/s^2), > 0
       MaxDeviation        : LREAL            (* Tether window (mm); <= 0 disables tether
       CycleTime           : LREAL            (* Task period (s), > 0
     VAR_OUTPUT:
       CommandedPosition   : LREAL            (* Command to Y_DirectControl.Position (mm)
       CommandedRate       : LREAL            (* Command rate (mm/s), diagnostic
       RateLimiting        : BOOL             (* Gate is shaping (target not reachable this scan)
       TetherHold          : BOOL             (* Command held at MaxDeviation ahead of axis
       ParamError          : BOOL             (* VelMax/AccelMax/CycleTime not positive
     VAR:
       rCmdPos             : LREAL := 0.0     (* Last command issued (mm)
       rCmdRate            : LREAL := 0.0     (* Rate of last command (mm/s)
       rDist               : LREAL            (* Target - rCmdPos
       rLead               : LREAL            (* rCmdPos - ActualPosition
       rAdt                : LREAL            (* AccelMax * CycleTime
       rRateStop           : LREAL            (* Max rate that can still stop at target
       rRateTgt            : LREAL            (* Rate the accel gate slews toward
       bHold               : BOOL
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

(* --------------------------------------------------------------------------
   Seed: disabled or reset - track the axis, output actual position
   -------------------------------------------------------------------------- *)

IF (NOT Enable) OR Reset THEN
    rCmdPos := ActualPosition;
    rCmdRate := LREAL#0.0;
    CommandedPosition := rCmdPos;
    CommandedRate := rCmdRate;
    RateLimiting := FALSE;
    TetherHold := FALSE;
    ParamError := FALSE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Parameter guard - hold the command rather than pass Target through
   -------------------------------------------------------------------------- *)

ParamError := (VelMax <= LREAL#0.0) OR (AccelMax <= LREAL#0.0) OR (CycleTime <= LREAL#0.0);

IF ParamError THEN
    rCmdRate := LREAL#0.0;
    CommandedPosition := rCmdPos;
    CommandedRate := rCmdRate;
    RateLimiting := TRUE;
    TetherHold := FALSE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Tether - hold if already MaxDeviation ahead of the axis and the target is
   further away still. Direction test is on rDist (where the target is), not
   on rCmdRate, so the hold is steady and cannot re-trigger on its own zeroing.
   -------------------------------------------------------------------------- *)

rDist := Target - rCmdPos;
rLead := rCmdPos - ActualPosition;

bHold := (MaxDeviation > LREAL#0.0) AND
         (ABS(rLead) >= MaxDeviation) AND
         ((rLead * rDist) > LREAL#0.0);

IF bHold THEN
    rCmdRate := LREAL#0.0;

ELSE
    (* ----------------------------------------------------------------------
       Stop law + rate gate
       ---------------------------------------------------------------------- *)
    rAdt := AccelMax * CycleTime;
    rRateStop := -rAdt + SQRT((rAdt * rAdt) + (LREAL#2.0 * AccelMax * ABS(rDist)));

    rRateTgt := MIN(VelMax, rRateStop);
    IF rDist < LREAL#0.0 THEN
        rRateTgt := -rRateTgt;
    END_IF;

    (* ----------------------------------------------------------------------
       Accel gate + integrate
       ---------------------------------------------------------------------- *)
    rCmdRate := rCmdRate + LIMIT(-rAdt, rRateTgt - rCmdRate, rAdt);
    rCmdPos := rCmdPos + (rCmdRate * CycleTime);
END_IF;

(* --------------------------------------------------------------------------
   Output
   -------------------------------------------------------------------------- *)

CommandedPosition := rCmdPos;
CommandedRate := rCmdRate;
TetherHold := bHold;
RateLimiting := bHold OR (ABS(Target - rCmdPos) > (VelMax * CycleTime));
