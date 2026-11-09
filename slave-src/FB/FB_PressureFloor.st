(* ============================================================================
   FB_PressureFloor - Stop-Law Floor That Follows the Outflow

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Sets the pressure loop's StopVelMin every scan from what the slave
   measures, in place of a fixed value, and says when to stop a forward
   command above the target (design/pressure-loop/README.md, "Stop-law floor
   policies", equations (46) and (48): the switched floor with the forward
   stop). PRG_Main calls it just before fbPressureSched and fbPressureCtrl
   and passes StopVelMin to both when G_cfgPressureFloorEnable was set on
   entry to pressure mode, and zeroes a positive pressure-loop command while
   StopForward is set.

   The stop law clamps the velocity to zero at zero error, which is right
   for a sealed chamber and wrong with the outlet open, where the piston
   must keep moving to hold the pressure. A fixed StopVelMin high enough for
   the outflow lets an integrator that holds a forward velocity run the
   piston into the chamber when the outlet shuts. This block keeps the floor
   at VelLow until it detects outflow and at VelHigh while the detection is
   latched, and needs no stiffness value.

   Window: over W = Window / CycleTime scans, with the position delayed by
   the measurement lag D (the filter time constant in scans, the median's
   half-width and the two pipeline scans) so that the two describe the same
   interval:
     dx = Position[n - D] - Position[n - D - W],   dP = Actual[n] - Actual[n - W]
   The rate is dP / (W CycleTime) and the speed dx / (W CycleTime). In a
   sealed chamber the pressure rises with the piston, so a piston that moves
   forward while the pressure does not rise is displacing outflow. Nothing is
   decided until W + D + 1 scans have been seen since Reset.

   Latch, eq. (46): set when, for TrigTime, the controller was saturated
   with a forward output on the previous scan, the error exceeded
   TrigError, the speed was at least TrigVel and the rate below TrigRate.
   Released when the rate exceeds RelRate with the error at or below
   RelError (the pressure at or above the target): the outlet has shut
   under a moving piston.

   Forward stop, eq. (48): while not latched, with the error negative and
   ForwardStop set, StopForward tells the caller to zero a positive command.
   The stop law limits the speed in either direction, so without it an
   integrator that holds a forward velocity keeps the piston moving into a
   chamber already above the target.

   Disabled, or with a setting out of range, the block outputs VelHigh, the
   configured StopVelMin, and no forward stop; Reset or a disabled scan
   clears the window and the latch.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Run the policy; FALSE outputs VelHigh and no forward stop
       Reset           : BOOL             (* Clear the window and the latch this scan
       Actual          : LREAL            (* Measured pressure (MPa gauge)
       Position        : LREAL            (* Piston position (mm), positive into the cylinder
       Error           : LREAL            (* Target - Actual this scan (MPa)
       Saturated       : BOOL             (* The controller's output was at its clamp on the previous scan
       VelocityPrev    : LREAL            (* The controller's output on the previous scan (mm/s)
       VelLow          : LREAL            (* Floor with no outflow detected (mm/s)
       VelHigh         : LREAL            (* Floor while outflow is latched, the configured StopVelMin (mm/s)
       Window          : LREAL            (* Window over which the rate and speed are taken (s)
       CycleTime       : LREAL            (* Task period (s)
       FilterTime      : LREAL            (* Pressure input low-pass time constant (s)
       MedianSize      : INT              (* Pressure input median window (3 or 5)
       TrigVel         : LREAL            (* Least piston speed for the trigger (mm/s)
       TrigRate        : LREAL            (* Pressure rise rate below which the trigger counts (MPa/s)
       TrigError       : LREAL            (* Error above which the trigger counts (MPa)
       TrigTime        : LREAL            (* Time the trigger must hold (s)
       RelRate         : LREAL            (* Pressure rise rate above which the release counts (MPa/s)
       RelError        : LREAL            (* Error at or below which the release counts (MPa)
       ForwardStop     : BOOL             (* Stop a forward command above the target while not latched
     VAR_OUTPUT:
       StopVelMin      : LREAL            (* Floor for the stop law this scan (mm/s)
       StopForward     : BOOL             (* Zero a positive pressure-loop command this scan
       Latched         : BOOL             (* Outflow is detected
       ParamError      : BOOL             (* A setting is out of range; outputs as disabled
     VAR:
       aPHist          : ARRAY64LREAL     (* Ring of measured pressures (MPa), newest at nIx
       aXHist          : ARRAY64LREAL     (* Ring of positions (mm), newest at nIx
       nIx             : INT := 63        (* Index of the newest sample
       nFill           : INT := 0         (* Samples in the rings since Reset, up to cDepth
       nCount          : INT := 0         (* Scans the trigger has held
       nW              : INT              (* Window (scans)
       nD              : INT              (* Position delay (scans)
       nOn             : INT              (* Trigger hold (scans)
       nMedHalf        : INT              (* Median half-width (scans)
       nI              : INT              (* Ring index
       cDepth          : INT := 64        (* Ring depth
       rDP             : LREAL            (* Pressure change over the window (MPa)
       rX1             : LREAL            (* Delayed position at the end of the window (mm)
       rDX             : LREAL            (* Delayed position change over the window (mm)
       rT              : LREAL            (* Window length (s)
       rRate           : LREAL            (* Pressure rise rate (MPa/s)
       rSpeed          : LREAL            (* Piston speed (mm/s)
       bStuck          : BOOL             (* The trigger condition holds this scan
   ============================================================================ *)

ParamError := (CycleTime <= LREAL#0.0) OR (Window <= LREAL#0.0) OR
              (FilterTime < LREAL#0.0) OR (TrigTime < LREAL#0.0) OR
              (VelLow < LREAL#0.0) OR (VelHigh < VelLow);

IF NOT ParamError THEN
    ParamError := (Window / CycleTime > LREAL#60.0) OR
                  (FilterTime / CycleTime > LREAL#60.0) OR
                  (TrigTime / CycleTime > LREAL#10000.0);
END_IF;

IF NOT ParamError THEN
    IF MedianSize >= 5 THEN
        nMedHalf := 2;
    ELSE
        nMedHalf := 1;
    END_IF;
    nW := LREAL_TO_INT(LIMIT(LREAL#1.0, Window / CycleTime, LREAL#60.0));
    nD := LREAL_TO_INT(FilterTime / CycleTime) + nMedHalf + 2;
    nOn := LREAL_TO_INT(TrigTime / CycleTime);
    ParamError := nW + nD + 1 > cDepth;
END_IF;

IF (NOT Enable) OR Reset OR ParamError THEN
    nIx := cDepth - 1;
    nFill := 0;
    nCount := 0;
    Latched := FALSE;
END_IF;

IF (NOT Enable) OR ParamError THEN
    StopVelMin := VelHigh;
    StopForward := FALSE;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Window: store this scan's samples
   -------------------------------------------------------------------------- *)
nIx := nIx + 1;
IF nIx >= cDepth THEN
    nIx := 0;
END_IF;
aPHist[nIx] := Actual;
aXHist[nIx] := Position;
IF nFill < cDepth THEN
    nFill := nFill + 1;
END_IF;

(* --------------------------------------------------------------------------
   Latch, eq. (46), once the window and the delay are filled
   -------------------------------------------------------------------------- *)
IF nFill >= nW + nD + 1 THEN
    nI := (nIx - nW + cDepth) MOD cDepth;
    rDP := Actual - aPHist[nI];
    nI := (nIx - nD + cDepth) MOD cDepth;
    rX1 := aXHist[nI];
    nI := (nIx - nD - nW + cDepth) MOD cDepth;
    rDX := rX1 - aXHist[nI];
    rT := INT_TO_LREAL(nW) * CycleTime;
    rRate := rDP / rT;
    rSpeed := rDX / rT;

    IF Latched THEN
        IF (rRate > RelRate) AND (Error <= RelError) THEN
            Latched := FALSE;
            nCount := 0;
        END_IF;
    ELSE
        bStuck := Saturated AND (VelocityPrev > LREAL#0.0) AND
                  (Error > TrigError) AND (rSpeed >= TrigVel) AND
                  (rRate < TrigRate);
        IF bStuck THEN
            IF nCount < 10000 THEN
                nCount := nCount + 1;
            END_IF;
        ELSE
            nCount := 0;
        END_IF;
        IF nCount >= nOn THEN
            Latched := TRUE;
        END_IF;
    END_IF;
END_IF;

IF Latched THEN
    StopVelMin := VelHigh;
ELSE
    StopVelMin := VelLow;
END_IF;

(* Forward stop above the target while not latched, eq. (48) *)
StopForward := ForwardStop AND (NOT Latched) AND (Error < LREAL#0.0);
