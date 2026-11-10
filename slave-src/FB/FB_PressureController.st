(* ============================================================================
   FB_PressureController - PI Pressure Loop Producing a Velocity Command

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Outer loop of the pressure cascade. Compares the target with the measured
   chamber pressure and produces a piston velocity command for
   Y_DirectControl in VelocityTLMode; the SERVOPACK velocity loop is the
   inner loop. Positive velocity drives the piston into the cylinder and
   raises the pressure.

   The plant is integrating (velocity -> volume -> pressure), so Kp alone
   gives zero steady-state error against a constant target in a sealed
   chamber. Ki is for sustained disturbances such as outflow during
   discharge.

   Stop law: the chamber stiffness dP/dx rises by orders of magnitude as the
   gas is compressed and dissolves, so a velocity that is harmless in the
   soft regime overshoots the target by many MPa once the chamber is stiff.
   The output is therefore also clamped to the velocity from which the
   axis, decelerating at DecelMax, comes to rest within the pressure error
   at the assumed stiffness: v = sqrt(2 * DecelMax * |Target - Actual| / Stiffness).
   Stiffness is a configured value, not a measurement: the filtered pressure
   lags the chamber, and in a gas-rich chamber the stiffness grows faster
   than any estimate from the lagging reading can follow, so a value above
   the liquid-only chamber's is what keeps the approach from overshooting.
   The clamp never falls below a floor (StopVelMin, or the floor that
   follows the outflow, below), so a sustained disturbance such as outflow
   can be balanced at zero error; a velocity that small overshoots by at
   most Stiffness * StopVelMin^2 / (2 * DecelMax). Above
   that the clamp only binds while the error is large; near the target it
   is well above what the PI terms command.

   Anti-windup: the integrator advances only while the output is not
   clamped, or while the error is driving the output back off the clamp.
   Reset clears the integrator; the first scan after Reset or Enable
   produces the proportional term only, so entry is bumpless when the error
   is small.

   Floor that follows the outflow (FloorEnable; design/pressure-loop/README.md,
   "Stop-law floor policies", equations (46) and (48)): a fixed StopVelMin
   high enough for the outflow lets an integrator that holds a forward
   velocity run the piston into the chamber when the outlet shuts. With
   FloorEnable the stop law's floor is VelLow until outflow is detected and
   StopVelMin while the detection is latched, and no stiffness value is
   needed.
     Window: over W = Window / CycleTime scans, with the position delayed by
     the measurement lag D (the filter time constant in scans and the
     median's half-width; PRG_Main passes the reading in the scan it is
     sampled) so that the two describe the same interval:
       dx = Position[n - D] - Position[n - D - W],   dP = Actual[n] - Actual[n - W]
     The rate is dP / (W CycleTime) and the speed dx / (W CycleTime). In a
     sealed chamber the pressure rises with the piston, so a piston that
     moves forward while the pressure does not rise is displacing outflow.
     Nothing is decided until W + D + 1 scans have been seen.
     Latch, eq. (46): set when, for TrigTime, the output was clamped and
     forward on the previous scan, the error exceeded TrigError, the speed
     was at least TrigVel and the rate below TrigRate. Released when the
     rate exceeds RelRate with the error at or below RelError (the pressure
     at or above the target): the outlet has shut under a moving piston.
     Forward stop, eq. (48): while not latched, with the error negative and
     ForwardStop set, a positive output is zeroed. The stop law limits the
     speed in either direction, so without it an integrator that holds a
     forward velocity keeps the piston moving into a chamber already above
     the target. The integrator and its anti-windup see the output before
     the forward stop.
   The window and the latch are cleared while the block is disabled, so each
   entry to pressure mode starts them afresh; Reset does not clear them. A
   floor setting out of range (FloorError) leaves the floor at StopVelMin
   with no forward stop.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Run the loop; FALSE holds output at zero
       Reset           : BOOL             (* Clear the integrator this scan
       Target          : LREAL            (* Target pressure (MPa gauge)
       Actual          : LREAL            (* Measured pressure (MPa gauge)
       Position        : LREAL            (* Piston position (mm), positive into the cylinder; used by the floor
       Kp              : LREAL            (* mm/s per MPa
       Ki              : LREAL            (* mm/s per (MPa * s)
       VelMax          : LREAL            (* Output clamp (mm/s), symmetric
       DecelMax        : LREAL            (* Deceleration the axis is configured for (mm/s^2)
       Stiffness       : LREAL            (* Chamber stiffness assumed by the stop law (MPa/mm)
       StopVelMin      : LREAL            (* The stop law never clamps below this (mm/s); with FloorEnable, the floor while outflow is latched
       CycleTime       : LREAL            (* Task period (s)
       FloorEnable     : BOOL             (* Switch the floor on detected outflow
       VelLow          : LREAL            (* Floor with no outflow detected (mm/s)
       Window          : LREAL            (* Window over which the rate and speed are taken (s)
       FilterTime      : LREAL            (* Pressure input low-pass time constant (s)
       MedianSize      : INT              (* Pressure input median window (3 or 5)
       TrigVel         : LREAL            (* Least piston speed for the trigger (mm/s)
       TrigRate        : LREAL            (* Pressure rise rate below which the trigger counts (MPa/s)
       TrigError       : LREAL            (* Error above which the trigger counts (MPa)
       TrigTime        : LREAL            (* Time the trigger must hold (s)
       RelRate         : LREAL            (* Pressure rise rate above which the release counts (MPa/s)
       RelError        : LREAL            (* Error at or below which the release counts (MPa)
       ForwardStop     : BOOL             (* Zero a forward output above the target while outflow is not latched
     VAR_OUTPUT:
       Velocity        : LREAL            (* Commanded velocity (mm/s), within +/-rVelLimit
       Latched         : BOOL             (* Outflow is detected (floor at StopVelMin)
       FloorError      : BOOL             (* A floor setting is out of range; the floor is StopVelMin, no forward stop
     VAR:
       rError          : LREAL            (* Target - Actual (MPa)
       rVelLimit       : LREAL            (* Clamp applied this scan: min of VelMax and the stop law (mm/s)
       bSaturated      : BOOL := FALSE    (* Output is at the clamp this scan
       bParamError     : BOOL             (* VelMax, DecelMax, Stiffness or CycleTime not positive, or StopVelMin negative
       rInteg          : LREAL := 0.0     (* Integral of the error (MPa * s)
       rRaw            : LREAL            (* Unclamped output this scan
       rVelPI          : LREAL := 0.0     (* Clamped output before the forward stop (mm/s)
       rDist           : LREAL            (* Travel that closes the error at the assumed stiffness (mm)
       rStep           : LREAL            (* Velocity change per scan at DecelMax (mm/s)
       rVelStop        : LREAL            (* Stop-law velocity (mm/s)
       rFloor          : LREAL            (* Stop-law floor this scan (mm/s)
       bFloorOn        : BOOL             (* The floor follows the outflow this scan
       aPHist          : ARRAY64LREAL     (* Ring of measured pressures (MPa), newest at nIx
       aXHist          : ARRAY64LREAL     (* Ring of positions (mm), newest at nIx
       nIx             : INT := 63        (* Index of the newest sample
       nFill           : INT := 0         (* Samples in the rings since they were cleared, up to cDepth
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

bParamError := (VelMax <= LREAL#0.0) OR (DecelMax <= LREAL#0.0) OR
              (Stiffness <= LREAL#0.0) OR (CycleTime <= LREAL#0.0) OR
              (StopVelMin < LREAL#0.0);

(* --------------------------------------------------------------------------
   Floor settings: the window, the position delay and the trigger hold in
   scans, within the rings
   -------------------------------------------------------------------------- *)
FloorError := FALSE;
IF FloorEnable AND NOT bParamError THEN
    FloorError := (Window <= LREAL#0.0) OR (FilterTime < LREAL#0.0) OR
                  (TrigTime < LREAL#0.0) OR (VelLow < LREAL#0.0) OR
                  (StopVelMin < VelLow);
    IF NOT FloorError THEN
        FloorError := (Window / CycleTime > LREAL#60.0) OR
                      (FilterTime / CycleTime > LREAL#60.0) OR
                      (TrigTime / CycleTime > LREAL#10000.0);
    END_IF;
    IF NOT FloorError THEN
        IF MedianSize >= 5 THEN
            nMedHalf := 2;
        ELSE
            nMedHalf := 1;
        END_IF;
        nW := LREAL_TO_INT(LIMIT(LREAL#1.0, Window / CycleTime, LREAL#60.0));
        nD := LREAL_TO_INT(FilterTime / CycleTime) + nMedHalf;
        nOn := LREAL_TO_INT(TrigTime / CycleTime);
        FloorError := nW + nD + 1 > cDepth;
    END_IF;
END_IF;
bFloorOn := FloorEnable AND (NOT FloorError) AND (NOT bParamError);

IF (NOT Enable) OR (NOT bFloorOn) THEN
    nIx := cDepth - 1;
    nFill := 0;
    nCount := 0;
    Latched := FALSE;
END_IF;

IF (NOT Enable) OR Reset OR bParamError THEN
    rInteg := LREAL#0.0;
END_IF;

IF (NOT Enable) OR bParamError THEN
    Velocity := LREAL#0.0;
    rVelPI := LREAL#0.0;
    bSaturated := FALSE;
    RETURN;
END_IF;

rError := Target - Actual;

(* --------------------------------------------------------------------------
   Floor: VelLow until outflow is latched, eq. (46), from the window and the
   output of the previous scan; StopVelMin otherwise
   -------------------------------------------------------------------------- *)
rFloor := StopVelMin;
IF bFloorOn THEN
    nIx := nIx + 1;
    IF nIx >= cDepth THEN
        nIx := 0;
    END_IF;
    aPHist[nIx] := Actual;
    aXHist[nIx] := Position;
    IF nFill < cDepth THEN
        nFill := nFill + 1;
    END_IF;

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
            IF (rRate > RelRate) AND (rError <= RelError) THEN
                Latched := FALSE;
                nCount := 0;
            END_IF;
        ELSE
            bStuck := bSaturated AND (rVelPI > LREAL#0.0) AND
                      (rError > TrigError) AND (rSpeed >= TrigVel) AND
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

    IF NOT Latched THEN
        rFloor := VelLow;
    END_IF;
END_IF;

(* --------------------------------------------------------------------------
   Stop law: the velocity from which DecelMax brings the axis to rest within
   the travel that closes the error at the assumed stiffness, accounting for
   the scan in which the command takes effect; never below the floor.
   -------------------------------------------------------------------------- *)
rDist := ABS(rError) / Stiffness;
rStep := DecelMax * CycleTime;
rVelStop := -rStep + SQRT(rStep * rStep + LREAL#2.0 * DecelMax * rDist);
rVelLimit := MIN(VelMax, MAX(rFloor, rVelStop));

(* --------------------------------------------------------------------------
   PI with conditional anti-windup
   -------------------------------------------------------------------------- *)
rRaw := Kp * rError + Ki * rInteg;
rVelPI := LIMIT(-rVelLimit, rRaw, rVelLimit);
bSaturated := (rRaw <> rVelPI);

IF (NOT bSaturated) OR
   ((rRaw > rVelPI) AND (rError < LREAL#0.0)) OR
   ((rRaw < rVelPI) AND (rError > LREAL#0.0)) THEN
    rInteg := rInteg + rError * CycleTime;
END_IF;

(* --------------------------------------------------------------------------
   Forward stop above the target while outflow is not latched, eq. (48)
   -------------------------------------------------------------------------- *)
Velocity := rVelPI;
IF bFloorOn AND ForwardStop AND (NOT Latched) AND
   (rError < LREAL#0.0) AND (Velocity > LREAL#0.0) THEN
    Velocity := LREAL#0.0;
END_IF;
