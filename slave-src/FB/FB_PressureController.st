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
   at the assumed stiffness: v = sqrt(2 * DecelMax * |Error| / Stiffness).
   Stiffness is a configured value, not a measurement: the filtered pressure
   lags the chamber, and in a gas-rich chamber the stiffness grows faster
   than any estimate from the lagging reading can follow, so a value above
   the liquid-only chamber's is what keeps the approach from overshooting.
   The clamp never falls below StopVelMin, so a sustained disturbance such
   as outflow can be balanced at zero error; a velocity that small
   overshoots by at most Stiffness * StopVelMin^2 / (2 * DecelMax). Above
   that the clamp only binds while the error is large; near the target it
   is well above what the PI terms command.

   Anti-windup: the integrator advances only while the output is not
   clamped, or while the error is driving the output back off the clamp.
   Reset clears the integrator; the first scan after Reset or Enable
   produces the proportional term only, so entry is bumpless when the error
   is small.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Run the loop; FALSE holds output at zero
       Reset           : BOOL             (* Clear the integrator and estimate this scan
       Target          : LREAL            (* Target pressure (MPa gauge)
       Actual          : LREAL            (* Measured pressure (MPa gauge)
       Kp              : LREAL            (* mm/s per MPa
       Ki              : LREAL            (* mm/s per (MPa * s)
       VelMax          : LREAL            (* Output clamp (mm/s), symmetric
       DecelMax        : LREAL            (* Deceleration the axis is configured for (mm/s^2)
       Stiffness       : LREAL            (* Chamber stiffness assumed by the stop law (MPa/mm)
       StopVelMin      : LREAL            (* The stop law never clamps below this (mm/s)
       CycleTime       : LREAL            (* Task period (s)
     VAR_OUTPUT:
       Velocity        : LREAL            (* Commanded velocity (mm/s), within +/-VelLimit
       Error           : LREAL            (* Target - Actual (MPa), for diagnostics
       VelLimit        : LREAL            (* Clamp applied this scan: min of VelMax and the stop law (mm/s)
       Saturated       : BOOL             (* Output is at the clamp this scan
       ParamError      : BOOL             (* VelMax, DecelMax, Stiffness or CycleTime not positive, or StopVelMin negative
     VAR:
       rInteg          : LREAL := 0.0     (* Integral of the error (MPa * s)
       rRaw            : LREAL            (* Unclamped output this scan
       rDist           : LREAL            (* Travel that closes the error at the assumed stiffness (mm)
       rStep           : LREAL            (* Velocity change per scan at DecelMax (mm/s)
       rVelStop        : LREAL            (* Stop-law velocity (mm/s)
   ============================================================================ *)

ParamError := (VelMax <= LREAL#0.0) OR (DecelMax <= LREAL#0.0) OR
              (Stiffness <= LREAL#0.0) OR (CycleTime <= LREAL#0.0) OR
              (StopVelMin < LREAL#0.0);

IF (NOT Enable) OR Reset OR ParamError THEN
    rInteg := LREAL#0.0;
END_IF;

IF (NOT Enable) OR ParamError THEN
    Velocity := LREAL#0.0;
    Error := LREAL#0.0;
    VelLimit := LREAL#0.0;
    Saturated := FALSE;
    RETURN;
END_IF;

Error := Target - Actual;

(* --------------------------------------------------------------------------
   Stop law: the velocity from which DecelMax brings the axis to rest within
   the travel that closes the error at the assumed stiffness, accounting for
   the scan in which the command takes effect; never below StopVelMin.
   -------------------------------------------------------------------------- *)
rDist := ABS(Error) / Stiffness;
rStep := DecelMax * CycleTime;
rVelStop := -rStep + SQRT(rStep * rStep + LREAL#2.0 * DecelMax * rDist);
VelLimit := MIN(VelMax, MAX(StopVelMin, rVelStop));

(* --------------------------------------------------------------------------
   PI with conditional anti-windup
   -------------------------------------------------------------------------- *)
rRaw := Kp * Error + Ki * rInteg;
Velocity := LIMIT(-VelLimit, rRaw, VelLimit);
Saturated := (rRaw <> Velocity);

IF (NOT Saturated) OR
   ((rRaw > Velocity) AND (Error < LREAL#0.0)) OR
   ((rRaw < Velocity) AND (Error > LREAL#0.0)) THEN
    rInteg := rInteg + Error * CycleTime;
END_IF;
