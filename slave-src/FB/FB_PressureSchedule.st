(* ============================================================================
   FB_PressureSchedule - Gain Schedule for the Pressure Loop

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Sets FB_PressureController's gains and stop-law limit every scan from an
   upper bound on the chamber stiffness (design/pressure-loop/README.md,
   equations (39) to (41)). PRG_Main calls it just before fbPressureCtrl and
   passes its outputs to that block when G_cfgPressureScheduleEnable was set
   on entry to pressure mode.

   Stiffness bound, eq. (39): the chamber volume is the swept volume from the
   piston to PosLimit plus DeadVolume. The free gas is the least there can
   be: the configured oxygen charge less what the configured saline volume
   can hold at the highest absolute pressure seen since Reset. The saline
   volume is the configured one, or the chamber volume when that is smaller.
   With BulkModulus the largest plausible and HoseCompliance the smallest,
     C = Vgas / Pabs + min(Vsaline, Vchamber) / BulkModulus + HoseCompliance
     KEst = PistonArea / C
   is never below the true stiffness while SalineVolume and DeadVolume are
   at or below the true values.

   Gains, eq. (40): Kp = LIMIT(KpMin, Crossover / KEst, KpMax) and
   Ki = KiRatio * Kp, so the loop's crossover is at most Crossover.

   Stop law, eq. (41): the piston travels for Lag at its speed and then
   decelerates at DecelMax, and must stop within the error at KEst:
     v = -DecelMax * Lag + SQRT((DecelMax * Lag)^2 + 2 * DecelMax * |e| / KEst)
   VelLimit = MIN(VelMax, MAX(StopVelMin, v)) is given to the controller as
   its VelMax, and Stiffness (a negligible value) disables the controller's
   own stop law. Lag is the measurement chain (filter time constant, median
   and pipeline scans, the scan in which the command takes effect) plus the
   drive's dead time and velocity-loop lag.

   Integrator: the controller multiplies its stored integral by Ki, so on a
   scan where Ki changes IntegScale = previous Ki / new Ki keeps the integral
   term continuous (FB_PressureController.IntegScale). IntegHold is TRUE until
   the error first comes within ArriveBand, and on the scan it does; PRG_Main
   ORs it into the controller's Reset, so the integrator is zero at the start
   of every scan up to and including the arrival scan.

   Reset starts a new run (highest pressure, arrival and previous Ki
   cleared) and computes this scan normally. Enable FALSE or ParamError
   clears the run state, zeroes the outputs and sets IntegScale to 1.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Compute the schedule; FALSE clears the run state
       Reset           : BOOL             (* Start a new run this scan
       Target          : LREAL            (* Target pressure, clamped by the caller (MPa gauge)
       Actual          : LREAL            (* Measured pressure (MPa gauge)
       Position        : LREAL            (* Piston position (mm), positive into the cylinder
       PosLimit        : LREAL            (* Position at which the chamber is down to DeadVolume (mm)
       VelMax          : LREAL            (* Velocity clamp in pressure mode (mm/s)
       DecelMax        : LREAL            (* Deceleration assumed by the stop law (mm/s^2)
       StopVelMin      : LREAL            (* The stop law never clamps below this (mm/s)
       CycleTime       : LREAL            (* Task period (s)
       FilterTime      : LREAL            (* Pressure input low-pass time constant (s)
       MedianSize      : INT              (* Pressure input median window (3 or 5)
       DriveDeadTime   : LREAL            (* Drive dead time from command to response (s)
       DriveLag        : LREAL            (* Drive velocity-loop time constant (s)
       Crossover       : LREAL            (* Target loop crossover (rad/s)
       KiRatio         : LREAL            (* Ki / Kp (1/s)
       KpMin           : LREAL            (* Lower bound on Kp (mm/s per MPa)
       KpMax           : LREAL            (* Upper bound on Kp (mm/s per MPa)
       PistonArea      : LREAL            (* Piston area (mm^2)
       SalineVolume    : LREAL            (* Saline volume, a lower bound (mL)
       DeadVolume      : LREAL            (* Chamber volume at PosLimit, a lower bound (mL)
       HoseCompliance  : LREAL            (* Hose compliance, a lower bound (m^3/Pa)
       BulkModulus     : LREAL            (* Saline bulk modulus, an upper bound (Pa)
       OxygenRatio     : LREAL            (* Oxygen charge as STP volume per saline volume (-)
       Henry           : LREAL            (* Henry constant of oxygen in saline (mol/(m^3 Pa))
       GasTemp         : LREAL            (* Gas temperature (K)
       Patm            : LREAL            (* Atmospheric pressure (Pa)
       ArriveBand      : LREAL            (* |error| that releases the integrator (MPa)
     VAR_OUTPUT:
       Kp              : LREAL            (* Scheduled proportional gain (mm/s per MPa)
       Ki              : LREAL            (* Scheduled integral gain (mm/s per (MPa * s))
       VelLimit        : LREAL            (* Velocity clamp for the controller's VelMax (mm/s)
       Stiffness       : LREAL            (* Stiffness for the controller's own stop law (MPa/mm)
       IntegScale      : LREAL            (* Factor for the controller's stored integral this scan
       IntegHold       : BOOL             (* Hold the controller's integrator at zero this scan
       KEst            : LREAL            (* Stiffness bound (MPa/mm), for diagnostics
       Lag             : LREAL            (* Lag assumed by the stop law (s), for diagnostics
       ParamError      : BOOL             (* A setting is out of range; outputs are zero
     VAR:
       bArrived        : BOOL := FALSE    (* The error has come within ArriveBand since Reset
       rPRef           : LREAL := 0.0     (* Highest absolute pressure since Reset (Pa)
       rKiPrev         : LREAL := 0.0     (* Ki output on the previous scan; 0 after Reset
       rMedScans       : LREAL            (* Median filter delay (scans)
       rPAbs           : LREAL            (* Absolute pressure (Pa)
       rArea           : LREAL            (* Piston area (m^2)
       rVch            : LREAL            (* Chamber volume (m^3)
       rVlCfg          : LREAL            (* Configured saline volume (m^3)
       rVl             : LREAL            (* Saline volume used by the bound (m^3)
       rNCharge        : LREAL            (* Oxygen charge (mol)
       rNSat           : LREAL            (* Oxygen the saline holds at rPRef (mol)
       rNGas           : LREAL            (* Free gas (mol)
       rRT             : LREAL            (* Gas constant times temperature (J/mol)
       rVGas           : LREAL            (* Free gas volume (m^3)
       rC              : LREAL            (* Chamber compliance (m^3/Pa)
       rErr            : LREAL            (* Target - Actual (MPa)
       rDist           : LREAL            (* Travel that closes the error at KEst (mm)
       rStep           : LREAL            (* Velocity lost over Lag at DecelMax (mm/s)
       rVelStop        : LREAL            (* Stop-law velocity (mm/s)
   ============================================================================ *)

ParamError := (VelMax <= LREAL#0.0) OR (DecelMax <= LREAL#0.0) OR
              (StopVelMin < LREAL#0.0) OR (CycleTime <= LREAL#0.0) OR
              (FilterTime < LREAL#0.0) OR (DriveDeadTime < LREAL#0.0) OR
              (DriveLag < LREAL#0.0) OR (Crossover <= LREAL#0.0) OR
              (KiRatio < LREAL#0.0) OR (KpMin <= LREAL#0.0) OR
              (KpMax < KpMin) OR (PistonArea <= LREAL#0.0) OR
              (SalineVolume <= LREAL#0.0) OR (DeadVolume <= LREAL#0.0) OR
              (HoseCompliance < LREAL#0.0) OR (BulkModulus <= LREAL#0.0) OR
              (OxygenRatio < LREAL#0.0) OR (Henry < LREAL#0.0) OR
              (GasTemp <= LREAL#0.0) OR (Patm <= LREAL#0.0) OR
              (ArriveBand < LREAL#0.0);

IF (NOT Enable) OR Reset OR ParamError THEN
    bArrived := FALSE;
    rPRef := LREAL#0.0;
    rKiPrev := LREAL#0.0;
END_IF;

IF (NOT Enable) OR ParamError THEN
    Kp := LREAL#0.0;
    Ki := LREAL#0.0;
    VelLimit := LREAL#0.0;
    Stiffness := LREAL#0.0;
    IntegScale := LREAL#1.0;
    IntegHold := FALSE;
    KEst := LREAL#0.0;
    Lag := LREAL#0.0;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Lag of the measurement and the drive
   -------------------------------------------------------------------------- *)
IF MedianSize >= 5 THEN
    rMedScans := LREAL#2.0;
ELSE
    rMedScans := LREAL#1.0;
END_IF;
Lag := FilterTime + (rMedScans + LREAL#3.0) * CycleTime + DriveDeadTime + DriveLag;

(* --------------------------------------------------------------------------
   Stiffness bound, eq. (39), in SI units
   -------------------------------------------------------------------------- *)
rPAbs := MAX(Actual, LREAL#0.0) * LREAL#1.0E6 + Patm;
rPRef := MAX(rPRef, rPAbs);
rArea := PistonArea / LREAL#1.0E6;
rVch := MAX(rArea * ((PosLimit - Position) / LREAL#1.0E3), LREAL#0.0)
        + DeadVolume / LREAL#1.0E6;
rVlCfg := SalineVolume / LREAL#1.0E6;
rVl := MIN(rVlCfg, rVch);
rNCharge := OxygenRatio * rVlCfg * Patm / (LREAL#8.314 * LREAL#273.15);
rNSat := Henry * rPRef * rVlCfg;
rNGas := MAX(rNCharge - rNSat, LREAL#0.0);
rRT := LREAL#8.314 * GasTemp;
rVGas := rNGas * rRT / rPAbs;
rC := rVGas / rPAbs + rVl / BulkModulus + HoseCompliance;
KEst := rArea / rC / LREAL#1.0E9;

(* --------------------------------------------------------------------------
   Gains, eq. (40), and the integral rescale on a change of Ki
   -------------------------------------------------------------------------- *)
Kp := LIMIT(KpMin, Crossover / KEst, KpMax);
Ki := KiRatio * Kp;
IF (rKiPrev > LREAL#0.0) AND (Ki > LREAL#0.0) AND (Ki <> rKiPrev) THEN
    IntegScale := rKiPrev / Ki;
ELSE
    IntegScale := LREAL#1.0;
END_IF;
rKiPrev := Ki;

(* --------------------------------------------------------------------------
   Stop law with the lag, eq. (41)
   -------------------------------------------------------------------------- *)
rErr := Target - Actual;
rDist := ABS(rErr) / KEst;
rStep := DecelMax * Lag;
rVelStop := -rStep + SQRT(rStep * rStep + LREAL#2.0 * DecelMax * rDist);
VelLimit := MIN(VelMax, MAX(StopVelMin, rVelStop));
Stiffness := LREAL#1.0E-6;

(* --------------------------------------------------------------------------
   Integrator hold until the first arrival
   -------------------------------------------------------------------------- *)
IntegHold := NOT bArrived;
IF (NOT bArrived) AND (ABS(rErr) < ArriveBand) THEN
    bArrived := TRUE;
END_IF;
