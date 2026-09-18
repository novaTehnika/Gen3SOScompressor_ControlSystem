(* ============================================================================
   FB_PistonExitGuard - Piston Exit Prevention Guard (Simplified)

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Monitors for dangerous condition where external pressure is pushing the
   piston out of the cylinder despite motor resistance. This is a DETECTION-ONLY
   function block - it does NOT modify commands, only triggers a fault when
   the dangerous condition is detected.

   The cylinder contains pressurized oxygen micro-bubble solution. If the piston
   exits the cylinder while pressurized, dangerous spray would result.

   SIGN CONVENTION (CRITICAL):
   - Positive direction: Toward cylinder EOT (compression, INTO cylinder)
   - Negative direction: Toward exit (retract, OUT OF cylinder)
   - Pressure pushes piston OUT (negative direction)
   - Motor RESISTS with POSITIVE torque to hold piston IN

   DANGEROUS CONDITION:
   When near the exit boundary, if:
   - Motor is applying positive torque (resisting/pushing into cylinder)
   - But actual velocity is negative (moving toward exit anyway)
   Then external pressure is overcoming motor resistance - FAULT!

   Integration:
   - Called from PRG_Main safety monitoring section
   - FaultTriggered connects to FB_SafetyMonitor.ExitGuardTriggered
   - Caller is responsible for transitioning to ST_FAULT when triggered

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the guard
       ActualPosition      : LREAL            (* Current position (mm)
       ActualVelocity      : LREAL            (* Current velocity (mm/s)
       ActualTorque        : LREAL            (* Current torque (%)
       CylinderEntry       : LREAL            (* Physical cylinder-entry position (mm). Below this
                                                 the piston is leaving the cylinder.
       GuardMargin         : LREAL            (* Safety margin above CylinderEntry (mm). Position
                                                 within this band triggers torque/velocity monitoring.
       VelocityThreshold   : LREAL            (* Motion detection threshold (mm/s)
       TorqueThreshold     : LREAL            (* Torque threshold for detection (%)
     VAR_OUTPUT:
       FaultTriggered      : BOOL             (* Dangerous condition detected - transition to fault
       GuardActive         : BOOL             (* Guard is actively monitoring
       InDangerZone        : BOOL             (* Position is within GuardMargin of cylinder entry
       ApproachingLimit    : BOOL             (* Moving toward exit limit
     VAR:
       rGuardBoundary      : LREAL            (* Boundary = CylinderEntry + GuardMargin
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    FaultTriggered := FALSE;
    GuardActive := FALSE;
    InDangerZone := FALSE;
    ApproachingLimit := FALSE;
    RETURN;
END_IF;

(* Calculate guard boundary: margin above the physical cylinder entry *)
rGuardBoundary := CylinderEntry + GuardMargin;

(* Determine position and motion status *)
InDangerZone := (ActualPosition <= rGuardBoundary);
ApproachingLimit := (ActualVelocity < -VelocityThreshold);  (* Negative = toward exit *)

(* Guard is always active when enabled *)
GuardActive := TRUE;

(* Reset fault before evaluation *)
FaultTriggered := FALSE;

(* --------------------------------------------------------------------------
   DANGEROUS CONDITION DETECTION

   Detect: Near exit + positive torque (resisting) + negative velocity (moving out)

   This indicates external pressure is pushing the piston out despite the motor
   applying resistance (positive torque). The motor is trying to hold/push the
   piston into the cylinder but the pressure is winning.

   Sign convention:
   - Positive torque = motor pushing INTO cylinder (resisting exit)
   - Negative velocity = piston moving OUT of cylinder (toward exit)
   - Both together near exit = pressure overcoming motor = DANGER
   -------------------------------------------------------------------------- *)

IF InDangerZone THEN
    (* In danger zone - check for anomalous pressure condition *)

    IF (ActualTorque > TorqueThreshold) AND          (* Motor applying positive torque - resisting *)
       (ActualVelocity < -VelocityThreshold) THEN    (* But still moving toward exit *)
        (* DANGEROUS: Pressure overcoming motor resistance *)
        FaultTriggered := TRUE;
    END_IF;
END_IF;
