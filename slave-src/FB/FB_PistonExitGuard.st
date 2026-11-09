(* ============================================================================
   FB_PistonExitGuard - Piston Exit Prevention Guard

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Detects the piston backing out toward the cylinder exit while the chamber
   is pressurised. The cylinder holds pressurised oxygen micro-bubble
   solution; if the piston leaves the bore under pressure the contents spray
   out. This is a DETECTION-ONLY function block: it does NOT modify
   commands, it only raises a fault, and the fault's MC_Stop arrests the
   axis.

   SIGN CONVENTION (CRITICAL):
   - Positive direction: Toward cylinder EOT (compression, INTO cylinder)
   - Negative direction: Toward exit (retract, OUT OF cylinder)
   - Pressure pushes piston OUT (negative direction)

   GUARD BAND:
   Every position at or below CylinderEntry + GuardMargin. The band has no
   lower edge: a pressure measured below CylinderEntry means the chamber is
   still sealed there (CylinderEntry set too high, or the frame lost), and
   it is acted on.

   ARMED (output Armed):
   In the band, and the chamber counts as pressurised:
   - Pressure above PressureThreshold, or
   - SensorFault with the piston in the bore (at or above CylinderEntry):
     the pressure is unknown and is taken to be present. Below
     CylinderEntry the cylinder is open and nothing is assumed, so the axis
     can be homed and parked with a failed transducer.

   DANGEROUS CONDITION:
   While armed the block keeps the innermost position the piston has
   reached (rAnchor) and trips when the piston has backed out more than
   RetreatMax from it. The criterion is a distance and not a velocity: a
   retreat trips at any speed, however it is driven (a command, or the
   pressure overcoming the motor or the brake), and motion about a held
   position that stays within RetreatMax does not.

   rAnchor follows the position while the guard is not armed and moves in
   with the piston while it is. A trip takes the position as the new
   anchor, so FaultTriggered is TRUE for that scan only (FB_SafetyMonitor
   latches it), the fault resets once the axis is at rest, and the piston
   can then be moved in. Each acknowledged fault allows RetreatMax of
   travel again.

   The travel after a trip is RetreatMax plus the stopping distance of the
   fault's MC_Stop, v^2 / (2 * G_cfgStopDecel). GuardMargin must exceed it.

   Integration:
   - Called from PRG_Main safety monitoring section, in every state
   - FaultTriggered connects to FB_SafetyMonitor.ExitGuardTriggered, which
     reports FAULT_PISTON_EXIT, or FAULT_PRESSURE_SENSOR when the trip was
     on a failed transducer
   - Caller is responsible for transitioning to ST_FAULT when triggered

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the guard
       ActualPosition      : LREAL            (* Current position (mm)
       Pressure            : LREAL            (* Chamber pressure (MPa gauge)
       SensorFault         : BOOL             (* Pressure reading not valid (FB_PressureInput.SensorFault)
       CylinderEntry       : LREAL            (* Physical cylinder-entry position (mm). Below this
                                                 the piston is leaving the cylinder.
       GuardMargin         : LREAL            (* Safety margin above CylinderEntry (mm). The guard
                                                 watches every position at or below their sum.
       PressureThreshold   : LREAL            (* Pressure above which the chamber counts as pressurised (MPa gauge)
       RetreatMax          : LREAL            (* Distance the piston may back out while armed (mm); 0 or less trips on any retreat
     VAR_OUTPUT:
       FaultTriggered      : BOOL             (* Dangerous condition detected - transition to fault
       Armed               : BOOL             (* In the guard band with the chamber pressurised or its pressure unknown
       InCylinder          : BOOL             (* Position at or above CylinderEntry: the piston is in the bore
     VAR:
       bInBand             : BOOL             (* Position at or below CylinderEntry + GuardMargin
       bArmedNow           : BOOL             (* Armed condition of this scan; Armed holds the previous scan's until the anchor is updated
       rAnchor             : LREAL            (* Innermost position reached since the guard armed or last tripped (mm)
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled. Armed FALSE makes the first enabled
       scan take the position as the anchor. *)
    FaultTriggered := FALSE;
    Armed := FALSE;
    InCylinder := FALSE;
    RETURN;
END_IF;

(* Position against the cylinder entry and the guard boundary above it *)
InCylinder := (ActualPosition >= CylinderEntry);
bInBand := (ActualPosition <= CylinderEntry + GuardMargin);

(* --------------------------------------------------------------------------
   ARMING
   The chamber counts as pressurised on a reading above the threshold, or on
   a failed reading while the piston is in the bore.
   -------------------------------------------------------------------------- *)

bArmedNow := bInBand AND
             ((Pressure > PressureThreshold) OR (SensorFault AND InCylinder));

(* --------------------------------------------------------------------------
   ANCHOR
   The innermost position reached while armed. It follows the position
   until the guard arms (Armed is still the previous scan's value here, so
   the scan the guard arms in takes the position too) and moves only inward
   afterwards.
   -------------------------------------------------------------------------- *)

IF (NOT bArmedNow) OR (NOT Armed) OR (ActualPosition > rAnchor) THEN
    rAnchor := ActualPosition;
END_IF;
Armed := bArmedNow;

(* --------------------------------------------------------------------------
   DANGEROUS CONDITION DETECTION

   Detect: armed + the piston backed out more than RetreatMax from the anchor

   Sign convention:
   - rAnchor - ActualPosition > 0 = piston has moved OUT of the cylinder
   - Armed + moved out beyond RetreatMax = leaving under pressure = DANGER

   The trip re-anchors at the position, so the fault is one event and the
   travel of the stop that follows is not counted as a further retreat
   until it exceeds RetreatMax again.
   -------------------------------------------------------------------------- *)

FaultTriggered := Armed AND
                  ((rAnchor - ActualPosition) > MAX(RetreatMax, LREAL#0.0));

IF FaultTriggered THEN
    rAnchor := ActualPosition;
END_IF;
