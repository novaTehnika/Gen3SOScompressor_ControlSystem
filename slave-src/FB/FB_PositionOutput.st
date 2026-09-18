(* ============================================================================
   FB_PositionOutput - Two-Stage Position Output Signal Mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Maps the actual position to analog output voltage using a two-stage
   scaling scheme for improved resolution in the critical in-cylinder region.
   This signal is sent to the master controller.

   Two-Stage Mapping:
   - Stage 1: In-cylinder region (higher resolution)
     Position 0-200mm maps to -10V to +5V (15V range for 200mm)
   - Stage 2: Out-of-cylinder region (lower resolution)
     Position 200-305mm maps to +5V to +10V (5V range for 105mm)

   This provides approximately 3x better resolution in the working region
   where the piston is inside the cylinder.

   Output:
   - Analog output: LREAL voltage (-10.0 to +10.0 V)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the mapper
       ActualPosition      : LREAL            (* Current position in mm
       Stage1PosMin        : LREAL            (* Stage 1 position start (mm)
       Stage1PosMax        : LREAL            (* Stage 1 position end (mm)
       Stage1VoltMin       : LREAL            (* Stage 1 voltage start (V)
       Stage1VoltMax       : LREAL            (* Stage 1 voltage end (V)
       Stage2PosMin        : LREAL            (* Stage 2 position start (mm)
       Stage2PosMax        : LREAL            (* Stage 2 position end (mm)
       Stage2VoltMin       : LREAL            (* Stage 2 voltage start (V)
       Stage2VoltMax       : LREAL            (* Stage 2 voltage end (V)
     VAR_OUTPUT:
       RawAnalogOutput     : LREAL            (* Analog output voltage (-10.0 to +10.0 V)
       OutputVoltage       : LREAL            (* Calculated output voltage
       InStage1            : BOOL             (* Position is in stage 1 region
       InStage2            : BOOL             (* Position is in stage 2 region
     VAR:
       rVoltage            : LREAL            (* Calculated voltage
       rNormalized         : LREAL            (* Normalized position within stage
     VAR CONSTANT:
       VOLT_MIN            : LREAL := -10.0
       VOLT_MAX            : LREAL := 10.0
       VOLT_RANGE          : LREAL := 20.0
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    RawAnalogOutput := LREAL#0.0;
    OutputVoltage := LREAL#0.0;
    InStage1 := FALSE;
    InStage2 := FALSE;
    RETURN;
END_IF;

(* Determine which stage the position falls into *)
IF ActualPosition <= Stage1PosMax THEN
    (* Stage 1: In-cylinder region *)
    InStage1 := TRUE;
    InStage2 := FALSE;

    (* Calculate normalized position within stage 1 *)
    IF (Stage1PosMax - Stage1PosMin) > LREAL#0.0 THEN
        rNormalized := (ActualPosition - Stage1PosMin) / (Stage1PosMax - Stage1PosMin);
    ELSE
        rNormalized := LREAL#0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rNormalized := LIMIT(LREAL#0.0, rNormalized, LREAL#1.0);

    (* Calculate voltage for stage 1 *)
    rVoltage := Stage1VoltMin + (rNormalized * (Stage1VoltMax - Stage1VoltMin));

ELSE
    (* Stage 2: Out-of-cylinder region *)
    InStage1 := FALSE;
    InStage2 := TRUE;

    (* Calculate normalized position within stage 2 *)
    IF (Stage2PosMax - Stage2PosMin) > LREAL#0.0 THEN
        rNormalized := (ActualPosition - Stage2PosMin) / (Stage2PosMax - Stage2PosMin);
    ELSE
        rNormalized := LREAL#0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rNormalized := LIMIT(LREAL#0.0, rNormalized, LREAL#1.0);

    (* Calculate voltage for stage 2 *)
    rVoltage := Stage2VoltMin + (rNormalized * (Stage2VoltMax - Stage2VoltMin));
END_IF;

(* Clamp voltage to valid range *)
rVoltage := LIMIT(VOLT_MIN, rVoltage, VOLT_MAX);
OutputVoltage := rVoltage;
RawAnalogOutput := rVoltage;
