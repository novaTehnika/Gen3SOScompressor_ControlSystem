(* ============================================================================
   FB_AnalogProcessor - Analog I/O Scaling and Processing

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Handles scaling of analog input signals for position, velocity, and torque
   reference based on the current operating mode.

   Analog Input Scaling:
   - Analog input: LREAL voltage (-10.0 to +10.0 V)
   - Scaled to engineering units based on mode and configuration

   Position Mode uses two-stage mapping for consistency with position feedback:
   - Stage 1: In-cylinder region (higher resolution)
   - Stage 2: Out-of-cylinder region (lower resolution)

   Modes:
   - Position (010): Scale to mm using two-stage mapping
   - Velocity (011): Scale to mm/s (linear)
   - Torque (100): Scale to % of rated torque (linear)

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the processor
       RawAnalogInput      : LREAL            (* Analog input voltage (-10.0 to +10.0 V)
       CurrentMode         : E_OperatingMode  (* Current operating mode
       Stage1PosMin        : LREAL            (* Stage 1 position start (mm)
       Stage1PosMax        : LREAL            (* Stage 1 position end (mm) = transition pos
       Stage1VoltMin       : LREAL            (* Stage 1 voltage start (V)
       Stage1VoltMax       : LREAL            (* Stage 1 voltage end (V) = transition volt
       Stage2PosMin        : LREAL            (* Stage 2 position start (mm) = transition pos
       Stage2PosMax        : LREAL            (* Stage 2 position end (mm)
       Stage2VoltMin       : LREAL            (* Stage 2 voltage start (V) = transition volt
       Stage2VoltMax       : LREAL            (* Stage 2 voltage end (V)
       VelMin              : LREAL            (* Velocity at -10V (mm/s)
       VelMax              : LREAL            (* Velocity at +10V (mm/s)
       TorqueMin           : LREAL            (* Torque at -10V (%)
       TorqueMax           : LREAL            (* Torque at +10V (%)
     VAR_OUTPUT:
       ScaledPosition      : LREAL            (* Scaled position reference (mm)
       ScaledVelocity      : LREAL            (* Scaled velocity reference (mm/s)
       ScaledTorque        : LREAL            (* Scaled torque reference (%)
       ActiveReference     : LREAL            (* Currently active reference value
       NormalizedInput     : LREAL            (* Input normalized to 0.0-1.0 range
       InputVoltage        : LREAL            (* Calculated input voltage (-10 to +10)
       InStage1            : BOOL             (* Position input is in stage 1 region
       InStage2            : BOOL             (* Position input is in stage 2 region
     VAR:
       rNormalized         : LREAL            (* Normalized value 0.0 to 1.0
       rStageNormalized    : LREAL            (* Normalized value within stage
     VAR CONSTANT:
       VOLT_MIN            : LREAL := -10.0
       VOLT_MAX            : LREAL := 10.0
       VOLT_RANGE          : LREAL := 20.0
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    ScaledPosition := LREAL#0.0;
    ScaledVelocity := LREAL#0.0;
    ScaledTorque := LREAL#0.0;
    ActiveReference := LREAL#0.0;
    NormalizedInput := LREAL#0.5;
    InputVoltage := LREAL#0.0;
    InStage1 := FALSE;
    InStage2 := FALSE;
    RETURN;
END_IF;

(* Calculate normalized input and voltage directly from LREAL analog input *)
InputVoltage := RawAnalogInput;
rNormalized := (InputVoltage - VOLT_MIN) / VOLT_RANGE;
NormalizedInput := rNormalized;

(* Scale velocity and torque (simple linear mapping) *)
ScaledVelocity := VelMin + (rNormalized * (VelMax - VelMin));
ScaledTorque := TorqueMin + (rNormalized * (TorqueMax - TorqueMin));

(* Scale position using two-stage mapping (inverse of FB_PositionMapping) *)
(* Determine which stage based on input voltage *)
(* Use < (not <=) so boundary value is handled by stage 2 only *)
IF InputVoltage < Stage1VoltMax THEN
    (* Stage 1: In-cylinder region *)
    InStage1 := TRUE;
    InStage2 := FALSE;

    (* Calculate normalized position within stage 1 *)
    IF (Stage1VoltMax - Stage1VoltMin) > LREAL#0.0 THEN
        rStageNormalized := (InputVoltage - Stage1VoltMin) / (Stage1VoltMax - Stage1VoltMin);
    ELSE
        rStageNormalized := LREAL#0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rStageNormalized := LIMIT(LREAL#0.0, rStageNormalized, LREAL#1.0);

    (* Calculate position for stage 1 *)
    ScaledPosition := Stage1PosMin + (rStageNormalized * (Stage1PosMax - Stage1PosMin));

ELSE
    (* Stage 2: Out-of-cylinder region *)
    InStage1 := FALSE;
    InStage2 := TRUE;

    (* Calculate normalized position within stage 2 *)
    IF (Stage2VoltMax - Stage2VoltMin) > LREAL#0.0 THEN
        rStageNormalized := (InputVoltage - Stage2VoltMin) / (Stage2VoltMax - Stage2VoltMin);
    ELSE
        rStageNormalized := LREAL#0.0;
    END_IF;

    (* Clamp to 0.0-1.0 range *)
    rStageNormalized := LIMIT(LREAL#0.0, rStageNormalized, LREAL#1.0);

    (* Calculate position for stage 2 *)
    ScaledPosition := Stage2PosMin + (rStageNormalized * (Stage2PosMax - Stage2PosMin));
END_IF;

(* Select active reference based on mode *)
IF CurrentMode = E_OperatingMode#MODE_POSITION THEN
    ActiveReference := ScaledPosition;

ELSIF CurrentMode = E_OperatingMode#MODE_VELOCITY THEN
    ActiveReference := ScaledVelocity;

ELSIF CurrentMode = E_OperatingMode#MODE_TORQUE THEN
    ActiveReference := ScaledTorque;

ELSE
    ActiveReference := LREAL#0.0;
END_IF;
