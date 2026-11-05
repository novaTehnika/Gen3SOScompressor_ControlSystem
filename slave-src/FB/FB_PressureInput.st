(* ============================================================================
   FB_PressureInput - 4-20 mA Pressure Transducer Scaling and Supervision

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts the filtered voltage across the transducer's shunt resistor to
   loop current and then to gauge pressure. The transducer is a 4-20 mA
   loop: ZeroMa reads RangeMin and ZeroMa + SpanMa reads RangeMax, linear
   between. Scaling is not clamped, so a reading slightly below zero at
   atmosphere is reported as it is.

   Live-zero supervision: a loop current outside [MinMa, MaxMa] for longer
   than FaultTime raises SensorFault. An open loop or unpowered transducer
   reads 0 mA; a shorted shunt or failed transmitter reads above span.
   The input is already median filtered, so a sustained excursion is real.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Enable scaling and supervision
       Volts           : LREAL            (* Filtered voltage across the shunt (V)
       ShuntOhms       : LREAL            (* Shunt resistance (ohm)
       ZeroMa          : LREAL            (* Loop current at RangeMin (mA), nominally 4
       SpanMa          : LREAL            (* Loop current span (mA), nominally 16
       RangeMin        : LREAL            (* Pressure at ZeroMa (MPa gauge)
       RangeMax        : LREAL            (* Pressure at ZeroMa + SpanMa (MPa gauge)
       MinMa           : LREAL            (* Below this the loop is broken (mA)
       MaxMa           : LREAL            (* Above this the loop is over range (mA)
       FaultTime       : TIME             (* Out-of-range must persist this long
     VAR_OUTPUT:
       PressureMPa     : LREAL            (* Scaled pressure (MPa gauge)
       SensorFault     : BOOL             (* Loop current outside [MinMa, MaxMa] sustained for FaultTime
     VAR:
       rLoopMa         : LREAL            (* Loop current (mA)
       bOutOfRange     : BOOL             (* Loop current outside [MinMa, MaxMa] this scan
       bParamError     : BOOL             (* ShuntOhms or SpanMa not positive
       tonOutOfRange   : TON              (* Out-of-range persistence timer
   ============================================================================ *)

IF NOT Enable THEN
    PressureMPa := LREAL#0.0;
    SensorFault := FALSE;
    tonOutOfRange(IN := FALSE, PT := FaultTime);
    RETURN;
END_IF;

bParamError := (ShuntOhms <= LREAL#0.0) OR (SpanMa <= LREAL#0.0);
IF bParamError THEN
    (* Fail to "sensor broken", never to a plausible pressure *)
    PressureMPa := LREAL#0.0;
    bOutOfRange := TRUE;
ELSE
    rLoopMa := Volts / ShuntOhms * LREAL#1000.0;
    PressureMPa := RangeMin + (rLoopMa - ZeroMa) / SpanMa * (RangeMax - RangeMin);
    bOutOfRange := (rLoopMa < MinMa) OR (rLoopMa > MaxMa);
END_IF;

tonOutOfRange(IN := bOutOfRange, PT := FaultTime);
SensorFault := tonOutOfRange.Q;
