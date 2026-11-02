(* ============================================================================
   FB_DigitalInputFilter - Digital Input Debounce Filter

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Debounces a digital input signal by requiring the input to be stable for
   a configurable time before the output changes.

   Features:
   - Configurable debounce time

   Variables (define in MWiec GUI):
     VAR_INPUT:
       RawInput            : BOOL             (* Raw digital input
       DebounceTime        : TIME := T#10MS   (* Debounce time
     VAR_OUTPUT:
       FilteredOutput      : BOOL             (* Debounced output
     VAR:
       tDebounce           : TON              (* Debounce timer (single call site)
       bDebounceIN         : BOOL             (* Latched IN value for tDebounce
       bLastRaw            : BOOL             (* Previous raw input state
       bPendingState       : BOOL             (* State we're debouncing toward
       bFirstScan          : BOOL := TRUE     (* First scan flag for initialization
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

(* First scan initialization - accept current input state immediately *)
IF bFirstScan THEN
    FilteredOutput := RawInput;
    bLastRaw := RawInput;
    bPendingState := RawInput;
    bDebounceIN := FALSE;
    bFirstScan := FALSE;
ELSE
    (* Detect raw input change - update pending state and force one-scan reset *)
    IF RawInput <> bLastRaw THEN
        bPendingState := RawInput;
        bDebounceIN := FALSE;  (* One scan at IN=FALSE resets TON before retrigger *)
    ELSIF bPendingState <> FilteredOutput THEN
        bDebounceIN := TRUE;
    ELSE
        bDebounceIN := FALSE;
    END_IF;

    bLastRaw := RawInput;
END_IF;

(* Single call site for tDebounce *)
tDebounce(IN := bDebounceIN, PT := DebounceTime);

IF tDebounce.Q AND NOT bFirstScan THEN
    FilteredOutput := bPendingState;
    bDebounceIN := FALSE;
END_IF;
