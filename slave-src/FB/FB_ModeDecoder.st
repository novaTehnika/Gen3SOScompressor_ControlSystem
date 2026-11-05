(* ============================================================================
   FB_ModeDecoder - Mode Command Decoder

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Debounces the mode code from the Modbus mode register (G_mbModeCode) and
   decodes it into an E_OperatingMode enumeration value. A new code is
   accepted once the register has held it for DebounceTime.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Enable the decoder
       ModeCode        : UINT             (* Mode code from the mode register
       DebounceTime    : TIME := T#50MS   (* Time a new code must be held
     VAR_OUTPUT:
       DecodedMode     : E_OperatingMode  (* Decoded operating mode
       ModeStable      : BOOL             (* Mode has been stable (no change from last scan)
     VAR:
       nModeValue      : INT              (* Numeric mode value 0-7
       nPreviousValue  : INT              (* Previous scan mode value
       nFilteredCode   : UINT             (* Debounced mode code
       nPendingCode    : UINT             (* Code being debounced toward
       nLastRaw        : UINT             (* Previous scan register value
       bFirstScan      : BOOL := TRUE     (* Accept the register value on the first scan
       tDebounce       : TON              (* Debounce timer (single call site)
       bDebounceIN     : BOOL             (* Latched IN value for tDebounce
       fbIntToMode     : FB_IntToMode     (* Code -> enum helper
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset outputs when disabled *)
    DecodedMode := E_OperatingMode#MODE_IDLE;
    nModeValue := 0;
    ModeStable := FALSE;
    nPreviousValue := 0;
    RETURN;
END_IF;

(* Debounce: a register change restarts the timer, the pending code is
   accepted when the timer expires *)
IF bFirstScan THEN
    bFirstScan := FALSE;
    nFilteredCode := ModeCode;
    nPendingCode := ModeCode;
    nLastRaw := ModeCode;
    bDebounceIN := FALSE;
ELSIF ModeCode <> nLastRaw THEN
    nPendingCode := ModeCode;
    bDebounceIN := FALSE;  (* One scan at IN=FALSE resets TON before retrigger *)
ELSE
    bDebounceIN := (nPendingCode <> nFilteredCode);
END_IF;
nLastRaw := ModeCode;

tDebounce(IN := bDebounceIN, PT := DebounceTime);

IF tDebounce.Q THEN
    nFilteredCode := nPendingCode;
    bDebounceIN := FALSE;
END_IF;

(* Decode the debounced code. Codes above 7 (the master mirrors fault codes
   in this register) decode to MODE_IDLE and read as value 0, as in FB_IntToMode. *)
fbIntToMode(Value := nFilteredCode);
DecodedMode := fbIntToMode.Mode;

IF nFilteredCode <= UINT#7 THEN
    nModeValue := UINT_TO_INT(nFilteredCode);
ELSE
    nModeValue := 0;
END_IF;

(* Mode is stable if value matches previous scan *)
ModeStable := (nModeValue = nPreviousValue);

(* Save for next scan *)
nPreviousValue := nModeValue;
