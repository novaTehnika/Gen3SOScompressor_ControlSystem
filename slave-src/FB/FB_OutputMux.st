(* ============================================================================
   FB_OutputMux - Digital Output Multiplexer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Multiplexes DO0-DO2 outputs between mode confirmation and fault code:
   - Normal mode: DO0-DO2 = Mode confirmation bits
   - Fault mode:  DO0-DO2 = Fault code bits

   Uses centralized conversion via FB_FaultToBits and FB_ModeToBits.

   Fault Codes (3-bit):
   000 = No fault / cleared
   001 = Handshake timeout/mismatch
   010 = Drive fault
   011 = Position limit exceeded
   100 = Homing requirement not met
   101 = Piston exit prevention triggered
   110 = Limit switch fault
   111 = Encoder/absolute position fault

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable the function block
       FaultActive         : BOOL             (* Fault state is active
       FaultCode           : E_FaultCode      (* Current fault code
       ConfirmedMode       : E_OperatingMode  (* Confirmed operating mode
     VAR_OUTPUT:
       DO0_Output          : BOOL             (* Output for DO0
       DO1_Output          : BOOL             (* Output for DO1
       DO2_Output          : BOOL             (* Output for DO2
       FaultActiveOutput   : BOOL             (* Output for doFaultActive (DO4)
     VAR:
       fbFaultToBits       : FB_FaultToBits   (* Fault code to bits converter
       fbModeToBits        : FB_ModeToBits    (* Mode to bits converter
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* All outputs low when disabled *)
    DO0_Output := FALSE;
    DO1_Output := FALSE;
    DO2_Output := FALSE;
    FaultActiveOutput := FALSE;
    RETURN;
END_IF;

(* Set fault active output *)
FaultActiveOutput := FaultActive;

IF FaultActive THEN
    (* Fault mode: Output fault code on DO0-DO2 *)
    fbFaultToBits(Fault := FaultCode);
    DO0_Output := fbFaultToBits.Bit0;
    DO1_Output := fbFaultToBits.Bit1;
    DO2_Output := fbFaultToBits.Bit2;
ELSE
    (* Normal mode: Output mode confirmation bits *)
    fbModeToBits(Mode := ConfirmedMode);
    DO0_Output := fbModeToBits.Bit0;
    DO1_Output := fbModeToBits.Bit1;
    DO2_Output := fbModeToBits.Bit2;
END_IF;
