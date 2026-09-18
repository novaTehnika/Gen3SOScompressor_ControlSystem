(* ============================================================================
   FB_FaultToBits - Convert Fault Code to 3 Output Bits

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_FaultCode enumeration value to 3 digital output bits.
   Uses FB_FaultToInt to get the integer value, then extracts individual bits.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to convert
     VAR_OUTPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR:
       nValue          : INT              (* Integer fault value
       byteValue       : BYTE             (* Byte fault value
       fbFaultToInt    : FB_FaultToInt    (* Enum -> int helper (was FN_FaultToInt)
*)

fbFaultToInt(Fault := Fault);
nValue := fbFaultToInt.Value;
byteValue := INT_TO_BYTE(nValue);
Bit0 := (byteValue AND INT_TO_BYTE(1)) <> BYTE#0;
Bit1 := (byteValue AND INT_TO_BYTE(2)) <> BYTE#0;
Bit2 := (byteValue AND INT_TO_BYTE(4)) <> BYTE#0;