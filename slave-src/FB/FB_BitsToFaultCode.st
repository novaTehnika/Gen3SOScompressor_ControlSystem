(* ============================================================================
   FB_BitsToFaultCode - Convert 3 Digital Input Bits to Fault Code

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts 3 digital input bits to the corresponding E_FaultCode
   enumeration value. Used during fault reset to decode the master's
   fault acknowledgement from DI0-DI2.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum return types.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR_OUTPUT:
       Fault           : E_FaultCode      (* Decoded fault code
     VAR:
       nValue          : INT              (* Calculated integer value
   ============================================================================ *)

(* Calculate integer value from bits *)
nValue := 0;
IF Bit0 THEN nValue := nValue + 1; END_IF;
IF Bit1 THEN nValue := nValue + 2; END_IF;
IF Bit2 THEN nValue := nValue + 4; END_IF;

(* Convert integer to fault code *)
IF nValue = 0 THEN
    Fault := E_FaultCode#FAULT_NONE;
ELSIF nValue = 1 THEN
    Fault := E_FaultCode#FAULT_HANDSHAKE;
ELSIF nValue = 2 THEN
    Fault := E_FaultCode#FAULT_DRIVE;
ELSIF nValue = 3 THEN
    Fault := E_FaultCode#FAULT_POSITION;
ELSIF nValue = 4 THEN
    Fault := E_FaultCode#FAULT_HOMING_REQ;
ELSIF nValue = 5 THEN
    Fault := E_FaultCode#FAULT_PISTON_EXIT;
ELSIF nValue = 6 THEN
    Fault := E_FaultCode#FAULT_LIMIT_SWITCH;
ELSIF nValue = 7 THEN
    Fault := E_FaultCode#FAULT_ENCODER;
ELSE
    Fault := E_FaultCode#FAULT_NONE;
END_IF;
