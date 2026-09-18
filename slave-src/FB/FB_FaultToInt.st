(* ============================================================================
   FB_FaultToInt - Convert Fault Code to Integer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_FaultCode enumeration value to its integer
   representation (0-7) for bit encoding operations.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum parameters.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Fault           : E_FaultCode      (* Fault code to convert
     VAR_OUTPUT:
       Value           : INT              (* Integer representation 0-7
   ============================================================================ *)

IF Fault = E_FaultCode#FAULT_NONE THEN
    Value := 0;
ELSIF Fault = E_FaultCode#FAULT_HANDSHAKE THEN
    Value := 1;
ELSIF Fault = E_FaultCode#FAULT_DRIVE THEN
    Value := 2;
ELSIF Fault = E_FaultCode#FAULT_POSITION THEN
    Value := 3;
ELSIF Fault = E_FaultCode#FAULT_HOMING_REQ THEN
    Value := 4;
ELSIF Fault = E_FaultCode#FAULT_PISTON_EXIT THEN
    Value := 5;
ELSIF Fault = E_FaultCode#FAULT_LIMIT_SWITCH THEN
    Value := 6;
ELSIF Fault = E_FaultCode#FAULT_ENCODER THEN
    Value := 7;
ELSE
    Value := 0;
END_IF;
