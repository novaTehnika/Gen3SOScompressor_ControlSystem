(* ============================================================================
   FB_ModeToInt - Convert Operating Mode to Integer

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts an E_OperatingMode enumeration value to its integer
   representation (0-7) for bit encoding operations.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum parameters.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Operating mode to convert
     VAR_OUTPUT:
       Value           : INT              (* Integer representation 0-7
   ============================================================================ *)

IF Mode = E_OperatingMode#MODE_IDLE THEN
    Value := 0;
ELSIF Mode = E_OperatingMode#MODE_BRAKE_HOLD THEN
    Value := 1;
ELSIF Mode = E_OperatingMode#MODE_POSITION THEN
    Value := 2;
ELSIF Mode = E_OperatingMode#MODE_VELOCITY THEN
    Value := 3;
ELSIF Mode = E_OperatingMode#MODE_TORQUE THEN
    Value := 4;
ELSIF Mode = E_OperatingMode#MODE_GO_HOME THEN
    Value := 5;
ELSIF Mode = E_OperatingMode#MODE_HOME_LIMIT THEN
    Value := 6;
ELSIF Mode = E_OperatingMode#MODE_RESERVED_111 THEN
    Value := 7;
ELSE
    Value := 0;
END_IF;
