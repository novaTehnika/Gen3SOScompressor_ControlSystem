(* ============================================================================
   FB_BitsToMode - Convert 3 Digital Input Bits to Operating Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts 3 digital input bits to the corresponding E_OperatingMode
   enumeration value. Calculates the integer value from bits then
   maps to the corresponding mode.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum return types.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Bit0            : BOOL             (* LSB
       Bit1            : BOOL
       Bit2            : BOOL             (* MSB
     VAR_OUTPUT:
       Mode            : E_OperatingMode  (* Decoded operating mode
     VAR:
       nValue          : INT              (* Calculated integer value
   ============================================================================ *)

(* Calculate integer value from bits *)
nValue := 0;
IF Bit0 THEN nValue := nValue + 1; END_IF;
IF Bit1 THEN nValue := nValue + 2; END_IF;
IF Bit2 THEN nValue := nValue + 4; END_IF;

(* Convert integer to mode *)
IF nValue = 0 THEN
    Mode := E_OperatingMode#MODE_IDLE;
ELSIF nValue = 1 THEN
    Mode := E_OperatingMode#MODE_BRAKE_HOLD;
ELSIF nValue = 2 THEN
    Mode := E_OperatingMode#MODE_POSITION;
ELSIF nValue = 3 THEN
    Mode := E_OperatingMode#MODE_VELOCITY;
ELSIF nValue = 4 THEN
    Mode := E_OperatingMode#MODE_TORQUE;
ELSIF nValue = 5 THEN
    Mode := E_OperatingMode#MODE_GO_HOME;
ELSIF nValue = 6 THEN
    Mode := E_OperatingMode#MODE_HOME_LIMIT;
ELSIF nValue = 7 THEN
    Mode := E_OperatingMode#MODE_RESERVED_111;
ELSE
    Mode := E_OperatingMode#MODE_IDLE;
END_IF;
