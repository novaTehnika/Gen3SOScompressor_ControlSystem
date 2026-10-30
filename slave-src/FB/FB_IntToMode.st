(* ============================================================================
   FB_IntToMode - Convert Integer to Operating Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts the mode code from the Modbus mode register to the
   corresponding E_OperatingMode enumeration value. Unknown values decode
   to MODE_IDLE.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum return types.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Value           : UINT             (* Mode code
     VAR_OUTPUT:
       Mode            : E_OperatingMode  (* Decoded operating mode
   ============================================================================ *)

IF Value = UINT#0 THEN
    Mode := E_OperatingMode#MODE_IDLE;
ELSIF Value = UINT#1 THEN
    Mode := E_OperatingMode#MODE_BRAKE_HOLD;
ELSIF Value = UINT#2 THEN
    Mode := E_OperatingMode#MODE_POSITION;
ELSIF Value = UINT#3 THEN
    Mode := E_OperatingMode#MODE_VELOCITY;
ELSIF Value = UINT#4 THEN
    Mode := E_OperatingMode#MODE_TORQUE;
ELSIF Value = UINT#5 THEN
    Mode := E_OperatingMode#MODE_GO_HOME;
ELSIF Value = UINT#6 THEN
    Mode := E_OperatingMode#MODE_HOME_LIMIT;
ELSIF Value = UINT#7 THEN
    Mode := E_OperatingMode#MODE_PRESSURE;
ELSE
    Mode := E_OperatingMode#MODE_IDLE;
END_IF;
