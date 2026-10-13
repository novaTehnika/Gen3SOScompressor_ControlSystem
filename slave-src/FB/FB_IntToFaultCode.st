(* ============================================================================
   FB_IntToFaultCode - Convert Integer to Fault Code

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Converts the integer value of a Modbus register to the corresponding
   E_FaultCode enumeration value. Used during fault reset to decode the
   master's fault acknowledgement mirrored in G_mbModeCode. Unknown values
   decode to FAULT_NONE, which never matches an active fault.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum return types.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Value           : UINT             (* Register value
     VAR_OUTPUT:
       Fault           : E_FaultCode      (* Decoded fault code
   ============================================================================ *)

IF Value = UINT#0 THEN
    Fault := E_FaultCode#FAULT_NONE;
ELSIF Value = UINT#1 THEN
    Fault := E_FaultCode#FAULT_HANDSHAKE;
ELSIF Value = UINT#2 THEN
    Fault := E_FaultCode#FAULT_DRIVE;
ELSIF Value = UINT#3 THEN
    Fault := E_FaultCode#FAULT_POSITION;
ELSIF Value = UINT#4 THEN
    Fault := E_FaultCode#FAULT_COMM;
ELSIF Value = UINT#5 THEN
    Fault := E_FaultCode#FAULT_PISTON_EXIT;
ELSIF Value = UINT#6 THEN
    Fault := E_FaultCode#FAULT_LIMIT_SWITCH;
ELSIF Value = UINT#7 THEN
    Fault := E_FaultCode#FAULT_ENCODER;
ELSE
    Fault := E_FaultCode#FAULT_NONE;
END_IF;
