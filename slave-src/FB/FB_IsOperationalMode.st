(* ============================================================================
   FB_IsOperationalMode - Check if Mode is an Operational Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Checks if the given operating mode is an operational mode that requires
   homing to have been completed before entry. Operational modes are those
   that perform motion control using absolute position references.

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect that rejects user-enum parameters on FUNCTIONs
   with E2001 "circular assembly references". FBs with the same signature
   compile correctly.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Mode to check
     VAR_OUTPUT:
       Result          : BOOL             (* TRUE if mode is operational
   ============================================================================ *)

Result := (Mode = E_OperatingMode#MODE_BRAKE_HOLD) OR
          (Mode = E_OperatingMode#MODE_POSITION) OR
          (Mode = E_OperatingMode#MODE_VELOCITY) OR
          (Mode = E_OperatingMode#MODE_TORQUE);
