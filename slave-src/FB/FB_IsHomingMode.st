(* ============================================================================
   FB_IsHomingMode - Check if Mode is a Homing Mode

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Checks if the given operating mode is one of the homing modes
   (Go Home or Home to Negative Overtravel / Home Reference).

   NOTE: Implemented as an FB (not an FN) to work around a MotionWorks IEC
   native-code-gen defect on FUNCTIONs with user-enum parameters.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Mode            : E_OperatingMode  (* Mode to check
     VAR_OUTPUT:
       Result          : BOOL             (* TRUE if mode is a homing mode
   ============================================================================ *)

Result := (Mode = E_OperatingMode#MODE_GO_HOME) OR
          (Mode = E_OperatingMode#MODE_HOME_LIMIT);
