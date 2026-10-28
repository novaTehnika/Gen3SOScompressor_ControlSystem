(* ============================================================================
   FB_FaultResetHandler - Fault Reset Validation

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Validates fault reset conditions from the master controller.
   Pure combinational check - no state machine, no timers.

   Fault reset is valid when ALL conditions are met (level-based):
   - FaultReset is HIGH (level, not edge)
   - MotionEnable is LOW
   - System is in a fault state
   - Active fault code is not FAULT_NONE
   - Decoded mode code is stable (no change from previous scan)
   - Master has mirrored the fault code in G_mbModeCode (MasterFaultCode matches ActiveFaultCode)

   NOTE: All inputs should be pre-filtered/debounced by the caller.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable           : BOOL             (* Enable fault reset handler
       FaultReset       : BOOL             (* Filtered fault reset signal from master
       MotionEnable     : BOOL             (* Filtered motion enable signal
       InFaultState     : BOOL             (* TRUE when system is in a fault state
       CodeStable       : BOOL             (* Decoded mode code stable (no change from previous scan)
       ActiveFaultCode  : E_FaultCode      (* Active fault code for mirroring check
       MasterFaultCode  : E_FaultCode      (* Fault acknowledgement decoded from G_mbModeCode
     VAR_OUTPUT:
       FaultResetValid  : BOOL             (* TRUE when all fault reset conditions met
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    FaultResetValid := FALSE;
    RETURN;
END_IF;

(* Fault reset is valid when ALL conditions are met (level-based):
   - FaultReset is HIGH (level, not edge)
   - MotionEnable is LOW
   - We are in fault state
   - Active fault code is not FAULT_NONE
   - Decoded mode code is stable
   - Master has mirrored the fault code in G_mbModeCode *)
FaultResetValid := FaultReset AND
                   (NOT MotionEnable) AND
                   InFaultState AND
                   (ActiveFaultCode <> E_FaultCode#FAULT_NONE) AND
                   CodeStable AND
                   (MasterFaultCode = ActiveFaultCode);
