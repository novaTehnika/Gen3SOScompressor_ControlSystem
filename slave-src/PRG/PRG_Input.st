(* ============================================================================
   PRG_Input - Input mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Connect default IO variables and the Modbus holding registers written by
   the master to project specific variables. PRG_Main reads the mode code,
   MotionEnable and FaultReset registers (G_mbModeCode, G_mbMotionEnable,
   G_mbFaultReset) directly through its debounce and decoder FBs. The
   reference arrives in engineering units (mm, mm/s or % by mode).

   ============================================================================ *)

(* Mode-dependent reference *)
G_sysReference := REAL_TO_LREAL(G_mbReference);

(* Digital Input *)
G_diOvertravelNeg := MO1_DI_06; (* Negative overtravel / home reference switch *)
G_diOvertravelPos := MO1_DI_07; (* Positive overtravel switch *)

(* Analog Input: voltage across the pressure transducer's loop shunt *)
G_aiPressure := MO1_AI_01;
