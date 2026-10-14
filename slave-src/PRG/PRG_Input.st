(* ============================================================================
   PRG_Input - Input mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Connect default IO variables and the Modbus holding registers written by
   the master to project specific variables. The mode code, MotionEnable
   and FaultReset registers map onto the same G_di* variables the discrete
   lines used, so the debounce, mode decoder and handshake FBs are
   unchanged. The reference arrives in engineering units (mm, mm/s or % by
   mode).

   ============================================================================ *)

(* Mode code register, low three bits *)
G_diModeBit0 := (G_mbModeCode MOD UINT#2) = UINT#1;
G_diModeBit1 := (G_mbModeCode MOD UINT#4) >= UINT#2;
G_diModeBit2 := (G_mbModeCode MOD UINT#8) >= UINT#4;
G_diMotionEnable := G_mbMotionEnable <> UINT#0;
G_diFaultReset := G_mbFaultReset <> UINT#0;

(* Mode-dependent reference *)
G_sysReference := REAL_TO_LREAL(G_mbReference);

(* Digital Input *)
G_diOvertravelNeg := MO1_DI_06; (* Negative overtravel / home reference switch *)
G_diOvertravelPos := MO1_DI_07; (* Positive overtravel switch *)

(* Analog Input: voltage across the pressure transducer's loop shunt *)
G_aiPressure := MO1_AI_01;
