(* ============================================================================
   PRG_Input - Input mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Connect default IO variables to project specific IO variables.

   ============================================================================ *)

(* Digital Input *)
G_diModeBit0 := MO1_DI_00; (* Mode bit 0 - LSB *)
G_diModeBit1 := MO1_DI_01; (* Mode bit 1 *)
G_diModeBit2 := MO1_DI_02; (* Mode bit 2 - MSB *)
G_diMotionEnable := MO1_DI_03; (* Motion enable from master *)
G_diFaultReset := MO1_DI_04; (* Fault reset command from master *)
(*G_diReserved := MO1_DI_05; (* Reserved for future use *)
G_diOvertravelNeg := MO1_DI_06; (* Negative overtravel / home reference switch *)
G_diOvertravelPos := MO1_DI_07; (* Positive overtravel switch *)


(* Analog Input *)
G_aiReference := MO1_AI_01; (* Mode-dependent reference signal *)