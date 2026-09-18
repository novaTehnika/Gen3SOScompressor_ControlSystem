(* ============================================================================
   PRG_Output - Output mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Connect default IO variables to project specific IO variables.

   ============================================================================ *)

(* Digital Output *)
MO1_DO_00 := G_doModeConfBit0; (* Mode confirm bit 0 / Fault code bit 0 *)
MO1_DO_01 := G_doModeConfBit1; (* Mode confirm bit 1 / Fault code bit 1 *)
MO1_DO_02 := G_doModeConfBit2; (* Mode confirm bit 2 / Fault code bit 2 *)
MO1_DO_03 := G_doPerformanceStatus; (* Mode-dependent status indicator *)
MO1_DO_04 := G_doFaultActive; (* Fault indicator: HIGH = fault active *)
MO1_DO_05 := G_doHomingComplete; (* Homing sequence complete *)
MO1_DO_06 := G_doInMotion; (* Axis in motion indicator *)
MO1_DO_07 := G_doBrakeDisengage; (* Brake release command *)

(* Analog Output *)
MO1_AO_01 := G_aoPositionOutput; (* Position output signal to master *)