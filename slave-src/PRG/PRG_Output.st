(* ============================================================================
   PRG_Output - Output mapping

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Connect project specific variables to the default IO variables and to
   the Modbus input registers read by the master. DO3-DO6 carry the status
   bits also published in the input registers; DO0-DO2 are not driven.

   ============================================================================ *)

(* Digital Output *)
MO1_DO_03 := G_doPerformanceStatus; (* Mode-dependent status indicator *)
MO1_DO_04 := G_doFaultActive; (* Fault indicator: HIGH = fault active *)
MO1_DO_05 := G_doHomingComplete; (* Homing sequence complete *)
MO1_DO_06 := G_doInMotion; (* Axis in motion indicator *)
MO1_DO_07 := G_doBrakeDisengage; (* Brake release command *)

(* Modbus input registers *)
G_mbHeartbeatEcho := G_mbHeartbeat;
G_mbSlaveCounter := G_sysScanCounter;
G_mbConfCode := G_sysConfirmedModeCode;
G_mbFaultCode := G_sysActiveFaultCode;
G_mbFaultActive := BOOL_TO_UINT(G_doFaultActive);
G_mbInMotion := BOOL_TO_UINT(G_doInMotion);
G_mbPerformanceStatus := BOOL_TO_UINT(G_doPerformanceStatus);
G_mbHomingComplete := BOOL_TO_UINT(G_doHomingComplete);
G_mbActualPosition := LREAL_TO_REAL(G_sysActualPosition);
G_mbActualVelocity := LREAL_TO_REAL(G_sysActualVelocity);
G_mbActualTorque := LREAL_TO_REAL(G_sysActualTorque);
G_mbCommandedPosition := LREAL_TO_REAL(G_sysCommandedPosition);
G_mbActualPressure := LREAL_TO_REAL(G_sysActualPressure);
