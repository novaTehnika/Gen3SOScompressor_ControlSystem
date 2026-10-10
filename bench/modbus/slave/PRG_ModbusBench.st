(* ============================================================================
   PRG_ModbusBench - Modbus TCP bench test

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Serves the master-side bench scripts in bench/modbus. Echoes the master
   heartbeat, counts scans, and publishes fixed probe patterns that reveal
   the register-to-address mapping and the word order of 32-bit values.
   Uses only its own globals, so it can run alongside PRG_Main.

   Assign this program and the four Modbus server blocks (Hardware
   Configuration, IO Task Assignment) to the 2 ms cyclic task.

   Modbus register k (1-based, as addressed by MATLAB/Simulink) maps to byte
   offset 28672 + 2*(k-1): holding registers in %I, input registers in %Q.

   Variables (define in MWiec GUI):
     VAR_GLOBAL (Modbus holding registers, master -> slave):
       G_mbHeartbeat       AT %IW28672 : UINT   (* HR1: master step counter
       G_mbProbeDintIn     AT %ID28872 : DINT   (* HR101-102: master writes 16#1234, 16#5678
     VAR_GLOBAL (Modbus input registers, slave -> master):
       G_mbHeartbeatEcho   AT %QW28672 : UINT   (* IR1: last G_mbHeartbeat seen
       G_mbSlaveCounter    AT %QW28674 : UINT   (* IR2: scan counter, wraps 65535 -> 0
       G_mbProbeDintConst  AT %QD28872 : DINT   (* IR101-102: 16#12345678
       G_mbProbeRealConst  AT %QD28876 : REAL   (* IR103-104: 1234.5 (16#449A5000)
       G_mbProbeDintEcho   AT %QD28880 : DINT   (* IR105-106: copy of G_mbProbeDintIn
       G_mbProbeWordOrder  AT %QW28884 : UINT   (* IR107: 1 high word first, 2 low word first, 0 other
       G_mbProbeUintConst  AT %QW28886 : UINT   (* IR108: 16#1234
     VAR:
       nCounter            : UINT := 0          (* Scan counter
   ============================================================================ *)

(* Heartbeat and scan counter *)
G_mbHeartbeatEcho := G_mbHeartbeat;

IF nCounter >= 65535 THEN
    nCounter := 0;
ELSE
    nCounter := nCounter + 1;
END_IF;
G_mbSlaveCounter := nCounter;

(* Probe patterns *)
G_mbProbeDintConst := DINT#16#12345678;
G_mbProbeRealConst := 1234.5;
G_mbProbeUintConst := UINT#16#1234;
G_mbProbeDintEcho := G_mbProbeDintIn;

IF G_mbProbeDintIn = DINT#16#12345678 THEN
    G_mbProbeWordOrder := 1;
ELSIF G_mbProbeDintIn = DINT#16#56781234 THEN
    G_mbProbeWordOrder := 2;
ELSE
    G_mbProbeWordOrder := 0;
END_IF;
