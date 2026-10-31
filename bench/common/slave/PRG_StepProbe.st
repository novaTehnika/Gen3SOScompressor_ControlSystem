(* ============================================================================
   PRG_StepProbe - Servo axis friction bench, scan-rate capture

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Serves bench/common/benchProbe.m, which bench/friction/frictionRun.m and
   bench/characterisation/charRun.m use (bench/common/benchConfig.m holds
   its registers).
   Every scan it appends one record to a ring buffer in plain memory: the
   commanded velocity, the actual velocity, the actual torque, the
   position and the chamber pressure. The ring holds the last 8192 scans
   (16 s at 2 ms). The master takes records out through a window of
   18 records in the input registers: it writes the index of the first
   record it wants and reads the window, the counters and a copy of the
   window's start index in one FC#04 read of IR401-520. So the registers
   mapped are 120 input and 3 holding registers, not the records
   themselves, and the capture length is limited by the ring and not by
   the register group.
   Writing the arm register (HR111) to a value different from the last one
   acted on publishes Mark, the index of the record written in that scan,
   and echoes the value in ArmEcho. An arm is accepted at any time,
   including while earlier records are being read. The master takes the
   records it wants from Mark on (or before it, while the ring still holds
   them).

   Window rule: it is refilled only when needed. When the requested start
   differs from the window's start, or the window holds fewer than 18
   records and more have been written since, it takes up to 18 records
   from the requested start that are written and not yet overwritten
   (index >= Written - Depth). A start that is overwritten or not yet
   written gives an empty window. A record is 6 words in the window:
   vCmd, vel, torque, position low word, position high word, pressure.
   StartTail is a copy of Start assigned after the window, so a read that
   straddles an update shows Start <> StartTail and is discarded.

   Reads G_sys* globals only and uses registers HR111-113 and IR401-520,
   so it runs beside PRG_Main. Not loaded together with PRG_ModbusBench or
   PRG_PressureProbe, which use the same holding register area.

   Assign to the cyclic task (the only task on this controller). The FC#04
   input register group must extend to IR520 and the holding group to
   HR113. The cost of the 120 mapped input registers is small against the
   3410 of a mapped capture buffer. The ring is 96 KB of plain variables.
   If the compiler refuses the arrays, shorten the ring: change cDepth, the
   five array bounds and G_mbStepDepth's assignment together (the master
   reads the depth from IR409), and a stream or a capture that spans more
   than the ring is lost.

   Samples are integers: velocities in 0.001 mm/s, torque in 0.01 % of
   rated, position in 0.01 um absolute (1e5 counts per mm), pressure in
   0.001 MPa gauge. Index values are UDINT and wrap after 2^32 scans.

   Register addresses: IR k is at %QB 73792 + 2*(k-1), HR k at
   %MB3.(483328 + 2*(k-1)). Every 32-bit input value is on a 4-byte
   boundary. The 32-bit request starts at HR112, which is 2 bytes past a
   4-byte boundary, so it is declared as two UINTs (low word first, as
   every 32-bit value on this controller) and combined here.

   Variables (define in MWiec GUI):
     VAR_GLOBAL (Modbus FC#03,06,16 holding registers, master -> slave):
       G_mbStepArm         AT %MW3.483548 : UINT   (* HR111: a changed value is an arm
       G_mbStepReqLo       AT %MW3.483550 : UINT   (* HR112: low word of the first record index wanted in the window
       G_mbStepReqHi       AT %MW3.483552 : UINT   (* HR113: high word
     VAR_GLOBAL (Modbus FC#04 input registers, slave -> master):
       G_mbStepWritten     AT %QD74592 : UDINT  (* IR401-402: records written so far
       G_mbStepMark        AT %QD74596 : UDINT  (* IR403-404: index of the record written in the scan of the last arm
       G_mbStepStart       AT %QD74600 : UDINT  (* IR405-406: index of the first record in the window
       G_mbStepArmEcho     AT %QW74604 : UINT   (* IR407: arm value last acted on
       G_mbStepCount       AT %QW74606 : UINT   (* IR408: records in the window (0..18)
       G_mbStepDepth       AT %QW74608 : UINT   (* IR409: ring depth in records (8192)
       G_mbStepWords       AT %QW74610 : UINT   (* IR410: words per record (6)
       G_mbStepWindow      AT %QW74612 : ARRAY[0..107] OF WORD   (* IR411-518: 18 records of 6 words
       G_mbStepStartTail   AT %QD74828 : UDINT  (* IR519-520: copy of Start, assigned after the window
     VAR CONSTANT:
       cDepth              : UDINT := 8192    (* Ring depth in records; the array bounds below are cDepth - 1
       cWindow             : UDINT := 18      (* Records in the window
     VAR:
       aVelCmd             : ARRAY[0..8191] OF INT    (* Ring: G_sysCommandedVelocity, 0.001 mm/s
       aVel                : ARRAY[0..8191] OF INT    (* Ring: G_sysActualVelocity, 0.001 mm/s
       aTorque             : ARRAY[0..8191] OF INT    (* Ring: G_sysActualTorque, 0.01 %
       aPos                : ARRAY[0..8191] OF DINT   (* Ring: G_sysActualPosition, 0.01 um
       aPressure           : ARRAY[0..8191] OF INT    (* Ring: G_sysActualPressure, 0.001 MPa gauge
       nWritten            : UDINT := 0       (* Records written so far
       nSlot               : UDINT := 0       (* Ring slot of the record written this scan, nWritten MOD cDepth
       nLastArm            : UINT := 0        (* Arm value already acted on
       nReq                : UDINT := 0       (* Requested start of the window
       nWinStart           : UDINT := 0       (* Start of the window
       nWinCount           : UDINT := 0       (* Records in the window
       nOldest             : UDINT := 0       (* Index of the oldest record still in the ring
       nAvail              : UDINT := 0       (* Records written from the requested start on
       nIdx                : UDINT := 0       (* Ring slot of the record being copied into the window
       nCnt                : INT := 0         (* Records to copy
       nWord               : INT := 0         (* First window word of that record
       nI                  : INT := 0
       dwPos               : DWORD            (* A position as two words
       bFill               : BOOL := FALSE
       bFirstScan          : BOOL := TRUE
   ============================================================================ *)

IF bFirstScan THEN
    bFirstScan := FALSE;
    nLastArm := G_mbStepArm;
    G_mbStepArmEcho := nLastArm;
    G_mbStepDepth := UINT#8192;
    G_mbStepWords := UINT#6;
END_IF;

(* --------------------------------------------------------------------------
   Arm: any change of the register is acted on, whatever else is going on.
   -------------------------------------------------------------------------- *)
IF G_mbStepArm <> nLastArm THEN
    nLastArm := G_mbStepArm;
    G_mbStepArmEcho := nLastArm;
    G_mbStepMark := nWritten;
END_IF;

(* --------------------------------------------------------------------------
   Record of this scan, into slot nWritten MOD cDepth
   -------------------------------------------------------------------------- *)
nSlot := nWritten MOD cDepth;
aVelCmd[nSlot]   := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysCommandedVelocity * LREAL#1000.0, LREAL#32767.0));
aVel[nSlot]      := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualVelocity * LREAL#1000.0, LREAL#32767.0));
aTorque[nSlot]   := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualTorque * LREAL#100.0, LREAL#32767.0));
aPos[nSlot]      := LREAL_TO_DINT(G_sysActualPosition * LREAL#100000.0);
aPressure[nSlot] := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualPressure * LREAL#1000.0, LREAL#32767.0));
nWritten := nWritten + UDINT#1;

(* --------------------------------------------------------------------------
   Window: refilled only when the request moved or it can hold more
   -------------------------------------------------------------------------- *)
nReq := UINT_TO_UDINT(G_mbStepReqHi) * UDINT#65536 + UINT_TO_UDINT(G_mbStepReqLo);
bFill := FALSE;
IF nReq <> nWinStart THEN
    bFill := TRUE;
ELSIF (nWinCount < cWindow) AND (nWinStart + nWinCount < nWritten) THEN
    bFill := TRUE;
END_IF;

IF bFill THEN
    nWinStart := nReq;
    nWinCount := UDINT#0;
    IF nWritten > cDepth THEN
        nOldest := nWritten - cDepth;
    ELSE
        nOldest := UDINT#0;
    END_IF;
    IF (nReq >= nOldest) AND (nReq < nWritten) THEN
        nAvail := nWritten - nReq;
        IF nAvail > cWindow THEN
            nAvail := cWindow;
        END_IF;
        nWinCount := nAvail;
        nCnt := UDINT_TO_INT(nWinCount);
        FOR nI := 0 TO nCnt - 1 DO
            nIdx := (nReq + INT_TO_UDINT(nI)) MOD cDepth;
            nWord := nI * 6;
            dwPos := DINT_TO_DWORD(aPos[nIdx]);
            G_mbStepWindow[nWord]     := INT_TO_WORD(aVelCmd[nIdx]);
            G_mbStepWindow[nWord + 1] := INT_TO_WORD(aVel[nIdx]);
            G_mbStepWindow[nWord + 2] := INT_TO_WORD(aTorque[nIdx]);
            G_mbStepWindow[nWord + 3] := DWORD_TO_WORD(dwPos);
            G_mbStepWindow[nWord + 4] := DWORD_TO_WORD(SHR(dwPos, UINT#16));
            G_mbStepWindow[nWord + 5] := INT_TO_WORD(aPressure[nIdx]);
        END_FOR;
    END_IF;
END_IF;

(* --------------------------------------------------------------------------
   Header; the tail copy of the window start goes last
   -------------------------------------------------------------------------- *)
G_mbStepWritten := nWritten;
G_mbStepStart := nWinStart;
G_mbStepCount := UDINT_TO_UINT(nWinCount);
G_mbStepStartTail := nWinStart;
