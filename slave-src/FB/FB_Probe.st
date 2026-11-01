(* ============================================================================
   FB_Probe - Scan-rate data capture for experiments and benches

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Every scan the block appends one record to a ring buffer in plain memory.
   The ring holds the last 4096 records (16 s at the 4 ms scan). The master
   takes records out through a window of 13 records in the input registers:
   it writes the index of the first record it wants and reads the window,
   the counters and a copy of the window's start index in one FC#04 read.
   The registers mapped are therefore few, and a capture is limited by the
   ring and not by the register group. Nothing in the master/slave protocol
   depends on the probe; with Enable FALSE the block records nothing.

   Record (8 words, defined here and only here; the master's decoder is
   bench/common/benchProbe.m):
     word 0   commanded velocity    INT    0.001 mm/s
     word 1   actual velocity       INT    0.001 mm/s
     word 2   actual torque         INT    0.01 % of rated
     word 3-4 actual position       DINT   0.01 um absolute, low word first
     word 5   chamber pressure      INT    0.001 MPa gauge
     word 6   raw pressure input    UINT   0.2 mV per count
     word 7   status                WORD   bit 0 in motion, bit 1 performance
                                           status, bit 2 fault active, bit 3
                                           homing complete, bits 8-15 the
                                           confirmed mode code
   Index values are UDINT and wrap after 2^32 scans.

   Recording state (State): PROBE_OFF while Enable is FALSE. Otherwise
   PROBE_RECORDING. A change of the arm register acts at once, whatever
   else is going on: it publishes Mark, the index of the record written in
   that scan, echoes the arm value in ArmEcho, latches the register Length
   (records wanted, 0 = open ended) and enters PROBE_CAPTURING. A capture
   whose last record (Mark + Length - 1) is written enters PROBE_HELD, and
   PROBE_HELD returns to PROBE_RECORDING once the ring has overwritten the
   record at Mark. An open-ended capture stays PROBE_CAPTURING until the
   next arm. The master writes Length before the arm register, or both in
   one FC#16 write.

   Fault mark: on the scan FaultState becomes TRUE (PRG_Main entering
   ST_FAULT) FaultMark takes the index of that scan's record, so the
   history up to every fault can be read back while it is in the ring.
   FaultMark is 0 until the first fault.

   Transfer state (Xfer): the master's request is the index of the first
   record it wants (ReqLo, ReqHi: two UINT registers, low word first,
   because the pair starts 2 bytes past a 4-byte boundary). A request that
   differs from the fill target restarts the fill; the same request written
   again changes nothing. The block copies at most FillPerScan records per
   scan from the ring into Stage, so a transfer never stretches a scan.
   When the copy is complete Publish is TRUE for that scan and Start, Count
   and StartTail hold the new window; they change only then. In that scan
   PRG_Main copies Stage to G_sysProbeStage, and PRG_Output copies that
   into the mapped window and writes StartTail last, so a read
   that straddles an update shows Start <> StartTail and is discarded.
     XFER_IDLE     the window is complete for the request
     XFER_FILLING  records are being copied
     XFER_PENDING  the requested start is not recorded yet
     XFER_OVERRUN  the requested start (or, during a fill, a record still
                   to be copied) has been overwritten
   A window with fewer than 13 records (the capture still being recorded) is
   extended from the records written since, copying only the new ones.

   Cost per scan: the 8-word record, the arm and fault logic and at most
   FillPerScan * 8 word copies (48 at the default of 6), plus, in the scan a
   fill completes, the 104-word copy in PRG_Output.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* FALSE: no recording, PROBE_OFF
       FillPerScan     : INT              (* Records copied into the window per scan, limited to 1-13
       VelCmd          : LREAL            (* Commanded velocity (mm/s)
       Vel             : LREAL            (* Actual velocity (mm/s)
       Torque          : LREAL            (* Actual torque (% of rated)
       Position        : LREAL            (* Actual position (mm)
       Pressure        : LREAL            (* Chamber pressure (MPa gauge)
       Volts           : LREAL            (* Raw pressure input (V)
       InMotion        : BOOL             (* Status bit 0
       Performance     : BOOL             (* Status bit 1
       FaultActive     : BOOL             (* Status bit 2
       HomingComplete  : BOOL             (* Status bit 3
       ModeCode        : UINT             (* Confirmed mode code, status bits 8-15
       FaultState      : BOOL             (* TRUE while PRG_Main is in ST_FAULT
       Arm             : UINT             (* Arm register: a changed value is an arm
       Length          : UINT             (* Length register: records wanted by the next arm, 0 = open ended
       ReqLo           : UINT             (* Low word of the first record index wanted in the window
       ReqHi           : UINT             (* High word
     VAR_OUTPUT:
       Stage           : ARRAY104WORD     (* The window being filled: 13 records of 8 words
       Written         : UDINT := 0       (* Records written so far
       Mark            : UDINT := 0       (* Index of the record written in the scan of the last arm
       FaultMark       : UDINT := 0       (* Index of the record written in the scan of the last fault
       Start           : UDINT := 0       (* Index of the first record in the published window
       StartTail       : UDINT := 0       (* Copy of Start, written after the window
       Count           : UINT := 0        (* Records in the published window (0-13)
       ArmEcho         : UINT := 0        (* Arm value last acted on
       Depth           : UINT := 0        (* Ring depth in records
       Words           : UINT := 0        (* Words per record
       State           : E_ProbeState := E_ProbeState#PROBE_RECORDING
       StateCode       : UINT := 0        (* State as the published integer
       Xfer            : E_ProbeXferState := E_ProbeXferState#XFER_IDLE
       XferCode        : UINT := 0        (* Xfer as the published integer
       Publish         : BOOL := FALSE    (* TRUE in the scan a fill completes: Stage, Start, Count are new
     VAR CONSTANT:
       cDepth          : UDINT := 4096    (* Ring depth in records; the ring array holds cDepth * cWords words
       cWords          : UDINT := 8       (* Words per record
       cWindow         : UDINT := 13      (* Records in the window; Stage holds cWindow * cWords words
     VAR:
       aRing           : ARRAY32768WORD   (* Ring: records of cWords words, record n in words (n MOD cDepth) * cWords
       bFirstScan      : BOOL := TRUE
       bPrevFault      : BOOL := FALSE    (* FaultState on the previous scan
       nLastArm        : UINT := 0        (* Arm value already acted on
       nCapLen         : UDINT := 0       (* Records the open capture wants, 0 = open ended
       nBase           : UDINT := 0       (* First ring word of the record written this scan
       nPos            : DINT := 0        (* Position in 0.01 um
       dwPos           : DWORD := 0       (* nPos as two words
       nStatus         : UINT := 0        (* Status word
       nOldest         : UDINT := 0       (* Index of the oldest record still in the ring
       nReq            : UDINT := 0       (* Index requested by the master
       nTarget         : UDINT := 0       (* Start of the window being filled
       nFilled         : UDINT := 0       (* Records of the target copied into Stage
       nPlanned        : UDINT := 0       (* Records the window is to hold
       nAvail          : UDINT := 0       (* Records written from the target on, up to cWindow
       nPerScan        : UDINT := 6       (* FillPerScan limited to 1..cWindow
       nStep           : UDINT := 0       (* Records to copy this scan
       nSrc            : UDINT := 0       (* First ring word of the records being copied
       nDst            : UDINT := 0       (* First Stage word they go to
       nWords          : DINT := 0        (* Words to copy this scan
       nI              : DINT := 0
       bOverrun        : BOOL := FALSE    (* The target (or the rest of a fill) was overwritten
   ============================================================================ *)

(* --------------------------------------------------------------------------
   Constants the master reads, and the one-scan flag
   -------------------------------------------------------------------------- *)
Publish := FALSE;
Depth := UDINT_TO_UINT(cDepth);
Words := UDINT_TO_UINT(cWords);

IF bFirstScan THEN
    bFirstScan := FALSE;
    nLastArm := Arm;
    ArmEcho := Arm;
END_IF;

IF NOT Enable THEN
    State := E_ProbeState#PROBE_OFF;
    StateCode := UINT#0;
    Xfer := E_ProbeXferState#XFER_IDLE;
    XferCode := UINT#0;
    nLastArm := Arm;
    bPrevFault := FaultState;
    RETURN;
END_IF;

IF State = E_ProbeState#PROBE_OFF THEN
    State := E_ProbeState#PROBE_RECORDING;
END_IF;

(* --------------------------------------------------------------------------
   Arm: any change of the register is acted on, whatever else is going on
   -------------------------------------------------------------------------- *)
IF Arm <> nLastArm THEN
    nLastArm := Arm;
    ArmEcho := Arm;
    Mark := Written;
    nCapLen := UINT_TO_UDINT(Length);
    State := E_ProbeState#PROBE_CAPTURING;
END_IF;

(* --------------------------------------------------------------------------
   Fault mark: the scan PRG_Main enters ST_FAULT
   -------------------------------------------------------------------------- *)
IF FaultState AND NOT bPrevFault THEN
    FaultMark := Written;
END_IF;
bPrevFault := FaultState;

(* --------------------------------------------------------------------------
   Record of this scan, into slot Written MOD cDepth
   -------------------------------------------------------------------------- *)
nBase := (Written MOD cDepth) * cWords;
nPos := LREAL_TO_DINT(LIMIT(LREAL#-2147483647.0, Position * LREAL#100000.0, LREAL#2147483647.0));
dwPos := DINT_TO_DWORD(nPos);
nStatus := BOOL_TO_UINT(InMotion) + BOOL_TO_UINT(Performance) * UINT#2
           + BOOL_TO_UINT(FaultActive) * UINT#4 + BOOL_TO_UINT(HomingComplete) * UINT#8
           + MIN(ModeCode, UINT#255) * UINT#256;

aRing[nBase]              := INT_TO_WORD(LREAL_TO_INT(LIMIT(LREAL#-32767.0, VelCmd * LREAL#1000.0, LREAL#32767.0)));
aRing[nBase + UDINT#1]    := INT_TO_WORD(LREAL_TO_INT(LIMIT(LREAL#-32767.0, Vel * LREAL#1000.0, LREAL#32767.0)));
aRing[nBase + UDINT#2]    := INT_TO_WORD(LREAL_TO_INT(LIMIT(LREAL#-32767.0, Torque * LREAL#100.0, LREAL#32767.0)));
aRing[nBase + UDINT#3]    := DWORD_TO_WORD(dwPos);
aRing[nBase + UDINT#4]    := DWORD_TO_WORD(SHR(dwPos, UINT#16));
aRing[nBase + UDINT#5]    := INT_TO_WORD(LREAL_TO_INT(LIMIT(LREAL#-32767.0, Pressure * LREAL#1000.0, LREAL#32767.0)));
aRing[nBase + UDINT#6]    := UINT_TO_WORD(LREAL_TO_UINT(LIMIT(LREAL#0.0, Volts * LREAL#5000.0, LREAL#65535.0)));
aRing[nBase + UDINT#7]    := UINT_TO_WORD(nStatus);
Written := Written + UDINT#1;

IF Written > cDepth THEN
    nOldest := Written - cDepth;
ELSE
    nOldest := UDINT#0;
END_IF;

(* --------------------------------------------------------------------------
   Recording state: CAPTURING -> HELD when the last record is written,
   HELD -> RECORDING when the record at Mark is overwritten
   -------------------------------------------------------------------------- *)
IF State = E_ProbeState#PROBE_CAPTURING THEN
    IF (nCapLen > UDINT#0) AND (Written >= Mark + nCapLen) THEN
        State := E_ProbeState#PROBE_HELD;
    END_IF;
END_IF;
IF State = E_ProbeState#PROBE_HELD THEN
    IF Mark < nOldest THEN
        State := E_ProbeState#PROBE_RECORDING;
    END_IF;
END_IF;

IF State = E_ProbeState#PROBE_CAPTURING THEN
    StateCode := UINT#2;
ELSIF State = E_ProbeState#PROBE_HELD THEN
    StateCode := UINT#3;
ELSE
    StateCode := UINT#1;
END_IF;

(* --------------------------------------------------------------------------
   Transfer: plan the window for the request
   -------------------------------------------------------------------------- *)
IF FillPerScan < 1 THEN
    nPerScan := UDINT#1;
ELSIF FillPerScan > 13 THEN
    nPerScan := cWindow;
ELSE
    nPerScan := INT_TO_UDINT(FillPerScan);
END_IF;

nReq := UINT_TO_UDINT(ReqHi) * UDINT#65536 + UINT_TO_UDINT(ReqLo);
IF nReq <> nTarget THEN
    nTarget := nReq;
    nFilled := UDINT#0;
    nPlanned := UDINT#0;
    bOverrun := FALSE;
END_IF;

IF NOT bOverrun THEN
    IF (nFilled = UDINT#0) AND (nTarget < nOldest) THEN
        bOverrun := TRUE;
    ELSIF (nFilled < nPlanned) AND (nTarget + nFilled < nOldest) THEN
        bOverrun := TRUE;
    ELSIF nTarget < Written THEN
        nAvail := Written - nTarget;
        IF nAvail > cWindow THEN
            nAvail := cWindow;
        END_IF;
        IF (nAvail > nPlanned) AND (nTarget + nFilled >= nOldest) THEN
            nPlanned := nAvail;
        END_IF;
    END_IF;
END_IF;

(* --------------------------------------------------------------------------
   Transfer: copy at most nPerScan records, publish when the window is complete
   -------------------------------------------------------------------------- *)
IF (NOT bOverrun) AND (nFilled < nPlanned) THEN
    nStep := nPlanned - nFilled;
    IF nStep > nPerScan THEN
        nStep := nPerScan;
    END_IF;
    nSrc := ((nTarget + nFilled) MOD cDepth) * cWords;
    nDst := nFilled * cWords;
    nWords := UDINT_TO_DINT(nStep * cWords);
    FOR nI := 0 TO nWords - 1 DO
        Stage[nDst + DINT_TO_UDINT(nI)] := aRing[(nSrc + DINT_TO_UDINT(nI)) MOD (cDepth * cWords)];
    END_FOR;
    nFilled := nFilled + nStep;
    IF nFilled = nPlanned THEN
        Start := nTarget;
        Count := UDINT_TO_UINT(nFilled);
        StartTail := nTarget;
        Publish := TRUE;
    END_IF;
END_IF;

IF bOverrun THEN
    Xfer := E_ProbeXferState#XFER_OVERRUN;
    XferCode := UINT#3;
ELSIF nFilled < nPlanned THEN
    Xfer := E_ProbeXferState#XFER_FILLING;
    XferCode := UINT#1;
ELSIF nPlanned = UDINT#0 THEN
    Xfer := E_ProbeXferState#XFER_PENDING;
    XferCode := UINT#2;
ELSE
    Xfer := E_ProbeXferState#XFER_IDLE;
    XferCode := UINT#0;
END_IF;
