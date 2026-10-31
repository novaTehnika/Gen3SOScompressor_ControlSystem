(* ============================================================================
   PRG_PressureProbe - Pressure transducer noise bench

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Serves bench/pressure/pressureNoiseProbe.m. Reads the raw voltage on the
   analog input every scan, keeps one-second statistics, and on request
   captures 250 consecutive scans so the master can see the noise at the
   full 2 ms sample rate, which the protocol registers (filtered, read
   every 50 ms) cannot show. Uses only the probe registers (101 and up),
   so it runs beside PRG_Main with the transducer wired as in service. Not
   loaded together with PRG_ModbusBench, which uses the same registers.

   Assign to the 2 ms cyclic task. The FC#04 input register group must
   extend to IR370 (see ../README.md).

   Variables (define in MWiec GUI):
     VAR_GLOBAL (Modbus FC#03,06,16 holding registers, master -> slave):
       G_mbNoiseArm        AT %MW3.483528 : UINT   (* HR101: a changed value starts a capture
     VAR_GLOBAL (Modbus FC#04 input registers, slave -> master):
       G_mbNoiseMean       AT %QD73992 : REAL   (* IR101-102: mean of the last window (V)
       G_mbNoiseRms        AT %QD73996 : REAL   (* IR103-104: rms about the mean (V)
       G_mbNoiseMin        AT %QD74000 : REAL   (* IR105-106: minimum (V)
       G_mbNoiseMax        AT %QD74004 : REAL   (* IR107-108: maximum (V)
       G_mbNoiseWindow     AT %QW74008 : UINT   (* IR109: completed windows, wraps 65535 -> 0
       G_mbNoiseState      AT %QW74010 : UINT   (* IR110: 0 idle, 1 capturing, 2 capture ready
       G_mbNoiseLive       AT %QD74012 : REAL   (* IR111-112: this scan's raw input (V)
       G_mbNoiseArmEcho    AT %QW74016 : UINT   (* IR113: arm value the ready capture was taken for
       G_mbNoiseSamples    AT %QW74032 : ARRAY[0..249] OF UINT  (* IR121-370: capture, 0.2 mV per count
     VAR:
       rVolts              : LREAL            (* This scan's raw input (V)
       rSum                : LREAL := 0.0     (* Window accumulators
       rSumSq              : LREAL := 0.0
       rMin                : LREAL
       rMax                : LREAL
       nCount              : INT := 0         (* Samples in the window so far
       rMean               : LREAL
       rVar                : LREAL
       nLastArm            : UINT := 0        (* Arm value already acted on
       nIdx                : INT := 0         (* Next capture slot
       bFirstScan          : BOOL := TRUE
   ============================================================================ *)

rVolts := MO1_AI_01;
G_mbNoiseLive := LREAL_TO_REAL(rVolts);

IF bFirstScan THEN
    bFirstScan := FALSE;
    nLastArm := G_mbNoiseArm;
    rMin := rVolts;
    rMax := rVolts;
    G_mbNoiseState := UINT#0;
END_IF;

(* --------------------------------------------------------------------------
   One-second statistics window: 500 scans at 2 ms
   -------------------------------------------------------------------------- *)
rSum := rSum + rVolts;
rSumSq := rSumSq + rVolts * rVolts;
IF rVolts < rMin THEN
    rMin := rVolts;
END_IF;
IF rVolts > rMax THEN
    rMax := rVolts;
END_IF;
nCount := nCount + 1;

IF nCount >= 500 THEN
    rMean := rSum / INT_TO_LREAL(nCount);
    rVar := rSumSq / INT_TO_LREAL(nCount) - rMean * rMean;
    IF rVar < LREAL#0.0 THEN
        rVar := LREAL#0.0;
    END_IF;
    G_mbNoiseMean := LREAL_TO_REAL(rMean);
    G_mbNoiseRms := LREAL_TO_REAL(SQRT(rVar));
    G_mbNoiseMin := LREAL_TO_REAL(rMin);
    G_mbNoiseMax := LREAL_TO_REAL(rMax);
    IF G_mbNoiseWindow >= UINT#65535 THEN
        G_mbNoiseWindow := UINT#0;
    ELSE
        G_mbNoiseWindow := G_mbNoiseWindow + UINT#1;
    END_IF;
    rSum := LREAL#0.0;
    rSumSq := LREAL#0.0;
    rMin := rVolts;
    rMax := rVolts;
    nCount := 0;
END_IF;

(* --------------------------------------------------------------------------
   Capture of 250 consecutive scans, started by a change of the arm
   register. The block is overwritten in place; the master reads it once
   the state shows ready.
   -------------------------------------------------------------------------- *)
IF (G_mbNoiseArm <> nLastArm) AND (G_mbNoiseState <> UINT#1) THEN
    nLastArm := G_mbNoiseArm;
    nIdx := 0;
    G_mbNoiseState := UINT#1;
END_IF;

IF G_mbNoiseState = UINT#1 THEN
    G_mbNoiseSamples[nIdx] := LREAL_TO_UINT(LIMIT(LREAL#0.0, rVolts * LREAL#5000.0, LREAL#65535.0));
    nIdx := nIdx + 1;
    IF nIdx >= 250 THEN
        G_mbNoiseArmEcho := nLastArm;
        G_mbNoiseState := UINT#2;
    END_IF;
END_IF;
