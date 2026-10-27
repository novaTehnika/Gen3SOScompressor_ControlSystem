(* ============================================================================
   PRG_StepProbe - Servo axis friction bench, scan-rate capture

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Serves bench/friction/frictionRun.m and bench/characterisation/charRun.m
   (bench/common/benchConfig.m holds its registers).
   On a change of the arm register it records 500 consecutive scans (1 s at
   2 ms) of the commanded velocity, the actual velocity, the actual torque,
   the position and the chamber pressure into the probe registers, so the
   master can see a velocity step response, the breakaway, the torque
   ripple and the pressure's response to the piston at the full sample
   rate, which the protocol registers (read every 50 ms) cannot show. Reads G_sys* globals only and
   uses registers 111 and 401 and up, so it runs beside PRG_Main. Not
   loaded together with PRG_ModbusBench or PRG_PressureProbe, which use
   the same holding register area.

   Assign to the 2 ms cyclic task. The FC#04 input register group must
   extend to IR3410 (see ../README.md and bench/friction/README.md 4.5). If the group cannot be made that
   large, shorten the capture: change 500 and the array bounds together,
   and nScan in benchConfig.m.

   Samples are integers: velocities in 0.001 mm/s, torque in 0.01 % of
   rated, position in 0.01 um from the first sample, whose absolute value
   is published as a REAL, pressure in 0.001 MPa gauge.

   Variables (define in MWiec GUI):
     VAR_GLOBAL (Modbus FC#03,06,16 holding registers, master -> slave):
       G_mbStepArm         AT %MW3.483548 : UINT   (* HR111: a changed value starts a capture
     VAR_GLOBAL (Modbus FC#04 input registers, slave -> master):
       G_mbStepState       AT %QW74592 : UINT   (* IR401: 0 idle, 1 capturing, 2 capture ready
       G_mbStepArmEcho     AT %QW74594 : UINT   (* IR402: arm value the ready capture was taken for
       G_mbStepCount       AT %QW74596 : UINT   (* IR403: scans per capture (500)
       G_mbStepPos0        AT %QD74598 : REAL   (* IR404-405: position at the first sample (mm)
       G_mbStepVelCmd      AT %QW74612 : ARRAY[0..499] OF INT   (* IR411-910: G_sysCommandedVelocity, 0.001 mm/s
       G_mbStepVel         AT %QW75612 : ARRAY[0..499] OF INT   (* IR911-1410: G_sysActualVelocity, 0.001 mm/s
       G_mbStepTorque      AT %QW76612 : ARRAY[0..499] OF INT   (* IR1411-1910: G_sysActualTorque, 0.01 %
       G_mbStepPos         AT %QD77612 : ARRAY[0..499] OF DINT  (* IR1911-2910: G_sysActualPosition - Pos0, 0.01 um
       G_mbStepPressure    AT %QW79612 : ARRAY[0..499] OF INT   (* IR2911-3410: G_sysActualPressure, 0.001 MPa gauge
     VAR:
       rPos0               : LREAL            (* Position at the first sample (mm)
       nLastArm            : UINT := 0        (* Arm value already acted on
       nIdx                : INT := 0         (* Next capture slot
       bFirstScan          : BOOL := TRUE
   ============================================================================ *)

IF bFirstScan THEN
    bFirstScan := FALSE;
    nLastArm := G_mbStepArm;
    G_mbStepState := UINT#0;
    G_mbStepCount := UINT#500;
END_IF;

(* --------------------------------------------------------------------------
   Capture of 500 consecutive scans, started by a change of the arm
   register. The block is overwritten in place; the master reads it once
   the state shows ready.
   -------------------------------------------------------------------------- *)
IF (G_mbStepArm <> nLastArm) AND (G_mbStepState <> UINT#1) THEN
    nLastArm := G_mbStepArm;
    nIdx := 0;
    rPos0 := G_sysActualPosition;
    G_mbStepPos0 := LREAL_TO_REAL(rPos0);
    G_mbStepState := UINT#1;
END_IF;

IF G_mbStepState = UINT#1 THEN
    G_mbStepVelCmd[nIdx] := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysCommandedVelocity * LREAL#1000.0, LREAL#32767.0));
    G_mbStepVel[nIdx]    := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualVelocity * LREAL#1000.0, LREAL#32767.0));
    G_mbStepTorque[nIdx] := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualTorque * LREAL#100.0, LREAL#32767.0));
    G_mbStepPos[nIdx]    := LREAL_TO_DINT((G_sysActualPosition - rPos0) * LREAL#100000.0);
    G_mbStepPressure[nIdx] := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualPressure * LREAL#1000.0, LREAL#32767.0));
    nIdx := nIdx + 1;
    IF nIdx >= 500 THEN
        G_mbStepArmEcho := nLastArm;
        G_mbStepState := UINT#2;
    END_IF;
END_IF;
