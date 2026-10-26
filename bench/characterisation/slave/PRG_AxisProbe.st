(* ============================================================================
   PRG_AxisProbe - Axis step response capture for the characterisation bench

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Serves bench/characterisation/charRun.m (opts.probe) and charDriveLag.m.
   Keeps the last 250 scans of the velocity command, the velocity feedback,
   the torque feedback and the pressure in four ring buffers. When armed by
   a change of the arm register it waits for the velocity command to change
   by more than 0.05 mm/s, records 200 more scans, and publishes the ring
   with the index of the trigger scan, so the master sees 50 scans before
   the step and 200 after it at the full 2 ms rate. The protocol registers
   carry the same feedback every 50 ms, which cannot resolve a drive lag of
   10 to 30 ms.

   The command channel is G_sysCommandedVelocity, the value PRG_Main hands
   to Y_DirectControl before the drive's own accel and decel ramp; the
   feedback channels are the G_sys* values the LD POU reads from the axis
   each scan. Samples are INT: velocity and command in 0.001 mm/s, torque in
   0.1 % of rated, pressure in 0.001 MPa gauge.

   Reads globals only, so it runs beside PRG_Main. Assign to the same 2 ms
   cyclic task. Uses the bench probe registers (101 and up) like
   PRG_PressureProbe and PRG_ModbusBench, so it is not loaded together with
   either. The FC#04 input register group must extend to IR1120
   (see ../README.md).

   Variables (define in MWiec GUI):
     VAR_GLOBAL (Modbus FC#03,06,16 holding registers, master -> slave):
       G_mbProbeArm        AT %MW3.483528 : UINT   (* HR101: a changed value arms a capture
     VAR_GLOBAL (Modbus FC#04 input registers, slave -> master):
       G_mbProbeState      AT %QW73992 : UINT   (* IR101: 0 idle, 1 armed, 2 triggered, 3 ready
       G_mbProbeArmEcho    AT %QW73994 : UINT   (* IR102: arm value the ready capture was taken for
       G_mbProbeTrigIdx    AT %QW73996 : UINT   (* IR103: ring index of the trigger scan
       G_mbProbeCount      AT %QW73998 : UINT   (* IR104: ring length, 250
       G_mbProbeVel        AT %QW74032 : ARRAY[0..249] OF INT  (* IR121-370: G_sysActualVelocity, 0.001 mm/s
       G_mbProbeCmd        AT %QW74532 : ARRAY[0..249] OF INT  (* IR371-620: G_sysCommandedVelocity, 0.001 mm/s
       G_mbProbeTorque     AT %QW75032 : ARRAY[0..249] OF INT  (* IR621-870: G_sysActualTorque, 0.1 %
       G_mbProbePressure   AT %QW75532 : ARRAY[0..249] OF INT  (* IR871-1120: G_sysActualPressure, 0.001 MPa
     VAR:
       nIdx                : INT := 0         (* Ring slot written this scan
       nPost               : INT := 0         (* Scans still to record after the trigger
       nLastArm            : UINT := 0        (* Arm value already acted on
       rLastCmd            : LREAL := 0.0     (* Command of the previous scan
       bFirstScan          : BOOL := TRUE
   ============================================================================ *)

IF bFirstScan THEN
    bFirstScan := FALSE;
    nLastArm := G_mbProbeArm;
    rLastCmd := G_sysCommandedVelocity;
    G_mbProbeState := UINT#0;
    G_mbProbeCount := UINT#250;
END_IF;

(* --------------------------------------------------------------------------
   Arm on a change of the arm register. A ready capture is released by the
   next arm; an armed or triggered capture is left to finish.
   -------------------------------------------------------------------------- *)
IF (G_mbProbeArm <> nLastArm) AND (G_mbProbeState <> UINT#1) AND (G_mbProbeState <> UINT#2) THEN
    nLastArm := G_mbProbeArm;
    G_mbProbeState := UINT#1;
END_IF;

(* --------------------------------------------------------------------------
   Ring buffers: written every scan while idle, armed or triggered, so the
   scans before the trigger are always present; frozen while ready.
   -------------------------------------------------------------------------- *)
IF G_mbProbeState <> UINT#3 THEN
    G_mbProbeVel[nIdx]      := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualVelocity * LREAL#1000.0, LREAL#32767.0));
    G_mbProbeCmd[nIdx]      := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysCommandedVelocity * LREAL#1000.0, LREAL#32767.0));
    G_mbProbeTorque[nIdx]   := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualTorque * LREAL#10.0, LREAL#32767.0));
    G_mbProbePressure[nIdx] := LREAL_TO_INT(LIMIT(LREAL#-32767.0, G_sysActualPressure * LREAL#1000.0, LREAL#32767.0));

    IF (G_mbProbeState = UINT#1) AND (ABS(G_sysCommandedVelocity - rLastCmd) > LREAL#0.05) THEN
        (* This scan's sample is the first with the changed command *)
        G_mbProbeTrigIdx := INT_TO_UINT(nIdx);
        nPost := 199;
        G_mbProbeState := UINT#2;
    ELSIF G_mbProbeState = UINT#2 THEN
        nPost := nPost - 1;
        IF nPost <= 0 THEN
            (* 50 scans before the trigger and 200 from it are in the ring *)
            G_mbProbeArmEcho := nLastArm;
            G_mbProbeState := UINT#3;
        END_IF;
    END_IF;

    IF nIdx >= 249 THEN
        nIdx := 0;
    ELSE
        nIdx := nIdx + 1;
    END_IF;
END_IF;

rLastCmd := G_sysCommandedVelocity;
