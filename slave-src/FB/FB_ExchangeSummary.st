(* ============================================================================
   FB_ExchangeSummary - Per-exchange summary of the pressure and the axis

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   The master samples the input registers every 50 ms, while the slave scans
   every 4 ms, so the pressure in IR19-20 is one scan's value out of about
   twelve. This block summarises all the scans between two master writes.
   A window is the run of scans from one change of the master's heartbeat
   register (HR1) up to the next. The scan in which the heartbeat differs
   from the previous scan's closes the open window, publishes it and opens a
   new one that starts with the current scan, so there is one window per
   master write, locked to the master's period. A late or missed write gives
   one longer window (Scans shows it).

   Published per window: the mean, minimum and maximum pressure, the mean
   velocity, torque and position, the number of scans (Scans) and the number
   of windows closed (Seq). The outputs change only in the scan a window
   closes; until the first window closes every output is 0, and Scans = 0
   marks that no window has closed yet.

   A window that reaches cMaxScans scans closes without a heartbeat change,
   so a lost link does not let the sums run on. The first write after the
   link returns closes the window open at that time.

   Lag: the slave does not see the master's reads. The window the master
   reads was opened by the write two exchanges earlier and closed by the
   write before the read, so the summary trails the IR19-20 snapshot by one
   to two exchanges.

   Cost per scan: one comparison, a few additions and two MIN/MAX; the
   divisions happen only in the scan a window closes. The block reads no
   G_* variable.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Heartbeat       : UINT             (* Master heartbeat register (HR1); a change closes the window
       Pressure        : LREAL            (* Chamber pressure (MPa gauge), the value published in IR19-20
       Velocity        : LREAL            (* Actual velocity (mm/s)
       Torque          : LREAL            (* Actual torque (% of rated)
       Position        : LREAL            (* Actual position (mm)
     VAR_OUTPUT:
       PressureMean    : LREAL := 0.0     (* Mean pressure of the last closed window (MPa gauge)
       PressureMin     : LREAL := 0.0     (* Minimum pressure of that window (MPa gauge)
       PressureMax     : LREAL := 0.0     (* Maximum pressure of that window (MPa gauge)
       VelocityMean    : LREAL := 0.0     (* Mean velocity of that window (mm/s)
       TorqueMean      : LREAL := 0.0     (* Mean torque of that window (% of rated)
       PositionMean    : LREAL := 0.0     (* Mean position of that window (mm)
       Scans           : UINT := 0        (* Scans in that window; 0 until the first window closes
       Seq             : UINT := 0        (* Windows closed, wraps 65535 -> 0
     VAR CONSTANT:
       cMaxScans       : UINT := 65535    (* A window this long closes without a heartbeat change (at least 1)
     VAR:
       bFirstScan      : BOOL := TRUE
       nPrevHeartbeat  : UINT := 0        (* Heartbeat of the previous scan
       nScans          : UINT := 0        (* Scans in the open window
       rScans          : LREAL := 0.0     (* nScans as LREAL, for the means
       rSumPressure    : LREAL := 0.0     (* Sum of the open window's pressures
       rMinPressure    : LREAL := 0.0     (* Smallest pressure of the open window
       rMaxPressure    : LREAL := 0.0     (* Largest pressure of the open window
       rSumVelocity    : LREAL := 0.0     (* Sum of the open window's velocities
       rSumTorque      : LREAL := 0.0     (* Sum of the open window's torques
       rSumPosition    : LREAL := 0.0     (* Sum of the open window's positions
   ============================================================================ *)

(* The first call takes the current heartbeat as the previous one, so the
   master's first value does not close an empty window *)
IF bFirstScan THEN
    bFirstScan := FALSE;
    nPrevHeartbeat := Heartbeat;
END_IF;

(* A heartbeat change (or the length cap) closes the open window *)
IF (Heartbeat <> nPrevHeartbeat) OR (nScans >= cMaxScans) THEN
    IF nScans > UINT#0 THEN
        rScans := UINT_TO_LREAL(nScans);
        PressureMean := rSumPressure / rScans;
        PressureMin := rMinPressure;
        PressureMax := rMaxPressure;
        VelocityMean := rSumVelocity / rScans;
        TorqueMean := rSumTorque / rScans;
        PositionMean := rSumPosition / rScans;
        Scans := nScans;
        IF Seq >= UINT#65535 THEN
            Seq := UINT#0;
        ELSE
            Seq := Seq + UINT#1;
        END_IF;
    END_IF;
    nScans := UINT#0;
    rSumPressure := LREAL#0.0;
    rSumVelocity := LREAL#0.0;
    rSumTorque := LREAL#0.0;
    rSumPosition := LREAL#0.0;
END_IF;
nPrevHeartbeat := Heartbeat;

(* This scan belongs to the window that is now open *)
IF nScans = UINT#0 THEN
    rMinPressure := Pressure;
    rMaxPressure := Pressure;
ELSE
    rMinPressure := MIN(rMinPressure, Pressure);
    rMaxPressure := MAX(rMaxPressure, Pressure);
END_IF;
rSumPressure := rSumPressure + Pressure;
rSumVelocity := rSumVelocity + Velocity;
rSumTorque := rSumTorque + Torque;
rSumPosition := rSumPosition + Position;
nScans := nScans + UINT#1;
