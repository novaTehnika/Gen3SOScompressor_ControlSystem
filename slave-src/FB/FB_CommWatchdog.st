(* ============================================================================
   FB_CommWatchdog - Master Link Supervision

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Raises Lost when the master heartbeat register stops changing for longer
   than Timeout. Holding registers keep their last value when the master
   stops writing, so a frozen heartbeat is the only sign of a lost link.

   The watchdog arms on the first heartbeat change it sees, so a controller
   that powers up before the master connects does not fault. It re-arms the
   same way after a communication fault has been reset.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Heartbeat       : UINT             (* Master heartbeat register
       Timeout         : TIME             (* Allowed time without a change
     VAR_OUTPUT:
       Lost            : BOOL             (* TRUE while the heartbeat is stale
     VAR:
       bArmed          : BOOL := FALSE    (* A heartbeat change has been seen
       bSame           : BOOL := FALSE    (* Heartbeat unchanged this scan
       nLast           : UINT := UINT#0   (* Heartbeat on the previous scan
       tonStale        : TON              (* Time since the last change
   ============================================================================ *)

bSame := (Heartbeat = nLast);
nLast := Heartbeat;
IF NOT bSame THEN
    bArmed := TRUE;
END_IF;

tonStale(IN := bSame, PT := Timeout);
Lost := bArmed AND tonStale.Q;
