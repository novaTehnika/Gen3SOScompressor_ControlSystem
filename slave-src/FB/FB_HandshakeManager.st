(* ============================================================================
   FB_HandshakeManager - Master-Slave Handshake Manager

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Manages the mode entry handshake protocol between master and slave.

   Mode Entry Handshake Sequence:
   1. Master sets mode bits (DI0-2)
   2. Slave decodes and validates mode
   3. Slave sets confirmation bits (DO0-2) to match
   4. Master sets diMotionEnable HIGH
   5. Slave verifies mode bits still match -> handshake complete
   6. If timeout or mismatch -> handshake failed

   NOTE: All inputs should be pre-filtered/debounced by the caller.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL             (* Enable handshake manager
       RequestedMode       : E_OperatingMode  (* Mode requested by master
       ModeStable          : BOOL             (* Mode inputs are stable
       MotionEnable        : BOOL             (* Motion enable from master - should be filtered
       HandshakeTimeout    : TIME := T#500MS  (* Timeout for handshake
     VAR_OUTPUT:
       ConfirmedMode       : E_OperatingMode  (* Current confirmed mode (valid when HandshakeComplete)
       HandshakeActive     : BOOL             (* TRUE from handshake start until complete/timeout
       HandshakeComplete   : BOOL             (* TRUE when handshake succeeded
       HandshakeTimeout_Q  : BOOL             (* TRUE when timeout occurred
     VAR:
       eState              : INT := 0         (* Handshake state - initialized to HS_IDLE
       tTimeout            : TON              (* Handshake timeout timer (single call site)
       bTimeoutIN          : BOOL             (* Latched IN for tTimeout
       eModeAtStart        : E_OperatingMode  (* Mode when handshake started
       bModeStableAtStart  : BOOL             (* Capture mode stability at handshake start
     VAR CONSTANT:
       HS_IDLE             : INT := 0         (* Idle, no handshake active
       HS_WAIT_MOTION_EN   : INT := 1         (* Waiting for motion enable
       HS_VERIFY_MODE      : INT := 2         (* Verifying mode match
       HS_COMPLETE         : INT := 3         (* Handshake complete
       HS_TIMEOUT          : INT := 4         (* Handshake timed out
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Reset all outputs when disabled *)
    ConfirmedMode := E_OperatingMode#MODE_IDLE;
    HandshakeActive := FALSE;
    HandshakeComplete := FALSE;
    HandshakeTimeout_Q := FALSE;
    eState := HS_IDLE;
    bTimeoutIN := FALSE;

ELSE
    (* Default timer IN: active only while waiting for motion enable *)
    bTimeoutIN := (eState = HS_WAIT_MOTION_EN);

(* --------------------------------------------------------------------------
   Mode Handshake State Machine
   -------------------------------------------------------------------------- *)

IF eState = HS_IDLE THEN

    (* HS_IDLE - Reset outputs *)
        HandshakeActive := FALSE;
        HandshakeComplete := FALSE;
        HandshakeTimeout_Q := FALSE;

        (* Start handshake when mode is stable and different from idle
           (or when transitioning to a new mode) *)
        IF ModeStable AND (RequestedMode <> E_OperatingMode#MODE_IDLE) AND NOT MotionEnable THEN
            ConfirmedMode := RequestedMode;
            eModeAtStart := RequestedMode;
            bModeStableAtStart := TRUE;
            HandshakeActive := TRUE;
            eState := HS_WAIT_MOTION_EN;
            bTimeoutIN := TRUE;
        ELSIF ModeStable AND (RequestedMode = E_OperatingMode#MODE_IDLE) THEN
            ConfirmedMode := E_OperatingMode#MODE_IDLE;
        END_IF;

ELSIF eState = HS_WAIT_MOTION_EN THEN

    (* HS_WAIT_MOTION_EN - Waiting for master to assert motion enable *)
        IF tTimeout.Q THEN
            bTimeoutIN := FALSE;
            eState := HS_TIMEOUT;

        ELSIF RequestedMode <> eModeAtStart THEN
            bTimeoutIN := FALSE;
            eState := HS_IDLE;

        ELSIF MotionEnable THEN
            eState := HS_VERIFY_MODE;
        END_IF;

ELSIF eState = HS_VERIFY_MODE THEN

    (* HS_VERIFY_MODE - Verify requested mode hasn't changed since handshake start *)
        bTimeoutIN := FALSE;
        IF RequestedMode = eModeAtStart THEN
            HandshakeComplete := TRUE;
            HandshakeActive := FALSE;
            eState := HS_COMPLETE;
        ELSE
            eState := HS_TIMEOUT;
        END_IF;

ELSIF eState = HS_COMPLETE THEN

    (* HS_COMPLETE - Handshake complete - stay here until motion enable drops *)
        IF NOT MotionEnable THEN
            (* Motion disabled - return to idle *)
            HandshakeComplete := FALSE;
            eState := HS_IDLE;
        END_IF;

ELSIF eState = HS_TIMEOUT THEN

    (* HS_TIMEOUT - Handshake timed out or failed *)
        HandshakeTimeout_Q := TRUE;
        HandshakeActive := FALSE;
        ConfirmedMode := E_OperatingMode#MODE_IDLE;
        bTimeoutIN := FALSE;

        IF NOT MotionEnable THEN
            HandshakeTimeout_Q := FALSE;
            eState := HS_IDLE;
        END_IF;

    END_IF;  (* state machine *)
END_IF;  (* Enable gate *)

(* --------------------------------------------------------------------------
   Timer call - exactly one site per scan
   -------------------------------------------------------------------------- *)
tTimeout(IN := bTimeoutIN, PT := HandshakeTimeout);
