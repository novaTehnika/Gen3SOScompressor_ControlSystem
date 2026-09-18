(* ============================================================================
   FB_AnalogInputFilter - Analog Input Filter with Median and Low-Pass

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Two-stage analog input filter:
   1. Median filter (size 3 or 5) - Removes spike noise and outliers
   2. Low-pass filter (first-order IIR) - Smooths signal, configured by alpha

   The median filter runs first to remove impulsive noise, then the low-pass
   filter smooths the result. This combination is effective for industrial
   analog signals that may have both spike noise and high-frequency interference.

   Low-pass filter implementation:
   - First-order IIR: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
   - Alpha should be calculated by caller: alpha = dt / (tau + dt)
   - Where dt = task cycle time, tau = desired time constant
   - Higher alpha = faster response, less smoothing
   - Lower alpha = slower response, more smoothing

   Configuration:
   - Alpha: Filter coefficient (0.0-1.0), calculated in PRG_Main ST_INIT
   - MedianFilterSize: 3 or 5 samples for median filter

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable              : BOOL := TRUE     (* Enable filtering
       RawInput            : LREAL            (* Raw analog input (volts, -10.0 to +10.0)
       Alpha               : LREAL            (* Low-pass filter coefficient (0.0-1.0)
       MedianFilterSize    : INT := 3         (* Median filter size: 3 or 5
     VAR_OUTPUT:
       FilteredOutput      : LREAL            (* Filtered output value (volts)
     VAR:
       aMedianBuffer       : ARRAY[0..4] OF LREAL  (* Median filter buffer
       nMedianIndex        : INT := 0
       aSortBuffer         : ARRAY[0..4] OF LREAL
       rFilteredValue      : LREAL := 0.0     (* Low-pass filter state
       rAlpha              : LREAL
       bInitialized        : BOOL := FALSE
       i, j                : INT
       rTemp               : LREAL
       rMedianResult       : LREAL
       nSize               : INT
       nIdx1, nIdx2        : INT              (* Index variables for array access
       nMedianIdx          : INT              (* Median middle index
   ============================================================================ *)

(* ============================================================================
   Main Logic
   ============================================================================ *)

IF NOT Enable THEN
    (* Pass through raw value when disabled *)
    FilteredOutput := RawInput;
    RETURN;
END_IF;

(* Clamp alpha to valid range *)
rAlpha := LIMIT(LREAL#0.0, Alpha, LREAL#1.0);

(* --------------------------------------------------------------------------
   Initialization
   -------------------------------------------------------------------------- *)

IF NOT bInitialized THEN
    (* Fill median buffer with current value *)
    FOR i := 0 TO 4 DO
        aMedianBuffer[i] := RawInput;
    END_FOR;
    rFilteredValue := RawInput;
    bInitialized := TRUE;
END_IF;

(* --------------------------------------------------------------------------
   Stage 1: Median Filter
   -------------------------------------------------------------------------- *)

(* Limit median size to 3 or 5 *)
IF MedianFilterSize >= 5 THEN
    nSize := 5;
ELSE
    nSize := 3;
END_IF;

(* Add new sample to circular buffer *)
aMedianBuffer[nMedianIndex] := RawInput;
nMedianIndex := nMedianIndex + 1;
IF nMedianIndex >= nSize THEN
    nMedianIndex := 0;
END_IF;

(* Copy to sort buffer *)
FOR i := 0 TO nSize - 1 DO
    aSortBuffer[i] := aMedianBuffer[i];
END_FOR;

(* Simple bubble sort (efficient for small arrays) *)
FOR i := 0 TO nSize - 2 DO
    FOR j := 0 TO nSize - 2 - i DO
        (* Pre-calculate indices for array access *)
        nIdx1 := j;
        nIdx2 := j + 1;
        IF aSortBuffer[nIdx1] > aSortBuffer[nIdx2] THEN
            rTemp := aSortBuffer[nIdx1];
            aSortBuffer[nIdx1] := aSortBuffer[nIdx2];
            aSortBuffer[nIdx2] := rTemp;
        END_IF;
    END_FOR;
END_FOR;

(* Get median value (middle element) *)
nMedianIdx := nSize / 2;
rMedianResult := aSortBuffer[nMedianIdx];

(* --------------------------------------------------------------------------
   Stage 2: Low-Pass Filter (First-Order IIR)
   -------------------------------------------------------------------------- *)

(* Apply low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1] *)
rFilteredValue := (rAlpha * rMedianResult) + ((LREAL#1.0 - rAlpha) * rFilteredValue);

(* --------------------------------------------------------------------------
   Output
   -------------------------------------------------------------------------- *)

FilteredOutput := rFilteredValue;
