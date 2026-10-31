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
       rFilteredValue      : LREAL := 0.0     (* Low-pass filter state
       rAlpha              : LREAL
       bInitialized        : BOOL := FALSE
       i                   : INT
       rS0, rS1, rS2       : LREAL            (* Median network working copies
       rS3, rS4            : LREAL
       rTemp               : LREAL            (* Compare-exchange scratch
       rMedianResult       : LREAL
       nSize               : INT
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

(* Median by compare-exchange. cx(x, y) is: rTemp := MIN(x, y); y := MAX(x, y);
   x := rTemp, leaving the smaller value in x. The buffer is left in place,
   so the network works on copies. *)
rS0 := aMedianBuffer[0];
rS1 := aMedianBuffer[1];
rS2 := aMedianBuffer[2];

IF nSize = 3 THEN
    (* Median of 3: cx(s0,s1), cx(s1,s2), cx(s0,s1) leaves it in s1 *)
    rTemp := MIN(rS0, rS1);  rS1 := MAX(rS0, rS1);  rS0 := rTemp;
    rTemp := MIN(rS1, rS2);  rS2 := MAX(rS1, rS2);  rS1 := rTemp;
    rTemp := MIN(rS0, rS1);  rS1 := MAX(rS0, rS1);  rS0 := rTemp;
    rMedianResult := rS1;
ELSE
    rS3 := aMedianBuffer[3];
    rS4 := aMedianBuffer[4];

    (* Median of 5. Order the pairs (s0,s1) and (s2,s3). *)
    rTemp := MIN(rS0, rS1);  rS1 := MAX(rS0, rS1);  rS0 := rTemp;
    rTemp := MIN(rS2, rS3);  rS3 := MAX(rS2, rS3);  rS2 := rTemp;

    (* Put the pair with the smaller minimum in (s0,s1). That minimum is
       below three of the other samples, so it cannot be the median and
       drops out. The median is the second smallest of s1, s2, s3, s4. *)
    IF rS0 > rS2 THEN
        rTemp := rS0;  rS0 := rS2;  rS2 := rTemp;
        rTemp := rS1;  rS1 := rS3;  rS3 := rTemp;
    END_IF;

    (* Order (s1,s4). Of the two ordered pairs (s1,s4) and (s2,s3), put the
       one with the smaller minimum first; the second smallest of the four
       is the smaller of that pair's maximum and the other pair's minimum. *)
    rTemp := MIN(rS1, rS4);  rS4 := MAX(rS1, rS4);  rS1 := rTemp;
    IF rS1 > rS2 THEN
        rTemp := rS1;  rS1 := rS2;  rS2 := rTemp;
        rTemp := rS4;  rS4 := rS3;  rS3 := rTemp;
    END_IF;
    rMedianResult := MIN(rS4, rS2);
END_IF;

(* --------------------------------------------------------------------------
   Stage 2: Low-Pass Filter (First-Order IIR)
   -------------------------------------------------------------------------- *)

(* Apply low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1] *)
rFilteredValue := (rAlpha * rMedianResult) + ((LREAL#1.0 - rAlpha) * rFilteredValue);

(* --------------------------------------------------------------------------
   Output
   -------------------------------------------------------------------------- *)

FilteredOutput := rFilteredValue;
