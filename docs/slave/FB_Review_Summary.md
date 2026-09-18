# Function Block Review Summary
## Original Review Date: 2025-12-29

This document summarizes the agent reviews of the function blocks in `slave-src/FB/`.

> **Architecture update**
>
> Since the original review, the codebase was refactored so that all built-in motion FBs (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_MoveVelocity`, `MC_ReadActual*`, `Y_DirectControl`, `MC_SetPosition`) live in a Ladder Diagram POU. Custom ST function blocks that previously instantiated built-in FBs internally (`FB_HomeLimit`, `FB_HomeEOT`, `FB_GoHome`) were converted to receive status via `VAR_INPUT` (`StaMoveVelocity`, `StaStop`, `StaDirectControl`, `StaMoveAbsolute`, `StaSetPosition`) and emit commands via `VAR_OUTPUT` (`CmdMoveVelocity`, `CmdStop`, `CmdDirectControl`, `CmdMoveAbsolute`, `CmdSetPosition`). `PRG_Main` wires these to the corresponding `G_cmd*`/`G_sta*` structured globals. `FB_EncoderManager` and the `AbsolutePositionManager` toolbox FB were removed in 2026-04 — encoder alarms now surface as `FAULT_DRIVE` via the servo amplifier.
>
> The FB-by-FB findings below still apply to the logic contained in each FB, but their instantiation footprint no longer includes internal built-in FB calls.

---

## FB_AnalogInputFilter.st
**Status**: Minor issues

### Findings:
1. **LOW**: `bInitialized` flag not reset when `Enable` goes FALSE - filter state persists across disable/enable cycles
2. **LOW**: Median index synchronization - `nMedianIndex` wraps correctly but sample overwriting order may cause brief transients

### Recommendations:
- Consider resetting `bInitialized` in the `NOT Enable` branch if fresh start desired
- Current behavior is acceptable for most applications

---

## FB_AnalogProcessor.st
**Status**: Issues found

### Findings:
1. **HIGH**: Stage boundary comparison at line 107 uses `<=` instead of `<`
   - Current: `ELSIF rPosActual <= rStage1Max THEN`
   - Should be: `ELSIF rPosActual < rStage1Max THEN`
   - This causes boundary value to be processed by both stages

2. **HIGH**: No voltage clamping on output
   - `rPositionVoltage` could exceed -10V to +10V range if position exceeds limits
   - Should clamp output to valid DAC range

3. **MEDIUM**: Missing check for `rStage1Max >= rStage2Min` overlap condition

### Recommendations:
- Fix boundary comparison operator
- Add output clamping: `rPositionVoltage := LIMIT(-10.0, rPositionVoltage, 10.0);`

---

## FB_DigitalInputFilter.st
**Status**: Minor issue

### Findings:
1. **MEDIUM**: First scan initialization issue
   - If `RawInput` is already TRUE on first scan, the filter may not immediately recognize this
   - `bLastRaw` initializes to FALSE, so first scan with TRUE input starts debounce timer
   - FilteredOutput stays FALSE until debounce completes (expected behavior)

2. **LOW**: `bPendingState` not explicitly initialized
   - Defaults to FALSE, which is correct for most cases

### Recommendations:
- Add initialization logic if immediate recognition of pre-existing TRUE state is required
- Current behavior is correct for most debounce applications

---

## FB_HandshakeManager.st
**Status**: Refactored

### Changes:
- **Fault reset logic extracted** to `FB_FaultResetHandler` (separate single-responsibility FB)
- **Mode-to-bits conversion removed** — `ConfirmBit0/1/2` outputs and `fbModeToBits` removed; `FB_OutputMux` now handles conversion internally
- Stripped down to mode handshake only: `ConfirmedMode`, `HandshakeActive`, `HandshakeComplete`, `HandshakeTimeout_Q`

### Previous Findings (resolved):
1. **CRITICAL** *(FIXED)*: Timer not called in HS_WAIT_MOTION_EN state
2. **LOW**: `eModeAtStart` persists across handshake cycles (cosmetic, no functional impact)

### Recommendations:
- None remaining

---

## FB_FaultResetHandler.st
**Status**: New (extracted from FB_HandshakeManager)

### Description:
Pure combinational check for fault reset validation. No state machine, no timers.
Validates: FaultReset HIGH, MotionEnable LOW, InFaultState, BitsStable, and MasterFaultCode matches ActiveFaultCode.

### Findings:
- Clean single-responsibility implementation
- Receives pre-decoded `MasterFaultCode : E_FaultCode` via `VAR_INPUT` — `PRG_Main` calls `FB_BitsToFaultCode` against DI0-2 and wires the result in, ensuring type-correct fault-code decoding (bits decoded as `E_FaultCode`, not `E_OperatingMode`)
- `BitsStable` input guards against transient bit patterns during DI0-2 transitions

### Recommendations:
- None required

---

## FB_ModeDecoder.st
**Status**: Fixed

### Findings:
1. **CRITICAL** *(FIXED)*: Edge detection logic error
   - Previous: `ModeChanged := NOT ModeStable AND bPrevStable;` (detected stability transitions, not value changes)
   - **Fix applied**: Removed `bPrevStable`, simplified to direct comparison of mode values

2. **LOW**: `nPreviousValue` initialized to 0 implicitly (correct, matches MODE_IDLE)

### Recommendations:
- None remaining

---

## FB_OutputMux.st
**Status**: Refactored

### Changes:
- **Input changed**: `ModeConfirmBit0/1/2 : BOOL` replaced with `ConfirmedMode : E_OperatingMode`
- **Added internal conversion**: `fbModeToBits : FB_ModeToBits` — both mode and fault now use consistent enum-to-bits conversion internally
- Both branches follow the same pattern: enum input → FB converter → bit outputs

### Findings:
- Clean implementation with proper enable handling
- Consistent conversion pattern for both modes and faults via `FB_ModeToBits` and `FB_FaultToBits`

### Recommendations:
- None required

---

## FB_PositionOutput.st
**Status**: Minor issue

### Findings:
1. **MEDIUM**: Out-of-range position produces misleading status flags
   - When position is outside both stages, `InStage1` and `InStage2` are both FALSE
   - `MappedVoltage` behavior undefined in this case

2. **LOW**: No explicit clamping of output voltage

### Recommendations:
- Consider adding `OutOfRange` status flag
- Add output voltage clamping for safety

---

## Priority Fixes (Ordered by Severity)

### CRITICAL:
1. ~~**FB_HandshakeManager**: Add timer call in HS_WAIT_MOTION_EN state~~ *(FIXED)*
2. ~~**FB_ModeDecoder**: Fix ModeChanged edge detection logic~~ *(FIXED)*

### HIGH:
3. **FB_AnalogProcessor**: Fix stage boundary comparison (`<=` to `<`)
4. **FB_AnalogProcessor**: Add output voltage clamping

### MEDIUM:
5. **FB_DigitalInputFilter**: Consider initialization handling
6. **FB_PositionOutput**: Add out-of-range handling

### LOW:
7. Various initialization and cosmetic issues (acceptable as-is)

---

## FB_PistonExitGuard.st
**Status**: Rewritten (2025-12-30)

### Changes Made:
1. **CRITICAL FIX**: Corrected torque sign convention
   - Previous: Assumed negative torque = pushing toward exit (WRONG)
   - Corrected: Positive torque = motor resisting exit pressure (CORRECT)

2. **Simplified to detect-only**: Removed all command correction logic
   - Removed inputs: CurrentMode, CommandedPosition, CommandedVelocity, CommandedTorque
   - Removed outputs: CorrectedPosition, CorrectedVelocity, CorrectedTorque, PositionLimited, VelocityHalted, TorqueAnomalyDetect
   - Now mode-independent, called once in safety monitoring section

3. **Detection logic**:
   - Triggers fault when: InDangerZone AND (ActualTorque > threshold) AND (ActualVelocity < -threshold)
   - This detects: Motor applying positive torque (resisting) but piston still moving toward exit = pressure overcoming motor

### Recommendations:
- Production ready after rewrite

---

## FB_SafetyMonitor.st
**Status**: Simplified (2025-12-30)

### Changes Made:
1. **Removed warning outputs** (not used, added complexity):
   - Removed: WarnNearLimitMin, WarnNearLimitMax, WarnLowBattery
   - Removed input: PositionMargin

2. **Updated fault state detection**:
   - Added ST_FAULT_IDLE to bInFaultState check

3. **Additional changes from restructuring**:
   - Deprecated homing sub-states removed from `bInHomingState` check (now only checks `ST_HOME_LIMIT`, `ST_HOME_EOT`, `ST_HOME_COMPLETE`)

4. **Encoder fault path removed (2026-04)**:
   - Inputs `EncoderValid`, `EncoderBatteryOK`, `AbsHomeRequired` and output `FaultEncoder` deleted
   - Encoder alarms now propagate via `DriveFault -> FAULT_DRIVE` from the servo amplifier

### Recommendations:
- Production ready after simplification
