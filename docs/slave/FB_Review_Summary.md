# Function Block Review Summary
## Original Review Date: 2025-12-29

This document summarizes the agent reviews of the function blocks in `slave-src/FB/`.

> **Architecture update**
>
> Since the original review, the codebase was refactored so that all built-in motion FBs (`MC_Power`, `MC_Stop`, `MC_Reset`, `MC_MoveAbsolute`, `MC_ReadActual*`, `Y_DirectControl`, `MC_SetPosition`) live in a Ladder Diagram POU. Custom ST function blocks that previously instantiated built-in FBs internally (`FB_HomeLimit`, `FB_GoHome`) were converted to receive status via `VAR_INPUT` (`StaStop`, `StaDirectControl`, `StaMoveAbsolute`, `StaSetPosition`) and emit commands via `VAR_OUTPUT` (`CmdStop`, `CmdDirectControl`, `CmdMoveAbsolute`, `CmdSetPosition`). `PRG_Main` wires these to the corresponding `G_cmd*`/`G_sta*` structured globals. There is no encoder-manager FB: encoder alarms surface as `FAULT_DRIVE` via the servo amplifier.
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
- Confirmed mode is an `E_OperatingMode` output; `PRG_Main` converts it to `G_sysConfirmedModeCode` for IR3 (`G_mbConfCode`)
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
Validates: FaultReset HIGH, MotionEnable LOW, InFaultState, CodeStable, and MasterFaultCode matches ActiveFaultCode.

### Findings:
- Clean single-responsibility implementation
- Receives pre-decoded `MasterFaultCode : E_FaultCode` via `VAR_INPUT` — `PRG_Main` calls `FB_IntToFaultCode` on `G_mbModeCode` (HR2) and wires the result in, so the mirrored code is decoded as `E_FaultCode`, not `E_OperatingMode`
- `CodeStable` input (from `FB_ModeDecoder.ModeStable`) guards against a mode code that is still changing

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

## Priority Fixes (Ordered by Severity)

### CRITICAL:
1. ~~**FB_HandshakeManager**: Add timer call in HS_WAIT_MOTION_EN state~~ *(FIXED)*
2. ~~**FB_ModeDecoder**: Fix ModeChanged edge detection logic~~ *(FIXED)*

### MEDIUM:
3. **FB_DigitalInputFilter**: Consider initialization handling

### LOW:
4. Various initialization and cosmetic issues (acceptable as-is)

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
   - Deprecated homing sub-states removed from `bInHomingState` check (now only checks `ST_HOME_LIMIT`, `ST_HOME_COMPLETE`)

4. **No encoder fault path**:
   - The FB has no encoder inputs and no `FaultEncoder` output
   - Encoder alarms propagate via `DriveFault -> FAULT_DRIVE` from the servo amplifier

### Recommendations:
- Production ready after simplification
