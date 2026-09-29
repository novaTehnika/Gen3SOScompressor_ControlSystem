# Fault Code Reference

## Gen3 SOS Compressor Servo Controller
### Fault Diagnosis and Recovery Procedures

---

## 1. Overview

When `G_doFaultActive` (DO4) is HIGH, the slave is in fault state and DO0-DO2 contain the fault code instead of mode confirmation. The master must handle fault conditions by:

1. Reading the fault code
2. Mirroring the code on DI0-DI2
3. Asserting `G_diFaultReset` with `G_diMotionEnable` LOW
4. Waiting for fault to clear

---

## 2. Fault Code Summary

| Binary | Decimal | Name | Severity | Auto-Recoverable |
|--------|---------|------|----------|------------------|
| 000 | 0 | FAULT_NONE | - | - |
| 001 | 1 | FAULT_HANDSHAKE | Medium | Yes |
| 010 | 2 | FAULT_DRIVE | High | Sometimes |
| 011 | 3 | FAULT_POSITION | Medium | Yes |
| 100 | 4 | FAULT_HOMING_REQ | Low | Yes (with homing) |
| 101 | 5 | FAULT_PISTON_EXIT | High | Conditional |
| 110 | 6 | FAULT_LIMIT_SWITCH | High | Manual jog-off via ST_RECOVERY (investigate cause) |
| 111 | 7 | FAULT_ENCODER | High | **Reserved (not emitted by current firmware — encoder alarms surface as FAULT_DRIVE)** |

---

## 3. Detailed Fault Descriptions

### FAULT_NONE (000)

**Description**: No fault condition present.

**When Seen**: After successful fault reset, or normal operation.

**Master Action**: None required.

---

### FAULT_HANDSHAKE (001)

**Description**: Mode handshake timeout or mismatch.

**Triggered When**:
- Mode command not confirmed within 500ms timeout
- Mode bits changed during handshake
- Mode confirmation doesn't match command

**Symptoms**:
- Slave stuck in transition state
- Mode bits stable but no confirmation

**Master Recovery**:
```
1. Read fault code (001)
2. Set G_diMotionEnable = FALSE
3. Mirror code: DI0=1, DI1=0, DI2=0
4. Pulse G_diFaultReset
5. After clear: retry mode command
```

**Prevention**:
- Ensure mode bits are stable before handshake
- Don't change mode during handshake
- Verify I/O wiring integrity

---

### FAULT_DRIVE (010)

**Description**: Servo amplifier reported a fault condition.

**Triggered When**:
- Overcurrent detected
- Overvoltage/undervoltage
- Motor overheating
- Encoder communication error
- Internal drive error

**Symptoms**:
- Drive not ready
- Motor not responding
- Drive status LED indicates fault

**Master Recovery**:
```
1. Read fault code (010)
2. Check drive status (if accessible)
3. Set G_diMotionEnable = FALSE
4. Mirror code: DI0=0, DI1=1, DI2=0
5. Hold G_diFaultReset HIGH (level-based, not a pulse)
6. If persists: may require power cycle
```

**Investigation**:
- Check drive front panel for error code
- Verify motor wiring
- Check motor temperature
- Review drive parameters

---

### FAULT_POSITION (011)

**Description**: Position exceeded software limits.

**Triggered When**:
- Actual position < `G_posSoftLimitMin`
- Actual position > `G_posSoftLimitMax`
- Position drifted outside limits during stop

**Symptoms**:
- Position near or beyond expected limits
- Motion was toward limit when stopped

**Master Recovery**:
```
1. Read fault code (011)
2. Note current position from AO0
3. Set G_diMotionEnable = FALSE
4. Mirror code: DI0=1, DI1=1, DI2=0
5. Hold G_diFaultReset HIGH (level-based, not a pulse)
6. After clear:
   - If position is back inside the soft limits -> ST_BRAKE_HOLD (normal)
   - If still beyond a soft limit -> slave enters ST_RECOVERY; jog back
     inside via MODE_POSITION/MODE_VELOCITY (see "Limit Recovery State")
```

**Important**: If the position is still outside the soft limits at reset, the slave no longer bounces straight back into fault — it routes to **`ST_RECOVERY`** so you can drive back inside. See [Limit Recovery State (ST_RECOVERY)](#limit-recovery-state-st_recovery).

---

### FAULT_HOMING_REQ (100)

**Description**: Operational mode attempted without homing complete.

**Triggered When**:
- An operational mode (001-100) is commanded while `G_flagHomingRequired = TRUE`
- `G_flagHomingRequired` is a single flag, set at every boot and cleared only
  when Mode 110 (Home to Limit) homing completes

**Symptoms**:
- Cannot enter operational modes
- Homing has not yet completed this power cycle

**Master Recovery**:
```
1. Read fault code (100)
2. Set G_diMotionEnable = FALSE
3. Mirror code: DI0=0, DI1=0, DI2=1
4. Hold G_diFaultReset HIGH (level-based, not a pulse)
5. After clear: command Mode 110 (Home to Limit), or command Mode 101
   (Go Home), which redirects into the Mode 110 sequence automatically
```

**Note**: This fault will recur if an operational mode is commanded again before Mode 110 homing completes.

---

### FAULT_PISTON_EXIT (101)

**Description**: Piston exit prevention guard triggered.

**Triggered When**:
- Position near `G_posSoftLimitMin` (exit boundary)
- Torque mode anomaly: commanding torque one direction, moving opposite (indicates external pressure overcoming motor)
- Guard detected dangerous approach to exit boundary

**Symptoms**:
- Motion halted near minimum position
- In torque mode: position drifting toward exit despite command

**Master Recovery**:
```
1. Read fault code (101)
2. CRITICAL: Check if cylinder is pressurized
3. Set G_diMotionEnable = FALSE
4. Mirror code: DI0=1, DI1=0, DI2=1
5. Hold G_diFaultReset HIGH (level-based, not a pulse)
6. After clear: command position away from exit
```

**Investigation**:
- Check cylinder pressure
- Verify piston seal integrity
- Reduce commanded torque/force
- Check for mechanical binding

**Safety Note**: This fault indicates a potentially dangerous condition. The piston was approaching or being pushed toward the cylinder exit. Investigate before resuming operation.

---

### FAULT_LIMIT_SWITCH (110)

**Description**: Unexpected limit switch activation during motion.

**Triggered When**:
- Limit switch triggered during operational mode (not homing)
- Could indicate position error, mechanical issue, or runaway

**Symptoms**:
- One of the limit switches (DI6 or DI7) went LOW unexpectedly
- Position may not match expected location

**Master Recovery**:
```
1. Read fault code (110)
2. Stop all operations
3. Set G_diMotionEnable = FALSE
4. Mirror code: DI0=0, DI1=1, DI2=1
5. DO NOT automatically retry
6. Requires investigation
```

**Investigation**:
- Which limit switch triggered?
- Does position make sense for that switch?
- Check for mechanical obstruction
- Verify limit switch wiring
- Check for position drift or encoder error

**Manual Recovery**:
1. Clear fault with mirrored code (DI0=0, DI1=1, DI2=1) while `G_diMotionEnable` is LOW
2. If the switch is still active, the slave enters **`ST_RECOVERY`** — jog off the
   switch via `MODE_POSITION`/`MODE_VELOCITY` (see below). Motion *into* the switch
   is blocked; motion *away* is allowed.
3. Once clear, consider re-homing if position is uncertain.

---

### Limit Recovery State (ST_RECOVERY)

`FAULT_LIMIT_SWITCH` and `FAULT_POSITION` can leave the axis resting **on** a hard
overtravel switch or **beyond** a soft limit. Any normal motion mode would
immediately re-trip the fault there, so the firmware provides a dedicated recovery
sub-machine that lets the operator jog **away** from the limit while refusing motion
*into* it.

**Entry is automatic at fault reset.** When a `FAULT_LIMIT_SWITCH` or `FAULT_POSITION`
reset succeeds:
- If the limit condition has cleared (switch released **and** position inside the
  soft limits) → normal recovery (`ST_BRAKE_HOLD`), as before.
- If the limit is still active → the slave enters **`ST_RECOVERY`** (drive stays
  energized, brake released). From `ST_FAULT` this is immediate; from `ST_FAULT_IDLE`
  (slow path) the drive is re-enabled first and then lands in `ST_RECOVERY`.

**Operating procedure (two-step — during the reset the DI bits carry the fault-ack
code, so the recovery mode is selected afterward):**
1. After the reset lands in `ST_RECOVERY`, set the mode bits to `MODE_POSITION` (010)
   or `MODE_VELOCITY` (011) and raise `G_diMotionEnable` (the normal mode handshake).
2. Drive the analog reference to retract. Motion **away** from the active limit runs
   at the normal velocity limit. A command **toward** the active limit is clamped to
   zero — it does **not** re-fault; just reverse the reference.
3. When the switch clears and position is back inside the soft limits, drop
   `G_diMotionEnable`: the slave rejoins the normal branch (`ST_HOLD_POSITION`) and you
   may select any mode. If a limit is still active when motion-enable drops, it
   returns to the recovery hub so you can keep jogging.

**Exit without recovering:** from the recovery hub request `MODE_BRAKE_HOLD` (drive
stays on) or `MODE_IDLE` (shut down) — honored even with a limit still active.

While in recovery the position-limit and limit-switch faults are suppressed; **drive
and piston-exit faults remain active**.

---

### FAULT_ENCODER (111) — RESERVED

**Status**: Reserved but not emitted by current firmware.

As of 2026-04, the dedicated encoder-validity path was removed from `FB_SafetyMonitor`. Encoder alarms (A.810 battery backup loss, A.CC0 multi-turn error, A.830 low battery) are detected by the Sigma-7 servo amplifier and surface as `FAULT_DRIVE` (010) through `G_sysDriveFault`.

A fault reset does not by itself require re-homing. `G_flagHomingRequired` is set once at power-up and cleared only when Mode 110 (Home to Limit) homing completes; it is unaffected by fault resets, so homing that completed before the fault remains valid afterward.

The enum value is retained in `E_FaultCode` for wire-protocol stability but is not currently asserted.

---

## 4. Fault Reset Procedure

### Complete Reset Sequence

```
DETECT_FAULT:
    // G_doFaultActive = HIGH
    fault_code = (DO2 << 2) | (DO1 << 1) | DO0
    log_fault(fault_code)

PREPARE_RESET:
    // Ensure motion disabled
    SET G_diMotionEnable = FALSE
    WAIT 20ms  // Ensure stable

    // Mirror fault code to mode bits
    DI0 = fault_code & 0x01
    DI1 = (fault_code >> 1) & 0x01
    DI2 = (fault_code >> 2) & 0x01
    WAIT 20ms  // Ensure stable

EXECUTE_RESET:
    // Assert reset (level-based: valid the whole time FaultReset is HIGH
    // and the other conditions hold - not an edge check)
    SET G_diFaultReset = TRUE

    // Wait for acknowledgment
    START timeout_timer (1000ms)

WAIT_CLEAR:
    IF G_doFaultActive = FALSE THEN
        // Fault cleared successfully
        SET G_diFaultReset = FALSE
        GOTO SUCCESS
    ELSIF timeout_timer expired THEN
        SET G_diFaultReset = FALSE
        GOTO RESET_FAILED

SUCCESS:
    // Return to idle state
    SET mode_bits = 000
    // Determine next action based on fault type

RESET_FAILED:
    // Fault persists - requires investigation
    // May need power cycle or mechanical fix
```

### Reset Timing Diagram

```
G_doFaultActive  ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                         ^
                   |                         |
                   Fault                     Cleared
                   detected

G_diMotionEnable ‾‾‾‾\________________________/‾‾‾‾
                    ^                       ^
                    |                       |
                    Disable                 Re-enable
                    for reset               after clear

mode_bits      XXXX|--- fault_code ---------|0000
                   ^                        ^
                   |                        |
                   Mirror                   Return
                   code                     to idle

G_diFaultReset   ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                   ^
                   |                   |
                   Asserted            Release
                   (level, held)       after clear
```

---

## 5. Recovery Decision Tree

```
FAULT DETECTED (G_doFaultActive = HIGH)
           |
           v
    Read fault code
           |
    +------+------+------+------+------+------+------+
    |      |      |      |      |      |      |      |
   001    010    011    100    101    110    111
    |      |      |      |      |      |      |
    v      v      v      v      v      v      v
 Retry   Check  Move   Home   Check  STOP   Home
 mode    drive  from   first  press  Invest required
         status limit         ure    igate

After reset, determine appropriate next action:

001 (Handshake) -> Retry previous mode command
010 (Drive)     -> May need power cycle, then re-home
011 (Position)  -> If still out of limits, jog back in via ST_RECOVERY
100 (Homing)    -> Command Mode 110
101 (Piston)    -> Check pressure, move away from exit
110 (Limit)     -> Investigate; if still on switch, jog off via ST_RECOVERY
111 (Encoder)   -> Must complete Mode 110 homing
```

---

## 6. Fault Prevention Guidelines

### Handshake Faults
- Ensure stable mode bits before raising motion enable
- Don't modify mode during handshake window
- Use adequate debounce on digital outputs

### Drive Faults
- Stay within motor thermal limits
- Ensure adequate power supply capacity
- Regular drive maintenance

### Position Faults
- Command positions well within limits
- Use appropriate acceleration/deceleration
- Monitor position feedback during motion

### Homing Requirement Faults
- Always complete homing after power cycle
- Latch `G_doHomingComplete` when it pulses HIGH (it drops again once
  `G_diMotionEnable` is released - it is not a persistent status you can
  poll later) and check that latch before operational modes
- Use Mode 101 (Go Home) for automatic handling

### Piston Exit Faults
- Avoid commanding toward exit boundary under load
- Monitor cylinder pressure
- Use appropriate force limits in torque mode

### Limit Switch Faults
- Verify position tracking after homing
- Check for mechanical obstruction
- Regular limit switch testing

### Encoder Faults
- Monitor encoder battery status
- Replace battery proactively
- Ensure proper cable routing (no noise sources)

---

## 7. Quick Reference

### Fault Code Table

| Code | Bits | Name | Quick Fix |
|------|------|------|-----------|
| 0 | 000 | None | - |
| 1 | 001 | Handshake | Retry mode |
| 2 | 010 | Drive | Check drive |
| 3 | 011 | Position | Jog back in (ST_RECOVERY) |
| 4 | 100 | Homing Req | Home axis |
| 5 | 101 | Piston Exit | Check pressure |
| 6 | 110 | Limit Switch | Investigate; jog off (ST_RECOVERY) |
| 7 | 111 | Encoder | Home axis |

### Reset Checklist

1. [ ] G_diMotionEnable = FALSE
2. [ ] Mode bits = fault code (mirror)
3. [ ] Wait 20ms for stable
4. [ ] Hold G_diFaultReset HIGH (level-based, not a pulse)
5. [ ] Wait for G_doFaultActive = FALSE
6. [ ] Release G_diFaultReset
7. [ ] Set mode bits = 000
8. [ ] Take corrective action per fault type
