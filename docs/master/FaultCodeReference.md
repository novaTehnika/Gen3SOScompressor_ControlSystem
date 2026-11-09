# Fault Code Reference

## Gen3 SOS Compressor Servo Controller
### Fault Diagnosis and Recovery Procedures

---

## 1. Overview

When `G_mbFaultActive` (IR5) is 1, the slave is in fault state. `G_mbFaultCode` (IR4) holds the full fault code; `G_mbConfCode` (IR3) reads 0 outside the handshake, so also during the fault. The master must handle fault conditions by:

1. Reading the fault code from IR4
2. Mirroring the full code in `G_mbModeCode` (HR2)
3. Setting `G_mbFaultReset` with `G_mbMotionEnable` at 0
4. Waiting for fault to clear

---

## 2. Fault Code Summary

| Code | Name | Severity | Auto-Recoverable |
|------|------|----------|------------------|
| 0 | FAULT_NONE | - | - |
| 1 | FAULT_HANDSHAKE | Medium | Yes |
| 2 | FAULT_DRIVE | High | Sometimes |
| 3 | FAULT_POSITION | Medium | Yes |
| 4 | FAULT_COMM | High | Yes, once the master link is back |
| 5 | FAULT_PISTON_EXIT | High | Conditional |
| 6 | FAULT_LIMIT_SWITCH | High | Manual jog-off via ST_RECOVERY (investigate cause) |
| 7 | FAULT_ENCODER | High | **Reserved (not emitted by current firmware — encoder alarms surface as FAULT_DRIVE)** |
| 8 | FAULT_PRESSURE_SENSOR | High | Yes, once the transducer loop reads in range |

---

## 3. Detailed Fault Descriptions

### FAULT_NONE (0)

**Description**: No fault condition present.

**When Seen**: After successful fault reset, or normal operation.

**Master Action**: None required.

---

### FAULT_HANDSHAKE (1)

**Description**: Mode handshake timeout or mismatch.

**Triggered When**:
- Mode command not confirmed within 500ms timeout
- Mode code changed during handshake
- Mode confirmation doesn't match command

**Symptoms**:
- Slave stuck in transition state
- Mode code stable but no confirmation

**Master Recovery**:
```
1. Read fault code from IR4 (1)
2. Set MotionEnable (HR3) = 0
3. Mirror code: HR2 = 1
4. Pulse FaultReset (HR4)
5. After clear: retry mode command
```

**Prevention**:
- Ensure the mode code is stable before handshake
- Don't change mode during handshake
- Verify the Modbus link

---

### FAULT_DRIVE (2)

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
1. Read fault code from IR4 (2)
2. Check drive status (if accessible)
3. Set MotionEnable (HR3) = 0
4. Mirror code: HR2 = 2
5. Hold FaultReset (HR4) = 1 (level-based, not a pulse)
6. If persists: may require power cycle
```

**Investigation**:
- Check drive front panel for error code
- Verify motor wiring
- Check motor temperature
- Review drive parameters

---

### FAULT_POSITION (3)

**Description**: Position exceeded software limits.

**Triggered When**:
- Actual position < `G_cfgPosSoftLimitMin` (-2.5 mm)
- Actual position > `G_cfgPosSoftLimitMax` (367 mm)

`FB_SafetyMonitor` checks this every scan, so it also catches a position that
drifts outside the limits while the axis is held. It is suppressed in
`ST_HOME_LIMIT` (the homing sequence drives below the minimum to find the
switch) and in the recovery states (the axis starts outside the limits and
is being jogged back), and in the activation states on the way to recovery
(`bRecoveryPending`). Pressure mode stops positive motion
`G_cfgPressureEndMargin` short of `G_cfgPosSoftLimitMax`, so a delivery to
the end of travel does not raise it. Overpressure is not a fault: above
`G_cfgPressureCeiling` pressurising motion is zeroed and `G_mbPressureBlock`
(IR9) is set.

**Symptoms**:
- Position near or beyond expected limits
- Motion was toward limit when stopped

**Master Recovery**:
```
1. Read fault code from IR4 (3)
2. Note current position from IR11-12 (G_mbActualPosition)
3. Set MotionEnable (HR3) = 0
4. Mirror code: HR2 = 3
5. Hold FaultReset (HR4) = 1 (level-based, not a pulse)
6. After clear:
   - If position is back inside the soft limits -> ST_BRAKE_HOLD (normal)
   - If still beyond a soft limit -> slave enters ST_RECOVERY; jog back
     inside via MODE_POSITION/MODE_VELOCITY (see "Limit Recovery State")
```

**Important**: If the position is still outside the soft limits at reset, the slave no longer bounces straight back into fault — it routes to **`ST_RECOVERY`** so you can drive back inside. See [Limit Recovery State (ST_RECOVERY)](#limit-recovery-state-st_recovery).

---

### FAULT_COMM (4)

**Description**: The master heartbeat register `G_mbHeartbeat` has not changed
for `G_cfgCommTimeout` (300 ms) while motion is commanded (position,
velocity and torque control, Go Home, the homing moves and the recovery
moves). There the frozen registers would keep the axis moving, so the slave
stops as for any fault. In every other state nothing moves, a stopped master
is routine (closing the app also drops the safety relay), and the hold
inactivity watchdog returns the slave to idle, so the link may come and go
without a fault.

**Detection**: `FB_CommWatchdog`, fed into `FB_SafetyMonitor` at priority
just below the drive fault. The watchdog arms on the first heartbeat change
it sees, so a controller that powers up before the master connects does not
fault, and it re-arms the same way after this fault is reset.

**Recovery**: Restore the link. The master mirrors code 4 and resets as for
any fault; the heartbeat is changing again by then, so the watchdog is
clear. If the master itself lost the link it is latched in its own
`LINK_LOST` state until the operator presses Stop, then the reset follows.

---

### FAULT_PISTON_EXIT (5)

**Description**: The piston backed out toward the cylinder exit while the
chamber was pressurised.

**Triggered When**:
All three at once, evaluated by `FB_PistonExitGuard` every scan in every state:
- Position at or below the exit boundary, `G_cfgPosCylinderEntry` +
  `G_cfgPosExitGuardMargin` (180 + 20 = 200 mm, which the operator app shows as 167 mm from the end
  of travel). This boundary is above the
  cylinder entry, not at the soft limit (`G_cfgPosSoftLimitMin`), and the
  band below it has no lower edge
- Chamber pressure (IR19-20) above `G_cfgExitGuardPressure` (0.1 MPa gauge)
- The piston has backed out more than `G_cfgExitGuardRetreat` (0.5 mm) from
  the innermost position it reached since the first two became true

The retreat is a distance, so the speed does not matter: a slow creep trips
as a fast move does, and small motion about a held position does not. The
motor torque plays no part. A transducer fault counts as pressure while the
piston is in the bore, and the trip is then reported as
`FAULT_PRESSURE_SENSOR` (8), not as this code.

The guard only detects. The fault's `MC_Stop` (`G_cfgStopDecel`) arrests
the axis: from the fastest move (`G_cfgVelLimitMax`, 7 mm/s) the piston
comes to rest about 3 mm inside the boundary, 17 mm short of the entry.

**Symptoms**:
- A retract (jog, position move, Go Home, homing, or the pressure loop
  lowering the pressure) halted just below 200 mm
- IR19-20 reads above 0.1 MPa

**Master Recovery**:
```
1. Read fault code from IR4 (5)
2. Read the pressure in IR19-20: the chamber is pressurised
3. Set MotionEnable (HR3) = 0
4. Mirror code: HR2 = 5
5. Hold FaultReset (HR4) = 1 (level-based, not a pulse)
6. After clear: vent the chamber before retracting, or move the piston in
```

The fault is an event, not a state: once the axis is at rest it resets with
the chamber still pressurised, and motion into the cylinder is never
refused. A further retract under pressure trips again, each time after at
most `G_cfgExitGuardRetreat` plus the stopping distance.

**Investigation**:
- Why the chamber held pressure during a retract: a valve left shut, an
  oxygen charge larger than the chamber holds at ambient pressure at the
  entry, or a pressure target the loop can only reach beyond the exit
- Whether the pressure reading is true (compare IR19-20 at a vented chamber)
- `G_cfgPosCylinderEntry` against the machine

**Safety Note**: This fault indicates a potentially dangerous condition. The piston was moving toward the cylinder exit with the chamber pressurised; had the seal left the bore, the contents would have sprayed out. Investigate before resuming operation.

---

### FAULT_LIMIT_SWITCH (6)

**Description**: Unexpected limit switch activation during motion.

**Triggered When**:
- Limit switch triggered during operational mode (not homing)
- Could indicate position error, mechanical issue, or runaway

**Symptoms**:
- One of the limit switches (DI6 or DI7) went LOW unexpectedly
- Position may not match expected location

**Master Recovery**:
```
1. Read fault code from IR4 (6)
2. Stop all operations
3. Set MotionEnable (HR3) = 0
4. Mirror code: HR2 = 6
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
1. Clear fault with mirrored code (HR2 = 6) while MotionEnable (HR3) is 0
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
  energized, brake released) by one of two paths. Direct: from `ST_FAULT` with the
  brake released, immediately. Re-power: from `ST_FAULT_IDLE`, or from `ST_FAULT`
  when the fault was latched with the brake applied (from idle, during activation,
  or at power-up beyond a limit), the slave sets `bRecoveryPending` and runs the
  activation states (`ST_DRIVE_RESET`, `ST_DRIVE_ENABLE`, `ST_BRAKE_RELEASE`), then
  lands in `ST_RECOVERY` at brake release. The position-limit suppression starts at
  the reset (`InRecovery` is `bInRecovery OR bRecoveryPending`), so the slave does
  not fault again on the way.

**Operating procedure (two-step — during the reset the mode register (HR2) carries
the fault-ack code, so the recovery mode is selected afterward):**
1. After the reset lands in `ST_RECOVERY`, set the mode code (HR2) to `MODE_POSITION` (2)
   or `MODE_VELOCITY` (3) and raise MotionEnable (HR3) (the normal mode handshake).
2. Drive the reference (HR5-6) to retract. Motion **away** from the active limit runs
   at the normal velocity limit. A command **toward** the active limit is clamped to
   zero — it does **not** re-fault; just reverse the reference.
3. When the switch clears and position is back inside the soft limits, drop
   MotionEnable (HR3): the slave rejoins the normal branch (`ST_HOLD_POSITION`) and you
   may select any mode. If a limit is still active when motion-enable drops, it
   returns to the recovery hub so you can keep jogging.

**Exit without recovering:** from the recovery hub request `MODE_BRAKE_HOLD` (drive
stays on) or `MODE_IDLE` (shut down) — honored even with a limit still active.

While in recovery the position-limit and limit-switch faults are suppressed; **drive
and piston-exit faults remain active**.

---

### FAULT_ENCODER (7) — RESERVED

**Status**: Reserved but not emitted by current firmware.

`FB_SafetyMonitor` has no encoder-validity path. Encoder alarms (A.810 battery backup loss, A.CC0 multi-turn error, A.830 low battery) are detected by the Sigma-7 servo amplifier and surface as `FAULT_DRIVE` (2) through `G_sysDriveFault`.

A fault reset does not require re-homing: the coordinate frame is held by the absolute encoder and by the `MC_SetPosition` offset the controller stores in battery-backed memory. Re-home with Mode 110 (Home to Limit) only when that frame is lost: after an encoder alarm (reported as `FAULT_DRIVE`), or after a controller SRAM battery failure or controller replacement, which lose the stored offset.

The enum value is retained in `E_FaultCode` for protocol stability but is not currently asserted.

---

### FAULT_PRESSURE_SENSOR (8)

**Description**: The pressure transducer's loop current is out of range while the slave depends on the reading: the pressure loop is closed on it, or the piston is backing out of the bore near the cylinder exit.

**Triggered When**:
- Loop current below `G_cfgPressureSensorMinMa` (3.6 mA: open loop, unpowered transducer, broken wire) or above `G_cfgPressureSensorMaxMa` (20.5 mA: shorted shunt, transducer over range)
- For longer than `G_cfgPressureSensorFaultTime` (100 ms) after the median and low-pass filters
- And either of:
  - the state is `ST_PRESSURE_CTRL`
  - in any state, the piston is in the bore inside the exit guard band (`G_cfgPosCylinderEntry` to `G_cfgPosCylinderEntry` + `G_cfgPosExitGuardMargin`, 180 to 200 mm) and backs out more than `G_cfgExitGuardRetreat` (0.5 mm): the piston exit guard cannot tell whether the chamber is pressurised and takes it to be (see `FAULT_PISTON_EXIT`)

**Symptoms**:
- `G_mbActualPressure` (IR19-20) reads below zero or above the transducer range
- The axis halts through `MC_Stop`, as for any fault

**Master Action**:
1. Read IR19-20: a value near -4 MPa means an open loop, far above 16 MPa a short or over-range
2. Check the transducer supply, the shunt and the analog input wiring
3. Reset through the normal handshake once IR19-20 reads a plausible pressure

**Notes**: Elsewhere a bad reading does not fault: at rest, moving into the cylinder, anywhere above the guard band, and below the cylinder entry, where the cylinder is open (so the axis can still be homed and parked). It is visible in IR19-20 and the master should refuse to start pressure control on it. With the transducer failed the piston cannot be withdrawn from the bore through the guard band; restore the reading first. The fault resets with the loop still open once the axis is at rest.

---

## 4. Fault Reset Procedure

### Complete Reset Sequence

```
DETECT_FAULT:
    // IR5 G_mbFaultActive = 1
    fault_code = IR4  // G_mbFaultCode, full value
    log_fault(fault_code)

PREPARE_RESET:
    // Ensure motion disabled
    SET HR3 (MotionEnable) = 0
    WAIT 20ms  // Ensure stable

    // Mirror the full fault code in the mode register
    SET HR2 (mode code) = fault_code
    WAIT 20ms  // Ensure stable

EXECUTE_RESET:
    // Assert reset (level-based: valid the whole time FaultReset is HIGH
    // and the other conditions hold - not an edge check)
    SET HR4 (FaultReset) = 1

    // Wait for acknowledgment
    START timeout_timer (1000ms)

WAIT_CLEAR:
    IF IR5 (G_mbFaultActive) = 0 THEN
        // Fault cleared successfully
        SET HR4 (FaultReset) = 0
        GOTO SUCCESS
    ELSIF timeout_timer expired THEN
        SET HR4 (FaultReset) = 0
        GOTO RESET_FAILED

SUCCESS:
    // Return to idle state
    SET HR2 (mode code) = 0
    // Determine next action based on fault type

RESET_FAILED:
    // Fault persists - requires investigation
    // May need power cycle or mechanical fix
```

### Reset Timing Diagram

```
FaultActive IR5 ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                         ^
                   |                         |
                   Fault                     Cleared
                   detected

MotionEnable HR3 ‾‾‾‾\________________________/‾‾‾‾
                    ^                       ^
                    |                       |
                    Disable                 Re-enable
                    for reset               after clear

mode_code HR2  XXXX|--- fault_code ---------|0
                   ^                        ^
                   |                        |
                   Mirror                   Return
                   code                     to idle

FaultReset HR4   ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
                   ^                   ^
                   |                   |
                   Asserted            Release
                   (level, held)       after clear
```

---

## 5. Recovery Decision Tree

```
FAULT DETECTED (IR5 G_mbFaultActive = 1)
           |
           v
    Read fault code
           |
    +------+------+------+------+------+------+------+
    |      |      |      |      |      |      |      |
    1      2      3      4      5      6      7      8
    |      |      |      |      |      |      |      |
    v      v      v      v      v      v      v      v
 Retry  Check  Move   Fix    Check  STOP   (res.) Check
 mode   drive  from   link   press  Invest        sensor
        status limit         ure    igate         loop

After reset, determine appropriate next action:

1 (Handshake)   -> Retry previous mode command
2 (Drive)       -> May need power cycle; re-home (Mode 110) after an encoder alarm
3 (Position)    -> If still out of limits, jog back in via ST_RECOVERY
4 (Comm)        -> Restore the link, press Stop on the master, reset
5 (Piston)      -> Check pressure, move away from exit
6 (Limit)       -> Investigate; if still on switch, jog off via ST_RECOVERY
7 (Encoder)     -> Reserved; encoder alarms arrive as 2
8 (Pressure)    -> Check the transducer loop, reset once IR19-20 is plausible
```

---

## 6. Fault Prevention Guidelines

### Handshake Faults
- Wait for the mode confirmation in IR3 before raising motion enable
- Don't modify mode during handshake window

### Drive Faults
- Stay within motor thermal limits
- Ensure adequate power supply capacity
- Regular drive maintenance

### Position Faults
- Command positions well within limits
- Use appropriate acceleration/deceleration
- Monitor position feedback during motion

### Piston Exit Faults
- Vent the chamber (IR19-20 below `G_cfgExitGuardPressure`) before retracting below the exit boundary
- Do not request Go Home or homing with a charged chamber
- Keep the pressure target above what the charge holds at the exit boundary

### Limit Switch Faults
- Verify position tracking after homing
- Check for mechanical obstruction
- Regular limit switch testing

### Pressure Sensor Faults
- Power the transducer from the same supply as the slave so both are present together
- Keep the shunt and its return to the controller's analog ground short
- Watch IR19-20 at atmosphere before starting pressure control

### Encoder Faults
- Monitor encoder battery status
- Replace battery proactively
- Ensure proper cable routing (no noise sources)
- Re-home (Mode 110) after an encoder alarm, an encoder battery loss, or a
  controller SRAM battery failure or controller replacement

---

## 7. Quick Reference

### Fault Code Table

| Code | Name | Quick Fix |
|------|------|-----------|
| 0 | None | - |
| 1 | Handshake | Retry mode |
| 2 | Drive | Check drive |
| 3 | Position | Jog back in (ST_RECOVERY) |
| 4 | Comm | Restore the link, then reset |
| 5 | Piston Exit | Vent the chamber, or move in |
| 6 | Limit Switch | Investigate; jog off (ST_RECOVERY) |
| 7 | Encoder | Reserved (see Drive) |
| 8 | Pressure Sensor | Check the transducer loop and shunt |

### Reset Checklist

1. [ ] MotionEnable (HR3) = 0
2. [ ] Mode code (HR2) = fault code (mirror, full value)
3. [ ] Wait 20ms for stable
4. [ ] Hold FaultReset (HR4) = 1 (level-based, not a pulse)
5. [ ] Wait for IR5 (G_mbFaultActive) = 0
6. [ ] Release FaultReset (HR4)
7. [ ] Set mode code (HR2) = 0
8. [ ] Take corrective action per fault type
