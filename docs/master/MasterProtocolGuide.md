# Master Protocol Guide

## Gen3 SOS Compressor Servo Controller
### Simulink Desktop Real-Time Master Development Guide

---

## 1. Overview

This document provides step-by-step guidance for implementing the Simulink Desktop Real-Time (DTRT) master controller that interfaces with the MP2600iec servo slave via digital and analog I/O signals through the NI PCI 6251 DAQ.

### Architecture Summary

```
+-------------------+        Digital/Analog I/O        +-------------------+
|                   |  =============================>  |                   |
|  Simulink DTRT    |        NI PCI 6251 DAQ           |  MP2600iec Slave  |
|  Master           |  <=============================  |                   |
|                   |        (8 DI, 8 DO, AI, AO)      |                   |
+-------------------+                                  +-------------------+
```

### Key Principles

1. **Master commands, slave executes**: The master requests modes via DI0-DI2, slave confirms via DO0-DO2
2. **Handshake protocol**: All mode transitions require confirmed handshake within timeout
3. **Fault code mirroring**: During fault reset, master must echo fault code back to slave
4. **Slave enforces safety**: Position limits are enforced by slave

---

## 2. Digital I/O Interface Summary

### Master Outputs (Slave Digital Inputs)

| Master DAQ Pin | Signal Name | Function |
|----------------|-------------|----------|
| P0.5 (pin 51) | `G_diModeBit0` | Mode command bit 0 (LSB) |
| P0.1 (pin 17) | `G_diModeBit1` | Mode command bit 1 |
| P0.6 (pin 16) | `G_diModeBit2` | Mode command bit 2 (MSB) |
| P0.2 (pin 49) | `G_diMotionEnable` | Motion enable signal |
| P0.7 (pin 48) | `G_diFaultReset` | Fault reset request |

The master DAQ also drives P0.0 (pin 52, safety control relay enable - brake
disengage / safe-torque-off) and P0.4 (pin 19, stir bar motor); these are not
part of the slave protocol. P0.3 (pin 47) is unused. See
[IOReference.md §7](IOReference.md#7-ni-pci-6251-daq-pin-mapping) for the full
wiring, including analog.

### Master Inputs (Slave Digital Outputs)

| Master DAQ Pin | Signal Name | Normal Mode | Fault Mode |
|----------------|-------------|-------------|------------|
| PFI14/P2.6 (pin 1) | `G_doModeConfBit0` | Mode confirm bit 0 | Fault code bit 0 |
| PFI12/P2.4 (pin 2) | `G_doModeConfBit1` | Mode confirm bit 1 | Fault code bit 1 |
| PFI9/P2.1 (pin 3) | `G_doModeConfBit2` | Mode confirm bit 2 | Fault code bit 2 |
| PFI8/P2.0 (pin 37) | `G_doPerformanceStatus` | Mode-dependent status | - |
| PFI7/P1.7 (pin 38) | `G_doFaultActive` | LOW (normal) | HIGH (fault) |
| PFI6/P1.6 (pin 5) | `G_doHomingComplete` | Homing complete pulse (ST_HOME_COMPLETE only) | - |
| PFI15/P2.7 (pin 39) | `G_doInMotion` | Axis moving indicator | - |

**Note**: The slave sinks these lines, so the DAQ reads logic 0 when the
slave signal is asserted - the master must invert them in software. The
slave's `G_doBrakeDisengage` output drives the brake circuit directly and is
**not wired to the master**; the master has no brake status input. DO0-DO2
read as mode confirmation only while the handshake manager is active
(ST_IDLE, ST_BRAKE_HOLD, ST_HOLD_POSITION, ST_RECOVERY) - during all other
(operating/homing) states they read 000 unless a fault is active. Do not
treat 000 during operation as loss of mode.

---

## 3. Mode Encoding

### 3-Bit Mode Values

| Binary | Decimal | Mode Name | Description |
|--------|---------|-----------|-------------|
| 000 | 0 | Idle | Drive OFF, brake engaged |
| 001 | 1 | Brake Hold | Drive ON, brake engaged |
| 010 | 2 | Position Control | Analog input = position command |
| 011 | 3 | Velocity Control | Analog input = velocity command |
| 100 | 4 | Torque Control | Analog input = torque command |
| 101 | 5 | Go Home | Move to home position (G_cfgGoHomePosition) |
| 110 | 6 | Home to Limit | Homing via limit switch; re-establishes the zero |
| 111 | 7 | Reserved | Not implemented - confirmed but ignored; do not command |

---

## 4. Mode Entry Handshake Protocol

### Sequence Diagram

```
Master                                          Slave
  | (G_diMotionEnable is LOW)                       |
  |                                               |
  |  1. Set mode bits (DI0-DI2)                   |
  |---------------------------------------------->|
  |                                               |
  |              [Slave sees stable mode bits]    |
  |                                               |
  |  2. Confirm bits match (DO0-DO2)              |
  |<----------------------------------------------|
  |                                               |
  |  3. Master verifies confirmation              |
  |     (command == confirmation)                 |
  |                                               |
  |  4. Set G_diMotionEnable = HIGH                 |
  |---------------------------------------------->|
  |                                               |
  |             [Slave sees MotionEnable HIGH     |
  |              and validates mode match]        |
  |                                               |
  |             [HANDSHAKE COMPLETE]              |
  |             [Slave begins motion/action]      |
  |                                               |
```

### Timing Requirements

| Parameter | Value | Description |
|-----------|-------|-------------|
| Handshake Timeout | 500 ms | Max time for slave to confirm mode |
| Mode Stable Time | 20 ms | Mode bits must be stable before handshake starts |
| Drive Enable Delay | 100 ms | Time for drive to become ready |
| Brake Release Delay | 100 ms | Mechanical brake release time |

### Master State Machine for Mode Entry

```
IDLE:
    IF mode_request != current_mode THEN
        SET mode_bits = mode_request   // G_diMotionEnable stays LOW
        START handshake_timer
        GOTO WAIT_CONFIRM
    END_IF

WAIT_CONFIRM:
    IF confirm_bits == mode_bits THEN
        SET G_diMotionEnable = TRUE    // only now, after confirmation
        STOP handshake_timer
        current_mode = mode_request
        GOTO MODE_ACTIVE
    ELSIF handshake_timer > 500ms THEN
        GOTO HANDSHAKE_TIMEOUT
    END_IF

MODE_ACTIVE:
    // Normal operation
    IF want_different_mode THEN
        SET G_diMotionEnable = FALSE
        GOTO WAIT_HOLD
    END_IF

WAIT_HOLD:
    // Slave transitions to ST_HOLD_POSITION
    IF G_doInMotion == FALSE THEN
        // Slave has halted, ready for new mode
        GOTO IDLE
    END_IF

HANDSHAKE_TIMEOUT:
    // Slave will enter fault state
    // Wait for G_doFaultActive = TRUE
    GOTO FAULT_HANDLING
```

### Important: Inter-Mode Transitions

When changing between operational modes (e.g., Position to Velocity):

1. **Drop G_diMotionEnable LOW** - signals mode change request
2. **Wait for G_doInMotion = FALSE** - slave performs controlled halt (from Position mode the command is first ramped to rest at `G_cfgPosGateAccelMax`, up to 1.0 s at defaults, before the halt)
3. **Set new mode bits** - while G_diMotionEnable still LOW
4. **Wait for confirmation** - slave confirms new mode on DO0-DO2 while G_diMotionEnable is still LOW
5. **Raise G_diMotionEnable HIGH** - only after confirmation, completing the handshake for the new mode

**WARNING**: Do NOT change mode bits while G_diMotionEnable is HIGH. This will cause handshake timeout fault.

---

## 5. Fault Detection and Recovery

### Detecting Faults

Monitor `G_doFaultActive` (slave DO4) continuously:
- **LOW**: Normal operation
- **HIGH**: Fault condition active

When `G_doFaultActive` goes HIGH:
- DO0-DO2 now contain fault code (not mode confirmation)
- All motion stops
- Slave is in ST_FAULT state

### Fault Code Reading

When `G_doFaultActive == HIGH`, read DO0-DO2 as fault code:

| Binary | Fault | Description | Recovery Action |
|--------|-------|-------------|-----------------|
| 000 | None | No fault / cleared | N/A |
| 001 | Handshake | Timeout or mismatch | Retry handshake |
| 010 | Drive | Servo amplifier fault | Check drive, reset |
| 011 | Position | Software limit exceeded | Command safe position; if still out, jog in via ST_RECOVERY |
| 100 | Reserved | Never raised | N/A |
| 101 | Piston Exit | Safety guard triggered | Reduce force, check pressure |
| 110 | Limit Switch | Unexpected limit activation | Check mechanics; if still on switch, jog off via ST_RECOVERY |
| 111 | Encoder | Reserved - encoder alarms arrive as Drive (010) | Re-home (Mode 110) after an encoder alarm |

### Fault Reset Handshake

**CRITICAL**: The master must MIRROR the fault code during reset.

```
Master                                          Slave
  |                                               |
  |  1. Observe G_doFaultActive = HIGH              |
  |<----------------------------------------------|
  |                                               |
  |  2. Read fault code from DO0-DO2              |
  |<----------------------------------------------|
  |     (e.g., fault_code = 011 = Position)       |
  |                                               |
  |  3. Set G_diMotionEnable = LOW                  |
  |---------------------------------------------->|
  |                                               |
  |  4. MIRROR fault code to DI0-DI2              |
  |---------------------------------------------->|
  |     (e.g., set DI0=0, DI1=0, DI2=1)           |
  |                                               |
  |  5. Hold G_diFaultReset HIGH (level, not edge)  |
  |---------------------------------------------->|
  |                                               |
  |              [Slave validates mirror]         |
  |              [If match: clear fault]          |
  |                                               |
  |  6. G_doFaultActive goes LOW                    |
  |<----------------------------------------------|
  |                                               |
  |  7. Release G_diFaultReset = LOW                |
  |---------------------------------------------->|
  |                                               |
  |  8. Return to normal mode commands            |
  |                                               |
```

### Master Fault Reset State Machine

```
FAULT_DETECTED:
    fault_code = READ(DO0-DO2)
    SET G_diMotionEnable = FALSE
    SET mode_bits = fault_code  // MIRROR the code
    WAIT 50ms  // Ensure stable
    GOTO ASSERT_RESET

ASSERT_RESET:
    SET G_diFaultReset = TRUE (level - hold while conditions remain true)
    START reset_timer
    GOTO WAIT_CLEAR

WAIT_CLEAR:
    IF G_doFaultActive == FALSE THEN
        SET G_diFaultReset = FALSE
        GOTO FAULT_CLEARED
    ELSIF reset_timer > 1000ms THEN
        // Reset failed - may need operator intervention
        SET G_diFaultReset = FALSE
        GOTO FAULT_PERSISTENT
    END_IF

FAULT_CLEARED:
    // Fault resolved, return to idle
    SET mode_bits = 000  // Idle
    GOTO IDLE

FAULT_PERSISTENT:
    // Cannot clear fault automatically
    // Display fault code to operator
    // May require power cycle or homing
```

---

## 6. Motion Control via Analog Interface

### Analog Output (Master to Slave): Reference Command

**Signal**: AI0 on slave (`G_aiReference`, `AT %IW0 : LREAL`), driven by the master's AO.
**Range**: -10V to +10V (presented in ST as an `LREAL` volt value directly — no raw INT scaling).

| Mode | Scaling | Formula |
|------|---------|---------|
| Position (two-stage) | 0–200 mm ↔ -10V..+5V, 200–305 mm ↔ +5V..+10V | Stage 1: `V = pos*0.075 - 10`; Stage 2: `V = (pos-200)*0.0476 + 5` |
| Velocity | -10V..+10V = -7..+7 mm/s (clamped to ±5 mm/s) | `voltage = velocity / 0.7` |
| Torque | -10V..+10V = -100%..+100% | `voltage = torque_percent / 10` |

**Note**: Position command uses the same two-stage mapping as position feedback — sending the voltage that corresponds to a given position in feedback will command that same position. See [AnalogScalingReference](../slave/reference/AnalogScalingReference.md) for the full formulas and derivation.

### Analog Input (Slave to Master): Position Feedback

**Signal**: AO0 on slave (to master's AI)
**Range**: -10V to +10V

**Two-Stage Position Mapping**:
- Stage 1: 0mm to 200mm maps to -10V to +5V
- Stage 2: 200mm to 305mm maps to +5V to +10V

**Inverse Formulas for Master**:
```
IF voltage <= 5.0V THEN
    position = (voltage + 10) / 15 * 200    // 0 to 200mm
ELSE
    position = 200 + (voltage - 5) / 5 * 105  // 200 to 305mm
END_IF
```

### Position Mode: Entry Requirement and Command Shaping

**WARNING**: Before asserting `G_diMotionEnable` for Position mode (010), the master must
drive its analog position reference (AO, per the two-stage mapping above) to match the
slave's currently reported actual position (read back via the inverse formulas above). On
entry to `ST_POSITION_CTRL` the slave seeds its internal command at the actual position and
then ramps it toward whatever the reference commands, at the configured velocity/acceleration
limits — it no longer faults on a mismatch at entry, so an unmatched reference will now be
**executed as a move** rather than rejected.

Recall that the analog position map is **not** centre-zero: 0V corresponds to **133.3mm**
(-10V -> 0mm, +5V -> 200mm, +10V -> 305mm), which is a position inside the cylinder. A DAC
resting at 0V with the piston retracted (near 300mm) would therefore command a long
compression stroke the moment motion is enabled. Match the reference to actual position
first.

**Setpoint behavior**: Position setpoints (steps in the reference) are executed as
trapezoidal-velocity moves at the slave's configured limits — velocity `G_cfgVelLimitNormal`
(5 mm/s default) and acceleration `G_cfgPosGateAccelMax` — landing on the setpoint without
overshoot. If the master instead ramps the reference itself, the slave's command follows with
a small lag of approximately `v^2 / (2*A)`, where `v` is the reference's rate of change and
`A` is `G_cfgPosGateAccelMax`.

**Stall behavior**: If the axis cannot follow the reference (e.g., the torque limit is
reached during compression, or the axis is mechanically blocked), the slave holds its
internal command no more than `G_cfgPosGateMaxDeviation` (2.0mm default) ahead of the actual
position rather than faulting. Motion resumes automatically once the axis catches up or the
reference reverses direction. The master should judge arrival using position feedback (AI),
not elapsed time.

---

## 7. Recommended Master Architecture

### Simulink Model Structure

```
+------------------+     +------------------+     +------------------+
| Mode Request     |---->| Handshake        |---->| Output Signals   |
| Subsystem        |     | State Machine    |     | to NI DAQ        |
+------------------+     +------------------+     +------------------+
                               ^
                               |
+------------------+     +------------------+
| Input Signals    |---->| Fault Handler    |
| from NI DAQ      |     | Subsystem        |
+------------------+     +------------------+
                               |
                               v
                        +------------------+
                        | Analog Reference |
                        | Generator        |
                        +------------------+
```

### Key Subsystems

#### 1. Handshake State Machine
- Stateflow chart implementing mode entry protocol
- Handles handshake timeout detection
- Manages inter-mode transitions via HOLD state

#### 2. Fault Handler
- Monitors G_doFaultActive continuously
- Implements fault code mirroring
- Manages fault reset handshake

#### 3. Analog Reference Generator
- Mode-dependent reference signal generation
- Position trajectory planning
- Velocity/torque command generation

### Recommended Sample Rates

| Function | Sample Rate | Rationale |
|----------|-------------|-----------|
| Digital I/O | 1 kHz | Handshake response time |
| Analog Output | 1 kHz | Smooth motion reference |
| Analog Input | 1 kHz | Position feedback |
| State Machine | 1 kHz | Responsive mode changes |

---

## 8. Startup Sequence

### Recommended Power-On Sequence

```
1. Power on MP2600iec
   - Slave initializes
   - Slave enters ST_IDLE; the absolute encoder and the controller's
     stored offset keep the coordinate frame, so no homing is needed

2. Initialize Simulink model
   - Set all outputs LOW initially
   - mode_bits = 000 (Idle)
   - G_diMotionEnable = FALSE
   - G_diFaultReset = FALSE

3. Verify communication
   - Read G_doFaultActive (should be LOW)
   - Read confirmation bits (should match 000)

4. Enter operational mode
   - Command desired mode (010, 011, or 100)
   - Wait for handshake confirmation
   - Begin motion control
```

---

## 9. Common Scenarios

### Scenario 1: Normal Position Control

```
1. Master: Set mode_bits = 010 (G_diMotionEnable stays LOW)
2. Slave: Sets confirm_bits = 010
3. Master: Verifies confirm_bits == mode_bits, raises G_diMotionEnable = TRUE
4. Slave: Handshake complete; enables drive, releases brake, enters ST_POSITION_CTRL
5. Master: Output position reference on AO
6. Slave: Follows position, outputs feedback
7. Master: Read position from AI
```

### Scenario 2: Mode Change (Position to Velocity)

```
1. Master: Set G_diMotionEnable = FALSE
2. Slave: Executes MC_Stop, enters ST_HOLD_POSITION
3. Master: Wait for G_doInMotion = FALSE
4. Master: Set mode_bits = 011 (Velocity), G_diMotionEnable still LOW
5. Slave: Confirms mode_bits = 011
6. Master: Verifies confirmation, raises G_diMotionEnable = TRUE
7. Slave: Handshake complete, begin velocity control
```

### Scenario 3: Fault Recovery

```
1. [Fault occurs - e.g., position limit exceeded]
2. Slave: Sets G_doFaultActive = HIGH, fault_code = 011
3. Master: Detects G_doFaultActive = HIGH
4. Master: Reads fault_code = 011
5. Master: Sets G_diMotionEnable = FALSE
6. Master: Mirrors code: mode_bits = 011
7. Master: Holds G_diFaultReset HIGH (level-based, not a pulse)
8. Slave: Validates mirror, clears fault, enters ST_BRAKE_HOLD (drive stays on)
9. Slave: Sets G_doFaultActive = LOW
10. Master: Select a mode - e.g. mode_bits = 000 to shut down gracefully to
    IDLE, or resume operation from Brake Hold
```

### Scenario 4: Limit Recovery (switch/soft-limit still active at reset)

For `FAULT_LIMIT_SWITCH` (110) or `FAULT_POSITION` (011), if the limit is still
active when the reset clears, the slave enters the `ST_RECOVERY` hub instead of
returning to idle/brake-hold. Jog off the limit, then resume normally:

```
1. [Limit fault active] Slave: G_doFaultActive = HIGH, fault_code = 110 (or 011)
2. Master: G_diMotionEnable = FALSE; mirror code to mode_bits; hold G_diFaultReset HIGH (level-based)
3. Slave: clears fault; limit still active -> enters ST_RECOVERY (drive stays on)
          (if it had timed out to ST_FAULT_IDLE, the drive re-enables first)
4. Master: set mode_bits = 010 (Position) or 011 (Velocity), G_diMotionEnable still LOW
5. Master: wait for confirm_bits to match, then raise G_diMotionEnable
6. Master: drive analog reference AWAY from the limit
          - toward-limit commands are clamped to zero (no re-fault); reverse to proceed
7. [Switch clears AND position inside soft limits]
8. Master: drop G_diMotionEnable -> slave goes to ST_HOLD_POSITION (normal branch)
9. Master: select any mode and continue (re-home if position is uncertain)
```

To abandon recovery without retracting, select Mode 001 (Brake Hold, drive stays on)
or Mode 000 (Idle, shut down) from the hub — both are honored even with the limit
still active. See the [Fault Code Reference](FaultCodeReference.md#limit-recovery-state-st_recovery).

---

## 10. Troubleshooting

### Handshake Timeout

**Symptom**: Mode confirmation never matches command

**Possible Causes**:
- Drive not ready (check drive status)
- I/O wiring issue

**Resolution**:
1. Check G_doFaultActive - if HIGH, handle fault first
2. Verify DI/DO wiring connections
3. Ensure mode bits are stable for 20ms before expecting confirmation

### Fault Won't Clear

**Symptom**: G_diFaultReset asserted but G_doFaultActive stays HIGH

**Possible Causes**:
- Fault code not mirrored correctly
- G_diMotionEnable not LOW
- Underlying condition still present

**Resolution**:
1. Verify G_diMotionEnable = FALSE
2. Verify mode_bits exactly matches fault code
3. Check if fault condition persists (e.g., still at position limit)

### Position Feedback Incorrect

**Symptom**: Analog input values don't match expected positions

**Possible Causes**:
- Two-stage mapping not applied
- Analog calibration offset
- Coordinate frame lost (encoder alarm, controller battery failure or
  controller replacement)

**Resolution**:
1. Apply correct two-stage inverse mapping
2. Verify analog I/O calibration
3. Re-home with Mode 110 if the frame was lost

---

## Appendix A: Quick Reference Card

### Mode Commands
| Mode | Binary | Decimal |
|------|--------|---------|
| Idle | 000 | 0 |
| Brake Hold | 001 | 1 |
| Position | 010 | 2 |
| Velocity | 011 | 3 |
| Torque | 100 | 4 |
| Go Home | 101 | 5 |
| Home Limit | 110 | 6 |
| Reserved | 111 | 7 |

### Fault Codes
| Fault | Binary | Decimal |
|-------|--------|---------|
| None | 000 | 0 |
| Handshake | 001 | 1 |
| Drive | 010 | 2 |
| Position | 011 | 3 |
| Reserved | 100 | 4 |
| Piston Exit | 101 | 5 |
| Limit Switch | 110 | 6 |
| Encoder | 111 | 7 |

### Critical Timings
| Parameter | Value |
|-----------|-------|
| Handshake Timeout | 500 ms |
| Fault Reset Timeout | 1000 ms |
| Mode Stable Time | 20 ms |
