# Master Protocol Guide

## Gen3 SOS Compressor Servo Controller
### Simulink Desktop Real-Time Master Development Guide

---

## 1. Overview

This document provides step-by-step guidance for implementing the Simulink Desktop Real-Time (DTRT) master controller that interfaces with the MP2600iec servo slave. The protocol signals travel over **Modbus TCP**: the master writes holding registers and reads input registers every 50 ms. The register layout is in [ModbusRegisterMap.md](ModbusRegisterMap.md). Only the safety relay line and the plant I/O stay on the NI PCI 6221 DAQ.

### Architecture Summary

```
+-------------------+      Modbus TCP, port 502        +-------------------+
|                   |  == HR1-8: heartbeat, mode,  ==> |                   |
|  Simulink DTRT    | enable, reset, reference, speed  |  MP2600iec Slave  |
|  Master           |  <= IR1-20: echo, counter, ===== |  (Modbus server)  |
|                   |  confirm, fault, status, feedback|                   |
+-------------------+                                  +-------------------+
         |  P0.0 safety relay (DAQ, hardwired)                  ^
         +------------------------------------------------------+
```

### Key Principles

1. **Master commands, slave executes**: The master requests modes in the mode code register, the slave confirms in the confirmation register
2. **Handshake protocol**: All mode transitions require confirmed handshake within timeout
3. **Fault code mirroring**: During fault reset, master must echo fault code back to slave
4. **Slave enforces safety**: Position limits are enforced by slave

---

## 2. Signal Summary

### Master to Slave (holding registers, one FC16 write)

| Register | Slave variable | Function |
|----------|----------------|----------|
| HR1 | `G_mbHeartbeat` | Master step counter; the slave faults if it stops |
| HR2 | `G_mbModeCode` | Mode code (0-7), or the full fault code during reset |
| HR3 | `G_mbMotionEnable` | 1 = MotionEnable |
| HR4 | `G_mbFaultReset` | 1 = FaultReset (level) |
| HR5-6 | `G_mbReference` | REAL reference: mm, mm/s or % by mode |
| HR7-8 | `G_mbPositionSpeed` | REAL speed of a position move, mm/s (0 holds position) |

The slave debounces HR2 (`G_cfgDebounceTimeMode`, 50 ms), HR3
(`G_cfgDebounceTimeMotion`, 20 ms) and HR4 (`G_cfgDebounceTimeFault`, 20 ms)
before acting on them. The master DAQ drives P0.0 (safety control relay
enable - brake disengage / safe-torque-off) and P0.4 (stir bar motor); these
are not part of the slave protocol. See [IOReference.md](IOReference.md) for
the wiring.

### Slave to Master (input registers, one FC04 read)

| Register | Slave variable | Normal Mode | Fault Mode |
|----------|----------------|-------------|------------|
| IR1 | `G_mbHeartbeatEcho` | Last HR1 received | |
| IR2 | `G_mbSlaveCounter` | Scan counter; the master faults if it stops | |
| IR3 | `G_mbConfCode` | Mode confirmation | 0 |
| IR4 | `G_mbFaultCode` | 0 | Full fault code |
| IR5 | `G_mbFaultActive` | 0 | 1 |
| IR6 | `G_mbInMotion` | Axis moving indicator | |
| IR7 | `G_mbPerformanceStatus` | Mode-dependent status | |
| IR8 | `G_mbHomingComplete` | Homing complete (ST_HOME_COMPLETE only) | |
| IR11-18 | `G_mbActual*`, `G_mbCommandedPosition` | REAL position, velocity, torque, commanded position | |
| IR19-20 | `G_mbActualPressure` | REAL pressure (MPa gauge) | |

**Note**: The slave's `G_doBrakeDisengage` output drives the brake circuit
directly and is **not carried by the link**; the master has no brake status
input. IR3 holds a mode confirmation only while the handshake manager is
active (ST_IDLE, ST_BRAKE_HOLD, ST_HOLD_POSITION, ST_RECOVERY) - during all
other states, including operating, homing and fault states, it reads 0. Do
not treat 0 during operation as loss of mode.

### Link Supervision

| Side | Condition | Action |
|------|-----------|--------|
| Slave | HR1 unchanged for `G_cfgCommTimeout` (300 ms) while motion is commanded | `FAULT_COMM` (code 4) through the normal fault path |
| Master | IR2 unchanged for 300 ms | E-stop path: MotionEnable low, safety relay off, status LINK_LOST. Latched until the counter advances again and the operator sends Stop |

A dropped TCP connection stops the Simulink model, which sets the DAQ
outputs to their final values (safety relay off).

### Step Probe (optional side channel)

Beside the protocol the slave carries a scan-rate data recorder, `FB_Probe`,
for experiments and benches: IR401-524 and HR111-114, read and written by
`bench/common/benchProbe.m` in the gaps between exchanges. It records the
commanded and actual velocity, torque, position, pressure, the raw pressure
input and a status word every scan into a ring of 8192 records, and marks the
record of the scan the slave entered `ST_FAULT`, so the history up to a fault
can be read back. Nothing in the protocol depends on it: the shell does not
read or write these registers, no master state or timeout depends on them, and
with `G_cfgProbeEnable` FALSE the slave records nothing. The registers and
state machines are in [ModbusRegisterMap.md](ModbusRegisterMap.md), section 6.

---

## 3. Mode Encoding

### Mode Values (HR2, confirmed in IR3)

| Binary | Decimal | Mode Name | Description |
|--------|---------|-----------|-------------|
| 000 | 0 | Idle | Drive OFF, brake engaged |
| 001 | 1 | Brake Hold | Drive ON, brake engaged |
| 010 | 2 | Position Control | Reference = position command (mm) |
| 011 | 3 | Velocity Control | Reference = velocity command (mm/s) |
| 100 | 4 | Torque Control | Reference = torque command (% of rated) |
| 101 | 5 | Go Home | Move to home position (G_cfgGoHomePosition) |
| 110 | 6 | Home to Limit | Homing via limit switch; re-establishes the zero |
| 111 | 7 | Pressure Control | Reference = target pressure (MPa gauge); the slave closes the loop on its transducer and commands velocity |

The binary form is used as a mode name in these documents (e.g. "Mode
110"); the register carries the decimal value. HR2 values above 7 decode
as Idle.

---

## 4. Mode Entry Handshake Protocol

### Sequence Diagram

```
Master                                          Slave
  | (MotionEnable, HR3, is 0)                     |
  |                                               |
  |  1. Write the mode code (HR2)                 |
  |---------------------------------------------->|
  |                                               |
  |              [Slave sees stable mode code]    |
  |                                               |
  |  2. Confirmation matches (IR3)                |
  |<----------------------------------------------|
  |                                               |
  |  3. Master verifies confirmation              |
  |     (command == confirmation)                 |
  |                                               |
  |  4. Set MotionEnable = 1 (HR3)                |
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
| Mode Code Debounce | 50 ms | HR2 must hold a new value this long before the slave acts on it (`G_cfgDebounceTimeMode`) |
| Confirmation Stable | 20 ms | The master requires IR3 to match this long (`cfg.tConfirmStable`) |
| Drive Enable Delay | 100 ms | Time for drive to become ready |
| Brake Release Delay | 100 ms | Mechanical brake release time |

### Master State Machine for Mode Entry

```
IDLE:
    IF mode_request != current_mode THEN
        SET mode_code = mode_request   // MotionEnable stays 0
        START handshake_timer
        GOTO WAIT_CONFIRM
    END_IF

WAIT_CONFIRM:
    IF conf_code == mode_code THEN
        SET MotionEnable = 1           // only now, after confirmation
        STOP handshake_timer
        current_mode = mode_request
        GOTO MODE_ACTIVE
    ELSIF handshake_timer > 500ms THEN
        GOTO HANDSHAKE_TIMEOUT
    END_IF

MODE_ACTIVE:
    // Normal operation
    IF want_different_mode THEN
        SET MotionEnable = 0
        GOTO WAIT_HOLD
    END_IF

WAIT_HOLD:
    // Slave transitions to ST_HOLD_POSITION
    IF InMotion (IR6) == 0 THEN
        // Slave has halted, ready for new mode
        GOTO IDLE
    END_IF

HANDSHAKE_TIMEOUT:
    // Slave will enter fault state
    // Wait for FaultActive (IR5) = 1
    GOTO FAULT_HANDLING
```

### Important: Inter-Mode Transitions

When changing between operational modes (e.g., Position to Velocity):

1. **Drop MotionEnable to 0** - signals mode change request. The slave checks the level, not the falling edge: MotionEnable at 0 in an operational or homing state stops the mode on the next scan, including when it dropped while the slave was still starting the mode (drive reset, enable, brake release)
2. **Wait for InMotion (IR6) = 0** - slave performs controlled halt (from Position mode the command is first ramped to rest at `G_cfgPosShaperAccelMax`, up to 1.0 s at defaults, before the halt)
3. **Write the new mode code** - while MotionEnable is still 0
4. **Wait for confirmation** - slave confirms the new mode in IR3 while MotionEnable is still 0
5. **Raise MotionEnable to 1** - only after confirmation, completing the handshake for the new mode

**WARNING**: Do NOT change the mode code while MotionEnable is 1. This will cause handshake timeout fault.

---

## 5. Fault Detection and Recovery

### Detecting Faults

Monitor `G_mbFaultActive` (IR5) continuously:
- **0**: Normal operation
- **1**: Fault condition active

When it goes to 1:
- IR4 holds the full fault code
- All motion stops
- Slave is in ST_FAULT state

### Fault Code Reading

When `G_mbFaultActive == 1`, read IR4 as the fault code:

| Code | Fault | Description | Recovery Action |
|------|-------|-------------|-----------------|
| 0 | None | No fault / cleared | N/A |
| 1 | Handshake | Timeout or mismatch | Retry handshake |
| 2 | Drive | Servo amplifier fault | Check drive, reset |
| 3 | Position | Software limit exceeded | Command safe position; if still out, jog in via ST_RECOVERY |
| 4 | Comm | Master heartbeat stopped | Restore the link, then reset |
| 5 | Piston Exit | Safety guard triggered | Reduce force, check pressure |
| 6 | Limit Switch | Unexpected limit activation | Check mechanics; if still on switch, jog off via ST_RECOVERY |
| 7 | Encoder | Reserved - encoder alarms arrive as Drive (2) | Re-home (Mode 110) after an encoder alarm |
| 8 | Pressure Sensor | Transducer loop current out of range during pressure control | Check the transducer loop; reset once IR19-20 reads a plausible pressure |

### Fault Reset Handshake

**CRITICAL**: The master must MIRROR the fault code during reset.

```
Master                                          Slave
  |                                               |
  |  1. Observe G_mbFaultActive = 1 (IR5)         |
  |<----------------------------------------------|
  |                                               |
  |  2. Read fault code from IR4                  |
  |<----------------------------------------------|
  |     (e.g., fault_code = 3 = Position)         |
  |                                               |
  |  3. Set G_mbMotionEnable = 0 (HR3)            |
  |---------------------------------------------->|
  |                                               |
  |  4. MIRROR the full fault code into HR2       |
  |---------------------------------------------->|
  |     (e.g., HR2 = 3)                           |
  |                                               |
  |  5. Hold G_mbFaultReset = 1 (level, not edge) |
  |---------------------------------------------->|
  |                                               |
  |              [Slave validates mirror]         |
  |              [If match: clear fault]          |
  |                                               |
  |  6. G_mbFaultActive goes to 0                 |
  |<----------------------------------------------|
  |                                               |
  |  7. Release G_mbFaultReset = 0                |
  |---------------------------------------------->|
  |                                               |
  |  8. Return to normal mode commands            |
  |                                               |
```

### Master Fault Reset State Machine

```
FAULT_DETECTED:
    fault_code = READ(IR4)
    SET MotionEnable = 0
    SET mode_code = fault_code  // MIRROR the code in HR2
    WAIT 50ms  // Ensure stable
    GOTO ASSERT_RESET

ASSERT_RESET:
    SET FaultReset = 1 (level - hold while conditions remain true)
    START reset_timer
    GOTO WAIT_CLEAR

WAIT_CLEAR:
    IF FaultActive (IR5) == 0 THEN
        SET FaultReset = 0
        GOTO FAULT_CLEARED
    ELSIF reset_timer > 1000ms THEN
        // Reset failed - may need operator intervention
        SET FaultReset = 0
        GOTO FAULT_PERSISTENT
    END_IF

FAULT_CLEARED:
    // Fault resolved, return to idle
    SET mode_code = 0  // Idle
    GOTO IDLE

FAULT_PERSISTENT:
    // Cannot clear fault automatically
    // Display fault code to operator
    // May require power cycle or homing
```

---

## 6. Reference and Feedback

### Reference (Master to Slave): HR5-6

**Signal**: `G_mbReference`, a REAL in engineering units, copied to
`G_sysReference` by `PRG_Input`. Low word first (see ModbusRegisterMap.md).

| Mode | Unit | Limits applied by the slave |
|------|------|-----------------------------|
| Position | mm | Soft limits, position path shaper (speed from HR7-8, acceleration `G_cfgPosShaperAccelMax`) |
| Velocity | mm/s | `G_cfgVelLimitMax` (6 mm/s) |
| Torque | % of rated | `G_cfgTorqueLimitNormal` |
| Pressure | MPa gauge | `0` to `G_cfgPressureTargetMax` (15 MPa); velocity within `G_cfgPressureVelMax` (5 mm/s) and the stop law; torque within `G_cfgPressureTorqueLimit` |

### Position Speed (Master to Slave): HR7-8

**Signal**: `G_mbPositionSpeed`, a REAL in mm/s, copied to `G_sysPositionSpeed` by
`PRG_Input`. The master writes it every exchange, with HR1-6 in the same FC16 write.
In `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION` the slave evaluates
`LIMIT(0, G_sysPositionSpeed, G_cfgVelLimitMax)` every scan as the speed limit of
`FB_PositionPathShaper`, so a change takes effect mid-move: a lower speed slows
the command at `G_cfgPosShaperAccelMax`, a higher one speeds it up. A speed of 0
decelerates the command to rest at `G_cfgPosShaperAccelMax` and holds position;
restoring the speed resumes the move. Other modes ignore it. The master limits it
to `0` to `cfg.positionSpeedMax` (not above `G_cfgVelLimitMax`); the operator app
sets it in its move-speed field.

The reference is read by the slave at most every 50 ms plus one 4 ms
service tick. The master sends the next value with the same write that
carries the mode code and MotionEnable.

### Feedback (Slave to Master): IR11-20

`G_mbActualPosition` (mm), `G_mbActualVelocity` (mm/s), `G_mbActualTorque`
(%), `G_mbCommandedPosition` (mm) and `G_mbActualPressure` (MPa gauge), as
REAL values from the `G_sys*` globals, published once per scan and read
every 50 ms. The pressure is valid in every mode.

### Pressure Mode: Reference and Status

The slave scales the transducer on its analog input (`FB_PressureInput`)
and closes a PI loop (`FB_PressureController`) that commands piston
velocity through `Y_DirectControl` in VelocityTLMode, so the master only
sends the target. `G_mbPerformanceStatus` (IR7) is 1 while the measured
pressure is within `G_cfgPressureInBand` (0.1 MPa) of the target. The
target may change at any time while the mode is active; it is clamped to
`G_cfgPressureTargetMax`. Dropping MotionEnable halts the axis as in
velocity mode. A transducer loop current out of range for
`G_cfgPressureSensorFaultTime` raises `FAULT_PRESSURE_SENSOR` (8) in this
mode only; the master should check IR19-20 for a plausible pressure before
requesting the mode.

### Position Mode: Entry Requirement and Command Shaping

**WARNING**: Before raising MotionEnable for Position mode (010), the master should set
the reference (HR5-6) to the position it wants, normally the slave's actual position
(IR11-12) when no move is intended. On entry to `ST_POSITION_CTRL` the slave seeds its
internal command at the actual position and then ramps it toward the reference at the
configured velocity/acceleration limits, so any difference between reference and actual
position is **executed as a move**. A reference left at 0 mm with the piston retracted
commands a full stroke the moment motion is enabled.

**Setpoint behavior**: Position setpoints (steps in the reference) are executed as
trapezoidal-velocity moves by `FB_PositionPathShaper`: `Y_DirectControl` does no path
shaping in PositionMode (the command reaches the drive as given), so the shaper's
speed (the position speed in HR7-8, at most `G_cfgVelLimitMax`) and acceleration
(`G_cfgPosShaperAccelMax`) limits are what bound a move — landing on the setpoint without
overshoot. If the master instead ramps the reference itself, the slave's command follows with
a small lag of approximately `v^2 / (2*A)`, where `v` is the reference's rate of change and
`A` is `G_cfgPosShaperAccelMax`.

**Stall behavior**: If the axis cannot follow the reference (e.g., the torque limit is
reached during compression, or the axis is mechanically blocked), the slave holds its
internal command no more than `G_cfgPosShaperMaxDeviation` (2.0mm default) ahead of the actual
position rather than faulting. Motion resumes automatically once the axis catches up or the
reference reverses direction. The master should judge arrival using position feedback
(IR11-12), not elapsed time.

---

## 7. Recommended Master Architecture

### Simulink Model Structure

```
+------------------+     +------------------+     +------------------+
| Mode Request     |---->| Handshake        |---->| Holding Registers|
| Subsystem        |     | State Machine    |     | (Modbus write)   |
+------------------+     +------------------+     +------------------+
                               ^
                               |
+------------------+     +------------------+
| Input Registers  |---->| Fault Handler    |
| (Modbus read)    |     | Subsystem        |
+------------------+     +------------------+
                               |
                               v
                        +------------------+
                        | Reference        |
                        | Generator        |
                        +------------------+
```

### Key Subsystems

#### 1. Handshake State Machine
- Stateflow chart implementing mode entry protocol
- Handles handshake timeout detection
- Manages inter-mode transitions via HOLD state

#### 2. Fault Handler
- Monitors FaultActive (IR5) continuously
- Implements fault code mirroring
- Manages fault reset handshake

#### 3. Reference Generator
- Mode-dependent reference signal generation
- Position trajectory planning
- Velocity/torque command generation

### Recommended Sample Rates

| Function | Period | Rationale |
|----------|--------|-----------|
| Modbus exchange (write + read) | 50 ms | The controller answers one request per 4 ms tick and a blocking exchange costs 10-25 ms on the PC |
| State machine (model step) | 25 ms | A blocking exchange may stall the model by one tick without loss |
| Safety relay, stir bar (DAQ) | 25 ms | Model step |

---

## 8. Startup Sequence

### Recommended Power-On Sequence

```
1. Power on MP2600iec
   - Slave initializes
   - Slave enters ST_IDLE; the absolute encoder and the controller's
     stored offset keep the coordinate frame, so no homing is needed

2. Initialize Simulink model
   - Safety relay (P0.0) off initially
   - Mode code = 0 (Idle)
   - MotionEnable = 0
   - FaultReset = 0

3. Verify communication
   - Wait for the slave counter (IR2) to advance
   - Read FaultActive (IR5, should be 0)
   - Read the confirmation (IR3, should be 0)

4. Enter operational mode
   - Command desired mode (010, 011, or 100)
   - Wait for handshake confirmation
   - Begin motion control
```

---

## 9. Common Scenarios

### Scenario 1: Normal Position Control

```
1. Master: Set mode code = 2 (MotionEnable stays 0)
2. Slave: Sets confirmation = 2
3. Master: Verifies confirmation == mode code, raises MotionEnable = 1
4. Slave: Handshake complete; enables drive, releases brake, enters ST_POSITION_CTRL
5. Master: Writes the position reference (HR5-6, mm) and the move speed (HR7-8, mm/s)
6. Slave: Follows position, publishes feedback
7. Master: Reads position from IR11-12
```

### Scenario 2: Mode Change (Position to Velocity)

```
1. Master: Set MotionEnable = 0
2. Slave: Executes MC_Stop, enters ST_HOLD_POSITION
3. Master: Wait for InMotion (IR6) = 0
4. Master: Set mode code = 3 (Velocity), MotionEnable still 0
5. Slave: Confirms mode 3
6. Master: Verifies confirmation, raises MotionEnable = 1
7. Slave: Handshake complete, begin velocity control
```

### Scenario 3: Fault Recovery

```
1. [Fault occurs - e.g., position limit exceeded]
2. Slave: Sets FaultActive (IR5) = 1, fault code (IR4) = 3
3. Master: Detects FaultActive = 1
4. Master: Reads fault code = 3
5. Master: Sets MotionEnable = 0
6. Master: Mirrors code: mode code (HR2) = 3
7. Master: Holds FaultReset = 1 (level-based, not a pulse)
8. Slave: Validates mirror, clears fault, enters ST_BRAKE_HOLD (drive stays on)
9. Slave: Sets FaultActive = 0
10. Master: Select a mode - e.g. mode code 0 to shut down gracefully to
    IDLE, or resume operation from Brake Hold
```

### Scenario 4: Limit Recovery (switch/soft-limit still active at reset)

For `FAULT_LIMIT_SWITCH` (6) or `FAULT_POSITION` (3), if the limit is still
active when the reset clears, the slave enters the `ST_RECOVERY` hub instead of
returning to idle/brake-hold. Jog off the limit, then resume normally:

```
1. [Limit fault active] Slave: FaultActive = 1, fault code = 6 (or 3)
2. Master: MotionEnable = 0; mirror the code in HR2; hold FaultReset = 1 (level-based)
3. Slave: clears fault; limit still active -> enters ST_RECOVERY (drive stays on)
          (if it had timed out to ST_FAULT_IDLE, the drive re-enables first)
4. Master: set mode code 2 (Position) or 3 (Velocity), MotionEnable still 0
5. Master: wait for the confirmation to match, then raise MotionEnable
6. Master: drive the reference AWAY from the limit
          - toward-limit commands are clamped to zero (no re-fault); reverse to proceed
7. [Switch clears AND position inside soft limits]
8. Master: drop MotionEnable -> slave goes to ST_HOLD_POSITION (normal branch)
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
- Modbus link not exchanging (IR2 not advancing, IR1 not echoing HR1)

**Resolution**:
1. Check FaultActive (IR5) - if 1, handle fault first
2. Verify the link: IR2 advances and IR1 echoes HR1
3. Hold the mode code for at least the 50 ms debounce before expecting confirmation

### Fault Won't Clear

**Symptom**: FaultReset asserted but FaultActive (IR5) stays 1

**Possible Causes**:
- Fault code not mirrored correctly
- MotionEnable not 0
- Underlying condition still present

**Resolution**:
1. Verify MotionEnable = 0
2. Verify HR2 exactly matches the fault code in IR4
3. Check if fault condition persists (e.g., still at position limit)

### Position Feedback Incorrect

**Symptom**: IR11-12 does not match the expected position

**Possible Causes**:
- REAL decoded with the wrong word order (the slave sends low word first)
- Coordinate frame lost (encoder alarm, controller battery failure or
  controller replacement)

**Resolution**:
1. Decode IR11-12 low word first (`modbusUnpack`)
2. Re-home with Mode 110 if the frame was lost

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
| Pressure | 111 | 7 |

### Fault Codes
| Fault | Code |
|-------|------|
| None | 0 |
| Handshake | 1 |
| Drive | 2 |
| Position | 3 |
| Comm | 4 |
| Piston Exit | 5 |
| Limit Switch | 6 |
| Encoder | 7 |
| Pressure Sensor | 8 |

### Critical Timings
| Parameter | Value |
|-----------|-------|
| Modbus exchange period | 50 ms |
| Link timeout (both sides) | 300 ms |
| Handshake Timeout | 500 ms |
| Fault Reset Timeout | 1000 ms |
| Mode Code Debounce (slave) | 50 ms |
| Confirmation Stable (master) | 20 ms |
