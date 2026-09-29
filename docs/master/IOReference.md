# I/O Reference

## Gen3 SOS Compressor Servo Controller
### Complete Signal Definitions

---

## 1. Digital Input Signals (Slave Perspective)

These are outputs from the NI DAQ master, inputs to the MP2600iec slave.

| Pin | Signal Name | Type | Description |
|-----|-------------|------|-------------|
| DI0 | `G_diModeBit0` | BOOL | Mode command bit 0 (LSB) |
| DI1 | `G_diModeBit1` | BOOL | Mode command bit 1 |
| DI2 | `G_diModeBit2` | BOOL | Mode command bit 2 (MSB) |
| DI3 | `G_diMotionEnable` | BOOL | Motion enable from master |
| DI4 | `G_diFaultReset` | BOOL | Fault reset command (level, not edge) |
| DI5 | Reserved | BOOL | Unused |
| DI6 | `G_diOvertravelNeg` | BOOL | Negative overtravel / home reference switch (PNP NC) |
| DI7 | `G_diOvertravelPos` | BOOL | Positive overtravel switch (PNP NC) |

### Signal Details

#### DI0-DI2: Mode Command Bits

Combined as 3-bit mode command value:
```
mode_command = DI2 * 4 + DI1 * 2 + DI0
```

| DI2 | DI1 | DI0 | Mode Value | Mode Name |
|-----|-----|-----|------------|-----------|
| 0 | 0 | 0 | 0 | Idle |
| 0 | 0 | 1 | 1 | Brake Hold |
| 0 | 1 | 0 | 2 | Position Control |
| 0 | 1 | 1 | 3 | Velocity Control |
| 1 | 0 | 0 | 4 | Torque Control |
| 1 | 0 | 1 | 5 | Go Home |
| 1 | 1 | 0 | 6 | Home to Limit |
| 1 | 1 | 1 | 7 | Reserved (raises FAULT_HOMING_REQ) |

#### DI3: Motion Enable

| State | Meaning | Slave Response |
|-------|---------|----------------|
| LOW | Motion disabled | Halt motion, enter HOLD state |
| HIGH | Motion enabled | Process mode command, enable motion |

**Timing**: Rising edge initiates handshake. Falling edge triggers controlled stop.

#### DI4: Fault Reset

| State | Meaning |
|-------|---------|
| HIGH | Valid while the other reset conditions hold - clears the fault |
| LOW | Normal (non-reset) state |

**Requirements for Valid Reset** (level-based - the slave checks this every
scan, it is not edge-triggered):
1. `G_diFaultReset` HIGH
2. `G_diMotionEnable` LOW
3. Mode bits (DI0-DI2) mirror the fault code and are stable

#### DI6-DI7: Overtravel Switches

**Wiring**: PNP Normally Closed (fail-safe)

`DI6` (`G_diOvertravelNeg`) is the negative overtravel switch, which also
serves as the home reference switch for Mode 110 homing. `DI7`
(`G_diOvertravelPos`) is the positive overtravel switch.

| Signal State | Physical Meaning |
|--------------|------------------|
| HIGH (TRUE) | Switch NOT triggered (normal) |
| LOW (FALSE) | Switch triggered OR wire broken |

**Safety**: Wire break or sensor failure results in LOW (triggered) state, which is the safe default.

---

## 2. Digital Output Signals (Slave Perspective)

These are outputs from the MP2600iec slave, inputs to the NI DAQ master.

| Pin | Signal Name | Normal Mode | Fault Mode |
|-----|-------------|-------------|------------|
| DO0 | `G_doModeConfBit0` | Mode confirm bit 0 | Fault code bit 0 |
| DO1 | `G_doModeConfBit1` | Mode confirm bit 1 | Fault code bit 1 |
| DO2 | `G_doModeConfBit2` | Mode confirm bit 2 | Fault code bit 2 |
| DO3 | `G_doPerformanceStatus` | Mode-dependent | - |
| DO4 | `G_doFaultActive` | LOW | HIGH |
| DO5 | `G_doHomingComplete` | Homing pulse | Homing pulse |
| DO6 | `G_doInMotion` | Motion indicator | - |
| DO7 | `G_doBrakeDisengage` | Brake release command (not wired to master) | - |

### Signal Details

#### DO0-DO2: Mode Confirmation / Fault Code

**Normal Mode** (`G_doFaultActive` = LOW):
```
mode_confirmed = DO2 * 4 + DO1 * 2 + DO0
```
Matches mode command when handshake complete. This is only meaningful while
the handshake manager is active (ST_IDLE, ST_BRAKE_HOLD, ST_HOLD_POSITION,
ST_RECOVERY); in every other state - including all operating and homing
states - DO0-DO2 read 000 (unless a fault is active, in which case they
carry the fault code). 000 while a mode is running is expected and must not
be treated as loss of mode.

**Fault Mode** (`G_doFaultActive` = HIGH):
```
fault_code = DO2 * 4 + DO1 * 2 + DO0
```

| DO2 | DO1 | DO0 | Fault Code | Fault Name |
|-----|-----|-----|------------|------------|
| 0 | 0 | 0 | 0 | No Fault |
| 0 | 0 | 1 | 1 | Handshake Timeout |
| 0 | 1 | 0 | 2 | Drive Fault |
| 0 | 1 | 1 | 3 | Position Limit |
| 1 | 0 | 0 | 4 | Homing Required |
| 1 | 0 | 1 | 5 | Piston Exit Guard |
| 1 | 1 | 0 | 6 | Limit Switch Fault |
| 1 | 1 | 1 | 7 | Encoder Fault |

#### DO3: Performance Status

Mode-dependent status indicator:

| Mode | DO3 Meaning |
|------|-------------|
| Position Control | At target position (within tolerance) |
| Velocity Control | At target velocity |
| Torque Control | At target torque |
| Homing | Homing phase indicator |
| Other | Reserved |

#### DO4: Fault Active

| State | Meaning |
|-------|---------|
| LOW | Normal operation |
| HIGH | Fault condition active, DO0-DO2 = fault code |

**Master must monitor continuously** and initiate fault handling when HIGH.

#### DO5: Homing Complete

| State | Meaning |
|-------|---------|
| HIGH | Slave is in ST_HOME_COMPLETE (Mode 110 homing just finished) |
| LOW | Not currently in ST_HOME_COMPLETE |

**Not a persistent "homed" flag**: this signal drops LOW again as soon as
`G_diMotionEnable` is released (the slave leaves ST_HOME_COMPLETE for
ST_HOLD_POSITION). The master must latch the HIGH pulse if it needs to
remember that homing succeeded.

#### DO6: In Motion

| State | Meaning |
|-------|---------|
| HIGH | Axis is moving (velocity > threshold) |
| LOW | Axis stopped or at position |

**Use for inter-mode transitions**: Wait for LOW before commanding new mode.

#### DO7: Brake Disengage

`G_doBrakeDisengage` drives the brake release circuit directly inside the
slave. It is **not wired to the master** - the master has no brake status
input.

---

## 3. Analog Input Signal (Slave Perspective)

### AI0: Reference Command

**From**: NI DAQ analog output
**To**: MP2600iec analog input

| Parameter | Value |
|-----------|-------|
| Range | -10V to +10V |
| Resolution | 16-bit |
| Update Rate | 1 kHz recommended |

### Scaling by Mode

#### Position Control (Mode 010)

**Two-stage piecewise linear mapping** — identical to the position feedback mapping in §4, so commanding `V` yields the same physical position as the feedback reports at `V`. Stage 1 covers 0–200 mm (higher resolution, in-cylinder region); Stage 2 covers 200–305 mm.

| Voltage | Position |
|---------|----------|
| -10.00V | 0 mm |
| 0.00V | 133.33 mm |
| +5.00V | 200 mm (stage boundary) |
| +7.50V | 252.5 mm |
| +10.00V | 305 mm |

**Formula (voltage to position)** — applied by `FB_AnalogProcessor` in the slave:
```
IF voltage < +5V THEN
    position = 0 + (voltage + 10) / 15 * 200     // Stage 1
ELSE
    position = 200 + (voltage - 5) / 5 * 105     // Stage 2
END_IF
```

**Formula (position to voltage)** — the master applies this to command a given position:
```
IF position <= 200 THEN
    voltage = (position / 200) * 15 - 10         // Stage 1
ELSE
    voltage = 5 + ((position - 200) / 105) * 5   // Stage 2
END_IF
```

#### Velocity Control (Mode 011)

**Linear mapping**:
```
velocity_mm_s = voltage * 0.7
```

| Voltage | Velocity |
|---------|----------|
| -10V | -7 mm/s (retract) |
| 0V | 0 mm/s (stopped) |
| +10V | +7 mm/s (extend) |

The slave clamps the command to ±`G_cfgVelLimitMax` (5.0 mm/s).

**Formula (voltage to velocity)**:
```
velocity = voltage * 0.7
```

**Formula (velocity to voltage)**:
```
voltage = velocity / 0.7
```

#### Torque Control (Mode 100)

**Linear mapping**:
```
torque_percent = voltage * 10
```

| Voltage | Torque |
|---------|--------|
| -10V | -100% (full retract) |
| 0V | 0% (no torque) |
| +10V | +100% (full extend) |

**Formula (voltage to torque)**:
```
torque_percent = voltage * 10
```

**Formula (torque to voltage)**:
```
voltage = torque_percent / 10
```

---

## 4. Analog Output Signal (Slave Perspective)

### AO0: Position Feedback

**From**: MP2600iec analog output
**To**: NI DAQ analog input

| Parameter | Value |
|-----------|-------|
| Range | -10V to +10V |
| Resolution | 16-bit |
| Update Rate | 1 kHz |

### Two-Stage Position Mapping

The position feedback uses a two-stage piecewise linear mapping for enhanced resolution in the primary operating region.

```
            Voltage
              +10V |...................*
                   |                  /
               +5V |................./
                   |               /
                   |             /  Stage 2
                   |           /    (200-305mm)
                   |         /
                   |       *
                   |      /
                   |    /   Stage 1
                   |  /     (0-200mm)
                   |/
              -10V *
                   +---------------------> Position
                   0      100    200    305 mm
```

#### Stage 1: 0 to 200 mm

| Position | Voltage |
|----------|---------|
| 0 mm | -10V |
| 100 mm | -2.5V |
| 200 mm | +5V |

**Formula (position to voltage)**:
```
voltage = (position / 200) * 15 - 10
```

**Formula (voltage to position)**:
```
position = (voltage + 10) / 15 * 200
```

**Sensitivity**: 75 mV/mm

#### Stage 2: 200 to 305 mm

| Position | Voltage |
|----------|---------|
| 200 mm | +5V |
| 252.5 mm | +7.5V |
| 305 mm | +10V |

**Formula (position to voltage)**:
```
voltage = ((position - 200) / 105) * 5 + 5
```

**Formula (voltage to position)**:
```
position = (voltage - 5) / 5 * 105 + 200
```

**Sensitivity**: 47.6 mV/mm

### Master Inverse Mapping (Combined)

```c
float voltage_to_position(float voltage) {
    if (voltage <= 5.0) {
        // Stage 1: -10V to +5V maps to 0-200mm
        return (voltage + 10.0) / 15.0 * 200.0;
    } else {
        // Stage 2: +5V to +10V maps to 200-305mm
        return (voltage - 5.0) / 5.0 * 105.0 + 200.0;
    }
}
```

---

## 5. Signal Timing Characteristics

### Digital Signal Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| Input Debounce (mode bits) | 50 ms | `G_cfgDebounceTimeMode` |
| Input Debounce (MotionEnable) | 20 ms | `G_cfgDebounceTimeMotion` |
| Input Debounce (FaultReset) | 20 ms | `G_cfgDebounceTimeFault` |
| Input Debounce (OvertravelNeg) | 5 ms | `G_cfgDebounceTimeOvertravelNeg` |
| Input Debounce (OvertravelPos) | 2 ms | `G_cfgDebounceTimeOvertravelPos` |
| Output Update | 2 ms | Scan cycle time |
| Handshake Timeout | 500 ms | Mode confirmation |
| Fault Code Stable | 10 ms | Before reading after `G_doFaultActive` rises |

### Analog Signal Timing

| Parameter | Value | Notes |
|-----------|-------|-------|
| AI Filter Time Constant | 50 ms | `G_cfgAnalogFilterTimeConst` (IIR low-pass after a 3-sample median) |
| AO Update Rate | 1 kHz | Position feedback |
| AI Sample Rate | 1 kHz minimum | For smooth control |

---

## 6. Electrical Specifications

### Digital I/O

| Parameter | Value |
|-----------|-------|
| Voltage Level | 24V DC |
| Logic HIGH | > 15V |
| Logic LOW | < 5V |
| Current Sink/Source | 100 mA max |

### Analog I/O

| Parameter | Value |
|-----------|-------|
| Voltage Range | -10V to +10V |
| Input Impedance | > 10 kOhm |
| Output Impedance | < 100 Ohm |
| Resolution | 16-bit (0.3 mV) |
| Accuracy | +/- 0.1% FS |

### Limit Switches (Tolomatic #8100-9092)

| Parameter | Value |
|-----------|-------|
| Type | PNP, Normally Closed |
| Voltage | 24V DC |
| Sensing | Solid State |
| Wire Break | Results in LOW (triggered) |

---

## 7. NI PCI 6251 DAQ Pin Mapping

### Configuration

| MP2600iec | Signal | Direction | NI 6251 |
|-----------|--------|-----------|---------|
| DI0 | G_diModeBit0 | NI→MP | P0.5 (pin 51) |
| DI1 | G_diModeBit1 | NI→MP | P0.1 (pin 17) |
| DI2 | G_diModeBit2 | NI→MP | P0.6 (pin 16) |
| DI3 | G_diMotionEnable | NI→MP | P0.2 (pin 49) |
| DI4 | G_diFaultReset | NI→MP | P0.7 (pin 48) |
| DO0 | G_doModeConfBit0 | MP→NI | PFI14/P2.6 (pin 1) |
| DO1 | G_doModeConfBit1 | MP→NI | PFI12/P2.4 (pin 2) |
| DO2 | G_doModeConfBit2 | MP→NI | PFI9/P2.1 (pin 3) |
| DO3 | G_doPerformanceStatus | MP→NI | PFI8/P2.0 (pin 37) |
| DO4 | G_doFaultActive | MP→NI | PFI7/P1.7 (pin 38) |
| DO5 | G_doHomingComplete | MP→NI | PFI6/P1.6 (pin 5) |
| DO6 | G_doInMotion | MP→NI | PFI15/P2.7 (pin 39) |
| AI0 | G_aiReference (`AT %IW0 : LREAL`) | NI→MP | AO 0 (pin 22) |
| AO0 | G_aoPositionOutput (`AT %QW0 : LREAL`) | MP→NI | AI 0 (pin 68) |
| - | Pressure transducer | local to master | AI 2 (pin 65) |

**Note**: DO0-DO6 are sunk by the slave's outputs, so the DAQ reads logic 0
when the slave signal is asserted - the master must invert these lines in
software. `G_doBrakeDisengage` (slave DO7) is not wired to the master.
Additional master DAQ digital lines not part of the slave protocol: P0.0
(pin 52, safety control relay enable - brake disengage / safe-torque-off),
P0.4 (pin 19, stir bar motor), P0.3 (pin 47, unused).

---

## 8. Summary Tables

### Input Summary (Master Outputs)

| Signal | Pin | Purpose | Update Rate |
|--------|-----|---------|-------------|
| Mode Bits | DI0-2 | Mode command | As needed |
| Motion Enable | DI3 | Enable control | State changes |
| Fault Reset | DI4 | Clear fault | Level |
| Reference | AI0 | Motion command | 1 kHz |

### Output Summary (Master Inputs)

| Signal | Pin | Purpose | Sample Rate |
|--------|-----|---------|-------------|
| Confirm/Fault | DO0-2 | Status | 1 kHz |
| Performance | DO3 | Mode-dependent | 100 Hz |
| Fault Active | DO4 | Critical monitor | 1 kHz |
| Homing Complete | DO5 | Homing pulse | 10 Hz |
| In Motion | DO6 | Motion status | 100 Hz |
| Position | AO0 | Feedback | 1 kHz |

Note: `G_doBrakeDisengage` (slave DO7) is not wired to the master.
