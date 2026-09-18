# Analog Scaling Reference

## Gen3 SOS Compressor Servo Controller
### Complete Analog I/O Mathematics

---

## 1. Overview

| Signal | Address | Type | Direction | Range | Purpose |
|--------|---------|------|-----------|-------|---------|
| AI0 (`G_aiReference`) | `%IW0` | `LREAL` | Master → Slave | -10V to +10V | Mode-dependent command |
| AO0 (`G_aoPositionOutput`) | `%QW0` | `LREAL` | Slave → Master | -10V to +10V | Position feedback |

**Signal representation.** MotionWorksIEC presents both analog channels to the ST application as `LREAL` voltage values directly (e.g. `-10.0` to `+10.0`). The slave-side code does **not** convert a raw INT DAC code — all scaling math below operates on the `LREAL` voltage. The 16-bit resolution figures in §4 describe the NI DAQ and MP2600iec DAC hardware, not the ST variable type.

---

## 2. Analog Input Scaling (Master → Slave)

### Position Mode (010)

**Physical Range**: 0 to 305 mm
**Voltage Range**: -10V to +10V
**Mapping**: **Two-stage piecewise linear** (matches the feedback mapping in §3 so that commanding 0V means the same physical position whether the signal is a command or feedback).

```
              Position (mm)
          0        200         305
          |---------|-----------|
          Stage 1   |   Stage 2
          |---------|-----------|
        -10V       +5V        +10V
                Voltage
```

#### Formulas

`FB_AnalogProcessor` applies the inverse of the feedback mapping (see §3):

**Voltage to Position (Slave Receives)**:
```
IF voltage < +5V THEN                     (* Stage 1 *)
    position_mm = 0 + (voltage - (-10)) / (5 - (-10)) * (200 - 0)
    position_mm = (voltage + 10) * 13.333
ELSE                                      (* Stage 2 *)
    position_mm = 200 + (voltage - 5) / (10 - 5) * (305 - 200)
    position_mm = (voltage - 5) * 21 + 200
END_IF
```

**Position to Voltage (Master Sends)** — mirror of the slave decode:
```
IF position_mm <= 200 THEN                (* Stage 1 *)
    voltage = -10 + (position_mm / 200) * 15
ELSE                                      (* Stage 2 *)
    voltage = 5 + ((position_mm - 200) / 105) * 5
END_IF
```

#### Example Values

| Voltage | Position |
|---------|----------|
| -10.00V | 0.0 mm |
| -5.00V | 66.67 mm |
| 0.00V | 133.33 mm |
| +5.00V | 200.0 mm (stage boundary) |
| +7.50V | 252.5 mm |
| +10.00V | 305.0 mm |

#### Sensitivity

```
Stage 1 sensitivity = 200 mm / 15 V = 13.33 mm/V  (higher resolution — in-cylinder region)
Stage 2 sensitivity = 105 mm / 5 V  = 21.0 mm/V   (lower resolution — out-of-cylinder region)
```

---

### Velocity Mode (011)

**Physical Range**: -100 to +100 mm/s
**Voltage Range**: -10V to +10V
**Mapping**: Linear, zero-centered

```
            Velocity (mm/s)
         -100       0       +100
            |-------|-------|
            |       |       |
         -10V      0V     +10V
                Voltage
```

#### Formulas

**Voltage to Velocity (Slave Receives)**:
```
velocity_mm_s = voltage * 10
```

**Velocity to Voltage (Master Sends)**:
```
voltage = velocity_mm_s / 10
```

#### Example Values

| Voltage | Velocity |
|---------|----------|
| -10.00V | -100.0 mm/s (full retract) |
| -5.00V | -50.0 mm/s |
| 0.00V | 0.0 mm/s (stopped) |
| +5.00V | +50.0 mm/s |
| +10.00V | +100.0 mm/s (full extend) |

#### Sensitivity

```
Sensitivity = 200 mm/s / 20V = 10 mm/s/V
Resolution = 10 mm/s / 65536 = 0.00015 mm/s/LSB (16-bit)
```

**Direction Convention**:
- Positive velocity = extend (toward EOT)
- Negative velocity = retract (toward home)

---

### Torque Mode (100)

**Physical Range**: -100% to +100%
**Voltage Range**: -10V to +10V
**Mapping**: Linear, zero-centered

```
            Torque (%)
         -100       0       +100
            |-------|-------|
            |       |       |
         -10V      0V     +10V
                Voltage
```

#### Formulas

**Voltage to Torque (Slave Receives)**:
```
torque_percent = voltage * 10
```

**Torque to Voltage (Master Sends)**:
```
voltage = torque_percent / 10
```

#### Example Values

| Voltage | Torque |
|---------|--------|
| -10.00V | -100.0% (full retract force) |
| -5.00V | -50.0% |
| 0.00V | 0.0% (no torque) |
| +5.00V | +50.0% |
| +10.00V | +100.0% (full extend force) |

#### Sensitivity

```
Sensitivity = 200% / 20V = 10%/V
Resolution = 10% / 65536 = 0.00015%/LSB (16-bit)
```

**Direction Convention**:
- Positive torque = extend force
- Negative torque = retract force

---

## 3. Analog Output Scaling (Slave → Master)

### Position Feedback (Two-Stage Mapping)

The position feedback uses a **two-stage piecewise linear** mapping to provide higher resolution in the primary operating region (0-200mm).

```
Voltage
  +10V |                           *
       |                         /
       |                       /  Stage 2
       |                     /    (lower resolution)
       |                   /
   +5V |................./*
       |               /
       |             /
       |           /    Stage 1
       |         /      (higher resolution)
       |       /
       |     /
       |   /
  -10V |*/
       +----------------------------> Position
       0        100       200      305 mm
```

### Stage 1: 0 to 200 mm

**Position Range**: 0 to 200 mm
**Voltage Range**: -10V to +5V (15V span)
**Resolution**: 75 mV/mm

#### Formulas

**Position to Voltage (Slave Sends)**:
```
voltage = (position_mm / 200) * 15 - 10
voltage = position_mm * 0.075 - 10
```

**Voltage to Position (Master Receives)**:
```
position_mm = (voltage + 10) / 15 * 200
position_mm = (voltage + 10) * 13.333
```

#### Example Values (Stage 1)

| Position | Voltage |
|----------|---------|
| 0 mm | -10.00V |
| 50 mm | -6.25V |
| 100 mm | -2.50V |
| 150 mm | +1.25V |
| 200 mm | +5.00V |

---

### Stage 2: 200 to 305 mm

**Position Range**: 200 to 305 mm (105 mm span)
**Voltage Range**: +5V to +10V (5V span)
**Resolution**: 47.6 mV/mm

#### Formulas

**Position to Voltage (Slave Sends)**:
```
voltage = ((position_mm - 200) / 105) * 5 + 5
voltage = (position_mm - 200) * 0.0476 + 5
```

**Voltage to Position (Master Receives)**:
```
position_mm = (voltage - 5) / 5 * 105 + 200
position_mm = (voltage - 5) * 21 + 200
```

#### Example Values (Stage 2)

| Position | Voltage |
|----------|---------|
| 200 mm | +5.00V |
| 225 mm | +6.19V |
| 250 mm | +7.38V |
| 275 mm | +8.57V |
| 305 mm | +10.00V |

---

### Combined Inverse Mapping (Master Algorithm)

```c
// C implementation for master
float voltage_to_position(float voltage) {
    if (voltage <= 5.0f) {
        // Stage 1: -10V to +5V maps to 0-200mm
        return (voltage + 10.0f) / 15.0f * 200.0f;
    } else {
        // Stage 2: +5V to +10V maps to 200-305mm
        return (voltage - 5.0f) / 5.0f * 105.0f + 200.0f;
    }
}
```

```matlab
% MATLAB/Simulink implementation
function pos = voltage_to_position(voltage)
    if voltage <= 5.0
        % Stage 1
        pos = (voltage + 10) / 15 * 200;
    else
        % Stage 2
        pos = (voltage - 5) / 5 * 105 + 200;
    end
end
```

---

## 4. Resolution Comparison

### Position Feedback Resolution

| Stage | Position Range | Voltage Range | Sensitivity | 16-bit Resolution |
|-------|----------------|---------------|-------------|-------------------|
| 1 | 0-200 mm | -10V to +5V | 75 mV/mm | 0.0041 mm/LSB |
| 2 | 200-305 mm | +5V to +10V | 47.6 mV/mm | 0.0064 mm/LSB |

### Position Command Resolution

| Mode | Range | Sensitivity | 16-bit Resolution |
|------|-------|-------------|-------------------|
| Position | 0-305 mm | 65.6 mV/mm | 0.0047 mm/LSB |

### Velocity Command Resolution

| Mode | Range | Sensitivity | 16-bit Resolution |
|------|-------|-------------|-------------------|
| Velocity | ±100 mm/s | 100 mV/(mm/s) | 0.003 mm/s/LSB |

### Torque Command Resolution

| Mode | Range | Sensitivity | 16-bit Resolution |
|------|-------|-------------|-------------------|
| Torque | ±100% | 100 mV/% | 0.003%/LSB |

---

## 5. Calibration Considerations

### Analog Input Calibration (AI0)

**Offset Adjustment**:
```
actual_value = (raw_voltage - offset) * gain
```

Typical calibration procedure:
1. Apply known 0V → verify reference reads correct value
2. Apply +10V → adjust gain
3. Apply -10V → verify linearity

### Analog Output Calibration (AO0)

**Two-point calibration per stage**:

Stage 1:
1. Move to 0mm → record voltage (should be -10V)
2. Move to 200mm → record voltage (should be +5V)
3. Calculate offset and gain corrections

Stage 2:
1. Move to 200mm → verify +5V
2. Move to 305mm → record voltage (should be +10V)
3. Calculate offset and gain corrections

### Expected Tolerances

| Parameter | Tolerance | Effect |
|-----------|-----------|--------|
| Voltage accuracy | ±0.1% FS | ±20 mV |
| Position error (Stage 1) | - | ±0.27 mm |
| Position error (Stage 2) | - | ±0.42 mm |
| Velocity error | ±0.1% FS | ±0.2 mm/s |
| Torque error | ±0.1% FS | ±0.2% |

---

## 6. Filtering

### Analog Input Filter (Slave Side)

```
Filter Type: Median (size 3 or 5) feeding a first-order IIR low-pass
Time Constant: G_cfgAnalogFilterTimeConst (default: T#50MS)
Cutoff Frequency: ~3.2 Hz (at 50 ms time constant)
```

Transfer function of the low-pass stage:
```
H(s) = 1 / (τs + 1)
τ = 0.050 s  (default)
```

### Effect on Step Response

| Filter TC | 10% Rise | 90% Rise | Settling (2%) |
|-----------|----------|----------|---------------|
| 50 ms (default) | 5.3 ms | 115 ms | 200 ms |
| 20 ms | 2.1 ms | 46 ms | 80 ms |
| 10 ms | 1.1 ms | 23 ms | 40 ms |

**Recommendation**: The default 50 ms is tuned for the master's typical reference bandwidth; reduce only if command tracking lag becomes limiting.

---

## 7. Implementation Examples

### Master Position Command (Simulink)

The command uses the same two-stage mapping as the feedback, so that a given position maps to the same voltage in either direction.

```matlab
% Position reference generator (two-stage)
function voltage = position_to_voltage(position_mm)
    % Clamp to valid range
    position_mm = max(0, min(305, position_mm));

    if position_mm <= 200
        % Stage 1: 0-200mm -> -10V..+5V
        voltage = (position_mm / 200) * 15 - 10;
    else
        % Stage 2: 200-305mm -> +5V..+10V
        voltage = ((position_mm - 200) / 105) * 5 + 5;
    end
end
```

### Master Position Feedback (Simulink)

```matlab
% Position feedback decoder
function position_mm = decode_position_feedback(voltage)
    % Clamp voltage to valid range
    voltage = max(-10, min(10, voltage));

    % Two-stage inverse mapping
    if voltage <= 5.0
        % Stage 1: 0-200mm
        position_mm = (voltage + 10) / 15 * 200;
    else
        % Stage 2: 200-305mm
        position_mm = (voltage - 5) / 5 * 105 + 200;
    end
end
```

### Master Velocity Command (Simulink)

```matlab
% Velocity reference generator
function voltage = velocity_to_voltage(velocity_mm_s)
    % Clamp to valid range
    velocity_mm_s = max(-100, min(100, velocity_mm_s));

    % Linear mapping
    voltage = velocity_mm_s / 10;
end
```

---

## 8. Quick Reference Tables

### Position Mode Scaling (two-stage, both directions)

| Direction | From | To | Formula |
|-----------|------|----|---------|
| Master → Slave (Stage 1) | mm | V | `V = pos*0.075 - 10` |
| Master → Slave (Stage 2) | mm | V | `V = (pos-200)*0.0476 + 5` |
| Slave → Master (Stage 1) | mm | V | `V = pos*0.075 - 10` |
| Slave → Master (Stage 2) | mm | V | `V = (pos-200)*0.0476 + 5` |
| Decode (Stage 1) | V | mm | `pos = (V+10)*13.333` |
| Decode (Stage 2) | V | mm | `pos = (V-5)*21 + 200` |

### Velocity Mode Scaling

| Direction | From | To | Formula |
|-----------|------|----|---------|
| Master → Slave | mm/s | V | `V = vel/10` |
| Slave Decode | V | mm/s | `vel = V*10` |

### Torque Mode Scaling

| Direction | From | To | Formula |
|-----------|------|----|---------|
| Master → Slave | % | V | `V = torque/10` |
| Slave Decode | V | % | `torque = V*10` |

### Key Voltage Points

| Position | Feedback Voltage |
|----------|------------------|
| 0 mm | -10.00 V |
| 100 mm | -2.50 V |
| 200 mm | +5.00 V |
| 250 mm | +7.38 V |
| 305 mm | +10.00 V |
