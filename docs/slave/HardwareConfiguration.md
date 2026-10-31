# Hardware Configuration Guide
## Gen3 SOS Compressor Servo Controller

This document provides step-by-step instructions for configuring the hardware in MotionWorksIEC Express.

---

## 1. Hardware Components

| Component | Model | Description |
|-----------|-------|-------------|
| Controller | Yaskawa MP2600iec | Motion controller with MotionWorksIEC Express |
| Servo Drive | SGD7S2R8FE0A000300 | Sigma-7 200V, 2.8A servopack |
| Servo Motor | SGM7J-04A6A6C | 400W with brake and absolute encoder |
| Actuator | Tolomatic RSA64 | Linear electromechanical actuator |
| Overtravel Switches | Tolomatic #8100-9092 | Solid State, Normally Closed, PNP (×2: negative & positive overtravel) |
| Master Interface | Modbus TCP (built-in server, port 502) | Master ↔ slave protocol registers |

---

## 2. MotionWorksIEC Express Project Setup

### 2.1 Create New Project
1. Open MotionWorksIEC Express
2. File → New Project
3. Select MP2600iec as the target controller
4. Name the project: `Gen3SOS_ServoController`

### 2.2 Hardware Configuration
1. Open the Hardware Configuration view
2. Add the MP2600iec controller
3. Configure the Mechatrolink network for the SGD7S servo drive

### 2.3 Task Configuration
MotionWorks IEC 3 Express has a single task. Every POU (`PRG_Input`,
`PRG_Main`, `PRG_Output`, the LD POU and any bench program) and the
Modbus groups' IO Task Assignment run on that one cyclic task, so work
cannot be moved to a slower task: anything that does not need every scan
has to be skipped or spread over scans inside its own POU (the step probe,
`FB_Probe`, copies at most `G_cfgProbeFillPerScan` records per scan for
this reason). The Modbus holding group spans HR1-114 and the input group
IR1-524 for the probe's registers (`docs/master/ModbusRegisterMap.md`,
section 6).

1. Assign `PRG_Main` (and the LD POU) to the cyclic task
2. Set the task interval to **4 ms** (the program takes about 2.5 ms a scan, so a 2 ms task overruns and runs on every second motion cycle)
3. Set `G_cfgScanTime` to the same value in seconds (`0.004`); the position path shaper integrates with it

---

## 3. Axis Configuration

### 3.1 Add Sigma-7 Axis
1. In Hardware Configuration, add a new axis
2. Select axis type: **Rotary (motor) with linear conversion**
3. Assign to the SGD7S drive on Mechatrolink

### 3.2 Mechanical Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motor Type | SGM7J-04A6A6C | 400W Sigma-7 |
| Gear Ratio | 70:1 | Motor turns per lead screw turn |
| Lead Screw Pitch | 5.08 mm/rev | 0.2 inch per revolution |
| Position Units | mm | Millimeters |
| Velocity Units | mm/s | Millimeters per second |

### 3.3 Unit Conversion
The axis should be configured so that:
- 1 motor revolution = 1/70 lead screw revolutions
- 1 lead screw revolution = 5.08 mm linear travel
- Therefore: 1 motor revolution = 5.08/70 = 0.0726 mm

Configure the axis scaling in MotionWorksIEC:
- **Encoder counts per motor revolution**: (from motor specs, typically 2^20 for Sigma-7 absolute)
- **Linear distance per motor revolution**: 0.0726 mm
- Or use gear ratio and lead screw pitch directly if supported

### 3.4 Axis Limits (Configure in Drive)
| Parameter | Value | Description |
|-----------|-------|-------------|
| Position Limit High | 320 mm | Hard limit - extend |
| Position Limit Low | -10 mm | Hard limit - retract |
| Velocity Limit | 200 mm/s | Maximum velocity |
| Torque Limit | 300% | Maximum torque (% of rated) |

---

## 4. Digital I/O Configuration

### 4.1 Digital Input Addresses
| Pin | Address | Signal | External Connection |
|-----|---------|--------|---------------------|
| DI0-DI5 | %IX0.0-%IX0.5 | unused | (Not connected) |
| DI6 | %IX0.6 | G_diOvertravelNeg | Negative Overtravel / Home Reference Switch |
| DI7 | %IX0.7 | G_diOvertravelPos | Positive Overtravel Switch |

The mode code, MotionEnable and FaultReset arrive in Modbus holding registers HR2-HR4 (`G_mbModeCode`, `G_mbMotionEnable`, `G_mbFaultReset`); see `docs/master/ModbusRegisterMap.md`.

### 4.2 Digital Output Addresses
| Pin | Address | Signal | External Connection |
|-----|---------|--------|---------------------|
| DO0-DO2 | %QX0.0-%QX0.2 | unused | (Not connected) |
| DO3 | %QX0.3 | G_doPerformanceStatus | Not wired to the master (IR7) |
| DO4 | %QX0.4 | G_doFaultActive | Not wired to the master (IR5) |
| DO5 | %QX0.5 | G_doHomingComplete | Not wired to the master (IR8) |
| DO6 | %QX0.6 | G_doInMotion | Not wired to the master (IR6) |
| DO7 | %QX0.7 | G_doBrakeDisengage | Brake Relay |

The mode confirmation and fault code are published in input registers IR3 and IR4 (`G_mbConfCode`, `G_mbFaultCode`); DO3-DO6 are mirrored in IR5-IR8.

### 4.3 Overtravel Switch Wiring (PNP, Normally Closed)

Two Tolomatic #8100-9092 switches are mounted as travel limits on the actuator:

- **Negative overtravel (DI6 / `G_diOvertravelNeg`)** — positioned so that the
  piston is still fully retracted but clear of mechanical hard stops. Also used
  as the **home reference** for Mode 110 — Mode 110 drives the ram negative
  into this switch, backs off, and sets the coordinate reference.
- **Positive overtravel (DI7 / `G_diOvertravelPos`)** — positioned so that the
  piston remains safely inside the cylinder with adequate margin against the
  PEEK seal/groove feature. The homing sequence never approaches this switch,
  so any activation is a `FAULT_LIMIT_SWITCH`.

Both switches raise `FAULT_LIMIT_SWITCH` in all motion states except where the
switch is the expected homing target (negative overtravel inside
`FB_HomeLimit`).

```
+24V ──────┬──────────────────────────────┐
           │                              │
           │    ┌─────────────────┐       │
           └────┤ Limit Switch    ├───────┼──── DI6 (or DI7)
                │ (Tolomatic      │       │
                │  #8100-9092)    │       │
                └─────────────────┘       │
                                          │
0V ───────────────────────────────────────┘
```

- **Switch NOT triggered**: Output HIGH (TRUE) → DI reads TRUE
- **Switch triggered**: Output LOW (FALSE) → DI reads FALSE
- **Wire broken**: No output → DI reads FALSE (fail-safe)

---

## 5. Analog I/O Configuration

### 5.1 Analog Input
| Pin | Address | Range | Type | Signal |
|-----|---------|-------|------|--------|
| AI0 | %IW0 | -10V to +10V | LREAL | G_aiPressure: voltage across the pressure transducer's 4-20 mA loop shunt (`G_cfgPressureShuntOhms`), scaled to MPa gauge by `FB_PressureInput` |

**Scaling**: MotionWorksIEC presents the analog input as an `LREAL` in volts directly (e.g. `-10.0` to `+10.0`). The application code reads and processes the signal as volts; no raw-to-voltage conversion is required in ST. The master reference arrives in engineering units through the Modbus holding registers (see `docs/master/ModbusRegisterMap.md`).

### 5.2 Analog Output
The analog output is not used. Position feedback is published in the Modbus input registers.

---

## 6. Brake Configuration

### 6.1 Brake Specifications
- Type: Spring-engaged, electrically released
- Motor: SGM7J-04A6A6C has integrated holding brake
- Capacity: Equivalent to motor rated torque

### 6.2 Brake Control Wiring
The brake is controlled via DO7 through a relay:
- **DO7 = LOW (FALSE)**: Relay off → Brake engaged (spring holds)
- **DO7 = HIGH (TRUE)**: Relay on → Brake released (motion allowed)

### 6.3 Timing Requirements
| Transition | Delay | Description |
|------------|-------|-------------|
| Disengage → Motion | 100 ms | Wait after brake release before motion |
| Motion → Engage | 200 ms | Wait after brake engage before drive disable |

---

## 7. Drive Parameters (SGD7S)

### 7.1 Critical Parameters to Configure
| Parameter | Value | Description |
|-----------|-------|-------------|
| Pn205 | (auto) | Multi-turn limit for absolute encoder |
| Pn50A | (set) | Brake output polarity |
| Pn001 | (set) | Control mode selection |

### 7.2 Absolute Encoder Setup
The SGM7J-04A6A6C has a 24-bit absolute encoder:
- Battery backup maintains position across power cycles. The zero set by Mode 110 is an `MC_SetPosition` offset that the controller stores in battery-backed memory (flash on Sigma-7Siec) for absolute-encoder axes (Pn002.2), so the coordinate frame survives controller and drive power cycles and no homing is needed at boot
- Re-home with Mode 110 when the frame is lost: after an encoder alarm or encoder reset, or after a controller SRAM battery failure or controller replacement (which lose the stored offset)
- Encoder alarms (A.810/A.CC0/A.830) surface as `FAULT_DRIVE` from the servo amplifier

---

## 8. Commissioning Checklist

### 8.1 Pre-Power Checks
- [ ] Verify all wiring connections
- [ ] Confirm negative and positive overtravel switches operate (manual test)
- [ ] Confirm positive overtravel switch position keeps piston safely inside cylinder
- [ ] Check brake relay wiring
- [ ] Verify 24V power supply for I/O
- [ ] Confirm Mechatrolink cable connections

### 8.2 Power-Up Sequence
1. [ ] Power on controller (MP2600iec)
2. [ ] Verify controller LED status
3. [ ] Power on servo drive (SGD7S)
4. [ ] Check for drive alarms (none expected)
5. [ ] Verify encoder battery status

### 8.3 Software Configuration
1. [ ] Download project to controller
2. [ ] Verify I/O mapping (digital and analog)
3. [ ] Test digital inputs with external signals
4. [ ] Test digital outputs with multimeter
5. [ ] Verify pressure input scaling (`G_aiPressure`)

### 8.4 Motion Testing (Manual Jog)
1. [ ] Enable drive power (MC_Power)
2. [ ] Release brake (DO7 = TRUE)
3. [ ] Jog positive direction at low velocity — verify positive overtravel triggers `FAULT_LIMIT_SWITCH` well before any mechanical interference
4. [ ] Jog negative direction at low velocity — verify negative overtravel triggers `FAULT_LIMIT_SWITCH` when commanded outside homing
5. [ ] Engage brake and disable drive

### 8.5 Homing Verification
1. [ ] Execute Mode 110 (Home to Negative Overtravel Switch)
2. [ ] Verify ram drives negative, stops at negative overtravel, backs off, and `MC_SetPosition` sets position to `G_cfgHomeLimSetPosition`
3. [ ] Verify `G_doHomingComplete` goes TRUE
4. [ ] Power-cycle the controller and drive — verify the reported position is unchanged and Position mode is accepted without homing

---

## 9. Troubleshooting

### 9.1 Drive Not Ready
- Check Mechatrolink connection
- Verify drive power supply
- Check for drive alarms (front panel)

### 9.2 Encoder Alarm (A.CC0)
- Battery may be low or disconnected
- Use `Y_ResetAbsoluteEncoder` to clear
- Re-home (Mode 110) after reset

### 9.3 Overtravel Switch Not Detecting
- Check wiring (24V, signal, 0V)
- Verify switch position on actuator (negative switch = home reference; positive switch = safety only)
- Test with multimeter

### 9.4 Brake Not Releasing
- Check relay power supply
- Verify DO7 output state
- Check relay coil and contacts
