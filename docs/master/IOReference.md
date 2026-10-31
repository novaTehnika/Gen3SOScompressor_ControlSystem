# I/O Reference

## Gen3 SOS Compressor Servo Controller
### Hardwired Signal Definitions

---

> **Transport.** The protocol signals (mode code, MotionEnable, FaultReset,
> reference, confirmation, fault code, status bits and feedback) travel over
> Modbus TCP; see [ModbusRegisterMap.md](ModbusRegisterMap.md) and
> [MasterProtocolGuide.md](MasterProtocolGuide.md). This document covers the
> hardwired lines only: the slave's overtravel switches, brake, status outputs
> and pressure transducer, and the master's DAQ lines for the safety relay
> and the stir bar.

## 1. Digital Input Signals (Slave Perspective)

| Pin | Signal Name | Type | Description |
|-----|-------------|------|-------------|
| DI0-DI5 | - | BOOL | Not used |
| DI6 | `G_diOvertravelNeg` | BOOL | Negative overtravel / home reference switch (PNP NC) |
| DI7 | `G_diOvertravelPos` | BOOL | Positive overtravel switch (PNP NC) |

### Signal Details

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

| Pin | Signal Name | Description |
|-----|-------------|-------------|
| DO0-DO2 | - | Not driven |
| DO3 | `G_doPerformanceStatus` | Mode-dependent status, also in IR7 |
| DO4 | `G_doFaultActive` | HIGH while a fault is active (ST_FAULT, ST_FAULT_IDLE), also in IR5 |
| DO5 | `G_doHomingComplete` | HIGH in ST_HOME_COMPLETE, also in IR8 |
| DO6 | `G_doInMotion` | Axis in motion, also in IR6 |
| DO7 | `G_doBrakeDisengage` | Brake release command |

DO3-DO6 are not wired to the master; the master reads the same values from
the input registers. Their meanings are given with the registers in
[MasterProtocolGuide.md](MasterProtocolGuide.md#slave-to-master-input-registers-one-fc04-read).

#### DO7: Brake Disengage

`G_doBrakeDisengage` drives the brake release circuit directly inside the
slave. It is **not wired to the master** and not carried by the link - the
master has no brake status input.

---

## 3. Analog Input Signal (Slave Perspective)

### AI: Pressure Transducer

| Parameter | Value |
|-----------|-------|
| Pin | `MO1_AI_01` → `G_aiPressure` (`AT %IW0 : LREAL`, volts) |
| Source | Voltage across the shunt (`G_cfgPressureShuntOhms`, 434 ohm) in the transducer's 4-20 mA loop |
| Input Range | -10 V to +10 V |
| Filtering | `FB_AnalogInputFilter`: median (`G_cfgAnalogMedianSize`) then IIR low-pass (`G_cfgAnalogFilterTimeConst`) |
| Scaling | `FB_PressureInput`: loop current from the shunt, then `G_cfgPressureZeroMa`/`G_cfgPressureSpanMa` over `G_cfgPressureRangeMin`..`G_cfgPressureRangeMax` (MPa gauge) |
| Supervision | Loop current outside `G_cfgPressureSensorMinMa`..`G_cfgPressureSensorMaxMa` for `G_cfgPressureSensorFaultTime` raises `FAULT_PRESSURE_SENSOR` in pressure mode |

The scaled pressure is `G_sysActualPressure`, published in IR19-20.

---

## 4. Signal Timing Characteristics

### Debounce

| Signal | Value | Notes |
|--------|-------|-------|
| Mode code (HR2) | 50 ms | `G_cfgDebounceTimeMode` |
| MotionEnable (HR3) | 20 ms | `G_cfgDebounceTimeMotion` |
| FaultReset (HR4) | 20 ms | `G_cfgDebounceTimeFault` |
| OvertravelNeg (DI6) | 10 ms | `G_cfgDebounceTimeOvertravelNeg` |
| OvertravelPos (DI7) | 10 ms | `G_cfgDebounceTimeOvertravelPos` |
| Output Update | 4 ms | Scan cycle time |

### Analog Input

| Parameter | Value | Notes |
|-----------|-------|-------|
| AI Filter Time Constant | 20 ms | `G_cfgAnalogFilterTimeConst` (IIR low-pass after the median) |
| AI Sample Rate | 4 ms | Scan cycle time |

---

## 5. Electrical Specifications

### Digital I/O

| Parameter | Value |
|-----------|-------|
| Voltage Level | 24V DC |
| Logic HIGH | > 15V |
| Logic LOW | < 5V |
| Current Sink/Source | 100 mA max |

### Analog Input

| Parameter | Value |
|-----------|-------|
| Voltage Range | -10V to +10V |
| Input Impedance | > 10 kOhm |
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

## 6. NI PCI 6221 DAQ Pin Mapping

| Line | Pin | Signal | Direction |
|------|-----|--------|-----------|
| P0.0 | 52 | Safety control relay enable (brake disengage / safe-torque-off); 0 on E-stop or link loss | DAQ out |
| P0.4 | 19 | Stir bar motor | DAQ out |

The blocks are created by `master-src/shell/addDaq.m` from
`master-src/shell/buildShell.m`. No other DAQ line is used.
