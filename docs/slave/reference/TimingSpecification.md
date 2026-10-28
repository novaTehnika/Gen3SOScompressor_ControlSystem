# Timing Specification

## Gen3 SOS Compressor Servo Controller
### Complete Timing Parameters Reference

---

## 1. System Timing Overview

| Category | Parameter | Value | Notes |
|----------|-----------|-------|-------|
| PLC Scan | Cycle Time | 2 ms | Cyclic task interval (MP2600iec minimum); `G_cfgScanTime` must match |
| Motion | Position Loop | 2 ms | Synchronized with scan |
| I/O | Digital Update | 2 ms | Per scan cycle |
| I/O | Analog Update | 2 ms | Per scan cycle |

---

## 2. Handshake Timing

### Mode Entry Handshake

```
Signal Timeline:
                    t0        t1           t2          t3
MotionEnable (HR3)    _____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
mode_code (HR2)     XXXX|---- stable mode value ----------
conf_code (IR3)        0|_____|‾‾‾‾‾‾‾‾‾ (matches mode)

                    |<--->|<---->|
                     20ms  <500ms
                    stable  timeout
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| Mode Stable Time | 20 ms | Mode code must be stable before handshake |
| Handshake Timeout | 500 ms | Max time for slave to confirm mode |
| Confirm Stable Time | 10 ms | Confirmation must be stable before accepted |

### Fault Reset Handshake

| Parameter | Value | Description |
|-----------|-------|-------------|
| Mirror Stable Time | 20 ms | Fault code must be mirrored stably |
| Reset Pulse Width | 50 ms min | Minimum FaultReset (HR4) HIGH duration |
| Reset Timeout | 1000 ms | Max time for fault to clear |
| Clear Detection | 10 ms | Time after `G_mbFaultActive` (IR5) reads 0 before release |

---

## 3. Drive and Brake Timing

### Drive Enable/Disable

| Parameter | Value | Description |
|-----------|-------|-------------|
| Drive Enable Time | 100 ms | Time from MC_Power.Enable to MC_Power.Status |
| Drive Ready Timeout | 5000 ms | Max wait for drive ready |
| Drive Disable Time | 50 ms | Time for drive to fully disable |

### Brake Timing

```
Brake Disengage Sequence:
Drive Ready ________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
Brake Cmd   ____________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
Brake Actual ______________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
Motion OK   __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                         |<-->|
                         100ms
                         disengage delay

Brake Engage Sequence:
Motion Stop ‾‾‾‾‾‾‾‾‾\__________________________
Brake Cmd   ‾‾‾‾‾‾‾‾‾‾‾‾‾\______________________
Brake Actual ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_________________
Hold Secure ______________________|‾‾‾‾‾‾‾‾‾‾‾‾
                               |<-->|
                               200ms
                               engage delay
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| Brake Disengage Delay | 100 ms | Wait after command before motion allowed |
| Brake Engage Delay | 200 ms | Wait after command before drive disable |
| Brake Settle Time | 50 ms | Additional settling after mechanical action |

---

## 4. Digital Input Timing

### Input Debounce (from `GVL_Config`)

| Signal | Debounce Time | Config Parameter |
|--------|---------------|------------------|
| G_mbModeCode (HR2) | 50 ms | `G_cfgDebounceTimeMode` (code value held this long) |
| G_mbMotionEnable (HR3) | 20 ms | `G_cfgDebounceTimeMotion` |
| G_mbFaultReset (HR4) | 20 ms | `G_cfgDebounceTimeFault` |
| G_diLimitRetract | 5 ms | `G_cfgDebounceTimeLimitStd` |
| G_diLimitHome | 2 ms | `G_cfgDebounceTimeLimitHome` (shorter for homing precision) |

### Edge Detection

| Parameter | Value | Description |
|-----------|-------|-------------|
| Rising Edge Detection | 1 scan | Detected on stable HIGH after debounce |
| Falling Edge Detection | 1 scan | Detected on stable LOW after debounce |

---

## 5. Analog Signal Timing

### Analog Input (Pressure Transducer)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Sample Rate | 500 Hz | Per scan cycle |
| Filter Time Constant | 20 ms | First-order IIR low-pass (`G_cfgAnalogFilterTimeConst`) |
| Median Filter Size | 5 samples | `G_cfgAnalogMedianSize` (valid: 3 or 5) |
| Step Response (10%) | 2.1 ms | Time to 10% of final value (τ = 20 ms) |
| Step Response (90%) | 46 ms | Time to 90% of final value (τ = 20 ms) |
| Settling Time (2%) | 80 ms | Time to within 2% of final (τ = 20 ms) |

---

## 6. Motion Timing

### Position Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| Following Error Update | 2 ms | Calculated each scan |
| Position Tolerance | 0.1 mm | "At position" threshold |
| Position Stable Time | 20 ms | Time in tolerance before "done" |

### Position Mode Move Timing

`FB_PositionCommandGate` shapes each position setpoint as a trapezoidal (or, for small steps,
triangular) velocity move at the configured velocity limit `V` (`G_cfgVelLimitNormal`) and
acceleration limit `A` (`G_cfgPosGateAccelMax`). For a setpoint step of size `d`, move time is:

| Profile | Condition | Move Time |
|---------|-----------|-----------|
| Trapezoidal | `d > V^2/A` | `d/V + V/A` |
| Triangular | `d <= V^2/A` | `2*SQRT(d/A)` |

**Worked example at defaults** (`V` = 5 mm/s, `A` = 5 mm/s², so `V^2/A` = 5 mm):

| Step Size | Profile | Move Time |
|-----------|---------|-----------|
| 50 mm | Trapezoidal | ≈ 11.0 s |
| 1 mm | Triangular | ≈ 0.89 s |

`A` (`G_cfgPosGateAccelMax`) is a commissioning parameter, started conservatively at 5 mm/s²
and expected to be raised once bench testing confirms available torque headroom (see
`docs/development/ConfigurationGuide.md`). The move times above therefore represent
**upper-bound** times at the conservative starting value — actual moves will be faster once
`A` is tuned up.

### Position Mode Exit

When MotionEnable (HR3) drops in position mode, the command is ramped to rest under
`G_cfgPosGateAccelMax` before the axis is handed to `MC_Stop`, so `G_doInMotion` stays
TRUE for up to `V/A` after the drop (1.0 s at `V` = 5 mm/s, `A` = 5 mm/s²) plus the
`MC_Stop` settling time.

### Velocity Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| Velocity Update | 2 ms | Reference applied each scan |
| Acceleration Rate | Axis default | From axis configuration |
| Velocity Threshold | 0.5 mm/s | "Stopped" detection threshold |

### Torque Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| Torque Update | 2 ms | Reference applied each scan |
| Torque Ramp | Axis default | From axis configuration |

---

## 7. Homing Timing

### Mode 110 (Home to Limit)

| Phase | Typical | Maximum | Description |
|-------|---------|---------|-------------|
| Approach | 5-15 s | 30 s | Moving to limit switch |
| Detect | 100 ms | 500 ms | Confirming switch trigger |
| Backoff | 1-2 s | 5 s | Moving off switch |
| Set Reference | 50 ms | 200 ms | `MC_SetPosition` via `G_cmdSetPosition` |
| **Total** | **7-18 s** | **36 s** | Complete sequence |

### Mode 111 (Home to EOT)

| Phase | Typical | Maximum | Description |
|-------|---------|---------|-------------|
| Fast Approach | 3-5 s | 10 s | High-speed move |
| Slow Approach | 1-3 s | 10 s | Torque-limited approach |
| Stall Detection | 200-500 ms | 2 s | Confirming stall |
| Set Reference | 50 ms | 200 ms | Compute `G_posEOTOffset` (no encoder re-zero; preserves Mode 110 reference) |
| **Total** | **5-9 s** | **22 s** | Complete sequence |

### Stall Detection Timing

| Parameter | Value | Description |
|-----------|-------|-------------|
| Stall Velocity Threshold | 0.5 mm/s | Below this = stalled |
| Stall Torque Threshold | 90% | Of torque limit |
| Stall Confirmation Time | 200 ms | Both conditions must persist |
| Stall Timeout | 10 s | Max time in slow approach |

---

## 8. Fault Timing

### Fault Detection

| Fault Type | Detection Time | Description |
|------------|----------------|-------------|
| Drive Fault | 1 scan (2 ms) | Direct from drive status (includes encoder alarms A.810/A.CC0/A.830) |
| Limit Switch | 1 scan + 5 ms debounce | Unexpected activation |
| Position Limit | 1 scan | Software limit exceeded |
| Piston Exit | 1 scan | Guard condition met |
| Handshake Timeout | 500 ms | No confirmation received |

### Fault Response

| Parameter | Value | Description |
|-----------|-------|-------------|
| Fault Latch Time | 1 scan | Fault latched in FB_SafetyMonitor |
| Motion Stop Initiate | 1 scan | MC_Stop issued |
| Fault Code Output | 1 scan | `G_mbFaultCode` (IR4) updated |
| G_doFaultActive Assert | 1 scan | DO4 and `G_mbFaultActive` (IR5) go HIGH |

---

## 9. Communication Timing

### Master-Slave Synchronization

```
Master step (25 ms)  |        |        |        |        |
Exchange (50 ms)     | FC16+FC04       | FC16+FC04       |
                     v                 v
Modbus service tick  |..|..|..|..|..|..|..|..|..|..|..|..|  (4 ms)
                     v
MP2600iec scan       ||||||||||||||||||||||||||||||||||||||  (2 ms)
```

The master writes holding registers HR1-HR6 and reads input registers IR1-IR20
once per exchange. The slave's Modbus server services the connection on its own
tick, independent of the PLC task; `PRG_Main` sees new holding register values
at its next scan. See `docs/master/ModbusRegisterMap.md`.

| Parameter | Value | Description |
|-----------|-------|-------------|
| Master Step | 25 ms | Simulink Desktop Real-Time step |
| Exchange Period | 50 ms | One FC16 write + one FC04 read |
| Modbus Service Tick | 4 ms | One request answered per tick |
| Transaction Time | 1-8 ms | Request to reply |
| Slave Scan Rate | 2 ms | PLC task period |
| Link Watchdog | 300 ms | Each side faults when the other's counter stops (slave only while motion is commanded) |

---

## 10. Timeout Summary

| Function | Timeout | Consequence |
|----------|---------|-------------|
| Mode Handshake | 500 ms | FAULT_HANDSHAKE |
| Fault Reset | 1000 ms | Fault persists |
| Drive Ready | 5000 ms | Stuck in DRIVE_ENABLE |
| Homing | 30000 ms | Homing fault |
| Motion Complete | Varies | Mode-dependent |

---

## 11. Timing Diagram Examples

### Successful Mode Entry

```
Time (ms)    0    50   100  150  200  250  300  350  400  450  500
             |    |    |    |    |    |    |    |    |    |    |
Mode Code    0    |‾‾‾‾‾‾‾‾‾‾‾‾ 2 ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  ^stable
MotionEnable ____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  ^
Conf Code    0    |_____________|‾‾‾‾‾‾ 2 ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                               ^confirmed (~120ms for enable+brake)

State        IDLE |ENABLE|RELEASE| POSITION_CTRL
             [---][---100ms---][---100ms---][---------------
```

### Motion Enable Drop (Mode Change)

```
Time (ms)    0    50   100  150  200  250  300  350  400
             |    |    |    |    |    |    |    |    |
MotionEnable ‾‾‾‾‾\___________________________________/‾‾‾‾
                   ^drop                              ^rise
InMotion     ‾‾‾‾‾‾‾‾‾‾‾\_______________________________|‾‾‾‾
                         ^halted (~50-100ms decel)
Mode Code    2    |‾‾‾‾‾‾|______ 3 __________|‾‾‾‾‾ 3
                         ^new mode set

State        POS_CTRL|HOLD_POSITION  |NEW_HANDSHAKE|VEL_CTRL
```

### Fault Detection and Reset

```
Time (ms)    0   100  200  300  400  500  600  700  800  900 1000
             |    |    |    |    |    |    |    |    |    |    |
FaultActive  ____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_____|___
                 ^fault
FaultCode    0   |‾‾‾ 3 ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|0
                 ^code output
MotionEnable ‾‾‾‾\_________________________________________|‾‾‾‾
                  ^drop
Mode Code    2    |‾‾‾‾ 3 ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|0
                        ^mirror fault code
FaultReset   _______________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\__|____
                            ^assert              ^release
```

---

## 12. Configuration Parameters

### Timing-Related Configuration

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| G_cfgHandshakeTimeout | 500 ms | 100-2000 ms | Mode confirmation timeout |
| G_cfgFaultIdleTimeout | 30000 ms | 10000-60000 ms | Fault-to-idle transition timeout |
| G_cfgBrakeEngageDelay | 200 ms | 100-500 ms | Brake engage wait |
| G_cfgBrakeDisengageDelay | 100 ms | 50-200 ms | Brake release wait |
| G_cfgDriveReadyTimeout | 5000 ms | 1000-10000 ms | Drive enable timeout |
| G_cfgDebounceTimeMode | 50 ms | 10-200 ms | Mode code debounce |
| G_cfgDebounceTimeMotion | 20 ms | 5-100 ms | MotionEnable debounce |
| G_cfgDebounceTimeFault | 20 ms | 5-100 ms | FaultReset debounce |
| G_cfgDebounceTimeLimitStd | 5 ms | 1-20 ms | Retract limit debounce |
| G_cfgDebounceTimeLimitHome | 2 ms | 1-10 ms | Home limit debounce |
| G_cfgAnalogFilterTimeConst | 20 ms | 1-200 ms | Pressure input low-pass time constant |
| G_cfgHomingTimeout | 30000 ms | 10000-60000 ms | Homing sequence timeout |
| G_cfgStallDetectTime | 200 ms | 100-500 ms | EOT stall confirmation |

---

## 13. Quick Reference

### Critical Timeouts

| Function | Timeout | Action on Expiry |
|----------|---------|------------------|
| Handshake | 500 ms | Fault |
| Drive Ready | 5 s | Stuck/Fault |
| Homing | 30 s | Fault |
| Fault Reset | 1 s | Retry needed |

### Minimum Wait Times

| After | Wait | Before |
|-------|------|--------|
| Mode code set | 20 ms | MotionEnable (HR3) HIGH |
| Brake disengage | 100 ms | Motion command |
| Motion stop | 50 ms | Brake engage |
| Brake engage | 200 ms | Drive disable |
| Fault code read | 10 ms | Mirror and reset |
