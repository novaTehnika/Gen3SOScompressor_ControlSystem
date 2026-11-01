# Timing Specification

## Gen3 SOS Compressor Servo Controller
### Complete Timing Parameters Reference

---

## 1. System Timing Overview

| Category | Parameter | Value | Notes |
|----------|-----------|-------|-------|
| PLC Scan | Cycle Time | 4 ms | Cyclic task interval (the program takes about 2.5 ms a scan, so a 2 ms task overruns); `G_cfgScanTime` must match |
| Motion | Position Loop | 4 ms | Synchronized with scan |
| I/O | Digital Update | 4 ms | Per scan cycle |
| I/O | Analog Update | 4 ms | Per scan cycle |

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
| Mode Stable Time | 50 ms | Mode code must be stable before handshake (`G_cfgDebounceTimeMode`) |
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
                         500ms
                         disengage delay

Brake Engage Sequence:
Motion Stop ‾‾‾‾‾‾‾‾‾\__________________________
Brake Cmd   ‾‾‾‾‾‾‾‾‾‾‾‾‾\______________________
Brake Actual ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_________________
Hold Secure ______________________|‾‾‾‾‾‾‾‾‾‾‾‾
                               |<-->|
                               500ms
                               engage delay
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| Brake Disengage Delay | 500 ms | Wait after command before motion allowed |
| Brake Engage Delay | 500 ms | Wait after command before drive disable |
| Brake Settle Time | 50 ms | Additional settling after mechanical action |

---

## 4. Digital Input Timing

### Input Debounce (from `GVL_Config`)

| Signal | Debounce Time | Config Parameter |
|--------|---------------|------------------|
| G_mbModeCode (HR2) | 50 ms | `G_cfgDebounceTimeMode` (code value held this long) |
| G_mbMotionEnable (HR3) | 20 ms | `G_cfgDebounceTimeCommand` |
| G_mbFaultReset (HR4) | 20 ms | `G_cfgDebounceTimeCommand` |
| G_diOvertravelNeg | 10 ms | `G_cfgDebounceTimeOvertravel` (negative overtravel / home reference) |
| G_diOvertravelPos | 10 ms | `G_cfgDebounceTimeOvertravel` |

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
| Sample Rate | 250 Hz | Per scan cycle (4 ms) |
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
| Following Error Update | 4 ms | Calculated each scan |
| Position Tolerance | 0.5 mm | "At target", judged by the master from feedback (`masterConfig.posTolerance`) |
| Position Stable Time | 100 ms | Time inside the tolerance before "at target" (`masterConfig.tArrive`) |

### Position Mode Move Timing

`FB_PositionPathShaper` shapes each position setpoint as a trapezoidal (or, for small steps,
triangular) velocity move at the position speed `V` (HR7-8, `G_sysPositionSpeed`, limited to `G_cfgVelLimitMax`) and
acceleration limit `A` (`G_cfgPosShaperAccelMax`). For a setpoint step of size `d`, move time is:

| Profile | Condition | Move Time |
|---------|-----------|-----------|
| Trapezoidal | `d > V^2/A` | `d/V + V/A` |
| Triangular | `d <= V^2/A` | `2*SQRT(d/A)` |

**Worked example at defaults** (`V` = 6 mm/s, the master's default position speed
`masterConfig.positionSpeed` and the ceiling `G_cfgVelLimitMax`; `A` = 10 mm/s², so
`V^2/A` = 3.6 mm):

| Step Size | Profile | Move Time |
|-----------|---------|-----------|
| 50 mm | Trapezoidal | ≈ 8.9 s |
| 1 mm | Triangular | ≈ 0.63 s |

A lower position speed from the master lengthens the trapezoidal moves by `d/V`. `A`
(`G_cfgPosShaperAccelMax`) is a commissioning parameter, set conservatively at 10 mm/s² and
to be raised once bench testing confirms the torque headroom (see
`docs/slave/development/ConfigurationGuide.md`); a larger `A` shortens every move.

### Position Mode Exit

When MotionEnable (HR3) drops in position mode, the command is ramped to rest under
`G_cfgPosShaperAccelMax` before the axis is handed to `MC_Stop`, so `G_sysInMotion` stays
TRUE for up to `V/A` after the drop (0.6 s at `V` = 6 mm/s, `A` = 10 mm/s²) plus the
`MC_Stop` settling time.

### Velocity Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| Velocity Update | 4 ms | Reference applied each scan |
| Acceleration Rate | 10 mm/s² | `G_cfgDirectCtrlAccel`, written to `Y_DirectControl`'s `Acceleration` (no deceleration input) |
| Velocity Threshold | 0.1 mm/s | "In motion" threshold (`G_cfgInMotionVelThreshold`) |

### Torque Mode

| Parameter | Value | Description |
|-----------|-------|-------------|
| Torque Update | 4 ms | Reference applied each scan |
| Torque Ramp | Axis default | From axis configuration |

---

## 7. Homing Timing

### Mode 110 (Home to Limit)

Durations follow from the `FB_HomeLimit` phases and the GVL defaults
(fast 6 mm/s, slow 1 mm/s, backoff/retract 5 mm/s, accel 5 mm/s², decel
50 mm/s²). The velocities are limited to `G_cfgVelLimitMax` (6 mm/s). The
switch's hysteresis is not measured; the backoff and slow approach are
timed for a couple of millimetres of it.

| Phase | Duration | Description |
|-------|----------|-------------|
| Fast Approach | distance / 6 mm/s, up to 62 s (63 s with the ramps) from the positive soft limit: 372 mm, from `G_cfgPosSoftLimitMax` (367 mm) to the switch at -`G_cfgHomeLimRetractDist` (-5 mm) | Jog reverse at `G_cfgHomeLimFastApproachVel` until the negative overtravel switch trips, then wait for the Jog to stop (0.12 s, 0.36 mm past the trip) |
| Inter-Backoff | 1-2 s | Jog forward at `G_cfgHomeLimBackoffVel` until the switch clears: switch hysteresis / 5 mm/s plus the acceleration |
| Slow Approach | 2-3 s | Jog reverse at `G_cfgHomeLimSlowApproachVel`: switch hysteresis / 1 mm/s plus the acceleration; the switch position is captured on trip |
| Retract | ≈ 1.6 s | `MC_MoveAbsolute` to the captured switch position plus `G_cfgHomeLimRetractDist` (5 mm), at `G_cfgHomeLimBackoffVel` |
| Set Reference | a few scans | `MC_SetPosition` via `G_cmdSetPosition` until `G_staSetPosition.Done` |
| **Total** | **≈ 70 s worst case, ≤ 90 s** | Bounded by `G_cfgHomingTimeout`; expiry raises `FAULT_LIMIT_SWITCH` |

Mode 110 is the only homing sequence. Mode code 111 (7) is `MODE_PRESSURE`, an
operational mode handled by `FB_PressureController`, not a homing mode.

---

## 8. Fault Timing

### Fault Detection

| Fault Type | Detection Time | Description |
|------------|----------------|-------------|
| Drive Fault | 1 scan (4 ms) | Direct from drive status (includes encoder alarms A.810/A.CC0/A.830) |
| Overtravel Switch | 1 scan + 10 ms debounce | Unexpected activation |
| Position Limit | 1 scan | Software limit exceeded |
| Piston Exit | 1 scan | Guard condition met |
| Handshake Timeout | 500 ms | No confirmation received |

### Fault Response

| Parameter | Value | Description |
|-----------|-------|-------------|
| Fault Latch Time | 1 scan | Fault latched in FB_SafetyMonitor |
| Motion Stop Initiate | 1 scan | MC_Stop issued |
| Fault Code Output | 1 scan | `G_mbFaultCode` (IR4) updated |
| G_sysFaultActive Assert | 1 scan | `G_mbFaultActive` (IR5) goes HIGH |

---

## 9. Communication Timing

### Master-Slave Synchronization

```
Master step (25 ms)  |        |        |        |        |
Exchange (50 ms)     | FC16+FC04       | FC16+FC04       |
                     v                 v
Modbus service tick  |..|..|..|..|..|..|..|..|..|..|..|..|  (4 ms)
                     v
MP2600iec scan       ||||||||||||||||||||||||||||||||||||||  (4 ms)
```

The master writes holding registers HR1-HR12 and reads input registers IR1-IR20
once per exchange. The slave's Modbus server services the connection on its own
tick, independent of the PLC task; `PRG_Main` sees new holding register values
at its next scan. See `docs/master/ModbusRegisterMap.md`.

| Parameter | Value | Description |
|-----------|-------|-------------|
| Master Step | 25 ms | Simulink Desktop Real-Time step |
| Exchange Period | 50 ms | One FC16 write + one FC04 read |
| Modbus Service Tick | 4 ms | One request answered per tick |
| Transaction Time | 1-8 ms | Request to reply |
| Slave Scan Rate | 4 ms | PLC task period |
| Link Watchdog | 300 ms | Each side faults when the other's counter stops (slave only while motion is commanded) |

---

## 10. Timeout Summary

| Function | Timeout | Consequence |
|----------|---------|-------------|
| Mode Handshake | 500 ms | FAULT_HANDSHAKE |
| Fault Reset | 1000 ms | Fault persists |
| Drive Ready | 5000 ms | Stuck in DRIVE_ENABLE |
| Homing | 90000 ms | FAULT_LIMIT_SWITCH |
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
| G_cfgBrakeEngageDelay | 500 ms | 100-1000 ms | Brake engage wait |
| G_cfgBrakeDisengageDelay | 500 ms | 100-1000 ms | Brake release wait |
| G_cfgDriveReadyTimeout | 5000 ms | 1000-10000 ms | Drive enable timeout |
| G_cfgDebounceTimeMode | 50 ms | 10-200 ms | Mode code debounce |
| G_cfgDebounceTimeCommand | 20 ms | 5-100 ms | MotionEnable and FaultReset debounce |
| G_cfgDebounceTimeOvertravel | 10 ms | 1-20 ms | Debounce of both overtravel inputs (the negative one is the home reference) |
| G_cfgAnalogFilterTimeConst | 20 ms | 1-200 ms | Pressure input low-pass time constant |
| G_cfgHomingTimeout | 90000 ms | 70000-140000 ms | Homing sequence timeout; derivation below |

**`G_cfgHomingTimeout` range.** The lower bound is the longest legitimate homing: the fast approach over the longest travel, 372 mm (soft limit at 367 mm to the switch at -5 mm) at `G_cfgHomeLimFastApproachVel` (6 mm/s), 63 s with the ramps (§7), plus up to 7 s for the backoff, slow approach, retract and set reference (§7): 70 s. A shorter timeout would fault a homing that started at the far end. Scale it with the speed: `372 mm / G_cfgHomeLimFastApproachVel + 7 s`. The upper bound, twice the lower, is a judgement: a switch that never trips runs the axis at the fast approach speed until the timeout, and a longer timeout lets it run further before the fault. The default, 90 s, is about 30 % above the longest homing.

---

## 13. Quick Reference

### Critical Timeouts

| Function | Timeout | Action on Expiry |
|----------|---------|------------------|
| Handshake | 500 ms | Fault |
| Drive Ready | 5 s | Stuck/Fault |
| Homing | 90 s | Fault |
| Fault Reset | 1 s | Retry needed |

### Minimum Wait Times

| After | Wait | Before |
|-------|------|--------|
| Mode code set | 20 ms | MotionEnable (HR3) HIGH |
| Brake disengage | 500 ms | Motion command |
| Motion stop | 50 ms | Brake engage |
| Brake engage | 500 ms | Drive disable |
| Fault code read | 10 ms | Mirror and reset |
