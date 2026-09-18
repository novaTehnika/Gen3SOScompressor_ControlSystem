# Configuration and Tuning Guide

## Gen3 SOS Compressor ServoController
### Configuration Parameter Reference

---

## 1. Introduction

This document provides a detailed reference for all configurable parameters documented in `slave-src/GVL/GlobalVariables_Reference.st` and defined in the MWiec GUI. These variables allow the system's behavior to be tuned and adapted without changing the core application logic.

**Warning:** Modifying these parameters can have a significant impact on machine performance and safety. Changes should be made carefully and tested thoroughly.

**Units:**
-   **Position:** millimeters (mm)
-   **Velocity:** millimeters per second (mm/s)
-   **Torque:** Percent of motor's rated torque (%)
-   **Time:** Milliseconds (ms) or Seconds (s)

---

## 2. Timing Parameters

These parameters control the duration of timeouts and delays within the state machine.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgHandshakeTimeout` | T#500MS | ms | Max time for the master-slave mode handshake. **Tuning:** Increase if the master controller is slow and causes nuisance handshake faults. Decrease for faster fault detection of an unresponsive master. |
| `G_cfgModeTransitionDelay` | T#100MS | ms | A small delay used in some state transitions to ensure stability. Generally should not be changed. |
| `G_cfgBrakeEngageDelay` | T#200MS | ms | Time to wait after commanding the brake to engage before proceeding. Should be slightly longer than the mechanical brake's specified engagement time. |
| `G_cfgBrakeDisengageDelay`| T#100MS | ms | Time to wait after commanding the brake to release before proceeding. Should be slightly longer than the mechanical brake's specified release time. |
| `G_cfgHomingTimeout` | T#30S | s | Maximum time allowed for an entire homing sequence (Mode 110 or 111) to complete. **Tuning:** Increase if the travel distance is long or homing velocities are very slow. |
| `G_cfgStallDetectTime` | T#200MS | ms | The duration a stall condition (high torque, low velocity) must be met continuously during EOT homing to be considered a valid hard stop. **Tuning:** Increase to prevent false stall detections from friction. Decrease for faster homing, but with higher risk of false detection. |
| `G_cfgDriveReadyTimeout` | T#5S | s | Max time to wait for the servo drive to report "Ready" after being enabled. Should not need adjustment unless the drive has an unusually long power-on sequence. |
| `G_cfgFaultIdleTimeout` | T#30S | s | When a fault occurs, the system enters a controlled stop. If the master does not acknowledge/reset the fault within this time, the slave will proceed to a safe-idle state (drive off, brake on). |

---

## 3. Position and Velocity Limits

These define the safe operating envelope of the machine.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPosSoftLimitMax` | 305.0 | mm | Software-enforced maximum extension position. A `FAULT_POSITION` occurs if exceeded. Should be set within the `G_cfgUsableStroke`. |
| `G_cfgPosSoftLimitMin` | 0.0 | mm | Software-enforced minimum retraction position (home position). A `FAULT_POSITION` occurs if exceeded. |
| `G_cfgPosExitGuardMargin`| 5.0 | mm | Safety margin for the piston exit guard. The guard becomes active when the position is `G_cfgPosSoftLimitMin + G_cfgPosExitGuardMargin`. |
| `G_cfgVelLimitMax` | 200.0 | mm/s | Absolute maximum velocity allowed in any mode. |
| `G_cfgVelLimitNormal` | 100.0 | mm/s | The velocity limit used during standard Position Control mode. |
| `G_cfgInMotionVelThreshold`| 0.1 | mm/s | The velocity above which the `G_doInMotion` output is considered active. **Tuning:** Increase if motor vibration at rest causes the flag to flicker. |

### Position Command Gate

Parameters for `fbPosGate` (`FB_PositionCommandGate`), which shapes the direct-control position command in `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`. Velocity mode and torque mode are unaffected.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgScanTime` | 0.002 | s | Cyclic task period (`LREAL`), used as `dt` by the position command gate and as the basis for the analog input filter's time-constant alpha. **Must equal the cyclic task interval set in the MotionWorks IEC task configuration** (2 ms is the MP2600iec minimum); a mismatch scales every commanded velocity and the filter time constant by the same ratio. |
| `G_cfgPosGateAccelMax` | 5.0 | mm/s² | Acceleration limit applied to the gated position command. Conservative starting value — raise during commissioning once bench testing shows torque headroom. **Tuning:** This is the transparency knob for an incidental master ramp: the gate lags a ramp at velocity `v` by `v²/(2·A)`, e.g. 0.9 mm at 3 mm/s with `A` = 5 mm/s², vs. 0.09 mm with `A` = 50 mm/s². |
| `G_cfgPosGateMaxDeviation` | 2.0 | mm | Tether window: once the gated command leads `G_sysActualPosition` by this much in the direction of travel, the command holds until the axis catches up or the target reverses. **Tuning:** Must sit comfortably below the drive's `Pn520` excessive-deviation level so the tether acts first, and comfortably above normal following error. Replaces the undocumented `G_cfgPosCommandDeviationLimit`, which has been deleted. |

---

## 4. Torque Limits

These parameters control the force applied by the motor.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgTorqueLimitMax` | 300.0 | % | Absolute maximum torque allowed. This is a primary safety limit to prevent mechanical damage. |
| `G_cfgTorqueLimitNormal`| 100.0 | % | Torque limit used during standard Position and Velocity control modes. |
| `G_cfgTorqueHomingLimit`| 60.0 | % | Torque limit applied during the final slow approach of EOT Homing (Mode 111). This should be high enough to overcome friction but low enough to avoid damaging the mechanical hard stop. |

---

## 5. Input Filtering

These parameters control the software filters applied to raw I/O signals to reject noise.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgDebounceTimeMode`| T#50MS | ms | Debounce time for the mode command bits. **Tuning:** Increase if noisy signals cause mode commands to be unstable. Decrease for faster mode changes. |
| `G_cfgDebounceTimeMotion`| T#20MS | ms | Debounce time for the Motion Enable signal. Should be relatively short for responsiveness. |
| `G_cfgDebounceTimeFault`| T#20MS | ms | Debounce time for the Fault Reset signal. |
| `G_cfgDebounceTimeLimitStd`| T#5MS | ms | Debounce time for the Retract limit switch. |
| `G_cfgDebounceTimeLimitHome`| T#2MS | ms | Debounce time for the Home limit switch. This is shorter to increase the precision and repeatability of the Home to Limit sequence. |
| `G_cfgAnalogFilterTimeConst`| T#50MS | ms | Time constant for the IIR low-pass filter on the analog reference input. **Tuning:** Increase for smoother but delayed response. Decrease for faster but potentially noisier response. |
| `G_cfgAnalogMedianSize`| 3 | count | The number of samples for the median filter on the analog input (valid: 3 or 5). The median filter is excellent at rejecting large, single-sample spikes. |

---

## 6. Homing Parameters

These parameters define the behavior of the three homing modes.

### Mode 110: Home to Limit Switch
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgHomeLimApproachVel`| 50.0 | mm/s | The velocity used to approach the home limit switch. |
| `G_cfgHomeLimBackoffDist`| 5.0 | mm | The distance the axis moves away from the switch after triggering it, before setting the final home position. |
| `G_cfgHomeLimSetPosition`| 0.0 | mm | The absolute position value that will be assigned to the axis once the homing is complete. This defines the origin of the coordinate system. |

### Mode 111: Home to End-of-Travel
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgHomeEOTFastVel` | 50.0 | mm/s | The velocity for the initial fast approach toward the hard stop. |
| `G_cfgHomeEOTSlowVel` | 5.0 | mm/s | The final, slow velocity for the torque-limited contact with the hard stop. |
| `G_cfgHomeEOTApproachDist`| 20.0 | mm | The distance from the expected EOT position at which the axis will switch from fast to slow velocity. |
| `G_cfgHomeEOTTorqueThresh`| 50.0 | % | The torque level (as % of motor rated torque) used to detect stall during EOT homing. The actual detection threshold is `0.9 * G_cfgHomeEOTTorqueThresh` (i.e. 45% by default; see [Stall_Detection_Calculation](../Stall_Detection_Calculation.md)). **Tuning:** Set it high enough to avoid false stalls from friction, but low enough to reliably detect contact with the hard stop. |
| `G_cfgHomeEOTSetPosition`| 300.0 | mm | The "ideal" master coordinate position at the EOT. The slave calculates an offset so that the master sees this position value when the slave is at the physical EOT. |

### Mode 101: Go Home
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgGoHomePosition` | 0.0 | mm | The target position for the Go Home command. This should match `G_cfgHomeLimSetPosition`. |
| `G_cfgGoHomeVelocity` | 50.0 | mm/s | The velocity used for the move-to-home motion. |

---

## 7. Analog I/O Scaling

These parameters define how analog voltages are mapped to physical units. For more details, see the [IOReference](../../master/IOReference.md).

| Parameter | Default Value | Units | Description |
|---|---|---|---|
| `G_cfgAnalogVelMin` | -100.0 | mm/s | Velocity corresponding to a -10V command. |
| `G_cfgAnalogVelMax` | 100.0 | mm/s | Velocity corresponding to a +10V command. |
| `G_cfgAnalogTorqueMin`| -100.0 | % | Torque corresponding to a -10V command. |
| `G_cfgAnalogTorqueMax`| 100.0 | % | Torque corresponding to a +10V command. |
| `G_cfgPosMapStage1PosMin` | 0.0 | mm | Start position for the high-resolution mapping region. |
| `G_cfgPosMapStage1VoltMin`| -10.0 | V | Voltage corresponding to the start position of Stage 1. |
| `G_cfgPosMapTransitionPos`| 200.0| mm | Position where mapping changes from Stage 1 to Stage 2. |
| `G_cfgPosMapTransitionVolt`| 5.0 | V | Voltage at the transition point. |
| `G_cfgPosMapStage2PosMax` | 305.0 | mm | End position for the low-resolution mapping region. |
| `G_cfgPosMapStage2VoltMax` | 10.0 | V | Voltage corresponding to the end position of Stage 2. |
