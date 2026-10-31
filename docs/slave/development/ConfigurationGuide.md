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
| `G_cfgBrakeEngageDelay` | T#500MS | ms | Time to wait after commanding the brake to engage before proceeding. Should be slightly longer than the mechanical brake's specified engagement time. |
| `G_cfgBrakeDisengageDelay`| T#500MS | ms | Time to wait after commanding the brake to release before proceeding. Should be slightly longer than the mechanical brake's specified release time. |
| `G_cfgHomingTimeout` | T#90S | s | Maximum time allowed for the Mode 110 homing sequence to complete. **Tuning:** Increase if the travel distance is long or homing velocities are very slow. |
| `G_cfgDriveReadyTimeout` | T#5S | s | Max time to wait for the servo drive to report "Ready" after being enabled. Should not need adjustment unless the drive has an unusually long power-on sequence. |
| `G_cfgFaultIdleTimeout` | T#30S | s | When a fault occurs, the system enters a controlled stop. If the master does not acknowledge/reset the fault within this time, the slave will proceed to a safe-idle state (drive off, brake on). |

---

## 3. Position and Velocity Limits

These define the safe operating envelope of the machine.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPosSoftLimitMax` | 367.0 | mm | Software-enforced maximum extension position. A `FAULT_POSITION` occurs if exceeded. |
| `G_cfgPosSoftLimitMin` | -2.5 | mm | Software-enforced minimum retraction position. It sits between position 0 (the homed retract endpoint) and the negative overtravel switch at `-G_cfgHomeLimRetractDist`; keep it bracketed by the two if `G_cfgHomeLimRetractDist` changes. A `FAULT_POSITION` occurs if exceeded. |
| `G_cfgPosExitGuardMargin`| 20.0 | mm | Safety margin for the piston exit guard. The guard's torque/velocity danger-zone monitoring is active below `G_cfgPosCylinderEntry + G_cfgPosExitGuardMargin`. |
| `G_cfgVelLimitMax` | 7.0 | mm/s | Absolute maximum velocity allowed in any mode. |
| `G_cfgVelLimitNormal` | 6.0 | mm/s | The velocity limit used during standard Position Control mode. |
| `G_cfgInMotionVelThreshold`| 0.1 | mm/s | The velocity above which the `G_doInMotion` output is considered active. **Tuning:** Increase if motor vibration at rest causes the flag to flicker. |

### Position Command Gate

Parameters for `fbPosGate` (`FB_PositionCommandGate`), which shapes the direct-control position command in `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`. Velocity mode and torque mode are unaffected.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgScanTime` | 0.004 | s | Cyclic task period (`LREAL`), used as `dt` by the position command gate and as the basis for the analog input filter's time-constant alpha. **Must equal the cyclic task interval set in the MotionWorks IEC task configuration**, and that interval must be one the program completes within: the present program takes about 2.5 ms a scan, so a 2 ms task overruns and runs on every second motion cycle, 4 ms, with nothing reported unless the task watchdog is enabled. A mismatch scales the filter time constant and every scan-based delay by the same ratio. |
| `G_cfgPosGateAccelMax` | 10.0 | mm/s² | Acceleration limit applied to the gated position command. Conservative starting value — raise during commissioning once bench testing shows torque headroom. **Tuning:** This is the transparency knob for an incidental master ramp: the gate lags a ramp at velocity `v` by `v²/(2·A)`, e.g. 0.9 mm at 3 mm/s with `A` = 5 mm/s², vs. 0.09 mm with `A` = 50 mm/s². |
| `G_cfgDirectCtrlAccel` / `G_cfgDirectCtrlDecel` | 10.0 | mm/s² | Written to `Y_DirectControl`'s `Acceleration` input (the block has no decel input) in every direct-control state except pressure mode, which uses `G_cfgPressureAccel`. A velocity-step test on the machine (`bench/characterisation/README.md` section 8) found the velocity reference followed within 60 ms whatever the step, so in VelocityTLMode this value does not slew the command; confirm by repeating a step at 1 and at 100 mm/s². |
| `G_cfgDirectCtrlFilterTime` | 0.05 | s | `Y_DirectControl`'s `FilterTimeConstant`, the block's own filter on the command it sends the drive, wired from `G_cmdDirectControl.FilterTimeConstant` (`docs/slave/ld/`). It is a lag inside the pressure loop, so it belongs with the loop's tuning; the same step test found no 50 ms lag in the velocity response, so which modes it acts on is to be established on the bench. |
| `G_cfgPosGateMaxDeviation` | 2.0 | mm | Tether window: once the gated command leads `G_sysActualPosition` by this much in the direction of travel, the command holds until the axis catches up or the target reverses. **Tuning:** Must sit comfortably below the drive's `Pn520` excessive-deviation level so the tether acts first, and comfortably above normal following error. |

---

## 4. Torque Limits

These parameters control the force applied by the motor.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgTorqueLimitMax` | 110.0 | % | Absolute maximum torque allowed. This is a primary safety limit to prevent mechanical damage. |
| `G_cfgTorqueLimitNormal`| 100.0 | % | Torque limit used during standard Position and Velocity control modes. |

---

## 5. Input Filtering

These parameters control the software filters applied to raw I/O signals to reject noise.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgDebounceTimeMode`| T#50MS | ms | Debounce time for the mode code register (`G_mbModeCode`, HR2): the code must hold one value this long before it is decoded. **Tuning:** Increase if mode commands are unstable. Decrease for faster mode changes. |
| `G_cfgDebounceTimeMotion`| T#20MS | ms | Debounce time for the Motion Enable register (`G_mbMotionEnable`, HR3). Should be relatively short for responsiveness. |
| `G_cfgDebounceTimeFault`| T#20MS | ms | Debounce time for the Fault Reset register (`G_mbFaultReset`, HR4). |
| `G_cfgDebounceTimeOvertravelNeg`| T#10MS | ms | Debounce time for the negative overtravel switch (`G_diOvertravelNeg`), which is also the Mode 110 home reference. |
| `G_cfgDebounceTimeOvertravelPos`| T#10MS | ms | Debounce time for the positive overtravel switch (`G_diOvertravelPos`). |
| `G_cfgAnalogFilterTimeConst`| T#20MS | ms | Time constant for the IIR low-pass filter on the pressure transducer input (`G_aiPressure`), after the median filter. The filter runs once per scan with alpha = `G_cfgScanTime` / (time constant + `G_cfgScanTime`), so the time constant is this value only when `G_cfgScanTime` is the real scan. **Tuning:** Its lag adds to the pressure loop's overshoot on a target step; the median filter and the capacitor across the loop shunt leave little noise for it to remove. Increase only if the filtered reading is visibly noisy. |
| `G_cfgAnalogMedianSize`| 5 | count | The number of samples for the median filter on the pressure transducer input (valid: 3 or 5). The median filter is excellent at rejecting large, single-sample spikes. |

---

## 6. Homing Parameters

These parameters define the behavior of the homing (110) and Go Home (101) modes. Mode code 111 (7) is Pressure Control, not a homing mode.

### Mode 110: Home to Limit Switch
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgHomeLimFastApproachVel` | 6.0 | mm/s | Coarse approach toward the negative overtravel switch. |
| `G_cfgHomeLimSlowApproachVel` | 1.0 | mm/s | Slow re-approach that captures the precise switch position. |
| `G_cfgHomeLimBackoffVel` | 5.0 | mm/s | Velocity for the backoff between approaches and for the final retract. |
| `G_cfgHomeLimRetractDist` | 5.0 | mm | Distance retracted from the precise switch position to the retract endpoint. The switch lands at `-G_cfgHomeLimRetractDist` in the homed frame. |
| `G_cfgHomeLimSetPosition`| 0.0 | mm | The position value assigned at the retract endpoint. This defines the origin of the coordinate system. |
| `G_cfgHomeLimAccel` | 5.0 | mm/s² | Acceleration for all homing moves. |
| `G_cfgHomeLimDecel` | 50.0 | mm/s² | Deceleration for all homing moves. |

### Mode 101: Go Home
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgGoHomePosition` | 0.0 | mm | The target position for the Go Home command. This should match `G_cfgHomeLimSetPosition`. |
| `G_cfgGoHomeVelocity` | 6.0 | mm/s | The velocity used for the move-to-home motion. |
| `G_cfgGoHomeAccel` | 5.0 | mm/s² | Acceleration for the Go Home move. |
| `G_cfgGoHomeDecel` | 10.0 | mm/s² | Deceleration for the Go Home move. |
| `G_cfgGoHomeStopDecel` | 10.0 | mm/s² | Deceleration for `MC_Stop` commands. |

---

## 7. Pressure Loop Gain Schedule

When `G_cfgPressureScheduleEnable` is TRUE on entry to pressure mode, `FB_PressureSchedule` sets the pressure loop's Kp, Ki and stop-law velocity limit every scan from an upper bound on the chamber stiffness, so the loop's crossover stays at or below `G_cfgPressureSchedCrossover`. The stiffness bound is evaluated at the present pressure and at the larger of the present and the target pressure, and the larger value is used. The choice is latched on entry. If a setting is out of range (`ParamError`), the loop runs on the fixed `G_cfgPressureKp`, `G_cfgPressureKi` and `G_cfgPressureStopStiffness`, which are unchanged and remain the fallback when the schedule is off.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPressureScheduleEnable` | FALSE | BOOL | Selects the gain schedule. Latched on entry to pressure mode, so changing it during a run changes nothing until the next entry. |
| `G_cfgPressureSchedCrossover` | 11.0 | rad/s | Target crossover of the pressure loop, wc. The scheduled Kp is `wc / KEst`, bounded by the two Kp limits. |
| `G_cfgPressureSchedKiRatio` | 0.4 | 1/s | Ki / Kp. |
| `G_cfgPressureSchedKpMin` | 0.001 | mm/s per MPa | Lower bound on the scheduled Kp. |
| `G_cfgPressureSchedKpMax` | 5.0 | mm/s per MPa | Upper bound on the scheduled Kp. |
| `G_cfgPressureSchedPistonArea` | 3431.0 | mm² | Piston area (2.602 in bore). |
| `G_cfgPressureSchedSalineVolume` | 343.0 | mL | Saline volume of the fill including the dead volume. Must be at or below the true fill: a smaller value makes the bound stiffer (lower gain, safe). |
| `G_cfgPressureSchedDeadVolume` | 74.0 | mL | Chamber volume with the piston at `G_cfgPosSoftLimitMax` and the hose open. 74 mL is a lower bound on the 79 mL at the end of travel (passages 50 mL + hose 29 mL). With the soft limit short of the end of travel (305 vs 365 mm in the reference file) the bound is stiffer than the chamber, which only lowers the gain; set 74 + 3.431·(365 − `G_cfgPosSoftLimitMax`) mL to remove that. It must never exceed the true volume. |
| `G_cfgPressureSchedHoseCompliance` | 1.2E-13 | m³/Pa | Hose compliance. Must be at or below the true value. Set 0 when the proximal valve is opened only after mixing, because the hose is then not part of the chamber during the compression. |
| `G_cfgPressureSchedBulkModulus` | 1.0E9 | Pa | Saline bulk modulus. Must be at or above the true value. |
| `G_cfgPressureSchedOxygenRatio` | 1.0 | – | Oxygen charge as STP volume per saline volume. Must equal the recipe of the fill (1.0 for 1:1). A lower configured charge makes the bound stiffer (safe, slower); a higher one makes it softer (unsafe). |
| `G_cfgPressureSchedHenry` | 1.22E-5 | mol/(m³·Pa) | Henry constant of oxygen in 0.9 % saline. |
| `G_cfgPressureSchedGasTemp` | 293.0 | K | Gas temperature. |
| `G_cfgPressureSchedPatm` | 101325.0 | Pa | Atmospheric pressure. |
| `G_cfgPressureSchedArriveBand` | 0.5 | MPa | |error| at which the integrator is first released; it is held at zero until then. |
| `G_cfgPressureSchedDriveDeadTime` | 0.0228 | s | Drive dead time from command to response, measured on the bench (`bench/characterisation`) with `Y_DirectControl`'s FilterTimeConstant as wired. Re-measure if that pin changes. |
| `G_cfgPressureSchedDriveLag` | 0.0182 | s | Drive velocity-loop time constant, measured as above. |
