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
| `G_cfgCommTimeout` | T#300MS | ms | Master heartbeat (`G_mbHeartbeat`, HR1) unchanged for this long raises `FAULT_COMM`, but only while motion is commanded. It spans several exchanges (50 ms each), to ride out a slow one. **Tuning:** Increase if a slow exchange causes nuisance link-loss faults; decrease for faster detection of a frozen master. |
| `G_cfgHoldPositionTimeout` | T#2M | min | Hold-position inactivity watchdog: if no handshake completes within this time after motion halts in `ST_HOLD_POSITION`, the slave shuts down gracefully to idle (no fault). |
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
| `G_cfgPosCylinderEntry` | 176.0 | mm | Physical cylinder-entry position in the homed frame. Below it the piston is leaving the cylinder bore; the exit guard's boundary is this value plus `G_cfgPosExitGuardMargin`. **Tuning:** Measure on the machine and set to the actual mechanical entry point. |
| `G_cfgPosExitGuardMargin`| 20.0 | mm | Safety margin for the piston exit guard. The guard's torque/velocity danger-zone monitoring is active below `G_cfgPosCylinderEntry + G_cfgPosExitGuardMargin`. |
| `G_cfgVelLimitMax` | 6.0 | mm/s | The one axis speed ceiling: `stCurrentLimits.VelMax` in velocity, torque and velocity recovery, and the upper bound of the position speed. The mode speeds (`G_cfgGoHomeVelocity`, the three `G_cfgHomeLim*Vel`, `G_cfgPressureVelMax`) reach their blocks through `MIN(.., G_cfgVelLimitMax)`, so none can exceed it, and `masterConfig.jogVelMax` must not either (checked in `tests/closedLoopMasterSlave.m`). `Y_DirectControl` does no path shaping in PositionMode (the position command reaches the drive as given), so its `Velocity` and `Acceleration` inputs set no limits there; `FB_PositionPathShaper` does the path shaping, and its `VelMax` and `AccelMax` (`G_cfgPosShaperAccelMax`) limit a position move. `VelMax` is the master's position speed (HR7-8, `G_mbPositionSpeed`) limited to `0..G_cfgVelLimitMax`, evaluated every scan; a speed of 0 brakes the command to rest and holds. `Y_DirectControl`'s `Velocity` input is given the same value in position mode, so the two never disagree. `masterConfig.positionSpeedMax` must not exceed the ceiling either (checked in `tests/closedLoopMasterSlave.m`). |
| `G_cfgStopDecel` | 10.0 | mm/s² | The one `MC_Stop` deceleration: hold, fault, recovery and the Go Home abort. |
| `G_cfgInMotionVelThreshold`| 0.1 | mm/s | The velocity above which the `G_sysInMotion` flag is set. **Tuning:** Increase if motor vibration at rest causes the flag to flicker. |

### Mechanical scaling (reference copies)

The controller's unit conversion is set in Hardware Configuration. These two globals are reference copies of it: nothing in `slave-src` reads them, and the benches take them as fallbacks (`bench/common/benchGvl.m`).

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgGearRatio` | 70.0 | motor turns per screw turn | Gear ratio of the actuator (Tolomatic RSA64 with the Sigma-7 400W motor). Changing it here changes nothing on the controller. |
| `G_cfgLeadScrewPitch` | 5.08 | mm per screw turn | Lead screw pitch (0.2 inch). Changing it here changes nothing on the controller. |

### Position Path Shaper

Parameters for `fbPosShaper` (`FB_PositionPathShaper`), which shapes the direct-control position command in `ST_POSITION_CTRL` and `ST_RECOVERY_POSITION`. Velocity mode and torque mode are unaffected.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgScanTime` | 0.004 | s | Cyclic task period (`LREAL`), used as `dt` by the position path shaper and as the basis for the analog input filter's time-constant alpha. **Must equal the cyclic task interval set in the MotionWorks IEC task configuration**, and that interval must be one the program completes within: the present program takes about 2.5 ms a scan, so a 2 ms task overruns and runs on every second motion cycle, 4 ms, with nothing reported unless the task watchdog is enabled. A mismatch scales the filter time constant and every scan-based delay by the same ratio. |
| `G_cfgPosShaperAccelMax` | 10.0 | mm/s² | Acceleration limit applied to the shaped position command. Conservative starting value — raise during commissioning once bench testing shows torque headroom. **Tuning:** This is the transparency knob for an incidental master ramp: the shaper lags a ramp at velocity `v` by `v²/(2·A)`, e.g. 0.9 mm at 3 mm/s with `A` = 5 mm/s², vs. 0.09 mm with `A` = 50 mm/s². |
| `G_cfgDirectCtrlAccel` | 10.0 | mm/s² | Written to `Y_DirectControl`'s `Acceleration` input (the block has no decel input, so the one rate applies in both directions) in every direct-control state except pressure mode, which uses `G_cfgPressureAccel`. A velocity-step test on the machine (`bench/characterisation/README.md` section 8) found the velocity reference followed within 60 ms whatever the step, so in VelocityTLMode this value does not slew the command; confirm by repeating a step at 1 and at 100 mm/s². |
| `G_cfgDirectCtrlFilterTime` | 0.05 | s | `Y_DirectControl`'s `FilterTimeConstant`, the block's own filter on the command it sends the drive, wired from `G_cmdDirectControl.FilterTimeConstant` (`docs/slave/ld/`). It is a lag inside the pressure loop, so it belongs with the loop's tuning; the same step test found no 50 ms lag in the velocity response, so which modes it acts on is to be established on the bench. |
| `G_cfgPosShaperMaxDeviation` | 2.0 | mm | Tether window: once the shaped command leads `G_sysActualPosition` by this much in the direction of travel, the command holds until the axis catches up or the target reverses. **Tuning:** Must sit comfortably below the drive's `Pn520` excessive-deviation level so the tether acts first, and comfortably above normal following error. |

---

## 4. Torque Limits

These parameters control the force applied by the motor.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgTorqueLimitMax` | 110.0 | % | Absolute maximum torque allowed. This is a primary safety limit to prevent mechanical damage. |
| `G_cfgTorqueLimitNormal`| 100.0 | % | Torque limit used during standard Position and Velocity control modes. |
| `G_cfgTorqueStallThreshold` | 60.0 | % | Torque above which the piston exit guard counts the motor as pushing into the cylinder (`FB_PistonExitGuard`'s `TorqueThreshold`). Inside the guard band, this torque with the piston still moving out raises `FAULT_PISTON_EXIT`. |

---

## 5. Input Filtering

These parameters control the software filters applied to raw I/O signals to reject noise.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgDebounceTimeMode`| T#50MS | ms | Debounce time for the mode code register (`G_mbModeCode`, HR2): the code must hold one value this long before it is decoded. **Tuning:** Increase if mode commands are unstable. Decrease for faster mode changes. |
| `G_cfgDebounceTimeCommand`| T#20MS | ms | Debounce time for the Motion Enable register (`G_mbMotionEnable`, HR3) and the Fault Reset register (`G_mbFaultReset`, HR4). Should be relatively short for responsiveness. |
| `G_cfgDebounceTimeOvertravel`| T#10MS | ms | Debounce time for both overtravel switches (`G_diOvertravelNeg`, `G_diOvertravelPos`). The negative switch is also the Mode 110 home reference, so keep it short for homing precision. |
| `G_cfgAnalogFilterTimeConst`| T#20MS | ms | Time constant for the IIR low-pass filter on the pressure transducer input (`G_aiPressure`), after the median filter. The filter runs once per scan with alpha = `G_cfgScanTime` / (time constant + `G_cfgScanTime`), so the time constant is this value only when `G_cfgScanTime` is the real scan. **Tuning:** Its lag adds to the pressure loop's overshoot on a target step; the median filter and the capacitor across the loop shunt leave little noise for it to remove. Increase only if the filtered reading is visibly noisy. |
| `G_cfgAnalogMedianSize`| 5 | count | The number of samples for the median filter on the pressure transducer input (valid: 3 or 5). The median filter is excellent at rejecting large, single-sample spikes. |

---

## 6. Homing Parameters

These parameters define the behavior of the homing (110) and Go Home (101) modes. Mode code 111 (7) is Pressure Control, not a homing mode.

### Homing and Go Home moves
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgMoveAccel` | 5.0 | mm/s² | Acceleration of every move the homing modes make: Mode 110's approaches, backoff and retract, and Mode 101's Go Home move. |

### Mode 110: Home to Limit Switch
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgHomeLimFastApproachVel` | 6.0 | mm/s | Coarse approach toward the negative overtravel switch. |
| `G_cfgHomeLimSlowApproachVel` | 1.0 | mm/s | Slow re-approach that captures the precise switch position. |
| `G_cfgHomeLimBackoffVel` | 5.0 | mm/s | Velocity for the backoff between approaches and for the final retract. |
| `G_cfgHomeLimRetractDist` | 5.0 | mm | Distance retracted from the precise switch position to the retract endpoint. The switch lands at `-G_cfgHomeLimRetractDist` in the homed frame. |
| `G_cfgHomeLimSetPosition`| 0.0 | mm | The position value assigned at the retract endpoint. This defines the origin of the coordinate system. |
| `G_cfgHomeLimDecel` | 50.0 | mm/s² | Deceleration for all homing moves. |

### Mode 101: Go Home
| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgGoHomePosition` | 0.0 | mm | The target position for the Go Home command. This should match `G_cfgHomeLimSetPosition`. |
| `G_cfgGoHomeVelocity` | 6.0 | mm/s | The velocity used for the move-to-home motion. |
| `G_cfgGoHomeDecel` | 10.0 | mm/s² | Deceleration for the Go Home move. |

---

## 7. Step Probe

The scan-rate recorder `FB_Probe` (ModbusRegisterMap.md, section 6). Nothing in the protocol depends on it.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgProbeEnable` | TRUE | | FALSE stops the probe: nothing is recorded, arms are not acted on and `G_mbProbeState` reads `PROBE_OFF`. The Modbus groups still span IR524 and HR114. |
| `G_cfgProbeFillPerScan` | 6 | records | Records `FB_Probe` copies into the window per scan (limited to 1-13), 8 words each: the bound on the cost a transfer adds to one scan. A window of 13 records takes 3 scans at 6 and 13 at 1. **Tuning:** Lower it if scan time is tight; raise it if a stream reader needs the window refilled faster than every 6 ms. |

---

## 8. Pressure Loop Gain Schedule

When `G_cfgPressureScheduleEnable` is TRUE on entry to pressure mode, `FB_PressureSchedule` sets the pressure loop's Kp, Ki and stop-law velocity limit every scan from an upper bound on the chamber stiffness, so the loop's crossover stays at or below `G_cfgPressureSchedCrossover`. The saline volume and the oxygen charge of the fill are not settings: the master writes them in holding registers HR7-10 (`G_mbSalineVolume`, `G_mbOxygenRatio`, see `docs/master/ModbusRegisterMap.md`), the slave latches them on entry to pressure mode, and an unwritten (zero) saline volume makes the schedule fall back to the configured gains. Both must be at or below the true values, and the charge must equal the recipe of the fill. The stiffness bound is evaluated at the present pressure and at the larger of the present and the target pressure, and the larger value is used. The choice is latched on entry. If a setting is out of range (`ParamError`), the loop runs on the fixed `G_cfgPressureKp`, `G_cfgPressureKi` and `G_cfgPressureStopStiffness`, which are unchanged and remain the fallback when the schedule is off.

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPressureScheduleEnable` | FALSE | BOOL | Selects the gain schedule. Latched on entry to pressure mode, so changing it during a run changes nothing until the next entry. |
| `G_cfgPressureSchedCrossover` | 11.0 | rad/s | Target crossover of the pressure loop, wc. The scheduled Kp is `wc / KEst`, bounded by the two Kp limits. |
| `G_cfgPressureSchedKiRatio` | 0.4 | 1/s | Ki / Kp. |
| `G_cfgPressureSchedKpMin` | 0.001 | mm/s per MPa | Lower bound on the scheduled Kp. |
| `G_cfgPressureSchedKpMax` | 5.0 | mm/s per MPa | Upper bound on the scheduled Kp. |
| `G_cfgPressureSchedPistonArea` | 3431.0 | mm² | Piston area (2.602 in bore). |
| `G_cfgPressureSchedEndOfTravel` | 367.0 | mm | Position at the end of travel, where the chamber is down to the passages (the same frame as `G_cfgPosSoftLimitMax`). The chamber volume is the swept volume from the piston to this position plus the passages and, when open, the hose. |
| `G_cfgPressureSchedPassagesVolume` | 57.0 | mL | Cylinder passages: chamber volume at the end of travel without the hose, measured with the present fittings. Must be at or below the true value; re-measure when the fittings change. |
| `G_cfgPressureSchedHoseVolume` | 29.0 | mL | Hose between the cylinder head and the outlet, measured. Counted only while `G_cfgPressureSchedHoseOpen` is TRUE. Must be at or below the true value. |
| `G_cfgPressureSchedHoseCompliance` | 1.2E-13 | m³/Pa | Hose compliance beyond its liquid (wall and any trapped air), a lower bound on the measured 0.24 mL/MPa. Counted only while `G_cfgPressureSchedHoseOpen` is TRUE. |
| `G_cfgPressureSchedHoseOpen` | FALSE | BOOL | The proximal valve is open during the compression, so the hose volume and compliance are part of the chamber. Leave FALSE while the valve opens after mixing; set TRUE for a procedure that opens it before the compression. The measured hose values above do not change with it. |
| `G_cfgPressureSchedBulkModulus` | 1.0E9 | Pa | Saline bulk modulus. Must be at or above the true value. |
| `G_cfgPressureSchedHenry` | 1.22E-5 | mol/(m³·Pa) | Henry constant of oxygen in 0.9 % saline. |
| `G_cfgPressureSchedGasTemp` | 293.0 | K | Gas temperature. |
| `G_cfgPressureSchedPatm` | 101325.0 | Pa | Atmospheric pressure. |
| `G_cfgPressureSchedArriveBand` | 0.5 | MPa | |error| at which the integrator is first released; it is held at zero until then. |
| `G_cfgPressureSchedDriveDeadTime` | 0.0228 | s | Drive dead time from command to response, measured on the bench (`bench/characterisation`) with `Y_DirectControl`'s FilterTimeConstant as wired. Re-measure if that pin changes. |
| `G_cfgPressureSchedDriveLag` | 0.0182 | s | Drive velocity-loop time constant, measured as above. |

---

## 9. Pressure Transducer and Pressure Loop

The transducer is a 4-20 mA loop read as the voltage across a shunt on the controller's analog input. `FB_PressureInput` scales it and checks the live zero; `FB_PressureController` closes a PI loop that commands piston velocity through `Y_DirectControl` in VelocityTLMode (mode 111), clamped by a stop law so the axis can stop within the pressure error at the assumed chamber stiffness. The gain schedule of section 8 replaces the fixed gains and stop stiffness below when it is enabled.

### Transducer

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPressureShuntOhms` | 421.4 | ohm | Volts the input reads per ampere of loop current: the 434 ohm shunt in parallel with the input's own load, calibrated end to end against a current source (`bench/pressure/README.md` section 6). |
| `G_cfgPressureZeroMa` | 4.0 | mA | Loop current at `G_cfgPressureRangeMin`. |
| `G_cfgPressureSpanMa` | 16.0 | mA | Loop current span. |
| `G_cfgPressureRangeMin` | 0.0 | MPa gauge | Pressure at `G_cfgPressureZeroMa`. |
| `G_cfgPressureRangeMax` | 16.0 | MPa gauge | Pressure at `G_cfgPressureZeroMa + G_cfgPressureSpanMa`. |
| `G_cfgPressureSensorMinMa` | 3.6 | mA | Loop current below this means an open loop or an unpowered transducer. |
| `G_cfgPressureSensorMaxMa` | 20.5 | mA | Loop current above this means over range or a shorted shunt. |
| `G_cfgPressureSensorFaultTime` | T#100MS | ms | An out-of-range current must persist this long before `FAULT_PRESSURE_SENSOR` (raised only while the loop is closed on the sensor). |

### Loop

| Parameter | Default Value | Units | Description & Tuning Considerations |
|---|---|---|---|
| `G_cfgPressureKp` | 0.5 | mm/s per MPa | Velocity per pressure error. Fallback when the schedule is off or out of range. |
| `G_cfgPressureKi` | 0.2 | mm/s per (MPa·s) | Velocity per integrated pressure error. Fallback as above. |
| `G_cfgPressureVelMax` | 5.0 | mm/s | Velocity clamp in pressure mode (also limited by `G_cfgVelLimitMax`). |
| `G_cfgPressureAccel` | 50.0 | mm/s² | `Y_DirectControl` acceleration in pressure mode. |
| `G_cfgPressureDecel` | 50.0 | mm/s² | Deceleration the pressure stop law assumes (`Y_DirectControl` has no decel input). The chamber stiffens by orders of magnitude as the gas dissolves, so the command must reverse within `v²/(2·decel)` of travel. |
| `G_cfgPressureStopStiffness` | 100.0 | MPa/mm | Chamber stiffness assumed by the stop law: the velocity is clamped to `sqrt(2·Decel·|error|/stiffness)`. Above the liquid-only chamber's about 20 MPa/mm because the filtered reading lags the chamber during the approach. Fallback when the schedule is off. |
| `G_cfgPressureStopVelMin` | 2.0 | mm/s | The stop law never clamps below this, so outflow can be balanced at zero error. It must exceed the piston speed the outlet needs (1.4 mm/s at 10 MPa through the orifice bank) or the loop holds a steady error below the target. Overshoot from this velocity is at most `stiffness·v²/(2·decel)`. |
| `G_cfgPressureTorqueLimit` | 100.0 | % of rated | Torque limit in pressure mode. Caps the piston force if the sensor reads low. |
| `G_cfgPressureTargetMax` | 15.0 | MPa gauge | Highest target accepted from the master. |
| `G_cfgPressureInBand` | 0.1 | MPa | `|actual - target|` below which the in-band status bit (`G_sysPerformanceStatus`, IR7) is set. |
