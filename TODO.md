# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21), the simulator, models and link timing work (PR #22), and
the pressure-loop design, bench and scheduled-controller work (PR #17 and
the scheduled pressure controller), and the pressure input calibration.
Grouped by where they have to be done. Settled: the mechanical pressure
ceiling is 12 MPa; the saline volume and oxygen charge reach the slave in
holding registers.

## MotionWorks project

Bring the controller project in line with `slave-src`. The headers of the
`.st` files and `slave-src/GVL/GlobalVariables_Reference.st` are the
reference for every declaration.

- [ ] LD POU: delete the `MC_MoveVelocity` instance, then delete
      `docs/slave/ld/MC_MoveVelocity_1.PNG`.
- [ ] Data types:
  - [ ] Remove the `ST_HOME_LIM_*` members from `E_SystemState`.
  - [ ] Remove `ST_CmdMoveVelocity`, `ST_StaMoveVelocity` and the
        `Deceleration` field of `ST_CmdDirectControl`.
  - [ ] Add `E_ProbeState`, `E_ProbeXferState`, `ARRAY104WORD` and
        `ARRAY65536WORD`.
- [ ] Function blocks:
  - [ ] Mode and confirmation path read from the Modbus registers
        (PR #14), if not already done: add `FB_IntToMode`; delete
        `FB_BitsToMode`, `FB_ModeToBits`, `FB_FaultToBits` and
        `FB_OutputMux`; rename the `FB_FaultResetHandler` input
        `BitsStable` to `CodeStable`.
  - [ ] Rename `FB_PositionCommandGate` to `FB_PositionPathShaper`.
  - [ ] Add `FB_Probe` (`Stage` is a `VAR_OUTPUT`).
  - [ ] Update the declarations of `FB_GoHome` (`StopDeceleration`
        input), `FB_SafetyMonitor` (state-property inputs),
        `FB_AnalogInputFilter` and `FB_ModeDecoder`.
  - [ ] Remove the `VAR_OUTPUT` members nothing reads, and declare the
        ones the block still uses as `VAR` locals (types and initial values
        as in the block's header):
    - [ ] `FB_DigitalInputFilter`: `RisingEdge`, `FallingEdge`, `Stable`,
          `RawPassthrough`; delete the local `bLastFiltered`.
    - [ ] `FB_GoHome`: `Active`, `AtPosition`. `FB_HomeLimit`: `Active`.
    - [ ] `FB_HandshakeManager`: `HandshakeActive`, `HandshakeTimeout_Q`.
    - [ ] `FB_ModeDecoder`: `ModeChanged`; `ModeValue` becomes the local
          `nModeValue : INT`.
    - [ ] `FB_PistonExitGuard`: `ApproachingLimit`.
    - [ ] `FB_PositionPathShaper`: `RateLimiting`, `TetherHold`;
          `ParamError` becomes the local `bParamError : BOOL`.
    - [ ] `FB_PressureController`: `Error`, `VelLimit`, `Saturated`,
          `ParamError` become the locals `rError : LREAL`,
          `rVelLimit : LREAL`, `bSaturated : BOOL`, `bParamError : BOOL`.
    - [ ] `FB_PressureInput`: `LoopMa`, `OutOfRange`, `ParamError` become
          the locals `rLoopMa : LREAL`, `bOutOfRange : BOOL`,
          `bParamError : BOOL`.
    - [ ] `FB_PressureSchedule`: `Lag` becomes the local `rLag : LREAL`.
    - [ ] `FB_Probe`: `StartTail` and `Xfer`; `State` becomes the local
          `eState : E_ProbeState := PROBE_RECORDING`.
    - [ ] `FB_SafetyMonitor`: `FaultDrive`, `FaultComm` and
          `FaultPistonExit` (used as the inputs directly); `FaultLimitSwitch`,
          `FaultPosition` and `FaultPressureSensor` become the locals
          `bFaultLimitSwitch`, `bFaultPosition` and `bFaultPressureSensor`
          (BOOL).
- [ ] `PRG_Main` locals:
  - [ ] PR #14, if not already done: remove `fbFilterModeBit0`-`2`,
        `bFilteredModeBit0`-`2` and `fbOutputMux`; add
        `fbConfModeToInt : FB_ModeToInt`.
  - [ ] Add the state-property BOOLs, `rPosShaperVelMax`, `fbProbe` and
        `nProbeWord`.
  - [ ] Rename `fbPosGate` to `fbPosShaper` and `bPosGateActive` to
        `bPosShaperActive`.
- [ ] Delete `PRG_PressureProbe` from the project and the task.
- [ ] Pressure gain schedule:
  - [ ] Add `FB_PressureSchedule`, and `IntegScale` (initial value 1.0) to
        `FB_PressureController`.
  - [ ] `PRG_Main` locals: `fbPressureSched`, `rAnalogFilterTime`,
        `rSchedSaline`, `rSchedOxygenRatio`, `bPressureSchedActive`,
        `bPressureSchedUsed`, `rPressureKp`, `rPressureKi`,
        `rPressureVelMax`, `rPressureStiffness` and `rPressureIntegScale`
        (initial value 1.0).
  - [ ] Globals: `G_mbSalineVolume`, `G_mbOxygenRatio`,
        `G_sysSalineVolume`, `G_sysOxygenRatio` and the
        `G_cfgPressureSched*` settings with `G_cfgPressureScheduleEnable`.
- [ ] Globals:
  - [ ] PR #14, if not already done: add `G_sysConfirmedModeCode : UINT`.
        First confirm the LD POU reads none of `G_diModeBit0`-`2`,
        `G_diMotionEnable`, `G_diFaultReset` and `G_doModeConfBit0`-`2`
        (`%IX0.0`-`0.4`, `%QX0.0`-`0.2`), then remove them.
  - [ ] Remove `G_cmdMoveVelocity`, `G_staMoveVelocity`,
        `G_sysCycleCount`, `G_posModeEntryPosition`,
        `G_posExitGuardBoundary`, `G_sysMotionActive`, `G_sysInPosition`,
        `G_sysAtVelocity`, `G_sysAtPosLimitHigh`, `G_sysAtPosLimitLow`,
        `G_sysStateEntryTime`, `G_sysTimeInState`, `G_cfgVelLimitNormal`,
        `G_cfgDirectCtrlDecel` and `G_cfgGoHomeStopDecel`.
  - [ ] Rename `G_cfgPosGateAccelMax` to `G_cfgPosShaperAccelMax` and
        `G_cfgPosGateMaxDeviation` to `G_cfgPosShaperMaxDeviation`.
  - [ ] Set `G_cfgVelLimitMax` to 6.0.
  - [ ] Replace `G_cfgGoHomeAccel` and `G_cfgHomeLimAccel` by one
        `G_cfgMoveAccel : LREAL := 5.0`.
  - [ ] Replace `G_cfgDebounceTimeMotion` and `G_cfgDebounceTimeFault` by
        `G_cfgDebounceTimeCommand : TIME := T#20MS`, and
        `G_cfgDebounceTimeOvertravelNeg` and
        `G_cfgDebounceTimeOvertravelPos` by
        `G_cfgDebounceTimeOvertravel : TIME := T#10MS`.
  - [ ] Delete `G_doPerformanceStatus`, `G_doFaultActive`,
        `G_doHomingComplete` and `G_doInMotion` (`%QX0.3`-`%QX0.6`) and
        add `G_sysPerformanceStatus`, `G_sysFaultActive`,
        `G_sysHomingComplete` and `G_sysInMotion` as `BOOL := FALSE` with no
        address; the output points `MO1_DO_03`-`MO1_DO_06` are no longer
        written. Check the LD POU does not use the old four.
  - [ ] `G_cfgGearRatio` and `G_cfgLeadScrewPitch` stay; their comments now
        call them reference copies of the Hardware Configuration scaling.
  - [ ] Add `G_cfgStopDecel`, `G_cfgProbeEnable`, `G_cfgProbeFillPerScan`,
        `G_mbPositionSpeed`, `G_sysPositionSpeed`, the `G_mbProbe*`
        globals and the `G_sysProbe*` globals.
- [ ] Modbus groups in Hardware Configuration: holding registers to HR114 (the protocol uses HR1-12),
      input registers to IR524. Keep the groups to what is used: a
      3410-register input group (the old step probe's) made the protocol
      exchange 50 ms (p99 70) instead of 20 (p99 30), programs loaded or
      not, and tripped the slave's 300 ms watchdog during motion.
- [ ] Enable the cyclic task's watchdog, at the 4 ms interval. The 2 ms
      task overran for weeks (about 2.5 ms of work a scan) and ran on
      every second motion cycle with nothing reported.
- [ ] Export the GVL from MotionWorks and diff it against
      `slave-src/GVL/GlobalVariables_Reference.st`. The soft limit, the
      cylinder entry, the velocities and the scan were found to differ
      from the machine one at a time; one diff settles the rest.

## Lab PC

- [ ] Rerun `buildShell`. The Modbus Client Write block becomes 12
      registers (HR1-12) and the `PositionSpeed`, `SalineVolume` and
      `OxygenRatio` Constant blocks are added.
- [ ] Relaunch the operator app and check the new "Move speed" field.
      The app does not write `SalineVolume` or `OxygenRatio`; HR9-12 stay
      at the `buildShell` defaults (`masterConfig.m`: 343 mL, 1.0) until
      the app's oxygen:saline field is wired to the shell.
- [ ] Run `tests\run_all.bat` under MATLAB, including the MEX build of the
      compiled simulated slave. It has only been run under Octave. The
      Python parity tests (`stTargetParity.py`, `plantParity.py`) also need
      a C compiler as `CC` (for example `set CC=gcc` with MinGW-w64).
      `charFitsSynthetic`, `frictionRunSim`, `benchProbeSim` and
      `modbusLinkTiming` seed with the legacy `randn('seed', n)` (and
      `rand('seed', n)`), which MATLAB accepts with a warning; `rng` is
      the replacement if it objects.

## Checks on the machine

- [ ] Compile in MotionWorks and confirm it accepts:
  - [ ] the 128 KB ring array inside `FB_Probe`;
  - [ ] array indexing with UDINT expressions;
  - [ ] `SHR` on a `DWORD`;
  - [ ] `INT_TO_WORD`, `DINT_TO_DWORD` and `DWORD_TO_WORD` wrapping on
        negative values.

  If the ring array is refused, shrink `cDepth` and its array type
  together.
- [ ] Confirm `Y_DirectControl` accepts a Velocity of 0 in PositionMode,
      which it gets when the master sets a position speed of 0.
- [ ] Measure the protocol exchange time with the enlarged Modbus groups
      (`bench/modbus`).
- [ ] Measure the task's execution time to see what the scan is spent on.
      It is expected to be mostly the LD motion blocks and the Modbus
      groups, but this has not been measured.
- [ ] Run a bench with the probe and check the timing of a command that
      waits for the probe's arm acknowledgement.
- [ ] When reading delays from probe captures, allow one scan: the probe
      logs the velocity feedback one scan older than the command.
- [ ] Run `bench/modbus/modbusCountSweep.m` with a Wireshark capture and
      analyse it with `modbusCaptureTiming.py` (README "Register count").
      Fill the table there and set `perRegister` in
      `slave-sim/modbusLinkConfig.m`. Writes need a block of holding
      registers inside the group that the program does not use, passed as
      `writeStart`. If the same-tick share falls with the register count,
      make `pNextTick` depend on the count too. Before running it, widen
      the sweep's protected range: `modbusCountSweep.m` refuses HR1-8 and
      HR111-114, but the protocol now uses HR1-12, so a `writeStart` of
      9 to 12 would overwrite the saline and oxygen registers.
- [ ] Repeat the characterisation (`bench/characterisation/README.md`) on
      the calibrated pressure input (`G_cfgPressureShuntOhms` 421.4) at
      the 4 ms scan:
  - [ ] Fill and purge longer than the 26 October runs and weigh the
        fill; one of those fills held enough gas to read 0.42 GPa.
  - [ ] Bulk modulus with the proximal valve shut: `K300` three times,
        then a second fill level so `charBulkModulus` separates the liquid
        from the series compliance.
  - [ ] Hose stiffness: the same path with the valve open, then again
        after a thorough hose purge.
  - [ ] Drive response: the open-chamber velocity steps (`s1`, `s2`)
        again now that `FilterTimeConstant` is wired, and the closed,
        liquid-filled `lag_wet` path with the probe armed for the lag at
        the piston.
  - [ ] Optionally a Cd check at two or three held pressures.
- [ ] Carry the results into `models/params.py` (`K`, `Cd`, `t_dead`,
      `tauV`) and rerun the studies (`study_robust`, `study_compare`,
      `study_delivery`). Until then `K` 0.72 GPa and `Cd` 0.85 are the
      26 October fits as logged; refitted on the calibrated scale
      (`opts.logShuntOhms = 434`) they are 0.74 GPa and 0.83.
- [ ] Pressure input calibration (`bench/pressure/README.md` section 6):
      it rests on a hand-held meter's mA range, about 1 %. Repeat it with
      a loop calibrator, and measure the 4.7 uF capacitor's leakage out of
      circuit (the input's load in parallel with the shunt fitted as about
      15 kohm).
- [ ] Verify every assumed position against the machine, in the homed
      frame, and correct each copy:
  - [ ] Positive overtravel switch trip point (just past 367 mm, not
        measured; trip it with a limit fault). The simulator assumes
        370 mm (`slave-sim/slaveSimConfig.m` `switchPosPos`) with a 3 mm
        hard stop beyond it (`hardStopMargin`).
  - [ ] Negative overtravel switch, assumed at -5 mm
        (`G_cfgHomeLimRetractDist`, `switchNegPos`), and
        `G_cfgPosSoftLimitMin` (-2.5 mm) between it and zero.
  - [ ] End of travel, `posEOT` (367 mm, `master-src/logic/masterConfig.m`),
        which sets the displayed position and volume.
  - [ ] Cylinder entry, `G_cfgPosCylinderEntry` (176 mm). The simulator's
        `chamberContactPos` is still 200 mm and
        `bench/friction/frictionBenchConfig.m` `exitGuardPos` still
        assumes 200 + 20 = 220 mm.
  - [ ] Margins between the limits. `posMax` and `G_cfgPosSoftLimitMax`
        are both 367 mm and the switch sits just past them, so a jog the
        master stops at `posMax` can overshoot into `FAULT_POSITION` or
        the switch. Leave room for the arrival tolerance (0.5 mm) and one
        exchange of travel at full speed (0.3 mm at 6 mm/s), then set
        `posMax` < `G_cfgPosSoftLimitMax` < the switch.
- [ ] Measure what the models still assume (`design/pressure-loop/README.md`,
      "Open questions"): the dissolution time constant `tauDiss`, the lag
      at the piston, and how much of the hose's compliance is wall and how
      much trapped air.
- [ ] Stopping distance under load at 8 to 10 MPa with scan-rate captures
      (`bench/characterisation/README.md` test 4, `charStopPath` with
      `levels_MPa [8 10]`, speeds 0.25 and 0.5 mm/s, probe armed for the
      stops). The 50 ms log gave 0.031 s times the speed from 2 to 10 MPa
      with no load dependence; the captures confirm it at the scan rate
      and settle the delay term. Then set the slave's allowance from the
      fit (`charStopDistance`: `r.T0_s`, `r.D_mm_s2`). Today the bench
      compares against 0.05 s (`charStopDistance` `opts.lag_s`, not a
      GVL setting) and `G_cfgPressureDecel` 50 mm/s², conservative by
      three to four times at 5 mm/s; the slave's unscheduled stop law has
      no delay term, and `FB_PressureSchedule` computes its own `Lag`.
- [ ] From the one-pass logs already taken (`bench/captures/stops_char_*`):
      the friction at speed under load (`frictionFit` on the legs) and the
      stiffness against pressure (`charBulkModulus` with `chargeRatio`).
      Neither fit has been run on them.
- [ ] Confirm that `Y_DirectControl`'s `Acceleration` input limits the
      drive in VelocityTLMode: velocity steps at 1 and at 100 mm/s². The
      bench saw every step complete in 60 ms whatever its size
      (`bench/characterisation/README.md` section 8), which says it does
      not.
- [ ] Hose procedure: the proximal valve stays shut during the
      compression and opens after mixing; a switched bypass orifice is
      deferred. Opening the proximal valve at 10 MPa drops the chamber to
      6.4 MPa in one scan and degasses any charge above about 1.8:1, so
      until the bypass is fitted, keep oxygen runs at or below that charge
      or accept the degassed slug. The bypass, when fitted, is a metering
      orifice of 0.3 mm or less (Cv about 0.003) opened ahead of the ball
      valve (`study_hose_opening.py`). The purge must carry the slug that
      degassed entering the hose, 2 to 3 mL behind the 29 mL of virgin
      saline.
- [ ] The hose model's inputs that only the procedure can give: whether
      the whole 29 mL is purged, the pause between purge and delivery,
      whether the tube drains in it (`Scenario.tube_refills` in
      `scenarios.py`), and how fast the distal valve is shut
      (`StudyParams.Tclose`).

## Simulator and tests

- [ ] Simplifications of the simulated slave to keep in mind or remove:
  - [ ] Position moves, jog and stop prescribe the piston's velocity; the
        drive's dead time and lag apply only in `Y_DirectControl`'s
        velocity and torque modes.
  - [ ] The chamber always holds a run's starting contents (the 1:1 charge
        at the supply pressure, hose open); there is no fill or vent.
  - [ ] Before the cylinder entry the pressure reads zero while the plant
        keeps its contents.
- [ ] Optional: a Modbus TCP server in front of the simulated slave, so the
      bench scripts and the Simulink shell can run against it over the
      network.

## Documentation

- [ ] `docs/slave/reference/TimingSpecification.md` §12: the Range column
      for `G_cfgBrakeEngageDelay` and `G_cfgBrakeDisengageDelay`
      (100-1000 ms) was widened only so the defaults fall inside it. Set
      the ranges from the brake's datasheet.
- [ ] Capture directory: `charRun` saves to `bench/common/captures` by
      default and `bench/characterisation/README.md` reads from
      `../common/captures`, but the committed logs are in
      `bench/captures`. Point both at one place.

## Open decisions

- [ ] The scheduled pressure controller is ported (`FB_PressureSchedule`
      behind `G_cfgPressureScheduleEnable`, off by default; the stiffness
      bound taken at the present and the target pressure; lockstep with
      the Python reference to 1e-9; the saline volume and charge arrive in
      HR9-12). Commission it in this order: the stiffness estimate against
      the measured slope on the characterisation bench; a water-only
      delivery with the full valve sequence, comparing the opening dip and
      the purge-closing rise with the model's 0.9 and 1.1 MPa at the
      11 rad/s crossover; oxygen at 1:1, then 2:1. The GVL gains let the
      purge closing reach 12.3 MPa in every plausible case; until the
      schedule is on, stop the piston before shutting the distal valve.
- [ ] An on-line stiffness estimate (dP against dx while the piston moves)
      as a monitor beside the configured bound, logging its value and
      flagging disagreement, which is the schedule's one failure mode (a
      wrong configuration). Letting it raise the gain is a later,
      reversible step; model predictive control was considered and set
      aside (scan rate, uncertain gas model, no sensor for the valve
      events).
- [ ] A transducer in the tube downstream of the distal valve would give
      the outflow and the valve state (the flow feedforward the studies
      carry as a bound, limited by the loop's 50 ms); a transducer at the
      distal end of the hose gives no usable lead (the pressure wave
      crosses the hose in 2.8 ms). Decide whether the extra input is worth
      it.
- [ ] Decide the schedule's default switch and crossover once it has run
      on the machine: `G_cfgPressureSchedCrossover` 11 rad/s overshoots a
      gas-dominated chamber by 1 MPa with the measured drive lag, 6 rad/s
      by 0.06 MPa (the figures are in the message of commit b457360;
      `design/pressure-loop/test_model.py` checks only the 6 rad/s case,
      overshoot below 0.3 MPa).
- [ ] The probe's record index wraps after 2^32 scans (about 200 days at
      4 ms of uninterrupted running), and the master does not handle the
      wrap.
