# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21) and the simulator, models and link timing work (PR #22).
Grouped by where they have to be done.

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
  - [ ] Rename `FB_PositionCommandGate` to `FB_PositionPathShaper`.
  - [ ] Add `FB_Probe` (`Stage` is a `VAR_OUTPUT`).
  - [ ] Update the declarations of `FB_GoHome` (`StopDeceleration`
        input), `FB_SafetyMonitor` (state-property inputs),
        `FB_AnalogInputFilter` and `FB_ModeDecoder`.
- [ ] `PRG_Main` locals:
  - [ ] Add the state-property BOOLs, `rPosShaperVelMax`, `fbProbe` and
        `nProbeWord`.
  - [ ] Rename `fbPosGate` to `fbPosShaper` and `bPosGateActive` to
        `bPosShaperActive`.
- [ ] `PRG_PressureProbe`: add the local `nWindow : INT`.
- [ ] Globals:
  - [ ] Remove `G_cmdMoveVelocity`, `G_staMoveVelocity`,
        `G_sysCycleCount`, `G_posModeEntryPosition`,
        `G_posExitGuardBoundary`, `G_sysMotionActive`, `G_sysInPosition`,
        `G_sysAtVelocity`, `G_sysAtPosLimitHigh`, `G_sysAtPosLimitLow`,
        `G_sysStateEntryTime`, `G_sysTimeInState`, `G_cfgVelLimitNormal`,
        `G_cfgDirectCtrlDecel` and `G_cfgGoHomeStopDecel`.
  - [ ] Rename `G_cfgPosGateAccelMax` to `G_cfgPosShaperAccelMax` and
        `G_cfgPosGateMaxDeviation` to `G_cfgPosShaperMaxDeviation`.
  - [ ] Set `G_cfgVelLimitMax` to 6.0.
  - [ ] Add `G_cfgStopDecel`, `G_cfgProbeEnable`, `G_cfgProbeFillPerScan`,
        `G_mbPositionSpeed`, `G_sysPositionSpeed`, the `G_mbProbe*`
        globals and the `G_sysProbe*` globals.
- [ ] Modbus groups in Hardware Configuration: holding registers to HR114,
      input registers to IR524.

## Lab PC

- [ ] Rerun `buildShell`. The Modbus Client Write block becomes 8
      registers and a `PositionSpeed` Constant block is added.
- [ ] Relaunch the operator app and check the new "Move speed" field.
- [ ] Run `tests\run_all.bat` under MATLAB, including the MEX build of the
      compiled simulated slave. It has only been run under Octave. The
      Python parity tests (`stTargetParity.py`, `plantParity.py`) also need
      a C compiler as `CC` (for example `set CC=gcc` with MinGW-w64).

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
      make `pNextTick` depend on the count too.
- [ ] Measure what the models still assume (`design/pressure-loop/README.md`,
      "Open questions"): the dissolution time constant `tauDiss`, the lag
      at the piston, and how much of the hose's compliance is wall and how
      much trapped air.

## Simulator and tests

- [ ] The link timing model (`slave-sim/modbusLink.m`) leaves out what the
      Simulink side adds: the master's step overrunning while a Modbus call
      blocks, the exchange that arrives a period late (about one a minute
      at 50 ms on the bench), and a lost link stopping the Simulink model
      (bench case D). `closedLoopModbusStep` assumes every exchange
      completes.
- [ ] Speed up `tests/fakeProbeLink.m`: `logScan` reads the whole
      `S.sl.G` every scan, which on the compiled slave builds a snapshot
      each time. `frictionRunSim` and `charRunSim` went from about 55 s and
      88 s to 135 s and 165 s with it. Read only the fields it needs.
- [ ] `PRG_PressureProbe` is emulated by hand in `tests/fakeProbeLink.m`,
      not translated from its ST. Translate the bench slave programs with
      the c target, or drop the emulation if the program is retired (see
      Open decisions).
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

- [ ] Write the pin tables of the other LD blocks in
      `docs/slave/ld/README.md`. Their screenshots are already in that
      directory.
- [ ] Redraw `docs/slave/diagrams/MainStateMachine.svg` from `PRG_Main`.
      It shows `ST_ENCODER_CHECK`, `ST_REQUIRE_ABS_HOME` and
      `ST_FAULT_RECOVERY`, which `E_SystemState` does not have, and lacks
      `ST_PRESSURE_CTRL`, `ST_RECOVERY`, `ST_RECOVERY_POSITION` and
      `ST_RECOVERY_VELOCITY`. `docs/master/StateTransitionDiagram.md` is
      the current text version.
- [ ] `docs/slave/development/ConfigurationGuide.md` says it covers every
      `G_cfg*` global but has no rows for `G_cfgCommTimeout`,
      `G_cfgHoldPositionTimeout`, `G_cfgTorqueStallThreshold`,
      `G_cfgGearRatio`, `G_cfgLeadScrewPitch` and the pressure
      transducer and loop settings (`G_cfgPressureShuntOhms`, `ZeroMa`,
      `SpanMa`, `RangeMin`, `RangeMax`, `SensorMinMa`, `SensorMaxMa`,
      `SensorFaultTime`, `Kp`, `Ki`, `Decel`, `StopStiffness`,
      `StopVelMin`, `TorqueLimit`, `TargetMax`, `InBand`). Take defaults
      and descriptions from the GVL.
- [ ] `docs/slave/reference/HomingSequenceGuide.md`: the Mode 110
      parameter table gives `G_cfgHomeLimFastApproachVel` as 5.0; the GVL
      has 6.0.
- [ ] `docs/slave/reference/TimingSpecification.md` §7: the Mode 110 table
      works from a 5 mm/s fast approach over about 310 mm. With the GVL's
      6 mm/s and `G_cfgPosSoftLimitMax` of 367 mm the worst-case approach
      is about 372 mm (still about 63 s). Update the stated speed and
      distance.
- [ ] `docs/slave/reference/TimingSpecification.md` §12: the Range column
      for `G_cfgBrakeEngageDelay`, `G_cfgBrakeDisengageDelay` (100-1000 ms)
      and `G_cfgHomingTimeout` (30-180 s) was widened only so the defaults
      fall inside it. Set ranges from the brake's datasheet and the
      longest homing travel.
- [ ] `docs/master/FaultCodeReference.md` (`FAULT_POSITION` and
      `FAULT_PISTON_EXIT` sections) names `G_posSoftLimitMin` and
      `G_posSoftLimitMax`; the globals are `G_cfgPosSoftLimitMin` and
      `G_cfgPosSoftLimitMax`. The piston exit boundary is
      `G_cfgPosCylinderEntry + G_cfgPosExitGuardMargin`, not the soft
      limit.

## Open decisions

- [ ] Merge `G_cfgGoHomeAccel` and `G_cfgHomeLimAccel` (both 5 mm/s², the
      same kind of move) into one `G_cfgMoveAccel`.
- [ ] `G_cfgGearRatio` and `G_cfgLeadScrewPitch` duplicate the Hardware
      Configuration scaling and only the benches read them: label them as
      bench copies or move them to the bench config.
- [ ] Share one debounce setting each between the two overtravel inputs
      (both 10 ms) and between MotionEnable and FaultReset (both 20 ms).
- [ ] About 25 function block outputs are read by nothing outside their
      block (diagnostics). Keep or remove.
- [ ] Retire `PRG_PressureProbe`: the main probe records the raw pressure
      voltage every scan, so the noise statistics could be computed on the
      master.
- [ ] On the plant model the slave's pressure loop overshoots a 10 MPa
      target by about 0.6 MPa and settles in about 13 s
      (`tests/slavePressureMode.m`), as `design/pressure-loop/README.md`
      reports for the GVL loop. In the studies `ScheduledSlaveController`
      arrives in 2.4 s without overshooting the reading. Decide whether to
      port it to the slave.
- [ ] The probe's record index wraps after 2^32 scans (about 200 days at
      4 ms of uninterrupted running), and the master does not handle the
      wrap.
