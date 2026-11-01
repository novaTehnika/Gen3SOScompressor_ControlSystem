# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21). Grouped by where they have to be done.

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
      compiled simulated slave. It has only been run under Octave.

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

## Documentation

- [ ] Write the pin tables of the other LD blocks in
      `docs/slave/ld/README.md`. Their screenshots are already in that
      directory.

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
- [ ] The probe's record index wraps after 2^32 scans (about 200 days at
      4 ms of uninterrupted running), and the master does not handle the
      wrap.
