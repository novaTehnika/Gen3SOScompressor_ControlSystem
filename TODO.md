# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21), the simulator, models and link timing work (PR #22), and
the pressure-loop design, bench and scheduled-controller work (PR #17 and
the `sched-pressure-controller` branch). Grouped by where they have to be
done.

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
      input registers to IR524. Keep the groups to what is used: a
      3410-register input group (the old step probe's) made the protocol
      exchange 50 ms (p99 70) instead of 20 (p99 30), programs loaded or
      not, and tripped the slave's 300 ms watchdog during motion.
- [ ] Enable the cyclic task's watchdog, at the 4 ms interval. The 2 ms
      task overran for weeks (about 2.5 ms of work a scan) and ran on
      every second motion cycle with nothing reported.
- [ ] LD POU, `Y_DirectControl_1`: wire `FilterTimeConstant` from
      `G_cmdDirectControl.FilterTimeConstant` (it is hard-wired to
      `LREAL#0.05`) and `Error` to `G_staDirectControl.Error` (it is wired
      to `.Active`), as `docs/slave/ld/README.md` records.
- [ ] Scheduled pressure controller (`sched-pressure-controller`): declare
      `FB_PressureSchedule`'s variables, `FB_PressureController.IntegScale`
      (initial 1.0), the new `PRG_Main` locals and the `G_cfgPressureSched*`
      globals when that branch is loaded.
- [ ] Export the GVL from MotionWorks and diff it against
      `slave-src/GVL/GlobalVariables_Reference.st`. The soft limit, the
      cylinder entry, the velocities and the scan were found to differ
      from the machine one at a time; one diff settles the rest.

## Lab PC

- [ ] Rerun `buildShell`. The Modbus Client Write block becomes 8
      registers and a `PositionSpeed` Constant block is added.
- [ ] Relaunch the operator app and check the new "Move speed" field.
- [ ] Run `tests\run_all.bat` under MATLAB, including the MEX build of the
      compiled simulated slave. It has only been run under Octave. The
      Python parity tests (`stTargetParity.py`, `plantParity.py`) also need
      a C compiler as `CC` (for example `set CC=gcc` with MinGW-w64).
      `charFitsSynthetic` and `frictionRunSim` seed with the legacy
      `randn('seed', n)`, which MATLAB accepts with a warning; `rng` is
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
      make `pNextTick` depend on the count too.
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
      fit (`charStopDistance`: `r.T0_s`, `r.D_mm_s2`; today 0.05 s and
      50 mm/s², conservative by three to four times at 5 mm/s). The
      bench's capture reader expects the old step probe's record; adapt
      `charRun`'s 'stops' probe mode and `charStopDistance` to `FB_Probe`.
- [ ] From the one-pass logs already taken (`bench/captures/stops_char_*`):
      the friction at speed under load (`frictionFit` on the legs) and the
      stiffness against pressure (`charBulkModulus` with `chargeRatio`).
      Neither fit has been run on them.
- [ ] Confirm that `Y_DirectControl`'s `Acceleration` input limits the
      drive in VelocityTLMode: velocity steps at 1 and at 100 mm/s². The
      bench saw the command followed within 30 ms whatever the step, which
      says it does not.
- [ ] Before the next oxygen run, choose the hose procedure. Opening the
      proximal valve at 10 MPa drops the chamber to 6.4 MPa in one scan
      and degasses any charge above about 1.8:1. Either open it before
      the compression, or fit a bypass metering orifice of 0.3 mm or less
      (Cv about 0.003) that is opened ahead of the ball valve; the model
      sizes both (`study_hose_opening.py`). The purge must then carry the
      slug that degassed entering the hose, 2 to 3 mL behind the 29 mL of
      virgin saline.
- [ ] The hose model's inputs that only the procedure can give: whether
      the whole 29 mL is purged, the pause between purge and delivery,
      whether the tube drains in it (`tube_refills`), and how fast the
      distal valve is shut (`Params.Tclose`).

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
- [ ] The scheduled pressure controller is ported on
      `sched-pressure-controller` (`FB_PressureSchedule` behind
      `G_cfgPressureScheduleEnable`, off by default; the stiffness bound
      taken at the present and the target pressure; lockstep with the
      Python reference to 1e-9; 24 commits not on main). Review and merge
      it, then commission it in this order: the stiffness estimate against
      the measured slope on the characterisation bench; a water-only
      delivery with the full valve sequence, comparing the opening dip and
      the purge-closing peak with the model's 0.8 and 1.0 MPa; oxygen at
      1:1, then 2:1. The GVL gains let the purge closing reach 12.2 MPa in
      every plausible case; until the schedule is on, stop the piston
      before shutting the distal valve.
- [ ] How the slave learns the recipe for the schedule: a GVL setting the
      operator edits per batch (enough to commission), or a holding
      register from the master (a protocol change: register map, protocol
      guide, master logic and operator app together).
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
- [ ] The probe's record index wraps after 2^32 scans (about 200 days at
      4 ms of uninterrupted running), and the master does not handle the
      wrap.
