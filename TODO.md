# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21), the simulator, models and link timing work (PR #22), and
the pressure-loop design, bench and scheduled-controller work (PR #17 and
the scheduled pressure controller), and the pressure input calibration.
Grouped by where they have to be done. Settled: the mechanical pressure
ceiling is 12 MPa; the saline volume and oxygen charge reach the slave in
holding registers.

## MotionWorks project

The project is built from `slave-src` and runs on the controller. Left
to confirm in it:

- [ ] LD POU: delete the `MC_MoveVelocity` instance if it is still
      there (no recaptured screenshot shows it), with `G_cmdMoveVelocity`,
      `G_staMoveVelocity`, their two types and `G_sysCycleCount`.
- [ ] Declarations this branch changed after the project was built from
      `main`; mirror them (the `.st` headers and the GVL reference are
      the reference for each):
  - [ ] `G_cfgMoveAccel : LREAL := 5.0` replaces `G_cfgGoHomeAccel` and
        `G_cfgHomeLimAccel` (on `main` `G_cfgGoHomeAccel` reads 510.0,
        which looks like a slip for 5.0).
  - [ ] `G_cfgDebounceTimeCommand : TIME := T#20MS` replaces
        `G_cfgDebounceTimeMotion` and `G_cfgDebounceTimeFault`;
        `G_cfgDebounceTimeOvertravel : TIME := T#10MS` replaces the
        `...OvertravelNeg` and `...OvertravelPos` pair.
  - [ ] `G_sysPerformanceStatus`, `G_sysFaultActive`,
        `G_sysHomingComplete` and `G_sysInMotion` (`BOOL := FALSE`, no
        address) replace `G_doPerformanceStatus`, `G_doFaultActive`,
        `G_doHomingComplete` and `G_doInMotion`; `MO1_DO_03`-`06` are no
        longer written.
  - [ ] Block outputs removed, or turned into `VAR` locals, as the
        headers now declare: `FB_DigitalInputFilter` (`RisingEdge`,
        `FallingEdge`, `Stable`, `RawPassthrough`), `FB_GoHome` (`Active`,
        `AtPosition`), `FB_HomeLimit` (`Active`), `FB_HandshakeManager`
        (`HandshakeActive`, `HandshakeTimeout_Q`), `FB_ModeDecoder`
        (`ModeChanged`; `ModeValue` local), `FB_PistonExitGuard`
        (`ApproachingLimit`), `FB_PositionPathShaper` (`RateLimiting`,
        `TetherHold`; `ParamError` local), `FB_PressureController` and
        `FB_PressureInput` (their `Error`/`VelLimit`/`Saturated`/
        `LoopMa`/`OutOfRange`/`ParamError` as locals),
        `FB_PressureSchedule` (`Lag` local), `FB_SafetyMonitor` (the six
        `Fault*`; three as locals).
  - [ ] `G_cfgGearRatio` and `G_cfgLeadScrewPitch` keep their values;
        only their comments changed.
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
- [ ] Pull the cable with the shell running and note when the model stops.
      `master-src/README.md` says it retries and stops about 20 s later
      when Windows TCP gives up; bench case D (`bench/modbus/README.md`)
      says the Read block errors on the first exchange after the drop. The
      link model (`slave-sim/modbusLinkConfig.m` `lostLinkErrorDelay`)
      uses the bench figure, 0; correct whichever is wrong.

## Checks on the machine

- [ ] Confirm `Y_DirectControl` accepts a Velocity of 0 in PositionMode,
      which it gets when the master sets a position speed of 0.
- [ ] Measure the protocol exchange time with the enlarged Modbus groups
      (`bench/modbus`).
- [ ] Measure the task's execution time to see what the scan is spent on.
      It is expected to be mostly the LD motion blocks and the Modbus
      groups, but this has not been measured.
- [ ] Run a bench with the probe and check the timing of a command that
      waits for the probe's arm acknowledgement.
- [ ] Run `bench/modbus/modbusCountSweep.m` with a Wireshark capture and
      analyse it with `modbusCaptureTiming.py` (README "Register count").
      Fill the table there and set `perRegister` in
      `slave-sim/modbusLinkConfig.m`. Writes need a block of holding
      registers inside the group that the program does not use, passed as
      `writeStart`. If the same-tick share falls with the register count,
      make `pNextTick` depend on the count too.
- [ ] Repeat the characterisation (`bench/characterisation/README.md`) on
      the calibrated pressure input (`G_cfgPressureShuntOhms` 421.4) at
      the 4 ms scan. `bench/MeasurementPlan.md` sequences every model
      measurement, these included, into three sessions on two fills:
  - [ ] Fill, weigh the fill against the geometry, and purge until they
        agree to a few mL. The October fits' gas-law term (0.65 mL at
        the ramps' mean pressure, about 40 mL at atmosphere as gas) is
        more air than the machine holds, so its origin is open: the
        weighed fill, the hold at 10 MPa (gas dissolves, a compliance
        does not) and the second fill level decide. If it persists with
        no gas, give `charBulkModulus` a pressure-dependent series term
        in place of the gas law. The transducer passage's air (`V_air`,
        about 0.1 mL by the geometry) is the floor of the gas term; note
        the passage's bore and length.
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
      (`opts.logShuntOhms = 434`) they are 0.74 GPa and 0.83. Run the
      scenarios with the settled hose procedure (`hose_before` False): the
      design README's results passage ("Every result below is for the
      first remedy") still reports the hose-before remedy; relabel it.
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
        `chamberContactPos` is still 200 mm.
  - [ ] Margins between the limits. `posMax` and `G_cfgPosSoftLimitMax`
        are both 367 mm and the switch sits just past them, so a jog the
        master stops at `posMax` can overshoot into `FAULT_POSITION` or
        the switch. Leave room for the arrival tolerance (0.5 mm) and one
        exchange of travel at full speed (0.35 mm at 7 mm/s), then set
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
