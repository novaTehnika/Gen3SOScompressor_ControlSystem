# TODO

Open items after the step probe, position speed, configuration and cleanup
work (PR #21), the simulator, models and link timing work (PR #22), and
the pressure-loop design and bench work (PR #17), the pressure input
calibration, and
the 5 November characterisation with its carry-over into the models, the
GVL and the design studies (PR #38).
Grouped by where they have to be done. Settled: the mechanical pressure
ceiling is 12 MPa; the fill (saline volume and oxygen charge) stays on the
master, in the procedure and the trace, and the protocol does not carry it.

## MotionWorks project

The project is built from `slave-src` and runs on the controller. The
build predates the step probe's 4096-record ring and the declarations
below, so rebuild with them (the `.st` headers and the GVL reference are
the reference for each) and confirm:

- [ ] Rebuild with the single pressure controller: delete the
      `FB_PressureSchedule` POU and its instance, every
      `G_cfgPressureSched*` and `G_cfgPressureScheduleEnable`, and
      `PRG_Main`'s schedule locals (`rSched*`, `bPressureSched*`,
      `rPressureKp`/`Ki`/`VelMax`/`Stiffness`/`IntegScale`); redeclare
      `FB_PressureController` from its header (the `Position` and floor
      inputs, the `Latched` and `FloorError` outputs, the two 64-sample
      rings of the new type `ARRAY64LREAL`, no `IntegScale`); add the ten
      `G_cfgPressureFloor*`; and set `G_cfgPressureKp` 1.5,
      `G_cfgPressureKi` 0.6 and `G_cfgPressureStopStiffness` 25.0. The
      gains were chosen with `G_cfgDirectCtrlFilterTime` at 0.02 s.
- [ ] Delete `G_mbSalineVolume` (`AT %MD3.483344`), `G_mbOxygenRatio`
      (`AT %MD3.483348`), `G_sysSalineVolume` and `G_sysOxygenRatio`,
      and the two fill lines of `PRG_Input`. With the probe the holding
      register group (FC#03,06,16) still spans HR1-114; without it the
      group ends at HR8. Rerun `buildShell` before shortening the group:
      a shell built earlier writes HR1-12, which a group ending at HR8
      does not hold. With the group at HR114 the order does not matter.
- [ ] Rebuild with the current `FB_Probe` (the 4096-record ring, 16 s;
      HR111-114, IR401-524) and `G_cfgProbeEnable`,
      `G_cfgProbeFillPerScan`.
- [ ] Exchange summary (IR21-34): add the `FB_ExchangeSummary` block;
      the eight `G_mbSum*` globals (`G_mbSumPressureMean AT %QD73832`,
      `G_mbSumPressureMin AT %QD73836`, `G_mbSumPressureMax AT %QD73840`,
      `G_mbSumVelocityMean AT %QD73844`, `G_mbSumTorqueMean AT %QD73848`,
      `G_mbSumPositionMean AT %QD73852`, all `REAL`; `G_mbSumScans AT
      %QW73856` and `G_mbSumSeq AT %QW73858`, `UINT`); the eight
      `G_sysSum*` globals (six `LREAL := 0.0`, two `UINT := UINT#0`); the
      `fbExchangeSummary : FB_ExchangeSummary` instance in `PRG_Main`; and
      the input register group (FC04) extended to at least IR34.
- [ ] LD POU: delete the `MC_MoveVelocity` instance if it is still
      there (no recaptured screenshot shows it), with `G_cmdMoveVelocity`,
      `G_staMoveVelocity`, their two types and `G_sysCycleCount`.
- [ ] Declarations changed after the project was built from `main`:
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
        `FB_SafetyMonitor` (the six
        `Fault*`; three as locals).
  - [ ] `G_cfgGearRatio` and `G_cfgLeadScrewPitch` keep their values;
        only their comments changed.
- [ ] Declaration for the operator speed (the header is the reference):
      `PRG_Main`: add `rOperatorSpeed` (`LREAL`).
- [ ] Declarations for the pressure ceiling, the end stop and the recovery
      fix: GVL `G_cfgPressureCeiling : LREAL := 12.0`,
      `G_cfgPressureCeilingHyst : LREAL := 0.5`,
      `G_cfgPressureEndMargin : LREAL := 1.0`,
      `G_sysPressureBlock : BOOL := FALSE`,
      `G_sysAtEndOfTravel : BOOL := FALSE`,
      `G_mbPressureBlock AT %QW73808 : UINT`,
      `G_mbAtEndOfTravel AT %QW73810 : UINT`; `PRG_Main` locals
      `bPosBlockReset` and `bPressureCmdBlocked` (`BOOL`).
- [ ] Declarations for the pressure-based piston exit guard (the header
      of `FB_PistonExitGuard.st` is the reference): redeclare the block
      (inputs `Pressure`, `SensorFault`, `PressureThreshold` and
      `RetreatMax` replace `ActualVelocity`, `ActualTorque`,
      `VelocityThreshold` and `TorqueThreshold`; outputs `Armed` and
      `InCylinder` replace `GuardActive` and `InDangerZone`; locals
      `bInBand`, `bArmedNow` and `rAnchor` replace `rGuardBoundary`); GVL:
      add `G_cfgExitGuardPressure : LREAL := 0.1` and
      `G_cfgExitGuardRetreat : LREAL := 0.5`, delete
      `G_cfgTorqueStallThreshold`.
- [ ] Enable the cyclic task's watchdog, at the 4 ms interval. The 2 ms
      task overran for weeks (about 2.5 ms of work a scan) and ran on
      every second motion cycle with nothing reported.
- [ ] Export the GVL from MotionWorks and diff it against
      `slave-src/GVL/GlobalVariables_Reference.st`. The soft limit, the
      cylinder entry, the velocities and the scan were found to differ
      from the machine one at a time; one diff settles the rest.

## Lab PC

- [ ] Rebuild the controller with the exchange summary (MotionWorks
      project, above) before `buildShell` or a bench: the shell's Read row
      becomes 34 registers, and a controller whose input group ends at
      IR20 answers it with an error and the model stops.
- [ ] Measure the per-register term of the link with `modbusCountSweep`
      at 20 and at 34 registers, and enter it in `modbusLinkConfig.m`.
- [ ] Relaunch the operator app and check the procedure panel (it replaces
      the old tabs): it fits the figure beside the readouts; the step
      list, heading, instruction, checklist and the parameter fields (two
      columns, labels on two lines) are legible; Start Procedure is
      enabled only connected, out of manual control, idle and with the
      slave link up; Manual Control is disabled while a procedure is
      active or paused; Confirm, Repeat, Pause/Resume and Abort act at
      once; a refused action (an unticked check, an out-of-range
      parameter) shows its reason; the step list scrolls to the current
      step (`ProcStepList` has no scroll call) and cannot be changed by
      clicking. Walk the seed procedure on the machine: the position,
      pressure and stop steps, the timer, the volume discharged and the
      held pressure readouts, E-STOP and a pulled cable pausing it
      (Resume refused until cleared), a slave fault pausing it (the reset
      dialog opens; Resume once reset), Abort mid-move,
      and Disconnect during a procedure.
- [ ] Rerun `buildShell`. The Modbus Client Write block becomes 8
      registers (HR1-8) and the `PositionSpeed` Constant block is added;
      `MasterStep` gains outputs 11 and 12, with the `PressureBlock` and
      `AtEndOfTravel` Gain blocks the app reads (an older shell makes
      every poll fail).
- [ ] Rebuild the shell again (`buildShell`) for the recorder interval,
      the live plot and the track step: the `TrackStep` MATLAB Function
      block (between `TargetPosition`/`TargetPressure` and `MasterStep`
      inputs 4 and 7), the `TrackTable`, `TrackMode`, `TrackSeq`,
      `ProcStep`, `SalineVolume` and `OxygenRatio` Constants, the
      `StatusDelay` Unit Delay, the `TrackIndex` Gain, `MasterStep` with
      10 inputs and `TraceStep` with 16 inputs (ring width 45). Check the
      build log for stack warnings on the 72001-value `trackTable`, and
      that `DefaultParameterBehavior` Tunable leaves the other Constants
      writable with `set_param` while the model runs.
- [ ] Track table update: check which path of `setTrackTable` works with
      the model running in Connected IO (the `Simulation` object's
      `setVariable` on the model-workspace `trackTable`, or the fallback of
      assigning the variable and updating the model), and time a full
      table (`cfg.trackTableLength` = 72001 values; the status line reports
      the first update). The slave faults after 300 ms without a write, so
      an update that blocks the app's thread near that long needs a smaller
      `cfg.trackMaxSeconds`, or the chunked table (a 1000-value constant
      copied into a persistent table over consecutive steps).
- [x] Controller process: the app starts the controller, Connect runs
      the shell, and resizing the windows and View > Side by side during
      motion give no link lost fault (`writeGapMax` 0.12 s).
- [x] Controller process, losing the app: ending the app's MATLAB from
      the Task Manager during a jog stops the jog at once (the controller
      finds the dead connection by a failing keepalive).
- [ ] Controller process, reattach and close: an app restarted within
      60 s of the kill connects to the running controller ("Controller
      ready") instead of starting another; without one, the model stops
      (safety relay off) and the controller's MATLAB ends after 60 s.
      Closing the app normally ends the controller within a few seconds.
- [ ] Controller process, track step (only once a procedure uses one;
      the SOP does not): run `experimentTemplate` and read the load time
      the controller notes ("Track table set in ..."). The load runs in
      the controller's MATLAB, so a load near 0.3 s during a kept
      pressure hold would fault the slave.
- [ ] Poll cost of the stream: with the live plot open for 10 minutes and a
      recording at 0.05 s, read `app.PollStats` (`mean`, `max`,
      `overruns`). The ring (512 x 45) is read on every poll and the plot
      is redrawn every `cfg.livePlotPeriod`; if the poll overruns, raise
      `livePlotPeriod` or plot every k-th point.
- [ ] One track step of each mode on the bench (a relative pressure track
      after a kept pressurize, which borrows the hold, and a position
      track from idle with its own operation): the series starts when the
      slave reports the mode (`TrackIndex` stays 1 until then), the
      targets in the trace follow the table, pause and resume restart it,
      and `TrackMode` is 0 afterwards. Run `experimentTemplate` with a
      short `holdTime` and check its files (1 s, 0.05 s and 10 s rows,
      `procStep` in the column).
- [ ] Relaunch the operator app and check the pressure units and the
      procedure rework: the psi/atm/MPa toggles fit the pressure panel and
      the choice persists across a restart; the target spinner's limits
      and steps in each unit (psi 5, atm 0.5, MPa 0.05) and its value
      converting when the unit changes; the panel and label titles say
      "abs"; the procedure's pressure fields, their range message and
      their texts in each unit ("114.7 psi abs"); the pressure readout
      turning red and the status suffixes while the slave blocks
      pressurising and at the end of travel; the speed label's "(step)"
      during the purge pushes and the operator's speed back afterwards;
      the held pressure label in the unit; a slave fault pausing the
      procedure ("Paused: Slave fault N"), Resume refused until the reset
      has gone through, then the step issued again; the delivery step
      ending at the end of travel with its note. Tune `PUSH_SPEED` and
      `PUSH_PRESSURE_MAX` in `sopProcedure.m` against the real discharge
      line, and the oxygen loading pressure default against the
      regulator.
- [ ] Relaunch the operator app and check the new speed control (percent field and presets).
      Outside a procedure the trace's `salineVolume_mL` and `oxygenRatio`
      hold the `buildShell` defaults (`masterConfig.m`: 343 mL, 1.0).
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

## Operator app features

Changes to `master-src/SOS_Gen3_Compressor_Interface.m`, `masterConfig.m`
and, where the slave must follow, the protocol and the ST. Each is
specified here; the implementation plan decides the files.

- [x] One operator speed for every move (the speed control,
      `cfg.speedMax`/`speedPresets`/`speedDefault`, the one value in
      `TargetVelocity` and `PositionSpeed`; the slave caps Go Home at its
      start with HR7-8, and homing keeps its configured speeds). Today jog takes the
      `JogSpeedDropDown` preset (`TargetVelocity`, limited by
      `jogVelMax`), position moves take `PositionSpeedField` (HR7-8,
      clamped by `positionSpeedMax` and `G_cfgVelLimitMax`), and Go Home
      and homing run at the slave's own `G_cfgGoHomeVelocity`,
      `G_cfgHomeLimFastApproachVel` and `G_cfgHomeLimBackoffVel`. Replace
      the drop-down and the field with one speed control: a row of
      preset buttons plus a field for a typed value, labelled in percent
      of the maximum (one master ceiling for jog and position moves, the
      `jogVelMax`/`positionSpeedMax` pair collapsed to one), the mm/s
      equivalent shown beside it. The one speed sets the jog velocity
      and HR7-8. For Go Home the slave takes HR7-8 as well:
      `FB_GoHome`'s `HomeVelocity` becomes
      `MIN(operator speed, G_cfgGoHomeVelocity)`, latched when the move
      starts, so the operator's speed caps it and the GVL value stays the
      ceiling; a speed of 0 does not start it. Homing keeps its
      configured speeds. Update `MasterProtocolGuide.md` (HR7-8),
      `ConfigurationGuide.md`, `HomingSequenceGuide.md`,
      `master-src/README.md`, the simulator and `closedLoopMasterSlave`.
- [x] Volume readout with three views, switched on the panel: swept
      volume (bore area times the travel left to the end of travel,
      without the dead volume: the saline still deliverable), mixing
      volume (the swept volume plus the part of the dead volume that
      takes part in mixing, a new `masterConfig` constant beside
      `deadVolume`, for the in-situ ratio) and an incremental volume
      (swept volume since the operator zeroed it, with a Zero button).
      Today `VolumeValue` shows swept volume plus the whole dead volume
      (`x2mL`). Keep the mL/mm scaling in `mLPerMm`.
- [x] Flow readout: the velocity feedback dithers about zero and the
      sign flickers (`-0.00`). Show a zero without a sign (round first,
      then drop the sign of a zero) and consider a deadband at the
      display resolution.
- [x] Faster display updates. The app polls the model on a 0.2 s timer
      (`updateTimer`, `pollModel`) through `get_param(...,
      'RuntimeObject')` per block; the exchange runs every 50 ms. Raise
      the rate (50 to 100 ms), measure the poll's cost on the lab PC and
      keep the readouts from lagging the timer (coalesce reads; skip a
      tick that overruns).
- [ ] Non-manual mode: the operator walk-through and the machine
      automation behind "Start Procedure". The tabs (Process
      Parameters, Saline Purge/Fill, Oxygen Fill, Pressurize/Discharge)
      are a checkbox stub; `StartProcedureButtonPushed` only enables
      them. A graduate student's standard operating procedure will set
      the steps; the backend comes first:
  - [x] A procedure engine in the app: an ordered list of steps, each an
        operator step (instruction, confirmed by a checkbox or a button)
        or a machine step (one of the existing operations with its
        target, run through `requestOp`/`trackOp`, complete on the
        model's status), with pause, abort (stop the axis) and the
        fault path (a fault aborts the procedure, the reset dialog
        stays). It runs from the poll timer, never blocks it and never
        bypasses the manual-mode interlocks (`canMove`).
  - [x] The procedure as data separate from the engine (a list of step
        structs from one `.m` file), seeded with the steps the tabs name
        today, so that the SOP replaces the data and not the code.
  - [ ] The walk-through panel: the current step, its instruction, the
        confirm/next, pause and abort controls, and progress; it replaces
        the tabs. Manual control and the procedure exclude each other.
        Written (`ProcedurePanel` in the app, the stub tabs removed);
        awaits the lab PC check (Lab PC, below).
  - [ ] Pressurize/Discharge needs the pressure target and the volume
        readouts (above) during delivery; `TimePressurized` and
        `VolumeDischarged` come from the engine's timing and the
        incremental volume. The panel shows them (`ProcTimerLabel`,
        `ProcVolumeLabel`, `ProcPressureTargetLabel`); awaits the lab PC
        check.
  - [x] Testable without the lab PC: the engine's sequencing against a
        stand-in for the model's status (as `smokeMasterStateMachine`
        does for the state machine), under Octave as well.
  - [ ] From the first run of the seed procedure on the machine (manual
        control and the readouts run smoothly; the procedure panel works
        but the seed's content and two slave rules do not):
    - [x] Too many inputs at the start: keep the saline volume, the
          oxygen ratio, the oxygen loading pressure, the discharge
          pressure, the purge volume (swept, default 100 mL) and the
          mixer switch; the Henry's limit and the two ratio fields go.
    - [x] The machine computes the oxygen fill position (the chamber
          volume for the saline plus the charge's gas volume at the
          loading pressure) and retracts to it before the regulator is
          opened.
    - [x] Purge draws the purge volume and pushes it out to the end of
          travel; only the final fill draws past the target and purges
          back to it.
    - [x] Instructions quote the pressure in the unit the pressure
          readout shows; a unit switch (psi, atm, MPa) shared by the
          readout and the procedure panel.
    - [x] A purge as a position move through the outlet reached 170 atm:
          the slave has no overpressure rule outside the pressure target
          cap. Add an overpressure block (motion in the pressurising
          direction zeroed above `G_cfgPressureCeiling`, reported in a
          status bit, no fault); moves that push fluid through the outlet
          run slowly or under pressure control; the engine guards each
          step with a pressure maximum.
    - [x] A delivery runs to the soft limit and faults: pressure mode
          stops a margin before `G_cfgPosSoftLimitMax` and reports the
          end of travel; the discharge step ends on the operator's confirm
          or the end of travel, whichever first.
    - [x] After `FAULT_POSITION` the reset looped back into the fault
          until the slave was power cycled: the reset from the drive-off
          fault state re-powers through the activation states, where the
          position check is not suppressed (`InRecovery` is false until
          `ST_RECOVERY`), so the slave refaults on the way. Suppress the
          check while recovery is pending.
    - [x] A slave fault pauses the procedure like E-STOP; after the reset
          the operator resumes or aborts.

## Checks on the machine

- [ ] Commission the piston exit guard (`TestPlanOutline.md` 3.5): a jog
      out through 200 mm on a gas charge that reads above
      `G_cfgExitGuardPressure` there must fault and stop about 3 mm inside
      the boundary, at the highest and the lowest jog speed; the same jog
      on a vented chamber must pass; an open transducer loop in the band
      must give code 8. Read the vented transducer through a full retract
      first: `G_cfgExitGuardPressure` (0.1 MPa) must clear that reading.
      The guard is only as good as `G_cfgPosCylinderEntry` (below).
- [ ] Measure the stop distance at the end margin (pressure mode at
      `G_cfgPressureVelMax` toward the end of travel; the model gives
      about 0.45 mm against the 1.0 mm `G_cfgPressureEndMargin`), and the
      overshoot past `G_cfgPressureCeiling` in a position move at full
      speed against a closed outlet.
- [ ] Measure the operator app's display poll on the lab PC: connect,
      run, and read `app.PollStats` at `cfg.displayPeriod` 0.1 s and
      0.05 s. If the mean cost exceeds 30 % of the period, add one Mux
      "Display" block in `buildShell` and read a single RuntimeObject.
- [ ] Measure the share of the dead volume that takes part in mixing and
      set `cfg.mixingDeadVolume` (it defaults to the whole `deadVolume`).
- [ ] Relaunch the app and check the speed row (percent field, mm/s label,
      preset buttons) fits the controls panel, and that the preset
      buttons, the field and the status line ("held: speed 0") behave.
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
- [x] Characterisation on the calibrated input at the 4 ms scan,
      `bench/MeasurementPlan.md` sessions 0 and 1 (5 November; results in
      `bench/characterisation/README.md` section 8) and its carry-over
      into `models/params.py` (with the series compliance of eq. (49)),
      `robust.py`, the GVL's pressure-loop gains and the design studies.
- [ ] Measurement plan session 2, the oxygen charge: the dissolution
      time constant `tauDiss` (a position hold at 10 MPa), the stiffness
      with free gas, the hose opening at pressure under the probe, and
      the supply pressure. The liquid-only hold at 10 MPa of session 1
      step 1, the leak baseline it subtracts, was not logged: log one on
      the next liquid fill.
- [ ] The parts of sessions 0 and 1 not run on 5 November:
  - [ ] Weigh a fill against the geometry. Every 5 November volume is
        the geometry's (A (x_EOT - x) + 57 mL), and the dead volume of
        the passages with the present fittings is assumed unchanged.
  - [ ] The `s3` steps with `G_cfgDirectCtrlAccel` at 1 mm/s² (whether
        the `Acceleration` input limits the drive; see below).
  - [ ] Weigh the first Q4 and Q7 deliveries' discharge next time; Q7
        at 7 MPa gave no steady window, so Cd rests on 4 and 10 MPa.
- [ ] The K ramps' gas-law term is still 0.2 to 0.8 mL at the ramps'
      mean pressure after the two fills separated the constant series
      compliance, more than the transducer pocket (`V_air`, 0.1 mL). A
      hold at 10 MPa says whether it is gas (the pressure decays at
      constant position) or a compliance that falls with pressure; if
      the latter, give `charBulkModulus` and the plant a
      pressure-dependent series term.
- [ ] The drive changed between the October and November benches
      because `Y_DirectControl`'s FilterTimeConstant went from a
      hard-wired 0.05 s to `G_cfgDirectCtrlFilterTime` 0.02 s. Decide the
      setting (a lag inside the pressure loop: shorter raises the
      loop's margin) and remeasure the drive with
      `frictionRun('s1', 'steps')` whenever it changes.
- [ ] `frictionRun`'s 90 % torque abort trips on the inertia of a 5 mm/s
      start or stop (80 to 100 % for tens of ms), so the unloaded sets
      stop at 3 mm/s. If the 5 mm/s point is wanted, make the abort act
      only on a torque held over a few exchanges, as a stall would, and
      run `frictionRunSim` before using it.
- [ ] `charStopPath` plans on a gas charge with one K for the liquid; give
      it the series compliance (or take `charBulkModulus`'s stiffness)
      so that a liquid-only plan needs no hand-set `stiffness`.
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
  - [ ] Cylinder entry, `G_cfgPosCylinderEntry` (180 mm: 187 mm of travel in the bore
        from `posEOT`, so it moves with `posEOT`). The simulator's
        `chamberContactPos` is still 200 mm.
  - [ ] Margins between the limits. `posMax` and `G_cfgPosSoftLimitMax`
        are both 367 mm and the switch sits just past them, so a jog the
        master stops at `posMax` can overshoot into `FAULT_POSITION` or
        the switch. Leave room for the arrival tolerance (0.5 mm) and one
        exchange of travel at full speed (0.35 mm at 7 mm/s), then set
        `posMax` < `G_cfgPosSoftLimitMax` < the switch.
- [ ] Measure what the models still assume (`design/pressure-loop/README.md`,
      "Open questions"): the dissolution time constant `tauDiss`
      (session 2). The lag at the piston is not resolved beyond the
      motor's; the hose's compliance is mostly wall (0.58 of 0.62 mL/MPa
      after a purge).
- [x] Stopping distance under load at 8 to 10 MPa with scan-rate
      captures (`stops_hp_char_20261105_175543`): 0.019 s times the
      speed plus v²/(2 × 196 mm/s²) compressing, 0.016 s and 77 mm/s²
      returning, every stop within the slave's allowance.
- [ ] Decide whether to tighten the stop law's allowance to the fit
      (`G_cfgPressureDecel` 50 mm/s² against the measured 77 or more;
      the stop law allows one scan of delay against the measured 16 to
      19 ms). The fit's residual
      is 28 % of stops a few µm long, so a second run at 0.5 and 1 mm/s
      would pin the two terms better.
- [ ] Confirm that `Y_DirectControl`'s `Acceleration` input limits the
      drive in VelocityTLMode: velocity steps at 1 and at 100 mm/s². The
      bench saw every step complete in 60 ms whatever its size
      (`bench/characterisation/README.md` section 8), which says it does
      not.
- [ ] Hose length: the hose is now 6 ft (58 mL, 0.58 mL/MPa). Opening the
      proximal valve at 10 MPa drops the chamber to 4.3 MPa in one scan
      (6.4 with the 3 ft hose), so with the present procedure only 1:1 is
      deliverable, and in 69 % of the plausible cases; the purge and the
      dead volume each take 58 mL, leaving 217 mL of solution from a
      343 mL fill. Decide between the 6 ft hose, the 3 ft one, or
      keeping the 6 ft one with one of the remedies below.
- [ ] Hose procedure: the proximal valve stays shut during the
      compression and opens after mixing; a switched bypass orifice is
      deferred. With the 6 ft hose, keep oxygen runs at 1:1 or accept
      the degassed slug. The remedies: open the proximal valve before
      the compression (the loop then delivers 2:1 at
      10 MPa in every plausible case), or fit a bypass metering orifice
      of 0.2 mm (Cv about 0.0013, five seconds to fill the hose) opened
      ahead of the ball valve (`study_hose_opening.py`). The purge must
      carry the slug that degassed entering the hose, 4 to 6 mL behind
      the 58 mL of virgin saline.
- [ ] The hose model's inputs that only the procedure can give: whether
      the whole 58 mL is purged, the pause between purge and delivery,
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

- [ ] Commission the retuned pressure loop (Kp 1.5, Ki 0.6, stop-law
      stiffness 25 MPa/mm) in this order: the loop's crossover on the
      bench's K ramps (`bench/characterisation/README.md` section 9); a
      water-only compression to 10 MPa, comparing the arrival and the
      overshoot with the model's 2.9 s and 0.1 MPa; a water-only
      delivery with the full valve sequence, comparing the opening dip
      and the purge-closing rise with the model's 0.8 and 0.9 MPa; then
      oxygen at 1:1. The gains are simulation-tuned on one bench
      session's series compliance; Kp 1.0 is the fallback if the bench
      shows less margin.
- [ ] An on-line stiffness estimate (dP against dx while the piston moves)
      as a monitor, logging its value and flagging a chamber stiffer than
      the fixed gain allows (a crossover above about 11 rad/s: hose shut
      at a low fill). Model predictive control and a gain schedule were
      considered and set aside (the schedule gave the results of the
      fixed gain on the measured chamber).
- [ ] A transducer in the tube downstream of the distal valve would give
      the outflow and the valve state (the flow feedforward the studies
      carry as a bound, limited by the loop's 50 ms); a transducer at the
      distal end of the hose gives no usable lead (the pressure wave
      crosses the 6 ft hose in 6 ms). Decide whether the extra input is worth
      it.
- [x] The stop-law floor (the switched floor with the forward stop) is
      part of `FB_PressureController`, behind `G_cfgPressureFloorEnable`,
      off by default.
- [ ] Commission the stop-law floor after the retuned loop: a water-only
      delivery with `G_cfgPressureFloorEnable` TRUE, checking that the
      latch (`fbPressureCtrl.Latched`) sets within a second of the distal
      valve opening and releases when it shuts, and that the closing peak
      stays near the model's 10.4 MPa (10.9 without the floor). It is
      what 2.5:1 at 11 MPa needs to stay under the ceiling in every
      plausible case.
- [ ] The probe's record index wraps after 2^32 scans (about 200 days at
      4 ms of uninterrupted running), and the master does not handle the
      wrap.
