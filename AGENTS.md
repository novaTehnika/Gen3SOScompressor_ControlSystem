# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## What this is

Control software for a single-axis servo compressor (Gen3 SOS, a pre-clinical research device). Two controllers talk over Modbus TCP plus one hardwired safety line:

- **Slave** (`slave-src/`): IEC 61131-3 Structured Text for a Yaskawa MP2600iec driving a Sigma-7 servo, edited in MotionWorks IEC 3 Express. Express has a single task: every POU (the ST programs, the LD POU, bench programs) and the Modbus groups run on the one cyclic task, so work cannot be moved to a slower task; anything that need not run every scan is skipped or spread over scans inside its POU. It owns the axis state machine, homing, safety and fault latching.
- **Master** (`master-src/`): MATLAB running under Simulink Desktop Real-Time on a lab PC. It talks to the slave through an NI PCI-6221 DAQ and drives it via the operator app.

The contract between them is a mode code + MotionEnable + FaultReset + reference in engineering units (master → slave, holding registers HR1–8, with a heartbeat and the position-move speed in HR7–8) and a confirmation code, fault code, status bits and position/velocity/torque/pressure feedback (slave → master, input registers IR1–20, pressure in IR19–20, with a scan counter). The master writes and reads every 50 ms; the slave services Modbus on a 4 ms tick. Mode entry is a handshake: the slave echoes the mode in its confirmation register before the master raises MotionEnable. Each side watches the other's counter and faults after 300 ms (the slave only while motion is commanded). The safety relay stays on DAQ line P0.0. `docs/master/MasterProtocolGuide.md` is the authoritative spec, `docs/master/ModbusRegisterMap.md` the register layout. Fault codes are in `docs/master/FaultCodeReference.md` and pin mapping in `docs/master/IOReference.md`.

## Commands

Everything runs headless with `python3` and either `octave-cli` or MATLAB; the MATLAB-language files are written to run in both. The simulated slave is compiled, so the tests also need a C compiler: `mkoctfile` for Octave (Debian/Ubuntu package `octave-dev`) or one configured for `mex` in MATLAB (MinGW-w64 on Windows), and `cc` (or `$CC`) for the Python parity tests.

```sh
tests/run_all.sh            # regenerate, then run the tests whose inputs changed
tests/run_all.sh --all      # run every test regardless
tests/run_all.sh -j 2 closedLoopMasterSlave   # chosen tests, at most 2 at once
tests\run_all.bat           # every test, one after another, on Windows (MATLAB)
```

`run_all.sh` builds the simulated slave (`slave-sim/buildSlaveSim.m`) and regenerates `design/generated`, then runs each test in its own process: the Python tests with `python3`, the MATLAB-language tests in `octave-cli` or, when that is not on the PATH, in `matlab -batch`. It runs as many at once as there are CPUs (one at a time with MATLAB). It skips a test whose inputs hash the same as when it last passed (recorded in the gitignored `.test-cache/`, which also holds each test's log). A test's inputs are its own file, the paths on its `Depends on:` header line and the interpreter version. Keep that line current when a test starts using another directory, or the test will be skipped when that code changes. A new test goes in `ALL_TESTS` in the script and, if it is MATLAB-language, in `tests/runAll.m`.

`runAll.m` runs the MATLAB-language tests one after another in one session, with no skipping. It builds the simulated slave when stale (with `python3`, `python` or `py -3` and the C compiler) unless called as `runAll('nogen')`, so from an open MATLAB or Octave session, `cd tests` and `runAll` runs them.

Run a single test (from the repo root, after generating and building):

```sh
octave-cli --quiet --eval "addpath('slave-sim'); buildSlaveSim"
octave-cli --quiet --eval "addpath('master-src/logic','slave-sim','slave-sim/generated','tests'); closedLoopMasterSlave"
```

or the same `addpath` and call in MATLAB. Tests are scripts that print `ok  ...` per scenario and fail on the first `assert`. `closedLoopMasterSlave(t)` takes an optional slave handshake timeout.

The Simulink shell and the operator app can only be exercised on the lab PC (`master-src/README.md` has the steps). The ST cannot be compiled here. It is built in MotionWorks IEC.

## Architecture

### Slave translation (`tools/st_translate.py`)

The slave ST is the single source of truth. `tools/st_translate.py --target T SRC OUT` translates it for simulation. `tools/st_translate.py` is the front end every target shares (declarations, tokens, and the `Translator` parser, whose spelling methods default to Octave's), and each target is a module `tools/st_target_<T>.py` that overrides the spellings where its language differs and writes the files:

- `c`: C for the compiled slave simulator, in `slave-sim/generated-c/` (see Slave structure).
- `py`: Python modules of the function blocks for design studies, in `design/generated/` (see below).

The output directories are gitignored, so never edit them by hand. Consequences:

- MotionWorks keeps variable declarations in its GUI, so the translator reads them from the **`Variables (define in MWiec GUI)` header comment** of each `.st` file, from `slave-src/GVL/GlobalVariables_Reference.st` (including `G_cfg*` default values) and from `slave-src/DUT/DataTypes.st`. Those comments are functional input. When you add or rename a variable, update the header comment or the translation breaks or goes stale.
- The translator supports only the ST subset already used: assignments, IF/ELSIF/ELSE, FOR, RETURN, named-input FB calls, typed/TIME literals, 1-D arrays (inline or as named array types of `DataTypes.st`), ABS/SQRT/MIN/MAX/LIMIT/SHR/SHL, MOD, integer conversions that wrap modulo 2^bits as on the controller (`INT_TO_WORD`, `DINT_TO_DWORD`, `DWORD_TO_WORD`), TON/R_TRIG/F_TRIG. It errors on anything else. If you use a new construct, extend the front end and every target's spelling of it; `tests/stTargetParity.py` runs the targets on the same input sequences and compares every variable. `CASE` is deliberately not used in the slave: it did not work out on this controller, so state machines stay as IF/ELSIF chains. `VAR_IN_OUT` is not used either, for the same reason, and the translator rejects it; a block hands an array back as a `VAR_OUTPUT`.
- FB instances become structs, and inputs omitted from a call keep their previous value (IEC semantics). Generated lines carry `% :N` (`// :N` in C) pointing back to the ST source line. The `c` target keeps the controller's types (INT as `int16_t`, UINT `uint16_t`, REAL `float`); TIME stays in seconds.

### Python target (`--target py`)

`python3 tools/st_translate.py --target py slave-src design/generated` translates every `FB_*.st` into a Python module for design studies.

- Output: one `FB_<Name>.py` per block with `new_FB_<Name>()` (a `SimpleNamespace` holding every VAR_INPUT/VAR_OUTPUT/VAR at its header default) and `FB_<Name>(s, dt)` (one scan, mutates `s` in place, `dt` in seconds), plus `st_runtime.py` (enumerations as `E`, `TON`/`R_TRIG`/`F_TRIG`, helpers, struct constructors). Imports are flat, so put `design/generated` on `sys.path`.
- Programs (`PRG_*`) are not translated, and blocks that reference `G_*` globals or I/O points are rejected. ST arrays must be 0-based and become lists.

### Slave structure

- `PRG_Main.st` holds the central state machine (idle → drive reset/enable → brake release → operational modes → hold/deactivate, plus fault and recovery states). It calls every user FB exactly once at the end of the file.
- All built-in motion FBs (`MC_Power`, `MC_Stop`, `MC_MoveAbsolute`, `Y_DirectControl`, `MC_SetPosition`, Jog, …) and `G_sysAxis` live in a **Ladder Diagram POU that is not in this repo**. ST never calls them directly. It writes `G_cmd*` structs and reads `G_sta*` structs (types in `DataTypes.st`). Custom FBs that drive motion emit `Cmd*` outputs and take `Sta*` inputs, and `PRG_Main` wires them to the globals.
- In the simulator, the axis model in `slave-sim/c/slave_sim.c` stands in for that LD POU (the motion function blocks' profiles and status), and the design model's plant stands in for the drive and the mechanics: `slave-sim/c/plant.c` is `design/pressure-loop/plant.py` in C (chamber with its saline and oxygen charge, outlet, drive dead time, lag, stall and friction); the drive's velocity lag is the plant's `tauV` (`sl.plant.par.tauV`), so the velocity feedback lags the commanded velocity in the plant and not in the registers, and `Y_DirectControl` has a single `Acceleration`, with no deceleration, held to it by `tests/plantParity.py`. Its parameters and a run's starting contents come from `design/pressure-loop` through `tools/plant_export.py` at build time, so they have one source. Y_DirectControl's velocity and torque modes drive the plant's actuator; the other motions prescribe the piston's velocity, and the plant integrates the chamber along it and can stall it. The plant's piston is at zero at `sim.plantStartPos`; before `sim.chamberContactPos` the cylinder is open and the pressure reads zero. Assigning `sl.ax.x` places the piston and settles the chamber there; `chamberPosition(sl, P)` gives the position for a pressure. `sl.plant` exposes the plant's state and `sl.plant.par` its parameters (SI units). `slaveStep.m` takes the holding registers and returns the input registers in engineering units and runs one scan: `PRG_Input` → `PRG_Main` → `PRG_Output` → axis model, so PRG_Main sees motion status one scan late. `slaveSimConfig.m` holds only the axis geometry and the transducer. The master-set position speed is a required field of `u` (`positionSpeed`, HR7–8); `slaveStep` has no default for it. The slave's own config comes from the ST. Test hooks (`sl.hook.driveFault`, `blocked`, `torque`, `outflow`, `valve`, `pressureVolt`) inject faults and drive the outlet.
- The simulator is compiled: `slaveInit` returns `SlaveSim`, a handle object over the MEX file `slaveSimMex`, built into `slave-sim/generated/` by `buildSlaveSim.m` (when stale) from the `c` target and the hand-written `slave-sim/c/`. Its fields (`G`, `IO`, `s` for PRG_Main, `ax`, `sim`, `hook`, `E`) read and assign like a struct's; a read of a group (`sl.G`) is a snapshot, so read single fields in per-scan code, and assigning a struct to a group sets each of its fields. The axis, `sim` and `hook` fields are declared once, in the X-macros of `slave_sim.h`; the `sim` values come from `slaveSimConfig.m` when a `SlaveSim` is created.
- The step probe (`FB_Probe`, instance `fbProbe`, called last in `PRG_Main`) is a permanent optional side channel for high-rate capture in experiments and benches (HR111-114, IR401-524, globals `G_mbProbe*`; `G_cfgProbeEnable`, `G_cfgProbeFillPerScan`). It records 8-word records every scan into a ring of 8192, runs a recording state machine (`E_ProbeState`) and a transfer state machine (`E_ProbeXferState`) and latches `FaultMark` on entering `ST_FAULT`. Its per-scan cost is bounded (at most `G_cfgProbeFillPerScan` records copied per scan, the window published in one 104-word copy by `PRG_Output` when a fill completes). It reads no `G_*` itself, so the py and c targets translate it, `tests/fakeProbeLink.m` reads its registers from the simulated slave (the global `G_mbProbe*` through `SlaveSim`), and `tests/benchProbeSim.m` and `tests/stTargetParity.py` check it. Nothing in the protocol depends on it.
- Naming: Hungarian-style local prefixes (`b`, `n`, `r`, `t`, `e`, `st`, `fb`, `rTrig`/`fTrig`) and global prefixes `G_cfg*`, `G_sys*`, `G_di*`/`G_do*` (hardwired I/O), `G_ai*` (pressure input), `G_mb*` (Modbus registers). See `docs/slave/development/SlaveDeveloperGuide.md`.

### Design studies

- `design/pressure-loop/` holds the Python model used to design the slave pressure loop (chamber, actuators, sensor chain) and the studies run on it. It runs the slave's own `FB_AnalogInputFilter`, `FB_PressureInput` and `FB_PressureController`, translated by `tools/st_translate.py --target py` into the gitignored `design/generated/` (regenerated automatically when stale) with their configuration read from the GVL, so the slave controller in the studies is the slave source. The model is split as model, driver and solver: `plant.py` (`Plant.derivatives` is the continuous-time model; the command dead time, stall and state bounds run between steps), `sim.py` (the scan loop) and `solver.py` (fixed-step Heun by default, forward Euler as a check). The plant includes the hose between the cylinder head and the outlet (its virgin saline, its compliance, and the proximal valve's opening as a one-scan equalisation or as a throttled fill through a given bore, with the admitted solution tracked as parcels and flagged where it degassed) and the purge of that saline and the degassed slug before the delivery; `Params.hose_before` selects whether the hose is opened before the compression or after mixing. The plant parameters are the bench measurements (`bench/`), and the requirements are the machine's: a pressure ceiling, no degassing below the saturation pressure of the dissolved oxygen, control kept to the limit switch. `controllers.py` holds the slave's controller and `ScheduledSlaveController`, the candidate that schedules the block's gains and stop law on a stiffness bound. `test_model.py`, `test_linear.py` and `test_robust.py` are the checks (run by `tests/run_all.sh`). The studies are `study_compare.py`, `study_delivery.py` (time-domain scenarios; the latter gives deliverable volume against oxygen charge) and `study_hose_opening.py` (the proximal valve opened at pressure against its bore), `study_linear.py` (discrete-time margins against chamber stiffness), `study_robust.py` (Monte Carlo over the measured parameter ranges against the requirements in `robust.py`, per charge and target) and `study_convergence.py` (integration error against the step); they run from the repo root or that directory with `--set key=value` and `--out DIR`. Python dependencies are in `design/requirements.txt`. Its README gives the model, the numbered model equations, the numerical method, the assumptions still to be measured and the results. It is not part of the product and nothing imports it.
- Code in `design/` is written to be read by someone assessing the analysis. Every file opens with a header docstring (file name and type, AUTHORS, CREATION DATE, PURPOSE/DESCRIPTION, FILE DEPENDENCY; no revision table and no license block). Module-level sections are marked `# %% ===== NAME =====` and class-level subsections `# ----- Name -----`. Lines stay within 75 columns. Every quantity carries its unit in square brackets (`# [Pa]`) and every conversion is commented (`# [mm/s -> m/s]`). Every equation or group of equations has a written label and cites its number in the README's "Model equations" section (`eq. (5)`); a new equation gets a number there. Comments and docstrings are complete sentences.

### Master structure

- `master-src/logic/` holds all master logic as plain `.m` functions that run in both MATLAB and Octave. `masterStep.m` is the per-step entry point called from the Simulink MATLAB Function block. It takes the 20-register read vector and returns the 8-register write vector (`modbusMap.m` for indices, `modbusPack.m`/`modbusUnpack.m` for REAL values, low word first), keeps persistent filter and link-watchdog state and calls `masterStateMachine.m`, which sequences the handshake, motion and fault reset. `masterConfig.m` holds every constant, including the 25 ms step and the 50 ms exchange.
- `master-src/shell/buildShell.m` generates `SOS_Gen3_Master_Shell.slx` (gitignored). The model holds no logic and is never hand-edited. `master-src/shell/addDaq.m` creates the DAQ blocks and is the one place the board (DAQ model, driver options) is named.
- `SOS_Gen3_Compressor_Interface.m` is the operator app. It writes `RequestedMode` and then increments `CommandSeq` in the shell's Constant blocks. The model acts only on a `CommandSeq` change, except that mode 0 (stop) always acts.
- `master-src/legacy/` and `SOS_Gen3_Compressor_Master.slx` hold an earlier unfinished Stateflow attempt, kept for reference only.

### Benches (`bench/`)

Each bench (`modbus`, `pressure`, `friction`, `characterisation`) is a directory with its own README, config and fits, run on the lab PC. What they share is in `bench/common/`: the controller connection (`benchConfig.m`), the slave settings parsed from the GVL reference (`benchGvl.m`), the raw Modbus link the run scripts use in place of the shell (`benchLink.m`), the safety-hold model, the master side of the step probe (`benchProbe.m` arms captures, reads them out of the slave's ring in the gaps between protocol exchanges and fetches the history before a fault; the probe itself is `FB_Probe` in `slave-src`) and the `captures/` directory every run logs to. Bench slave programs live beside the bench that owns them and are never part of `slave-src/`.

### Tests

- `smokeMasterStateMachine`: master state machine against a protocol-level stand-in slave (not the ST sim).
- `closedLoopMasterSlave`: `masterStateMachine` against the generated slave in engineering units. The master steps every 25 ms, the exchange runs every 50 ms and the slave every 4 ms (its scan) on a 1 ms tick.
- `closedLoopModbusStep`: `masterStep` through the packed register vectors, including link loss on either side.
- `stTargetParity.py` (Python, run by `run_all.sh`): every scalar-input function block translated by the py and c targets, driven with the same seeded random inputs; all variables must agree to 1e-9, including `FB_Probe` (its ring compared once after the last call, its register-style inputs reaching the pending, filling and overrun paths, the fault mark).
- `plantParity.py` (Python): the C port of the design plant (`slave-sim/c/plant.c`) against `design/pressure-loop/plant.py`, built with `cc` and driven through ctypes with the same inputs every scan; every state must agree to 1e-9 (they agree exactly). `plant.py` is the reference: a change to it needs the same change in `plant.c`.
- `benchProbeSim`: `bench/common/benchProbe.m` against the slave's own `FB_Probe` (the compiled slave behind `tests/fakeProbeLink.m`, which only emulates the link: lost arm writes, torn reads, and a shortened ring by setting the instance's `cDepth`): the reader's retry, tear, overrun and drain paths, the recording and transfer state machines scan by scan, the fault mark and the new record columns.
- `charRunSim`, `frictionRunSim`: the characterisation and friction benches against the simulated slave through `tests/fakeProbeLink.m`; their fits must recover the plant's own drive lag, dead time and chamber compliance (and the friction law the stand-in reports as torque). `charFitsSynthetic`: the four characterisation fits on synthetic logs with known parameters.

## Conventions

- Source comments and docs describe mechanism only. Dates, symptoms, history and failed approaches belong in commit messages. The one exception is the CREATION DATE in the `design/` file headers.
- When slave behaviour or the protocol changes, update the matching docs under `docs/master/` and `docs/slave/` in the same change.
