# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## What this is

Control software for a single-axis servo compressor (Gen3 SOS, a pre-clinical research device). Two controllers talk over Modbus TCP plus one hardwired safety line:

- **Slave** (`slave-src/`): IEC 61131-3 Structured Text for a Yaskawa MP2600iec driving a Sigma-7 servo, edited in MotionWorks IEC Express. It owns the axis state machine, homing, safety and fault latching.
- **Master** (`master-src/`): MATLAB running under Simulink Desktop Real-Time on a lab PC. It talks to the slave through an NI PCI-6221 DAQ and drives it via the operator app.

The contract between them is a mode code + MotionEnable + FaultReset + reference in engineering units (master → slave, holding registers HR1–6, with a heartbeat) and a confirmation code, fault code, status bits and position/velocity/torque feedback (slave → master, input registers IR1–18, with a scan counter). The master writes and reads every 50 ms; the slave services Modbus on a 4 ms tick. Mode entry is a handshake: the slave echoes the mode in its confirmation register before the master raises MotionEnable. Each side watches the other's counter and faults after 300 ms (the slave only while motion is commanded). The safety relay stays on DAQ line P0.0. `docs/master/MasterProtocolGuide.md` is the authoritative spec, `docs/master/ModbusRegisterMap.md` the register layout. Fault codes are in `docs/master/FaultCodeReference.md` and pin mapping in `docs/master/IOReference.md`.

## Commands

Everything runs headless with `python3` and either `octave-cli` or MATLAB; the MATLAB-language files are written to run in both.

```sh
tests/run_all.sh            # regenerate the slave sim from slave-src, run all tests
tests\run_all.bat           # the same on Windows (MATLAB)
```

`run_all.sh` runs the Python tests, then `tests/runAll.m` in Octave or, when `octave-cli` is not on the PATH, in `matlab -batch`. `runAll` regenerates `slave-sim/generated` itself (with `python3`, `python` or `py -3`) unless called as `runAll('nogen')`, so from an open MATLAB or Octave session, `cd tests` and `runAll` runs the MATLAB-language tests alone.

Run a single test (from the repo root, after generating):

```sh
python3 tools/st2m.py slave-src slave-sim/generated
octave-cli --quiet --eval "addpath('master-src/logic','slave-sim','slave-sim/generated','tests'); closedLoopMasterSlave"
```

or the same `addpath` and call in MATLAB. Tests are scripts that print `ok  ...` per scenario and fail on the first `assert`. `closedLoopMasterSlave(t)` takes an optional slave handshake timeout.

The Simulink shell and the operator app can only be exercised on the lab PC (`master-src/README.md` has the steps). The ST cannot be compiled here. It is built in MotionWorks IEC.

## Architecture

### Slave → simulator translation (`tools/st2m.py`)

The slave ST is the single source of truth. `tools/st2m.py` translates it into Octave functions in `slave-sim/generated/`. That directory is gitignored, so never edit it by hand. Consequences:

- MotionWorks keeps variable declarations in its GUI, so st2m reads them from the **`Variables (define in MWiec GUI)` header comment** of each `.st` file, from `slave-src/GVL/GlobalVariables_Reference.st` (including `G_cfg*` default values) and from `slave-src/DUT/DataTypes.st`. Those comments are functional input. When you add or rename a variable, update the header comment or the translation breaks or goes stale.
- st2m supports only the ST subset already used: assignments, IF/ELSIF/ELSE, FOR, RETURN, named-input FB calls, typed/TIME literals, 1-D arrays, ABS/SQRT/MIN/MAX/LIMIT, TON/R_TRIG/F_TRIG. It errors on anything else. If you use a new construct, extend st2m.
- FB instances become structs, and inputs omitted from a call keep their previous value (IEC semantics). Generated lines carry `% :N` pointing back to the ST source line.

### Slave → Python translation (`tools/st2py.py`)

`python3 tools/st2py.py slave-src design/generated` translates every `FB_*.st` into a Python module for design studies (`design/generated/` is gitignored; never edit it by hand). It subclasses st2m's `Translator` and overrides only the target-language spellings, and it reads declarations the same way, so st2py and st2m must stay in step on the ST subset: a construct added to one needs the matching override in the other, and `tests/st2pyParity.py` runs both on the same input sequences and compares every variable.

- Output: one `FB_<Name>.py` per block with `new_FB_<Name>()` (a `SimpleNamespace` holding every VAR_INPUT/VAR_OUTPUT/VAR at its header default) and `FB_<Name>(s, dt)` (one scan, mutates `s` in place, `dt` in seconds), plus `st_runtime.py` (enumerations as `E`, `TON`/`R_TRIG`/`F_TRIG`, helpers, struct constructors). Imports are flat, so put `design/generated` on `sys.path`.
- Programs (`PRG_*`) are not translated, and blocks that reference `G_*` globals or I/O points are rejected. ST arrays must be 0-based and become lists.

### Slave structure

- `PRG_Main.st` holds the central state machine (idle → drive reset/enable → brake release → operational modes → hold/deactivate, plus fault and recovery states). It calls every user FB exactly once at the end of the file.
- All built-in motion FBs (`MC_Power`, `MC_Stop`, `MC_MoveAbsolute`, `Y_DirectControl`, `MC_SetPosition`, Jog, …) and `G_sysAxis` live in a **Ladder Diagram POU that is not in this repo**. ST never calls them directly. It writes `G_cmd*` structs and reads `G_sta*` structs (types in `DataTypes.st`). Custom FBs that drive motion emit `Cmd*` outputs and take `Sta*` inputs, and `PRG_Main` wires them to the globals.
- In the simulator, `slave-sim/axisModel.m` stands in for that LD POU plus the drive and mechanics. `slaveStep.m` takes the holding registers and returns the input registers in engineering units and runs one scan: `PRG_Input` → `PRG_Main` → `PRG_Output` → `axisModel`, so PRG_Main sees motion status one scan late. `slaveSimConfig.m` holds only the physical axis parameters. The slave's own config comes from the ST. Test hooks (`sl.hook.driveFault`, `blocked`, `torque`) inject faults.
- Naming: Hungarian-style local prefixes (`b`, `n`, `r`, `t`, `e`, `st`, `fb`, `rTrig`/`fTrig`) and global prefixes `G_cfg*`, `G_sys*`, `G_pos*`, `G_di*`/`G_do*`, `G_ai*`/`G_ao*`. See `docs/slave/development/SlaveDeveloperGuide.md`.

### Design studies

- `design/pressure-loop/` holds the Python model used to design the slave pressure loop (chamber, actuators, sensor chain) and the studies run on it. It runs the slave's own `FB_AnalogInputFilter`, `FB_PressureInput` and `FB_PressureController`, translated by `tools/st2py.py` into the gitignored `design/generated/` (regenerated automatically when stale) with their configuration read from the GVL, so the slave controller in the studies is the slave source. The model is split as model, driver and solver: `plant.py` (`Plant.derivatives` is the continuous-time model; the command dead time, stall and state bounds run between steps), `sim.py` (the scan loop) and `solver.py` (fixed-step Heun by default, forward Euler as a check). The plant includes the hose between the cylinder head and the outlet (its virgin saline, its compliance, and the proximal valve's opening as a one-scan equalisation or as a throttled fill through a given bore, with the admitted solution tracked as parcels and flagged where it degassed) and the purge of that saline and the degassed slug before the delivery; `Params.hose_before` selects whether the hose is opened before the compression or after mixing. The plant parameters are the bench measurements (`bench/`), and the requirements are the machine's: a pressure ceiling, no degassing below the saturation pressure of the dissolved oxygen, control kept to the limit switch. `controllers.py` holds the slave's controller and `ScheduledSlaveController`, the candidate that schedules the block's gains and stop law on a stiffness bound. `test_model.py`, `test_linear.py` and `test_robust.py` are the checks (run by `tests/run_all.sh`). The studies are `study_compare.py`, `study_delivery.py` (time-domain scenarios; the latter gives deliverable volume against oxygen charge) and `study_hose_opening.py` (the proximal valve opened at pressure against its bore), `study_linear.py` (discrete-time margins against chamber stiffness), `study_robust.py` (Monte Carlo over the measured parameter ranges against the requirements in `robust.py`, per charge and target) and `study_convergence.py` (integration error against the step); they run from the repo root or that directory with `--set key=value` and `--out DIR`. Python dependencies are in `design/requirements.txt`. Its README gives the model, the numbered model equations, the numerical method, the assumptions still to be measured and the results. It is not part of the product and nothing imports it.
- Code in `design/` is written to be read by someone assessing the analysis. Every file opens with a header docstring (file name and type, AUTHORS, CREATION DATE, PURPOSE/DESCRIPTION, FILE DEPENDENCY; no revision table and no license block). Module-level sections are marked `# %% ===== NAME =====` and class-level subsections `# ----- Name -----`. Lines stay within 75 columns. Every quantity carries its unit in square brackets (`# [Pa]`) and every conversion is commented (`# [mm/s -> m/s]`). Every equation or group of equations has a written label and cites its number in the README's "Model equations" section (`eq. (5)`); a new equation gets a number there. Comments and docstrings are complete sentences.

### Master structure

- `master-src/logic/` holds all master logic as plain `.m` functions that run in both MATLAB and Octave. `masterStep.m` is the per-step entry point called from the Simulink MATLAB Function block. It takes the 18-register read vector and returns the 6-register write vector (`modbusMap.m` for indices, `modbusPack.m`/`modbusUnpack.m` for REAL values, low word first), keeps persistent filter and link-watchdog state and calls `masterStateMachine.m`, which sequences the handshake, motion and fault reset. `masterConfig.m` holds every constant, including the 25 ms step and the 50 ms exchange.
- `master-src/shell/buildShell.m` generates `SOS_Gen3_Master_Shell.slx` (gitignored). The model holds no logic and is never hand-edited. `master-src/shell/addDaq.m` creates the DAQ blocks and is the one place the board (DAQ model, driver options) is named.
- `SOS_Gen3_Compressor_Interface.m` is the operator app. It writes `RequestedMode` and then increments `CommandSeq` in the shell's Constant blocks. The model acts only on a `CommandSeq` change, except that mode 0 (stop) always acts.
- `master-src/legacy/` and `SOS_Gen3_Compressor_Master.slx` hold an earlier unfinished Stateflow attempt, kept for reference only.

### Benches (`bench/`)

Each bench (`modbus`, `pressure`, `friction`, `characterisation`) is a directory with its own README, config and fits, run on the lab PC. What they share is in `bench/common/`: the controller connection (`benchConfig.m`), the slave settings parsed from the GVL reference (`benchGvl.m`), the raw Modbus link the run scripts use in place of the shell (`benchLink.m`), the safety-hold model, the step probe program and the `captures/` directory every run logs to. Bench slave programs live beside the bench that owns them and are never part of `slave-src/`.

### Tests

- `smokeMasterStateMachine`: master state machine against a protocol-level stand-in slave (not the ST sim).
- `closedLoopMasterSlave`: `masterStateMachine` against the generated slave in engineering units. The master steps every 25 ms, the exchange runs every 50 ms and the slave every 2 ms on a 1 ms tick.
- `closedLoopModbusStep`: `masterStep` through the packed register vectors, including link loss on either side.
- `st2pyParity.py` (Python, run by `run_all.sh`): every scalar-input function block translated by st2py and by st2m, driven with the same seeded random inputs; all variables must agree to 1e-9.
- `charRunSim`, `charFitsSynthetic`: the characterisation bench (`bench/characterisation/`) against the simulated slave, and its three fits on synthetic logs with known parameters.

## Conventions

- Source comments and docs describe mechanism only. Dates, symptoms, history and failed approaches belong in commit messages. The one exception is the CREATION DATE in the `design/` file headers.
- When slave behaviour or the protocol changes, update the matching docs under `docs/master/` and `docs/slave/` in the same change.
