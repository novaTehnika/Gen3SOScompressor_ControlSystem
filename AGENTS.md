# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## What this is

Control software for a single-axis servo compressor (Gen3 SOS, a pre-clinical research device). Two controllers talk over discrete wires, not a bus:

- **Slave** (`slave-src/`): IEC 61131-3 Structured Text for a Yaskawa MP2600iec driving a Sigma-7 servo, edited in MotionWorks IEC Express. It owns the axis state machine, homing, safety and fault latching.
- **Master** (`master-src/`): MATLAB running under Simulink Desktop Real-Time on a lab PC. It talks to the slave through an NI PCI-6251 DAQ and drives it via the operator app.

The contract between them is a 3-bit mode code + MotionEnable + FaultReset (master → slave), a 3-bit confirmation/fault code + status bits (slave → master), one analog reference (AO) and one analog position feedback (AI). Mode entry is a handshake: the slave echoes the mode on its confirmation bits before the master raises MotionEnable. `docs/master/MasterProtocolGuide.md` is the authoritative spec. Fault codes are in `docs/master/FaultCodeReference.md` and pin mapping in `docs/master/IOReference.md`.

## Commands

Everything runs headless with `python3` and `octave-cli`. No MATLAB is needed.

```sh
tests/run_all.sh            # regenerate the slave sim from slave-src, run all tests
```

Run a single test (from the repo root, after generating):

```sh
python3 tools/st2m.py slave-src slave-sim/generated
octave-cli --quiet --eval "addpath('master-src/logic','slave-sim','slave-sim/generated','tests'); closedLoopMasterSlave"
```

Tests are scripts that print `ok  ...` per scenario and fail on the first `assert`. `closedLoopMasterSlave(t)` takes an optional slave handshake timeout.

The Simulink shell and the operator app can only be exercised on the lab PC (`master-src/README.md` has the steps). The ST cannot be compiled here. It is built in MotionWorks IEC.

## Architecture

### Slave → simulator translation (`tools/st2m.py`)

The slave ST is the single source of truth. `tools/st2m.py` translates it into Octave functions in `slave-sim/generated/`. That directory is gitignored, so never edit it by hand. Consequences:

- MotionWorks keeps variable declarations in its GUI, so st2m reads them from the **`Variables (define in MWiec GUI)` header comment** of each `.st` file, from `slave-src/GVL/GlobalVariables_Reference.st` (including `G_cfg*` default values) and from `slave-src/DUT/DataTypes.st`. Those comments are functional input. When you add or rename a variable, update the header comment or the translation breaks or goes stale.
- st2m supports only the ST subset already used: assignments, IF/ELSIF/ELSE, FOR, RETURN, named-input FB calls, typed/TIME literals, 1-D arrays, ABS/SQRT/MIN/MAX/LIMIT, TON/R_TRIG/F_TRIG. It errors on anything else. If you use a new construct, extend st2m.
- FB instances become structs, and inputs omitted from a call keep their previous value (IEC semantics). Generated lines carry `% :N` pointing back to the ST source line.

### Slave structure

- `PRG_Main.st` holds the central state machine (idle → drive reset/enable → brake release → operational modes → hold/deactivate, plus fault and recovery states). It calls every user FB exactly once at the end of the file.
- All built-in motion FBs (`MC_Power`, `MC_Stop`, `MC_MoveAbsolute`, `Y_DirectControl`, `MC_SetPosition`, Jog, …) and `G_sysAxis` live in a **Ladder Diagram POU that is not in this repo**. ST never calls them directly. It writes `G_cmd*` structs and reads `G_sta*` structs (types in `DataTypes.st`). Custom FBs that drive motion emit `Cmd*` outputs and take `Sta*` inputs, and `PRG_Main` wires them to the globals.
- In the simulator, `slave-sim/axisModel.m` stands in for that LD POU plus the drive and mechanics. `slaveStep.m` runs one scan: `PRG_Input` → `PRG_Main` → `PRG_Output` → `axisModel`, so PRG_Main sees motion status one scan late. `slaveSimConfig.m` holds only the physical axis parameters. The slave's own config comes from the ST. Test hooks (`sl.hook.driveFault`, `blocked`, `torque`) inject faults.
- Naming: Hungarian-style local prefixes (`b`, `n`, `r`, `t`, `e`, `st`, `fb`, `rTrig`/`fTrig`) and global prefixes `G_cfg*`, `G_sys*`, `G_pos*`, `G_di*`/`G_do*`, `G_ai*`/`G_ao*`. See `docs/slave/development/SlaveDeveloperGuide.md`.

### Master structure

- `master-src/logic/` holds all master logic as plain `.m` functions that run in both MATLAB and Octave. `masterStep.m` is the per-step entry point called from the Simulink MATLAB Function block. It keeps persistent filter state, applies the DAQ polarity (slave outputs sink the DAQ lines, so a raw 0 means asserted) and calls `masterStateMachine.m`, which sequences the handshake, motion and fault reset. `masterConfig.m` holds every constant, including bench limits and scaling.
- `master-src/shell/buildShell.m` generates `SOS_Gen3_Master_Shell.slx` (gitignored). The model holds no logic and is never hand-edited. It copies DAQ blocks from `SimulinkMaster.slx` to keep the board setup.
- `SOS_Gen3_Compressor_Interface.m` is the operator app. It writes `RequestedMode` and then increments `CommandSeq` in the shell's Constant blocks. The model acts only on a `CommandSeq` change, except that mode 0 (stop) always acts.
- `master-src/legacy/` and `SOS_Gen3_Compressor_Master.slx` hold an earlier unfinished Stateflow attempt, kept for reference only.

### Tests

- `smokeMasterStateMachine`: master state machine against a protocol-level stand-in slave (not the ST sim).
- `closedLoopMasterSlave`: `masterStateMachine` against the generated slave. The master steps every 3 ms and the slave every 2 ms on a 1 ms tick.
- `closedLoopMasterStep`: `masterStep` through the real DAQ wiring and polarity, with the slave's position map set to match `masterConfig`.

## Conventions

- Source comments and docs describe mechanism only. Dates, symptoms, history and failed approaches belong in commit messages.
- When slave behaviour or the protocol changes, update the matching docs under `docs/master/` and `docs/slave/` in the same change.
