# Master

The master runs on the lab PC under Simulink Desktop Real-Time and talks to the
MP2600iec slave through the NI PCI-6251 DAQ.

| Path | Contents |
|------|----------|
| `logic/` | All master logic, as plain `.m` functions that run under MATLAB and GNU Octave. `masterStep.m` is the entry point called once per model step; `masterStateMachine.m` sequences the handshake, motion and fault handling; `masterConfig.m` holds every constant. |
| `shell/buildShell.m` | Generates `shell/SOS_Gen3_Master_Shell.slx`: DAQ blocks, one MATLAB Function block calling `masterStep`, the Constant blocks the app writes and the Gain blocks it reads. The model holds no logic and is rebuilt from this script, not edited by hand. |
| `SOS_Gen3_Compressor_Interface.m` | Operator app (programmatic App Designer export). Starts the shell model, writes operator requests to its Constant blocks and polls its outputs. |
| `SimulinkMaster.slx` | Bench model with manual mode control. `buildShell.m` copies its DAQ blocks to keep the board setup. |
| `legacy/`, `SOS_Gen3_Compressor_Master.slx` | Earlier Stateflow-based app model and its `.mlapp`, kept for reference. |

## Running on the lab PC

```matlab
cd master-src
run('shell/buildShell.m')          % once, and after any change to the block interface
addpath logic shell
SOS_Gen3_Compressor_Interface      % then press "Connect to Compressor"
```

## App to model interface

The app writes `RequestedMode` (0 stop, 1 home, 2 position, 3 jog, 4 pressure,
5 reset fault) and then increments `CommandSeq`; the model acts on an operation
only when `CommandSeq` changes, except that `RequestedMode = 0` always stops.
`TargetPosition`, `TargetVelocity`, `JogDirection` and `TargetPressure` are read
continuously while their operation is active. `ESTOP` and `Mixer` act at once.

The app reads `StatusCode`, `FaultCode`, `Homed`, `State`, `Position`,
`Velocity` and `Pressure`. Status codes are listed in `masterStateMachine.m`.

## Operator app buttons

Home, the two Go buttons and Jog Up/Down start their operation and, while it
runs, stop it again (the model's `RequestedMode = 0`). The app follows the
operation through `StatusCode`: the button turns amber while the mode is being
entered and green once its running status appears, and returns to normal when
that status goes (stopped, finished, faulted, E-STOP or rejected). While
position or pressure control runs, changing the target turns its button back
to Go, which sends the new target without leaving the mode; the other jog
button reverses a running jog. STOP turns dark red while E-STOP is requested
but not yet reported by the model, and yellow while the model reports
E-STOP (`StatusCode` 20); pressing it then releases E-STOP.

## Testing without MATLAB

`tests/run_all.sh` (repository root) needs `python3` and `octave-cli`. It
translates `slave-src` into a simulated slave with `tools/st2m.py` and runs the
master logic against it in closed loop. The Simulink shell and the app are not
covered; they can only be exercised on the lab PC.
