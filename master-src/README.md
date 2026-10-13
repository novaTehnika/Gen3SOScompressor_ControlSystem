# Master

The master runs on the lab PC under Simulink Desktop Real-Time and talks to the
MP2600iec slave over Modbus TCP (`docs/master/ModbusRegisterMap.md`), with the
safety relay, the stir bar motor and the pressure transducer on the NI
PCI-6221 DAQ.

| Path | Contents |
|------|----------|
| `logic/` | All master logic, as plain `.m` functions that run under MATLAB and GNU Octave. `masterStep.m` is the entry point called once per model step; `masterStateMachine.m` sequences the handshake, motion and fault handling; `masterConfig.m` holds every constant. |
| `shell/buildShell.m` | Generates `shell/SOS_Gen3_Master_Shell.slx`: Modbus Client blocks at the 50 ms exchange, DAQ blocks, one MATLAB Function block calling `masterStep`, the Constant blocks the app writes and the Gain blocks it reads. The model holds no logic and is rebuilt from this script, not edited by hand. |
| `shell/modbusShellDiscover.m` | Prints the Modbus block and client settings of a hand-configured shell so they can be pasted into `buildShell.m`. |
| `shell/addDaq.m` | Creates one Simulink Desktop Real-Time DAQ block. Holds the board setup (PCI-6221, driver options, missed-tick limit) and the digital channel numbering. |
| `SOS_Gen3_Compressor_Interface.m` | Operator app (programmatic App Designer export). Starts the shell model, writes operator requests to its Constant blocks and polls its outputs. |
| `legacy/`, `SOS_Gen3_Compressor_Master.slx` | Earlier Stateflow-based app model and its `.mlapp`, and the hand-built `SimulinkMaster.slx` bench model with manual mode control, kept for reference. |

## Running on the lab PC

```matlab
cd master-src
run('shell/buildShell.m')          % once, and after any change to the block interface
addpath logic shell
SOS_Gen3_Compressor_Interface      % then press "Connect to Compressor"
```

### Modbus settings in the shell

`buildShell` sets the Modbus blocks' register rows (the Write block writes as
many registers as its input is wide, so its row has no count; the Read row
reads 18). The Modbus client is still made by hand on a fresh build:

1. Open the model. In either Modbus block click **Configure clients**. In the
   Modbus pane add a client named `ModbusClient`: TCP/IP, the controller's
   IP, port 502; under Transport Settings timeout 0.1 s, byte order
   big-endian, word order little-endian.
2. In both blocks click **Refresh** and select the client. Save.
3. If the builder printed any line to connect, the block had no ports when it
   was built; connect it as printed.

`modbusShellDiscover` prints the model's Modbus settings. Once the client
table's column labels and the transport settings' parameter names are known,
`MODBUS_SETUP` in `buildShell.m` can script the client too.

Timing: model step 25 ms, Modbus exchange 50 ms. A dropped TCP connection
stops the model with an error from the Read block; the DAQ outputs go to
their final values (safety relay off) and the app reports the stopped model.
Press **Connect** again once the slave is reachable.

## App to model interface

The app writes `RequestedMode` (0 stop, 1 home to the limit switch, 2 position,
3 jog, 4 pressure, 5 reset fault, 6 go home) and then increments `CommandSeq`; the model acts on an operation
only when `CommandSeq` changes, except that `RequestedMode = 0` always stops.
`TargetPosition`, `TargetVelocity`, `JogDirection` and `TargetPressure` are read
continuously while their operation is active. `ESTOP` and `Mixer` act at once.

The app reads `StatusCode`, `FaultCode`, `State`, `Position`,
`Velocity`, `Pressure` and `LinkOk`. Status codes are listed in
`masterStateMachine.m`; a fault shows as status 10 with the code in
`FaultCode`, and a lost slave link as status 34 until the operator presses
Stop.

## Operator app buttons

Home, Go Home, the two Go buttons and Jog Up/Down start their operation and,
while it runs, stop it again (the model's `RequestedMode = 0`). The app follows
the operation through `StatusCode`: the button turns amber while the mode is
being entered and green once its running status appears, and returns to normal
when that status goes (stopped, finished, faulted, E-STOP or not confirmed by
the slave). While position or pressure control runs, changing the target turns
its button back to Go, which sends the new target without leaving the mode; the
other jog button reverses a running jog. STOP turns dark red while E-STOP is
requested but not yet reported by the model, and yellow while the model reports
E-STOP (`StatusCode` 20); pressing it then releases E-STOP.

## Testing without MATLAB

`tests/run_all.sh` (repository root) needs `python3` and `octave-cli`. It
translates `slave-src` into a simulated slave with `tools/st2m.py` and runs the
master logic against it in closed loop. The Simulink shell and the app are not
covered; they can only be exercised on the lab PC.
