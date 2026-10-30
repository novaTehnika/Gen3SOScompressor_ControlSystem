# Shared bench resources

What every bench under `bench/` uses: the controller connection, the raw
Modbus link the run scripts drive the slave with, the model that holds the
safety relay while a script takes the shell's place as Modbus master, the
step probe that captures the axis at the 2 ms scan, and the directory the
runs log to. Each bench keeps its own constants and fits in its own
directory and takes these from here.

| File | Contents |
|------|----------|
| `benchConfig.m` | Controller IP, port and unit; slave scan; the captures directory; the safety-hold model; the step probe's registers, length and scales |
| `benchGvl.m` | The slave's `G_cfg*` settings parsed from `slave-src/GVL/GlobalVariables_Reference.st` (slew limits, velocity clamp, soft limits, pressure filter, scan time), with reference fallbacks and a warning when a name is not found; the file must match what is loaded on the controller |
| `benchLink.m` | Raw Modbus TCP link over `tcpclient` (FC04 read, FC06 and FC16 write), about 4 ms per transaction against the controller |
| `benchReadCsv.m` | Reads a run log or a capture into a struct of columns named by the header |
| `buildSafetyHold.m` | Generates `SafetyHold.slx` (gitignored): a Desktop Real-Time model that holds the safety relay (P0.0) on and the stir bar off; the run scripts start it before the first command and stop it at the end |
| `slave/PRG_StepProbe.st` | Slave program: on request captures 500 scans of commanded velocity, velocity, torque, position and pressure into the probe registers |
| `captures/` | Run logs and captures of every bench (gitignored) |

## Setup, once

1. Set `cfg.address` in `benchConfig.m` to the controller IP (the address
   MotionWorks connects to).
2. In MATLAB with Simulink Desktop Real-Time, run `buildSafetyHold`, open
   `SafetyHold.slx`, select **Connected IO** on the Desktop Real-Time tab
   and save.
3. For scan-rate captures, load `slave/PRG_StepProbe.st` as described in
   `bench/friction/README.md` section 4.5: the FC#04 input register group
   enlarged to at least 3410 registers, the globals of the program header
   declared, the program on the 2 ms task beside `PRG_Main`. It reads
   globals only. Only one bench program using the probe registers (this
   one, `PRG_PressureProbe` or `PRG_ModbusBench`) is loaded at a time.

## How a run works

The shell model must be stopped: a second Modbus client beside it delays
the shell's exchange and the slave raises `FAULT_COMM` while motion is
commanded. A run script (`pressureNoiseRun`, `frictionRun`, `charRun`)
opens one connection with `benchLink`, exchanges the protocol registers
every 50 ms through `masterStep` as the shell does, reads its probe in the
gaps, and logs every exchange. Ctrl-C stops the axis: the cleanup sends one
exchange with MotionEnable low and mode 0, and the slave's own watchdog
stops it within 300 ms if the script dies.

## Log formats

Run logs (`frictionRun`, `charRun`): one row per exchange with `t`, `step`,
`status` (`masterStateMachine` code), `mode` (slave mode of the step),
`ref` (HR5-6 as sent), `position`, `velocity`, `torque`, `pressure`
(MPa gauge), `inMotion`, `inBand`, `exchangeMs`. Captures: `t`, `vCmd`,
`velocity`, `torque`, `position`, `pressure`, one row per scan. The fits of
the friction and characterisation benches read either script's files.
