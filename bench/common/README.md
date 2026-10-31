# Shared bench resources

What every bench under `bench/` uses: the controller connection, the raw
Modbus link the run scripts drive the slave with, the model that holds the
safety relay while a script takes the shell's place as Modbus master, the
step probe that captures the axis at the 4 ms scan, and the directory the
runs log to. Each bench keeps its own constants and fits in its own
directory and takes these from here.

| File | Contents |
|------|----------|
| `benchConfig.m` | Controller IP, port and unit; slave scan; the captures directory; the safety-hold model; the step probe's registers, scales and timing |
| `benchGvl.m` | The slave's `G_cfg*` settings parsed from `slave-src/GVL/GlobalVariables_Reference.st` (slew limits, velocity clamp, soft limits, pressure filter, scan time), with reference fallbacks and a warning when a name is not found; the file must match what is loaded on the controller |
| `benchLink.m` | Raw Modbus TCP link over `tcpclient` (FC04 read, FC06 and FC16 write), about 4 ms per transaction against the controller |
| `benchReadCsv.m` | Reads a run log or a capture into a struct of columns named by the header |
| `buildSafetyHold.m` | Generates `SafetyHold.slx` (gitignored): a Desktop Real-Time model that holds the safety relay (P0.0) on and the stir bar off; the run scripts start it before the first command and stop it at the end |
| `benchProbe.m` | Master side of the step probe, used by `frictionRun` and `charRun`: arms captures, reads them out of the slave's ring through the register window, decodes them |
| `slave/PRG_StepProbe.st` | Slave program: records commanded velocity, velocity, torque, position and pressure every scan into a ring of 8192 records and serves them through an 18-record window |
| `captures/` | Run logs and captures of every bench (gitignored) |

## Setup, once

1. Set `cfg.address` in `benchConfig.m` to the controller IP (the address
   MotionWorks connects to).
2. In MATLAB with Simulink Desktop Real-Time, run `buildSafetyHold`, open
   `SafetyHold.slx`, select **Connected IO** on the Desktop Real-Time tab
   and save.
3. For scan-rate captures, load `slave/PRG_StepProbe.st` as described in
   `bench/friction/README.md` section 4.5: the FC#04 input register group
   to IR520 and the holding group to HR113, the globals of the program
   header declared, the program on the cyclic task beside `PRG_Main`. It
   reads globals only. Only one bench program using the probe registers
   (this one, `PRG_PressureProbe` or `PRG_ModbusBench`) is loaded at a
   time. The controller has one task, so every mapped register is copied
   every scan and the group size is paid on every protocol exchange
   whether or not a program uses it: a 3410-register group (the capture
   held in registers) took the exchange from 20 ms (p99 30) to 50 ms (p99
   70), too slow for the 50 ms exchange against the slave's 300 ms
   watchdog. The probe therefore keeps its records in plain memory and maps
   only IR401-520 and HR111-113, so the group can stay configured.

## The step probe

The slave records one record per scan into a ring of 8192 (32 s at 4 ms):
commanded velocity, velocity, torque, position (absolute) and pressure.
Nothing else is mapped but a header and a window of 18 records:

| Register | Content |
|----------|---------|
| HR111 | Arm: any change of the value is acted on, at any time |
| HR112-113 | Index of the first record wanted in the window (low word first) |
| IR401-402 | Records written so far |
| IR403-404 | Mark: index of the record written in the scan of the last arm |
| IR405-406 | Start: index of the first record in the window |
| IR407-410 | ArmEcho (the arm value last acted on), Count (records in the window), ring depth, words per record |
| IR411-518 | The window: Count records of 6 words (vCmd, velocity, torque, position low word, position high word, pressure) |
| IR519-520 | StartTail: a copy of Start written after the window |

`benchProbe` does the master's part in the gap between two protocol
exchanges, one read of IR401-520 (120 registers) and at most one write:

- An arm is a write of a new value to HR111, the slave's ArmEcho + 1 for
  the first. It is complete when ArmEcho equals the value written, and the
  Mark then gives the capture's first record (Mark less `nPre`). An arm not
  acknowledged within `armRetry` is written again; after `timeout` it is
  given up and counted as a probe failure. The run applies the command the
  capture is meant to see only once the arm is acknowledged.
- A read request is a write of the next index wanted to HR112-113. The
  slave fills the window from it, only when the request moved or the
  window can hold more records. A read is used when Start equals StartTail
  (a read that straddles the slave's update does not) and Start equals the
  index requested; otherwise the request is written again. A window with
  fewer than 18 records is used and the next request advances by its
  count, so a capture can be read while it is still being recorded and a
  stream (`nRecords` Inf) can be followed.
- A capture is lost when the ring has overwritten its next record before
  it is read. The reader keeps up at 18 records a gap (360 records/s at
  the 50 ms exchange; 900 records/s while draining at `drainGap`) against
  250 records/s recorded, so a stream longer than the ring needs the
  faster rate.
- Captures are queued, read oldest first, and an arm is accepted while
  earlier captures are still being read. `drain` reads everything pending,
  with the gaps `drainGap` apart, and runs after the last step and before
  the files are written.

## How a run works

The shell model must be stopped: a second Modbus client beside it delays
the shell's exchange and the slave raises `FAULT_COMM` while motion is
commanded. A run script (`pressureNoiseRun`, `frictionRun`, `charRun`)
opens one connection with `benchLink`, exchanges the protocol registers
every 50 ms through `masterStep` as the shell does, reads its probe in the
gaps (one transaction pair per gap, skipped when the exchange before it
took over 150 ms), and logs every exchange. Ctrl-C stops the axis: the cleanup sends one
exchange with MotionEnable low and mode 0, and the slave's own watchdog
stops it within 300 ms if the script dies.

## Log formats

Run logs (`frictionRun`, `charRun`): one row per exchange with `t`, `step`,
`status` (`masterStateMachine` code), `mode` (slave mode of the step),
`ref` (HR5-6 as sent), `position`, `velocity`, `torque`, `pressure`
(MPa gauge), `inMotion`, `inBand`, `exchangeMs`. Captures: `t`, `vCmd`,
`velocity`, `torque`, `position`, `pressure`, one row per scan. The fits of
the friction and characterisation benches read either script's files.
