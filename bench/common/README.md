# Shared bench resources

What every bench under `bench/` uses: the controller connection, the raw
Modbus link the run scripts drive the slave with, the model that holds the
safety relay while a script takes the shell's place as Modbus master, the
step probe that records the axis at the 4 ms scan (part of the slave
program, `FB_Probe`), and the directory the runs log to. Each bench keeps its own constants and fits in its own
directory and takes these from here.

| File | Contents |
|------|----------|
| `benchConfig.m` | Controller IP, port and unit; slave scan; the captures directory; the safety-hold model; the step probe's registers, scales and timing |
| `benchGvl.m` | The slave's `G_cfg*` settings parsed from `slave-src/GVL/GlobalVariables_Reference.st` (slew limits, velocity clamp, soft limits, pressure filter, scan time), with reference fallbacks and a warning when a name is not found; the file must match what is loaded on the controller |
| `benchLink.m` | Raw Modbus TCP link over `tcpclient` (FC04 read, FC06 and FC16 write), about 4 ms per transaction against the controller |
| `benchReadCsv.m` | Reads a run log or a capture into a struct of columns named by the header |
| `buildSafetyHold.m` | Generates `SafetyHold.slx` (gitignored): a Desktop Real-Time model that holds the safety relay (P0.0) on and the stir bar off; the run scripts start it before the first command and stop it at the end |
| `benchProbe.m` | Master side of the step probe, used by `frictionRun` and `charRun`: arms captures, reads them out of the slave's ring through the register window, fetches the history before a fault, decodes them |
| `benchFiles.m` | Expands a file name, a pattern or a cell array of either into a cell array of paths; the characterisation fits call it, so they take `'../captures/K300_char_*.csv'` |
| `captures/` | Where the run scripts write their logs and captures (gitignored). The committed reference logs, the 26 October characterisation and friction runs and the 30 and 31 October one-pass stops runs, are in `bench/captures/`; the fits take either directory, and nothing is moved between them |

## Setup, once

1. Set `cfg.address` in `benchConfig.m` to the controller IP (the address
   MotionWorks connects to).
2. In MATLAB with Simulink Desktop Real-Time, run `buildSafetyHold`, open
   `SafetyHold.slx`, select **Connected IO** on the Desktop Real-Time tab
   and save.
3. For scan-rate captures the main program carries the probe
   (`slave-src/FB/FB_Probe.st`, called last in `PRG_Main`, enabled by
   `G_cfgProbeEnable`, default TRUE): the Modbus FC#04 input register group
   must extend to IR524 and the holding group to HR114, as described in
   `docs/master/ModbusRegisterMap.md` section 6 and
   `bench/friction/README.md` section 4.5. The controller has one task, so
   every mapped register is copied every scan and the group size is paid on
   every protocol exchange whether or not a program uses it: a
   3410-register group (a capture held in registers) took the exchange from
   20 ms (p99 30) to 50 ms (p99 70), too slow for the 50 ms exchange
   against the slave's 300 ms watchdog. The probe therefore keeps its
   records in plain memory and maps only IR401-524 and HR111-114. The
   bench program `PRG_ModbusBench` uses other registers (HR101-110,
   IR101-108) and is not loaded with the main program in service.

## The step probe

The slave records one record per scan into a ring of 8192 (33 s at the 4 ms scan).
A record is 8 words: commanded velocity, velocity, torque, position
(absolute, two words), pressure, the raw pressure input and a status word
(in motion, performance status, fault active, homing complete, confirmed
mode code). Nothing else is mapped but a header, a window of 13 records and
state registers; the layout is `docs/master/ModbusRegisterMap.md` section 6
and `FB_Probe.st` alone defines the record.

| Register | Content |
|----------|---------|
| HR111 | Arm: any change of the value is acted on, at any time |
| HR112-113 | Index of the first record wanted in the window (low word first) |
| HR114 | Records the arm wants (0 = open ended) |
| IR401-402 | Records written so far |
| IR403-404 | Mark: index of the record written in the scan of the last arm |
| IR405-406 | Start: index of the first record in the window |
| IR407-410 | ArmEcho (the arm value last acted on), Count (records in the window), ring depth, words per record |
| IR411-514 | The window: Count records of 8 words |
| IR519-520 | StartTail: a copy of Start written after the window |
| IR521 | Recording state: 0 off, 1 recording, 2 capturing, 3 held |
| IR522 | Transfer state: 0 idle, 1 filling, 2 pending, 3 overrun |
| IR523-524 | FaultMark: index of the record written in the scan the slave entered `ST_FAULT` |

**What a record's columns are.** The probe is called last in `PRG_Main`.
The velocity, position and torque it records are the motion status
`PRG_Main` reads in the scan, which the LD POU's read blocks last updated
at the end of the previous scan (`PRG_Main` sees motion status one scan
late), while the commanded velocity is the one `PRG_Main` computes in this
very scan. A record's feedback is one scan (`G_cfgScanTime`, 4 ms) older
than the command in the same row. A delay or a stopping travel read
from a capture's rows (from the command row to the row where the feedback
responds) includes that scan: allow one scan when comparing it with the
drive's own response, with a model's dead time or with a figure from the
50 ms log. `charDriveLag`, `stepResponseFit` and `charStopDistance`
report what the rows show and do not remove it.

`benchProbe` does the master's part in the gap between two protocol
exchanges, one read of IR401-524 (124 registers) and at most one write:

- An arm is a write of HR111-114 in one transaction: a new value for HR111
  (the slave's ArmEcho + 1 for the first), the request in force (so the
  slave does not restart a transfer) and the capture's length. It is
  complete when ArmEcho equals the value written, and the Mark then gives
  the capture's first record (Mark less `nPre`). An arm not acknowledged
  within `armRetry` is written again; after `timeout` it is given up and
  counted as a probe failure. The run applies the command the capture is
  meant to see only once the arm is acknowledged.
- A read request is a write of the next index wanted to HR112-113. The
  slave copies the records into the window a few per scan and publishes it
  when it is complete (transfer state `XFER_FILLING`, then `XFER_IDLE`; the
  request is not rewritten while the slave is filling or waiting for the
  records, `XFER_PENDING`). A read is used when Start equals StartTail (a
  read that straddles the slave's update does not) and Start equals the
  index requested; otherwise the request is written again. A window with
  fewer than 13 records is used and the next request advances by its
  count, so a capture can be read while it is still being recorded and a
  stream (`nRecords` Inf) can be followed. The word count of a record is
  read from IR410.
- A capture is lost when the ring has overwritten its next record before
  it is read (or the slave reports `XFER_OVERRUN` for the request). The
  reader keeps up at 13 records a gap (260 records/s at the 50 ms
  exchange; 650 records/s while draining at `drainGap`) against 500
  records/s recorded, so a stream longer than the ring needs the faster
  rate.
- Captures are queued, read oldest first, and an arm is accepted while
  earlier captures are still being read. `drain` reads everything pending,
  with the gaps `drainGap` apart, and runs after the last step and before
  the files are written.
- The history before a fault: the slave latches FaultMark, the index of the
  record written in the scan it entered `ST_FAULT`, and the ring holds the
  last 33 s. `benchProbe('fault', p, info)` queues a capture of
  `info.nPre` records before and `info.nPost` after it (the slave is not
  armed); with `info.wait` true it waits for a fault after the first read,
  so a script queues it before provoking the fault. `p.faultMark`, `p.probeState`
  and `p.xferState` hold the slave's registers after any read
  (`benchProbe('poll', p, link, t)` reads once). The status column
  decodes with `benchProbe('status', [], data(:, 8))`.

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
`velocity`, `torque`, `position`, `pressure`, `volts` (the raw pressure
input, V) and `status` (the record's status word), one row per scan. The
first six columns are the earlier captures' and in the same order, so the
fits of the friction and characterisation benches read old and new files
alike.
