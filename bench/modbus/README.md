# Modbus TCP Bench Test

Measures whether Modbus TCP between the lab PC and the MP2600iec is fast and
reliable enough to carry the master ↔ slave protocol at a 50 ms exchange
period, and how fast it can go. Background and sources are in
[EthernetCommsReference.md](../../docs/master/EthernetCommsReference.md). The
proposed protocol registers are in
[ModbusRegisterMap.md](../../docs/master/ModbusRegisterMap.md); the bench uses
the same heartbeat registers (HR1, IR1–2) and a separate probe area (101+).

| File | Contents |
|------|----------|
| `slave/PRG_ModbusBench.st` | Slave program: heartbeat echo, scan counter, probe patterns |
| `modbusBenchConfig.m` | Controller IP, timeout, register numbers, step |
| `modbusProbe.m` | MATLAB-only check of address mapping and word order, plus timed round trips |
| `modbusRawProbe.m` | Timed heartbeat exchange with hand-built frames over `tcpclient`: client floor and pipelined requests |
| `modbusCaptureTiming.py` | Splits a Wireshark capture into controller time and client time (needs `tshark`) |
| `captures/` | Wireshark captures from bench runs (gitignored) |
| `buildModbusBench.m` | Generates `ModbusBench.slx` (gitignored) |
| `modbusBenchStep.m` | Called each model step: writes the heartbeat, reads echo and counter |
| `modbusBenchReport.m` | Summarises a model run |

The bench model reads the DAQ inputs only, to load and pace the model like the
shell. It drives no DAQ output.

## 1. Slave setup

1. Hardware Configuration, online: configure the controller as a **Modbus
   slave (server)**. Select **Set all outputs off when PLC stops**. Under
   **IO Task Assignment**, assign the four server blocks to the 2 ms cyclic task.
2. Save and cycle power. Four Modbus groups appear in the Global Variables
   worksheet.
3. Declare the globals listed in the header of `slave/PRG_ModbusBench.st` in the
   matching groups (holding registers under FC#06,16: `%IW`/`%ID` from
   `%IB28672`; input registers under FC#04: `%QW`/`%QD` from `%QB73792`).
   The group address ranges shown in the worksheet are the reference.
4. Add `PRG_ModbusBench` to the project and assign it to the 2 ms cyclic task.
   It uses only its own globals and can run alongside `PRG_Main`.
5. Compile, download, start.

`PRG_ModbusBench` must be removed before the protocol registers in
ModbusRegisterMap.md are added, since both use HR1 and IR1–2.

## 2. Mapping and word order

On the lab PC, set `cfg.address` in `modbusBenchConfig.m` to the controller IP
(the address MotionWorks connects to), then:

```matlab
cd bench/modbus
r = modbusProbe;
```

Expected output: UINT `16#1234`, DINT words `16#1234 16#5678` in one order,
REAL words `16#449A 16#5000` in the same order, and the slave decoding the
master's DINT in that order too. Set `cfg.wordOrder` to the reported order.
"Unexpected" or "other", or all-zero reads, mean the declared addresses are
outside the Modbus groups: check the group address ranges in the Global
Variables worksheet against the header of `slave/PRG_ModbusBench.st`.

The probe also times 2000 heartbeat round trips from plain MATLAB, without
Simulink. It uses a 1 s timeout so that a slow server shows up as a large
transaction time rather than as failures, prints the first error message if
any transaction fails, and reports the share of cycles slower than
`cfg.timeout`.

### Controller time and client time

The probe's figures include the MATLAB client's own overhead. To separate
the two, capture the run in Wireshark on the lab PC (filter `tcp.port == 502`),
save it under `captures/`, and run:

```sh
python3 modbusCaptureTiming.py captures/cap_run01.pcapng
```

It reports, per transaction, request to TCP ACK and request to response
(the controller) and response to next request (the client). The ACK delay
histogram shows the controller's service tick: a spread evenly over 0 to
T ms means requests wait for a T ms cycle.

`modbusRawProbe` times the same heartbeat exchange with hand-built frames
over `tcpclient`, in three variants: sequential (as `modbusProbe`), both
requests as two `write` calls, and both requests in one `write` call. The
last two show whether the controller answers a write and a read in the same
service tick, which is what the protocol exchange needs.

```matlab
r = modbusRawProbe;
```

## 3. Simulink run

```matlab
buildModbusBench
out = sim('ModbusBench');           % cfg.duration seconds
s = modbusBenchReport(out.benchLog);
```

If the report warns that steps run faster than real time, the model was not
paced by Simulink Desktop Real-Time. Select **Connected IO** on the Desktop
Real-Time tab and run it from there, then pass the logged `benchLog` to the
report.

The generated model calls the `modbus` object from a MATLAB Function block
every step. The call blocks the model for the whole exchange, so under
Connected IO this variant mostly measures how a blocking call stalls the
model. Section 6 replaces it with the toolbox blocks.

## 4. Test cases

| Case | Conditions |
|------|------------|
| A | MotionWorks offline, `PRG_Main` loaded, axis in Idle |
| B | As A, MotionWorks online with a watch window open |
| C | As A, with the operator app open and polling the bench model |
| D | As A, unplug the Ethernet cable for about 3 s during the run, then reconnect |

Case D shows how a lost link appears: transaction failures, their duration
(the timeout), missed ticks, and whether the connection recovers without a
restart.

## 5. Pass criteria

The protocol exchange is one write and one read, sent back to back, at a
period of up to 50 ms. Fast signals stay on the digital lines, and the
pressure loop runs on the slave, so the master step rate does not apply to
the link.

| Result | Criteria (cases A–C) | Consequence |
|--------|----------------------|-------------|
| Pass | No failures in 60 s; exchange p99 < 25 ms; no missed ticks | Modbus at a 50 ms exchange period with margin for one retry |
| Marginal | Exchange p99 < 50 ms; no failures | Modbus at 50 ms with no retry margin; comm timeout must allow a lost exchange |
| Fail | Otherwise | Use UDP |

The exchange p99 is measured with the client that will be used in the shell,
not with the `modbus` object (see section 7).

## 6. Modbus Client blocks

`ModbusBench.slx` is generated and gitignored, so it can be edited by hand
for this variant. It needs Industrial Communication Toolbox R2024b or later.

1. Run `buildModbusBench` and open the model. Delete `BenchStep` and its
   lines; keep `Clock`, `Mux`, `Log` and the DAQ blocks.
2. Set the model step to 5 ms (Configuration Parameters, Fixed-step size):
   the blocks' 50 ms sample time must be a multiple of it.
3. From the Library Browser, Industrial Communication Toolbox, Modbus, add
   one **Modbus Client Write** and one **Modbus Client Read**.
4. In the Write block click **Configure clients**. In the Modbus pane click
   **Add**: transport TCP/IP, device address `cfg.address`, port 502. Under
   **Transport Settings** set the timeout to 0.1 s, byte order big-endian,
   word order little-endian. Click OK.
5. Write block: **Refresh**, select the client, Server ID 1. Registers: one
   row, address 1, holding registers, uint16, count 1. Block sample time
   0.05.
6. Read block: same client and Server ID. Registers: one row, address 1,
   input registers, uint16, count 2 (IR1 echo, IR2 slave counter). Block
   sample time 0.05.
7. Step counter for the Write input: a MATLAB Function block fed from
   `Clock` with `y = mod(round(t / 0.005), 65536)`.
8. On `AI Position`, tick **Show "Missed Ticks" port**. This is the pass/fail
   measurement: the port reports how far the model lags the kernel, and the
   model stops when it exceeds the block's Maximum missed ticks (default 10).
9. Wire the Mux in this order: step counter, Read block output (1-by-2),
   Missed Ticks. The Read output is a row vector, so `Log` produces a
   1-by-4-by-steps array; the report accepts it.
10. Desktop Real-Time tab: Connected IO, Run in Real Time. Then:

```matlab
s = modbusBenchReport(out.benchLog, {'seq', 'echo', 'counter', 'missed'}, 0.005);
```

The blocks have no status output. If echo or counter stay zero after the
first exchange, check the register types and look in the Diagnostic Viewer.

## 7. Results

`modbusProbe` (MATLAB R2025b, `modbus` object, sequential write then read),
%M bidirectional option on, Modbus groups and `PRG_ModbusBench` on the same
task:

| Case | Failures | Write + read median / p99 / max (ms) | Echo same cycle (%) | Notes |
|------|----------|--------------------------------------|---------------------|-------|
| A | 0 / 2000 | 20.0 / 33.9 / 39.9 | 100 | Word order low word first. Slave counter period 4 ms. An earlier run gave the same median with max 67.7 ms. |
| B | | | | |
| C | | | | |
| D | | | | |

Capture of case A (`captures/cap_run01.pcapng`, 4003 transactions, no
retransmissions):

| Measure | median | p99 | max (ms) |
|---------|--------|-----|----------|
| Controller, request → TCP ACK | 2.2 | 4.0 | 4.1 |
| Controller, request → response | 3.5 | 7.3 | 7.9 |
| Client (`modbus` object), response → next request | 6.9 | 17.2 | 61.3 |

The ACK delay is spread evenly over 0.5 to 4 ms: the controller services the
connection on a 4 ms tick. The response comes in the same tick 81 % of the
time and one tick later otherwise, so a transaction costs one or two ticks.

`modbusRawProbe` (case A, 1000 cycles per variant, no failures) and its
capture (`captures/cap_run02.pcapng`):

| Variant | Exchange, MATLAB median / p99 / max (ms) | Controller, first request → last response, median / p99 / max (ms) | Echo same cycle (%) |
|---------|------------------------------------------|--------------------------------------------------------------------|---------------------|
| sequential | 19.8 / 34.1 / 50.5 | 13.5 / 24.1 / 27.0 | 100 |
| two writes | 9.8 / 25.0 / 29.9 | 8.4 / 11.7 / 11.8 | 89 |
| one write | 9.8 / 19.8 / 20.4 | 5.9 / 7.8 / 7.9 | 16 |

Findings:

- The controller answers **one request per tick** on a connection. With both
  frames in one segment the write is answered in the first tick and the read
  always in the next, so a write-plus-read exchange costs two ticks, 6 to
  12 ms, whatever the client does.
- The client-side delay of about 7 ms (p99 17 ms) after each blocking socket
  read is the same with `tcpclient` as with the `modbus` object. It is
  MATLAB's blocking read, not the Modbus layer. Sending both requests before
  waiting pays it once per exchange instead of twice.
- A read answered in the tick after the write returns the value from before
  the write reached the program: the written value shows up two ticks later.
  The slave's reply to a command therefore appears in the following exchange.
- Against the 50 ms target the exchange passes with retry margin. What is not
  yet known is how the exchange behaves inside the Simulink model, where a
  blocking call of 10 to 25 ms stalls the 3 ms step.

Modbus Client blocks (section 6), case A, 50 ms block sample time,
`modbusBenchReport(..., {'seq','echo','counter','missed'}, dt)`, 60 s runs:

| Model step | Max lag (ticks) | Steps lagging (%) | Lagging > 5 ticks (%) | Exchanges fresh in the same step (%) | Longest gap between read results (slave scans) |
|------------|-----------------|-------------------|-----------------------|--------------------------------------|------------------------------------------------|
| 25 ms | 2 | 0.21 | 0 | 100 | 24 (one late exchange) |
| 5 ms | 17 | 36.3 | 1.55 | 100 | 26 (one late exchange) |

The exchange behaves the same at both steps: every read returns the seq
written in the same block step, and one exchange per minute arrives a
period late. The difference is what the blocking exchange does to the rest
of the model. At 25 ms the stall is at most two ticks; at 5 ms the model is
behind on a third of its steps.

The DAQ blocks copied from `SimulinkMaster.slx` have **Maximum missed ticks
set to 1e6**, so the model never stops for lag and drops DAQ samples while
it is behind. The Missed Ticks port is the only indication. The shell
inherits the same setting.

Case D (cable unplugged for about 3 s during a 25 ms run): the Read block
raises an error on the first exchange after the connection drops and the
simulation stops:

```
MATLAB System block 'ModbusBench/Modbus Client Read/internalModbus' error
occurred when invoking 'stepImpl' method of 'icomm.system.ModbusRead'.
Caused by: Invalid operation. Object must be connected to the remote server.
```

The blocks wrap the same `icomm.interface.modbus` client as the `modbus`
object. There is no status output, no retry and no reconnect; a lost link
ends the run.

Conclusion: Modbus Client blocks at a 50 ms sample time in a Connected IO
model with a step of 25 ms or longer. A lost link stops the model, which
under Connected IO sets the DAQ outputs to their Final values. The generated
model and the `modbus` object are not a candidate for the shell.
