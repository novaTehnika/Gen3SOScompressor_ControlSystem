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

## 6. Optional: Modbus Client blocks

To compare with the Industrial Communication Toolbox blocks, replace
`BenchStep` with a Modbus Client Write (HR1, `uint16`, from the step counter)
and a Modbus Client Read (IR1, count 2, `uint16`). Configure the client in the
model's Configuration Parameters, **Modbus** pane: controller IP, port 502,
timeout as `cfg.timeout`, word order as found in step 2.

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

Simulink bench model and Modbus Client blocks: not yet run.
