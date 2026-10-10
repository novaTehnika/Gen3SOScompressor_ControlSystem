# Modbus TCP Bench Test

Measures whether Modbus TCP between the lab PC and the MP2600iec is fast and
reliable enough to replace the discrete and analog master ↔ slave signals at
the master's 3 ms step. Background and sources are in
[EthernetCommsReference.md](../../docs/master/EthernetCommsReference.md). The
proposed protocol registers are in
[ModbusRegisterMap.md](../../docs/master/ModbusRegisterMap.md); the bench uses
the same heartbeat registers (HR1, IR1–2) and a separate probe area (101+).

| File | Contents |
|------|----------|
| `slave/PRG_ModbusBench.st` | Slave program: heartbeat echo, scan counter, probe patterns |
| `modbusBenchConfig.m` | Controller IP, timeout, register numbers, step |
| `modbusProbe.m` | MATLAB-only check of address mapping and word order, plus timed round trips |
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
   matching groups (holding registers: `%IW`/`%ID`, input registers: `%QW`/`%QD`).
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
"Unexpected" or "other" means the register-to-address mapping
(`28672 + 2*(k-1)`) is wrong: check the group address ranges in the Global
Variables worksheet.

The probe also times 2000 heartbeat round trips from plain MATLAB, without
Simulink.

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

| Result | Criteria (cases A–C) | Consequence |
|--------|----------------------|-------------|
| Full rate | No failures in 60 s; transaction p99 < 3 ms; no missed ticks; echo lag ≤ 1 step in ≥ 99.9 % of steps | Modbus at the 3 ms step |
| Reduced rate | Transaction p99 < 10 ms; no failures | Modbus at a slower rate than the model step; the pressure loop delay must be assessed |
| Fail | Otherwise | Use UDP |

The protocol in ModbusRegisterMap.md needs one write and one read per step,
the same as the bench, with slightly larger payloads.

## 6. Optional: Modbus Client blocks

To compare with the Industrial Communication Toolbox blocks, replace
`BenchStep` with a Modbus Client Write (HR1, `uint16`, from the step counter)
and a Modbus Client Read (IR1, count 2, `uint16`). Configure the client in the
model's Configuration Parameters, **Modbus** pane: controller IP, port 502,
timeout as `cfg.timeout`, word order as found in step 2.

## 7. Results

| Case | Failures | Txn median / p99 / max (ms) | Missed ticks | Echo lag ≤ 1 (%) | Notes |
|------|----------|-----------------------------|--------------|------------------|-------|
| A | | | | | |
| B | | | | | |
| C | | | | | |
| D | | | | | |
