# Modbus Register Map

## Master ↔ Slave over Modbus TCP

---

## 1. Overview

This map carries the protocol in [MasterProtocolGuide.md](MasterProtocolGuide.md)
over Modbus TCP: the same mode codes, handshake and fault-mirror reset, with
the reference and feedback in engineering units. Word order and timing were
measured by the bench test in `bench/modbus`.

| Role | Device |
|------|--------|
| Server | MP2600iec, built-in Modbus TCP server, port 502 |
| Client | Simulink shell on the lab PC |

Each exchange is one **Write Multiple Registers (FC16)** of holding registers
1–12 followed at once by one **Read Input Registers (FC04)** of input registers
1–18, so both land in the same controller service tick. The exchange period is
up to 50 ms, a multiple of the master step; see
[EthernetCommsReference.md](EthernetCommsReference.md).

### Addressing

Register numbers are 1-based, as used by MATLAB and the Simulink Modbus blocks.
Register *k* maps to a byte offset inside the group that Hardware
Configuration creates in the Global Variables worksheet:

- Holding registers (master → slave, group FC#03,06,16, with **Use %M for
  bidirectional Holding Registers** enabled): `%MW3.` / `%MD3.` at
  `483328 + 2*(k-1)`
- Input registers (slave → master, group FC#04): `%QW` / `%QD` at
  `73792 + 2*(k-1)`

The group address ranges are shown in the worksheet and must be checked
there; published application notes give other bases. The simulated slave
derives its registers from the globals' `AT` addresses with these bases
(`MODBUS_GROUPS` in `tools/st_target_c.py`), packing 32-bit values low word
first, and `tests/closedLoopModbusStep.m` checks `modbusMap.m` against them
before exchanging the 16-bit words with it.

### Data types

| Type | Registers | Notes |
|------|-----------|-------|
| `UINT` | 1 | Codes, flags (0/1), counters |
| `REAL` | 2 | IEEE 754 single precision. Low word first (client word order little-endian, byte order big-endian) |

`REAL` resolves about 2e-5 mm at 300 mm, finer than any quantity carried here.
`LREAL` values are converted to `REAL` on the slave.

### Slave configuration

In Hardware Configuration: Modbus slave (server) enabled, **Use %M for
bidirectional Holding Registers** checked, **Set all outputs off when PLC
stops**, and the four Modbus groups assigned to the cyclic task that runs
`PRG_Main` (MotionWorks IEC 3 Express has only that one task), so the
registers update once per scan. Every register in the groups is copied
every scan, so the groups are sized to the registers in use (the step probe of section 6 extends them to IR524 and HR114). The controller services the
connection on a 4 ms tick, so a transaction takes 1 to 8 ms regardless of the
task interval.

---

## 2. Holding Registers (master → slave)

| HR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 1 | `%MW3.483328` | UINT | `G_mbHeartbeat` | Master step counter, incremented every step, wraps 65535 → 0 |
| 2 | `%MW3.483330` | UINT | `G_mbModeCode` | Mode code 0–7 (`E_OperatingMode`). During fault reset the master mirrors the **full** fault code here. The mode decoder maps any value above 7 to mode 0; the slave ignores the decoded mode on the fault-acknowledge path |
| 3 | `%MW3.483332` | UINT | `G_mbMotionEnable` | 1 = MotionEnable |
| 4 | `%MW3.483334` | UINT | `G_mbFaultReset` | 1 = FaultReset (level) |
| 5–6 | `%MD3.483336` | REAL | `G_mbReference` | Position mode: mm. Velocity mode: mm/s. Torque mode: % of rated. Pressure mode (code 7): target pressure, MPa gauge |
| 7–8 | `%MD3.483340` | REAL | `G_mbPositionSpeed` | Speed of a position move, mm/s. The slave limits it to `0` to `G_cfgVelLimitMax`; 0 holds position. Written every exchange |
| 9–10 | `%MD3.483344` | REAL | `G_mbSalineVolume` | Saline volume of the fill, mL, for the pressure gain schedule. Latched by the slave on entry to pressure mode; 0 (never written) makes the schedule fall back to the configured gains |
| 11–12 | `%MD3.483348` | REAL | `G_mbOxygenRatio` | Oxygen charge as STP volume per saline volume (1.0 for 1:1), for the pressure gain schedule. Latched on entry to pressure mode |
| 13–20 | | | | Reserved |
| 101–110 | | | | Bench areas (`bench/modbus`, `bench/pressure`): not loaded with the main program in service |
| 111 | `%MW3.483548` | UINT | `G_mbProbeArm` | Step probe (section 6): a changed value is an arm |
| 112–113 | `%MW3.483550` | UINT ×2 | `G_mbProbeReqLo`, `G_mbProbeReqHi` | Step probe: index of the first record wanted in the window (low word first) |
| 114 | `%MW3.483554` | UINT | `G_mbProbeLength` | Step probe: records the next arm wants, 0 = open ended |

---

## 3. Input Registers (slave → master)

| IR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 1 | `%QW73792` | UINT | `G_mbHeartbeatEcho` | Last `G_mbHeartbeat` received |
| 2 | `%QW73794` | UINT | `G_mbSlaveCounter` | Incremented every scan, wraps 65535 → 0 |
| 3 | `%QW73796` | UINT | `G_mbConfCode` | Mode confirmation: the mode the handshake manager has confirmed, 0 outside the handshake (and so while a fault is active) |
| 4 | `%QW73798` | UINT | `G_mbFaultCode` | `G_sysActiveFault` as its full integer value, 0 when no fault. May exceed 7 |
| 5 | `%QW73800` | UINT | `G_mbFaultActive` | 1 = fault active (`G_doFaultActive`, also on DO4) |
| 6 | `%QW73802` | UINT | `G_mbInMotion` | 1 = axis moving (`G_doInMotion`, also on DO6) |
| 7 | `%QW73804` | UINT | `G_mbPerformanceStatus` | `G_doPerformanceStatus` (also on DO3) |
| 8 | `%QW73806` | UINT | `G_mbHomingComplete` | `G_doHomingComplete` (also on DO5) |
| 9–10 | `%QW73808` | UINT | | Reserved |
| 11–12 | `%QD73812` | REAL | `G_mbActualPosition` | `G_sysActualPosition` (mm) |
| 13–14 | `%QD73816` | REAL | `G_mbActualVelocity` | `G_sysActualVelocity` (mm/s) |
| 15–16 | `%QD73820` | REAL | `G_mbActualTorque` | `G_sysActualTorque` (% of rated) |
| 17–18 | `%QD73824` | REAL | `G_mbCommandedPosition` | `G_sysCommandedPosition` (mm) |
| 19–20 | `%QD73828` | REAL | `G_mbActualPressure` | `G_sysActualPressure` (MPa gauge): the transducer on the slave's analog input, scaled by `FB_PressureInput`. Valid in every mode; below zero or above range when the loop current is out of range |
| 101–370 | | | | Bench areas (`bench/modbus`, `bench/pressure`): not loaded with the main program in service |
| 401–524 | | | | Step probe, the next table (section 6) |

---

## 4. Link Supervision

Holding registers keep their last written value when the master stops writing,
so a stopped master leaves MotionEnable and the reference in place. Both sides
therefore supervise the link.

| Side | Condition | Action |
|------|-----------|--------|
| Slave | `G_mbHeartbeat` unchanged for `G_cfgCommTimeout` (300 ms) while motion is commanded; armed by the first change seen | `FAULT_COMM` (code 4) through `FB_SafetyMonitor`, so the fault latches, mirrors and resets like any other. In every other state a stopped master is not a fault |
| Master | `G_mbSlaveCounter` not yet seen to advance | E-stop path and safety relay off, status `LINK_WAIT`; clears by itself once the counter moves |
| Master | `G_mbSlaveCounter` unchanged for `cfg.commTimeout` (300 ms) after that | E-stop path: MotionEnable low, safety relay (P0.0) off, status `LINK_LOST`. Latched until the counter advances and the operator sends Stop |

Both timeouts ride through a few late exchanges at the 50 ms period. The
Modbus client's own transaction timeout (0.5 s in `buildShell`) must stay
above the slowest live reply, because a timeout closes and reopens the TCP
connection, which itself silences the heartbeat for longer than the comm
timeout. A dropped TCP connection stops the Simulink model, which sets the
DAQ outputs to their final values. With **Set all outputs off when PLC stops**, a stopped slave
reads as a counter frozen at 0.

---

## 5. Integration Notes

- `PRG_Main` reads `G_mbModeCode`, `G_mbMotionEnable` and `G_mbFaultReset`
  directly. `FB_ModeDecoder` debounces the mode code
  (`G_cfgDebounceTimeMode`) and decodes it with `FB_IntToMode`;
  MotionEnable and FaultReset pass through `FB_DigitalInputFilter`
  (`G_cfgDebounceTimeMotion`, `G_cfgDebounceTimeFault`). The full value of
  `G_mbModeCode` is decoded by `FB_IntToFaultCode` for the fault
  acknowledgement. `PRG_Input` copies `G_mbReference` to `G_sysReference`.
- `PRG_Output` fills the input registers from the `G_do*` and `G_sys*`
  globals. `G_mbConfCode` comes from `G_sysConfirmedModeCode`,
  `G_mbSlaveCounter` from `G_sysScanCounter` and `G_mbFaultCode` from
  `G_sysActiveFaultCode`, all maintained by `PRG_Main`.
- The controller's only analog input, `G_aiPressure`, reads the pressure
  transducer's loop shunt; `FB_PressureInput` scales it to MPa gauge for
  IR19-20 and for the slave's pressure loop (mode 7).
- On the master, `masterStep` takes the 20-register read vector and returns
  the 12-register write vector; `modbusMap.m` holds the indices and
  `modbusPack.m` / `modbusUnpack.m` the REAL packing (low word first). The
  Simulink blocks handle plain `uint16`.
- The step probe's registers (section 6) are filled by `PRG_Output` from
  the `G_sysProbe*` globals that `PRG_Main` sets from `fbProbe`; `PRG_Main`
  passes the probe's holding registers (`G_mbProbe*`) to the block directly.
- Hardwired signals stay as they are: safety relay, brake, overtravel
  switches and stir bar motor
  (see [EthernetCommsReference.md](EthernetCommsReference.md)).

---

## 6. Step Probe (optional side channel)

`FB_Probe` records the axis at the scan rate for experiments and benches and
serves the records to `bench/common/benchProbe.m`. The protocol does not use
it: with `G_cfgProbeEnable` FALSE the registers hold their last values and
the master and slave behave identically. It is read in its own FC04 read
(IR401-524, 124 registers) and written with FC16 (HR111-114) in the gaps
between the protocol's exchanges, never in them. The input group must extend
to IR524 and the holding group to HR114. A group is copied every scan over its
whole span, so these spans (gaps included) are the probe's standing cost; a
project that does not use the probe keeps the groups at IR20 and HR6 and
`G_cfgProbeEnable` FALSE. Every 32-bit value is on a 4-byte boundary except the request at HR112,
which is two UINT registers, low word first.

Holding registers:

| HR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 111 | `%MW3.483548` | UINT | `G_mbProbeArm` | A changed value is an arm: the slave publishes Mark and echoes the value in ArmEcho |
| 112–113 | `%MW3.483550` | UINT ×2 | `G_mbProbeReqLo`, `G_mbProbeReqHi` | Index of the first record wanted in the window, low word first |
| 114 | `%MW3.483554` | UINT | `G_mbProbeLength` | Records the next arm wants, latched with the arm; 0 = open ended. Written before the arm, or together with it in one FC16 write of HR111-114 |

Input registers:

| IR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 401–402 | `%QD74592` | UDINT | `G_mbProbeWritten` | Records written so far |
| 403–404 | `%QD74596` | UDINT | `G_mbProbeMark` | Index of the record written in the scan of the last arm |
| 405–406 | `%QD74600` | UDINT | `G_mbProbeStart` | Index of the first record in the window |
| 407 | `%QW74604` | UINT | `G_mbProbeArmEcho` | Arm value last acted on |
| 408 | `%QW74606` | UINT | `G_mbProbeCount` | Records in the window (0 to 13) |
| 409 | `%QW74608` | UINT | `G_mbProbeDepth` | Ring depth in records (8192) |
| 410 | `%QW74610` | UINT | `G_mbProbeWords` | Words per record (8) |
| 411–514 | `%QW74612` | WORD ×104 | `G_mbProbeWindow` | 13 records of 8 words (the record layout below). IR515-518 are not used |
| 519–520 | `%QD74828` | UDINT | `G_mbProbeStartTail` | Copy of Start written after the window; a read with Start ≠ StartTail is torn |
| 521 | `%QW74832` | UINT | `G_mbProbeState` | `E_ProbeState`: 0 `PROBE_OFF`, 1 `PROBE_RECORDING`, 2 `PROBE_CAPTURING`, 3 `PROBE_HELD` |
| 522 | `%QW74834` | UINT | `G_mbProbeXfer` | `E_ProbeXferState`: 0 `XFER_IDLE`, 1 `XFER_FILLING`, 2 `XFER_PENDING`, 3 `XFER_OVERRUN` |
| 523–524 | `%QD74836` | UDINT | `G_mbProbeFaultMark` | Index of the record written in the scan `PRG_Main` last entered `ST_FAULT`; 0 until the first fault |

Record (8 words, defined in `FB_Probe.st`; the master reads the word count
from IR410):

| Word | Type | Content | Unit |
|------|------|---------|------|
| 0 | INT | Commanded velocity | 0.001 mm/s |
| 1 | INT | Actual velocity | 0.001 mm/s |
| 2 | INT | Actual torque | 0.01 % of rated |
| 3–4 | DINT | Actual position, absolute, low word first | 0.01 µm |
| 5 | INT | Chamber pressure | 0.001 MPa gauge |
| 6 | UINT | Raw pressure input (`G_aiPressure`) | 0.2 mV per count |
| 7 | WORD | Status: bit 0 in motion, bit 1 performance status, bit 2 fault active, bit 3 homing complete, bits 8-15 confirmed mode code | |

Protocol:

- **Ring.** One record per scan is appended to a ring of 8192 records (33 s at
  the 4 ms scan) in plain memory; the master reads records through the
  window, so the capture length is limited by the ring, not by the register
  groups. Index values wrap after 2^32 scans.
- **Arm.** Writing a value different from the last one acted on publishes
  Mark, echoes the value and starts a capture of Length records from Mark
  (0 = open ended). An arm is accepted at any time. The master takes the
  records it wants from Mark on (or before it, while the ring holds them).
- **Recording state** (IR521): `PROBE_RECORDING` while no armed capture is open;
  `PROBE_CAPTURING` from the arm until the capture's last record
  (Mark + Length - 1) is written, for ever if open ended; `PROBE_HELD` while the
  capture is fully recorded and still entirely in the ring; back to
  `PROBE_RECORDING` once the ring overwrites the record at Mark (a capture
  longer than the ring goes straight there). `PROBE_OFF` with
  `G_cfgProbeEnable` FALSE: nothing is recorded and arms are not acted on.
- **Window and transfer state** (IR522): the master writes the index of the
  first record it wants. A request that differs from the one being served
  restarts the fill; writing the same request again changes nothing. The slave
  copies at most `G_cfgProbeFillPerScan` records (default 6) per scan into a
  staging array and, when the window is complete, publishes it in one scan:
  Start, Count and StartTail change then and not before, so the master never
  accepts a half-filled window. `XFER_FILLING` while copying, `XFER_IDLE`
  when the window is complete for the request, `XFER_PENDING` when the
  requested records are not recorded yet (the window is served from the first
  record written, and a window with fewer than 13 records grows as records
  are written), `XFER_OVERRUN` when the requested start has been overwritten
  (nothing is published). The master accepts a read when Start equals
  StartTail and equals the index it requested.
- **Fault mark.** On the scan `PRG_Main` enters `ST_FAULT`, FaultMark takes
  the index of that scan's record, so the history up to the fault can be read
  from the ring for as long as it holds (about 33 s).
- **Cost.** Per scan: the 8-word record, the arm and fault logic and at most
  `G_cfgProbeFillPerScan` x 8 word copies; in the scan a fill completes,
  `PRG_Output` adds a 104-word copy of the staging array into the mapped
  window.
