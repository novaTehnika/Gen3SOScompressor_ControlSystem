# Modbus Register Map

## Master ↔ Slave over Modbus TCP (proposed)

---

## 1. Overview

**Status: proposed, not implemented.** The current interface is the discrete
and analog wiring in [MasterProtocolGuide.md](MasterProtocolGuide.md). This map
carries the same protocol over Modbus TCP: the same mode codes, handshake and
fault-mirror reset, with the reference and feedback in engineering units. The
word order and timing assumptions are confirmed by the bench test in
`bench/modbus`.

| Role | Device |
|------|--------|
| Server | MP2600iec, built-in Modbus TCP server, port 502 |
| Client | Simulink shell on the lab PC |

Per master step the client issues one **Write Multiple Registers (FC16)** of
holding registers 1–6 and one **Read Input Registers (FC04)** of input registers
1–18.

### Addressing

Register numbers are 1-based, as used by MATLAB and the Simulink Modbus blocks.
Register *k* maps to byte offset `28672 + 2*(k-1)`:

- Holding registers (master → slave): `%IW` / `%ID` at that offset
- Input registers (slave → master): `%QW` / `%QD` at that offset

### Data types

| Type | Registers | Notes |
|------|-----------|-------|
| `UINT` | 1 | Codes, flags (0/1), counters |
| `REAL` | 2 | IEEE 754 single precision. Word order as measured by `modbusProbe` |

`REAL` resolves about 2e-5 mm at 300 mm, finer than any quantity carried here.
`LREAL` values are converted to `REAL` on the slave.

### Slave configuration

In Hardware Configuration: Modbus slave (server) enabled, **Set all outputs off
when PLC stops**, and the four server blocks assigned to the 2 ms cyclic task so
the registers update once per scan, in step with `PRG_Main`.

---

## 2. Holding Registers (master → slave)

| HR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 1 | `%IW28672` | UINT | `G_mbHeartbeat` | Master step counter, incremented every step, wraps 65535 → 0 |
| 2 | `%IW28674` | UINT | `G_mbModeCode` | Mode code 0–7, same meaning as DI0–DI2, including the fault code mirror during reset |
| 3 | `%IW28676` | UINT | `G_mbMotionEnable` | 1 = MotionEnable |
| 4 | `%IW28678` | UINT | `G_mbFaultReset` | 1 = FaultReset (level) |
| 5–6 | `%ID28680` | REAL | `G_mbReference` | Position mode: mm. Velocity mode: mm/s. Torque mode: % of rated |
| 7–20 | | | | Reserved |
| 101–110 | | | | Bench probe area (`bench/modbus`) |

---

## 3. Input Registers (slave → master)

| IR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 1 | `%QW28672` | UINT | `G_mbHeartbeatEcho` | Last `G_mbHeartbeat` received |
| 2 | `%QW28674` | UINT | `G_mbSlaveCounter` | Incremented every scan, wraps 65535 → 0 |
| 3 | `%QW28676` | UINT | `G_mbConfCode` | Same value as DO0–DO2: mode confirmation, or fault code while a fault is active |
| 4 | `%QW28678` | UINT | `G_mbFaultCode` | `G_sysActiveFault` as 0–7, 0 when no fault |
| 5 | `%QW28680` | UINT | `G_mbFaultActive` | 1 = fault active (DO4) |
| 6 | `%QW28682` | UINT | `G_mbInMotion` | 1 = axis moving (DO6) |
| 7 | `%QW28684` | UINT | `G_mbPerformanceStatus` | DO3 |
| 8 | `%QW28686` | UINT | `G_mbHomingComplete` | DO5 |
| 9 | `%QW28688` | UINT | `G_mbState` | `G_sysCurrentState` as its enumeration value |
| 10 | `%QW28690` | UINT | | Reserved |
| 11–12 | `%QD28692` | REAL | `G_mbActualPosition` | `G_sysActualPosition` (mm) |
| 13–14 | `%QD28696` | REAL | `G_mbActualVelocity` | `G_sysActualVelocity` (mm/s) |
| 15–16 | `%QD28700` | REAL | `G_mbActualTorque` | `G_sysActualTorque` (% of rated) |
| 17–18 | `%QD28704` | REAL | `G_mbCommandedPosition` | `G_sysCommandedPosition` (mm) |
| 19–20 | | | | Reserved |
| 101–110 | | | | Bench probe area (`bench/modbus`) |

---

## 4. Link Supervision

Holding registers keep their last written value when the master stops writing,
so a stopped master leaves MotionEnable and the reference in place. Both sides
therefore supervise the link.

| Side | Condition | Action |
|------|-----------|--------|
| Slave | `G_mbHeartbeat` unchanged for longer than the comm timeout | Latch a communication fault (code 4, currently `FAULT_RESERVED_100`) and halt |
| Master | `G_mbSlaveCounter` unchanged, or reads failing, for longer than the comm timeout | Treat as a fault and drop the hardwired safety relay (P0.0) |

The comm timeout is set from the bench results. With **Set all outputs off when
PLC stops**, a stopped slave reads as a counter frozen at 0.

---

## 5. Integration Notes

- `PRG_Input` maps the holding-register globals onto the existing `G_di*`
  globals (mode bits from `G_mbModeCode`), so the handshake and fault-reset
  function blocks are unchanged. `PRG_Output` fills the input registers from the
  `G_do*` and `G_sys*` globals.
- `G_mbReference` replaces `G_aiReference`, and the actual-position registers
  replace `G_aoPositionOutput`. The two-stage voltage scaling in
  `FB_AnalogProcessor` / `FB_PositionOutput` and in the master's `x2V`/`V2x` is
  no longer used on this path.
- The ST constructs for bit extraction and `LREAL` → `REAL` conversion must be
  supported by `tools/st2m.py`, or added to it.
- The variable declarations follow the `Variables (define in MWiec GUI)`
  convention so st2m picks them up.
- Hardwired signals stay as they are: safety relay, brake, overtravel switches,
  pressure transducer and stir bar motor
  (see [EthernetCommsReference.md](EthernetCommsReference.md)).
