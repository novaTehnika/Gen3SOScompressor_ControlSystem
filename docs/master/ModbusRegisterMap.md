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

Each exchange is one **Write Multiple Registers (FC16)** of holding registers
1–6 followed at once by one **Read Input Registers (FC04)** of input registers
1–18, so both land in the same controller service tick. The exchange period is
up to 50 ms, a multiple of the master step. Fast signals stay on the digital
lines; see [EthernetCommsReference.md](EthernetCommsReference.md).

### Addressing

Register numbers are 1-based, as used by MATLAB and the Simulink Modbus blocks.
Register *k* maps to a byte offset inside the group that Hardware
Configuration creates in the Global Variables worksheet:

- Holding registers (master → slave, group FC#06,16): `%IW` / `%ID` at
  `28672 + 2*(k-1)`
- Input registers (slave → master, group FC#04): `%QW` / `%QD` at
  `73792 + 2*(k-1)`

The group address ranges are shown in the worksheet and must be checked
there; published application notes give other bases.

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
`PRG_Main`, so the registers update once per scan. The controller services the
connection on a 4 ms tick, so a transaction takes 1 to 8 ms regardless of the
task interval.

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
| 1 | `%QW73792` | UINT | `G_mbHeartbeatEcho` | Last `G_mbHeartbeat` received |
| 2 | `%QW73794` | UINT | `G_mbSlaveCounter` | Incremented every scan, wraps 65535 → 0 |
| 3 | `%QW73796` | UINT | `G_mbConfCode` | Same value as DO0–DO2: mode confirmation, or fault code while a fault is active |
| 4 | `%QW73798` | UINT | `G_mbFaultCode` | `G_sysActiveFault` as 0–7, 0 when no fault |
| 5 | `%QW73800` | UINT | `G_mbFaultActive` | 1 = fault active (DO4) |
| 6 | `%QW73802` | UINT | `G_mbInMotion` | 1 = axis moving (DO6) |
| 7 | `%QW73804` | UINT | `G_mbPerformanceStatus` | DO3 |
| 8 | `%QW73806` | UINT | `G_mbHomingComplete` | DO5 |
| 9 | `%QW73808` | UINT | `G_mbState` | `G_sysCurrentState` as its enumeration value |
| 10 | `%QW73810` | UINT | | Reserved |
| 11–12 | `%QD73812` | REAL | `G_mbActualPosition` | `G_sysActualPosition` (mm) |
| 13–14 | `%QD73816` | REAL | `G_mbActualVelocity` | `G_sysActualVelocity` (mm/s) |
| 15–16 | `%QD73820` | REAL | `G_mbActualTorque` | `G_sysActualTorque` (% of rated) |
| 17–18 | `%QD73824` | REAL | `G_mbCommandedPosition` | `G_sysCommandedPosition` (mm) |
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

The comm timeout is a small multiple of the exchange period, long enough to
ride through one lost exchange (for example 150 ms at a 50 ms period). With
**Set all outputs off when PLC stops**, a stopped slave reads as a counter
frozen at 0.

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
