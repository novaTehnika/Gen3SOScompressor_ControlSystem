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
1–6 followed at once by one **Read Input Registers (FC04)** of input registers
1–18, so both land in the same controller service tick. The exchange period is
up to 50 ms, a multiple of the master step. Fast signals stay on the digital
lines; see [EthernetCommsReference.md](EthernetCommsReference.md).

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
| 1 | `%MW3.483328` | UINT | `G_mbHeartbeat` | Master step counter, incremented every step, wraps 65535 → 0 |
| 2 | `%MW3.483330` | UINT | `G_mbModeCode` | Mode code 0–7 (same meaning as DI0–DI2). During fault reset the master mirrors the **full** fault code here. The mode decoder reads the low three bits, so a mirrored code of 8 or above looks like mode 0 to it, which is what mirroring code 0 looks like today; the slave ignores the decoded mode on the fault-acknowledge path |
| 3 | `%MW3.483332` | UINT | `G_mbMotionEnable` | 1 = MotionEnable |
| 4 | `%MW3.483334` | UINT | `G_mbFaultReset` | 1 = FaultReset (level) |
| 5–6 | `%MD3.483336` | REAL | `G_mbReference` | Position mode: mm. Velocity mode: mm/s. Torque mode: % of rated. Pressure mode (code 7, planned): MPa gauge |
| 7–20 | | | | Reserved |
| 101–110 | | | | Bench probe area (`bench/modbus`) |

---

## 3. Input Registers (slave → master)

| IR | Address | Type | Global | Content |
|----|---------|------|--------|---------|
| 1 | `%QW73792` | UINT | `G_mbHeartbeatEcho` | Last `G_mbHeartbeat` received |
| 2 | `%QW73794` | UINT | `G_mbSlaveCounter` | Incremented every scan, wraps 65535 → 0 |
| 3 | `%QW73796` | UINT | `G_mbConfCode` | Same value as DO0–DO2: mode confirmation, or the low three bits of the fault code while a fault is active |
| 4 | `%QW73798` | UINT | `G_mbFaultCode` | `G_sysActiveFault` as its full integer value, 0 when no fault. May exceed 7 |
| 5 | `%QW73800` | UINT | `G_mbFaultActive` | 1 = fault active (DO4) |
| 6 | `%QW73802` | UINT | `G_mbInMotion` | 1 = axis moving (DO6) |
| 7 | `%QW73804` | UINT | `G_mbPerformanceStatus` | DO3 |
| 8 | `%QW73806` | UINT | `G_mbHomingComplete` | DO5 |
| 9–10 | `%QW73808` | UINT | | Reserved |
| 11–12 | `%QD73812` | REAL | `G_mbActualPosition` | `G_sysActualPosition` (mm) |
| 13–14 | `%QD73816` | REAL | `G_mbActualVelocity` | `G_sysActualVelocity` (mm/s) |
| 15–16 | `%QD73820` | REAL | `G_mbActualTorque` | `G_sysActualTorque` (% of rated) |
| 17–18 | `%QD73824` | REAL | `G_mbCommandedPosition` | `G_sysCommandedPosition` (mm) |
| 19–20 | `%QD73828` | REAL | `G_mbActualPressure` | Reserved for the slave-side pressure loop: actual pressure, MPa gauge |
| 101–110 | | | | Bench probe area (`bench/modbus`) |

---

## 4. Link Supervision

Holding registers keep their last written value when the master stops writing,
so a stopped master leaves MotionEnable and the reference in place. Both sides
therefore supervise the link.

| Side | Condition | Action |
|------|-----------|--------|
| Slave | `G_mbHeartbeat` unchanged for `G_cfgCommTimeout` (300 ms) while moving, homing or starting up; armed by the first change seen | `FAULT_COMM` (code 4) through `FB_SafetyMonitor`, so the fault latches, mirrors and resets like any other. In idle, brake hold and hold position a stopped master is not a fault |
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

- `PRG_Input` maps `G_mbModeCode` (low three bits), `G_mbMotionEnable` and
  `G_mbFaultReset` onto the `G_di*` globals, so the debounce, mode decoder,
  handshake and fault-reset FBs are unchanged. `G_mbReference` is copied to
  `G_sysReference`, which `PRG_Main` uses where the analog processor's scaled
  outputs were used. The full value of `G_mbModeCode` is decoded by
  `FB_IntToFaultCode` for the fault acknowledgement.
- `PRG_Output` fills the input registers from the `G_do*` and `G_sys*`
  globals. `G_mbSlaveCounter` comes from `G_sysScanCounter` and
  `G_mbFaultCode` from `G_sysActiveFaultCode`, both maintained by `PRG_Main`.
- The analog reference and position-output path (`FB_AnalogProcessor`,
  `FB_PositionOutput`, the two-stage voltage map constants) is removed. The
  analog input `G_aiReference` and its filter remain declared for the
  pressure transducer, which takes the controller's only analog input.
- On the master, `masterStep` takes the 18-register read vector and returns
  the 6-register write vector; `modbusMap.m` holds the indices and
  `modbusPack.m` / `modbusUnpack.m` the REAL packing (low word first). The
  Simulink blocks handle plain `uint16`.
- Hardwired signals stay as they are: safety relay, brake, overtravel
  switches, pressure transducer and stir bar motor
  (see [EthernetCommsReference.md](EthernetCommsReference.md)).
