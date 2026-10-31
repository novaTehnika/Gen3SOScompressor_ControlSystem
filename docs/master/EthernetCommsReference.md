# Ethernet Communication Reference

## Master ↔ Slave over Ethernet: Options and Sources

---

## 1. Scope

This document collects the vendor documentation and bench measurements behind
the Ethernet link between the lab PC and the MP2600iec. Modbus TCP is the
chosen transport; the protocol over it is in
[ModbusRegisterMap.md](ModbusRegisterMap.md). It carries every master ↔ slave
signal except the hardwired ones listed below.
It covers the transport options available with the installed software, how each
is configured, and the open questions that must be answered on the bench. It is
not a protocol specification. The current protocol is defined in
[MasterProtocolGuide.md](MasterProtocolGuide.md).

### Signals that stay hardwired

Safety and plant I/O do not depend on the network:

| Signal | Reason |
|--------|--------|
| P0.0 safety relay / STO enable | Safety function |
| Brake circuit (`G_doBrakeDisengage`) | Driven directly by the slave |
| Overtravel switches | Slave inputs |
| Pressure transducer | On the slave's analog input; the slave closes the pressure loop and reports the reading in IR19-20 |
| P0.4 stir bar motor | Not part of the slave protocol |

---

## 2. Execution Environment

### Master

The shell model runs in Simulink **normal mode** (the app starts it with
`simulation('SOS_Gen3_Master_Shell')`), which under Simulink Desktop Real-Time is
**Connected IO mode**. Simulink executes the model on the host and a kernel-mode
process runs the DAQ drivers, exchanging data through shared memory each tick.

Consequences:

- Host-based blocks (Modbus Client, OPC, TCP/IP and UDP blocks) can be used in the
  shell alongside the DAQ blocks.
- Timing is not deterministic. When a step takes longer than the tick, Simulink
  counts missed ticks and stops with an error once they exceed **Maximum Missed
  Ticks**. A network block that waits on a reply consumes step time directly.

Source: [Real-Time Execution in Connected IO Mode](https://www.mathworks.com/help/sldrt/ug/real-time-execution-in-connected-io-mode.html)

### Slave

The MP2600iec has **two 100 Mbps Ethernet ports** supporting Modbus/TCP,
EtherNet/IP and OPC
([Yaskawa MP2600iec](https://www.yaskawa.com/products/motion/machine-controllers/mpiec-series/mp2600iec)).
The application runs in a 2 ms cyclic task (see
[HardwareConfiguration.md](../slave/HardwareConfiguration.md)).

### Physical link

The PC and controller are joined by one direct cable, also used by MotionWorks
IEC. Control traffic can share it: MotionWorks, Modbus TCP (port 502) and UDP use
separate ports on the same link. Online monitoring from MotionWorks adds traffic
and may add latency. The controller's second port can carry a dedicated link if
the PC is fitted with a second network card, on its own subnet.

---

## 3. Option A: Modbus TCP

### Slave: MP2600iec as Modbus TCP server

No ST communication code is needed. The server is configured in Hardware
Configuration and exchanges data with global variables declared at fixed
addresses.

Setup (Toolbox manual, *Configure the MPiec Controller as a Modbus TCP server*,
p. 305, `docs/external/MotionWorks_IEC_Toolbox_Manual.pdf`):

1. Go online with Hardware Configuration and configure the controller as a Modbus
   slave (server).
2. Optionally check *use %M for bidirectional Holding Registers*.
3. Select **Set all outputs off when PLC stops**.
4. **IO Task Assignment**: assign the four server blocks to the task that runs
   `PRG_Main`, so the data updates in step with the scan. MotionWorks IEC 3
   Express has only that one task, so there is no slower task to assign
   them to.
5. Save the configuration and cycle power. Four Modbus groups are created in the
   Global Variables worksheet. Declare variables under them with the required
   data types and addresses.

Address map, as created by Hardware Configuration on this controller. The
groups and their address ranges appear in the Global Variables worksheet and
are the reference; published application notes
([Exor: MPiec via Modbus TCP](https://www.exorint.com/faq/2017/06/13/how-to-communicate-with-an-yaskawa-mpiec-controller-via-modbus-tcp),
Yaskawa AN.MPiec.05) give `%QW28672` and `%IX24560` for other firmware.

| Modbus area | Group | Direction | MPiec address | Size |
|-------------|-------|-----------|---------------|------|
| Holding registers (4x) | FC#03,06,16 | master → slave | `%MB3.483328`–`%MB3.503327` with **Use %M for bidirectional Holding Registers** (`%IB28672`–`%IB30719`, 1024 registers, without it) | 10000 registers |
| Input registers (3x) | FC#04 | slave → master | `%QB73792`–`%QB93791` | 10000 registers |
| Coils | FC#05 | master → slave | `%IB73728`–`%IB73791` | 512 bits |
| Discrete inputs | FC#02 | slave → master | `%QB73728`–`%QB73791` | 512 bits |

Function codes served: 1, 2, 3, 4, 5, 6 and 16 (MotionWorks IEC Hardware
Configuration manual). FC15 (write multiple coils) and FC23 (read/write in
one transaction) are not served, so a write and a read are separate
transactions, and command bits are best carried in holding registers. FC01
and FC03 return the values last written by the client, not program globals,
unless the %M bidirectional option is used.

Limits and known issues (MotionWorks IEC 2.1.0 release notes, RN.MWIEC.14):

- Maximum of **20 Modbus TCP data blocks**.
- Renaming an I/O device, or deleting and re-adding one with the same settings,
  can desynchronise I/O group addresses from the variable group. Save Hardware
  Configuration between deleting a device and re-entering it.

### Master: Industrial Communication Toolbox

- [Modbus Client Read](https://www.mathworks.com/help/icomm/ug/modbusclientread.html)
  and [Modbus Client Write](https://www.mathworks.com/help/icomm/ug/modbusclientwrite.html)
  blocks. The client connection is defined in the model's Configuration Parameters,
  **Modbus** pane.
- Register table per block: name, 1-based address, register type, precision
  (`int16` … `double`, `bit` for coils/inputs) and count. All outputs are `double`.
- Server ID 0–255 (default 1). Block sample time inherits from the model by default.
- Connection settings
  ([Configure a Modbus connection](https://www.mathworks.com/help/icomm/ug/configure-a-connection-in-the-modbus-explorer.html)):
  - **Timeout** default **3 s**. It must be reduced to well under the missed-tick
    budget, otherwise a lost reply stalls the model.
  - **Byte order** and **word order** default to big-endian and must match the
    server. Word order applies to values spanning two registers (`int32`,
    `single`).
- [Troubleshooting Modbus](https://www.mathworks.com/help/icomm/ug/troubleshooting-modbus-interface.html).

### Characteristics

- Each block performs a request/response transaction per step.
- 32-bit and 64-bit values occupy 2 or 4 registers. The MP2600iec holds them
  low word first; the client's word order must be little-endian.
- The protocol has no freshness indication. A heartbeat register written by the
  master and checked by the slave is required to detect a lost link.
- Measured on the bench (`bench/modbus`): the controller services the
  connection on a **4 ms tick** and answers **one request per tick**. A
  transaction is answered 1 to 8 ms after the request; a write and a read
  sent back to back are answered in consecutive ticks, 6 to 12 ms for the
  pair. A read in the tick after a write still returns the pre-write value.
- Any MATLAB code that blocks on a socket read (`modbus` object or
  `tcpclient`) adds about 7 ms (p99 17 ms) per blocking call on the lab PC.
  Sending both requests before waiting pays this once per exchange. In
  Connected IO mode such a call stalls the model for its duration, and the
  Modbus Client blocks behave the same way: at a 25 ms model step the stall
  costs at most two ticks, at 5 ms the model falls behind on a third of its
  steps (`bench/modbus/README.md`, section 7).
- The Modbus Client blocks have no status output and **stop the model with an
  error when the connection is lost**. Under Connected IO the SLDRT output
  blocks then write their Final values, which the shell sets to 0, so a lost
  link drops the safety relay and the mode lines within one step. The
  operator app must detect the stopped model and report it; the run is
  restarted from the app. If this proves a nuisance, a MATLAB System block
  wrapping the same `modbus` object with a try/catch, a status output and a
  reconnect would turn link loss into a comm fault instead of a stop.

---

## 4. Option B: UDP

### Slave: Y_DeviceComm and Comm Toolbox

- **Y_DeviceComm** firmware library (firmware 2.1.0.229 and later) provides TCP
  socket functions, with UDP also supported
  ([MotionWorks IEC 2.1.0 release notes](https://www.yaskawa.com/delegate/getAttachment?documentId=RN.MWIEC.14&cmd=documents&documentName=RN.MWIEC.14_MotionWorksIEC_2_1_0_Release_Notes.pdf)).
  Its function reference ships with the MotionWorks IEC help and is not published
  online.
- [PP.MPiec.02](https://www.yaskawa.com/downloads/search-index/details?showType=details&docnum=PP.MPiec.02)
  is a Yaskawa presentation that uses Y_DeviceComm.
- Comm Toolbox wrappers (Toolbox manual):
  - `ReName_UserComm` (p. 237): template copied into the project and customised to
    exchange a user-defined structure as raw binary over a socket.
  - `CommunicationChannel`, `CommStruct`, `EthernetConfig` (p. 186): TCP/UDP
    interface configuration (local/remote IP and port, inactivity timeout).
  - Y_DeviceComm needs the controller's own IP address to select the interface.
    It can be read from a `CONTROLLER_INFO` global at `%MD3.66560`.
  - The manual recommends running communication function blocks in slower tasks
    (about 4–50 ms, or the DEFAULT task) to avoid the PLC watchdog. MotionWorks
    IEC 3 Express has a single task, so they would run in the control task
    and its scan time.
- Socket code would sit in a POU that `tools/st2m.py` does not translate, and
  exchange data with `PRG_Main` through global structs, in the same way as the
  Ladder Diagram motion POU.

### Master: Instrument Control Toolbox

- [UDP Receive](https://www.mathworks.com/help/instrument/udpreceive.html) and UDP
  Send blocks.
- UDP Receive parameters relevant here:
  - **Blocking mode off**: adds a **Status** output, 1 when new data arrived this
    step, 0 otherwise.
  - **Output latest data**: returns the newest packet rather than the oldest in
    the FIFO.
  - Data size, source data type (`uint8`, `single`, `double`, integer types) and
    **byte order** (default big-endian).
  - Block sample time (default 0.01 s) must be set to the model step.

### Characteristics

- One packet each way per step, no request/response wait.
- A sequence number in each packet provides the freshness check and a round-trip
  latency measurement.
- The packet layout is defined by the application.

---

## 5. Option C: OPC

- The MPiec OPC interface is the **MotionWorks IEC OPC Server**, an optional
  Windows application on the PC that exchanges data with the controller
  ([Yaskawa MP2600iec overview](https://www.yaskawa.com/products/motion/machine-controllers/mpiec-series/mp2600iec/-/content/_feac85d6-ebb4-4af8-9e09-d9eb087d1f29_Overview)).
  It is OPC DA (COM-based). No source indicates an OPC UA server on the
  MP2600iec, so the Simulink [OPC UA Read](https://www.mathworks.com/help/icomm/ug/opcuaread.html)
  / Write blocks do not apply.
- Data path: Simulink OPC Read/Write → OPC Server → controller.
- Global variables are exposed individually and addressed by name, so no register
  map or word ordering is involved.
- Values pass through the server's update cycle and cache, typically tens of
  milliseconds, with no controller-side sample time. The quality flag reflects the
  server connection, not whether the slave is scanning.
- Suited to supervisory and diagnostic data, not the 3 ms control path. Requires
  the OPC Server licence.

---

## 6. Comparison

| | Modbus TCP | UDP | OPC DA |
|---|---|---|---|
| Fit for the 3 ms control step | No: 4 ms service tick, 1–8 ms per transaction | Possible, unmeasured | Poor |
| Fit for a 50 ms exchange | Yes | Yes | Marginal |
| Slave effort | Hardware Configuration + addressed globals | Socket POU, packet packing | Expose variables |
| Additional software | None | None | MotionWorks IEC OPC Server |
| Link-loss detection | Heartbeat register | Sequence number | Heartbeat variable |
| Diagnostics access | Mapped registers only | Packet fields only | Any exposed global |

---

## 7. Requirements Common to All Options

- **Exchange period up to 50 ms.** The link carries mode requests, set points,
  status and feedback. The safety enable, which must act within a few
  milliseconds, stays on the hardwired relay line (P0.0), and the pressure loop
  runs on the slave, so the master's step rate does not set the link rate.
- **Slave watchdog**: if the master heartbeat or sequence number does not advance
  within a set time, latch a communication fault and halt (`FAULT_COMM`, code 4,
  see [FaultCodeReference.md](FaultCodeReference.md)).
- **Master watchdog**: treat stale slave data as a fault and drop the hardwired
  safety relay.
- Reference and feedback in engineering units (mm, mm/s, %, MPa).
- Fault code and mode confirmation in separate fields (IR4, IR3).

---

## 8. Open Questions for the Bench

Answered by the bench: word order is low word first; the controller services
Modbus on a 4 ms tick, 1 to 8 ms per transaction (`bench/modbus/README.md`).

Answered by the bench model: the Simulink Modbus Client blocks at a 50 ms
sample time block the model for the exchange. At a 25 ms model step the lag
is at most two ticks on 0.2 % of steps; at 5 ms the model lags on a third of
its steps, up to 17 ticks. The shell should step at 25 ms or longer with the
blocks in it. The DAQ blocks' Maximum missed ticks is 1e6, so lag drops
samples silently instead of stopping the model.

Bench case D: a lost connection stops the model with an error from the Read
block. Accepted as the link-loss response for now (see section 3).

1. **Service tick origin**: whether the 4 ms tick follows the scan of the
   one task (MotionWorks IEC 3 Express has no other), or is fixed in the
   controller's network stack.
2. **Shared cable**: compare latency with MotionWorks online and offline.
3. **Y_DeviceComm reference**: export from MotionWorks IEC help on the lab PC,
   only if UDP is still wanted for a faster path.
4. **OPC Server licence**: confirm whether it is installed, if OPC is wanted for
   diagnostics.

---

## 9. Sources

| Source | Topic |
|--------|-------|
| `docs/external/MotionWorks_IEC_Toolbox_Manual.pdf` p. 186, 237, 305 | Comm Toolbox data types, `ReName_UserComm`, Modbus TCP server setup |
| [RN.MWIEC.14 MotionWorks IEC 2.1.0 release notes](https://www.yaskawa.com/delegate/getAttachment?documentId=RN.MWIEC.14&cmd=documents&documentName=RN.MWIEC.14_MotionWorksIEC_2_1_0_Release_Notes.pdf) | Y_DeviceComm, Modbus limits |
| [PP.MPiec.02](https://www.yaskawa.com/downloads/search-index/details?showType=details&docnum=PP.MPiec.02) | Y_DeviceComm usage example |
| [Yaskawa MP2600iec](https://www.yaskawa.com/products/motion/machine-controllers/mpiec-series/mp2600iec) | Ethernet ports, protocols |
| [Yaskawa MP2600iec overview](https://www.yaskawa.com/products/motion/machine-controllers/mpiec-series/mp2600iec/-/content/_feac85d6-ebb4-4af8-9e09-d9eb087d1f29_Overview) | OPC Server |
| [Exor FAQ: MPiec via Modbus TCP](https://www.exorint.com/faq/2017/06/13/how-to-communicate-with-an-yaskawa-mpiec-controller-via-modbus-tcp) | Modbus address map |
| [Inductive Automation forum: MP2600iec connectivity](https://forum.inductiveautomation.com/t/yaskawa-mp2600iec-what-module-for-connectivity/65133) | Variables must be exposed individually per protocol |
| [Connected IO mode](https://www.mathworks.com/help/sldrt/ug/real-time-execution-in-connected-io-mode.html) | Master timing |
| [Modbus Client Read](https://www.mathworks.com/help/icomm/ug/modbusclientread.html), [Write](https://www.mathworks.com/help/icomm/ug/modbusclientwrite.html) | Modbus blocks |
| [Configure a Modbus connection](https://www.mathworks.com/help/icomm/ug/configure-a-connection-in-the-modbus-explorer.html), [Troubleshooting](https://www.mathworks.com/help/icomm/ug/troubleshooting-modbus-interface.html) | Timeout, byte/word order |
| [UDP Receive](https://www.mathworks.com/help/instrument/udpreceive.html) | UDP block |
| [OPC UA Read](https://www.mathworks.com/help/icomm/ug/opcuaread.html) | OPC UA blocks |
