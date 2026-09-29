# State Transition Diagram

## Gen3 SOS Compressor Servo Controller
### Visual State Machine Reference

---

## 1. High-Level State Overview

```
                           POWER ON
                               |
                               v
+------------------------------------------------------------------+
|                    INITIALIZATION PHASE                           |
|  +----------+                                                     |
|  | ST_INIT  | --> First scan transitions directly to ST_IDLE     |
|  |          |                                                     |
|  +----------+                                                     |
+------------------------|-----------------------------------------+
                         |
                         v
                    +----------+
                    | ST_IDLE  |
                    +----------+
                           |
            (Mode != 000 + G_diMotionEnable)
                           |
            +--------------+--------------+
            |                             |
     (Homing Modes              (Operational Modes
      101,110)                   001,010,011,100)
      [Mode 111 is reserved -
       no handler, no action]
            |                             |
            +----------+------------------+
                       |
                       v
              +----------------+
              | ST_DRIVE_ENABLE|
              +----------------+
                       |
                       v
              +----------------+
              | ST_BRAKE       |
              | _RELEASE       |
              +----------------+
                       |
         +------+------+------+------+------+------+
         |      |      |      |      |      |      |
         v      v      v      v      v      v
       001    010    011    100    101    110
       BRAKE  POS    VEL    TRQ    GO     HOME
       HOLD   CTRL   CTRL   CTRL   HOME   LIMIT
```

---

## 2. Operational State Flow

```
                    OPERATIONAL STATES
    +------------------------------------------------+
    |                                                |
    |   +----------+    +----------+    +----------+ |
    |   | POSITION |    | VELOCITY |    | TORQUE   | |
    |   | CTRL     |    | CTRL     |    | CTRL     | |
    |   | (010)    |    | (011)    |    | (100)    | |
    |   +----+-----+    +----+-----+    +----+-----+ |
    |        |              |              |         |
    |        +------+-------+------+-------+         |
    |               |              |                 |
    |      (G_diMotionEnable   (Mode change           |
    |       = LOW)            or fault)             |
    |               |              |                 |
    |               v              v                 |
    |        +-------------+  +-----------+          |
    |        | ST_HOLD     |  | ST_FAULT  |          |
    |        | _POSITION   |  |           |          |
    |        +------+------+  +-----------+          |
    |               |                                |
    |      (New mode handshake)                      |
    |               |                                |
    |        +------v------+                         |
    |        | New Mode    |                         |
    |        | (via brake  |                         |
    |        |  release)   |                         |
    |        +-------------+                         |
    +------------------------------------------------+
```

---

## 3. Homing State Sequences

> **Note on FB-to-motion-FB references.** The `Jog` and `MC_MoveAbsolute` labels in the diagrams below are shorthand. The actual control flow is: the custom homing FB writes to its `CmdXxx` `VAR_OUTPUT`, `PRG_Main` copies that into the `G_cmd*` global, and the Ladder Diagram POU's built-in FB acts on it. Status returns via the paired `G_sta*` global into the FB's `StaXxx` `VAR_INPUT`. See the [System Architecture](../slave/development/SystemArchitecture.md) document for the full wiring.

### Mode 110: Home to Limit Switch

```
+---------------+
| ST_HOME_LIMIT |  Entry point from mode command
+-------+-------+  (FB_HomeLimit executes internally)
        |
        v
  [FB_HomeLimit internal steps]
        |
+-------------------+
| FAST_APPROACH      |  Jog Reverse @ FastApproachVelocity
|                   |  Moving toward negative overtravel / home switch
+-------+-----------+
        |
        | (OvertravelNegActive = TRUE)
        v
+-------------------+
| FAST_AWAIT         |  Wait for Jog to decelerate to a stop
+-------+-----------+
        |
        v
+-------------------+
| INTER_BACKOFF      |  Jog Forward @ BackoffVelocity
|                   |  Off the switch, just far enough to clear it
+-------+-----------+
        |
        | (OvertravelNegActive = FALSE)
        v
+-------------------+
| INTER_AWAIT        |  Wait for Jog to decelerate to a stop
+-------+-----------+
        |
        v
+-------------------+
| SLOW_APPROACH      |  Jog Reverse @ SlowApproachVelocity
|                   |  Precise re-detect; captures switch position
+-------+-----------+
        |
        | (OvertravelNegActive = TRUE)
        v
+-------------------+
| SLOW_AWAIT         |  Wait for Jog to decelerate to a stop
+-------+-----------+
        |
        v
+-------------------+
| RETRACT            |  MC_MoveAbsolute(switch position + RetractDist)
|                   |  Backs off the switch by exactly RetractDist
+-------+-----------+
        |
        v
+-------------------+
| RETRACT_AWAIT      |  Wait for MC_MoveAbsolute.Done
+-------+-----------+
        |
        v
+-------------------+
| SETREF             |  CmdSetPosition.Execute -> G_cmdSetPosition
|                   |  -> MC_SetPosition (LD POU); retract endpoint
+-------+-----------+  becomes the new coordinate-frame zero
        |
        v
+---------------+
| ST_HOME       |  PRG_Main sets G_flagHomingComplete and
| _COMPLETE     |  G_doHomingComplete; wait for mode change
+---------------+
```

### Mode 111: Reserved

Mode 111 has no state handler. It is neither an operational nor a homing
mode, so the slave confirms the handshake but stays in its current state and
raises no fault. The master must not command it.

### Mode 101: Go Home

```
+---------------+
| ST_GO_HOME    |  Entry point from mode command
+-------+-------+
        |
        v
+-------------------+
| MC_MoveAbsolute   |
| to HomePosition   |
+--------+----------+
         |
         | (At position)
         v
+---------------+
| Hold position |
| until mode    |
| change        |
+---------------+
```

---

## 4. Fault Handling Flow

```
        ANY STATE
            |
            | (Fault condition detected)
            v
    +---------------+
    | ST_FAULT      |
    |               |
    | - G_doFaultActive = HIGH
    | - Output fault code on DO0-DO2
    | - MC_Stop (motion stopped)
    +-------+-------+
            |
            +---------------------------+
            |                           |
    (Fast recovery:                (Slow recovery:
     G_diFaultReset HIGH              G_cfgFaultIdleTimeout
     + valid mirrored code          expires)
     + G_diMotionEnable LOW)              |
            |                           v
            v                   +---------------+
    +---------------+           | ST_FAULT_IDLE |
    | ST_BRAKE_HOLD |           |               |
    | (drive stays  |           | - Brake engaged
    |  enabled)     |           | - Drive disabled
    +---------------+           | - G_doFaultActive = HIGH
                                +-------+-------+
                                        |
                                (G_diFaultReset HIGH
                                 + valid mirrored code
                                 + G_diMotionEnable LOW)
                                        |
                                        v
                                +---------------+
                                | ST_IDLE       |
                                +---------------+
```

### Limit Recovery on Reset

When the cleared fault was `FAULT_LIMIT_SWITCH` or `FAULT_POSITION` **and** a limit
is still active at reset (switch held or position beyond a soft limit), the reset
routes into the recovery sub-machine instead of the normal targets above:

```
   ST_FAULT --reset, limit still active--------------------> ST_RECOVERY
   ST_FAULT_IDLE --reset, limit still active--> (DRIVE_RESET
                              -> DRIVE_ENABLE -> BRAKE_RELEASE) -> ST_RECOVERY
            (re-enable sequence; carried by bRecoveryPending)

   ST_RECOVERY  (drive ON, brake released, position held via MC_Stop)
        |  select MODE_POSITION (010) / MODE_VELOCITY (011) + G_diMotionEnable
        v
   ST_RECOVERY_POSITION / ST_RECOVERY_VELOCITY
        |  jog AWAY from the limit (toward-limit commands are clamped to zero)
        |
        +-- G_diMotionEnable toggle, limit still active --> ST_RECOVERY (keep jogging)
        +-- G_diMotionEnable toggle, limit cleared -------> ST_HOLD_POSITION (normal)

   ST_RECOVERY --select MODE_BRAKE_HOLD--> ST_BRAKE_HOLD   (manual exit, drive on)
   ST_RECOVERY --select MODE_IDLE-------> ST_BRAKE_ENGAGE  (shutdown)
```

If the limit has already cleared at reset, the normal `ST_BRAKE_HOLD` / `ST_IDLE`
targets apply (no recovery detour). Limit-switch and position faults are suppressed
while in recovery; drive and piston-exit faults stay active. See the
[Fault Code Reference](FaultCodeReference.md#limit-recovery-state-st_recovery) for
the master-side procedure.

---

## 5. Valid Mode Transitions

### From IDLE (Mode 000)

| To Mode | Condition | Path |
|---------|-----------|------|
| 001 (Brake Hold) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → BRAKE_HOLD |
| 010 (Position) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → POSITION_CTRL |
| 011 (Velocity) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → VELOCITY_CTRL |
| 100 (Torque) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → TORQUE_CTRL |
| 101 (Go Home) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → GO_HOME |
| 110 (Home Limit) | Always allowed | DRIVE_ENABLE → BRAKE_RELEASE → HOME_LIMIT |
| 111 (Reserved) | Never - no state handler | Handshake confirmed, no state change, no fault |

### From Operational Modes

| From | To | Condition | Path |
|------|----|-----------| -----|
| Any Operational | Different Mode | G_diMotionEnable LOW→HIGH | HOLD_POSITION → DRIVE_ENABLE → New Mode |
| Any Operational | IDLE (000) | G_diMotionEnable LOW (mode bits 000), or the hold-position watchdog times out | HOLD_POSITION → BRAKE_ENGAGE → DRIVE_DISABLE → IDLE (graceful, no fault) |
| Any Operational | FAULT | Fault detected | Direct transition |

### Homing Mode Exits

| From | To | Condition | Path |
|------|----|-----------| -----|
| HOME_COMPLETE | Any Mode | New mode command | Handshake → DRIVE_ENABLE → New Mode |
| Any Homing State | FAULT | Abort or timeout | Direct transition |

---

## 6. State Enumeration Reference

The enum uses dense sequential numbering (0, 1, 2, ...). Code never references numeric values directly.

```
STATE                      DESCRIPTION
-----------------------------------------------------
ST_INIT                    Default initializer (first scan -> ST_IDLE)
ST_IDLE                    Drive OFF, awaiting commands
ST_DRIVE_ENABLE            Powering on drive
ST_BRAKE_RELEASE           Releasing brake
ST_BRAKE_HOLD              Mode 001: Drive ON, brake ON
ST_POSITION_CTRL           Mode 010: Position control
ST_VELOCITY_CTRL           Mode 011: Velocity control
ST_TORQUE_CTRL             Mode 100: Torque control
ST_GO_HOME                 Mode 101: Go home sequence
ST_HOME_LIMIT              Mode 110: Homing (FB_HomeLimit)
ST_HOME_COMPLETE           Homing done
ST_HOLD_POSITION           Controlled stop, await mode
ST_BRAKE_ENGAGE            Engaging brake
ST_DRIVE_DISABLE           Powering off drive
ST_FAULT                   Fault active
ST_FAULT_IDLE              Safe powered-down fault state
ST_RECOVERY                Limit-recovery hub: drive ON, brake released, held via MC_Stop
ST_RECOVERY_POSITION       Position-control retract (direction clamped away from limit)
ST_RECOVERY_VELOCITY       Velocity-control retract (direction clamped away from limit)
```

---

## 7. Master State Machine

The master should implement a complementary state machine:

```
MASTER STATES
+----------------------------------------------------------+
|                                                          |
|  +--------+                                              |
|  | INIT   | --> Initialize DAQ, set outputs LOW          |
|  +----+---+                                              |
|       |                                                  |
|       v                                                  |
|  +--------+                                              |
|  | IDLE   | <--+  Monitor G_doFaultActive                  |
|  +----+---+    |  Mode bits = 000                        |
|       |        |                                         |
|       | (Mode request)                                   |
|       v        |                                         |
|  +----------+  |                                         |
|  | REQUEST  |  |  Set mode bits (G_diMotionEnable stays   |
|  | _MODE    |  |  LOW); start handshake timer             |
|  +----+-----+  |                                         |
|       |        |                                         |
|   +---+---+    |                                         |
|   |       |    |                                         |
| (Match:  (Timeout)                                       |
|  raise                                                    |
|  G_diMotionEnable)                                        |
|   |       |    |                                         |
|   v       v    |                                         |
|  +------+ +----+---+                                     |
|  | OPER | | TIMEOUT|  Handle timeout (wait for fault)    |
|  | ATING| | _WAIT  |                                     |
|  +--+---+ +--------+                                     |
|     |          |                                         |
|     | (Mode change)                                      |
|     v          |                                         |
|  +--------+    |                                         |
|  | STOP   |----+  Drop G_diMotionEnable, wait for halt     |
|  | _MOTION|                                              |
|  +--------+                                              |
|                                                          |
|  +--------+                                              |
|  | FAULT  | <-- When G_doFaultActive goes HIGH             |
|  | _HANDLE|     Read code, mirror, reset                 |
|  +----+---+                                              |
|       |                                                  |
|       | (Fault cleared)                                  |
|       v                                                  |
|  +--------+                                              |
|  | IDLE   |                                              |
|  +--------+                                              |
|                                                          |
+----------------------------------------------------------+
```

---

## 8. Timing Considerations

### Critical Timing Points

| Transition | Timing | Notes |
|------------|--------|-------|
| Mode request → Confirm | < 500 ms | Handshake timeout |
| G_diMotionEnable LOW → G_doInMotion LOW | Variable | Deceleration time |
| Fault detect → Fault code stable | < 10 ms | Wait before reading |
| G_diFaultReset asserted → G_doFaultActive LOW | < 1000 ms | Reset timeout (level-based, not edge) |
| Brake engage → Brake hold | 200 ms | Mechanical delay |
| Brake release → Motion allowed | 100 ms | Mechanical delay |

### State Transition Timing

```
Time ------>

G_diMotionEnable  __________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________
                            ^                        ^
                            |                        |
                         Confirmed ->              Stop
                         raised HIGH               motion

Slave State     IDLE | ENABLE | RELEASE | OPERATING | HOLD | ENGAGE | IDLE
                     |<-100ms->|<-100ms->|           |      |<-200ms>|

Mode Confirm    000_/‾‾‾\_____000________________________|__000___
                    ^   ^
                    |   |
                 Request Confirmed - handshake manager disables as
                 mode    soon as ST_IDLE is left, so confirm bits
                 bits    read 000 through OPERATING/HOLD (unless a
                         fault is active); this is normal, not a
                         loss of mode
```

---

## 9. Decision Points Summary

### At ST_IDLE
- Check: `G_diMotionEnable` rising edge?
- Check: Mode bits != 000?

### At Operational State
- Check: `G_diMotionEnable` falling edge → ST_HOLD_POSITION
- Check: Fault condition → ST_FAULT
- Check: Mode bits changed (invalid) → ST_FAULT

### At ST_HOLD_POSITION
- Check: New mode handshake started?
- Check: Handshake timeout → ST_FAULT
- Check: Mode bits = 000, or hold-position watchdog (`G_cfgHoldPositionTimeout`)
  expires → graceful shutdown to ST_IDLE via ST_BRAKE_ENGAGE → ST_DRIVE_DISABLE
  (no fault, no handshake needed)

### At ST_FAULT
- Check: Valid reset handshake?
- Check: Fault condition persists after reset?
- Check: Was it a limit/position fault with the limit still active? → route to `ST_RECOVERY`
  (from `ST_FAULT_IDLE`, re-enable the drive first) instead of `ST_BRAKE_HOLD`/`ST_IDLE`.

### At ST_RECOVERY (and sub-states)
- Check: Mode select `MODE_POSITION`/`MODE_VELOCITY` → enter retract sub-state.
- Check: Commanded direction toward the active limit → clamp command to zero (no fault).
- Check: `G_diMotionEnable` toggle → limit cleared? `ST_HOLD_POSITION` (normal) : `ST_RECOVERY` (keep jogging).
- Check: Mode select `MODE_BRAKE_HOLD`/`MODE_IDLE` → leave recovery.
