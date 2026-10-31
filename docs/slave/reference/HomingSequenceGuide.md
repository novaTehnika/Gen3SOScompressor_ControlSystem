# Homing Sequence Guide

## Gen3 SOS Compressor Servo Controller
### Detailed Homing Procedures

---

## 1. Overview

Homing is not required at boot. The absolute encoder keeps the motor position
across power cycles, and the zero set by Mode 110 is an `MC_SetPosition`
offset that the controller stores in battery-backed memory (flash on
Sigma-7Siec) for absolute-encoder axes. Operational modes are accepted
directly after power-up, and soft limits are enforced from boot.

Mode 110 is the only homing sequence. Mode code 111 (7) is `MODE_PRESSURE`,
an operational mode (see `docs/master/MasterProtocolGuide.md`).

| Mode | Name | Purpose | When Used |
|------|------|---------|-----------|
| 110 | Home to Limit | Establish the coordinate-frame zero using the negative overtravel switch | On demand from the master: after an encoder alarm or encoder reset (reported as `FAULT_DRIVE`), or after a controller SRAM battery failure or controller replacement, which lose the stored offset |
| 101 | Go Home | Move to the home position (`G_cfgGoHomePosition`) | User convenience |

---

## 2. Mode 110: Home to Limit Switch

### Purpose
Establish the coordinate-system zero reference using the negative overtravel switch (which doubles as the home reference). The new two-stage scheme places position 0 at a deterministic, geometrically-defined point — exactly `G_cfgHomeLimRetractDist` positive of the precise switch — so the frame is repeatable across power cycles and doesn't depend on backoff hysteresis.

> **Note on Implementation**
>
> The sequence is encapsulated in `FB_HomeLimit`. `PRG_Main` runs a single `ST_HOME_LIMIT` state that enables the FB each scan; the deprecated sub-state members (`ST_HOME_LIM_APPROACH` / `DETECT` / `BACKOFF` / `SETREF`) still exist in `E_SystemState` but are unused. The internal sub-states below (`FAST_APPROACH`, etc.) are members of the `E_HomeLimitState` enum.
>
> **Command/status routing.** `FB_HomeLimit` does not instantiate any built-in motion FBs. It emits `CmdJog`, `CmdMoveAbsolute`, and `CmdSetPosition` `VAR_OUTPUT` structs which `PRG_Main` copies into `G_cmdJog` / `G_cmdMoveAbsolute` / `G_cmdSetPosition` while `G_sysCurrentState = ST_HOME_LIMIT`; the LD POU's `Jog` (PLCopen Toolbox), `MC_MoveAbsolute`, and `MC_SetPosition` instances act on those and report back through `G_sta*` globals wired into the FB's `Sta*` `VAR_INPUT` structs. Jog is used for the indefinite-duration approach/backoff phases (level-sensitive Forward/Reverse, internal auto-decel); MoveAbsolute is used only for the final fixed-distance retract.

### Sequence Steps

```
Step 1: FAST APPROACH                Step 2: FAST AWAIT
+----------------------------------+  +---------------------------------+
| State: FAST_APPROACH             |  | State: FAST_AWAIT               |
| Action: CmdJog.Reverse := TRUE   |  | Action: CmdJog.Reverse := FALSE |
| Velocity: FastApproachVelocity   |  |  (Jog auto-decels using         |
|   (= G_cfgHomeLimFastApproachVel)|  |   Deceleration input)           |
|   default 5.0 mm/s               |  | Exit: StaJog.Done               |
| Exit: OvertravelNegActive = TRUE |  +---------------------------------+
+----------------------------------+               |
        |                                          v
        +------------------------------------------+
                                                   |
Step 3: INTER BACKOFF              Step 4: INTER AWAIT
+----------------------------------+  +---------------------------------+
| State: INTER_BACKOFF             |  | State: INTER_AWAIT              |
| Action: CmdJog.Forward := TRUE   |  | Action: CmdJog.Forward := FALSE |
| Velocity: BackoffVelocity        |  | Exit: StaJog.Done               |
|   (= G_cfgHomeLimBackoffVel)     |  +---------------------------------+
|   default 5.0 mm/s               |              |
| Exit: OvertravelNegActive = FALSE|              |
+----------------------------------+              v
        |                          +---------------------------------+
        +------------------------->|
                                   |
Step 5: SLOW APPROACH              Step 6: SLOW AWAIT
+----------------------------------+  +---------------------------------+
| State: SLOW_APPROACH             |  | State: SLOW_AWAIT               |
| Action: CmdJog.Reverse := TRUE   |  | Action: CmdJog.Reverse := FALSE |
| Velocity: SlowApproachVelocity   |  | Exit: StaJog.Done — axis now    |
|   (= G_cfgHomeLimSlowApproachVel)|  |   precisely at the switch       |
|   default 1.0 mm/s               |  +---------------------------------+
| Exit: OvertravelNegActive = TRUE |              |
+----------------------------------+              v
        |                          +---------------------------------+
        +------------------------->|
                                   |
Step 7: SETREF                     Step 8: RETRACT
+----------------------------------+  +---------------------------------+
| State: SETREF                    |  | State: RETRACT                  |
| Action: CmdSetPosition.Execute   |  | Action: CmdMoveAbsolute.Execute |
|   Position :=                    |  |   Position := SetPosition       |
|     SetPosition - RetractDist    |  |     (= G_cfgHomeLimSetPosition, |
|     (default 0 - 5 = -5 mm)      |  |      default 0.0)               |
|   Mode := FALSE (absolute)       |  |   Velocity := BackoffVelocity   |
| Effect: switch is now declared   |  |   Direction := Positive         |
|   to be at -RetractDist          |  | Effect: traverses exactly       |
| Exit: StaSetPosition.Done        |  |   RetractDist positive of switch|
+----------------------------------+  +---------------------------------+
                                                   |
                                                   v
Step 9: RETRACT AWAIT              Step 10: COMPLETE
+----------------------------------+  +---------------------------------+
| State: RETRACT_AWAIT             |  | State: ST_HOME_COMPLETE         |
| Action: CmdMoveAbsolute.Execute  |  |   (PRG_Main, not FB)            |
|   := FALSE (one-scan latch)      |  | G_sysActualPosition ≈ 0.0       |
| Exit: StaMoveAbsolute.Done       |  | G_doHomingComplete = TRUE       |
| FB sets Done = TRUE              |  | G_flagHomingComplete = TRUE     |
| PRG_Main sets                    |  | Exit: New mode commanded        |
|   G_flagHomingComplete           |  +---------------------------------+
+----------------------------------+
```

### Coordinate Frame After Homing

| Position (mm)                | Meaning                                                       |
|------------------------------|---------------------------------------------------------------|
| `0`                          | Retract endpoint — where the axis lives at end of homing.     |
| `G_cfgPosSoftLimitMin` (-2.5)| Software floor; lives between zero and the switch.            |
| `-G_cfgHomeLimRetractDist`   | Negative overtravel switch position (-5.0 mm with defaults).  |

### Timing Expectations

| Phase | Typical Duration | Maximum |
|-------|------------------|---------|
| Fast approach | 2-10 seconds | 20 seconds |
| Inter-approach backoff | <1 second | 5 seconds |
| Slow approach | 1-5 seconds | 20 seconds |
| SetReference | 50 ms | 200 ms |
| Retract (MC_MoveAbsolute) | ~1 second | 5 seconds |
| **Total** | **5-18 seconds** | **51 seconds** |

### Abort Handling

If MotionEnable (`G_mbMotionEnable`, HR3) goes LOW during homing:
1. `FB_HomeLimit` clears all `Cmd*.Execute` lines and transitions to `ABORTING`.
2. The currently-busy FB (Jog or MoveAbsolute) decelerates per its configured deceleration.
3. Once both `StaJog.Done`/`!Busy` and `StaMoveAbsolute.!Busy` are observed, the FB returns to `IDLE`.
4. `PRG_Main` transitions to `ST_HOLD_POSITION`. Homing must be restarted from the beginning — partial frame is not retained.

### Master Coordination

```
MASTER:
1. Command mode 110 (HR2 G_mbModeCode = 6)
2. Wait for confirmation (IR3 G_mbConfCode = 6)
3. Set MotionEnable HIGH (HR3 = 1)
4. Monitor progress:
   - IR6 G_mbInMotion = 1 during approach/backoff
   - IR6 G_mbInMotion = 0 during detect/setref
5. Wait for IR8 G_mbHomingComplete = 1
6. Homing complete - the new zero is stored
```

---

## 3. Mode 101: Go Home

### Purpose
Move to the home position (`G_cfgGoHomePosition`). Provides convenience for returning to a known position.

### Behavior

1. Execute MC_MoveAbsolute to `G_cfgGoHomePosition` (default: 0.0 mm)
2. Velocity: `G_cfgGoHomeVelocity`
3. Hold at position until mode change

### Master Coordination

```
MASTER:
1. Command mode 101 (HR2 G_mbModeCode = 5)
2. Wait for confirmation (IR3 G_mbConfCode = 5), then set MotionEnable HIGH (HR3 = 1)
3. Wait for IR6 G_mbInMotion = 0
4. Axis is now at home position
```

---

## 4. Homing Configuration Parameters

> These parameters are global variables documented in `slave-src/GVL/GlobalVariables_Reference.st` and configured in the MWiec GUI.

### Mode 110 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `G_cfgHomeLimFastApproachVel` | 5.0 | mm/s | Coarse approach velocity toward switch |
| `G_cfgHomeLimSlowApproachVel` | 1.0 | mm/s | Slow re-approach velocity for precise switch detect |
| `G_cfgHomeLimBackoffVel` | 5.0 | mm/s | Inter-approach backoff AND retract velocity |
| `G_cfgHomeLimRetractDist` | 5.0 | mm | Distance retracted from precise switch to position 0 |
| `G_cfgHomeLimSetPosition` | 0.0 | mm | Position value at the retract endpoint (becomes coord-frame zero) |
| `G_cfgHomeLimAccel` | 5.0 | mm/s² | Acceleration for all homing moves |
| `G_cfgHomeLimDecel` | 50.0 | mm/s² | Deceleration for all homing moves |
| `G_cfgHomingTimeout` | T#90S | — | Overall FB timeout for the entire homing sequence |

### Mode 101 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `G_cfgGoHomePosition` | 0.0 | mm | Target home position |
| `G_cfgGoHomeVelocity` | 6.0 | mm/s | Movement velocity |

---

## 5. Startup and Re-Homing

### Power-Up

```
POWER ON
    |
    v
ST_IDLE - coordinate frame retained, no homing needed
    |
    v
Ready for operational modes
```

### Re-establishing the Zero

```
Frame lost (encoder alarm / controller battery failure / controller replaced)
    |
    v
MASTER: Command Mode 110 (Home to Limit)
    |
    v
Limit homing executes; MC_SetPosition stores the new offset
    |
    v
G_doHomingComplete = TRUE (until MotionEnable, HR3, drops)
```

---

## 6. Troubleshooting

### Homing Timeout

**Symptom**: Homing doesn't complete within expected time

**Possible Causes**:
- Limit switch not triggering (wiring issue)
- Mechanical obstruction
- Velocity too slow
- Position wrong (switch in unexpected location)

**Resolution**:
1. Check limit switch wiring and function
2. Verify mechanical path is clear
3. Check homing velocity parameters
4. Manually inspect switch position

### Position Drift After Homing

**Symptom**: Position accuracy degrades over time

**Possible Causes**:
- Coordinate frame lost (encoder alarm, controller battery failure or
  controller replacement)
- Mechanical wear changing EOT position
- Thermal expansion effects

**Resolution**:
1. Re-home with Mode 110
2. Consider temperature compensation if significant

---

## 7. Quick Reference

### Homing Mode Commands

| Mode | Binary | Action |
|------|--------|--------|
| 101 | 101 | Go Home |
| 110 | 110 | Home to Limit Switch (re-establishes the zero) |

### When to Home

1. Power-up: no homing needed
2. After an encoder alarm, controller battery failure or controller replacement: Mode 110
