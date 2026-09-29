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

| Mode | Name | Purpose | When Used |
|------|------|---------|-----------|
| 110 | Home to Limit | Establish the coordinate-frame zero using the negative overtravel switch | On demand from the master: after an encoder alarm or encoder reset (reported as `FAULT_DRIVE`), or after a controller SRAM battery failure or controller replacement, which lose the stored offset |
| 101 | Go Home | Move to the home position (`G_cfgGoHomePosition`) | User convenience |
| 111 | Reserved | No state handler; the slave stays in its current state | Not used |

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

If `G_diMotionEnable` goes LOW during homing:
1. `FB_HomeLimit` clears all `Cmd*.Execute` lines and transitions to `ABORTING`.
2. The currently-busy FB (Jog or MoveAbsolute) decelerates per its configured deceleration.
3. Once both `StaJog.Done`/`!Busy` and `StaMoveAbsolute.!Busy` are observed, the FB returns to `IDLE`.
4. `PRG_Main` transitions to `ST_HOLD_POSITION`. Homing must be restarted from the beginning — partial frame is not retained.

### Master Coordination

```
MASTER:
1. Command mode 110 (DI0=0, DI1=1, DI2=1)
2. Set G_diMotionEnable = HIGH
3. Wait for confirmation (DO0=0, DO1=1, DO2=1)
4. Monitor progress:
   - G_doInMotion = TRUE during approach/backoff
   - G_doInMotion = FALSE during detect/setref
5. Wait for G_doHomingComplete = TRUE
6. Homing complete - the new zero is stored
```

---

## 3. Mode 111: Home to End-of-Travel

### Purpose
Calibrate the cylinder end-of-travel position by detecting mechanical stall. This ensures the position reference accounts for any mechanical variations or wear.

> **Note on Implementation**
>
> The sequence below is encapsulated within the `FB_HomeEOT` function block. `PRG_Main` runs a single `ST_HOME_EOT` state; the deprecated sub-state enum members (`ST_HOME_EOT_FAST` / `SLOW` / `DETECT` / `SETREF`) still exist in `E_SystemState` but are unused by the live state machine. The labels below (`HE_FAST_APPROACH`, etc.) are the FB's internal `INT` constants.
>
> **Command/status routing.** `FB_HomeEOT` emits `CmdMoveVelocity`, `CmdDirectControl`, and `CmdStop` `VAR_OUTPUT` structs, which `PRG_Main` copies into the matching `G_cmd*` globals so the LD POU's `MC_MoveVelocity`, `Y_DirectControl`, and `MC_Stop` instances act on them. Status flows back via `G_sta*` globals wired into `FB_HomeEOT.Sta*` inputs. Unlike Mode 110, this FB **does not** command `MC_SetPosition` — the encoder reference established by Mode 110 is preserved; Mode 111 only calculates a master/slave coordinate offset.

### Sequence Steps

```
Step 1: FAST APPROACH
+------------------------------------------+
| Step: HE_FAST_APPROACH                   |
| Action: CmdMoveVelocity -> MC_MoveVelocity|
| Velocity: FastVelocity FB input          |
|           (= G_cfgHomeEOTFastVel)        |
|           (default: +50 mm/s)            |
| Exit: Position within ApproachDist of    |
|       expected EOT                       |
|       (G_cfgHomeEOTApproachDist = 20 mm) |
+------------------------------------------+
        |
        v
Step 2: SLOW APPROACH
+------------------------------------------+
| Step: HE_SLOW_APPROACH                   |
| Action: CmdDirectControl                 |
|         -> G_cmdDirectControl            |
|         -> Y_DirectControl (velocity +   |
|            torque-limit mode)            |
| Velocity: SlowVelocity FB input          |
|           (= G_cfgHomeEOTSlowVel)        |
|           (default: +5 mm/s)             |
| Torque Limit: G_cfgTorqueHomingLimit     |
|               (default: 60%)             |
| Exit: |vel| < 0.5 mm/s AND               |
|       |torque| >= 0.9 * TorqueThreshold  |
|       (TorqueThreshold =                 |
|        G_cfgHomeEOTTorqueThresh = 50%,   |
|        effective detection at 45% rated) |
+------------------------------------------+
        |
        v
Step 3: STALL DETECT
+------------------------------------------+
| Step: HE_STALL_DETECT                    |
| Condition 1: |ActualVelocity| < 0.5 mm/s |
|              (rVelocityThreshold)        |
| Condition 2: |ActualTorque| >=           |
|              0.9 * TorqueThreshold       |
|              (rTorqueMultiplier = 0.9)   |
| Duration: G_cfgStallDetectTime (200 ms)  |
| Exit: Conditions held for duration       |
+------------------------------------------+
        |
        v
Step 4: SET REFERENCE (offset only)
+------------------------------------------+
| Step: HE_SETREF                          |
| Action: Record EOTPosition from          |
|         ActualPosition input, then       |
|   EOTOffset := ExpectedEOTPosition       |
|                - EOTPosition             |
|   where ExpectedEOTPosition =            |
|     G_cfgHomeEOTSetPosition (300 mm)     |
| Note: No MC_SetPosition call —           |
|       encoder zero from Mode 110 kept.   |
| PRG_Main writes offset to G_posEOTOffset |
+------------------------------------------+
        |
        v
Step 5: COMPLETE
+------------------------------------------+
| State: ST_HOME_COMPLETE                  |
| Action: Hold position                    |
| Output: G_doHomingComplete = TRUE          |
| Exit: New mode commanded                 |
+------------------------------------------+
```

### Stall Detection Algorithm

```
STALL DETECTION CRITERIA (from FB_HomeEOT):
  - |ActualVelocity| < rVelocityThreshold         (= 0.5 mm/s, FB constant)
  - |ActualTorque|   >= TorqueThreshold * 0.9     (TorqueThreshold input =
                                                   G_cfgHomeEOTTorqueThresh = 50%,
                                                   detection fires at 45% rated)
  - Both held for G_cfgStallDetectTime            (= 200 ms)

PSEUDOCODE:
IF ABS(ActualVelocity) < rVelocityThreshold AND
   ABS(ActualTorque)   >= TorqueThreshold * rTorqueMultiplier THEN
    tStallConfirm(IN := TRUE, PT := G_cfgStallDetectTime);
    IF tStallConfirm.Q THEN
        (* stall confirmed, proceed to HE_SETREF *)
    END_IF
ELSE
    tStallConfirm(IN := FALSE);    (* resets on any criterion failure *)
END_IF
```

### Timing Expectations

| Phase | Typical Duration | Maximum |
|-------|------------------|---------|
| Fast Approach | 3-5 seconds | 10 seconds |
| Slow Approach | 1-3 seconds | 10 seconds |
| Stall Detect | 200-500 ms | 2 seconds |
| Set Reference | 50 ms | 200 ms |
| **Total** | **5-9 seconds** | **22 seconds** |

### Safety Considerations

- Torque limit prevents motor/mechanical damage during stall
- Slow approach velocity reduces impact energy
- Stall confirmation time prevents false triggers
- If stall not detected within timeout, fault is raised

### Master Coordination

```
MASTER:
1. Ensure Mode 110 completed (if encoder was invalid)
2. Command mode 111 (DI0=1, DI1=1, DI2=1)
3. Set G_diMotionEnable = HIGH
4. Wait for confirmation (DO0=1, DO1=1, DO2=1)
5. Monitor progress:
   - G_doInMotion = TRUE during approach phases
   - G_doInMotion = FALSE when stall detected
6. Wait for G_doHomingComplete = TRUE
7. Homing complete - full position calibration done
```

---

## 4. Mode 101: Go Home

### Purpose
Move to the home position (`G_cfgGoHomePosition`). Provides convenience for returning to a known position.

### Behavior

1. Execute MC_MoveAbsolute to `G_cfgGoHomePosition` (default: 0.0 mm)
2. Velocity: `G_cfgGoHomeVelocity`
3. Hold at position until mode change

### Master Coordination

```
MASTER:
1. Command mode 101 (DI0=1, DI1=0, DI2=1)
2. Wait for confirmation, then set G_diMotionEnable = HIGH
3. Wait for G_doInMotion = FALSE
4. Axis is now at home position
```

---

## 5. Homing Configuration Parameters

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
| `G_cfgHomeLimDecel` | 5.0 | mm/s² | Deceleration for all homing moves |
| `G_cfgHomingTimeout` | T#30S | — | Overall FB timeout for the entire homing sequence |

### Mode 111 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `G_cfgHomeEOTFastVel` | 50.0 | mm/s | Fast approach velocity |
| `G_cfgHomeEOTApproachDist` | 20.0 | mm | Distance from expected EOT at which to switch to slow approach |
| `G_cfgHomeEOTSlowVel` | 5.0 | mm/s | Slow torque-limited approach velocity |
| `G_cfgTorqueHomingLimit` | 60.0 | % | Torque limit applied during slow approach |
| `G_cfgHomeEOTTorqueThresh` | 50.0 | % | Stall detection torque threshold (actual trigger at 90% × threshold = 45%) |
| `G_cfgStallDetectTime` | T#200MS | — | Stall confirmation duration |
| `G_cfgHomeEOTSetPosition` | 300.0 | mm | Expected master-view position at EOT (used for offset calculation) |
| `G_cfgHomingTimeout` | T#30S | — | Overall sequence timeout |

Notes:
- The stall velocity threshold (`0.5 mm/s`) and torque multiplier (`0.9`) are FB-internal constants inside `FB_HomeEOT` (`rVelocityThreshold`, `rTorqueMultiplier`) and are not exposed as configurable globals.
- See [Stall_Detection_Calculation](../Stall_Detection_Calculation.md) for the full derivation.

### Mode 101 Parameters

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `G_cfgGoHomePosition` | 0.0 | mm | Target home position |
| `G_cfgGoHomeVelocity` | 50.0 | mm/s | Movement velocity |

---

## 6. Startup and Re-Homing

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
G_doHomingComplete = TRUE (until G_diMotionEnable drops)
```

---

## 7. Troubleshooting

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

### Stall Not Detected (Mode 111)

**Symptom**: EOT homing runs but never confirms stall

**Possible Causes**:
- Torque limit too high (motor doesn't reach threshold)
- Stall velocity threshold too low
- Mechanical friction too high
- Not actually reaching EOT

**Resolution**:
1. Reduce torque limit parameter
2. Increase stall velocity threshold slightly
3. Check mechanical friction
4. Verify expected EOT position

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

## 8. Quick Reference

### Homing Mode Commands

| Mode | Binary | Action |
|------|--------|--------|
| 101 | 101 | Go Home |
| 110 | 110 | Home to Limit Switch (re-establishes the zero) |
| 111 | 111 | Reserved |

### When to Home

1. Power-up: no homing needed
2. After an encoder alarm, controller battery failure or controller replacement: Mode 110
