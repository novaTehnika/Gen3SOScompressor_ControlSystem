# Stall Detection Calculation for EOT Homing (Mode 111) — **OBSOLETE**

> **This document is obsolete.** Torque-based EOT homing (Mode 111,
> `FB_HomeEOT`) has been removed. Characterization showed the ram can drive
> through a 1/8" aluminum plate with only a 1–2% torque rise above the ~8%
> unloaded baseline — within the unfiltered noise floor — so stall detection
> cannot reliably protect the PEEK piston against its weakened seal-groove
> feature. Homing is now single-phase (Mode 110 only), against the negative
> overtravel switch. The bits-`111` slot is a reserved placeholder
> (`MODE_RESERVED_111`) and commanding it faults.
>
> Retained as a record of the design rationale behind the removal.

## Purpose
This document clarifies the torque threshold calculation used for stall detection during End-of-Travel (EOT) homing in Mode 111.

---

## Specification Reference

**From devPlan_V03.md (line 433)**:
> "Stall detection: Velocity + Torque threshold (velocity < 0.5mm/s AND torque >= 90% threshold for 200ms)"

---

## Implementation Details

### Location
- **FB_HomeEOT.st**: `HE_SLOW_APPROACH` state (condition trigger) and `HE_STALL_DETECT` state (timed confirmation). `PRG_Main` routes `G_sysActualVelocity` / `G_sysActualTorque` into the FB via the `ActualVelocity` and `ActualTorque` VAR_INPUTs.

### Code (from FB_HomeEOT.st)
```st
(* Stall condition: velocity near zero AND torque at threshold.
   rVelocityThreshold := 0.5 and rTorqueMultiplier := 0.9 are FB-internal constants. *)
IF (ABS(ActualVelocity) < rVelocityThreshold) AND
   (ABS(ActualTorque) >= TorqueThreshold * rTorqueMultiplier) THEN
    (* HE_SLOW_APPROACH: proceed to HE_STALL_DETECT              *)
    (* HE_STALL_DETECT:  if also tStallConfirm.Q, proceed to HE_SETREF *)
END_IF
```

`TorqueThreshold` is wired from `G_cfgHomeEOTTorqueThresh`, so the runtime detection threshold is still `G_cfgHomeEOTTorqueThresh * 0.9`.

---

## Parameter Values (from GVL_Config.st)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `G_cfgHomeEOTTorqueThresh` | 50.0 | Torque threshold in % of rated |
| `G_cfgStallDetectTime` | 200 | Confirmation time in ms |
| `G_cfgHomeEOTSlowVel` | 5.0 | Commanded slow approach velocity (mm/s) |
| `G_cfgTorqueHomingLimit` | 60.0 | Torque limit during slow approach (%) |

---

## Calculation Breakdown

### Step 1: Torque Limit Applied During Slow Approach
During ST_HOME_EOT_SLOW, Y_DirectControl limits torque to:
- **60%** of rated torque (`G_cfgTorqueHomingLimit`)

### Step 2: Stall Detection Threshold
The stall is detected when actual torque reaches:
- **90% of 50%** = **45%** of rated torque

### Step 3: Detection Sequence
```
1. Axis approaches EOT at 5 mm/s with 60% torque limit
2. Contact with mechanical stop
3. Velocity drops as resistance increases
4. When: velocity < 0.5 mm/s AND torque >= 45%
   → Stall condition detected
5. Condition must persist for 200ms
   → Stall confirmed
6. Position reference set at current location
```

---

## Interpretation Clarification

The specification phrase "torque >= 90% threshold" could be interpreted two ways:

### Interpretation A (Implemented)
"90% of the configured torque threshold"
- G_cfgHomeEOTTorqueThresh = 50%
- 90% of 50% = **45% of rated torque**

### Interpretation B (Not Used)
"90% of the torque limit being applied"
- G_cfgTorqueHomingLimit = 60%
- 90% of 60% = **54% of rated torque**

### Rationale for Interpretation A
1. **Earlier Detection**: Lower threshold (45%) detects stall sooner, reducing mechanical stress
2. **Safety Margin**: Stall detected before torque limit is fully reached
3. **Configurable**: `G_cfgHomeEOTTorqueThresh` is specifically named for this purpose
4. **Separate Concerns**: Detection threshold is independent of torque limiting

---

## Stall Detection State Diagram

```
HE_SLOW_APPROACH (FB_HomeEOT internal state)
    |
    | Commanding: 5 mm/s with 60% torque limit
    | (via CmdDirectControl -> G_cmdDirectControl -> LD POU Y_DirectControl)
    |
    v
[Contact with EOT]
    |
    | Velocity decreasing, torque increasing
    |
    v
[Velocity < 0.5 mm/s AND Torque >= 45%]
    |
    | Stall condition met
    |
    v
HE_STALL_DETECT
    |
    | Start 200ms tStallConfirm timer
    |
    v
[Condition persists for 200ms?]
    |
   / \
  NO  YES
  |    |
  v    v
back to  HE_SETREF
HE_SLOW   (calculate EOTOffset, then HE_DONE)
APPROACH
```

Note: The homing sub-state names `ST_HOME_EOT_SLOW` / `ST_HOME_EOT_DETECT` / `ST_HOME_EOT_SETREF` still exist in `E_SystemState` but are marked DEPRECATED. The live logic runs inside `FB_HomeEOT` under a single `ST_HOME_EOT` state in `PRG_Main`.

---

## Safety Considerations

### Why Not Use Higher Thresholds?

1. **Mechanical Protection**: Lower torque threshold reduces stress on leadscrew and actuator
2. **Motor Protection**: Prevents sustained high-current operation against stall
3. **Reliable Detection**: Clear velocity drop with moderate torque indicates solid contact

### Why 200ms Confirmation?

1. **Debouncing**: Filters momentary velocity dips or torque spikes
2. **Solid Contact**: Ensures actuator has fully seated against EOT
3. **Not Too Long**: Limits sustained stall current duration

---

## Tuning Guidelines

| Parameter | Effect of Increasing | Effect of Decreasing |
|-----------|---------------------|---------------------|
| `G_cfgHomeEOTTorqueThresh` | Later detection, more force | Earlier detection, less force |
| `G_cfgStallDetectTime` | More reliable but slower | Faster but risk of false triggers |
| `G_cfgHomeEOTSlowVel` | Faster approach, harder impact | Slower approach, softer contact |
| `G_cfgTorqueHomingLimit` | More force available | Gentler but may not reach EOT |

### Recommended Starting Values (Current Defaults)
- Torque threshold: 50% (detects at 45%)
- Confirmation time: 200ms
- Slow velocity: 5 mm/s
- Torque limit: 60%

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-29 | Initial documentation |
