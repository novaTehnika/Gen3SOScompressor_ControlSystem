# Position Reference Setting — Current Implementation

## Purpose
This document describes how the absolute position reference is established during homing.

## Summary

Position reference setting flows through a bare `MC_SetPosition` instance in the Ladder Diagram POU, commanded through the `G_cmdSetPosition` / `G_staSetPosition` globals.

For an absolute-encoder axis (Pn002.2) the controller stores the offset written by `MC_SetPosition` in battery-backed memory (flash on Sigma-7Siec; MotionWorks IEC Toolbox manual, axis parameter 1838), so the reference survives controller and drive power cycles and the slave does not require homing at boot. The offset is lost on a controller SRAM battery failure or controller replacement; Mode 110 is then run again.

Mode 110 (Home to Limit Switch) is the only sequence that sets position: `FB_HomeLimit` drives `CmdSetPosition.Execute := TRUE` with the target absolute position (`G_cfgHomeLimSetPosition`, default `0.0 mm`). `PRG_Main` copies this command output into `G_cmdSetPosition`; the LD POU's `MC_SetPosition` performs the position-reference write and reports back via `G_staSetPosition.Done`.

## Data Flow

```
FB_HomeLimit (ST, HL_SETREF state)
     │
     │  CmdSetPosition.Execute := TRUE
     │  CmdSetPosition.Position := SetPosition   (= G_cfgHomeLimSetPosition)
     │  CmdSetPosition.Mode := FALSE             (absolute)
     ▼
PRG_Main routes fbHomeLimit.CmdSetPosition -> G_cmdSetPosition
     ▼
Ladder Diagram POU
     MC_SetPosition instance (bound to G_sysAxis)
     (reads G_cmdSetPosition, writes G_staSetPosition)
     ▼
G_staSetPosition.Done  -> back into FB_HomeLimit.StaSetPosition
     │
     └─► FB_HomeLimit transitions HL_SETREF -> HL_DONE
```

## Coordinate System After Homing

| Situation                        | Position zero  | Master sees                             |
|----------------------------------|----------------|------------------------------------------|
| After Mode 110                   | `G_cfgHomeLimRetractDist` positive of the negative overtravel switch | Master position = axis position |

Position feedback is published to the master in millimeters through the Modbus input registers (`PRG_Output`), and the position reference arrives the same way; no offset is applied on either path.

## Where the Logic Lives

| Concern                         | Location                                   |
|---------------------------------|--------------------------------------------|
| Position reference write        | `MC_SetPosition` instance in the LD POU    |
| Command emission (Mode 110)     | `FB_HomeLimit.st`, state `HL_SETREF`       |
| Status receipt                  | `FB_HomeLimit` input `StaSetPosition`      |
| ST ↔ LD bridge                  | `G_cmdSetPosition`, `G_staSetPosition`     |
