# Position Reference Setting — Current Implementation

## Purpose
This document describes how the absolute position reference is established during homing.

## Summary

Position reference setting flows through a bare `MC_SetPosition` instance in the Ladder Diagram POU, commanded through the `G_cmdSetPosition` / `G_staSetPosition` globals.

The two homing modes set position differently:

- **Mode 110 (Home to Limit Switch)** — `FB_HomeLimit` drives `CmdSetPosition.Execute := TRUE` with the target absolute position (`G_cfgHomeLimSetPosition`, default `0.0 mm`). `PRG_Main` copies this command output into `G_cmdSetPosition`; the LD POU's `MC_SetPosition` performs the position-reference write and reports back via `G_staSetPosition.Done`.
- **Mode 111 (Home to End-of-Travel)** — `FB_HomeEOT` **does not set position**. Instead, once stall is confirmed it records the encoder position (`EOTPosition := ActualPosition`) and computes a master/slave coordinate offset (`EOTOffset := ExpectedEOTPosition - EOTPosition`). This preserves the encoder zero reference established by Mode 110.

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

| Situation                        | Encoder zero   | Master sees                             |
|----------------------------------|----------------|------------------------------------------|
| After Mode 110 only              | At home switch | Master position = encoder position       |
| After Mode 110 then Mode 111     | At home switch | Master position = encoder position + `G_posEOTOffset` |

`G_posEOTOffset` is applied in `FB_PositionOutput` when generating the analog feedback (AO0) and when translating master position commands in position-control mode.

## Where the Logic Lives

| Concern                         | Location                                   |
|---------------------------------|--------------------------------------------|
| Position reference write        | `MC_SetPosition` instance in the LD POU    |
| Command emission (Mode 110)     | `FB_HomeLimit.st`, state `HL_SETREF`       |
| Status receipt                  | `FB_HomeLimit` input `StaSetPosition`      |
| ST ↔ LD bridge                  | `G_cmdSetPosition`, `G_staSetPosition`     |
| EOT offset calculation          | `FB_HomeEOT.st`, state `HE_SETREF`         |

## Document History
| Version | Date       | Changes |
|---------|------------|---------|
| 1.0     | 2025-12-29 | Initial verification (MC_SetPosition direct call) |
| 2.0     | 2026-04-17 | Position reference via `AbsolutePositionManager` / `G_cmdEncMngr` |
| 3.0     | 2026-04-17 | AbsolutePositionManager removed; reverted to direct `MC_SetPosition` via `G_cmdSetPosition` / `G_staSetPosition` |
