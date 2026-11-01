"""
studyparams.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/31/2026

PURPOSE/DESCRIPTION:
This module defines StudyParams, the parameters of the design studies.
It extends the models' Params (models/params.py) with the settings that
belong to the studies rather than to the plant: the valve timing and the
procedure, the legacy controller's filter, the process target and the
requirements the metrics judge against.

A study overrides a model default by declaring the field again here with
its own default, and changes any value for one run with with_() or the
--set option, as for Params.

FILE DEPENDENCY:
./
    modelpath.py
../../models/
    params.py
"""

from dataclasses import dataclass

import modelpath  # noqa: F401  (puts models/ on sys.path)
from params import Params


@dataclass(frozen=True)
class StudyParams(Params):
    """Model parameters (Params) and the settings of the studies.

    StudyParams is immutable. Use with_() to make a copy with changed
    values.
    """

    # ----- Outlet procedure ----------------------------------------------
    # [s] Valve opening time; 0 opens the valve in one scan. The ball
    # valve is opened rapidly.
    Topen: float = 0.0
    # [s] Valve closing time at the end of the purge; 0 closes it in
    # one scan. Set from the hand's speed on the ball valve.
    Tclose: float = 0.0
    # True if the proximal valve is opened before the compression, so
    # the hose is part of the chamber throughout (the proposed
    # procedure); False if it is opened after mixing, at pressure (the
    # present procedure), which costs the equalisation drop of eq. (43).
    hose_before: bool = False
    # [s] Opening time of the proximal valve; 0 for one scan
    Topen_prox: float = 0.0

    # ----- Legacy controller ---------------------------------------------
    # [s] Time constant of the derivative filter in the legacy
    # controller
    Tf: float = 0.02

    # ----- Process -------------------------------------------------------
    # [Pa] Target pressure for the compression, hold and delivery
    P_target: float = 10e6

    # ----- Metrics and requirements --------------------------------------
    # [Pa] Half-width of the band used for the settling time
    tol: float = 0.1e6
    # [Pa] Required margin of the chamber pressure above the saturation
    # pressure during the hold and delivery (no degassing)
    degas_margin: float = 0.5e6
    # [Pa] Highest pressure allowed at any time (mechanical limit)
    P_ceiling: float = 12e6
    # [Pa] Largest error tolerated during a delivery once the opening
    # dip has had t_recover to settle; a larger error counts as loss of
    # control
    e_crash: float = 1.0e6
    # [s] Time after the valve opens allowed for the opening dip to
    # recover; the dip metric is the lowest pressure in this window
    t_recover: float = 5.0
