"""
modelpath.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/31/2026

PURPOSE/DESCRIPTION:
This module puts the repository's models/ directory on sys.path, so that
the studies can import the plant, sensor, solver and parameter modules
by their flat names ("from plant import Plant"). Importing it is enough.

FILE DEPENDENCY:
../../models/
"""

import sys
from pathlib import Path

# The repository's models/ directory
MODELS = Path(__file__).resolve().parents[2] / 'models'
if str(MODELS) not in sys.path:
    sys.path.insert(0, str(MODELS))
