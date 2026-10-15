#!/bin/sh
# Regenerate the simulated slave and the Python function blocks from slave-src
# and run every test.
# Usage: tests/run_all.sh   (from anywhere; needs python3 and octave-cli)
set -e
cd "$(dirname "$0")/.."

python3 tools/st2m.py slave-src slave-sim/generated
python3 tools/st2py.py slave-src design/generated
python3 tests/st2pyParity.py
python3 design/pressure-loop/test_model.py

octave-cli --quiet --eval "
  addpath('master-src/logic', 'slave-sim', 'slave-sim/generated', 'tests');
  smokeMasterStateMachine;
  closedLoopMasterSlave;
  closedLoopModbusStep;
  slavePressureMode;
"
