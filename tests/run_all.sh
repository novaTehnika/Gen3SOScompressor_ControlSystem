#!/bin/sh
# Regenerate the simulated slave from slave-src and run every test.
# Usage: tests/run_all.sh   (from anywhere; needs python3 and octave-cli)
set -e
cd "$(dirname "$0")/.."

python3 tools/st2m.py slave-src slave-sim/generated

octave-cli --quiet --eval "
  addpath('master-src/logic', 'slave-sim', 'slave-sim/generated', 'tests');
  smokeMasterStateMachine;
  closedLoopMasterSlave;
  closedLoopMasterStep;
"
