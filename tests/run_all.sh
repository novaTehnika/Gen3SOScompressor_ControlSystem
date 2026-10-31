#!/bin/sh
# Regenerate the simulated slave and the Python function blocks from slave-src
# and run every test: the Python tests here, the MATLAB-language tests through
# tests/runAll.m in Octave or, when octave-cli is not installed, in MATLAB.
# Usage: tests/run_all.sh   (from anywhere; needs python3 with
#        design/requirements.txt, and octave-cli or matlab on the PATH)
set -e
cd "$(dirname "$0")/.."

python3 tools/st2m.py slave-src slave-sim/generated
python3 tools/st2py.py slave-src design/generated
python3 tests/st2pyParity.py
python3 design/pressure-loop/test_model.py
python3 design/pressure-loop/test_linear.py
python3 design/pressure-loop/test_robust.py

if command -v octave-cli >/dev/null 2>&1; then
  octave-cli --quiet --eval "cd tests; runAll('nogen')"
elif command -v matlab >/dev/null 2>&1; then
  matlab -batch "cd tests; runAll('nogen')"
else
  echo "run_all.sh: neither octave-cli nor matlab found on the PATH" >&2
  exit 1
fi
