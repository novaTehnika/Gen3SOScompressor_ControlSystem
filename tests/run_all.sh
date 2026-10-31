#!/bin/sh
# Regenerate the simulated slave from slave-src and run the tests, in
# parallel, skipping those whose inputs are unchanged since they last passed.
#
# Usage: tests/run_all.sh [--all] [-j N] [test ...]   (from anywhere)
#   --all   run every selected test, ignoring the record of passes
#   -j N    run at most N tests at once (default: the number of CPUs)
#   test    run only these tests (default: all of them)
#
# A test's inputs are its own file, the paths on its "Depends on:" header
# line and the Octave version. Their hash is recorded in .test-cache/ when
# the test passes; a later run skips the test while the hash matches.
# Needs python3, octave-cli and sha256sum.
set -e
cd "$(dirname "$0")/.."

# Longest first, so the parallel run finishes as early as it can.
ALL_TESTS="closedLoopMasterSlave closedLoopModbusStep slavePressureMode pressureNoiseRunSim smokeMasterStateMachine"
CACHE=.test-cache

force=0
jobs=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
tests=
while [ $# -gt 0 ]; do
    case $1 in
        --all) force=1 ;;
        -j) shift; jobs=$1 ;;
        -j*) jobs=${1#-j} ;;
        -h|--help) sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option $1" >&2; exit 2 ;;
        *) tests="$tests $1" ;;
    esac
    shift
done
[ -n "$tests" ] || tests=$ALL_TESTS

python3 tools/st2m.py slave-src slave-sim/generated
mkdir -p "$CACHE"
octaveVersion=$(octave-cli --version | head -n 1)

# Hash of a test's inputs. slave-sim/generated is left out: it is derived
# from slave-src and tools, which are inputs themselves.
inputHash() {
    t=tests/$1.m
    [ -f "$t" ] || { echo "no test $t" >&2; return 1; }
    deps=$(sed -n 's/^%   Depends on: //p' "$t")
    for d in $deps; do
        [ -e "$d" ] || { echo "$t depends on missing $d" >&2; return 1; }
    done
    {
        echo "$octaveVersion"
        find "$t" $deps -type f ! -path '*/generated/*' | LC_ALL=C sort | xargs sha256sum
    } | sha256sum | cut -d ' ' -f 1
}

torun=
for t in $tests; do
    h=$(inputHash "$t")
    if [ $force -eq 0 ] && [ -f "$CACHE/$t.pass" ] && [ "$(cat "$CACHE/$t.pass")" = "$h" ]; then
        echo "skip $t (unchanged since it passed)"
    else
        rm -f "$CACHE/$t.pass" "$CACHE/$t.fail"
        echo "$h" > "$CACHE/$t.hash"
        torun="$torun $t"
    fi
done
[ -n "$torun" ] || { echo "nothing to run"; exit 0; }

echo "running$torun ($jobs at a time)"
# Each test runs in its own octave-cli; its output is printed in one piece
# when it ends.
printf '%s\n' $torun | xargs -P "$jobs" -I {} sh -c '
    t=$1; c=$2
    start=$(date +%s)
    if octave-cli --quiet --eval "addpath(\"master-src/logic\", \"slave-sim\", \"slave-sim/generated\", \"tests\"); $t" \
        > "$c/$t.log" 2>&1; then
        mv "$c/$t.hash" "$c/$t.pass"; result=passed
    else
        touch "$c/$t.fail"; result=FAILED
    fi
    { echo "== $t"; cat "$c/$t.log"; echo "== $t $result in $(( $(date +%s) - start )) s"; echo; }
' sh {} "$CACHE"

failed=
for t in $torun; do
    [ -f "$CACHE/$t.fail" ] && failed="$failed $t"
done
if [ -n "$failed" ]; then
    echo "FAILED:$failed (logs in $CACHE/)"
    exit 1
fi
echo "all passed"
