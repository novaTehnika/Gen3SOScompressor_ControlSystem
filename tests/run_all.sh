#!/bin/sh
# Regenerate the simulated slave and the Python function blocks from
# slave-src and run the tests: the Python tests with python3, the
# MATLAB-language tests in Octave or, when octave-cli is not installed, in
# MATLAB. Tests run in parallel and are skipped while their inputs are
# unchanged since they last passed.
#
# Usage: tests/run_all.sh [--all] [-j N] [test ...]   (from anywhere)
#   --all   run every selected test, ignoring the record of passes
#   -j N    run at most N tests at once (default: the number of CPUs with
#           Octave, 1 with MATLAB)
#   test    run only these tests, by name without extension (default: all)
#
# A test's inputs are its own file, the paths on its "Depends on:" header
# line, the interpreter version and the simulated slave (SLAVE_SIM=c, the
# compiled one, built here by slave-sim/buildSlaveSim.m, or SLAVE_SIM=m, the
# generated Octave functions). Their hash is recorded in .test-cache/
# when the test passes; a later run skips the test while the hash matches.
# Needs python3 with design/requirements.txt, octave-cli or matlab, and
# sha256sum.
set -e
cd "$(dirname "$0")/.."

# Longest first, so the parallel run finishes as early as it can. The
# MATLAB-language tests are also listed in tests/runAll.m.
ALL_TESTS="
tests/slaveSimParity.m
tests/frictionRunSim.m
tests/charRunSim.m
tests/closedLoopMasterSlave.m
tests/closedLoopModbusStep.m
tests/slavePressureMode.m
tests/charFitsSynthetic.m
tests/pressureNoiseRunSim.m
design/pressure-loop/test_model.py
design/pressure-loop/test_robust.py
tests/stTargetParity.py
design/pressure-loop/test_linear.py
tests/smokeMasterStateMachine.m
"
CACHE=.test-cache

force=0
jobs=
names=
while [ $# -gt 0 ]; do
    case $1 in
        --all) force=1 ;;
        -j) shift; jobs=$1 ;;
        -j*) jobs=${1#-j} ;;
        -h|--help) sed -n '2,/^set -e/p' "$0" | sed -n 's/^# \{0,1\}//p'; exit 0 ;;
        -*) echo "unknown option $1" >&2; exit 2 ;;
        *) names="$names $1" ;;
    esac
    shift
done

if command -v octave-cli >/dev/null 2>&1; then
    mrunner=octave
    mversion=$(octave-cli --version | head -n 1)
    [ -n "$jobs" ] || jobs=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
elif command -v matlab >/dev/null 2>&1; then
    mrunner=matlab
    mversion=matlab
    [ -n "$jobs" ] || jobs=1
else
    echo "run_all.sh: neither octave-cli nor matlab found on the PATH" >&2
    exit 1
fi
pversion=$(python3 --version)
mkdir -p "$CACHE"

tests=
if [ -z "$names" ]; then
    tests=$ALL_TESTS
else
    for n in $names; do
        match=
        for t in $ALL_TESTS; do
            b=${t##*/}
            [ "${b%.*}" = "$n" ] && match=$t
        done
        [ -n "$match" ] || { echo "no test named $n" >&2; exit 2; }
        tests="$tests $match"
    done
fi

python3 tools/st_translate.py --target m slave-src slave-sim/generated
python3 tools/st_translate.py --target py slave-src design/generated

# The compiled simulated slave, unless SLAVE_SIM=m asks for the generated
# Octave one; without a working compiler the tests fall back to it.
if [ "${SLAVE_SIM:-c}" = c ]; then
    if [ $mrunner = octave ]; then
        build="octave-cli --quiet --eval"
    else
        build="matlab -batch"
    fi
    if $build "addpath('slave-sim'); buildSlaveSim" >"$CACHE/build.log" 2>&1; then
        SLAVE_SIM=c
    else
        cat "$CACHE/build.log" >&2
        echo "run_all.sh: could not build the compiled slave; using SLAVE_SIM=m" >&2
        SLAVE_SIM=m
    fi
    rm -f "$CACHE/build.log"
fi
export SLAVE_SIM
mversion="$mversion slave=$SLAVE_SIM"

# Hash of a test's inputs. Generated directories are left out: they are
# derived from slave-src and tools, which are inputs themselves.
inputHash() {
    deps=$(sed -n 's/^[%#[:space:]]*Depends on: //p' "$1" | head -n 1)
    [ -n "$deps" ] || { echo "$1 has no Depends on: line" >&2; return 1; }
    for d in $deps; do
        [ -e "$d" ] || { echo "$1 depends on missing $d" >&2; return 1; }
    done
    case $1 in *.py) echo "$pversion" ;; *) echo "$mversion" ;; esac
    find "$1" $deps -type f ! -path '*/generated/*' ! -path '*/__pycache__/*' \
        | LC_ALL=C sort | xargs sha256sum
}

torun=
for t in $tests; do
    key=$(echo "$t" | tr / _)
    h=$(inputHash "$t" | sha256sum | cut -d ' ' -f 1)
    if [ $force -eq 0 ] && [ -f "$CACHE/$key.pass" ] && [ "$(cat "$CACHE/$key.pass")" = "$h" ]; then
        echo "skip $t (unchanged since it passed)"
    else
        rm -f "$CACHE/$key.pass" "$CACHE/$key.fail"
        echo "$h" > "$CACHE/$key.hash"
        torun="$torun $t"
    fi
done
[ -n "$torun" ] || { echo "nothing to run"; exit 0; }

echo "running $(echo $torun | wc -w) tests, $jobs at a time"
# Each test runs in its own process; its output is printed in one piece
# when it ends.
printf '%s\n' $torun | xargs -P "$jobs" -I {} sh -c '
    t=$1; c=$2; mrunner=$3
    key=$(echo "$t" | tr / _)
    b=${t##*/}; name=${b%.*}
    paths="addpath(\"master-src/logic\", \"slave-sim\", \"slave-sim/generated\", \"tests\")"
    start=$(date +%s)
    case $t in
        *.py) set -- python3 "$t" ;;
        *) if [ "$mrunner" = octave ]; then
               set -- octave-cli --quiet --eval "$paths; $name"
           else
               set -- matlab -batch "$(echo "$paths" | tr \" \047); $name"
           fi ;;
    esac
    if "$@" > "$c/$key.log" 2>&1; then
        mv "$c/$key.hash" "$c/$key.pass"; result=passed
    else
        touch "$c/$key.fail"; result=FAILED
    fi
    { echo "== $t"; cat "$c/$key.log"; echo "== $t $result in $(( $(date +%s) - start )) s"; echo; }
' sh {} "$CACHE" "$mrunner"

failed=
for t in $torun; do
    key=$(echo "$t" | tr / _)
    [ -f "$CACHE/$key.fail" ] && failed="$failed $t"
done
if [ -n "$failed" ]; then
    echo "FAILED:$failed (logs in $CACHE/)"
    exit 1
fi
echo "all passed"
