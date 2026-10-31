@echo off
rem Regenerate the Python function blocks from slave-src and run every test
rem on Windows: the Python tests here, then tests\runAll.m in MATLAB, which
rem builds the simulated slave.
rem Usage: tests\run_all.bat   (from anywhere; needs Python 3 with
rem        design\requirements.txt, matlab on the PATH with a C compiler
rem        configured for mex, and that compiler as CC for the Python
rem        parity tests, e.g. set CC=gcc with MinGW-w64)
setlocal
cd /d "%~dp0.."
set PY=python
where py >nul 2>nul && set PY=py -3

%PY% tools\st_translate.py --target py slave-src design\generated || exit /b 1
%PY% tests\stTargetParity.py || exit /b 1
%PY% tests\plantParity.py || exit /b 1
%PY% design\pressure-loop\test_model.py || exit /b 1
%PY% design\pressure-loop\test_linear.py || exit /b 1
%PY% design\pressure-loop\test_robust.py || exit /b 1

matlab -batch "cd tests; runAll" || exit /b 1
endlocal
