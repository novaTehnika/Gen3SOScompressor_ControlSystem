@echo off
rem Regenerate the simulated slave and the Python function blocks from
rem slave-src and run every test on Windows: the Python tests here, the
rem MATLAB-language tests through tests\runAll.m in MATLAB.
rem Usage: tests\run_all.bat   (from anywhere; needs Python 3 with
rem        design\requirements.txt, and matlab on the PATH)
setlocal
cd /d "%~dp0.."
set PY=python
where py >nul 2>nul && set PY=py -3

%PY% tools\st2m.py slave-src slave-sim\generated || exit /b 1
%PY% tools\st2py.py slave-src design\generated || exit /b 1
%PY% tests\st2pyParity.py || exit /b 1
%PY% design\pressure-loop\test_model.py || exit /b 1
%PY% design\pressure-loop\test_linear.py || exit /b 1
%PY% design\pressure-loop\test_robust.py || exit /b 1

matlab -batch "cd tests; runAll('nogen')" || exit /b 1
endlocal
