function runAll(varargin)
%RUNALL  Build the simulated slave and run every MATLAB/Octave test.
%
%   runAll            builds the simulated slave when it is stale
%                     (slave-sim/buildSlaveSim.m: slave-src translated to C
%                     and compiled with slave-sim/c), then runs each test in
%                     order and stops at the first failure.
%   runAll('nogen')   uses the slave as built (slaveSimMex must exist).
%
%   The build needs Python 3 and a C compiler: mkoctfile in Octave, one
%   configured for mex in MATLAB (MinGW-w64 on Windows).
%
%   It runs in MATLAB and in Octave and sets the paths it needs
%   relative to its own location, so from the repository root:
%
%     matlab -batch "cd tests; runAll"
%     octave-cli --eval "cd tests; runAll"
%
%   or, in an open MATLAB session, cd to tests and type runAll.
%
%   The Python tests (stTargetParity, plantParity and design/pressure-loop's)
%   are run by tests/run_all.sh, which also runs these tests, in parallel.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'master-src', 'logic'), ...
        fullfile(root, 'slave-sim'), ...
        fullfile(root, 'slave-sim', 'generated'), ...
        fullfile(root, 'tests'));

if ~any(strcmp(varargin, 'nogen'))
    buildSlaveSim();
end

tests = {'smokeMasterStateMachine', 'closedLoopMasterSlave', ...
         'closedLoopModbusStep', 'slavePressureMode', ...
         'pressureNoiseRunSim', 'benchProbeSim', 'frictionRunSim', 'charRunSim', ...
         'charFitsSynthetic'};
t0 = tic;
for k = 1:numel(tests)
    fprintf('--- %s\n', tests{k});
    feval(tests{k});
end
fprintf('--- %d test scripts passed in %.0f s\n', numel(tests), toc(t0));
end

