function runAll(varargin)
%RUNALL  Regenerate the simulated slave and run every MATLAB/Octave test.
%
%   runAll            regenerates slave-sim/generated from slave-src with
%                     tools/st2m.py, then runs each test in order and
%                     stops at the first failure.
%   runAll('nogen')   skips the regeneration (the generated files must
%                     already exist).
%
%   It runs in MATLAB and in Octave and sets the paths it needs
%   relative to its own location, so from the repository root:
%
%     matlab -batch "cd tests; runAll"
%     octave-cli --eval "cd tests; runAll"
%
%   or, in an open MATLAB session, cd to tests and type runAll.
%
%   The Python tests (st2pyParity and design/pressure-loop) are run by
%   tests/run_all.sh, which calls this function for the MATLAB side.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'master-src', 'logic'), ...
        fullfile(root, 'slave-sim'), ...
        fullfile(root, 'slave-sim', 'generated'), ...
        fullfile(root, 'tests'));

if ~any(strcmp(varargin, 'nogen'))
    generate(root);
end

tests = {'smokeMasterStateMachine', 'closedLoopMasterSlave', ...
         'closedLoopModbusStep', 'slavePressureMode', ...
         'pressureNoiseRunSim', 'frictionRunSim', 'charRunSim', ...
         'charFitsSynthetic'};
t0 = tic;
for k = 1:numel(tests)
    fprintf('--- %s\n', tests{k});
    feval(tests{k});
end
fprintf('--- %d test scripts passed in %.0f s\n', numel(tests), toc(t0));
end


function generate(root)
% Run tools/st2m.py with the first Python interpreter found: python3 on
% Linux and macOS, python or the py launcher on Windows.
cmds = {'python3', 'python', 'py -3'};
script = fullfile(root, 'tools', 'st2m.py');
src = fullfile(root, 'slave-src');
dst = fullfile(root, 'slave-sim', 'generated');
for k = 1:numel(cmds)
    cmd = sprintf('%s "%s" "%s" "%s"', cmds{k}, script, src, dst);
    [status, out] = system(cmd);
    if status == 0
        fprintf('%s', out);
        return
    end
end
error('runAll:generate', ...
      ['could not run tools/st2m.py with python3, python or py; ' ...
       'install Python 3 or call runAll(''nogen'') with the generated ' ...
       'files in place']);
end
