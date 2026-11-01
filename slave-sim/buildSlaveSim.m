function buildSlaveSim(force)
%BUILDSLAVESIM Translate slave-src to C and compile the simulated slave's
%   MEX file, slaveSimMex, into slave-sim/generated. Runs in Octave
%   (mkoctfile --mex) and in MATLAB (mex, with a configured C compiler).
%
%   buildSlaveSim         rebuild only when a source is newer than the MEX
%   buildSlaveSim(true)   rebuild regardless
%
%   The C code is generated into slave-sim/generated-c by
%   tools/st_translate.py --target c (the slave) and models/plant_export.py
%   (the plant's parameters and initial state, from models/); the plant
%   is models/c and the rest of the hand-written part is slave-sim/c.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
gen = fullfile(here, 'generated-c');
out = fullfile(here, 'generated');
src = fullfile(here, 'c');
if ~exist(out, 'dir')
    mkdir(out);
end
mexFile = fullfile(out, ['slaveSimMex.' mexext()]);

inputs = [dir(fullfile(root, 'slave-src', '**', '*.st')); dir(fullfile(src, '*.*')); ...
          dir(fullfile(root, 'tools', '*.py')); ...
          dir(fullfile(root, 'models', '*.py')); dir(fullfile(root, 'models', 'c', '*.*'))];
if (nargin < 1 || ~force) && exist(mexFile, 'file') == 2
    built = dir(mexFile);
    if all([inputs.datenum] < built.datenum)
        return
    end
end

py = {'python3', 'python', 'py -3'};
ok = false;
for k = 1:numel(py)
    status = system(sprintf('%s "%s" --target c "%s" "%s"', py{k}, ...
        fullfile(root, 'tools', 'st_translate.py'), fullfile(root, 'slave-src'), gen));
    if status == 0
        status = system(sprintf('%s "%s" "%s"', py{k}, ...
            fullfile(root, 'models', 'plant_export.py'), fullfile(gen, 'plant_defaults.h')));
        ok = status == 0;
        break
    end
end
if ~ok
    error('buildSlaveSim:translate', ['could not run tools/st_translate.py --target c ' ...
        'and models/plant_export.py (Python 3 with design/requirements.txt)']);
end

files = {fullfile(src, 'slaveSimMex.c'), fullfile(src, 'slave_sim.c'), ...
         fullfile(root, 'models', 'c', 'plant.c'), ...
         fullfile(gen, 'st_slave.c'), fullfile(gen, 'st_names.c')};
incs = {['-I' src], ['-I' gen], ['-I' fullfile(root, 'models', 'c')]};
if exist('OCTAVE_VERSION', 'builtin')
    setenv('CFLAGS', '-O2 -std=c11');
    [out_, status] = mkoctfile('--mex', incs{:}, files{:}, '-o', mexFile);
    if status ~= 0
        error('buildSlaveSim:compile', 'mkoctfile failed:\n%s', out_);
    end
else
    mex('-silent', 'COPTIMFLAGS=-O2', incs{:}, files{:}, '-output', mexFile);
end
fprintf('built %s\n', mexFile);
end
