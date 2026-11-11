function cmd = ctlLaunchCommand(cfg, folder, exe, pc)
%CTLLAUNCHCOMMAND The system command that starts the controller.
%   cmd = ctlLaunchCommand(cfg, folder) starts a second MATLAB, minimized
%   and without its desktop, that runs runController from folder (the
%   controller's own folder) and returns at once, so the app goes on.
%   cmd = ctlLaunchCommand(cfg, folder, exe, pc) names the MATLAB
%   executable and the platform (pc true for Windows) instead of taking
%   cfg.controllerMatlab (or, when empty, the running MATLAB's) and ispc.
%
%   On Windows the command is start "<title>" /min "<exe>" -batch "...";
%   elsewhere it runs the same in the background with &. -batch ends the
%   MATLAB when runController returns.

if nargin < 3 || isempty(exe)
    exe = cfg.controllerMatlab;
    if isempty(exe)
        exe = fullfile(matlabroot, 'bin', 'matlab');
    end
end
if nargin < 4
    pc = ispc;
end
folder = strrep(folder, '''', '''''');
script = sprintf('cd(''%s''); runController', folder);
if pc
    cmd = sprintf('start "SOS Gen3 controller" /min "%s" -batch "%s"', exe, script);
else
    cmd = sprintf('"%s" -batch "%s" > /dev/null 2>&1 &', exe, script);
end
end
