function [exe, args, cmd] = ctlLaunchCommand(cfg, folder, exe, pc)
%CTLLAUNCHCOMMAND How the app starts the controller.
%   [exe, args, cmd] = ctlLaunchCommand(cfg, folder) gives the MATLAB that
%   runs the controller and its arguments: -batch "cd('<folder>');
%   runController", with folder the controller's own folder (a single
%   quote doubled for MATLAB). -batch ends that MATLAB when runController
%   returns. exe is cfg.controllerMatlab or, when empty, the running
%   MATLAB's bin/matlab.
%
%   On Windows the app starts exe with args itself, with no window
%   (launchController, through .NET). cmd is the system command for where
%   that is not available: on Windows start "<title>" /min "<exe>" args,
%   a minimized window; elsewhere "<exe>" args in the background with &.
%
%   [exe, args, cmd] = ctlLaunchCommand(cfg, folder, exe, pc) names the
%   executable and the platform (pc true for Windows) instead.

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
args = sprintf('-batch "cd(''%s''); runController"', folder);
if pc
    cmd = sprintf('start "SOS Gen3 controller" /min "%s" %s', exe, args);
else
    cmd = sprintf('"%s" %s > /dev/null 2>&1 &', exe, args);
end
end
