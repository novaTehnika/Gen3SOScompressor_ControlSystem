function args = launchArguments(folder)
%LAUNCHARGUMENTS The MATLAB command-line arguments of the desktop shortcut.
%   args = launchArguments(folder) starts MATLAB without its desktop and
%   splash screen, its command window minimized, and runs
%   launchOperatorApp('exit') from folder (master-src/launch), which opens
%   the operator app and ends that MATLAB when the app is closed. A single
%   quote in folder is doubled for MATLAB.

folder = strrep(folder, '''', '''''');
args = sprintf('-nosplash -nodesktop -minimize -r "cd(''%s''); launchOperatorApp(''exit'')"', ...
    folder);
end
