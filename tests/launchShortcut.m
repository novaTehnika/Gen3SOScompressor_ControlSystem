function launchShortcut()
%LAUNCHSHORTCUT The desktop shortcut's command line and the PowerShell
%   script that writes it (launchArguments, shortcutScript), without
%   Windows: the MATLAB arguments run launchOperatorApp('exit') from the
%   launch folder, quotes in paths survive both MATLAB's and PowerShell's
%   quoting, and the script targets the user's or the Public desktop.
%
%   Run from the repository root:
%     addpath master-src/launch tests; launchShortcut
%   Depends on: master-src/launch

% ----- MATLAB arguments -----
a = launchArguments('C:\SOS\master-src\launch');
assert(strcmp(a, ['-nosplash -nodesktop -minimize -r ' ...
    '"cd(''C:\SOS\master-src\launch''); launchOperatorApp(''exit'')"']));
a = launchArguments('C:\Users\o''brien\SOS\master-src\launch');
assert(~isempty(strfind(a, 'cd(''C:\Users\o''''brien\SOS\master-src\launch'')')));
% What MATLAB runs: the text between the double quotes, evaluated
inner = regexp(a, '-r "(.*)"$', 'tokens', 'once');
cmd = inner{1};
parts = regexp(cmd, '^cd\((.*)\); launchOperatorApp\(''exit''\)$', 'tokens', 'once');
assert(strcmp(eval(parts{1}), 'C:\Users\o''brien\SOS\master-src\launch'));
fprintf('ok  arguments: no desktop, minimized, launcher with exit, quotes doubled\n');

% ----- PowerShell script -----
s = struct('name', 'SOS Gen3 Compressor', 'allUsers', false, ...
    'target', 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe', ...
    'arguments', a, 'workingDir', 'C:\Users\o''brien\SOS\master-src', ...
    'icon', 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe,0', ...
    'description', 'Operator app of the Gen3 SOS compressor');
txt = shortcutScript(s);
lines = strsplit(txt, sprintf('\r\n'));
assert(isempty(lines{end}), 'every line ends with CR LF');
lines = lines(1:end - 1);
assert(strcmp(lines{1}, '$desktop = [Environment]::GetFolderPath(''Desktop'')'));
assert(strcmp(lines{2}, '$path = Join-Path $desktop ''SOS Gen3 Compressor.lnk'''));
assert(any(strcmp(lines, ...
    '$lnk.TargetPath = ''C:\Program Files\MATLAB\R2025b\bin\matlab.exe''')));
assert(any(strcmp(lines, ...
    '$lnk.WorkingDirectory = ''C:\Users\o''''brien\SOS\master-src''')));
assert(any(strcmp(lines, '$lnk.WindowStyle = 7')));
assert(strcmp(lines{end - 1}, '$lnk.Save()') && strcmp(lines{end}, 'Write-Output $path'));
% The Arguments value, unquoted as PowerShell does, is the arguments
k = find(strncmp(lines, '$lnk.Arguments = ', 17));
v = lines{k}(18:end);
assert(v(1) == '''' && v(end) == '''');
assert(strcmp(regexprep(v(2:end - 1), '''''', ''''), a));   % regexprep: no overlapping matches
s.allUsers = true;
txt = shortcutScript(s);
first = '$desktop = [Environment]::GetFolderPath(''CommonDesktopDirectory'')';
assert(strncmp(txt, first, numel(first)));
fprintf('ok  script: user and Public desktop, values single-quoted with quotes doubled\n');
end
