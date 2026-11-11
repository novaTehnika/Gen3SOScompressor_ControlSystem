function file = makeDesktopShortcut(varargin)
%MAKEDESKTOPSHORTCUT Put an icon for the operator app on the desktop.
%   makeDesktopShortcut runs once on the lab PC, from the MATLAB that is to
%   run the app, and writes "SOS Gen3 Compressor.lnk" to the current
%   user's desktop. Double-clicking it starts that MATLAB without its
%   desktop, its command window minimized, and opens the operator app
%   (launchOperatorApp); closing the app ends that MATLAB, and the app
%   ends the controller it started. Run it again after moving the
%   repository or changing MATLAB version: the shortcut holds both paths.
%
%   makeDesktopShortcut('allUsers') writes it to the Public desktop, for
%   every account on the PC (MATLAB must run as administrator).
%   makeDesktopShortcut(..., 'name', text) names the icon otherwise.
%   file = makeDesktopShortcut(...) returns the shortcut's path.
%
%   The shortcut is written by PowerShell (shortcutScript) from a script
%   in tempdir; Windows only.

if ~ispc
    error('makeDesktopShortcut: Windows only');
end
s.name = 'SOS Gen3 Compressor';
s.allUsers = false;
k = 1;
while k <= numel(varargin)
    switch lower(varargin{k})
        case 'allusers'
            s.allUsers = true;
        case 'name'
            k = k + 1;
            s.name = varargin{k};
        otherwise
            error('makeDesktopShortcut: unknown option %s', varargin{k});
    end
    k = k + 1;
end

here = fileparts(mfilename('fullpath'));          % master-src/launch
s.target = fullfile(matlabroot, 'bin', 'matlab.exe');
s.arguments = launchArguments(here);
s.workingDir = fileparts(here);                    % master-src
s.icon = [s.target ',0'];
s.description = 'Operator app of the Gen3 SOS compressor';

script = [tempname '.ps1'];
fid = fopen(script, 'w');
if fid < 0
    error('makeDesktopShortcut: cannot write %s', script);
end
fwrite(fid, shortcutScript(s));
fclose(fid);
cleanup = onCleanup(@() delete(script));
[code, out] = system(sprintf( ...
    'powershell -NoProfile -ExecutionPolicy Bypass -File "%s"', script));
out = strtrim(out);
if code ~= 0
    error('makeDesktopShortcut: PowerShell failed: %s', out);
end
lines = strsplit(out, newline);
file = strtrim(lines{end});
fprintf('Shortcut written: %s\n  runs %s %s\n', file, s.target, s.arguments);
end
