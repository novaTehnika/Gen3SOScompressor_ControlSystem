function txt = shortcutScript(s)
%SHORTCUTSCRIPT The PowerShell script that writes a Windows shortcut.
%   txt = shortcutScript(s) returns the script text for a .lnk file named
%   s.name on the desktop, made with the WScript.Shell COM object:
%     name         file name without .lnk
%     allUsers     true: the Public desktop (every account; needs an
%                  administrator), false: the current user's desktop
%                  (followed where it is redirected, e.g. to OneDrive)
%     target       the program the shortcut runs
%     arguments    its command-line arguments
%     workingDir   the folder it starts in
%     icon         the icon, as 'file,index'
%     description  the shortcut's tooltip
%   The script prints the path of the shortcut it wrote. Each value is put
%   in a PowerShell single-quoted string, in which a single quote is
%   doubled and nothing else is interpreted.

folder = 'Desktop';
if s.allUsers
    folder = 'CommonDesktopDirectory';
end
lines = {
    sprintf('$desktop = [Environment]::GetFolderPath(''%s'')', folder)
    sprintf('$path = Join-Path $desktop %s', psq([s.name '.lnk']))
    '$shell = New-Object -ComObject WScript.Shell'
    '$lnk = $shell.CreateShortcut($path)'
    sprintf('$lnk.TargetPath = %s', psq(s.target))
    sprintf('$lnk.Arguments = %s', psq(s.arguments))
    sprintf('$lnk.WorkingDirectory = %s', psq(s.workingDir))
    sprintf('$lnk.IconLocation = %s', psq(s.icon))
    sprintf('$lnk.Description = %s', psq(s.description))
    '$lnk.WindowStyle = 7'
    '$lnk.Save()'
    'Write-Output $path'
    };
txt = sprintf('%s\r\n', lines{:});
end

% %% ===== LOCAL FUNCTIONS =====

% A PowerShell single-quoted string
function q = psq(v)
q = ['''' strrep(v, '''', '''''') ''''];
end
