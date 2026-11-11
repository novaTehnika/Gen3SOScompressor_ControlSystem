function app = launchOperatorApp(mode)
%LAUNCHOPERATORAPP Open the operator app with its paths set.
%   launchOperatorApp puts master-src's folders on the path, opens the
%   operator app (SOS_Gen3_Compressor_Interface) and returns at once.
%   app = launchOperatorApp returns the app, for PollStats and StallLog.
%
%   launchOperatorApp('exit') is what the desktop shortcut runs
%   (makeDesktopShortcut): it waits until the app's window is closed and
%   then ends this MATLAB. When the app cannot open, the reason is shown
%   in a dialog first, so the operator sees it instead of a window that
%   vanishes.

if nargin < 1
    mode = '';
end
here = fileparts(mfilename('fullpath'));          % master-src/launch
root = fileparts(here);                            % master-src
addpath(root, fullfile(root, 'logic'), fullfile(root, 'shell'), ...
    fullfile(root, 'trace'), fullfile(root, 'controller'), ...
    fullfile(root, 'procedure'));
cd(root);
ending = strcmp(mode, 'exit');
try
    app = SOS_Gen3_Compressor_Interface;
catch ME
    h = errordlg(sprintf('The operator app could not open:\n\n%s', ME.message), ...
        'SOS Gen3 Compressor');
    if ending
        uiwait(h);
        exit;
    end
    rethrow(ME);
end
if ending
    waitfor(app.UIFigure);
    exit;
end
end
