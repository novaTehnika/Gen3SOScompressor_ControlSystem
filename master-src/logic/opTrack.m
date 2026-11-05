function [confirmed, ended] = opTrack(op, confirmed, statusCode, sinceSent, cfg)
%OPTRACK Follow an operation requested from the app through StatusCode.
%   [confirmed, ended] = opTrack(op, confirmed, statusCode, sinceSent, cfg)
%
%   op          operation code the app wrote to RequestedMode: 0 stop,
%               1 home, 2 position, 3 jog, 4 pressure, 6 go home
%   confirmed   true once the operation's running status has been seen
%   statusCode  the model's StatusCode
%   sinceSent   s since the request was sent
%   cfg         masterConfig (opConfirmWait)
%
%   The operation is confirmed once its running status appears and ended
%   when that status goes. Before confirmation, starting (7) and stopping
%   (6), as when switching from another operation, are waited out and any
%   other status ends it once sinceSent has reached cfg.opConfirmWait.
%   op = 0 (stop) never ends. The app and the procedure engine track their
%   operations with this one rule.

STATUS_STOPPING = 6;
STATUS_STARTING = 7;

ended = false;
if op == 0
    return
end
if any(statusCode == opRunningStatus(op))
    confirmed = true;
elseif confirmed
    ended = true;
elseif statusCode ~= STATUS_STARTING && statusCode ~= STATUS_STOPPING && ...
        sinceSent >= cfg.opConfirmWait
    ended = true;
end
end

function codes = opRunningStatus(op)
% Status codes that show the operation running.
switch op
    case 1, codes = 1;       % home
    case 6, codes = 8;       % go home
    case 2, codes = [2 3];   % position
    case 3, codes = 4;       % jog
    case 4, codes = 5;       % pressure
    otherwise, codes = [];
end
end
