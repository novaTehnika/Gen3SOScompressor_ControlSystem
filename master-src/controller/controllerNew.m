function b = controllerNew()
%CONTROLLERNEW State of the controller's decisions (controllerStep).
%   b = controllerNew() at the controller's start, time 0:
%     cmdSeq     the CommandSeq the model holds, as last written
%     lastMsgAt  time of the app's last message (-Inf: none yet)
%     aliveAt    last time the app counted as present
%     appLost    the stop for the present loss of the app is issued
%     stopping   a stop of the model waits for the axis to stop
%     stopSince  when that wait began
%     quitting   the controller ends once the model is stopped
%     sentAt     time of the last status or keepalive to the app

b = struct('cmdSeq', 0, 'lastMsgAt', -Inf, 'aliveAt', 0, ...
    'appLost', false, 'stopping', false, 'stopSince', 0, ...
    'quitting', false, 'sentAt', -Inf);
end
