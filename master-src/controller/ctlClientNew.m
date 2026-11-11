function c = ctlClientNew()
%CTLCLIENTNEW The app's view of the controller before any status.
%   c = ctlClientNew() returns
%     running   the model runs
%     sim       the simulation's status text
%     live      the model has stepped, so values are readings
%     cmdSeq    the CommandSeq the model holds
%     values    struct of the ctlBlocks().read outputs (NaN unknown)
%     rows      trace records received and not yet taken by the app
%     lastSeq   newest record number the controller has drained
%     missed    records the ring overwrote before the controller read them
%     notes     texts received and not yet shown
%     statusAt  time of the last status (-Inf: none)
%     nStatus   statuses received

B = ctlBlocks();
v = struct();
for k = 1:numel(B.read)
    v.(B.read{k}) = NaN;
end
L = traceLayout();
c = struct('running', false, 'sim', '', 'live', false, 'cmdSeq', 0, ...
    'values', v, 'rows', zeros(0, L.nCol), 'lastSeq', 0, 'missed', 0, ...
    'notes', {{}}, 'statusAt', -Inf, 'nStatus', 0);
end
