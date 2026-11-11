function msg = ctlStatusMessage(running, simStatus, values, rows, reader, cmdSeq, notes)
%CTLSTATUSMESSAGE The status the controller sends the app on every pass.
%   msg = ctlStatusMessage(running, simStatus, values, rows, reader,
%   cmdSeq, notes)
%
%   running    the model runs
%   simStatus  the simulation's status text
%   values     struct of the ctlBlocks().read outputs, or [] when the model
%              has no runtime values yet (live false, every value NaN)
%   rows       the trace records drained in this pass, k x
%              traceLayout().nCol
%   reader     the controller's traceDrain buffer (lastSeq, missed), or []
%   cmdSeq     the CommandSeq the model holds (the app continues from it)
%   notes      cell of texts for the app's status line
%
%   msg.rec holds the rows one after the other (row-major) and msg.cols
%   their width, so a single row and no rows decode the same way.

B = ctlBlocks();
live = isstruct(values);
v = struct();
for k = 1:numel(B.read)
    name = B.read{k};
    v.(name) = NaN;
    if live && isfield(values, name) && ~isempty(values.(name))
        v.(name) = double(values.(name)(1));
    end
end
L = traceLayout();
if isempty(rows)
    rows = zeros(0, L.nCol);
end
lastSeq = 0;
missed = 0;
if isstruct(reader)
    lastSeq = reader.lastSeq;
    missed = reader.missed;
end
msg = struct('t', 'status', 'running', logical(running), ...
    'sim', char(simStatus), 'live', live, 'cmdSeq', cmdSeq, ...
    'values', v, 'cols', L.nCol, 'rec', reshape(double(rows).', 1, []), ...
    'lastSeq', lastSeq, 'missed', missed, 'notes', {cellstr(notes)});
end
