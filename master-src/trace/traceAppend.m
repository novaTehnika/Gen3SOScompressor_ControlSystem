function buf = traceAppend(buf, rows)
%TRACEAPPEND Append records to a trace buffer, doubling its capacity as
%   needed.
%   buf = traceAppend(buf, rows) puts the k x traceLayout().nCol matrix
%   rows after the n recorded rows of buf.data and adds k to buf.n.
%   Nothing else of buf changes. For an empty buf it first creates the
%   buffer: n 0, missed 0, no gaps and 1024 preallocated rows.

L = traceLayout();
if isempty(buf)
    buf = struct('n', 0, 'missed', 0, 'gaps', zeros(0, 2), ...
        'data', zeros(1024, L.nCol));
end
k = size(rows, 1);
if k == 0
    return
end
cap = size(buf.data, 1);
if buf.n + k > cap
    while buf.n + k > cap
        cap = 2 * cap;
    end
    buf.data(cap, L.nCol) = 0;      % grows by zero fill
end
buf.data(buf.n + (1:k), :) = rows;
buf.n = buf.n + k;
end
