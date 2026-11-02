function buf = traceDrain(buf, ring)
%TRACEDRAIN Append the records of the ring that are newer than the buffer.
%   buf = traceDrain(buf, ring) takes the valid rows of ring whose record
%   number is above buf.lastSeq (see traceStart for the buffer and for
%   validity), puts them in record order and appends the leading
%   consecutive run to buf.data, doubling its capacity as needed. The
%   rows after a missing number stay in the ring and are taken by a later
%   call, once the row between is written.
%
%   When the first new record is not buf.lastSeq + 1, the ring overwrote
%   records before they were read: the number lost is added to buf.missed
%   and [first new number, number lost] is appended to buf.gaps. The
%   record numbers in buf.data then jump there.
%
%   A ring of another size than traceLayout() (readBlock returns the
%   scalar 0 when the model is not running) leaves buf unchanged.

L = traceLayout();
[ring, ok] = traceRingCheck(ring);
if ~ok
    return
end

seq = ring(:, L.col.seq);
valid = seq > buf.lastSeq & ring(:, L.guard) == seq;
if ~any(valid)
    return
end
rows = find(valid);
[s, order] = sort(seq(valid));
rows = rows(order);

% Leading consecutive run in record order.
brk = find(diff(s) ~= 1, 1);
if ~isempty(brk)
    s = s(1:brk);
    rows = rows(1:brk);
end

lost = s(1) - buf.lastSeq - 1;
if lost > 0
    buf.missed = buf.missed + lost;
    buf.gaps(end + 1, :) = [s(1), lost];
end

nNew = numel(s);
cap = size(buf.data, 1);
if buf.n + nNew > cap
    while buf.n + nNew > cap
        cap = 2 * cap;
    end
    buf.data(cap, L.nCol) = 0;      % grows by zero fill
end
buf.data(buf.n + (1:nNew), :) = ring(rows, 1:L.nCol);
buf.n = buf.n + nNew;
buf.lastSeq = s(end);
end
