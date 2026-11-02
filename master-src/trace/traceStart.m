function buf = traceStart(ring)
%TRACESTART Empty trace buffer that continues after the newest record
%   already in the ring.
%   buf = traceStart(ring) returns the buffer traceDrain appends to:
%
%     lastSeq   newest record number taken so far: the largest valid one
%               in ring, 0 when there is none or the ring is unusable
%     n         records held in data
%     missed    records lost to ring overruns, summed
%     gaps      one row [first seq after the loss, records lost] per loss
%     data      preallocated records, traceLayout().nCol columns; the
%               recorded ones are buf.data(1:buf.n, :)
%
%   A record is valid when its number is above 0 and equals its guard
%   (traceLayout), so a row read while being written does not count. The
%   records already in the ring belong to before the start and are not
%   taken.

L = traceLayout();
[ring, ok] = traceRingCheck(ring);
lastSeq = 0;
if ok
    seq = ring(:, L.col.seq);
    valid = seq > 0 & ring(:, L.guard) == seq;
    if any(valid)
        lastSeq = max(seq(valid));
    end
end
buf = struct('lastSeq', lastSeq, 'n', 0, 'missed', 0, ...
    'gaps', zeros(0, 2), 'data', zeros(1024, L.nCol));
end
