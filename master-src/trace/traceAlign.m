function [a, out] = traceAlign(a, raw, final)
%TRACEALIGN Move each window summary onto the record whose write opened
%   the window.
%   [a, out] = traceAlign(a, raw) takes the raw records of traceDrain.
%   [a, out] = traceAlign(a, raw, final) with final true also emits the
%   records still held.
%
%   a     state of traceAlignNew; the last two records are kept in
%         a.held between calls
%   raw   k x traceLayout().nCol records in record order (empty allowed)
%   out   the records that are complete, in order
%
%   The slave publishes a window one to two exchanges after it closes, so
%   record k+2 carries the summary (columns pressureMeanGauge_MPa to
%   summarySeq) of the window opened by the write of record k. Row k of
%   out takes the summary columns of record k+2 when
%     seq(k+2) == seq(k) + 2,
%     summaryScans(k+2) > 0 (the slave had published a window) and
%     summarySeq(k+2) differs from the window last given to a row (a
%     late exchange merged with the next is read twice, and its window
%     counts once).
%   Otherwise the row has no window: the six summary values, summaryScans
%   and summarySeq are 0 (zero, not NaN, so the file stays finite;
%   summaryScans 0 marks it). With final the held records are emitted
%   with no window, as nothing follows them. The result does not depend
%   on how the records are split between calls.

L = traceLayout();
c = L.col;
sumCols = c.pressureMeanGauge_MPa:c.summarySeq;
rows = [a.held; raw];
n = size(rows, 1);
nOut = max(n - 2, 0);
if final
    nOut = n;
end
out = zeros(nOut, L.nCol);
for k = 1:nOut
    out(k, :) = rows(k, :);
    out(k, sumCols) = 0;
    if k + 2 <= n
        s = rows(k + 2, c.summarySeq);
        if rows(k + 2, c.seq) == rows(k, c.seq) + 2 ...
                && rows(k + 2, c.summaryScans) > 0 ...
                && ~(s == a.lastSummarySeq)
            out(k, sumCols) = rows(k + 2, sumCols);
            a.lastSummarySeq = s;
        end
    end
end
a.held = rows(nOut + 1:n, :);
end
