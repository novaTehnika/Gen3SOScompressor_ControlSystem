function [d, out] = traceDecimate(d, rows, flush)
%TRACEDECIMATE Combine aligned records into rows of d.n exchanges.
%   [d, out] = traceDecimate(d, rows) takes the aligned records of
%   traceAlign; [d, out] = traceDecimate(d, rows, flush) with flush true
%   also emits an open interval that is not complete.
%
%   d     state of traceDecimateNew
%   rows  k x traceLayout().nCol aligned records in record order
%   out   the finished interval rows, same columns
%
%   The grid of intervals is anchored at the first record of a state:
%   an interval is seq0 + m*n ... seq0 + (m+1)*n - 1. It is emitted when
%   its last record arrives, or early when
%     a record beyond it arrives (records are missing),
%     the procStep of the next record differs, so no row spans two
%     procedure steps; the grid then starts again at that record, or
%     flush is true.
%   A row with fewer than n exchanges is partial. To change n, flush and
%   make a new state.
%
%   Over the interval's records with summaryScans > 0 the windows
%   combine as
%     mean   sum(scans .* mean) / sum(scans), for pressure, velocity,
%            torque and position
%     min    least window minimum;  max  greatest window maximum
%     summaryScans  the sum of the scans;  summarySeq  the last window's
%   and are 0 when the interval has none. Every other column, seq and
%   t_s included, is that of the last record, so seq and t_s identify the
%   exchange the snapshot comes from. nExchanges is the number of
%   records combined; a record with no window counts its exchange but not
%   scans. Rows of out span t_s - (nExchanges-1)*exchangePeriod to
%   t_s + exchangePeriod.

if nargin < 3
    flush = false;
end
L = traceLayout();
c = L.col;
nIn = size(rows, 1);
out = zeros(nIn + 1, L.nCol);
nOut = 0;
for i = 1:nIn
    r = rows(i, :);
    if d.count > 0 && (r(c.seq) > d.endSeq || r(c.procStep) ~= d.procStep)
        [d, nOut, out] = emit(d, nOut, out);
    end
    if d.count == 0
        if isnan(d.endSeq) || r(c.procStep) ~= d.procStep
            d.endSeq = r(c.seq) + d.n - 1;          % anchor the grid here
        elseif r(c.seq) > d.endSeq
            d.endSeq = d.endSeq + d.n * ceil((r(c.seq) - d.endSeq) / d.n);
        end
        d.procStep = r(c.procStep);
    end
    d.count = d.count + 1;
    d.last = r;
    s = r(c.summaryScans);
    if s > 0
        d.scans = d.scans + s;
        d.pSum = d.pSum + s * r(c.pressureMeanGauge_MPa);
        d.vSum = d.vSum + s * r(c.velocityMean_mm_s);
        d.tSum = d.tSum + s * r(c.torqueMean_pct);
        d.xSum = d.xSum + s * r(c.positionMean_mm);
        d.pMin = min(d.pMin, r(c.pressureMinGauge_MPa));
        d.pMax = max(d.pMax, r(c.pressureMaxGauge_MPa));
        d.sumSeq = r(c.summarySeq);
    end
    if r(c.seq) == d.endSeq
        [d, nOut, out] = emit(d, nOut, out);
        d.endSeq = d.endSeq + d.n;
    end
end
if flush && d.count > 0
    [d, nOut, out] = emit(d, nOut, out);
end
out = out(1:nOut, :);
end

% %% ===== LOCAL FUNCTIONS =====

function [d, nOut, out] = emit(d, nOut, out)
%EMIT Close the open interval into the next row of out and reset the sums.
L = traceLayout();
c = L.col;
r = d.last;
if d.scans > 0
    r(c.pressureMeanGauge_MPa) = d.pSum / d.scans;
    r(c.pressureMinGauge_MPa) = d.pMin;
    r(c.pressureMaxGauge_MPa) = d.pMax;
    r(c.velocityMean_mm_s) = d.vSum / d.scans;
    r(c.torqueMean_pct) = d.tSum / d.scans;
    r(c.positionMean_mm) = d.xSum / d.scans;
    r(c.summaryScans) = d.scans;
    r(c.summarySeq) = d.sumSeq;
else
    r(c.pressureMeanGauge_MPa:c.summarySeq) = 0;
end
r(c.nExchanges) = d.count;
nOut = nOut + 1;
out(nOut, :) = r;
d.count = 0;
d.scans = 0;
d.pSum = 0;
d.vSum = 0;
d.tSum = 0;
d.xSum = 0;
d.pMin = Inf;
d.pMax = -Inf;
d.sumSeq = 0;
end
