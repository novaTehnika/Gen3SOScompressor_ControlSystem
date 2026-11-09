function a = traceAlignNew()
%TRACEALIGNNEW Empty state of traceAlign.
%   a = traceAlignNew() returns struct with
%     held            the records traceAlign keeps back, 0 x nCol
%     lastSummarySeq  window number last given to a row, NaN before the first

L = traceLayout();
a = struct('held', zeros(0, L.nCol), 'lastSummarySeq', NaN);
end
