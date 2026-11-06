function d = traceDecimateNew(n)
%TRACEDECIMATENEW Empty state of traceDecimate for intervals of n exchanges.
%   d = traceDecimateNew(n) returns struct with
%     n        exchanges per interval
%     count    exchanges in the open interval, 0 when none
%     endSeq   record number that ends the open interval, NaN before the
%              first record (the grid is anchored there)
%     procStep procStep of the open or last interval, NaN before the first
%     scans    sum of the scans of the interval's windows
%     pSum, vSum, tSum, xSum  scan-weighted sums of the pressure,
%              velocity, torque and position means
%     pMin, pMax  least minimum and greatest maximum of the windows
%     sumSeq   number of the last window
%     last     the last record of the interval, 1 x traceLayout().nCol

L = traceLayout();
d = struct('n', n, 'count', 0, 'endSeq', NaN, 'procStep', NaN, 'scans', 0, ...
    'pSum', 0, 'vSum', 0, 'tSum', 0, 'xSum', 0, 'pMin', Inf, 'pMax', -Inf, ...
    'sumSeq', 0, 'last', zeros(1, L.nCol));
end
