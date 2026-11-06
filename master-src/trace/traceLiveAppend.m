function b = traceLiveAppend(b, rows)
%TRACELIVEAPPEND Add aligned records to the live plot's buffer.
%   b = traceLiveAppend(b, rows) converts the k x traceLayout().nCol
%   aligned records (traceAlign) into the buffer's columns
%   (traceLiveBuffer) and writes them after the newest row, overwriting
%   the oldest when it is full. The pressure is gauge MPa * atmPerMPa +
%   atmAmbient and the position posEOT - position.
%
%   A row with summaryScans 0 has no window: its mean, minimum and
%   maximum columns are NaN. When a record number jumps by more than 1
%   from the newest row (b.lastSeq, or the previous row), one all-NaN row
%   with the time of the next row is inserted before it, so the lines
%   break there.

k = size(rows, 1);
if k == 0
    return
end
L = traceLayout();
c = L.col;
m = [rows(:, c.t_s), ...
    rows(:, c.pressureGauge_MPa) * b.atmPerMPa + b.atmAmbient, ...
    rows(:, c.pressureMeanGauge_MPa) * b.atmPerMPa + b.atmAmbient, ...
    rows(:, c.pressureMinGauge_MPa) * b.atmPerMPa + b.atmAmbient, ...
    rows(:, c.pressureMaxGauge_MPa) * b.atmPerMPa + b.atmAmbient, ...
    b.posEOT - rows(:, c.position_mm), ...
    rows(:, c.velocity_mm_s), rows(:, c.torque_pct)];
m(rows(:, c.summaryScans) == 0, 3:5) = NaN;

jump = diff([b.lastSeq; rows(:, c.seq)]) > 1;
if b.lastSeq == 0
    jump(1) = false;                % nothing precedes the first record
end
if any(jump)
    at = find(jump);
    nj = numel(at);
    mm = NaN(k + nj, 8);
    dest = (1:k)' + cumsum(jump);       % rows of the records in mm
    mm(dest, :) = m;
    mm(dest(at) - 1, 1) = m(at, 1);     % the NaN row takes the next row's time
    m = mm;
end
b.lastSeq = rows(end, c.seq);

cap = size(b.data, 1);
k = size(m, 1);
if k > cap
    m = m(k - cap + 1:end, :);
    k = cap;
end
idx = mod(b.head - 1 + (0:k - 1)', cap) + 1;
b.data(idx, :) = m;
b.head = mod(b.head - 1 + k, cap) + 1;
b.count = min(cap, b.count + k);
end
