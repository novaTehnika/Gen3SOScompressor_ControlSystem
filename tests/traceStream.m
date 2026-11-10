function traceStream()
%TRACESTREAM Host side of the trace recorder: the alignment of the window
%   summaries (traceAlign), the combination of records into intervals
%   (traceDecimate, traceIntervalExchanges) and the live plot's buffer and
%   figure (traceLive*), on synthetic records.
%
%   The synthetic raw records follow the slave's lag: record k+2 carries
%   the window of record k, with a window number, 10 to 12 scans and
%   per-scan samples that the expected values are computed from.
%
%   Run from the repository root:
%     addpath master-src/logic master-src/trace tests; traceStream
%   Depends on: master-src/logic master-src/trace

L = traceLayout();
c = L.col;
cfg = masterConfig();
sumCols = c.pressureMeanGauge_MPa:c.summarySeq;

% ----- Alignment -----
nRec = 200;
[raw, win] = synthRaw(nRec);
chunks = chunkSizes(nRec);

[a, out] = traceAlign(traceAlignNew(), raw, false);
assert(size(out, 1) == nRec - 2 && size(a.held, 1) == 2);
assert(isequal(a.held, raw(nRec - 1:nRec, :)));
for k = 1:nRec - 2
    assert(out(k, c.summarySeq) == k && out(k, c.summaryScans) == win(k).scans);
    assert(isequal(out(k, sumCols), raw(k + 2, sumCols)));
    assert(isequal(out(k, setdiff(1:L.nCol, sumCols)), raw(k, setdiff(1:L.nCol, sumCols))));
end
% The same in random chunks
a2 = traceAlignNew();
out2 = zeros(0, L.nCol);
at = 0;
for i = 1:numel(chunks)
    [a2, o] = traceAlign(a2, raw(at + 1:at + chunks(i), :), false);
    out2 = [out2; o]; %#ok<AGROW>
    at = at + chunks(i);
end
assert(isequal(out2, out) && isequal(a2, a));
% final emits the held rows with no window
[~, outF] = traceAlign(a, zeros(0, L.nCol), true);
assert(size(outF, 1) == 2 && all(outF(:, c.summaryScans) == 0));
assert(isequal(outF(:, sumCols), zeros(2, numel(sumCols))));
assert(isequal(outF(:, c.seq)', [nRec - 1, nRec]));
[~, full] = traceAlign(traceAlignNew(), raw, true);
assert(isequal(full(1:nRec - 2, :), out) && size(full, 1) == nRec);
assert(isequal(full(nRec - 1:nRec, c.seq)', [nRec - 1, nRec]));
fprintf('ok  alignment: record k+2 summary on row k, chunk-independent, final\n');

% A seq gap: the two rows before it have no window
cut = raw([1:100, 103:nRec], :);
[~, o] = traceAlign(traceAlignNew(), cut, true);
assert(size(o, 1) == nRec - 2);
none = find(o(:, c.summaryScans) == 0);
rowsOf = o(none, c.seq)';
assert(isequal(rowsOf, [99 100 199 200]));
assert(o(98, c.summarySeq) == 98 && o(101, c.seq) == 103 && o(101, c.summarySeq) == 103);
fprintf('ok  alignment: a seq gap leaves the two rows before it without window\n');

% A window read twice (a late exchange merged with the next) counts once
dup = raw;
dup(60, sumCols) = raw(59, sumCols);
[~, o] = traceAlign(traceAlignNew(), dup, true);
assert(o(57, c.summarySeq) == 57 && o(58, c.summaryScans) == 0);
assert(o(59, c.summarySeq) == 59);
assert(nnz(o(1:nRec - 2, c.summaryScans) == 0) == 1);
fprintf('ok  alignment: a duplicated window gives its second row no window\n');

% ----- Decimation -----
A = full;                               % 200 aligned rows, the last two with no window
n = 20;
[d, dec] = traceDecimate(traceDecimateNew(n), A, false);
assert(size(dec, 1) == 10 && d.count == 0);
for i = 1:10
    r0 = (i - 1) * n + 1;
    r1 = i * n;
    ids = r0:min(r1, nRec - 2);
    p = [];
    v = [];
    t = [];
    x = [];
    for w = ids
        p = [p, win(w).p]; %#ok<AGROW>
        v = [v, win(w).v]; %#ok<AGROW>
        t = [t, win(w).t]; %#ok<AGROW>
        x = [x, win(w).x]; %#ok<AGROW>
    end
    e = dec(i, :);
    assert(relErr(e(c.pressureMeanGauge_MPa), mean(p)) < 1e-12);
    assert(e(c.pressureMinGauge_MPa) == min(p) && e(c.pressureMaxGauge_MPa) == max(p));
    assert(relErr(e(c.velocityMean_mm_s), mean(v)) < 1e-12);
    assert(relErr(e(c.torqueMean_pct), mean(t)) < 1e-12);
    assert(relErr(e(c.positionMean_mm), mean(x)) < 1e-12);
    assert(e(c.summaryScans) == numel(p) && e(c.summarySeq) == ids(end));
    assert(e(c.nExchanges) == n);
    keepCols = setdiff(1:L.nCol, [sumCols, c.nExchanges]);
    assert(isequal(e(keepCols), A(r1, keepCols)));
    assert(e(c.seq) == A(r1, c.seq) && e(c.t_s) == A(r1, c.t_s));
end
% In chunks the result is the same
d2 = traceDecimateNew(n);
dec2 = zeros(0, L.nCol);
at = 0;
for i = 1:numel(chunks)
    [d2, o] = traceDecimate(d2, A(at + 1:at + chunks(i), :), false);
    dec2 = [dec2; o]; %#ok<AGROW>
    at = at + chunks(i);
end
assert(isequal(dec2, dec));
fprintf('ok  decimation n = 20: scan-weighted means, min, max, last snapshot\n');

% A procStep change in row 111 splits the interval and starts a new grid
B = A;
B(111:end, c.procStep) = 1;
[d, o] = traceDecimate(traceDecimateNew(n), B, false);
assert(isequal(o(:, c.nExchanges)', [20 20 20 20 20 10 20 20 20 20]));
assert(d.count == 10);
[d, o2] = traceDecimate(d, zeros(0, L.nCol), true);
assert(size(o2, 1) == 1 && o2(c.nExchanges) == 10 && o2(c.seq) == 200 && d.count == 0);
assert(o(6, c.seq) == 110 && o(6, c.procStep) == 0 && o(7, c.procStep) == 1);
assert(all(o(1:6, c.procStep) == 0) && all(o(7:end, c.procStep) == 1));
fprintf('ok  decimation: a procStep change ends the interval, flush emits the rest\n');

% Missing records: inside an interval and across a boundary
names = traceColumnNames();
G = A([1:104, 110:200], :);                         % rows 105-109 missing
[~, o] = traceDecimate(traceDecimateNew(n), G, true);
assert(isequal(o(:, c.nExchanges)', [20 20 20 20 20 15 20 20 20 20]));
dd = struct('meta', struct());
for k = 1:L.nCol
    dd.(names{k}) = o(:, k);
end
dd = traceDerive(dd);
assert(isequal(find(dd.gap)', 6));
G = A([1:114, 126:200], :);                         % rows 115-125 missing
[~, o] = traceDecimate(traceDecimateNew(n), G, true);
assert(isequal(o(:, c.nExchanges)', [20 20 20 20 20 14 15 20 20 20]));
assert(isequal(o(6:7, c.seq)', [114 140]));
dd = struct('meta', struct());
for k = 1:L.nCol
    dd.(names{k}) = o(:, k);
end
dd = traceDerive(dd);
assert(isequal(find(dd.gap)', 7));
fprintf('ok  decimation: missing records give partial rows and gaps\n');

% Flush of a partial interval
[d, o] = traceDecimate(traceDecimateNew(n), A(1:50, :), false);
assert(size(o, 1) == 2 && d.count == 10);
[d, o3] = traceDecimate(d, zeros(0, L.nCol), true);
assert(size(o3, 1) == 1 && o3(c.nExchanges) == 10 && o3(c.seq) == 50 && d.count == 0);
[~, o4] = traceDecimate(d, zeros(0, L.nCol), true);
assert(isempty(o4));
fprintf('ok  decimation: flush emits the partial interval once\n');

% n = 1 is the identity but for nExchanges
[~, o] = traceDecimate(traceDecimateNew(1), A, false);
assert(isequal(size(o), size(A)) && all(o(:, c.nExchanges) == 1));
others = setdiff(1:L.nCol, [sumCols, c.nExchanges]);
assert(isequal(o(:, others), A(:, others)));
assert(max(max(abs(o(:, sumCols) - A(:, sumCols)) ./ max(1, abs(A(:, sumCols))))) < 1e-12);
assert(isequal(o(:, c.summaryScans), A(:, c.summaryScans)));
fprintf('ok  decimation n = 1 keeps every record\n');

% A row with no window counts its exchange, not its scans; an interval
% without windows is zero
C = A(1:40, :);
C(5, sumCols) = 0;
C(21:40, sumCols) = 0;
[~, o] = traceDecimate(traceDecimateNew(n), C, false);
assert(size(o, 1) == 2 && all(o(:, c.nExchanges) == 20));
scans = arrayfun(@(w) w.scans, win(1:20));
assert(o(1, c.summaryScans) == sum(scans) - scans(5));
assert(isequal(o(2, sumCols), zeros(1, numel(sumCols))));
fprintf('ok  decimation: rows without a window count the exchange only\n');

% Interval to exchanges
cx = struct('exchangePeriod', 0.05);
assert(isequal(arrayfun(@(v) traceIntervalExchanges(v, cx), [0.01 0.05 0.12 1 60]), ...
    [1 1 2 20 1200]));
assert(traceIntervalExchanges(0.5, cfg) == round(0.5 / cfg.exchangePeriod));
fprintf('ok  record interval in exchanges\n');

% ----- Live buffer -----
b = traceLiveBuffer(cfg);
cap = round(cfg.livePlotSeconds / cfg.exchangePeriod);
assert(cap > 0 && isequal(size(b.data), [cap 8]) && b.count == 0 && b.lastSeq == 0);
assert(b.span == cfg.livePlotSeconds && b.posEOT == cfg.posEOT);
N = cap + 3000;     % more rows than the buffer holds, so it wraps
R = liveRows(N);
at = 0;
k = 0;
while at < N
    k = k + 1;
    m = min(N - at, 1 + mod(round(1e4 * abs(sin(k))), 700));
    b = traceLiveAppend(b, R(at + 1:at + m, :));
    at = at + m;
end
assert(b.count == cap && b.lastSeq == N && b.head == mod(N, cap) + 1);
idx = mod(b.head - 1 - b.count + (0:b.count - 1)', cap) + 1;
M = b.data(idx, :);
last = R(N - cap + 1:N, :);
assert(isequal(M(:, 1), last(:, c.t_s)));
assert(max(abs(M(:, 2) - (last(:, c.pressureGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient))) < 1e-12);
assert(max(abs(M(:, 6) - (cfg.posEOT - last(:, c.position_mm)))) < 1e-12);
assert(isequal(M(:, 7), last(:, c.velocity_mm_s)) && isequal(M(:, 8), last(:, c.torque_pct)));
noWin = last(:, c.summaryScans) == 0;
assert(nnz(noWin) > 100);
assert(all(isnan(M(noWin, 3))) && all(isnan(M(noWin, 4))) && all(isnan(M(noWin, 5))));
assert(max(abs(M(~noWin, 3) - (last(~noWin, c.pressureMeanGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient))) < 1e-12);
assert(max(abs(M(~noWin, 4) - (last(~noWin, c.pressureMinGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient))) < 1e-12);
assert(max(abs(M(~noWin, 5) - (last(~noWin, c.pressureMaxGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient))) < 1e-12);
fprintf('ok  live buffer keeps the newest %d of %d rows in order\n', cap, N);

% A seq jump inserts one NaN row with the next row's time, between and within calls
b = traceLiveBuffer(cfg);
R = liveRows(30);
b = traceLiveAppend(b, R(1:5, :));
assert(b.count == 5);
b = traceLiveAppend(b, R(9:12, :));
assert(b.count == 10 && b.lastSeq == 12);
assert(all(isnan(b.data(6, [2:8]))) && b.data(6, 1) == R(9, c.t_s));
assert(isequal(b.data(7:10, 1), R(9:12, c.t_s)));
b = traceLiveAppend(b, R([13:15, 20:22], :));
assert(b.count == 17 && all(isnan(b.data(14, 2:8))) && b.data(14, 1) == R(20, c.t_s));
assert(~any(isnan(b.data([1:5, 7:13, 15:17], 2))));
fprintf('ok  live buffer breaks the lines at a seq jump\n');

% ----- Live figure -----
if graphicsUsable()
    b = traceLiveBuffer(cfg);
    R = liveRows(100);
    b = traceLiveAppend(b, R);
    h = traceLiveNew('off');
    assert(strcmp(get(h.fig, 'Visible'), 'off') && numel(h.ax) == 4);
    assert(traceLiveUpdate(h, traceLiveBuffer(cfg)));
    assert(traceLiveUpdate(h, b));
    tNew = R(end, c.t_s);
    x = get(h.lines.p, 'XData');
    assert(max(abs(x(:) - (R(:, c.t_s) - tNew))) < 1e-12);
    y = get(h.lines.p, 'YData');
    assert(max(abs(y(:) - (R(:, c.pressureGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient))) < 1e-12);
    y = get(h.lines.pos, 'YData');
    assert(max(abs(y(:) - (cfg.posEOT - R(:, c.position_mm)))) < 1e-12);
    y = get(h.lines.torque, 'YData');
    assert(max(abs(y(:) - R(:, c.torque_pct))) < 1e-12);
    y = get(h.lines.pMean, 'YData');
    yw = R(:, c.pressureMeanGauge_MPa) * cfg.atmPerMPa + cfg.atmAmbient;
    yw(R(:, c.summaryScans) == 0) = NaN;
    assert(isequaln(y(:), yw));
    for k = 1:4
        assert(isequal(get(h.ax(k), 'XLim'), [-cfg.livePlotSeconds 0]));
    end
    close(h.fig);
    assert(~traceLiveUpdate(h, b));
    fprintf('ok  live figure draws the buffer and reports it closed\n');
else
    fprintf('ok  live figure skipped (no graphics toolkit)\n');
end
end

% %% ===== LOCAL FUNCTIONS =====

function e = relErr(a, b)
e = abs(a - b) / max(1, abs(b));
end

function sizes = chunkSizes(total)
%CHUNKSIZES Irregular chunk sizes (1 to 9) that sum to total.
sizes = [];
k = 0;
while sum(sizes) < total
    k = k + 1;
    sizes(end + 1) = min(total - sum(sizes), 1 + mod(round(1e4 * abs(sin(k))), 9)); %#ok<AGROW>
end
end

function ok = graphicsUsable()
%GRAPHICSUSABLE True when an invisible figure can be created and drawn in.
ok = false;
try
    if exist('OCTAVE_VERSION', 'builtin') ~= 0 && isempty(available_graphics_toolkits())
        return
    end
    f = figure('Visible', 'off');
    cleaner = onCleanup(@() close(f));
    subplot(2, 1, 1);
    plot(1:3, [1 NaN 3]);
    patch([0 1 1 0], [0 0 1 1], [1 0.8 0.8], 'EdgeColor', 'none');
    ok = strcmp(get(f, 'Visible'), 'off');
catch
    ok = false;
end
end

function [raw, win] = synthRaw(nRec)
%SYNTHRAW nRec raw records in the lag of the slave: record r holds the
%   summary of window r-2 (none for r = 1, 2). Window w has 10 + mod(w,3)
%   scans whose pressure, velocity, torque and position samples are in
%   win(w) (p, v, t, x, scans); the summary registers are their mean,
%   minimum and maximum.
L = traceLayout();
c = L.col;
win = struct('scans', {}, 'p', {}, 'v', {}, 't', {}, 'x', {});
for w = 1:nRec
    s = 10 + mod(w, 3);
    j = 1:s;
    win(w).scans = s;
    win(w).p = 2 + sin(0.37 * w + 0.9 * j) + 0.01 * w;
    win(w).v = 5 * cos(0.11 * w + 0.5 * j);
    win(w).t = 20 * sin(0.07 * w) + 0.3 * j;
    win(w).x = 100 + 0.5 * w + 0.01 * j;
end
raw = zeros(nRec, L.nCol);
for r = 1:nRec
    row = r + (1:L.nCol) / 100;
    row(c.seq) = r;
    row(c.t_s) = 0.05 * (r - 1);
    row(c.procStep) = 0;
    row(c.nExchanges) = 1;
    row(sumCols(c)) = 0;
    if r >= 3
        w = win(r - 2);
        row(c.pressureMeanGauge_MPa) = mean(w.p);
        row(c.pressureMinGauge_MPa) = min(w.p);
        row(c.pressureMaxGauge_MPa) = max(w.p);
        row(c.velocityMean_mm_s) = mean(w.v);
        row(c.torqueMean_pct) = mean(w.t);
        row(c.positionMean_mm) = mean(w.x);
        row(c.summaryScans) = w.scans;
        row(c.summarySeq) = r - 2;
    end
    raw(r, :) = row;
end
end

function cols = sumCols(c)
cols = c.pressureMeanGauge_MPa:c.summarySeq;
end

function R = liveRows(n)
%LIVEROWS n aligned records with a smooth signal, every 7th without a window.
L = traceLayout();
c = L.col;
k = (1:n)';
R = zeros(n, L.nCol);
R(:, c.seq) = k;
R(:, c.t_s) = 0.05 * (k - 1);
R(:, c.pressureGauge_MPa) = 2 + sin(0.01 * k);
R(:, c.position_mm) = 100 + 0.01 * k;
R(:, c.velocity_mm_s) = cos(0.02 * k);
R(:, c.torque_pct) = 10 * sin(0.03 * k);
R(:, c.pressureMeanGauge_MPa) = 2 + sin(0.01 * k) + 0.01;
R(:, c.pressureMinGauge_MPa) = 2 + sin(0.01 * k) - 0.1;
R(:, c.pressureMaxGauge_MPa) = 2 + sin(0.01 * k) + 0.2;
R(:, c.summaryScans) = 12;
R(:, c.summarySeq) = mod(k, 65536);
R(mod(k, 7) == 0, c.pressureMeanGauge_MPa:c.summarySeq) = 0;
R(:, c.nExchanges) = 1;
end
