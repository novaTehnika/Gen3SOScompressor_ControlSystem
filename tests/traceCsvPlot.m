function traceCsvPlot()
%TRACECSVPLOT Trace CSV writer and reader, derived quantities and plots.
%
%   The end-to-end scenarios record a stand-in slave through masterStep
%   and traceStep, drain it at irregular polls and round-trip the result
%   through a CSV file, once as drained (a record per exchange) and once
%   aligned and decimated to 1 s. The other scenarios use a synthetic
%   record matrix with a gap, a fault, a link loss and an E-STOP, as the
%   ring holds it (format 2 reads the summary two records late) and
%   aligned (format 3), and fixtures of formats 1 and 2. The stand-in
%   stops short of the slave simulator, so the test is fast.
%
%   Run from the repository root:
%     addpath master-src/logic master-src/trace tests; traceCsvPlot
%   Depends on: master-src/logic master-src/trace

L = traceLayout();
cfg = masterConfig();
names = traceColumnNames();
tmp = tempname();
cleanup = onCleanup(@() deleteIfExists([tmp '.csv'], [tmp '_gap.csv'], [tmp '_empty.csv'], [tmp '_f1.csv'], [tmp '_f2.csv'], [tmp '_iv.csv'], [tmp '_a3.csv']));

% End to end: stand-in slave, irregular drain, CSV, read back.
clear masterStep masterStateMachine traceStep
[buf, ref] = recordStandIn(1200, 100);
assert(buf.n > 400 && buf.missed == 0);
assert(isequal(buf.data(1:buf.n, :), ref(1:buf.n, :)));
meta = traceMeta(buf, cfg, '2026-01-02T03:04:05');
file = [tmp '.csv'];
traceWriteCsv(file, buf.data(1:buf.n, :), meta);
d = traceReadCsv(file);
assert(isequal(d.names, names));
assert(isequal(size(d.data), [buf.n L.nCol]));
for k = 1:L.nCol
    x = buf.data(1:buf.n, k);
    assert(all(abs(d.(names{k}) - x) <= 1e-9 * max(1, abs(x))));
end
assert(isnumeric(d.meta.format) && d.meta.format == 3);
assert(d.meta.recordInterval_s == cfg.exchangePeriod);
assert(find(strcmp(fieldnames(d.meta), 'recordInterval_s')) == ...
    find(strcmp(fieldnames(d.meta), 'exchangePeriod_s')) + 1);
mI = traceMeta(buf, cfg, '2026-01-02T03:04:05', 1);
assert(mI.recordInterval_s == 1);
assert(ischar(d.meta.started) && strcmp(d.meta.started, '2026-01-02T03:04:05'));
assert(strcmp(d.meta.source, 'SOS_Gen3_Compressor_Interface'));
assert(d.meta.records == buf.n && d.meta.missed == buf.missed);
assert(d.meta.dt_s == cfg.dt && d.meta.exchangePeriod_s == cfg.exchangePeriod);
assert(abs(d.meta.boreDiameter_mm - cfg.boreDiameter) <= 1e-9 * cfg.boreDiameter);
assert(ischar(d.meta.referenceUnits));
assert(strcmp(strjoin(fieldnames(d.meta)', ','), strjoin(fieldnames(meta)', ',')));
assert(strcmp(d.file, file));
delete(file);
assert(all(d.procStep == 0) && all(d.nExchanges == 1));
fprintf('ok  recorded trace round-trips through the CSV\n');

% A header-only file reads back as empty columns.
emptyFile = [tmp '_empty.csv'];
traceWriteCsv(emptyFile, zeros(0, L.nCol), meta);
e = traceReadCsv(emptyFile);
assert(isequal(e.names, names));
assert(isequal(size(e.seq), [0 1]) && isequal(size(e.mixer), [0 1]));
assert(isequal(size(e.data), [0 L.nCol]));
assert(e.meta.records == buf.n);
e = traceDerive(e);
assert(isempty(e.gap) && isempty(e.tRel_s) && isempty(e.sweptVolume_mL));
fprintf('ok  header-only file reads back empty\n');

% The writer refuses data it cannot store faithfully.
good = synthData();
for bad = {setAt(good, 5, 7, NaN), setAt(good, 5, 7, Inf), setAt(good, 5, 7, -Inf), ...
        good(:, 1:L.nCol - 1), [good ones(size(good, 1), 1)]}
    raised = false;
    try
        traceWriteCsv([tmp '_bad.csv'], bad{1}, meta);
    catch
        raised = true;
    end
    assert(raised);
    assert(~exist([tmp '_bad.csv'], 'file'));
end
raised = false;
try
    traceWriteCsv(fullfile(tmp, 'no_such_dir', 'x.csv'), good, meta);
catch
    raised = true;
end
assert(raised);
raised = false;
try
    fid = fopen([tmp '_cols.csv'], 'w');
    fprintf(fid, 'a,b,c\n1,2\n');
    fclose(fid);
    traceReadCsv([tmp '_cols.csv']);
catch
    raised = true;
end
delete([tmp '_cols.csv']);
assert(raised);
fprintf('ok  writer and reader reject bad input\n');

% Derived quantities.
gapFile = [tmp '_gap.csv'];
[data, gapAt] = synthData();
traceWriteCsv(gapFile, data, meta);
d = traceDerive(traceReadCsv(gapFile));
n = size(data, 1);
pos = data(:, L.col.position_mm);
assert(max(abs(d.sweptVolume_mL - sweptVolume(pos, cfg))) < 1e-6);
assert(max(abs(d.mixingVolume_mL - mixingVolume(pos, cfg))) < 1e-6);
assert(max(abs(d.flow_mL_s - mLPerMm(cfg) * data(:, L.col.velocity_mm_s))) < 1e-6);
assert(max(abs(d.pressureAbs_atm - (data(:, L.col.pressureGauge_MPa) * cfg.atmPerMPa ...
    + cfg.atmAmbient))) < 1e-6);
assert(max(abs(d.posFromEOT_mm - (cfg.posEOT - pos))) < 1e-9);
assert(max(abs(d.commandedFromEOT_mm - (cfg.posEOT - data(:, L.col.commandedPosition_mm)))) < 1e-9);
assert(max(abs(d.targetFromEOT_mm - (cfg.posEOT - data(:, L.col.targetPosition_mm)))) < 1e-9);
assert(d.tRel_s(1) == 0 && abs(d.tRel_s(end) - (d.t_s(end) - d.t_s(1))) < 1e-12);
expectGap = false(n, 1);
expectGap(gapAt) = true;
assert(isequal(d.gap, expectGap));
% The file's geometry wins over masterConfig.
meta300 = meta;
meta300.posEOT_mm = 300;
meta300.mixingDeadVolume_mL = 40;
meta300.atmAmbient_atm = 1.5;
traceWriteCsv(gapFile, data, meta300);
d3 = traceDerive(traceReadCsv(gapFile));
assert(max(abs(d3.posFromEOT_mm - (300 - pos))) < 1e-9);
c3 = cfg;
c3.posEOT = 300;
c3.mixingDeadVolume = 40;
assert(max(abs(d3.sweptVolume_mL - sweptVolume(pos, c3))) < 1e-6);
assert(max(abs(d3.mixingVolume_mL - mixingVolume(pos, c3))) < 1e-6);
assert(max(abs(d3.pressureAbs_atm - (data(:, L.col.pressureGauge_MPa) * cfg.atmPerMPa + 1.5))) < 1e-6);
% A key missing from the file falls back to masterConfig.
d4 = traceDerive(rmfield(traceReadCsv(gapFile), 'meta'));
assert(max(abs(d4.posFromEOT_mm - (cfg.posEOT - pos))) < 1e-9);
fprintf('ok  derived quantities, file geometry and gap flags\n');

% The window summary lines up with the row whose write opened the window:
% row k takes the summary of record k+2. It is missing where record k+2 is
% missing (the last two rows), across the gap (the two rows before it) and
% where the slave had published no window yet (summaryScans == 0).
meta2 = meta;
meta2.format = 2;
traceWriteCsv(gapFile, data, meta2);
d = traceDerive(traceReadCsv(gapFile));
conv = @(x) x * cfg.atmPerMPa + cfg.atmAmbient;     % [MPa gauge -> atm absolute]
expectNaN = false(n, 1);
expectNaN([n - 1, n]) = true;
expectNaN([gapAt - 2, gapAt - 1]) = true;
expectNaN(28:31) = true;
assert(isequal(isnan(d.pressureMeanAbs_atm), expectNaN));
assert(isequal(isnan(d.pressureMinAbs_atm), expectNaN));
assert(isequal(isnan(d.pressureMaxAbs_atm), expectNaN));
ok = find(~expectNaN);
assert(max(abs(d.pressureMeanAbs_atm(ok) - conv(data(ok + 2, L.col.pressureMeanGauge_MPa)))) < 1e-6);
assert(max(abs(d.pressureMinAbs_atm(ok) - conv(data(ok + 2, L.col.pressureMinGauge_MPa)))) < 1e-6);
assert(max(abs(d.pressureMaxAbs_atm(ok) - conv(data(ok + 2, L.col.pressureMaxGauge_MPa)))) < 1e-6);
% In the synthetic data row k's window mean is the pressure of row k itself
assert(max(abs(d.pressureMeanAbs_atm(ok) - d.pressureAbs_atm(ok))) < 1e-6);
assert(all(d.pressureMinAbs_atm(ok) < d.pressureMeanAbs_atm(ok)));
assert(all(d.pressureMaxAbs_atm(ok) > d.pressureMeanAbs_atm(ok)));
% The file's atmAmbient is used for the summary as for the snapshot
meta2_300 = meta300;
meta2_300.format = 2;
traceWriteCsv(gapFile, data, meta2_300);
d3 = traceDerive(traceReadCsv(gapFile));
assert(max(abs(d3.pressureMeanAbs_atm(ok) - (data(ok + 2, L.col.pressureMeanGauge_MPa) ...
    * cfg.atmPerMPa + 1.5))) < 1e-6);
assert(all(isnan(d.posMeanFromEOT_mm)));
fprintf('ok  format 2: window summary taken from record k+2, %d of %d rows\n', numel(ok), n);

% Format 3: the summary is on the row of its window (traceAlign), so the
% derived means are those of the same row, NaN where summaryScans is 0
[~, aligned] = traceAlign(traceAlignNew(), data, true);
assert(isequal(size(aligned), size(data)));
a3File = [tmp '_a3.csv'];
traceWriteCsv(a3File, aligned, meta);
d = traceDerive(traceReadCsv(a3File));
assert(~d.decimated && all(d.interval_s == cfg.exchangePeriod));
assert(isequal(isnan(d.pressureMeanAbs_atm), expectNaN));
assert(isequal(isnan(d.posMeanFromEOT_mm), expectNaN));
assert(max(abs(d.pressureMeanAbs_atm(ok) - conv(aligned(ok, L.col.pressureMeanGauge_MPa)))) < 1e-7);
assert(max(abs(d.pressureMinAbs_atm(ok) - conv(aligned(ok, L.col.pressureMinGauge_MPa)))) < 1e-7);
assert(max(abs(d.pressureMaxAbs_atm(ok) - conv(aligned(ok, L.col.pressureMaxGauge_MPa)))) < 1e-7);
assert(max(abs(d.pressureMeanAbs_atm(ok) - d.pressureAbs_atm(ok))) < 1e-6);
assert(max(abs(d.posMeanFromEOT_mm(ok) - (cfg.posEOT - aligned(ok, L.col.positionMean_mm)))) < 1e-9);
assert(isequal(d.gap, expectGap) && max(abs(d.tStartRel_s - d.tRel_s)) == 0);
assert(isequal(d.procStep, zeros(n, 1)));
fprintf('ok  format 3 at 0.05 s: summary columns are the row''s own\n');

% A format 1 file (columns 1-34) still reads: the summary columns derive
% as NaN, and the plot draws without them.
f1File = [tmp '_f1.csv'];
traceWriteCsv(gapFile, data, meta2);
writeFormat1(gapFile, f1File, 34, 1);
d1 = traceReadCsv(f1File);
assert(d1.meta.format == 1 && numel(d1.names) == 34 && size(d1.data, 2) == 34);
assert(isequal(d1.names, names(1:34)) && ~isfield(d1, 'summarySeq'));
d1 = traceDerive(d1);
assert(numel(d1.pressureMeanAbs_atm) == n && all(isnan(d1.pressureMeanAbs_atm)));
assert(all(isnan(d1.pressureMinAbs_atm)) && all(isnan(d1.pressureMaxAbs_atm)));
assert(max(abs(d1.pressureAbs_atm - d.pressureAbs_atm)) < 1e-9);
if graphicsUsable()
    figs = tracePlot(f1File, 'visible', 'off');
    assert(numel(figs) == 3 && all(ishghandle(figs)));
    close(figs);
end
assert(all(d1.nExchanges == 1) && all(d1.procStep == 0) && ~d1.decimated);
assert(all(isnan(d1.posMeanFromEOT_mm)));

% A format 2 file (columns 1-42) still reads and derives by the k+2 rule
f2File = [tmp '_f2.csv'];
writeFormat1(gapFile, f2File, 42, 2);
d2 = traceReadCsv(f2File);
assert(d2.meta.format == 2 && numel(d2.names) == 42 && ~isfield(d2, 'nExchanges'));
d2 = traceDerive(d2);
assert(all(d2.nExchanges == 1) && all(d2.procStep == 0) && ~d2.decimated);
assert(isequal(isnan(d2.pressureMeanAbs_atm), expectNaN));
assert(isequal(d2.gap, expectGap) && isequal(d2.tStartRel_s, d2.tRel_s));
if graphicsUsable()
    figs = tracePlot(f2File, 'visible', 'off');
    assert(numel(figs) == 3 && all(ishghandle(figs)));
    close(figs);
end
fprintf('ok  format 2 and format 1 files read, derive (nExchanges 1, procStep 0) and plot\n');

% End to end at 1 s: the drained records are aligned, decimated to 20
% exchanges with 10 records cut out, written, read and derived.
clear masterStep masterStateMachine traceStep
[buf, ~] = recordStandIn(1200, 100);
raw = buf.data(1:buf.n, :);
cutAt = 130:139;
cutSeq = raw(cutAt, L.col.seq);
raw(cutAt, :) = [];
[~, al] = traceAlign(traceAlignNew(), raw, true);
assert(size(al, 1) == size(raw, 1));
[dec, rowsIv] = traceDecimate(traceDecimateNew(traceIntervalExchanges(1, cfg)), al, true);
assert(dec.count == 0);
assert(sum(rowsIv(:, L.col.nExchanges)) == size(al, 1));
assert(all(rowsIv(:, L.col.nExchanges) <= 20) && any(rowsIv(:, L.col.nExchanges) < 20));
bufIv = struct('n', size(rowsIv, 1), 'missed', buf.missed);
ivFile = [tmp '_iv.csv'];
traceWriteCsv(ivFile, rowsIv, traceMeta(bufIv, cfg, '2026-01-02T03:04:05', 1));
di = traceReadCsv(ivFile);
assert(di.meta.format == 3 && di.meta.recordInterval_s == 1);
di = traceDerive(di);
assert(di.decimated && all(isfinite(di.pressureMeanAbs_atm(di.summaryScans > 0))));
assert(all(di.summaryScans(1:end - 1) > 0) && di.summaryScans(end) > 0);
assert(all(isfinite(di.posMeanFromEOT_mm(di.summaryScans > 0))));
assert(nnz(di.gap) == 1);
g = find(di.gap);
assert(di.seq(g) > max(cutSeq) && di.seq(g - 1) < min(cutSeq));
assert(max(abs(di.interval_s - di.nExchanges * cfg.exchangePeriod)) < 1e-12);
assert(max(abs(di.tStartRel_s - (di.tRel_s - (di.nExchanges - 1) * cfg.exchangePeriod))) < 1e-12);
assert(abs(di.tStartRel_s(1) + 19 * cfg.exchangePeriod) < 1e-9);
assert(all(di.pressureMinAbs_atm <= di.pressureMeanAbs_atm + 1e-9) ...
    && all(di.pressureMaxAbs_atm >= di.pressureMeanAbs_atm - 1e-9));
fprintf('ok  interval file: aligned, decimated to 1 s, written, read and derived (%d rows, one gap)\n', numel(di.seq));

% Plot.
if graphicsUsable()
    figs = tracePlot(ivFile, 'visible', 'off');
    assert(numel(figs) == 3 && all(ishghandle(figs)));
    assert(~isempty(get(figs(1), 'Children')));
    close(figs);
    traceWriteCsv(gapFile, aligned, meta);
    figs = tracePlot(gapFile, 'visible', 'off');
    assert(numel(figs) == 3);
    for k = 1:3
        assert(ishghandle(figs(k)));
        assert(strcmp(get(figs(k), 'Visible'), 'off'));
    end
    assert(~isempty(get(figs(1), 'Children')));
    close(figs);
    % A struct source, and saving as .fig next to the CSV.
    saveDir = tempname();
    mkdir(saveDir);
    figs = tracePlot(traceReadCsv(gapFile), 'visible', 'off', 'save', true, ...
        'saveDir', saveDir);
    assert(numel(figs) == 3 && all(ishghandle(figs)));
    [~, base] = fileparts(gapFile);
    for tag = {'overview', 'delivery', 'status'}
        assert(exist(fullfile(saveDir, [base '_' tag{1} '.fig']), 'file') == 2);
    end
    close(figs);
    delete(fullfile(saveDir, '*.fig'));
    rmdir(saveDir);
    % A trace with no records is refused.
    raised = false;
    try
        tracePlot(e, 'visible', 'off');
    catch
        raised = true;
    end
    assert(raised);
    fprintf('ok  plot overview, delivery and status\n');
else
    fprintf('ok  plot skipped (no graphics toolkit)\n');
end
end

% %% ===== LOCAL FUNCTIONS =====

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

function deleteIfExists(varargin)
for k = 1:numel(varargin)
    if exist(varargin{k}, 'file')
        delete(varargin{k});
    end
end
end

function writeFormat1(src, dst, nKeep, format)
%WRITEFORMAT1 Copy of a trace CSV with only its first nKeep columns and
%   the given format in its header, as an earlier recorder wrote it.
txt = fileread(src);
lines = strsplit(txt, "\n");
for k = 1:numel(lines)
    ln = lines{k};
    if isempty(strtrim(ln))
        continue
    end
    if ln(1) == '#'
        tok = regexp(ln, '^(#\s*format:\s*)\d+', 'tokens', 'once');
        if ~isempty(tok)
            lines{k} = [tok{1} sprintf('%d', format)];
        end
    else
        f = strsplit(strtrim(ln), ',');
        lines{k} = strjoin(f(1:nKeep), ',');
    end
end
fid = fopen(dst, 'w');
fprintf(fid, '%s', strjoin(lines, "\n"));
fclose(fid);
end

function m = setAt(m, i, j, v)
m(i, j) = v;
end

function [data, gapAt] = synthData()
%SYNTHDATA N x nCol record matrix with a seq jump (a gap), a fault, a
%   link loss and an E-STOP, in the units of the link.
L = traceLayout();
n = 400;
k = (1:n)';
gapAt = 201;
seq = k;
seq(gapAt:end) = seq(gapAt:end) + 7;
t = 0.05 * (seq - 1);
data = zeros(n, L.nCol);
c = L.col;
data(:, c.seq) = seq;
data(:, c.t_s) = t;
data(:, c.heartbeat) = mod(seq, 65536);
data(:, c.heartbeatEcho) = mod(seq - 2, 65536);
data(:, c.slaveCounter) = mod(25 * seq, 65536);
data(:, c.performanceStatus) = 1;
data(:, c.homingComplete) = 1;
data(:, c.position_mm) = 100 + 0.5 * t;
data(:, c.velocity_mm_s) = 0.5;
data(:, c.torque_pct) = 10 * sin(t);
data(:, c.commandedPosition_mm) = 100 + 0.5 * t;
data(:, c.pressureGauge_MPa) = 2 + sin(t);
% The summary of row i describes the window opened by the write of row
% i-2, so it is the pressure of that row's time (seq - 2)
pw = 2 + sin(0.05 * (seq - 3));
data(:, c.pressureMeanGauge_MPa) = pw;
data(:, c.pressureMinGauge_MPa) = pw - 0.1;
data(:, c.pressureMaxGauge_MPa) = pw + 0.1;
data(:, c.velocityMean_mm_s) = 0.5;
data(:, c.torqueMean_pct) = 10 * sin(0.05 * (seq - 3));
data(:, c.positionMean_mm) = 100 + 0.5 * 0.05 * (seq - 3);
data(:, c.summaryScans) = 12;
data(:, c.summarySeq) = mod(seq, 65536);
data(:, c.nExchanges) = 1;
data(30:33, c.summaryScans) = 0;        % no window yet: rows 28-31 have no summary
data(:, c.modeCode) = 2;
data(:, c.motionEnable) = 1;
data(:, c.confCode) = 2;
data(:, c.inMotion) = 1;
data(:, c.statusCode) = 2;
data(:, c.state) = 3;
data(:, c.linkOk) = 1;
data(:, c.doSafety) = 1;
data(:, c.op) = 2;
data(:, c.targetPosition_mm) = 250;
data(:, c.targetPressure_atm) = 20;
data(51:90, c.statusCode) = 5;
data(51:90, c.modeCode) = 7;
data(51:90, c.confCode) = 7;
data(100:130, [c.statusCode c.faultActive]) = repmat([10 1], 31, 1);
data(100:130, c.slaveFaultCode) = 4101;
data(100:130, c.state) = 8;
data(250:280, c.linkOk) = 0;
data(250:280, c.statusCode) = 34;
data(250:280, c.doSafety) = 0;
data(250:280, c.slaveCounter) = data(250, c.slaveCounter);
data(320:340, c.statusCode) = 20;
data(320:340, c.estop) = 1;
data(320:340, c.doSafety) = 0;
end

function [buf, ref] = recordStandIn(nSteps, startStep)
%RECORDSTANDIN Run masterStep and traceStep against a synthetic slave for
%   nSteps model steps and drain at irregular polls from startStep on.
%   The slave's registers change at even steps, as the RT Read output
%   does; its counter advances every second step and its echo returns the
%   heartbeat last written. ref holds the records as the slave stand-in
%   and masterStep produced them, taken from the ring on the step each
%   was written.
L = traceLayout();
cfg = masterConfig();
map = modbusMap();
buf = [];
ref = zeros(nSteps, L.nCol);
nRef = 0;
mbRead = zeros(1, map.nRead);
mbWrite = zeros(1, map.nWrite);
counter = 100;
echo = 0;
pollAt = startStep;
k = 0;
lastRing = [];
for step = 0:nSteps - 1
    t = step * cfg.dt;
    if mod(step, 2) == 0
        counter = mod(counter + 1, 65536);
        echo = mbWrite(map.hr.heartbeat);
        pos = 100 + 0.5 * t;
        mbRead = zeros(1, map.nRead);
        mbRead(map.ir.heartbeatEcho) = echo;
        mbRead(map.ir.slaveCounter) = counter;
        mbRead(map.ir.homingComplete) = 1;
        mbRead(map.ir.position:map.ir.position + 1) = modbusPack(pos);
        mbRead(map.ir.velocity:map.ir.velocity + 1) = modbusPack(0.5);
        mbRead(map.ir.torque:map.ir.torque + 1) = modbusPack(10 * sin(t));
        mbRead(map.ir.commandedPosition:map.ir.commandedPosition + 1) = modbusPack(pos);
        mbRead(map.ir.pressure:map.ir.pressure + 1) = modbusPack(2 + sin(t));
        % Summary registers IR21-34: mean, min, max, velocity, torque and
        % position means, scans in the window, window number
        mbRead(map.ir.pressureMean:map.ir.pressureMean + 1) = modbusPack(2 + sin(t));
        mbRead(map.ir.pressureMin:map.ir.pressureMin + 1) = modbusPack(1.9 + sin(t));
        mbRead(map.ir.pressureMax:map.ir.pressureMax + 1) = modbusPack(2.1 + sin(t));
        mbRead(map.ir.velocityMean:map.ir.velocityMean + 1) = modbusPack(0.5);
        mbRead(map.ir.torqueMean:map.ir.torqueMean + 1) = modbusPack(10 * sin(t));
        mbRead(map.ir.positionMean:map.ir.positionMean + 1) = modbusPack(pos);
        mbRead(map.ir.summaryScans) = 12;
        mbRead(map.ir.summarySeq) = mod(counter, 65536);
    end
    [mbWrite, doSafety, ~, ~, ~, ~, statusCode, faultCode, state, linkOk] = ...
        masterStep(mbRead, 0, 0, 250, 0, 0, 20, 0, 0, speedFromPercent(cfg.speedDefault, cfg));
    ring = traceStep(mbRead, mbWrite, doSafety, statusCode, faultCode, state, ...
        linkOk, 0, 0, 250, 20, 0, 0, 0);
    if step == startStep
        buf = traceStart(ring);
    end
    if step >= startStep
        % Reference: the newest record, copied on the step it was written.
        newest = max(ring(:, L.col.seq));
        if newest > nRef
            row = ring(mod(newest - 1, L.depth) + 1, :);
            nRef = newest;
            ref(nRef, :) = row(1:L.nCol);
        end
        if step >= pollAt
            buf = traceDrain(buf, ring);
            k = k + 1;
            pollAt = pollAt + 1 + mod(round(1e4 * abs(sin(k))), 40);
        end
    end
    lastRing = ring;
end
buf = traceDrain(buf, lastRing);
% Records written before the drain started are not in buf.
first = buf.data(1, L.col.seq);
ref = ref(first:buf.lastSeq, :);
end
