function traceCsvPlot()
%TRACECSVPLOT Trace CSV writer and reader, derived quantities and plots.
%
%   The end-to-end scenario records a stand-in slave through masterStep
%   and traceStep, drains it at irregular polls and round-trips the
%   result through a CSV file. The other scenarios use a synthetic record
%   matrix with a gap, a fault, a link loss and an E-STOP. The stand-in
%   stops short of the slave simulator, so the test is fast.
%
%   Run from the repository root:
%     addpath master-src/logic master-src/trace tests; traceCsvPlot
%   Depends on: master-src/logic master-src/trace

L = traceLayout();
cfg = masterConfig();
names = traceColumnNames();
tmp = tempname();
cleanup = onCleanup(@() deleteIfExists([tmp '.csv'], [tmp '_gap.csv'], [tmp '_empty.csv']));

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
assert(isnumeric(d.meta.format) && d.meta.format == 1);
assert(ischar(d.meta.started) && strcmp(d.meta.started, '2026-01-02T03:04:05'));
assert(strcmp(d.meta.source, 'SOS_Gen3_Compressor_Interface'));
assert(d.meta.records == buf.n && d.meta.missed == buf.missed);
assert(d.meta.dt_s == cfg.dt && d.meta.exchangePeriod_s == cfg.exchangePeriod);
assert(abs(d.meta.boreDiameter_mm - cfg.boreDiameter) <= 1e-9 * cfg.boreDiameter);
assert(ischar(d.meta.referenceUnits));
assert(strcmp(strjoin(fieldnames(d.meta)', ','), strjoin(fieldnames(meta)', ',')));
assert(strcmp(d.file, file));
delete(file);
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
assert(isempty(e.gap) && isempty(e.tRel_s) && isempty(e.volume_mL));
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
assert(max(abs(d.volume_mL - x2mL(pos, cfg))) < 1e-6);
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
meta300.deadVolume_mL = 40;
meta300.atmAmbient_atm = 1.5;
traceWriteCsv(gapFile, data, meta300);
d3 = traceDerive(traceReadCsv(gapFile));
assert(max(abs(d3.posFromEOT_mm - (300 - pos))) < 1e-9);
c3 = cfg;
c3.posEOT = 300;
c3.deadVolume = 40;
assert(max(abs(d3.volume_mL - x2mL(pos, c3))) < 1e-6);
assert(max(abs(d3.pressureAbs_atm - (data(:, L.col.pressureGauge_MPa) * cfg.atmPerMPa + 1.5))) < 1e-6);
% A key missing from the file falls back to masterConfig.
d4 = traceDerive(rmfield(traceReadCsv(gapFile), 'meta'));
assert(max(abs(d4.posFromEOT_mm - (cfg.posEOT - pos))) < 1e-9);
fprintf('ok  derived quantities, file geometry and gap flags\n');

% Plot.
if graphicsUsable()
    traceWriteCsv(gapFile, data, meta);
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
    end
    [mbWrite, doSafety, ~, ~, ~, ~, statusCode, faultCode, state, linkOk] = ...
        masterStep(mbRead, 0, 0, 250, 0, 0, 20, 0, 0, cfg.positionSpeed);
    ring = traceStep(mbRead, mbWrite, doSafety, statusCode, faultCode, state, ...
        linkOk, 0, 0, 250, 20, 0, 0);
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
