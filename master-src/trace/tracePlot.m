function figs = tracePlot(src, varargin)
%TRACEPLOT Plot a recorded trace in three figures: overview, delivery
%   and status.
%
%   figs = tracePlot(file)
%   figs = tracePlot(d)
%   figs = tracePlot([])                 choose a file (uigetfile, MATLAB)
%   figs = tracePlot(src, 'visible', 'off', 'save', true, 'saveDir', dir)
%
%   src   a trace CSV's file name, or the struct traceReadCsv returns; an
%         empty src opens a file dialog in master-src/captures
%   'visible'  'on' (default) or 'off'
%   'save'     true saves each figure as <csvbase>_<tag>.fig in saveDir
%              (savefig, or hgsave where savefig does not exist)
%   'saveDir'  folder for the saved figures (default: the CSV's folder)
%
%   figs is the row of the three figure handles:
%     overview  pressure (atm absolute) with the target while the master
%               is in pressure mode, position from the end of travel with
%               the command and the target while in position mode,
%               velocity, torque, on one time axis
%     delivery  chamber volume, flow, and pressure against volume
%     status    status code and master state, mode request and
%               confirmation, motion flags, link and safety lines, and
%               the link health: slave counter step per record and
%               heartbeat round trip, in master steps
%   Lines break at a gap (a jump of seq) and a dashed vertical line marks
%   it. Background bands mark the runs of fault (statusCode 10 or the
%   slave's FaultActive; red), link down (linkOk 0; grey) and E-STOP
%   (statusCode 20; yellow).
%
%   Uses subplot, plot, stairs, patch, linkaxes, legend, grid, title,
%   xlabel and ylabel, which MATLAB and Octave share.

opts.visible = 'on';
opts.save = false;
opts.savedir = '';
for k = 1:2:numel(varargin)
    at = find(strcmpi(varargin{k}, fieldnames(opts)));
    if isempty(at)
        error('tracePlot: unknown option %s', varargin{k});
    end
    keys = fieldnames(opts);
    opts.(keys{at}) = varargin{k + 1};
end

if isempty(src)
    here = fileparts(mfilename('fullpath'));
    startDir = fullfile(here, '..', 'captures');
    if ~exist(startDir, 'dir')
        startDir = here;
    end
    [f, p] = uigetfile(fullfile(startDir, '*.csv'), 'Plot a trace');
    if isequal(f, 0)
        figs = [];
        return
    end
    src = fullfile(p, f);
end
if ischar(src)
    d = traceReadCsv(src);
else
    d = src;
end
d = traceDerive(d);
if isempty(d.seq)
    error('tracePlot: the trace holds no records');
end
if isempty(opts.savedir) && isfield(d, 'file') && ~isempty(d.file)
    opts.savedir = fileparts(d.file);
end

t = d.tRel_s;
bands = shadeRuns(d);
cTarget = [0.85 0.33 0.10];
cLine = [0 0.45 0.74];
cLine2 = [0.47 0.67 0.19];
cLine3 = [0.49 0.18 0.56];

% ----- Overview -----
f1 = newFigure('overview', opts.visible);
ax = zeros(1, 4);
ax(1) = subplot(4, 1, 1);
pTarget = maskOut(d.targetPressure_atm, d.statusCode == 5);
h = drawSeries(t, {d.pressureAbs_atm, pTarget}, d.gap, ...
    {cLine, cTarget}, {'-', '-'}, bands, @plot);
legend(h, {'pressure', 'target'}, 'Location', 'northeast');
ylabel('pressure [atm abs]');
title('Overview');
ax(2) = subplot(4, 1, 2);
tgt = maskOut(d.targetFromEOT_mm, d.statusCode == 2 | d.statusCode == 3);
h = drawSeries(t, {d.posFromEOT_mm, d.commandedFromEOT_mm, tgt}, d.gap, ...
    {cLine, cLine2, cTarget}, {'-', '--', '-'}, bands, @plot);
legend(h, {'position', 'commanded', 'target'}, 'Location', 'northeast');
ylabel('from EOT [mm]');
ax(3) = subplot(4, 1, 3);
drawSeries(t, {d.velocity_mm_s}, d.gap, {cLine}, {'-'}, bands, @plot);
ylabel('velocity [mm/s]');
ax(4) = subplot(4, 1, 4);
drawSeries(t, {d.torque_pct}, d.gap, {cLine}, {'-'}, bands, @plot);
ylabel('torque [%]');
xlabel('time [s]');
linkaxes(ax, 'x');

% ----- Delivery -----
f2 = newFigure('delivery', opts.visible);
ax = zeros(1, 2);
ax(1) = subplot(3, 1, 1);
drawSeries(t, {d.volume_mL}, d.gap, {cLine}, {'-'}, bands, @plot);
ylabel('volume [mL]');
title('Delivery');
ax(2) = subplot(3, 1, 2);
drawSeries(t, {d.flow_mL_s}, d.gap, {cLine}, {'-'}, bands, @plot);
ylabel('flow [mL/s]');
xlabel('time [s]');
linkaxes(ax, 'x');
subplot(3, 1, 3);
drawSeries(d.volume_mL, {d.pressureAbs_atm}, d.gap, {cLine}, {'-'}, ...
    struct('x0', {}, 'x1', {}, 'color', {}), @plot, false);
ylabel('pressure [atm abs]');
xlabel('volume [mL]');

% ----- Status -----
f3 = newFigure('status', opts.visible);
ax = zeros(1, 5);
ax(1) = subplot(5, 1, 1);
h = drawSeries(t, {d.statusCode, d.state}, d.gap, {cLine, cTarget}, ...
    {'-', '-'}, bands, @stairs);
legend(h, {'statusCode', 'state'}, 'Location', 'northeast');
ylabel('code');
title('Status');
ax(2) = subplot(5, 1, 2);
h = drawSeries(t, {d.modeCode, d.confCode}, d.gap, {cLine, cTarget}, ...
    {'-', '--'}, bands, @stairs);
legend(h, {'modeCode', 'confCode'}, 'Location', 'northeast');
ylabel('mode');
ax(3) = subplot(5, 1, 3);
flags = {d.motionEnable, d.inMotion + 2, d.faultActive + 4, d.homingComplete + 6};
h = drawSeries(t, flags, d.gap, {cLine, cLine2, cTarget, cLine3}, ...
    {'-', '-', '-', '-'}, bands, @stairs);
legend(h, {'motionEnable', 'inMotion', 'faultActive', 'homingComplete'}, ...
    'Location', 'northeast');
ylabel('flags (offset)');
ax(4) = subplot(5, 1, 4);
h = drawSeries(t, {d.linkOk, d.doSafety + 2}, d.gap, {cLine, cTarget}, ...
    {'-', '-'}, bands, @stairs);
legend(h, {'linkOk', 'doSafety'}, 'Location', 'northeast');
ylabel('link, safety (offset)');
ax(5) = subplot(5, 1, 5);
n = numel(t);
if n > 1
    tt = t(2:end);
    counterStep = mod(diff(d.slaveCounter), 65536);
    roundTrip = mod(d.heartbeat(2:end) - d.heartbeatEcho(2:end), 65536);
    h = drawSeries(tt, {counterStep, roundTrip}, d.gap(2:end), ...
        {cLine, cTarget}, {'-', '-'}, bands, @stairs);
    legend(h, {'slave counter step per record', 'heartbeat round trip [steps]'}, ...
        'Location', 'northeast');
end
ylabel('link health');
xlabel('time [s]');
linkaxes(ax, 'x');

figs = [f1 f2 f3];
if opts.save
    base = 'trace';
    if isfield(d, 'file') && ~isempty(d.file)
        [~, base] = fileparts(d.file);
    end
    tags = {'overview', 'delivery', 'status'};
    % Octave's hgsave warns that it skips the onCleanup object inside the
    % linked axes' property link; the figure itself is saved.
    warnState = warning();
    warning('off', 'all');
    restore = onCleanup(@() warning(warnState));
    for k = 1:3
        name = fullfile(opts.savedir, [base '_' tags{k} '.fig']);
        if exist('savefig') %#ok<EXIST>
            savefig(figs(k), name);
        else
            hgsave(figs(k), name);
        end
    end
end
end

% %% ===== LOCAL FUNCTIONS =====

function f = newFigure(tag, visible)
%NEWFIGURE Figure named by its tag, visible or not, sized for three to five rows.
f = figure('Visible', visible, 'Name', tag, 'NumberTitle', 'off');
pos = get(f, 'Position');
set(f, 'Position', [pos(1), pos(2), 900, 700]);
end

function y = maskOut(y, keep)
%MASKOUT NaN where keep is false, so the line shows only the kept samples.
y(~keep) = NaN;
end

function [tt, yy] = breakAtGaps(t, y, gap)
%BREAKATGAPS Insert a NaN sample before each gap index so the line breaks.
n = numel(t);
shift = cumsum(gap(:));
at = (1:n)' + shift;
tt = NaN(n + sum(gap(:)), 1);
yy = tt;
tt(at) = t(:);
yy(at) = y(:);
end

function runs = shadeRuns(d)
%SHADERUNS Time spans of the fault, link-down and E-STOP runs: a struct
%   array with the span ends x0, x1 and the band colour.
masks = {d.statusCode == 10 | d.faultActive ~= 0, d.linkOk == 0, d.statusCode == 20};
colors = {[1 0.85 0.85], [0.88 0.88 0.88], [1 0.96 0.75]};
t = d.tRel_s;
half = 0.025;
if numel(t) > 1
    half = median(diff(t)) / 2;
end
runs = struct('x0', {}, 'x1', {}, 'color', {});
for m = 1:3
    on = masks{m}(:);
    % A run ends where the mask drops or a gap interrupts the record.
    start = find(on & ~[false; on(1:end-1)] | on & d.gap(:));
    for s = 1:numel(start)
        e = start(s);
        while e < numel(on) && on(e + 1) && ~d.gap(e + 1)
            e = e + 1;
        end
        runs(end + 1) = struct('x0', t(start(s)) - half, 'x1', t(e) + half, ...
            'color', colors{m}); %#ok<AGROW>
    end
end
end

function h = drawSeries(x, ys, gap, colors, styles, bands, plotFcn, markGaps)
%DRAWSERIES Draw the bands, the dashed gap markers and the series on the
%   current axes; returns the handles of the series for the legend. The
%   y limits are fixed from the data before the bands are drawn, so the
%   bands span the axes without taking part in the autoscale. markGaps
%   is false when x is not time.
if nargin < 8
    markGaps = true;
end
hold on;
pooled = [];
for k = 1:numel(ys)
    v = ys{k}(isfinite(ys{k}));
    pooled = [pooled; v(:)]; %#ok<AGROW>
end
if isempty(pooled)
    lim = [0 1];
else
    lim = [min(pooled) max(pooled)];
    pad = 0.05 * (lim(2) - lim(1));
    if pad == 0
        pad = max(0.5, 0.05 * abs(lim(1)));
    end
    lim = lim + [-pad pad];
end
for b = 1:numel(bands)
    patch([bands(b).x0 bands(b).x1 bands(b).x1 bands(b).x0], ...
        [lim(1) lim(1) lim(2) lim(2)], bands(b).color, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end
gx = x(gap(:) & markGaps);
for g = 1:numel(gx)
    plot([gx(g) gx(g)], lim, 'k--', 'HandleVisibility', 'off');
end
h = zeros(1, numel(ys));
for k = 1:numel(ys)
    [xx, yy] = breakAtGaps(x, ys{k}, gap);
    h(k) = plotFcn(xx, yy, 'Color', colors{k}, 'LineStyle', styles{k});
end
set(gca, 'YLim', lim);
if numel(x) > 1 && max(x) > min(x)
    set(gca, 'XLim', [min(x) max(x)]);
end
grid on;
end
