function r = pressureNoiseProbe(label, nWindows, opts)
%PRESSURENOISEPROBE Measure the transducer noise with the axis at rest from
%   the slave's step probe (FB_Probe, read by benchProbe), report it, and
%   save the capture to bench/common/captures/<label>_<yyyymmdd_HHMMSS>.csv.
%   Runs from the MATLAB command line on the lab PC (no Simulink, no
%   Industrial Communication Toolbox). Nothing is commanded: the script
%   writes only the probe's arm, request and length registers.
%
%   r = pressureNoiseProbe('idle')        5 one-second windows, one capture
%   r = pressureNoiseProbe('jog', 10)     10 windows
%
%   Each window is a probe capture of round(1 / scan) consecutive records
%   (1 s, 250 at the 4 ms scan) of the raw pressure input (record word 6,
%   0.2 mV per count); its mean, rms, minimum and maximum are computed here
%   (pressureNoiseStats). The capture is one more 250-record capture, for
%   the spectrum. The captures are armed one after another, each once the
%   one before is read out, so every window is a second of consecutive
%   scans.
%
%   opts.link, opts.now, opts.wait, opts.saveDir, opts.report
%                      transport, clock, output directory and printing,
%                      replaced by the simulator test

if nargin < 1 || isempty(label)
    label = 'run';
end
if nargin < 2 || isempty(nWindows)
    nWindows = 5;
end
if nargin < 3
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'common'));
bcfg = benchConfig();
pr = bcfg.probe;
fs = 1 / bcfg.slaveScan;
nRecords = round(fs);                   % records in one second

if ~isfield(opts, 'saveDir')
    opts.saveDir = bcfg.captures;
end
if ~isfield(opts, 'report')
    opts.report = true;
end
if ~isfield(opts, 'now')
    t0 = tic;
    opts.now = @() toc(t0);
end
if ~isfield(opts, 'wait')
    opts.wait = @(tTarget) waitUntil(opts.now, tTarget);
end
if ~isfield(opts, 'link')
    opts.link = benchLink();
    cleanup = onCleanup(@() opts.link.close());
end
link = opts.link;

% --- Windows, then the capture: one 1 s capture at a time -------------------
probe = benchProbe('new', pr, bcfg.slaveScan);
stats = zeros(0, 4);
capture = [];
tStart = opts.now();
tLimit = (nWindows + 1) * (nRecords * bcfg.slaveScan + 10) + 10;   % s
fprintf('pressureNoiseProbe %s: %d windows and a capture of %d records\n', ...
    label, nWindows, nRecords);
while isempty(capture)
    t = opts.now();
    if probe.nPending == 0
        if size(stats, 1) < nWindows
            tag = 'window';
        else
            tag = 'capture';
        end
        probe = benchProbe('arm', probe, struct('step', size(stats, 1) + 1, ...
            'tag', tag, 'nRecords', nRecords, 'nPre', 0));
    end
    [probe, got] = benchProbe('slot', probe, link, t);
    for k = 1:numel(got)
        c = got{k};
        if c.lost || size(c.data, 1) < nRecords
            error('capture lost: the slave''s ring was overwritten before it was read');
        end
        if strcmp(c.tag, 'window')
            stats(end + 1, :) = pressureNoiseStats(c.data(:, 7)); %#ok<AGROW>
        else
            capture = c;
        end
    end
    if probe.failures > 5
        error(['probe not answering (%d failed transactions): is the slave\n' ...
            'running with G_cfgProbeEnable TRUE and the register groups to IR524 / HR114?'], ...
            probe.failures);
    end
    if t - tStart > tLimit
        error('noise measurement not finished after %.0f s', tLimit);
    end
    opts.wait(t + pr.drainGap);
end
volts = capture.data(:, 7);

% --- Report and save --------------------------------------------------------
if opts.report
    r = pressureNoiseReport(volts, fs, stats, label);
else
    r = struct('mpaPerVolt', 1000 / benchGvl().G_cfgPressureShuntOhms, ...
        'shuntOhm', benchGvl().G_cfgPressureShuntOhms);
end
r.stats = stats;
r.volts = volts;
r.probeFailures = probe.failures;

if ~exist(opts.saveDir, 'dir')
    mkdir(opts.saveDir);
end
fname = fullfile(opts.saveDir, sprintf('%s_%s.csv', label, datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(fname, 'w');
fprintf(fid, 't,volts,MPa\n');
fprintf(fid, '%.4f,%.5f,%.5f\n', [capture.data(:, 1) volts(:) ...
    r.mpaPerVolt * (volts(:) - 4 * r.shuntOhm / 1000)]');
fclose(fid);
fprintf('saved %s\n', fname);
r.file = fname;
end


function waitUntil(nowFcn, tTarget)
while true
    remaining = tTarget - nowFcn();
    if remaining <= 0
        return;
    end
    pause(min(remaining, 0.002));
end
end
