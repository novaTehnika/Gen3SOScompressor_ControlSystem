function r = pressureNoiseRun(label, path, opts)
%PRESSURENOISERUN Drive the axis along a path and measure the noise on the
%   transducer input while it moves. The script is the master for the run:
%   one Modbus TCP connection carries the protocol exchange (HR1-12 out,
%   IR1-20 back, every 50 ms, sequenced by masterStep as in the shell) and,
%   between exchanges, the slave's step probe (FB_Probe, read by
%   benchProbe), which records the raw pressure input every scan. The
%   shell model must not be running: a second client on the controller
%   stalls the shell's exchange and the slave raises FAULT_COMM while motion
%   is commanded. The safety relay is held by bench/common/SafetyHold.slx
%   (buildSafetyHold), started and stopped by this script.
%
%   r = pressureNoiseRun(label)             jog +2 mm/s 8 s, then -2 mm/s 8 s
%   r = pressureNoiseRun(label, path)
%   r = pressureNoiseRun(label, path, opts)
%
%   path  cell array of steps, run in order with a stop between them:
%     {'idle', seconds}             no mode commanded
%     {'jog', mm_per_s, seconds}    velocity mode (3), signed
%     {'position', mm, seconds}     position mode (2): move, then hold
%     {'pressure', MPa, seconds}    pressure mode (7), target in MPa gauge
%   Each step takes one-second windows while it runs and one more
%   one-second capture opts.settle seconds after its running status appears
%   (position: after arrival). A window and the capture are each a probe
%   capture of round(1 / scan) consecutive records (250 at the 4 ms scan),
%   armed one at a time, the next window once the one before is read out
%   of the slave's ring; the window's mean, rms, minimum and maximum are
%   computed here (pressureNoiseStats). The step ends after its seconds,
%   once the records of its last window and of the capture are written;
%   they are read out in the following gaps, as the ring keeps them.
%
%   opts.settle        s before the capture (default 1)
%   opts.safetyModel   model that holds the safety relay ('SafetyHold';
%                      '' when the relay is held another way)
%   opts.link, opts.now, opts.wait
%                      transport and clock, replaced by the simulator test
%
%   Captures are saved as bench/common/captures/<label>_<n>_<step>_<timestamp>.csv in
%   the format pressureNoiseReport reads, and the run as
%   captures/<label>_run_<timestamp>.csv (t, position, velocity, MPa
%   gauge, status, step, exchange ms). r holds the same with the per-step
%   windows (r.windows rows [step, t armed, mean, rms, min, max], volts)
%   and the reports.
%
%   The run ends early when the master reports a fault, link loss or an
%   unconfirmed mode; r.abort says which. Ctrl-C stops the axis: the
%   cleanup sends one exchange with MotionEnable low and mode 0, and the
%   slave's own watchdog stops it within 300 ms if the script dies.

if nargin < 1 || isempty(label)
    label = 'run';
end
if nargin < 2 || isempty(path)
    path = {{'jog', 2, 8}, {'jog', -2, 8}};
end
if nargin < 3
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'common'), fullfile(here, '..', '..', 'master-src', 'logic'));
bcfg = benchConfig();
cfg = masterConfig();
map = modbusMap();

opts = withDefault(opts, 'settle', 1);
opts = withDefault(opts, 'safetyModel', bcfg.safetyModel);
opts = withDefault(opts, 'saveDir', bcfg.captures);
opts = withDefault(opts, 'report', true);
if ~isfield(opts, 'now')
    t0 = tic;
    opts.now = @() toc(t0);
end
if ~isfield(opts, 'wait')
    opts.wait = @(tTarget) waitUntil(opts.now, tTarget);
end
if ~isfield(opts, 'link')
    opts.link = benchLink();
end
link = opts.link;

fs = 1 / bcfg.slaveScan;
nRecords = round(fs);                   % records in one second

C = codes();

steps = parsePath(path);
nSteps = numel(steps);

% --- Safety relay and the stop on exit --------------------------------------
clear masterStep masterStateMachine
safety = startSafety(opts.safetyModel, bcfg.safetyDir);
cleanup = onCleanup(@() shutdown(link, map, safety));

% --- Run state --------------------------------------------------------------
app = struct('op', C.OP_STOP, 'cmdSeq', 0, 'targetPosition', 0, ...
    'targetVelocity', 0, 'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0, ...
    'positionSpeed', masterConfig().positionSpeed);
mbWrite = zeros(1, map.nWrite);
mbRead = zeros(1, map.nRead);
m = struct('statusCode', C.STATUS_LINK_WAIT, 'state', 0, 'position', 0, ...
    'velocity', 0, 'pressure', 0, 'faultCode', 0, 'linkOk', 0);

seq.idx = 0;            % current step, 0 before the first
seq.phase = 'wait';     % wait, next, enter, run, stop, finish
seq.tPhase = 0;
seq.tRun = 0;
seq.armed = false;
seq.armRequest = false;
seq.armTag = '';
seq.tCapEnd = 0;       % s, when the records of the last arm are written
seq.abort = '';

probe = benchProbe('new', bcfg.probe, bcfg.slaveScan);
clock = struct('now', opts.now, 'wait', opts.wait);
ctx = struct('settle', opts.settle, 'capSeconds', nRecords * bcfg.slaveScan);

windows = zeros(0, 6);          % [step t mean rms min max], volts
captures = {};
runLog = zeros(0, 7);
exchangeFailures = 0;

fprintf('pressureNoiseRun %s: %d steps\n', label, nSteps);
k = 0;
while true
    tCycle = k * cfg.exchangePeriod;
    opts.wait(tCycle);
    tNow = opts.now();

    % --- Protocol exchange, as the shell's Modbus blocks do it ------------
    tx = tic;
    try
        link.write(map.hr.heartbeat, mbWrite);
        mbRead = link.read(1, map.nRead);
    catch err
        exchangeFailures = exchangeFailures + 1;
        if exchangeFailures <= 3
            fprintf('  exchange failed at %.2f s: %s\n', tNow, err.message);
        end
    end
    exchangeMs = toc(tx) * 1000;
    [mbWrite, m] = stepMaster(mbRead, app, map);

    % --- Path sequencing on the master's status ----------------------------
    [seq, app] = sequence(seq, app, m, steps, tNow, probe.armBusy, probe.nPending, ctx, C, cfg);
    if seq.armRequest
        probe = benchProbe('arm', probe, struct('step', seq.idx, 'tag', seq.armTag, ...
            'nRecords', nRecords, 'nPre', 0));
        seq.armRequest = false;
    end
    runLog(end + 1, :) = [tNow m.position m.velocity ...
        (m.pressure - cfg.atmAmbient) / cfg.atmPerMPa m.statusCode seq.idx exchangeMs];

    if strcmp(seq.phase, 'finish') && tNow - seq.tPhase > 0.2 && ...
            (m.statusCode == C.STATUS_IDLE || any(m.statusCode == C.abortStatus) || ...
             tNow - seq.tPhase > cfg.tStopMax + 1)
        break;
    end

    % --- One probe transaction in the gap ----------------------------------
    if exchangeMs <= 150
        [probe, got] = benchProbe('slot', probe, link, tNow);
        [windows, captures] = keepCaptures(windows, captures, got, fs);
    end

    % --- Second master step of the cycle, registers held -------------------
    opts.wait(tCycle + cfg.dt);
    [mbWrite, m] = stepMaster(mbRead, app, map);
    k = k + 1;
end

% --- Captures still in the slave's ring ---------------------------------------
[probe, got] = benchProbe('drain', probe, link, clock);
[windows, captures] = keepCaptures(windows, captures, got, fs);

% --- Reports and files --------------------------------------------------------
r.label = label;
r.steps = steps;
r.abort = seq.abort;
r.runLog = runLog;
r.windows = windows;
r.captures = captures;
r.exchangeFailures = exchangeFailures;
r.probeFailures = probe.failures + probe.lost;
r.files = {};
if ~isempty(seq.abort)
    fprintf('\nRUN ABORTED: %s; master status %d, fault code %d\n', seq.abort, ...
        m.statusCode, m.faultCode);
end
if ~exist(opts.saveDir, 'dir')
    mkdir(opts.saveDir);
end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
shuntOhm = benchGvl().G_cfgPressureShuntOhms;
mpaPerVolt = 1000 / shuntOhm;
for c = 1:numel(captures)
    cap = captures{c};
    name = sprintf('%s_%d_%s', label, cap.step, steps(cap.step).name);
    stats = windows(windows(:, 1) == cap.step, 3:6);
    if opts.report
        cap.report = pressureNoiseReport(cap.volts, fs, stats, name);
    end
    fname = fullfile(opts.saveDir, sprintf('%s_%s.csv', name, stamp));
    t = (0:numel(cap.volts) - 1)' / fs;
    fid = fopen(fname, 'w');
    fprintf(fid, 't,volts,MPa\n');
    fprintf(fid, '%.4f,%.5f,%.5f\n', [t cap.volts(:) mpaPerVolt * (cap.volts(:) - 4 * shuntOhm / 1000)]');
    fclose(fid);
    cap.file = fname;
    captures{c} = cap;
    r.files{end + 1} = fname;
end
r.captures = captures;
fname = fullfile(opts.saveDir, sprintf('%s_run_%s.csv', label, stamp));
fid = fopen(fname, 'w');
fprintf(fid, 't,position,velocity,MPa,status,step,exchangeMs\n');
fprintf(fid, '%.3f,%.3f,%.3f,%.4f,%d,%d,%.1f\n', runLog');
fclose(fid);
r.runFile = fname;

fprintf('\n%-4s %-10s %-12s %8s %12s %12s %12s\n', 'step', 'type', 'target', 'windows', ...
    'window rms', 'capture rms', 'strongest');
for s = 1:nSteps
    inStep = windows(:, 1) == s;
    wrms = mean(windows(inStep, 4)) * 1000;
    crms = NaN; line = '';
    for c = 1:numel(captures)
        if captures{c}.step == s && isfield(captures{c}, 'report')
            crms = captures{c}.report.rmsMv;
            line = sprintf('%g Hz %.2f mV', captures{c}.report.peakHz(1), ...
                captures{c}.report.peakMv(1));
        end
    end
    fprintf('%-4d %-10s %-12s %8d %9.2f mV %9.2f mV %12s\n', s, steps(s).name, ...
        steps(s).targetText, sum(inStep), wrms, crms, line);
end
fprintf('exchange failures %d, probe failures %d, exchange p99 %.1f ms\n', ...
    exchangeFailures, probe.failures + probe.lost, prctileOf(runLog(:, 7), 99));
fprintf('saved %s\n', fname);
end


% ===========================================================================
function [mbWrite, m] = stepMaster(mbRead, app, map)
[mbWrite, ~, ~, m.position, m.velocity, m.pressure, m.statusCode, ...
    m.faultCode, m.state, m.linkOk] = masterStep(mbRead, app.op, app.cmdSeq, ...
    app.targetPosition, app.targetVelocity, app.jogDirection, ...
    app.targetPressure, app.estop, app.mixer, app.positionSpeed);
mbWrite = double(mbWrite(:)');
end


function C = codes()
% Operation and status codes of masterStateMachine, and the master state
% that marks the axis idle.
C.OP_STOP = 0; C.OP_POSITION = 2; C.OP_JOG = 3; C.OP_PRESSURE = 4;
C.STATUS_IDLE = 0; C.STATUS_POS_AT_TARGET = 3; C.STATUS_JOG = 4; C.STATUS_PRESSURE = 5;
C.STATUS_LINK_WAIT = 35;
C.abortStatus = [10 30 32 33 34];       % fault, no confirm, reset failed, stop timeout, link lost (E-stop shows for one step at link-up)
C.ST_IDLE = 1;
end


function [seq, app] = sequence(seq, app, m, steps, tNow, armBusy, nPending, ctx, C, cfg)
% Advance the path: command each step through the app inputs, wait for its
% running status, time it, then stop and wait for idle. Arms the capture
% once the settle time has passed and a window whenever none is waiting to
% be read out; the step ends once the records of what it armed are written.
seq.armRequest = false;
settle = ctx.settle;
inPhase = tNow - seq.tPhase;

if any(m.statusCode == C.abortStatus) && ~strcmp(seq.phase, 'finish')
    seq.abort = sprintf('master status %d during step %d, phase %s', ...
        m.statusCode, seq.idx, seq.phase);
    app.op = C.OP_STOP;
    seq.phase = 'finish';
    seq.tPhase = tNow;
    return;
end

switch seq.phase
    case 'wait'
        % Commands sent before the master is idle with the link up are
        % dropped by its state machine.
        if m.statusCode == C.STATUS_IDLE && m.state == C.ST_IDLE && m.linkOk
            seq.phase = 'next';
            seq.tPhase = tNow;
        elseif inPhase > 5
            seq.abort = sprintf('master not idle with the link up after %.0f s (status %d)', ...
                inPhase, m.statusCode);
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end

    case 'next'
        seq.idx = seq.idx + 1;
        if seq.idx > numel(steps)
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
            return;
        end
        s = steps(seq.idx);
        switch s.name
            case 'idle'
                app.op = C.OP_STOP;
            case 'jog'
                app.op = C.OP_JOG;
                app.targetVelocity = abs(s.target);
                app.jogDirection = sign(s.target);
            case 'position'
                app.op = C.OP_POSITION;
                app.targetPosition = s.target;
            case 'pressure'
                app.op = C.OP_PRESSURE;
                app.targetPressure = s.target * cfg.atmPerMPa + cfg.atmAmbient;   % the app's atm absolute
        end
        app.cmdSeq = app.cmdSeq + 1;
        seq.phase = 'enter';
        seq.tPhase = tNow;
        seq.armed = false;
        fprintf('  step %d: %s %s\n', seq.idx, s.name, s.targetText);

    case 'enter'
        s = steps(seq.idx);
        switch s.name
            case 'idle',     running = m.statusCode == C.STATUS_IDLE && m.state == C.ST_IDLE;
            case 'jog',      running = m.statusCode == C.STATUS_JOG;
            case 'position', running = m.statusCode == C.STATUS_POS_AT_TARGET;
            otherwise,       running = m.statusCode == C.STATUS_PRESSURE;
        end
        if strcmp(s.name, 'position')
            limit = 10 + abs(s.target - m.position) / 0.5;   % arrival at >= 0.5 mm/s
        else
            limit = 10;
        end
        if running
            seq.phase = 'run';
            seq.tPhase = tNow;
            seq.tRun = tNow;
        elseif inPhase > limit
            seq.abort = sprintf('step %d (%s) not running after %.0f s (status %d)', ...
                seq.idx, s.name, inPhase, m.statusCode);
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end

    case 'run'
        s = steps(seq.idx);
        if ~seq.armed && inPhase >= settle && ~armBusy
            seq.armRequest = true;
            seq.armTag = 'capture';
            seq.armed = true;
        elseif ~armBusy && nPending == 0 && inPhase < s.seconds
            seq.armRequest = true;
            seq.armTag = 'window';
        end
        if seq.armRequest
            % The slave acts on the arm a gap or two after it is written.
            seq.tCapEnd = tNow + ctx.capSeconds + 2 * cfg.exchangePeriod;
        end
        if inPhase >= max(s.seconds, settle + 0.1) && seq.armed && ~armBusy && ...
                tNow >= seq.tCapEnd
            app.op = C.OP_STOP;
            seq.phase = 'stop';
            seq.tPhase = tNow;
        end

    case 'stop'
        if m.statusCode == C.STATUS_IDLE && m.state == C.ST_IDLE
            seq.phase = 'next';
            seq.tPhase = tNow;
        elseif inPhase > cfg.tStopMax + cfg.tModeClear + 2
            seq.abort = sprintf('axis not idle %.0f s after stop (status %d)', ...
                inPhase, m.statusCode);
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end
end
end


function [windows, captures] = keepCaptures(windows, captures, got, fs)
% File the probe captures read out in this gap: a window gives its
% statistics, the capture its samples for the spectrum.
for k = 1:numel(got)
    c = got{k};
    if c.lost
        fprintf('  %s of step %d lost: the ring was overwritten before it was read\n', ...
            c.tag, c.step);
        continue;
    end
    volts = c.data(:, 7);               % raw pressure input [V]
    if strcmp(c.tag, 'window')
        windows(end + 1, :) = [c.step c.tArm pressureNoiseStats(volts)]; %#ok<AGROW>
    else
        captures{end + 1} = struct('step', c.step, 'tArm', c.tArm, ...
            'tRead', c.tRead, 'volts', volts, 'fs', fs); %#ok<AGROW>
        fprintf('  capture %d of step %d armed at %.2f s, read at %.2f s\n', ...
            numel(captures), c.step, c.tArm, c.tRead);
    end
end
end


function steps = parsePath(path)
steps = struct('name', {}, 'target', {}, 'seconds', {}, 'targetText', {});
for k = 1:numel(path)
    p = path{k};
    name = lower(p{1});
    switch name
        case 'idle'
            s = struct('name', name, 'target', 0, 'seconds', p{2}, 'targetText', '');
        case 'jog'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g mm/s', p{2}));
        case 'position'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g mm', p{2}));
        case 'pressure'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g MPa', p{2}));
        otherwise
            error('unknown step type "%s" (idle, jog, position, pressure)', p{1});
    end
    steps(end + 1) = s;
end
end


function safety = startSafety(model, here)
% Start the model that drives the safety relay. Returns '' when none.
safety = '';
if isempty(model)
    return;
end
file = fullfile(here, [model '.slx']);
if ~exist(file, 'file')
    error(['%s not found: run bench/common/buildSafetyHold once, or pass ' ...
        'opts.safetyModel = '''' if the relay is held another way'], file);
end
load_system(file);
set_param(model, 'SimulationCommand', 'start');
safety = model;
t0 = tic;
while ~strcmp(get_param(model, 'SimulationStatus'), 'running')
    if toc(t0) > 30
        error('%s did not start in 30 s (status %s)', model, get_param(model, 'SimulationStatus'));
    end
    pause(0.1);
end
pause(1);     % the relay is closed before the first command
end


function shutdown(link, map, safety)
% Last exchange with MotionEnable low and mode 0, then release the relay.
try
    w = zeros(1, map.nWrite);
    w(map.hr.heartbeat) = floor(rand() * 65535);
    link.write(map.hr.heartbeat, w);
catch
end
try
    link.close();
catch
end
if ~isempty(safety)
    try
        set_param(safety, 'SimulationCommand', 'stop');
    catch
    end
end
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


function opts = withDefault(opts, name, value)
if ~isfield(opts, name)
    opts.(name) = value;
end
end


function v = prctileOf(x, p)
x = sort(x(~isnan(x)));
if isempty(x)
    v = NaN;
else
    v = x(max(1, ceil(p / 100 * numel(x))));
end
end
