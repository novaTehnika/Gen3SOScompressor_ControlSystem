function r = pressureNoiseRun(label, path, opts)
%PRESSURENOISERUN Drive the axis along a path and probe the transducer
%   while it moves. The script is the master for the run: one Modbus TCP
%   connection carries the protocol exchange (HR1-6 out, IR1-20 back,
%   every 50 ms, sequenced by masterStep as in the shell) and, between
%   exchanges, the probe registers of PRG_PressureProbe. The shell model
%   must not be running: a second client on the controller stalls the
%   shell's exchange and the slave raises FAULT_COMM while motion is
%   commanded. The safety relay is held by SafetyHold.slx (buildSafetyHold),
%   started and stopped by this script.
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
%   Each step collects the one-second windows while it runs and takes one
%   250-scan capture opts.settle seconds after its running status appears
%   (position: after arrival). The step ends after its seconds, once the
%   capture is in.
%
%   opts.settle        s before the capture (default 1)
%   opts.safetyModel   model that holds the safety relay ('SafetyHold';
%                      '' when the relay is held another way)
%   opts.link, opts.now, opts.wait
%                      transport and clock, replaced by the simulator test
%
%   Captures are saved as captures/<label>_<n>_<step>_<timestamp>.csv in
%   the format pressureNoiseReport reads, and the run as
%   captures/<label>_run_<timestamp>.csv (t, position, velocity, pressure,
%   status, step, exchange ms). r holds the same with the per-step
%   windows and the reports.
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
addpath(fullfile(here, '..', 'modbus'), fullfile(here, '..', '..', 'master-src', 'logic'));
bcfg = modbusBenchConfig();
cfg = masterConfig();
map = modbusMap();

opts = withDefault(opts, 'settle', 1);
opts = withDefault(opts, 'safetyModel', 'SafetyHold');
opts = withDefault(opts, 'saveDir', fullfile(here, 'captures'));
opts = withDefault(opts, 'report', true);
if ~isfield(opts, 'now')
    t0 = tic;
    opts.now = @() toc(t0);
end
if ~isfield(opts, 'wait')
    opts.wait = @(tTarget) waitUntil(opts.now, tTarget);
end
if ~isfield(opts, 'link')
    opts.link = rawLink(bcfg);
end
link = opts.link;

% Probe registers (1-based), as declared in slave/PRG_PressureProbe.st.
pr.hrArm      = 101;
pr.irStats    = 101;    % IR101-108 mean, rms, min, max (REAL, low word first)
pr.irWindow   = 109;    % IR109 window, IR110 state, IR111-112 live, IR113 arm echo
pr.irSamples  = 121;    % IR121-370
pr.nSamples   = 250;
pr.countsPerVolt = 5000;
pr.captureTimeout = 3;  % s from arm to ready
fs = 1 / bcfg.slaveScan;

C = codes();

steps = parsePath(path);
nSteps = numel(steps);

% --- Safety relay and the stop on exit --------------------------------------
clear masterStep masterStateMachine
safety = startSafety(opts.safetyModel, here);
cleanup = onCleanup(@() shutdown(link, map, safety));

% --- Run state --------------------------------------------------------------
app = struct('op', C.OP_STOP, 'cmdSeq', 0, 'targetPosition', 0, ...
    'targetVelocity', 0, 'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0);
mbWrite = zeros(1, map.nWrite);
mbRead = zeros(1, map.nRead);
m = struct('statusCode', C.STATUS_LINK_WAIT, 'state', 0, 'position', 0, ...
    'velocity', 0, 'pressure', 0, 'faultCode', 0, 'linkOk', 0);

seq.idx = 0;            % current step, 0 before the first
seq.phase = 'wait';     % wait, next, enter, run, stop, finish
seq.tPhase = 0;
seq.tRun = 0;
seq.armed = false;
seq.abort = '';

probe.next = 'poll';    % poll, stats, cap1, cap2
probe.armRequest = false;
probe.armValue = 0;
probe.pending = false;
probe.tArm = 0;
probe.lastWindow = NaN;
probe.counts = zeros(1, pr.nSamples);
probe.failures = 0;

windows = zeros(0, 7);          % [step inRun t mean rms min max], volts
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
    [seq, app] = sequence(seq, app, m, steps, tNow, opts.settle, probe.pending, C, cfg);
    if seq.armRequest
        probe.armRequest = true;
        seq.armRequest = false;
    end
    runLog(end + 1, :) = [tNow m.position m.velocity m.pressure / cfg.atmPerMPa ...
        m.statusCode seq.idx exchangeMs];

    if strcmp(seq.phase, 'finish') && tNow - seq.tPhase > 0.2 && ...
            (m.statusCode == C.STATUS_IDLE || any(m.statusCode == C.abortStatus) || ...
             tNow - seq.tPhase > cfg.tStopMax + 1)
        break;
    end

    % --- One probe transaction in the gap ----------------------------------
    [probe, windows, captures] = probeSlot(probe, link, pr, windows, captures, ...
        seq, tNow, fs);

    % --- Second master step of the cycle, registers held -------------------
    opts.wait(tCycle + cfg.dt);
    [mbWrite, m] = stepMaster(mbRead, app, map);
    k = k + 1;
end

% --- Reports and files --------------------------------------------------------
r.label = label;
r.steps = steps;
r.abort = seq.abort;
r.runLog = runLog;
r.windows = windows;
r.captures = captures;
r.exchangeFailures = exchangeFailures;
r.probeFailures = probe.failures;
r.files = {};
if ~isempty(seq.abort)
    fprintf('\nRUN ABORTED: %s; master status %d, fault code %d\n', seq.abort, ...
        m.statusCode, m.faultCode);
end
if ~exist(opts.saveDir, 'dir')
    mkdir(opts.saveDir);
end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
mpaPerVolt = 1000 / 434;
for c = 1:numel(captures)
    cap = captures{c};
    name = sprintf('%s_%d_%s', label, cap.step, steps(cap.step).name);
    inRun = windows(:, 1) == cap.step & windows(:, 2) == 1;
    stats = windows(inRun, 4:7);
    if opts.report
        cap.report = pressureNoiseReport(cap.volts, fs, stats, name);
    end
    fname = fullfile(opts.saveDir, sprintf('%s_%s.csv', name, stamp));
    t = (0:pr.nSamples - 1)' / fs;
    fid = fopen(fname, 'w');
    fprintf(fid, 't,volts,MPa\n');
    fprintf(fid, '%.4f,%.5f,%.5f\n', [t cap.volts(:) mpaPerVolt * (cap.volts(:) - 4 * 0.434)]');
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
    inRun = windows(:, 1) == s & windows(:, 2) == 1;
    wrms = mean(windows(inRun, 5)) * 1000;
    crms = NaN; line = '';
    for c = 1:numel(captures)
        if captures{c}.step == s && isfield(captures{c}, 'report')
            crms = captures{c}.report.rmsMv;
            line = sprintf('%g Hz %.2f mV', captures{c}.report.peakHz(1), ...
                captures{c}.report.peakMv(1));
        end
    end
    fprintf('%-4d %-10s %-12s %8d %9.2f mV %9.2f mV %12s\n', s, steps(s).name, ...
        steps(s).targetText, sum(inRun), wrms, crms, line);
end
fprintf('exchange failures %d, probe failures %d, exchange p99 %.1f ms\n', ...
    exchangeFailures, probe.failures, prctileOf(runLog(:, 7), 99));
fprintf('saved %s\n', fname);
end


% ===========================================================================
function [mbWrite, m] = stepMaster(mbRead, app, map)
[mbWrite, ~, ~, m.position, m.velocity, m.pressure, m.statusCode, ...
    m.faultCode, m.state, m.linkOk] = masterStep(mbRead, app.op, app.cmdSeq, ...
    app.targetPosition, app.targetVelocity, app.jogDirection, ...
    app.targetPressure, app.estop, app.mixer);
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


function [seq, app] = sequence(seq, app, m, steps, tNow, settle, capturePending, C, cfg)
% Advance the path: command each step through the app inputs, wait for its
% running status, time it, then stop and wait for idle.
seq.armRequest = false;
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
                app.targetPressure = s.target * cfg.atmPerMPa;
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
        if ~seq.armed && inPhase >= settle
            seq.armRequest = true;
            seq.armed = true;
        end
        if inPhase >= max(s.seconds, settle + 0.1) && seq.armed && ~capturePending
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


function [probe, windows, captures] = probeSlot(probe, link, pr, windows, captures, seq, tNow, fs)
% One probe transaction per cycle: arm when asked, otherwise poll the
% window counter and capture state, and fetch statistics or the capture
% block when they are new.
isRun = double(strcmp(seq.phase, 'run'));
if probe.pending && tNow - probe.tArm > pr.captureTimeout
    fprintf('  capture armed at %.2f s not ready after %g s: skipped\n', ...
        probe.tArm, pr.captureTimeout);
    probe.pending = false;
    probe.next = 'poll';
end
try
    if probe.armRequest && strcmp(probe.next, 'poll')
        probe.armValue = mod(probe.armValue, 65534) + 1;
        link.writeOne(pr.hrArm, probe.armValue);
        probe.armRequest = false;
        probe.pending = true;
        probe.tArm = tNow;
        return;
    end
    switch probe.next
        case 'poll'
            s = link.read(pr.irWindow, 5);
            if s(1) ~= probe.lastWindow
                probe.lastWindow = s(1);
                probe.next = 'stats';
            elseif probe.pending && s(2) == 2 && s(5) == probe.armValue
                probe.next = 'cap1';
            end
        case 'stats'
            raw = link.read(pr.irStats, 8);
            windows(end + 1, :) = [seq.idx isRun tNow realOf(raw(1:2)) realOf(raw(3:4)) ...
                realOf(raw(5:6)) realOf(raw(7:8))];
            probe.next = 'poll';
        case 'cap1'
            probe.counts(1:125) = link.read(pr.irSamples, 125);
            probe.next = 'cap2';
        case 'cap2'
            probe.counts(126:250) = link.read(pr.irSamples + 125, 125);
            cap.step = seq.idx;
            cap.tArm = probe.tArm;
            cap.tRead = tNow;
            cap.volts = probe.counts / pr.countsPerVolt;
            cap.fs = fs;
            captures{end + 1} = cap;
            probe.pending = false;
            probe.next = 'poll';
            fprintf('  capture %d taken at %.2f s\n', numel(captures), probe.tArm);
    end
catch err
    probe.failures = probe.failures + 1;
    if probe.failures <= 3
        fprintf('  probe transaction failed at %.2f s: %s\n', tNow, err.message);
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
    error(['%s not found: run buildSafetyHold once, or pass ' ...
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


function x = realOf(words)
% REAL from two registers, low word first (the controller's word order).
x = double(typecast(uint16(words(:)'), 'single'));
end


function v = prctileOf(x, p)
x = sort(x(~isnan(x)));
if isempty(x)
    v = NaN;
else
    v = x(max(1, ceil(p / 100 * numel(x))));
end
end


% ===========================================================================
% Raw Modbus TCP over tcpclient: FC04 read, FC06 single write, FC16 write.
% The controller answers in about 4 ms; the modbus object adds some 7 ms
% per transaction, which the 25 ms gaps cannot spare.
function link = rawLink(bcfg)
t = tcpclient(bcfg.address, bcfg.port, 'Timeout', 0.5);
uid = uint8(bcfg.serverId);
link.t = t;
link.read = @(start, n) rawTxn(t, uid, [4, u16(start - 1), u16(n)], 4, n);
link.write = @(start, vals) rawTxn(t, uid, ...
    [16, u16(start - 1), u16(numel(vals)), 2 * numel(vals), u16s(vals)], 16, 0);
link.writeOne = @(addr, val) rawTxn(t, uid, [6, u16(addr - 1), u16(val)], 6, 0);
link.close = @() delete(t);
end


function regs = rawTxn(t, uid, pdu, fc, nRegs)
% Send one request and parse its response. Returns the registers of an
% FC04 read, or [] for a write.
persistent tid
if isempty(tid)
    tid = 0;
end
tid = mod(tid + 1, 65536);
flush(t);
pdu = uint8(pdu);
write(t, [u16(tid), uint8([0, 0]), u16(numel(pdu) + 1), uid, pdu]);
hdr = double(read(t, 7, 'uint8'));
if numel(hdr) < 7
    error('no response (%d of 7 header bytes)', numel(hdr));
end
len = hdr(5) * 256 + hdr(6);
body = double(read(t, len - 1, 'uint8'));
if numel(body) < len - 1
    error('short response (%d of %d bytes)', numel(body), len - 1);
end
if hdr(1) * 256 + hdr(2) ~= tid
    error('transaction ID %d, expected %d', hdr(1) * 256 + hdr(2), tid);
end
if body(1) >= 128
    error('exception response: function %d, code %d', body(1) - 128, body(2));
end
if body(1) ~= fc
    error('function %d in response, expected %d', body(1), fc);
end
if fc == 4
    if body(2) < 2 * nRegs
        error('%d bytes of registers, expected %d', body(2), 2 * nRegs);
    end
    regs = body(3:2:2 + 2 * nRegs) * 256 + body(4:2:3 + 2 * nRegs);
else
    regs = [];
end
end


function b = u16(v)
% Big-endian 16-bit value as two bytes.
v = mod(round(v), 65536);
b = uint8([floor(v / 256), mod(v, 256)]);
end


function b = u16s(vals)
b = uint8(zeros(1, 2 * numel(vals)));
for k = 1:numel(vals)
    b(2 * k - 1:2 * k) = u16(vals(k));
end
end
