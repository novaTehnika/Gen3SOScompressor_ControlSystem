function r = frictionRun(label, set, opts)
%FRICTIONRUN Drive the axis through the friction bench's speed points and
%   log the slave's feedback. The script is the master for the run: one
%   Modbus TCP connection carries the protocol exchange (HR1-6 out, IR1-20
%   back, every 50 ms, sequenced by masterStep as in the shell) and every
%   exchange is logged with the torque feedback, which the shell and the
%   operator app do not show. The shell model must not be running: a
%   second client on the controller stalls the shell's exchange and the
%   slave raises FAULT_COMM while motion is commanded. The safety relay is
%   held by bench/pressure/SafetyHold.slx (buildSafetyHold), started and
%   stopped by this script.
%
%   r = frictionRun(label)                  the 'unloaded' set
%   r = frictionRun(label, set)
%   r = frictionRun(label, set, opts)
%
%   set   'unloaded'  jog at each speed of frictionBenchConfig in both
%                     directions, descending then ascending, with the
%                     chamber vented
%         'gas'       the same with the steps shortened to gasStrokeMax,
%                     for a chamber holding a gas charge
%         'delivery'  pressure mode at opts.target (MPa gauge) for
%                     opts.seconds, or until the position reaches
%                     opts.xStop; the operator opens the outlet once the
%                     in-band status is printed
%         a cell array of steps, run in order with a stop between them:
%           {'idle', seconds}             no mode commanded
%           {'jog', mm_per_s, seconds}    velocity mode (3), signed
%           {'position', mm, seconds}     position mode (2): move, then hold
%           {'pressure', MPa, seconds}    pressure mode (7), MPa gauge
%
%   opts.speeds, opts.tSteady, opts.repeats, opts.strokeMax
%                      override frictionBenchConfig for the jog sets
%   opts.startPosition mm: a position step before the set (default none)
%   opts.target, opts.seconds
%                      delivery set target and duration
%   opts.pMax          MPa gauge: the run stops the axis above this
%                      (default by set from frictionBenchConfig)
%   opts.torqueAbort   % of rated: the run stops the axis above this
%   opts.xStop, opts.xMin
%                      mm: a step ends when the position passes xStop
%                      moving in or xMin moving out (default none)
%   opts.safetyModel   model that holds the safety relay ('SafetyHold';
%                      '' when the relay is held another way)
%   opts.link, opts.now, opts.wait
%                      transport and clock, replaced by the simulator test
%
%   The run is saved as captures/<label>_<set>_<timestamp>.csv with one row
%   per exchange: t (s), step, status (masterStateMachine code), mode
%   (slave mode code of the step), ref (HR5-6 as sent: mm/s, mm or MPa),
%   position (mm), velocity (mm/s), torque (% rated), pressure (MPa gauge),
%   inMotion, inBand, exchangeMs. frictionFit reads it. r holds the same
%   with the steps and the abort reason.
%
%   The run ends early, with the axis stopped, when the master reports a
%   fault, link loss or an unconfirmed mode, or when the pressure or the
%   torque passes its limit; r.abort says which. Ctrl-C stops the axis: the
%   cleanup sends one exchange with MotionEnable low and mode 0, and the
%   slave's own watchdog stops it within 300 ms if the script dies.

if nargin < 1 || isempty(label)
    label = 'run';
end
if nargin < 2 || isempty(set)
    set = 'unloaded';
end
if nargin < 3
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'modbus'), fullfile(here, '..', '..', 'master-src', 'logic'));
bcfg = modbusBenchConfig();
fcfg = frictionBenchConfig();
cfg = masterConfig();
map = modbusMap();

if iscell(set)
    setName = 'custom';
    path = set;
else
    setName = lower(set);
    path = {};
end
opts = withDefault(opts, 'speeds', fcfg.speeds);
opts = withDefault(opts, 'tSteady', fcfg.tSteady);
opts = withDefault(opts, 'repeats', fcfg.repeats);
opts = withDefault(opts, 'strokeMax', fcfg.gasStrokeMax);
opts = withDefault(opts, 'startPosition', NaN);
opts = withDefault(opts, 'target', fcfg.deliveryTarget);
opts = withDefault(opts, 'seconds', fcfg.deliverySeconds);
opts = withDefault(opts, 'torqueAbort', fcfg.torqueAbort);
opts = withDefault(opts, 'xStop', NaN);
opts = withDefault(opts, 'xMin', NaN);
opts = withDefault(opts, 'safetyModel', 'SafetyHold');
opts = withDefault(opts, 'saveDir', fullfile(here, 'captures'));
if ~isfield(opts, 'pMax')
    if isfield(fcfg.pMax, setName)
        opts.pMax = fcfg.pMax.(setName);
    else
        opts.pMax = fcfg.pMax.delivery;
    end
end
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

if isempty(path)
    path = setPath(setName, opts, fcfg);
end
if ~isnan(opts.startPosition)
    path = [{{'position', opts.startPosition, 1}} path];
end
steps = parsePath(path);
nSteps = numel(steps);

C = codes();

% --- Safety relay and the stop on exit --------------------------------------
clear masterStep masterStateMachine
safety = startSafety(opts.safetyModel, fullfile(here, '..', 'pressure'));
cleanup = onCleanup(@() shutdown(link, map, safety));

% --- Run state --------------------------------------------------------------
app = struct('op', C.OP_STOP, 'cmdSeq', 0, 'targetPosition', 0, ...
    'targetVelocity', 0, 'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0);
mbWrite = zeros(1, map.nWrite);
mbRead = zeros(1, map.nRead);
m = struct('statusCode', C.STATUS_LINK_WAIT, 'state', 0, 'position', 0, ...
    'velocity', 0, 'pressure', 0, 'torque', 0, 'inMotion', 0, 'inBand', 0, ...
    'faultCode', 0, 'linkOk', 0);

seq.idx = 0;            % current step, 0 before the first
seq.phase = 'wait';     % wait, next, enter, run, stop, finish
seq.tPhase = 0;
seq.abort = '';
seq.endStep = false;
seq.inBandSeen = false;

runLog = zeros(0, 12);
exchangeFailures = 0;

fprintf('frictionRun %s (%s): %d steps, pMax %g MPa, torque abort %g %%\n', ...
    label, setName, nSteps, opts.pMax, opts.torqueAbort);
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
    ref = modbusUnpack(mbWrite(map.hr.reference), mbWrite(map.hr.reference + 1));
    [mbWrite, m] = stepMaster(mbRead, app, map, cfg);

    % --- Limits the run enforces -------------------------------------------
    if isempty(seq.abort) && ~strcmp(seq.phase, 'finish')
        if m.linkOk && m.pressure > opts.pMax
            seq.abort = sprintf('pressure %.2f MPa above pMax %.2f MPa during step %d', ...
                m.pressure, opts.pMax, seq.idx);
        elseif m.linkOk && abs(m.torque) > opts.torqueAbort
            seq.abort = sprintf('torque %.1f %% above %.0f %% during step %d', ...
                m.torque, opts.torqueAbort, seq.idx);
        end
        if ~isempty(seq.abort)
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end
    end
    seq.endStep = (m.velocity > 0 && m.position >= opts.xStop) || ...
                  (m.velocity < 0 && m.position <= opts.xMin);

    % --- Path sequencing on the master's status ----------------------------
    [seq, app] = sequence(seq, app, m, steps, tNow, C, cfg);

    if seq.idx >= 1 && seq.idx <= nSteps
        mode = steps(seq.idx).mode;
    else
        mode = 0;
    end
    runLog(end + 1, :) = [tNow seq.idx m.statusCode mode ref m.position m.velocity ...
        m.torque m.pressure m.inMotion m.inBand exchangeMs];

    if strcmp(seq.phase, 'finish') && tNow - seq.tPhase > 0.2 && ...
            (m.statusCode == C.STATUS_IDLE || any(m.statusCode == C.abortStatus) || ...
             tNow - seq.tPhase > cfg.tStopMax + 1)
        break;
    end

    % --- Second master step of the cycle, registers held -------------------
    opts.wait(tCycle + cfg.dt);
    [mbWrite, m] = stepMaster(mbRead, app, map, cfg);
    k = k + 1;
end

% --- Report and file ----------------------------------------------------------
r.label = label;
r.set = setName;
r.steps = steps;
r.abort = seq.abort;
r.runLog = runLog;
r.columns = {'t', 'step', 'status', 'mode', 'ref', 'position', 'velocity', ...
    'torque', 'pressure', 'inMotion', 'inBand', 'exchangeMs'};
r.exchangeFailures = exchangeFailures;
if ~isempty(seq.abort)
    fprintf('\nRUN ABORTED: %s; master status %d, fault code %d\n', seq.abort, ...
        m.statusCode, m.faultCode);
end
if ~exist(opts.saveDir, 'dir')
    mkdir(opts.saveDir);
end
fname = fullfile(opts.saveDir, sprintf('%s_%s_%s.csv', label, setName, ...
    datestr(now, 'yyyymmdd_HHMMSS')));
fid = fopen(fname, 'w');
fprintf(fid, '%s\n', strjoin(r.columns, ','));
fprintf(fid, '%.3f,%d,%d,%d,%.4f,%.4f,%.4f,%.3f,%.4f,%d,%d,%.1f\n', runLog');
fclose(fid);
r.file = fname;

fprintf('\n%-4s %-10s %-12s %8s %10s %10s %10s %10s\n', 'step', 'type', 'target', ...
    'samples', 'mean v', 'mean T %', 'std T %', 'mean MPa');
for s = 1:nSteps
    sel = runLog(:, 2) == s & runLog(:, 3) == steps(s).runStatus;
    fprintf('%-4d %-10s %-12s %8d %10.3f %10.2f %10.2f %10.3f\n', s, steps(s).name, ...
        steps(s).targetText, sum(sel), mean(runLog(sel, 7)), mean(runLog(sel, 8)), ...
        std(runLog(sel, 8)), mean(runLog(sel, 9)));
end
fprintf('exchange failures %d, exchange p99 %.1f ms\n', exchangeFailures, ...
    prctileOf(runLog(:, 12), 99));
fprintf('saved %s\n', fname);
end


% ===========================================================================
function path = setPath(setName, opts, fcfg)
% The built-in sets. Each jog step lasts its steady time plus the slew to
% speed; the stop that follows it brings the axis to rest before the
% reverse step, so a pair returns to its start.
switch setName
    case {'unloaded', 'gas'}
        v = opts.speeds(:)';
        t = opts.tSteady(:)';
        if strcmp(setName, 'gas')
            t = min(t, opts.strokeMax ./ v);
        end
        t = t + v / fcfg.accel;
        path = {};
        for rep = 1:opts.repeats
            if mod(rep, 2) == 1
                order = numel(v):-1:1;
            else
                order = 1:numel(v);
            end
            for i = order
                path{end + 1} = {'jog',  v(i), t(i)}; %#ok<AGROW>
                path{end + 1} = {'jog', -v(i), t(i)}; %#ok<AGROW>
            end
        end
    case 'delivery'
        path = {{'pressure', opts.target, opts.seconds}};
    otherwise
        error('unknown set "%s" (unloaded, gas, delivery, or a cell array of steps)', setName);
end
end


function [mbWrite, m] = stepMaster(mbRead, app, map, cfg)
[mbWrite, ~, ~, m.position, m.velocity, pressureAtm, m.statusCode, ...
    m.faultCode, m.state, m.linkOk] = masterStep(mbRead, app.op, app.cmdSeq, ...
    app.targetPosition, app.targetVelocity, app.jogDirection, ...
    app.targetPressure, app.estop, app.mixer);
mbWrite = double(mbWrite(:)');
r = double(mbRead(:)');
m.pressure = (pressureAtm - cfg.atmAmbient) / cfg.atmPerMPa;     % MPa gauge, as the slave sends it
m.torque = modbusUnpack(r(map.ir.torque), r(map.ir.torque + 1));
m.inMotion = double(r(map.ir.inMotion) ~= 0);
m.inBand = double(r(map.ir.performanceStatus) ~= 0);
end


function C = codes()
% Operation and status codes of masterStateMachine, and the master state
% that marks the axis idle.
C.OP_STOP = 0; C.OP_POSITION = 2; C.OP_JOG = 3; C.OP_PRESSURE = 4;
C.STATUS_IDLE = 0; C.STATUS_POS_AT_TARGET = 3; C.STATUS_JOG = 4; C.STATUS_PRESSURE = 5;
C.STATUS_LINK_WAIT = 35;
C.abortStatus = [10 30 32 33 34];       % fault, no confirm, reset failed, stop timeout, link lost
C.ST_IDLE = 1;
end


function [seq, app] = sequence(seq, app, m, steps, tNow, C, cfg)
% Advance the path: command each step through the app inputs, wait for its
% running status, time it, then stop and wait for idle.
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
        seq.inBandSeen = false;
        fprintf('  step %d: %s %s\n', seq.idx, s.name, s.targetText);

    case 'enter'
        s = steps(seq.idx);
        if strcmp(s.name, 'idle')
            running = m.statusCode == C.STATUS_IDLE && m.state == C.ST_IDLE;
        else
            running = m.statusCode == s.runStatus;
        end
        if strcmp(s.name, 'position')
            limit = 10 + abs(s.target - m.position) / 0.5;   % arrival at >= 0.5 mm/s
        else
            limit = 10;
        end
        if running
            seq.phase = 'run';
            seq.tPhase = tNow;
        elseif inPhase > limit
            seq.abort = sprintf('step %d (%s) not running after %.0f s (status %d)', ...
                seq.idx, s.name, inPhase, m.statusCode);
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end

    case 'run'
        s = steps(seq.idx);
        if strcmp(s.name, 'pressure') && m.inBand && ~seq.inBandSeen
            seq.inBandSeen = true;
            fprintf('  step %d in band at %.1f s: open the outlet valve\n', seq.idx, tNow);
        end
        if inPhase >= s.seconds || seq.endStep
            if seq.endStep
                fprintf('  step %d ended at %.1f s by the position limit (%.2f mm)\n', ...
                    seq.idx, tNow, m.position);
            end
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


function steps = parsePath(path)
% Steps with the slave mode code and the master status that marks them
% running.
steps = struct('name', {}, 'target', {}, 'seconds', {}, 'targetText', {}, ...
    'mode', {}, 'runStatus', {});
for k = 1:numel(path)
    p = path{k};
    name = lower(p{1});
    switch name
        case 'idle'
            s = struct('name', name, 'target', 0, 'seconds', p{2}, 'targetText', '', ...
                'mode', 0, 'runStatus', 0);
        case 'jog'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g mm/s', p{2}), 'mode', 3, 'runStatus', 4);
        case 'position'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g mm', p{2}), 'mode', 2, 'runStatus', 3);
        case 'pressure'
            s = struct('name', name, 'target', p{2}, 'seconds', p{3}, ...
                'targetText', sprintf('%g MPa', p{2}), 'mode', 7, 'runStatus', 5);
        otherwise
            error('unknown step type "%s" (idle, jog, position, pressure)', p{1});
    end
    steps(end + 1) = s; %#ok<AGROW>
end
end


function safety = startSafety(model, dir)
% Start the model that drives the safety relay. Returns '' when none.
safety = '';
if isempty(model)
    return;
end
file = fullfile(dir, [model '.slx']);
if ~exist(file, 'file')
    error(['%s not found: run bench/pressure/buildSafetyHold once, or pass ' ...
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


% ===========================================================================
% Raw Modbus TCP over tcpclient: FC04 read, FC16 write. The controller
% answers in about 4 ms; the modbus object adds some 7 ms per transaction.
function link = rawLink(bcfg)
t = tcpclient(bcfg.address, bcfg.port, 'Timeout', 0.5);
uid = uint8(bcfg.serverId);
link.t = t;
link.read = @(start, n) rawTxn(t, uid, [4, u16(start - 1), u16(n)], 4, n);
link.write = @(start, vals) rawTxn(t, uid, ...
    [16, u16(start - 1), u16(numel(vals)), 2 * numel(vals), u16s(vals)], 16, 0);
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
