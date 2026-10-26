function r = charRun(label, path, opts)
%CHARRUN Drive the axis along a path of steps and log the protocol feedback
%   for the pressure loop characterisation (bench/characterisation/README.md).
%   The script is the master for the run: one Modbus TCP connection carries
%   the protocol exchange (HR1-6 out, IR1-20 back, every 50 ms, sequenced by
%   masterStep as in the shell) and, between exchanges, the registers of the
%   optional axis probe (slave/PRG_AxisProbe.st), which captures the
%   velocity command and feedback at the 2 ms scan. The shell model must not
%   be running: a second client stalls the shell's exchange and the slave
%   raises FAULT_COMM while motion is commanded. The safety relay is held by
%   bench/pressure/SafetyHold.slx (buildSafetyHold), started and stopped here.
%
%   r = charRun(label, path)
%   r = charRun(label, path, opts)
%
%   path  cell array of steps, run in order:
%     {'idle', seconds}                 no mode commanded
%     {'jog', mm_per_s, seconds}        velocity mode at a constant command
%     {'ramp', mm_per_s, MPa, maxSec}   velocity mode until the pressure
%                                       crosses MPa (upward for a positive
%                                       command, downward for a negative one)
%                                       or maxSec elapse
%     {'dwell', seconds}                velocity mode, zero command
%     {'velstep', [v1 v2 ...], [s1 s2 ...]}
%                                       velocity mode; each command held for
%                                       its seconds, the axis probe armed
%                                       before each change
%     {'position', mm, seconds}         position mode: move, then hold
%     {'hold', seconds}                 position mode at the position where
%                                       the step starts
%     {'pressure', MPa, seconds}        pressure mode (target in MPa gauge)
%   Consecutive steps of the same mode (jog, ramp, dwell, velstep; position,
%   hold; pressure) run in one session: the command changes without a stop
%   and a handshake in between. A step of another mode stops the axis first.
%
%   Guards, every master step: the run stops (MotionEnable low, mode 0) when
%   the pressure exceeds opts.pressureStop, the position leaves
%   [opts.posMin, opts.posStop], or the master reports a fault, link loss or
%   an unconfirmed mode. r.abort says which. Ctrl-C stops the axis the same
%   way, and the slave's own watchdog stops it within 300 ms if the script
%   dies.
%
%   opts.pressureStop  MPa gauge (charConfig.pressureStop)
%   opts.posStop, opts.posMin
%                      mm (charConfig: inside the slave's soft limits)
%   opts.probe         true to arm and read the axis probe (default false)
%   opts.safetyModel   model that holds the safety relay ('SafetyHold';
%                      '' when the relay is held another way)
%   opts.saveDir       where the CSV files go (default captures/)
%   opts.link, opts.now, opts.wait
%                      transport and clock, replaced by the simulator test
%
%   Files, in opts.saveDir:
%     <label>_run_<timestamp>.csv    t, position, velocity, torque, MPa,
%                                    reference, modeCode, motionEnable,
%                                    status, inBand, step, exchangeMs
%     <label>_<n>_<stepname>_<timestamp>.csv
%                                    axis probe capture n: t (0 at the
%                                    scan the command changed), velocity,
%                                    command, torque, MPa, every 2 ms
%   r holds the same (r.runLog, r.captures), the step table, r.abort and
%   the file names. charBulkModulus, charOrificeFlow and charDriveLag read
%   these files.

if nargin < 1 || isempty(label)
    label = 'run';
end
if nargin < 2 || isempty(path)
    error('charRun needs a path of steps; see the help text');
end
if nargin < 3
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'), fullfile(here, '..', 'modbus'));
ccfg = charConfig();
cfg = masterConfig();
map = modbusMap();
pr = ccfg.probe;

opts = withDefault(opts, 'pressureStop', ccfg.pressureStop);
opts = withDefault(opts, 'posStop', ccfg.posStop);
opts = withDefault(opts, 'posMin', ccfg.posMin);
opts = withDefault(opts, 'probe', false);
opts = withDefault(opts, 'safetyModel', 'SafetyHold');
opts = withDefault(opts, 'saveDir', fullfile(here, 'captures'));
if ~isfield(opts, 'now')
    t0 = tic;
    opts.now = @() toc(t0);
end
if ~isfield(opts, 'wait')
    opts.wait = @(tTarget) waitUntil(opts.now, tTarget);
end
if ~isfield(opts, 'link')
    opts.link = charLink();
end
link = opts.link;
assert(opts.pressureStop <= ccfg.pressureCeiling, ...
    'pressureStop %.1f MPa is above the %.1f MPa ceiling', opts.pressureStop, ccfg.pressureCeiling);

C = codes();
steps = parsePath(path);
nSteps = numel(steps);

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
    'velocity', 0, 'pressure', 0, 'torque', 0, 'inBand', 0, 'faultCode', 0, 'linkOk', 0);

seq.idx = 0;            % current step, 0 before the first
seq.phase = 'wait';     % wait, next, enter, run, stop, finish
seq.tPhase = 0;
seq.sub = 0;            % velstep sub-step
seq.tSub = 0;
seq.armPending = false; % reference change waits one cycle for the probe arm
seq.abort = '';

probe.enabled = opts.probe;
probe.next = 'poll';
probe.armRequest = false;
probe.armValue = 0;
probe.pending = false;
probe.tArm = 0;
probe.chunk = 0;
probe.regs = zeros(1, 4 * pr.nSamples);
probe.failures = 0;
probe.step = 0;

captures = {};
runLog = zeros(0, 12);
exchangeFailures = 0;

fprintf('charRun %s: %d steps, guards %.1f MPa, %.1f..%.1f mm, probe %s\n', label, nSteps, ...
    opts.pressureStop, opts.posMin, opts.posStop, mat2str(opts.probe));
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
    [mbWrite, m] = stepMaster(mbRead, app, map, cfg);

    % --- Guards and path sequencing on the master's status ------------------
    [seq, app] = guard(seq, app, m, tNow, opts, C);
    [seq, app] = sequence(seq, app, m, steps, tNow, probe, C, cfg);
    if seq.armRequest
        probe.armRequest = true;
        probe.step = seq.idx;
        seq.armRequest = false;
    end
    runLog(end + 1, :) = [tNow m.position m.velocity m.torque m.pressure ...
        modbusUnpack(mbWrite(map.hr.reference), mbWrite(map.hr.reference + 1)) ...
        mbWrite(map.hr.modeCode) mbWrite(map.hr.motionEnable) m.statusCode m.inBand ...
        seq.idx exchangeMs];

    if strcmp(seq.phase, 'finish') && tNow - seq.tPhase > 0.2 && ...
            (m.statusCode == C.STATUS_IDLE || any(m.statusCode == C.abortStatus) || ...
             tNow - seq.tPhase > cfg.tStopMax + 1)
        break;
    end

    % --- One probe transaction in the gap ----------------------------------
    if probe.enabled
        [probe, captures] = probeSlot(probe, link, pr, captures, tNow, ccfg.scanTime);
    end

    % --- Second master step of the cycle, registers held -------------------
    opts.wait(tCycle + cfg.dt);
    [mbWrite, m] = stepMaster(mbRead, app, map, cfg);
    k = k + 1;
end

% --- Files and summary --------------------------------------------------------
r.label = label;
r.steps = steps;
r.abort = seq.abort;
r.runLog = runLog;
r.captures = captures;
r.exchangeFailures = exchangeFailures;
r.probeFailures = probe.failures;
r.files = {};
if ~isempty(seq.abort)
    fprintf('\nRUN STOPPED: %s; master status %d, fault code %d\n', seq.abort, ...
        m.statusCode, m.faultCode);
end
if ~exist(opts.saveDir, 'dir')
    mkdir(opts.saveDir);
end
stamp = datestr(now, 'yyyymmdd_HHMMSS');
for c = 1:numel(captures)
    cap = captures{c};
    fname = fullfile(opts.saveDir, sprintf('%s_%d_%s_%s.csv', label, c, ...
        steps(max(cap.step, 1)).name, stamp));
    fid = fopen(fname, 'w');
    fprintf(fid, 't,velocity,command,torque,MPa\n');
    fprintf(fid, '%.4f,%.4f,%.4f,%.2f,%.4f\n', [cap.t(:) cap.velocity(:) cap.command(:) ...
        cap.torque(:) cap.pressure(:)]');
    fclose(fid);
    cap.file = fname;
    captures{c} = cap;
    r.files{end + 1} = fname;
end
r.captures = captures;
fname = fullfile(opts.saveDir, sprintf('%s_run_%s.csv', label, stamp));
fid = fopen(fname, 'w');
fprintf(fid, 't,position,velocity,torque,MPa,reference,modeCode,motionEnable,status,inBand,step,exchangeMs\n');
fprintf(fid, '%.3f,%.4f,%.4f,%.2f,%.4f,%.4f,%d,%d,%d,%d,%d,%.1f\n', runLog');
fclose(fid);
r.runFile = fname;

fprintf('\n%-4s %-9s %-18s %8s %9s %9s %9s %9s\n', 'step', 'type', 'target', 'seconds', ...
    'x start', 'x end', 'P start', 'P end');
for s = 1:nSteps
    rows = runLog(runLog(:, 11) == s, :);
    if isempty(rows)
        fprintf('%-4d %-9s %-18s %8s\n', s, steps(s).name, steps(s).targetText, 'not run');
        continue;
    end
    fprintf('%-4d %-9s %-18s %8.1f %6.3f mm %6.3f mm %6.3f MPa %6.3f MPa\n', s, steps(s).name, ...
        steps(s).targetText, rows(end, 1) - rows(1, 1), rows(1, 2), rows(end, 2), ...
        rows(1, 5), rows(end, 5));
end
fprintf('%d captures, exchange failures %d, probe failures %d, exchange p99 %.1f ms\n', ...
    numel(captures), exchangeFailures, probe.failures, prctileOf(runLog(:, 12), 99));
fprintf('saved %s\n', fname);
end


% ===========================================================================
function [mbWrite, m] = stepMaster(mbRead, app, map, cfg)
[mbWrite, ~, ~, m.position, m.velocity, pAtm, m.statusCode, ...
    m.faultCode, m.state, m.linkOk] = masterStep(mbRead, app.op, app.cmdSeq, ...
    app.targetPosition, app.targetVelocity, app.jogDirection, ...
    app.targetPressure, app.estop, app.mixer);
mbWrite = double(mbWrite(:)');
m.pressure = (pAtm - cfg.atmAmbient) / cfg.atmPerMPa;      % MPa gauge, as the slave reports it
r = double(mbRead(:)');
m.torque = modbusUnpack(r(map.ir.torque), r(map.ir.torque + 1));
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


function [seq, app] = guard(seq, app, m, tNow, opts, C)
% Stop the run when the pressure or the position leaves the envelope. The
% pressure guard applies throughout; the position guard only while a motion
% step is active, so a run can start from wherever the axis was left.
if strcmp(seq.phase, 'finish') || seq.idx == 0
    return;
end
why = '';
if m.pressure > opts.pressureStop
    why = sprintf('pressure guard: %.2f MPa above %.2f MPa', m.pressure, opts.pressureStop);
elseif any(strcmp(seq.phase, {'enter', 'run'})) && m.linkOk && ...
        (m.position > opts.posStop || m.position < opts.posMin)
    why = sprintf('position guard: %.2f mm outside %.1f..%.1f mm', m.position, opts.posMin, opts.posStop);
end
if ~isempty(why)
    seq.abort = sprintf('%s during step %d', why, seq.idx);
    app.op = C.OP_STOP;
    seq.phase = 'finish';
    seq.tPhase = tNow;
end
end


function [seq, app] = sequence(seq, app, m, steps, tNow, probe, C, cfg)
% Advance the path: command each step through the app inputs, wait for its
% running status, time it or watch its end condition, then either change
% the command for a step of the same mode or stop and wait for idle.
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
        chained = seq.idx >= 1 && seq.idx < numel(steps) && ...
            steps(seq.idx).op == steps(seq.idx + 1).op && steps(seq.idx).op ~= C.OP_STOP;
        seq.idx = seq.idx + 1;
        if seq.idx > numel(steps)
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
            return;
        end
        s = steps(seq.idx);
        seq.sub = 1;
        if ~chained
            app.op = s.op;
            app.cmdSeq = app.cmdSeq + 1;
        end
        [app, seq] = applyCommand(app, seq, s, 1, m, probe, cfg);
        seq.phase = 'enter';
        seq.tPhase = tNow;
        fprintf('  step %d: %s %s%s\n', seq.idx, s.name, s.targetText, chainText(chained));

    case 'enter'
        s = steps(seq.idx);
        if seq.armPending
            % The probe arm went out in the last gap; the command follows now.
            [app, seq] = applyCommand(app, seq, s, seq.sub, m, probe, cfg, true);
        end
        running = isRunning(s, m, C);
        if s.op == C.OP_POSITION
            limit = 10 + abs(app.targetPosition - m.position) / 0.5;   % arrival at >= 0.5 mm/s
        else
            limit = 10;
        end
        if running && ~seq.armPending
            seq.phase = 'run';
            seq.tPhase = tNow;
            seq.tSub = tNow;
        elseif inPhase > limit
            seq.abort = sprintf('step %d (%s) not running after %.0f s (status %d)', ...
                seq.idx, s.name, inPhase, m.statusCode);
            app.op = C.OP_STOP;
            seq.phase = 'finish';
            seq.tPhase = tNow;
        end

    case 'run'
        s = steps(seq.idx);
        if seq.armPending
            [app, seq] = applyCommand(app, seq, s, seq.sub, m, probe, cfg, true);
        end
        done = false;
        switch s.name
            case 'ramp'
                crossed = (s.target > 0 && m.pressure >= s.pressureEnd) || ...
                          (s.target < 0 && m.pressure <= s.pressureEnd);
                done = crossed || inPhase >= s.seconds;
                if crossed
                    fprintf('  step %d: %.3f MPa reached at %.3f mm after %.1f s\n', ...
                        seq.idx, m.pressure, m.position, inPhase);
                end
            case 'velstep'
                if ~seq.armPending && tNow - seq.tSub >= s.seconds(seq.sub)
                    if seq.sub < numel(s.target)
                        seq.sub = seq.sub + 1;
                        seq.tSub = tNow;
                        [app, seq] = applyCommand(app, seq, s, seq.sub, m, probe, cfg);
                    else
                        done = true;
                    end
                end
            otherwise
                done = inPhase >= s.seconds;
        end
        if done
            nextSame = seq.idx < numel(steps) && steps(seq.idx + 1).op == s.op && s.op ~= C.OP_STOP;
            if nextSame
                seq.phase = 'next';
            else
                app.op = C.OP_STOP;
                seq.phase = 'stop';
            end
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


function [app, seq] = applyCommand(app, seq, s, sub, m, probe, cfg, armed)
% Put a step's command into the app inputs. In a velocity session with the
% probe enabled, the first call only arms the probe (seq.armPending); the
% command is applied on the next call, one exchange later, so the capture
% is armed before the slave sees the change.
if nargin < 8
    armed = false;
end
seq.armPending = false;
switch s.name
    case 'idle'
        app.op = 0;
    case {'jog', 'ramp', 'dwell', 'velstep'}
        if strcmp(s.name, 'velstep')
            v = s.target(sub);
        elseif strcmp(s.name, 'dwell')
            v = 0;
        else
            v = s.target;
        end
        if probe.enabled && ~armed && ~probe.pending && v ~= app.targetVelocity * app.jogDirection
            seq.armRequest = true;
            seq.armPending = true;
            return;
        end
        app.targetVelocity = abs(v);
        app.jogDirection = sign(v);
        if v == 0
            app.jogDirection = 0;
        end
    case 'position'
        app.targetPosition = s.target;
    case 'hold'
        app.targetPosition = m.position;
    case 'pressure'
        app.targetPressure = s.target * cfg.atmPerMPa + cfg.atmAmbient;   % the app's atm absolute
end
end


function running = isRunning(s, m, C)
switch s.op
    case C.OP_STOP,     running = m.statusCode == C.STATUS_IDLE && m.state == C.ST_IDLE;
    case C.OP_JOG,      running = m.statusCode == C.STATUS_JOG;
    case C.OP_POSITION, running = m.statusCode == C.STATUS_POS_AT_TARGET;
    otherwise,          running = m.statusCode == C.STATUS_PRESSURE;
end
end


function txt = chainText(chained)
if chained
    txt = ' (same mode, no stop)';
else
    txt = '';
end
end


function [probe, captures] = probeSlot(probe, link, pr, captures, tNow, Ts)
% One probe transaction per cycle: arm when asked, otherwise poll the
% state and fetch the rings in 125-register chunks once a capture is ready.
if probe.pending && tNow - probe.tArm > pr.timeout
    fprintf('  probe armed at %.2f s not ready after %g s: skipped\n', probe.tArm, pr.timeout);
    probe.pending = false;
    probe.next = 'poll';
    probe.chunk = 0;
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
            s = link.read(pr.irState, 4);
            if probe.pending && s(1) == 3 && s(2) == probe.armValue
                probe.next = 'fetch';
                probe.chunk = 0;
                probe.trigIdx = s(3);
                probe.count = s(4);
            end
        case 'fetch'
            nTotal = 4 * pr.nSamples;
            start = probe.chunk * 125;
            n = min(125, nTotal - start);
            probe.regs(start + 1:start + n) = link.read(pr.irSamples + start, n);
            probe.chunk = probe.chunk + 1;
            if probe.chunk * 125 >= nTotal
                captures{end + 1} = unroll(probe, pr, Ts);
                probe.pending = false;
                probe.next = 'poll';
                fprintf('  capture %d taken (armed at %.2f s, step %d)\n', numel(captures), ...
                    probe.tArm, probe.step);
            end
    end
catch err
    probe.failures = probe.failures + 1;
    if probe.failures <= 3
        fprintf('  probe transaction failed at %.2f s: %s\n', tNow, err.message);
    end
end
end


function cap = unroll(probe, pr, Ts)
% Order the four rings from the oldest sample, with t = 0 at the trigger
% scan (the first scan in which the changed command was visible).
N = pr.nSamples;
if probe.count ~= N
    warning('charRun:ring', 'probe ring length %d, expected %d', probe.count, N);
end
order = mod(probe.trigIdx - pr.preTrigger + (0:N - 1), N) + 1;
raw = reshape(probe.regs, N, 4);
raw(raw > 32767) = raw(raw > 32767) - 65536;          % INT registers
raw = raw(order, :);
cap.step = probe.step;
cap.tArm = probe.tArm;
cap.t = ((0:N - 1)' - pr.preTrigger) * Ts;
cap.velocity = raw(:, 1) / pr.scale(1);
cap.command  = raw(:, 2) / pr.scale(2);
cap.torque   = raw(:, 3) / pr.scale(3);
cap.pressure = raw(:, 4) / pr.scale(4);
end


function steps = parsePath(path)
C = codes();
steps = struct('name', {}, 'op', {}, 'target', {}, 'seconds', {}, 'pressureEnd', {}, 'targetText', {});
for k = 1:numel(path)
    p = path{k};
    name = lower(p{1});
    s = struct('name', name, 'op', C.OP_STOP, 'target', 0, 'seconds', 0, 'pressureEnd', NaN, 'targetText', '');
    switch name
        case 'idle'
            s.seconds = p{2};
        case 'jog'
            s.op = C.OP_JOG; s.target = p{2}; s.seconds = p{3};
            s.targetText = sprintf('%g mm/s', p{2});
        case 'ramp'
            s.op = C.OP_JOG; s.target = p{2}; s.pressureEnd = p{3}; s.seconds = p{4};
            assert(p{2} ~= 0, 'ramp needs a nonzero velocity');
            s.targetText = sprintf('%g mm/s to %g MPa', p{2}, p{3});
        case 'dwell'
            s.op = C.OP_JOG; s.seconds = p{2};
            s.targetText = '0 mm/s';
        case 'velstep'
            s.op = C.OP_JOG; s.target = p{2}(:)'; s.seconds = p{3}(:)';
            assert(numel(s.target) == numel(s.seconds), 'velstep needs one duration per velocity');
            s.targetText = mat2str(s.target);
        case 'position'
            s.op = C.OP_POSITION; s.target = p{2}; s.seconds = p{3};
            s.targetText = sprintf('%g mm', p{2});
        case 'hold'
            s.op = C.OP_POSITION; s.seconds = p{2};
            s.targetText = 'at entry position';
        case 'pressure'
            s.op = C.OP_PRESSURE; s.target = p{2}; s.seconds = p{3};
            s.targetText = sprintf('%g MPa', p{2});
        otherwise
            error('unknown step type "%s" (idle, jog, ramp, dwell, velstep, position, hold, pressure)', p{1});
    end
    steps(end + 1) = s;
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
