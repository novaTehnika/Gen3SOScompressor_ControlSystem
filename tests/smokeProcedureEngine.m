function smokeProcedureEngine()
%SMOKEPROCEDUREENGINE Run the procedure engine (procNew/procAction/procStep/
%   procView, opTrack) against a stand-in for the model's status, and the
%   seed procedure through procNew.
%
%   The stand-in reproduces the status codes the operator app sees for each
%   operation, with injectable failures; it is not the master state machine.
%   The host loop mirrors the app: read the model, step the engine, execute
%   its requests.
%
%   Run from the repository root:  addpath master-src/logic tests; smokeProcedureEngine
%   Depends on: master-src/logic master-src/procedure

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'master-src', 'procedure'));

testOpTrack();
testSeedAndBadData();
for dt = [0.05 0.2]
    testHappyPath(dt);
    testPause(dt);
    testAbort(dt);
end
testFault(0.05);
testStopsAndLoss(0.05);
testFunctionTarget(0.05);
testSpeed(0.05);
testPressureGuards(0.05);
testAlsoWhen(0.05);
testSeedWalk(0.1);
testEarlyEnd(0.05);
testBackgroundLost(0.05);
testTimeout(0.05);
testRepeat(0.05);
testWrongState(0.05);
testPurity();
testView();
testPressureView();
end

% ===== Scenarios =====

function testOpTrack()
cfg = masterConfig();
[c, e] = opTrack(4, false, 5, 0, cfg);
assert(c && ~e);
[c, e] = opTrack(4, true, 5, 3, cfg);
assert(c && ~e);
[c, e] = opTrack(4, true, 0, 0.1, cfg);
assert(c && e);
% Position is confirmed by moving or at target; the others by their own.
assert(opTrack(2, false, 3, 0, cfg));
assert(opTrack(1, false, 1, 0, cfg));
assert(opTrack(6, false, 8, 0, cfg));
assert(opTrack(3, false, 4, 0, cfg));
assert(~opTrack(6, false, 1, 0, cfg));
% Starting and stopping are waited out, any other status ends it after
% opConfirmWait.
[~, e] = opTrack(2, false, 7, 5, cfg);   assert(~e);
[~, e] = opTrack(2, false, 6, 5, cfg);   assert(~e);
[~, e] = opTrack(2, false, 0, 0.2, cfg); assert(~e);
[~, e] = opTrack(2, false, 0, cfg.opConfirmWait, cfg); assert(e);
[~, e] = opTrack(2, false, 30, 1, cfg);  assert(e);
% Stop never ends.
[c, e] = opTrack(0, false, 30, 10, cfg); assert(~c && ~e);
fprintf('ok  opTrack rule\n');
end

function testSeedAndBadData()
cfg = masterConfig();
proc = sopProcedure();
p = procNew(proc, cfg);
assert(strcmp(p.state, 'idle'));
for k = 1:numel(p.def.steps)
    assert(~isempty(p.def.steps(k).title));
end
assert(numel(p.def.steps) == 21 && numel(p.def.params) == 6);
% The oxygen fill target on the defaults: the saline plus the oxygen at
% the loading pressure (ideal gas at room temperature).
st = p.def.steps(13);
assert(strcmp(st.op, 'position') && strcmp(st.unit, 'mL'));
want = 343 + 343 * (1 / 7.8) * 293.15 / 273.15;
assert(abs(st.target(p.values, p.cfg) - want) < 1e-9);
% A charge the chamber cannot hold is reported by the target.
v = p.values;
v.salineVolume = 900;
v.oxygenRatio = 4;
v.oxygenLoadPressure = 2;
try
    st.target(v, p.cfg);
    error('smoke:noError', 'the oxygen fill target accepted too large a charge');
catch err
    assert(~isempty(strfind(err.message, 'more than the chamber holds')), err.message);
end
fprintf('ok  seed procedure validates (%d steps, %d parameters)\n', ...
    numel(p.def.steps), numel(p.def.params));

base = mk({ ...
    struct('id', 'a', 'type', 'confirm', 'title', 'A', 'checks', {{'x'}}), ...
    struct('type', 'machine', 'title', 'P', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true), ...
    struct('type', 'wait', 'title', 'W', 'until', 'settled'), ...
    struct('type', 'machine', 'title', 'M', 'op', 'position', 'target', 1)});
procNew(base, cfg);   % the base procedure is valid
procNew(setStep(base, 4, 'speed', 0.5), cfg);
procNew(setStep(base, 4, 'pressureMax', 5), cfg);
procNew(setStep(base, 1, 'alsoWhen', 'endOfTravel'), cfg);
procNew(setStep(base, 2, 'alsoWhen', 'endOfTravel'), cfg);
procNew(setStep(base, 4, 'target', @(v, c) c.deadVolume + v.vol), cfg);
bad = {};
bad{end+1} = setStep(base, 1, 'type', 'bogus');
bad{end+1} = setStep(base, 4, 'op', 'bogus');
bad{end+1} = setStep(base, 4, 'unit', 'cm');
bad{end+1} = setStep(base, 4, 'until', 'bogus');
bad{end+1} = setStep(base, 1, 'title', '');
bad{end+1} = setStep(base, 4, 'target', []);
bad{end+1} = setStep(base, 2, 'target', 'noSuchKey');
bad{end+1} = setStep(base, 2, 'id', 'a');
bad{end+1} = setStep(base, 1, 'repeatFrom', 'a');
bad{end+1} = setStep(base, 1, 'repeatFrom', 'later');
bad{end+1} = setStep(setStep(base, 3, 'id', 'later'), 1, 'repeatFrom', 'later');
bad{end+1} = setStep(base, 1, 'checks', {'1', '2', '3', '4', '5', '6', '7'});
bad{end+1} = setStep(base, 1, 'title', 'uses {noSuchKey}');
bad{end+1} = setStep(base, 2, 'keep', false);   % the settled wait loses its hold
bad{end+1} = setStep(setStep(base, 2, 'op', 'home'), 2, 'target', []);   % keep on home
bad{end+1} = setStep(base, 4, 'keep', true);    % keep on a move is fine, below
bad{end} = setStep(setStep(setStep(base, 4, 'op', 'stop'), 4, 'target', []), ...
    4, 'keep', true);
bad{end+1} = setStep(base, 4, 'target', @(v, c) error('smoke:fn', 'no'));
bad{end+1} = setStep(base, 4, 'target', @(v, c) [1 2]);
bad{end+1} = setStep(base, 4, 'target', {1});
bad{end+1} = setStep(base, 2, 'speed', 1);          % speed on a pressure step
bad{end+1} = setStep(base, 4, 'speed', 0);
bad{end+1} = setStep(base, 4, 'speed', -1);
bad{end+1} = setStep(base, 4, 'speed', [1 2]);
bad{end+1} = setStep(base, 4, 'pressureMax', 0);
bad{end+1} = setStep(base, 1, 'pressureMax', 5);    % pressureMax on a confirm
bad{end+1} = setStep(base, 4, 'alsoWhen', 'bogus');
bad{end+1} = setStep(base, 4, 'alsoWhen', 'endOfTravel');   % on a position step
bad{end+1} = setStep(base, 3, 'alsoWhen', 'endOfTravel');   % on a wait
bad{end+1} = setParam(base, 2, 'kind', 'bogus');
bad{end+1} = setParam(base, 1, 'kind', 'pressure');         % with a unit
many = base;
for i = 1:9
    many.params{end+1} = struct('key', sprintf('q%d', i), 'value', 1);
end
bad{end+1} = many;
sink = base;
sink.params{1}.sink = 'Bogus';
bad{end+1} = sink;
for i = 1:numel(bad)
    ok = false;
    try
        procNew(bad{i}, cfg);
    catch err
        ok = strcmp(err.identifier, 'procNew:bad');
        if ~ok
            fprintf('case %d raised %s: %s\n', i, err.identifier, err.message);
        end
    end
    assert(ok, 'bad procedure %d was accepted or raised the wrong error', i);
end
fprintf('ok  %d invalid procedures raise procNew:bad\n', numel(bad));
end

function testHappyPath(dt)
proc = mk({ ...
    struct('type', 'param', 'title', 'Set', 'section', 'A', ...
        'params', {{'vol', 'pTarget'}}), ...
    struct('type', 'confirm', 'title', 'Check', 'checks', {{'a', 'b'}}), ...
    struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 100), ...
    struct('type', 'wait', 'title', 'Wait', 'duration', 1), ...
    struct('type', 'machine', 'title', 'Home', 'op', 'home'), ...
    struct('type', 'machine', 'title', 'Pressurize', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true, 'timerStart', true, 'mixer', 'mix'), ...
    struct('type', 'wait', 'title', 'Settle', 'until', 'settled', 'duration', 1), ...
    struct('type', 'confirm', 'title', 'Deliver', 'zeroVolume', true), ...
    struct('type', 'machine', 'title', 'End', 'op', 'stop', ...
        'timerStop', true, 'mixer', 'off')});
h = newHost(proc, dt);
h = act(h, 'start');
h = tick(h);
assert(strcmp(h.p.state, 'waitOperator') && h.p.k == 1);
% A value out of range is refused, then fixed.
[~, ok] = procAction(h.p, 'setParam', 'vol', 50);
assert(~ok);
h = act(h, 'setParam', 'vol', 12);
h = act(h, 'confirm');
h = tick(h);
assert(isequal(h.params{end}, struct('SalineVolume', 12)));
assert(h.p.k == 2);
% Confirm is refused until both checks are ticked.
h = refuse(h, 'confirm');
h = act(h, 'check', 1, true);
h = refuse(h, 'confirm');
h = act(h, 'check', 2, true);
h = act(h, 'confirm');
h = tick(h);
h = runUntil(h, @(h) h.p.k == 8 && strcmp(h.p.state, 'waitOperator'), 40);
assert(isequal(h.events(:, 2)', [2 -1 1 4]), 'no stop after the pressure step');
assert(h.p.timer.running);
assert(h.zero == 1);
% The piston moves toward the end of travel by 5 mm.
h.m.d = h.m.d - 5;
h = tick(h);
assert(abs(h.p.volumeDischarged - 5 * mLPerMm(h.cfg)) < 1e-9);
h = act(h, 'confirm');
h = tick(h);
assert(~h.p.timer.running);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 10);
assert(strcmp(h.p.outcome, 'completed'));
assert(isequal(h.events(:, 2)', [2 -1 1 4 -1]));
assert(isequal(h.mixer(:, 2)', [1 0]));
assert(abs(h.p.timer.elapsed - (h.tStop - h.tStart)) <= dt + 1e-9);
assert(h.p.timer.elapsed > 1);
fprintf('ok  happy path through every step type (dt %.2f)\n', dt);
end

function testPause(dt)
proc = mk({ ...
    struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 100), ...
    struct('type', 'wait', 'title', 'Wait', 'duration', 1), ...
    struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true), ...
    struct('type', 'confirm', 'title', 'Operator'), ...
    struct('type', 'machine', 'title', 'End', 'op', 'stop')});
h = newHost(proc, dt);
h = act(h, 'start');
h = runFor(h, 1.0);
assert(strcmp(h.p.phase, 'track'));
n = size(h.events, 1);
h = act(h, 'pause');
h = tick(h);
assert(strcmp(h.p.state, 'paused'));
assert(size(h.events, 1) == n + 1 && h.events(end, 2) == -1);
h = runFor(h, 0.6);
assert(strcmp(h.p.state, 'paused') && h.p.k == 1);
h = act(h, 'resume');
h = runUntil(h, @(h) h.p.k == 2, 30);
assert(sum(h.events(:, 2) == 2) == 2, 'resume re-issues the move');
% A timed wait freezes while paused.
h = runFor(h, 0.4);
h = act(h, 'pause');
h = tick(h);
s1 = h.p.stepT;
h = runFor(h, 1.0);
assert(h.p.stepT == s1);
h = act(h, 'resume');
h = tick(h);
tResume = h.t;
h = runUntil(h, @(h) h.p.k == 3, 5);
assert(abs((h.t - tResume) - (1 - s1)) <= 2 * dt + 1e-9);
% Pause in an operator step keeps the pressure hold running.
h = runUntil(h, @(h) h.p.k == 4, 10);
n = size(h.events, 1);
h = act(h, 'pause');
h = tick(h);
assert(strcmp(h.p.state, 'paused') && size(h.events, 1) == n);
h = runFor(h, 0.5);
assert(h.m.status == 5, 'the pressure hold keeps running');
h = act(h, 'resume');
h = tick(h);
assert(strcmp(h.p.state, 'waitOperator') && h.p.k == 4);
fprintf('ok  pause and resume in a move, a wait and an operator step (dt %.2f)\n', dt);
end

function testAbort(dt)
cfg = masterConfig();
move = struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 100);
proc = mk({struct('type', 'confirm', 'title', 'Operator'), move});
procMove = mk({move});

% From a machine step in flight.
h = newHost(procMove, dt);
h = act(h, 'start');
h = runFor(h, 0.5);
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'aborting'));
assert(sum(h.events(:, 2) == -1) == 1);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted') && sum(h.events(:, 2) == -1) == 1);
assert(h.m.status == 0);

% From an operator step.
h = newHost(proc, dt);
h = act(h, 'start');
h = tick(h);
assert(strcmp(h.p.state, 'waitOperator'));
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'aborting') && sum(h.events(:, 2) == -1) == 1);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted'));

% From pause.
h = newHost(procMove, dt);
h = act(h, 'start');
h = runFor(h, 0.5);
h = act(h, 'pause');
h = tick(h);
n = sum(h.events(:, 2) == -1);
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'aborting') && sum(h.events(:, 2) == -1) == n + 1);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted'));

% The model never reports stopped.
h = newHost(procMove, dt);
h = act(h, 'start');
h = runFor(h, 0.5);
h.m.inject = 'stuck';
h = act(h, 'abort');
h = tick(h);
t0 = h.t;
h = runFor(h, cfg.tStopMax);
assert(strcmp(h.p.state, 'aborting'));
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 2);
assert(strcmp(h.p.outcome, 'aborted') && strcmp(h.p.reason, 'Stop not confirmed'));
assert(abs((h.t - t0) - (cfg.tStopMax + 0.5)) <= dt + 1e-9);

% The model stops while aborting.
h = newHost(procMove, dt);
h = act(h, 'start');
h = runFor(h, 0.5);
h = act(h, 'abort');
h = tick(h);
h.m.connected = false;
h = tick(h);
assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'aborted'));
fprintf('ok  abort from a move, an operator step and pause; stop not confirmed; model gone (dt %.2f)\n', dt);
end

function testFault(dt)
% A slave fault pauses the procedure in every situation: nothing is sent,
% the mixer stays on, resume is refused until the status has left 9, 10 and
% 32, and the run goes on from the same step.
kinds = {'machine', 'wait', 'operator', 'paused'};
for i = 1:numel(kinds)
    h = faultHost(kinds{i}, dt);
    n = size(h.events, 1);
    nMix = size(h.mixer, 1);
    stepBefore = h.p.k;
    ticksBefore = h.p.checks;
    h.m.force = 10;
    h.m.faultCode = 7;
    h = tick(h);   % the model's status changes, then the engine reads it
    h = tick(h);
    assert(strcmp(h.p.state, 'paused') && strcmp(h.p.reason, 'Slave fault 7'), kinds{i});
    assert(h.p.faultCode == 7 && isempty(h.p.outcome));
    assert(strcmp(procView(h.p).stateText, 'Paused: Slave fault 7'));
    assert(~procView(h.p).pause.enable);
    h = runFor(h, 1.0);
    assert(size(h.events, 1) == n, 'no request after a fault');
    assert(size(h.mixer, 1) == nMix && h.mixer(end, 2) == 1, 'the mixer is left on');
    % Resume is refused through the reset (9) and a failed reset (32).
    for force = [10 9 32]
        h.m.force = force;
        h = tick(h);
        h = act(h, 'resume');
        h = tick(h);
        assert(strcmp(h.p.state, 'paused') && strcmp(h.p.reason, 'Slave fault 7'));
        assert(~isempty(strfind(h.p.message, 'Cannot resume: Slave fault 7')));
    end
    % The fault is gone.
    h.m.force = [];
    h = tick(h);
    h = tick(h);
    assert(strcmp(h.p.state, 'paused') && isempty(h.p.reason));
    assert(~isempty(strfind(h.p.message, 'cleared')));
    assert(procView(h.p).pause.enable);
    h = act(h, 'resume');
    h = tick(h);
    switch kinds{i}
        case 'machine'
            h = runUntil(h, @(h) h.p.k == 3, 30);
            assert(sum(h.events(:, 2) == 2) == 2, 'the move is issued again');
        case 'wait'
            assert(strcmp(h.p.state, 'running') && h.p.k == stepBefore);
        case 'operator'
            assert(strcmp(h.p.state, 'waitOperator') && h.p.k == stepBefore);
            assert(isequal(h.p.checks, ticksBefore), 'the ticks are kept');
        case 'paused'
            assert(strcmp(h.p.state, 'running') && h.p.k == stepBefore);
    end
    h = runUntil(h, @(h) h.p.k == 4 && strcmp(h.p.state, 'waitOperator'), 30);
    h = act(h, 'check', 1, true);
    h = act(h, 'check', 2, true);
    h = act(h, 'confirm');
    h = tick(h);
    assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'completed'), kinds{i});
end
fprintf('ok  a slave fault pauses a move, a wait, an operator step and a pause; resume follows the reset\n');

% Abort while faulted: a stop is sent and the run ends aborted.
h = faultHost('machine', dt);
h.m.force = 10;
h.m.faultCode = 7;
h = runFor(h, 0.2);
assert(strcmp(h.p.state, 'paused'));
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'aborting') && h.events(end, 2) == -1);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted'));
assert(h.mixer(end, 2) == 0, 'abort switches the mixer off');
% And with the model not running.
h = faultHost('operator', dt);
h.m.force = 10;
h.m.faultCode = 7;
h = runFor(h, 0.2);
h.m.connected = false;
h = tick(h);
assert(strcmp(h.p.state, 'paused') && strcmp(h.p.reason, 'Model stopped'));
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'aborted'));
fprintf('ok  abort after a slave fault ends aborted\n');

% A kept pressure hold: the fault counts it as lost and resume re-runs it
% before the interrupted step.
proc = mk({struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true, 'timerStart', true), ...
    struct('type', 'confirm', 'title', 'Operator', 'checks', {{'a', 'b'}})});
h = newHost(proc, dt);
h = act(h, 'start');
h = runUntil(h, @(h) h.p.k == 2, 10);
h = act(h, 'check', 1, true);
h = tick(h);
n = size(h.events, 1);
h.m.force = 10;
h.m.faultCode = 4;
h = runFor(h, 0.3);
assert(strcmp(h.p.state, 'paused') && h.p.bg.lost && strcmp(h.p.reason, 'Slave fault 4'));
assert(size(h.events, 1) == n);
h.m.mode = 'idle';      % the master was latched: the hold is gone
h.m.op = 0;
h.m.force = [];
h = runFor(h, 0.3);
h = act(h, 'resume');
h = tick(h);
assert(h.p.k == 1 && strcmp(h.p.state, 'running'), 'the hold is re-run first');
h = runUntil(h, @(h) h.p.k == 2, 10);
assert(strcmp(h.p.state, 'waitOperator') && isequal(h.p.checks, [true false]));
assert(sum(h.events(:, 2) == 4) == 2);
fprintf('ok  a slave fault re-runs the kept pressure step on resume\n');
end

function testStopsAndLoss(dt)
% E-STOP, a lost link and a stopped model pause the procedure; resume is
% refused while the condition holds and re-issues the move once it is gone.
cases = {'estop', 'E-STOP'; 'status20', 'E-STOP'; 'status34', 'Slave link lost'; ...
    'status35', 'Slave link lost'; 'model', 'Model stopped'};
for i = 1:size(cases, 1)
    h = faultHost('machine', dt);
    n = size(h.events, 1);
    h.m.mode = 'idle';      % the master has stopped everything
    h.m.op = 0;
    switch cases{i, 1}
        case 'estop',    h.m.estop = true;
        case 'status20', h.m.force = 20;
        case 'status34', h.m.force = 34;
        case 'status35', h.m.force = 35;
        case 'model',    h.m.connected = false;
    end
    h = tick(h);   % the model's status changes, then the engine reads it
    h = tick(h);
    assert(strcmp(h.p.state, 'paused'), cases{i, 1});
    assert(strcmp(h.p.reason, cases{i, 2}), cases{i, 1});
    assert(strcmp(h.p.phase, 'issue'));
    assert(size(h.events, 1) == n, 'no stop is sent');
    assert(~procView(h.p).pause.enable);
    h = act(h, 'resume');
    h = tick(h);
    assert(strcmp(h.p.state, 'paused') && ~isempty(strfind(h.p.message, 'Cannot resume')));
    % The condition goes.
    h.m.estop = false;
    h.m.force = [];
    h.m.connected = true;
    h = tick(h);
    h = tick(h);
    assert(strcmp(h.p.state, 'paused') && isempty(h.p.reason));
    assert(procView(h.p).pause.enable);
    h = act(h, 'resume');
    h = runUntil(h, @(h) h.p.k == 3, 30);
    assert(sum(h.events(:, 2) == 2) == 2, 'the interrupted move is issued again');
    h = runUntil(h, @(h) h.p.k == 4, 10);
    h = act(h, 'check', 1, true);
    h = act(h, 'check', 2, true);
    h = act(h, 'confirm');
    h = tick(h);
    assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'completed'));
end
fprintf('ok  E-STOP, link lost and model stopped pause; resume once cleared completes\n');

% With a kept pressure hold: the hold counts as lost, the timer keeps
% counting and resume re-runs the hold before returning to the step.
proc = mk({struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true, 'timerStart', true), ...
    struct('type', 'confirm', 'title', 'Operator', 'checks', {{'a', 'b'}})});
h = newHost(proc, dt);
h = act(h, 'start');
h = runUntil(h, @(h) h.p.k == 2, 10);
h = act(h, 'check', 1, true);
h = tick(h);
h.m.mode = 'idle';
h.m.op = 0;
h.m.estop = true;
h = tick(h);
assert(strcmp(h.p.state, 'paused') && h.p.bg.lost && strcmp(h.p.reason, 'E-STOP'));
e0 = h.p.timer.elapsed;
h = runFor(h, 1.0);
assert(h.p.timer.elapsed > e0 + 0.9, 'the timer keeps counting');
h.m.estop = false;
h = runFor(h, 0.2);
h = act(h, 'resume');
h = tick(h);
assert(h.p.k == 1 && strcmp(h.p.state, 'running'));
h = runUntil(h, @(h) h.p.k == 2, 10);
assert(strcmp(h.p.state, 'waitOperator') && isequal(h.p.checks, [true false]));
assert(sum(h.events(:, 2) == 4) == 2);
fprintf('ok  a pause by E-STOP re-runs the kept pressure step on resume\n');

% Abort from the pause: at once when the model is gone.
h = newHost(proc, dt);
h = act(h, 'start');
h = runUntil(h, @(h) h.p.k == 2, 10);
h.m.connected = false;
h = tick(h);
assert(strcmp(h.p.state, 'paused') && strcmp(h.p.reason, 'Model stopped'));
n = size(h.events, 1);
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'aborted'));
assert(size(h.events, 1) == n);
% And with the model running (E-STOP): stop, then aborted.
h = newHost(proc, dt);
h = act(h, 'start');
h = runUntil(h, @(h) h.p.k == 2, 10);
h.m.estop = true;
h = tick(h);
h = act(h, 'abort');
h = tick(h);
assert(strcmp(h.p.state, 'aborting') && h.events(end, 2) == -1);
h.m.estop = false;
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted'));
fprintf('ok  abort from a supervision pause\n');
end

function testEarlyEnd(dt)
proc = mk({struct('type', 'machine', 'title', 'Home', 'op', 'home'), ...
    struct('type', 'confirm', 'title', 'Done')});
h = newHost(proc, dt);
h.m.inject = 'noConfirm';
h = act(h, 'start');
h = runUntil(h, @(h) strcmp(h.p.state, 'paused'), 10);
assert(~isempty(strfind(h.p.message, '30')));
h.m.inject = '';
h = act(h, 'resume');
h = runUntil(h, @(h) strcmp(h.p.state, 'waitOperator'), 10);
assert(h.p.k == 2);

h = newHost(proc, dt);
h.m.inject = 'stopTimeout';
h = act(h, 'start');
h = runUntil(h, @(h) strcmp(h.p.state, 'paused'), 15);
assert(~isempty(strfind(h.p.message, '33')));
h.m.inject = '';
h = act(h, 'resume');
h = runUntil(h, @(h) strcmp(h.p.state, 'waitOperator'), 15);
assert(h.p.k == 2);
fprintf('ok  an operation not confirmed or not stopped pauses with a message; resume retries\n');
end

function testBackgroundLost(dt)
proc = mk({struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pTarget', 'keep', true, 'timerStart', true), ...
    struct('type', 'confirm', 'title', 'Operator', 'checks', {{'a', 'b'}})});
h = newHost(proc, dt);
h = act(h, 'start');
h = runUntil(h, @(h) h.p.k == 2, 10);
h = act(h, 'check', 1, true);
h = tick(h);
% The master leaves pressure control.
h.m.mode = 'idle';
h.m.op = 0;
h = runUntil(h, @(h) strcmp(h.p.state, 'paused'), 5);
assert(~isempty(strfind(h.p.message, 'ended')));
elapsed = h.p.timer.elapsed;
h = act(h, 'resume');
h = tick(h);
assert(h.p.k == 1 && strcmp(h.p.state, 'running'), 're-runs the pressure step');
h = runUntil(h, @(h) h.p.k == 2, 10);
assert(strcmp(h.p.state, 'waitOperator'));
assert(isequal(h.p.checks, [true false]), 'the paused step keeps its ticks');
assert(h.p.timer.elapsed >= elapsed, 'the timer is not restarted');
assert(sum(h.events(:, 2) == 4) == 2);
fprintf('ok  a lost background operation pauses, resume re-runs it and returns\n');
end

function testTimeout(dt)
proc = mk({struct('type', 'machine', 'title', 'Move', 'op', 'position', ...
    'target', 290, 'timeout', 1)});
h = newHost(proc, dt);
h.m.speed = 0;
h = act(h, 'start');
h = runUntil(h, @(h) strcmp(h.p.state, 'paused'), 5);
assert(~isempty(strfind(h.p.message, 'within 1 s')));
assert(h.events(end, 2) == -1, 'stop is sent on a timeout');
h.m.speed = 50;
h = act(h, 'resume');
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 20);
assert(sum(h.events(:, 2) == 2) == 2);
fprintf('ok  a step timeout pauses and stops the operation\n');
end

function testRepeat(dt)
proc = mk({struct('id', 'a', 'type', 'confirm', 'title', 'A', 'checks', {{'x', 'y'}}), ...
    struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 280), ...
    struct('type', 'confirm', 'title', 'B', 'repeatFrom', 'a')});
h = newHost(proc, dt);
h = act(h, 'start');
h = tick(h);
h = act(h, 'check', 1, true);
h = act(h, 'check', 2, true);
h = act(h, 'confirm');
h = runUntil(h, @(h) h.p.k == 3 && strcmp(h.p.state, 'waitOperator'), 10);
assert(isequal(h.p.stepDone, [true true false]));
h = act(h, 'repeat');
h = tick(h);
assert(h.p.k == 1 && ~any(h.p.stepDone) && ~any(h.p.checks));
h = act(h, 'check', 1, true);
h = act(h, 'check', 2, true);
h = act(h, 'confirm');
h = runUntil(h, @(h) h.p.k == 3 && strcmp(h.p.state, 'waitOperator'), 10);
h = act(h, 'confirm');
h = tick(h);
assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'completed'));
assert(all(h.p.stepDone));
fprintf('ok  repeat jumps back, clears ticks and done marks, and the run completes\n');
end

function testWrongState(dt)
proc = mk({struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 100), ...
    struct('type', 'confirm', 'title', 'C')});
h = newHost(proc, dt);
for a = {'confirm', 'pause', 'resume', 'abort', 'repeat', 'reset'}
    h = refuse(h, a{1});
end
h = refuse(h, 'check', 1, true);
h = refuse(h, 'setParam', 'vol', 1);
h = act(h, 'start');
h = runFor(h, 0.5);
assert(strcmp(h.p.state, 'running'));
for a = {'confirm', 'start', 'resume', 'repeat', 'reset'}
    h = refuse(h, a{1});
end
h = refuse(h, 'check', 1, true);
h = refuse(h, 'setParam', 'vol', 1);
h = refuse(h, 'bogus');
fprintf('ok  actions in the wrong state are refused and leave the state unchanged\n');
end

function testPurity()
proc = mk({struct('type', 'confirm', 'title', 'A', 'checks', {{'x'}}), ...
    struct('type', 'wait', 'title', 'W', 'duration', 1), ...
    struct('type', 'confirm', 'title', 'B')});
cfg = masterConfig();
dt = 0.05;
pa = procNew(proc, cfg);
pb = procNew(proc, cfg);
% Script for each engine: input and action at each tick.
n = 60;
inA = @(i) struct('connected', true, 'statusCode', 0, 'faultCode', 0, ...
    'estop', false, 'pressure', 1 + 0.1 * i, 'volume', 100 - i, ...
    'pressureBlock', false, 'atEndOfTravel', false);
inB = @(i) struct('connected', true, 'statusCode', 10 * (i > 20), ...
    'faultCode', 3, 'estop', false, 'pressure', 2, 'volume', 50, ...
    'pressureBlock', false, 'atEndOfTravel', false);
soloA = runScript(pa, inA, n, dt);
soloB = runScript(pb, inB, n, dt);
% Interleaved, one tick of each in turn.
qa = pa;
qb = pb;
for i = 1:n
    qa = scriptTick(qa, inA, i, dt);
    qb = scriptTick(qb, inB, i, dt);
end
assert(isequaln(qa, soloA) && isequaln(qb, soloB));
assert(strcmp(soloA.state, 'done'));
assert(strcmp(soloB.state, 'paused') && strcmp(soloB.reason, 'Slave fault 3'));
fprintf('ok  the engine is pure: interleaved engines match their solo runs\n');
end

function p = runScript(p, inFn, n, dt)
for i = 1:n
    p = scriptTick(p, inFn, i, dt);
end
end

function p = scriptTick(p, inFn, i, dt)
% Operator actions at fixed ticks: start, tick the check, confirm, confirm.
switch i
    case 1,  [p, ~] = procAction(p, 'start');
    case 5,  [p, ~] = procAction(p, 'check', 1, true);
    case 6,  [p, ~] = procAction(p, 'confirm');
    case 45, [p, ~] = procAction(p, 'confirm');
end
p = procStep(p, inFn(i), dt);
end

function testView()
proc = mk({ ...
    struct('type', 'param', 'title', 'Set', 'section', 'Setup', ...
        'params', {{'pTarget'}}, 'text', 'Hold {pTarget} atm.'), ...
    struct('type', 'confirm', 'title', 'Check', 'section', 'Run', ...
        'checks', {{'one', 'two'}}), ...
    struct('type', 'wait', 'title', 'Wait', 'duration', 5)});
cfg = masterConfig();
p = procNew(proc, cfg);
v = procView(p);
assert(v.startEnable && ~v.active && ~v.abortEnable && ~v.confirm.visible);
assert(isequal(v.list, {'  Set'; '  Check'; '  Wait'}));
p = procAction(p, 'start');
p = procStep(p, idleIn(), 0.05);
v = procView(p);
assert(strcmp(v.heading, 'Step 1 of 3 - Setup'));
assert(strcmp(v.text, 'Hold 3.0 atm.'), 'the format of the parameter is used');
assert(isequal(v.list, {'> Set'; '  Check'; '  Wait'}) && v.listIndex == 1);
assert(isequal(v.params.editable, [false true false]));
assert(v.confirm.visible && v.confirm.enable && ~v.startEnable && v.active);
assert(strcmp(v.pause.label, 'Pause') && v.pause.enable && v.abortEnable);
p = procAction(p, 'confirm');
p = procStep(p, idleIn(), 0.05);
v = procView(p);
assert(strcmp(v.heading, 'Step 2 of 3 - Run'));
assert(isequal(v.list, {'[x] Set'; '> Check'; '  Wait'}) && v.listIndex == 2);
assert(isequal(v.checks.labels, {'one', 'two'}) && ~any(v.checks.values));
assert(~v.confirm.enable && ~any(v.params.editable));
p = procAction(p, 'check', 1, true);
p = procAction(p, 'check', 2, true);
v = procView(p);
assert(v.confirm.enable && all(v.checks.values));
p = procAction(p, 'pause');
p = procStep(p, idleIn(), 0.05);
v = procView(p);
assert(strcmp(v.pause.label, 'Resume') && ~v.confirm.enable);
p = procAction(p, 'resume');
p = procStep(p, idleIn(), 0.05);
p.timer.elapsed = 65;
v = procView(p);
assert(strcmp(v.timerText, '1:05'));
assert(strcmp(v.volumeText, '--'));
p = procNew(proc, cfg);
assert(strcmp(procView(p).timerText, '--'));
fprintf('ok  procView: heading, list markers, substitution, buttons, readouts\n');
end

function testFunctionTarget(dt)
% A target given as a function of the parameter values and the cfg is
% evaluated when the step issues its operation.
proc = mk({struct('type', 'machine', 'title', 'Move', 'op', 'position', ...
    'unit', 'mL', 'target', @(v, c) c.deadVolume + 2 * v.vol)});
h = newHost(proc, dt);
h = act(h, 'start');
h = runFor(h, 0.2);
op = h.events(h.events(:, 2) == 2, :);
assert(size(op, 1) == 1 && abs(op(1, 4) - (h.cfg.deadVolume + 20)) < 1e-12);
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 30);
fprintf('ok  a target function is evaluated at the step (%.1f mL)\n', op(1, 4));

% A function that fails once a value is set: the step pauses with the
% message and issues nothing; resume tries again.
proc = mk({struct('type', 'param', 'title', 'Set', 'params', {{'vol'}}), ...
    struct('type', 'machine', 'title', 'Move', 'op', 'position', ...
        'unit', 'mL', 'target', @(v, c) failAbove(v.vol, 15))});
h = newHost(proc, dt);
h = act(h, 'start');
h = tick(h);
h = act(h, 'setParam', 'vol', 18);
h = act(h, 'confirm');
h = runFor(h, 0.5);
assert(strcmp(h.p.state, 'paused') && isempty(h.p.reason));
assert(strncmp(h.p.message, 'Step target could not be computed: too large', 44));
assert(isempty(h.events), 'nothing is issued');
h = act(h, 'resume');
h = runFor(h, 0.3);
assert(strcmp(h.p.state, 'paused') && isempty(h.events));
h = act(h, 'abort');
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 5);
assert(strcmp(h.p.outcome, 'aborted'));
fprintf('ok  a target function that fails pauses the step and issues nothing\n');
end

function v = failAbove(x, limit)
if x > limit
    error('smoke:target', 'too large');
end
v = x;
end

function testSpeed(dt)
proc = mk({struct('type', 'machine', 'title', 'Slow', 'op', 'position', ...
        'target', 250, 'speed', 0.5), ...
    struct('type', 'machine', 'title', 'Free', 'op', 'position', 'target', 200)});
h = newHost(proc, dt);
h.m.speed = 500;
h = act(h, 'start');
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 20);
op = h.events(h.events(:, 2) == 2, :);
assert(size(op, 1) == 2 && op(1, 3) == 0.5 && isnan(op(2, 3)));
fprintf('ok  a position step issues its speed cap (NaN when it has none)\n');
end

function testPressureGuards(dt)
% The pressure above the step's limit and the slave's pressure block pause
% a machine step with a stop; resume issues the step again.
move = struct('type', 'machine', 'title', 'Move', 'op', 'position', ...
    'target', 100, 'pressureMax', 3);
cases = {'pressure', 'Pressure above the step''s limit'; ...
    'block', 'Pressure ceiling reached'};
for i = 1:size(cases, 1)
    h = newHost(mk({move}), dt);
    h = act(h, 'start');
    h = runFor(h, 0.5);
    assert(strcmp(h.p.phase, 'track'));
    n = size(h.events, 1);
    if i == 1
        h.m.pressure = 4;
    else
        h.m.pressureBlock = true;
    end
    h = tick(h);
    assert(strcmp(h.p.state, 'paused') && strcmp(h.p.message, cases{i, 2}), cases{i, 1});
    assert(isempty(h.p.reason));
    assert(size(h.events, 1) == n + 1 && h.events(end, 2) == -1, 'a stop is sent');
    h.m.pressure = 1;
    h.m.pressureBlock = false;
    h = runFor(h, 0.5);
    h = act(h, 'resume');
    h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 30);
    assert(sum(h.events(:, 2) == 2) == 2, 'the move is issued again');
end
% A step without the limit rides through a high pressure.
h = newHost(mk({struct('type', 'machine', 'title', 'Move', 'op', 'position', ...
    'target', 100)}), dt);
h = act(h, 'start');
h.m.pressure = 50;
h = runUntil(h, @(h) strcmp(h.p.state, 'done'), 30);
fprintf('ok  the pressure limit and the pressure block pause a move with a stop\n');
end

function testAlsoWhen(dt)
mk2 = @(extra) mk({fieldsOver(struct('type', 'confirm', 'title', 'Deliver', ...
    'zeroVolume', true), extra), struct('type', 'confirm', 'title', 'After')});
% The slave's end-of-travel flag completes the step without the click.
h = newHost(mk2(struct('alsoWhen', 'endOfTravel')), dt);
h = act(h, 'start');
h = runFor(h, 0.3);
assert(h.p.k == 1 && strcmp(h.p.state, 'waitOperator'));
h.m.atEnd = true;
h = tick(h);
assert(h.p.k == 2 && h.p.stepDone(1));
assert(~isempty(strfind(procView(h.p).message, 'Deliver ended at the end of travel')));
% The swept volume at or below the end volume does the same.
h = newHost(mk2(struct('alsoWhen', 'endOfTravel')), dt);
h = act(h, 'start');
h = runFor(h, 0.3);
h.m.d = 1.0;     % mm: about 3.4 mL
assert(mLPerMm(h.cfg) * h.m.d <= h.cfg.procEndVolume);
h = tick(h);
assert(h.p.k == 2);
h.m.d = 300;
% Without alsoWhen the step waits for the click.
h = newHost(mk2(struct()), dt);
h = act(h, 'start');
h = runFor(h, 0.3);
h.m.atEnd = true;
h.m.d = 1.0;
h = runFor(h, 0.5);
assert(h.p.k == 1 && strcmp(h.p.state, 'waitOperator'));
% A pressure step that is not kept ends with a stop at the end of travel.
proc = mk({struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pTarget', 'until', 'settled', 'duration', 100, ...
        'alsoWhen', 'endOfTravel'), ...
    struct('type', 'confirm', 'title', 'After')});
h = newHost(proc, dt);
h = act(h, 'start');
h = runFor(h, 1.0);
assert(h.p.k == 1 && strcmp(h.p.phase, 'track') && h.m.status == 5);
n = sum(h.events(:, 2) == -1);
h.m.atEnd = true;
h = runUntil(h, @(h) h.p.k == 2, 5);
assert(sum(h.events(:, 2) == -1) == n + 1, 'a stop is sent');
assert(~isempty(strfind(h.p.note, 'Hold ended at the end of travel')));
fprintf('ok  alsoWhen ends a confirm or a pressure step at the end of travel\n');
end

function s = fieldsOver(s, extra)
f = fieldnames(extra);
for i = 1:numel(f)
    s.(f{i}) = extra.(f{i});
end
end

function testSeedWalk(dt)
% The seed procedure through the stand-in: every machine step's request,
% the speed of the two pushes and the order of the operations.
proc = sopProcedure();
h = newHost(proc, dt);
cfg = h.cfg;
h = act(h, 'start');
guard = 0;
while ~strcmp(h.p.state, 'done') && guard < 4000
    h = tick(h);
    guard = guard + 1;
    if strcmp(h.p.state, 'waitOperator')
        st = h.p.def.steps(h.p.k);
        for i = 1:numel(st.checks)
            h = act(h, 'check', i, true);
        end
        h = act(h, 'confirm');
    end
end
assert(strcmp(h.p.state, 'done') && strcmp(h.p.outcome, 'completed'));
assert(all(h.p.stepDone));
ops = h.events(:, 2)';
assert(isequal(ops, [2 -1 2 -1 2 -1 2 -1 2 -1 4 -1]), mat2str(ops));
mv = h.events(h.events(:, 2) == 2, :);
assert(abs(mv(1, 4) - (cfg.deadVolume + 100)) < 1e-9);
assert(mv(2, 4) == 0 && mv(2, 3) == 0.5);
assert(abs(mv(3, 4) - (343 + 30)) < 1e-9);
assert(abs(mv(4, 4) - 343) < 1e-9 && mv(4, 3) == 0.5);
assert(abs(mv(5, 4) - (343 + 343 * (1 / 7.8) * 293.15 / 273.15)) < 1e-9);
assert(all(isnan(mv([1 3 5], 3))));
assert(isequal(h.params{1}, struct('SalineVolume', 343, 'OxygenRatio', 1)));
fprintf('ok  the seed procedure walks to the end (%d operations)\n', size(h.events, 1));
end

function testPressureView()
cfg = masterConfig();
params = {struct('key', 'pp', 'label', 'Hold', 'kind', 'pressure', ...
        'value', 3, 'min', 1, 'max', 10), ...
    struct('key', 'vol', 'label', 'Volume', 'unit', 'mL', 'value', 10, ...
        'min', 0, 'max', 20, 'format', '%.1f')};
proc = mk({struct('type', 'param', 'title', 'Set', 'params', {{'pp', 'vol'}}, ...
        'text', 'Hold {pp} and {vol}.'), ...
    struct('type', 'machine', 'title', 'Hold', 'op', 'pressure', ...
        'target', 'pp', 'until', 'running', 'keep', true), ...
    struct('type', 'confirm', 'title', 'Next')}, params);
p = procNew(proc, cfg);
p = procAction(p, 'start');
p = procStep(p, idleIn(), 0.05);
% Atm by default.
v = procView(p);
assert(strcmp(v.text, 'Hold 3.00 atm abs and 10.0.'), v.text);
assert(strcmp(v.params.labels{1}, 'Hold (atm abs)') && v.params.values(1) == 3);
assert(isequal(v.params.kinds, {'pressure', ''}));
% Psi.
psi = pressureUnitInfo('psi', cfg);
v = procView(p, psi);
k = cfg.psiPerAtm;
assert(strcmp(v.text, sprintf('Hold %.1f psi abs and 10.0.', 3 * k)), v.text);
assert(strcmp(v.params.labels{1}, 'Hold (psi abs)'));
assert(abs(v.params.values(1) - 3 * k) < 1e-12);
assert(abs(v.params.mins(1) - k) < 1e-12 && abs(v.params.maxs(1) - 10 * k) < 1e-12);
assert(strcmp(v.params.formats{1}, '%.1f') && strcmp(v.params.formats{2}, '%.1f'));
assert(strcmp(v.params.labels{2}, 'Volume (mL)') && v.params.maxs(2) == 20);
% The engine keeps atm and gives a unit-free range message.
[~, ok, msg] = procAction(p, 'setParam', 'pp', 11);
assert(~ok && strcmp(msg, 'Hold is out of range'));
p = procAction(p, 'setParam', 'pp', 4);
p = procAction(p, 'confirm');
p = procStep(p, idleIn(), 0.05);
assert(p.values.pp == 4);
% The held pressure.
assert(strcmp(procView(p, psi).pressureTargetText, '--'));
h = newHost(proc, 0.05);
h = act(h, 'start');
h = tick(h);
h = act(h, 'confirm');
h = runUntil(h, @(h) h.p.k == 3, 10);
v = procView(h.p, psi);
assert(v.pressureTarget == 3);
assert(strcmp(v.pressureTargetText, sprintf('%.1f psi abs', 3 * k)));
assert(strcmp(procView(h.p).pressureTargetText, '3.00 atm abs'));
mpa = procView(h.p, pressureUnitInfo('MPa', cfg));
assert(strcmp(mpa.pressureTargetText, sprintf('%.3f MPa abs', 3 / cfg.atmPerMPa)));
fprintf('ok  procView converts pressure parameters, ranges, texts and the held pressure\n');
end

% ===== Fixtures =====

function proc = mk(steps, params)
% A procedure with the default test parameters unless params is given.
if nargin < 2
    params = {struct('key', 'vol', 'label', 'Volume', 'unit', 'mL', ...
        'value', 10, 'min', 0, 'max', 20, 'sink', 'SalineVolume'), ...
        struct('key', 'pTarget', 'value', 3, 'min', 1, 'max', 10, ...
        'format', '%.1f'), ...
        struct('key', 'mix', 'value', 1, 'min', 0, 'max', 1)};
end
proc = struct('name', 'synthetic', 'params', {params}, 'steps', {steps});
end

function proc = setStep(proc, k, field, value)
proc.steps{k}.(field) = value;
end

function proc = setParam(proc, k, field, value)
proc.params{k}.(field) = value;
end

function in = idleIn()
in = struct('connected', true, 'statusCode', 0, 'faultCode', 0, ...
    'estop', false, 'pressure', 1, 'volume', 100, 'pressureBlock', false, ...
    'atEndOfTravel', false);
end

function h = faultHost(kind, dt)
% A run in the given situation: a move in flight, a timed wait, an operator
% step with the pressure held, or paused. The mixer is on.
proc = mk({struct('type', 'confirm', 'title', 'Mixer on', 'mixer', 'on'), ...
    struct('type', 'machine', 'title', 'Move', 'op', 'position', 'target', 100), ...
    struct('type', 'wait', 'title', 'Wait', 'duration', 3), ...
    struct('type', 'confirm', 'title', 'Operator', 'checks', {{'a', 'b'}})});
h = newHost(proc, dt);
h = act(h, 'start');
h = tick(h);
h = act(h, 'confirm');
h = tick(h);
assert(h.mixer(end, 2) == 1);
if strcmp(kind, 'machine')
    h = runFor(h, 0.5);
    assert(strcmp(h.p.phase, 'track'));
    return
end
h = runUntil(h, @(h) h.p.k == 3, 30);
h = runFor(h, 0.3);
if strcmp(kind, 'paused')
    h = act(h, 'pause');
    h = tick(h);
    assert(strcmp(h.p.state, 'paused'));
elseif strcmp(kind, 'operator')
    h = runUntil(h, @(h) h.p.k == 4, 10);
    h = act(h, 'check', 1, true);
    h = tick(h);
end
end

% ===== Stand-in model and host =====

function m = modelInit(cfg)
% The model's side: the status code the app reads and what drives it.
m = struct('status', 0, 'mode', 'idle', 'tm', 0, 'op', 0, 'unit', 'mm', ...
    'target', 0, 'd', 300, 'speed', 50, 'homeT', 1.0, 'pressure', 1, ...
    'connected', true, 'estop', false, 'faultCode', 0, 'inject', '', ...
    'force', [], 'pressureBlock', false, 'atEnd', false);
m.cfg = cfg;
end

function m = modelStep(m, req, dt)
% Status codes as masterStateMachine reports them for each operation.
cfg = m.cfg;
m.tm = m.tm + dt;
if req.stop
    m.op = 0;
    if strcmp(m.inject, 'stuck')
        m.mode = 'stuck';
    else
        m.mode = 'stopping';
    end
    m.tm = 0;
end
if req.op >= 0
    if req.op == 2
        if strcmp(req.unit, 'mL')
            tgt = (req.target - cfg.deadVolume) / mLPerMm(cfg);
        else
            tgt = req.target;
        end
    else
        tgt = req.target;
    end
    if strcmp(m.mode, 'run') && m.op == req.op
        m.target = tgt;        % retarget live
    else
        m.op = req.op;
        m.target = tgt;
        m.mode = 'starting';
        m.tm = 0;
    end
end
switch m.mode
    case 'idle'
        m.status = 0;
    case 'starting'
        m.status = 7;
        if m.tm >= 0.15
            if strcmp(m.inject, 'noConfirm')
                m.mode = 'noconf';
                m.status = 30;
            else
                m.mode = 'run';
                m.tm = 0;
            end
        end
    case 'noconf'
        m.status = 30;
    case 'run'
        switch m.op
            case 2
                step = max(-m.speed * dt, min(m.speed * dt, m.target - m.d));
                m.d = m.d + step;
                if abs(m.target - m.d) <= 0.2
                    m.status = 3;
                else
                    m.status = 2;
                end
            case 4
                m.status = 5;
                m.pressure = m.pressure + (m.target - m.pressure) * min(1, dt / 0.3);
            otherwise
                if m.op == 1, m.status = 1; else, m.status = 8; end
                if m.tm >= m.homeT
                    m.mode = 'stopping';
                    m.tm = 0;
                    m.status = 6;
                end
        end
    case 'stopping'
        m.status = 6;
        if strcmp(m.inject, 'stopTimeout')
            if m.tm >= 3.2
                m.mode = 'stoptimeout';
                m.status = 33;
            end
        elseif m.tm >= 0.2
            m.mode = 'idle';
            m.status = 0;
            m.op = 0;
        end
    case 'stoptimeout'
        m.status = 33;
    case 'stuck'
        m.status = 2;
end
if ~isempty(m.force)
    m.status = m.force;
end
end

function h = newHost(proc, dt)
h.cfg = masterConfig();
h.dt = dt;
h.p = procNew(proc, h.cfg);
h.m = modelInit(h.cfg);
h.t = 0;
h.events = zeros(0, 4);      % [time, op code or -1 for a stop, speed, target]
h.mixer = zeros(0, 2);       % [time, 0/1]
h.params = {};
h.zero = 0;
h.tStart = NaN;
h.tStop = NaN;
end

function h = tick(h)
% One poll: read the model, step the engine, execute its requests.
m = h.m;
in = struct('connected', m.connected, 'statusCode', m.status, ...
    'faultCode', m.faultCode, 'estop', m.estop, 'pressure', m.pressure, ...
    'volume', mLPerMm(h.cfg) * m.d, 'pressureBlock', m.pressureBlock, ...
    'atEndOfTravel', m.atEnd);
[h.p, req] = procStep(h.p, in, h.dt);
h.t = h.t + h.dt;
if ~isempty(req.params)
    h.params{end + 1} = req.params;
end
if req.mixer >= 0
    h.mixer(end + 1, :) = [h.t req.mixer];
end
if req.stop
    h.events(end + 1, :) = [h.t -1 NaN NaN];
end
if req.op >= 0
    h.events(end + 1, :) = [h.t req.op req.speed req.target];
end
if req.zeroVolume
    h.zero = h.zero + 1;
end
if h.p.timer.running && isnan(h.tStart)
    h.tStart = h.t;
end
if ~h.p.timer.running && ~isnan(h.tStart) && isnan(h.tStop)
    h.tStop = h.t;
end
h.m = modelStep(m, req, h.dt);
end

function h = runFor(h, T)
n = round(T / h.dt);
for i = 1:n
    h = tick(h);
end
end

function h = runUntil(h, pred, T)
t0 = h.t;
while ~pred(h) && h.t - t0 < T
    h = tick(h);
end
assert(pred(h), 'condition not reached within %g s (state %s, step %d)', ...
    T, h.p.state, h.p.k);
end

function h = act(h, varargin)
[h.p, ok, msg] = procAction(h.p, varargin{:});
assert(ok, 'action %s refused: %s', varargin{1}, msg);
end

function h = refuse(h, varargin)
% The action is refused and the state is unchanged.
[q, ok, msg] = procAction(h.p, varargin{:});
assert(~ok && ~isempty(msg), 'action %s was accepted', varargin{1});
assert(isequaln(q, h.p), 'a refused action changed the state');
end
