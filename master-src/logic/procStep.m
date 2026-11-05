function [p, req] = procStep(p, in, dt)
%PROCSTEP Advance the procedure engine by one poll and return its requests.
%   [p, req] = procStep(p, in, dt)
%
%   p    engine state from procNew
%   dt   s since the previous call
%   in   what the app read from the model this poll:
%          connected   the model is running
%          statusCode  the model's StatusCode
%          faultCode   the model's FaultCode (read when statusCode is 10)
%          estop       the E-STOP button is down or the model reports E-STOP
%          pressure    atm absolute
%          volume      mL; only its differences are used
%
%   req is what the app is to do, in this order (an empty value means do
%   nothing):
%          params      struct, sink name -> value, written to the shell
%                      Constant blocks, or []
%          mixer       -1 none, 0 or 1: switch the stir bar
%          stop        true: request a stop (RequestedMode 0)
%          op          -1 none, else RequestedMode: 1 home, 2 position,
%                      4 pressure, 6 go home (target and unit below)
%          target      mm from the end of travel or mL (unit 'mm'/'mL') for
%                      a position, atm absolute (unit 'atm') for a pressure
%          unit        '' when op has no target
%          zeroVolume  true: take the reference of the volume readout
%
%   The engine does everything the operator and the model decide here, and
%   nothing else: procAction only records the operator's intent, and the
%   app turns req into commands through its own operation requests, where
%   the interlocks are enforced.
%
%   States: idle, running, waitOperator (a confirm or param step), paused,
%   aborting (stop sent, waiting for the axis), done (outcome completed or
%   aborted) and faulted (a slave fault; nothing resumes, the operator
%   starts again). E-STOP, a lost slave link and a stopped model pause the
%   procedure with reason set; resume is refused until the condition has
%   gone, and abort ends at once if the model is not running. Fields of p:
%     def, c        the normalized procedure and the cfg values it needs
%     state, outcome, reason, faultCode, message
%     k, stepT      current step and the time in it (not counted paused)
%     phase         within a machine step: issue, track, stopping, settle
%     track         the current step's operation: op, confirmed, sinceSent,
%                   target (op as RequestedMode)
%     settleT       s the pressure has been inside the band
%     settleOk      the operation in settle ended as the step expected
%     bg            the operation a keep step left running: op, target,
%                   step, confirmed, sinceSent, lost
%     returnTo, ret step to come back to after re-running the lost
%                   background step, and what to restore there
%     prevState     state to return to on resume
%     checks        ticked items of the current step
%     values        parameter key -> value
%     stepDone      done flag per step
%     timer         running, elapsed (s; NaN when never started)
%     volumeRef, volumeDischarged   mL
%     mixerOn       the procedure switched the mixer on
%     intent        operator intent waiting for the next call
%
%   Status codes are those of masterStateMachine.m: 0 idle, 1 homing, 2/3
%   position moving/at target, 4 jog, 5 pressure, 6 stopping, 7 starting,
%   8 going home, 10 fault, 20 E-STOP, 30 not confirmed, 33 stop timeout,
%   34/35 slave link lost/waiting.

STATUS_FAULT     = 10;
STATUS_ESTOP     = 20;
STATUS_LINK_LOST = 34;
STATUS_LINK_WAIT = 35;

req = emptyReq();
c = p.c;
status = in.statusCode;

% ----- Terminal or idle -----
if any(strcmp(p.state, {'idle', 'done', 'faulted'}))
    if strcmp(p.state, 'idle') && strcmp(p.intent, 'start')
        p.intent = '';
        why = '';
        if ~in.connected
            why = 'The model is not running';
        elseif status == STATUS_LINK_LOST || status == STATUS_LINK_WAIT
            why = 'The slave link is not up';
        elseif in.estop || status == STATUS_ESTOP
            why = 'E-STOP is active';
        elseif status ~= 0
            why = sprintf('The machine is not idle (status %d)', status);
        end
        if ~isempty(why)
            p.message = why;
            return
        end
        p = freshRun(p);
        [p, req] = enterStep(p, req, in, 1);
    else
        p.intent = '';
    end
    return
end

% ----- Clocks -----
if p.timer.running
    p.timer.elapsed = p.timer.elapsed + dt;
end
if ~isnan(p.volumeRef)
    p.volumeDischarged = p.volumeRef - in.volume;
end
if ~strcmp(p.state, 'paused')
    p.stepT = p.stepT + dt;
end

% ----- Supervision -----
% E-STOP, a lost link and a stopped model pause the procedure: the master
% has stopped everything already, so no stop is sent. A slave fault ends it.
cause = supervisionCause(in);
if strcmp(p.state, 'aborting')
    if ~in.connected
        [p, req] = finishAbort(p, req, 'Model stopped');
        return
    end
elseif ~isempty(cause)
    p = supervisionPause(p, cause);
elseif strcmp(p.state, 'paused') && ~isempty(p.reason)
    p.reason = '';
    p.message = 'The condition has cleared: resume or abort';
end
if status == STATUS_FAULT && in.connected && isempty(cause)
    [p, req] = endFault(p, req, sprintf('Slave fault %d', in.faultCode), ...
        in.faultCode);
    return
end

% ----- Background operation -----
if p.bg.op ~= 0 && ~p.bg.lost && isempty(cause)
    p.bg.sinceSent = p.bg.sinceSent + dt;
    [p.bg.confirmed, ended] = opTrack(p.bg.op, p.bg.confirmed, status, ...
        p.bg.sinceSent, c);
    if ended
        if engineStopping(p)
            p.bg = emptyBg();
        else
            p.bg.lost = true;
            [p, req] = pauseNow(p, req, sprintf('%s ended (status %d)', ...
                opLabel(p.bg.op), status), false);
        end
    end
end

% ----- Intents -----
intent = p.intent;
p.intent = '';
if ~isempty(intent)
    [p, req] = doIntent(p, req, in, intent, cause);
    return
end

% ----- State work -----
if strcmp(p.state, 'aborting')
    if p.stepT >= c.tStopMin && ~busy(status)
        [p, req] = finishAbort(p, req, '');
    elseif p.stepT >= c.tStopMax + 0.5
        [p, req] = finishAbort(p, req, 'Stop not confirmed');
    end
elseif strcmp(p.state, 'running')
    st = p.def.steps(p.k);
    if strcmp(st.type, 'machine')
        [p, req] = machineWork(p, req, in, dt, st);
    elseif strcmp(st.type, 'wait')
        [p, req] = waitWork(p, req, in, dt, st);
    end
end
end

% ===== Intents =====

function [p, req] = doIntent(p, req, in, intent, cause)
% Act on the operator's intent if it is still valid in the current state.
st = p.def.steps(max(p.k, 1));
switch intent
    case 'abort'
        if any(strcmp(p.state, {'running', 'waitOperator', 'paused'})) ...
                && ~in.connected
            [p, req] = finishAbort(p, req, 'Model stopped');
        elseif any(strcmp(p.state, {'running', 'waitOperator', 'paused'}))
            p.state = 'aborting';
            p.reason = '';
            p.stepT = 0;
            p.message = '';
            req.stop = true;
        end
    case 'pause'
        if any(strcmp(p.state, {'running', 'waitOperator'}))
            [p, req] = pauseStep(p, req, '', true);
        end
    case 'resume'
        if strcmp(p.state, 'paused')
            if isempty(cause)
                [p, req] = resumeRun(p, req, in);
            else
                p.message = sprintf('Cannot resume: %s', cause);
            end
        end
    case 'confirm'
        if strcmp(p.state, 'waitOperator')
            if strcmp(st.type, 'param')
                req.params = sinkParams(p, st);
            end
            [p, req] = completeStep(p, req, in);
        end
    case 'repeat'
        if strcmp(p.state, 'waitOperator') && ~isempty(st.repeatFrom)
            j = find(strcmp(st.repeatFrom, {p.def.steps.id}), 1);
            p.stepDone(j:end) = false;
            p.returnTo = 0;
            [p, req] = enterStep(p, req, in, j);
        end
end
end

function cause = supervisionCause(in)
% Why the procedure cannot run now, or ''.
cause = '';
if ~in.connected
    cause = 'Model stopped';
elseif in.statusCode == 34 || in.statusCode == 35
    cause = 'Slave link lost';
elseif in.estop || in.statusCode == 20
    cause = 'E-STOP';
end
end

function p = supervisionPause(p, cause)
% Pause because of the cause: whatever was running is gone, so the current
% machine step is to be issued again and a kept operation counts as lost.
% Nothing is sent. While the cause lasts, reason says why.
if any(strcmp(p.state, {'running', 'waitOperator'}))
    p.prevState = p.state;
    p.state = 'paused';
    st = p.def.steps(p.k);
    if strcmp(st.type, 'machine') && ~strcmp(p.phase, 'stopping')
        p.phase = 'issue';
        p.track = emptyTrack();
    end
end
if strcmp(p.state, 'paused') && isempty(p.reason)
    if p.bg.op ~= 0
        p.bg.lost = true;
    end
    p.reason = cause;
    p.message = sprintf('%s. Resume once cleared, or abort', cause);
elseif strcmp(p.state, 'paused')
    p.reason = cause;
end
end

function [p, req] = resumeRun(p, req, in)
% Leave pause: re-run a lost background step, or continue the current one.
p.message = '';
if p.bg.lost && p.bg.step < p.k
    p.ret = struct('state', p.prevState, 'checks', p.checks, ...
        'stepT', p.stepT, 'phase', p.phase);
    p.returnTo = p.k;
    j = p.bg.step;
    p.bg = emptyBg();
    [p, req] = enterStep(p, req, in, j);
    return
end
if p.bg.lost
    p.bg = emptyBg();
end
p.state = p.prevState;
st = p.def.steps(p.k);
if strcmp(st.type, 'machine')
    if strcmp(p.phase, 'stopping')
        req.stop = true;
    else
        p.phase = 'issue';
        p.track = emptyTrack();
    end
    p.stepT = 0;
    p.settleT = 0;
end
end

function [p, req] = pauseStep(p, req, msg, allowStop)
% Pause. A machine step's own operation is stopped, unless the step keeps
% it, in which case it carries on as the background operation.
if strcmp(p.state, 'running')
    st = p.def.steps(p.k);
    if strcmp(st.type, 'machine') && any(strcmp(p.phase, {'track', 'settle'}))
        if st.keep && strcmp(p.phase, 'track')
            p.bg = struct('op', p.track.op, 'target', p.track.target, ...
                'step', p.k, 'confirmed', p.track.confirmed, ...
                'sinceSent', p.track.sinceSent, 'lost', false);
        elseif ~st.keep && allowStop
            req.stop = true;
        end
        p.phase = 'issue';
    end
end
[p, req] = pauseNow(p, req, msg, false);
end

function [p, req] = pauseNow(p, req, msg, doStop)
% Enter the paused state, remembering the one to return to.
if ~strcmp(p.state, 'paused')
    p.prevState = p.state;
    p.state = 'paused';
end
p.message = msg;
if doStop
    req.stop = true;
end
end

% ===== Machine and wait steps =====

function [p, req] = machineWork(p, req, in, dt, st)
status = in.statusCode;
c = p.c;
switch p.phase
    case 'issue'
        if strcmp(st.op, 'stop')
            req.stop = true;
            p.phase = 'stopping';
            p.stepT = 0;
            return
        end
        code = opCode(st.op);
        target = NaN;
        req.unit = '';
        if strcmp(st.op, 'position')
            target = resolveTarget(p, st);
            req.unit = st.unit;
        elseif strcmp(st.op, 'pressure')
            target = resolveTarget(p, st);
            req.unit = 'atm';
        end
        req.op = code;
        req.target = target;
        % The master runs one operation: the same one is retargeted live
        % and keeps its confirmation, a different one replaces it.
        carry = p.bg.op == code && ~p.bg.lost;
        confirmed = carry && p.bg.confirmed;
        p.bg = emptyBg();
        p.track = struct('op', code, 'confirmed', confirmed, ...
            'sinceSent', 0, 'target', target);
        p.phase = 'track';
        p.settleT = 0;
        p.settleOk = false;
    case 'track'
        tr = p.track;
        tr.sinceSent = tr.sinceSent + dt;
        [tr.confirmed, ended] = opTrack(tr.op, tr.confirmed, status, ...
            tr.sinceSent, c);
        p.track = tr;
        late = tr.sinceSent >= c.procStatusDelay;
        done = false;
        switch st.op
            case 'position'
                done = late && status == 3;
            case 'pressure'
                if strcmp(st.until, 'running')
                    done = late && tr.confirmed;
                else
                    if late && tr.confirmed && ...
                            abs(in.pressure - tr.target) <= st.tolerance
                        p.settleT = p.settleT + dt;
                    else
                        p.settleT = 0;
                    end
                    done = p.settleT >= st.duration;
                end
        end
        if done
            if st.keep
                p.bg = struct('op', tr.op, 'target', tr.target, ...
                    'step', p.k, 'confirmed', true, ...
                    'sinceSent', tr.sinceSent, 'lost', false);
                [p, req] = completeStep(p, req, in);
            else
                p.phase = 'stopping';
                p.stepT = 0;
                req.stop = true;
            end
        elseif ended
            p.phase = 'settle';
            p.settleOk = tr.confirmed && any(strcmp(st.op, {'home', 'goHome'}));
        elseif p.stepT >= st.timeout
            [p, req] = pauseStep(p, req, timeoutMsg(st), true);
            p.stepT = 0;
            p.settleT = 0;
        end
    case 'settle'
        if ~busy(status)
            if p.settleOk && status == 0
                [p, req] = completeStep(p, req, in);
            else
                [p, req] = pauseStep(p, req, endedMsg(st, status), false);
            end
        elseif p.stepT >= st.timeout
            [p, req] = pauseStep(p, req, timeoutMsg(st), true);
            p.stepT = 0;
        end
    case 'stopping'
        if p.stepT >= c.tStopMin && ~busy(status)
            if strcmp(st.op, 'stop')
                p.bg = emptyBg();
            end
            [p, req] = completeStep(p, req, in);
        elseif p.stepT >= c.tStopMax + 0.5
            [p, req] = pauseNow(p, req, 'Stop not confirmed', false);
        end
end
end

function [p, req] = waitWork(p, req, in, dt, st)
if isempty(st.until)
    if p.stepT >= st.duration
        [p, req] = completeStep(p, req, in);
    end
    return
end
% Settled about the background pressure target.
if p.bg.op ~= 4 || p.bg.lost
    [p, req] = pauseNow(p, req, 'No pressure control to settle on', false);
    return
end
if abs(in.pressure - p.bg.target) <= st.tolerance
    p.settleT = p.settleT + dt;
else
    p.settleT = 0;
end
if p.settleT >= st.duration
    [p, req] = completeStep(p, req, in);
elseif p.stepT >= st.timeout
    [p, req] = pauseNow(p, req, sprintf( ...
        'Pressure did not settle within %g s', st.timeout), false);
    p.stepT = 0;
    p.settleT = 0;
end
end

% ===== Entering, completing and ending =====

function [p, req] = enterStep(p, req, in, k)
% Start step k: reset its clocks and apply its entry actions.
st = p.def.steps(k);
p.k = k;
p.stepT = 0;
p.phase = 'issue';
p.track = emptyTrack();
p.settleT = 0;
p.settleOk = false;
p.message = '';
p.checks = false(1, numel(st.checks));
if st.timerStop
    p.timer.running = false;
end
if st.zeroVolume
    p.volumeRef = in.volume;
    p.volumeDischarged = 0;
    req.zeroVolume = true;
end
if ~isempty(st.mixer)
    switch st.mixer
        case 'on',  m = 1;
        case 'off', m = 0;
        otherwise,  m = double(p.values.(st.mixer) ~= 0);
    end
    req.mixer = m;
    p.mixerOn = m == 1;
end
if any(strcmp(st.type, {'confirm', 'param'}))
    p.state = 'waitOperator';
else
    p.state = 'running';
end
end

function [p, req] = completeStep(p, req, in)
% Step k is done: start its timer, then go to the next step (or back to
% the step that was interrupted by re-running a lost background step).
st = p.def.steps(p.k);
if st.timerStart && p.returnTo == 0
    p.timer.running = true;
    p.timer.elapsed = 0;
end
p.stepDone(p.k) = true;
if p.returnTo > 0
    p.k = p.returnTo;
    p.returnTo = 0;
    p.state = p.ret.state;
    p.checks = p.ret.checks;
    p.stepT = p.ret.stepT;
    p.phase = p.ret.phase;
    p.track = emptyTrack();
    p.message = '';
    return
end
if p.k >= numel(p.def.steps)
    p.state = 'done';
    p.outcome = 'completed';
    p.reason = '';
    p.message = '';
    p.timer.running = false;
    if p.bg.op ~= 0 && ~p.bg.lost
        req.stop = true;
    end
    p.bg = emptyBg();
    [p, req] = mixerOff(p, req);
    return
end
[p, req] = enterStep(p, req, in, p.k + 1);
end

function [p, req] = endFault(p, req, reason, code)
% Terminal: the master is latched or in E-STOP, so no operation is sent.
p.state = 'faulted';
p.outcome = 'fault';
p.reason = reason;
p.message = reason;
p.faultCode = code;
p.timer.running = false;
p.bg = emptyBg();
p.intent = '';
[p, req] = mixerOff(p, req);
end

function [p, req] = finishAbort(p, req, reason)
p.state = 'done';
p.outcome = 'aborted';
p.reason = reason;
p.message = reason;
p.timer.running = false;
p.bg = emptyBg();
[p, req] = mixerOff(p, req);
end

function [p, req] = mixerOff(p, req)
% Switch the stir bar off if the procedure switched it on.
if p.mixerOn
    req.mixer = 0;
    p.mixerOn = false;
end
end

function p = freshRun(p)
p.outcome = '';
p.reason = '';
p.faultCode = 0;
p.message = '';
p.k = 0;
p.stepDone = false(1, numel(p.def.steps));
p.timer = struct('running', false, 'elapsed', NaN);
p.volumeRef = NaN;
p.volumeDischarged = NaN;
p.bg = emptyBg();
p.returnTo = 0;
p.mixerOn = false;
p.prevState = '';
end

% ===== Helpers =====

function req = emptyReq()
req = struct('op', -1, 'target', NaN, 'unit', '', 'stop', false, ...
    'mixer', -1, 'params', [], 'zeroVolume', false);
end

function t = emptyTrack()
t = struct('op', 0, 'confirmed', false, 'sinceSent', 0, 'target', NaN);
end

function b = emptyBg()
b = struct('op', 0, 'target', NaN, 'step', 0, 'confirmed', false, ...
    'sinceSent', 0, 'lost', false);
end

function tf = busy(status)
% The master is homing, moving, jogging, holding pressure, stopping or
% starting.
tf = status >= 1 && status <= 8;
end

function tf = engineStopping(p)
% The engine itself is stopping the operation, so its end is expected.
tf = strcmp(p.state, 'aborting');
if ~tf && p.k >= 1
    st = p.def.steps(p.k);
    tf = strcmp(p.phase, 'stopping') || ...
        (strcmp(st.type, 'machine') && strcmp(st.op, 'stop'));
end
end

function code = opCode(name)
% RequestedMode of the operation (README, "App to model interface").
switch name
    case 'home',     code = 1;
    case 'position', code = 2;
    case 'pressure', code = 4;
    case 'goHome',   code = 6;
    otherwise,       code = 0;
end
end

function s = opLabel(code)
switch code
    case 1, s = 'Home';
    case 2, s = 'Position move';
    case 4, s = 'Pressure control';
    case 6, s = 'Go home';
    otherwise, s = 'Operation';
end
end

function v = resolveTarget(p, st)
% A target is a number or the key of a parameter.
if ischar(st.target)
    v = p.values.(st.target);
else
    v = st.target;
end
end

function s = sinkParams(p, st)
% The parameters of a param step that feed a shell Constant block.
s = [];
for i = 1:numel(st.params)
    q = p.def.params(strcmp(st.params{i}, {p.def.params.key}));
    if ~isempty(q.sink)
        if isempty(s)
            s = struct();
        end
        s.(q.sink) = p.values.(q.key);
    end
end
end

function m = timeoutMsg(st)
m = sprintf('Step did not complete within %g s', st.timeout);
end

function m = endedMsg(st, status)
% Why a machine step's operation ended before its condition was met.
switch status
    case 30
        m = sprintf('%s was not confirmed by the slave (status 30)', st.title);
    case 33
        m = sprintf('%s: the slave did not report stopped (status 33)', st.title);
    otherwise
        m = sprintf('%s ended early (status %d)', st.title, status);
end
end
