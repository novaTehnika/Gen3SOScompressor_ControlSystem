function [p, ok, msg] = procAction(p, action, varargin)
%PROCACTION Record an operator action on the procedure engine.
%   [p, ok, msg] = procAction(p, action)
%   [p, ok, msg] = procAction(p, 'check', i, tf)
%   [p, ok, msg] = procAction(p, 'setParam', key, value)
%
%   action   'start' (idle), 'confirm' (waitOperator: every check ticked, or
%            every parameter of the step in range), 'repeat' (waitOperator
%            on a step with repeatFrom), 'check' (tick or untick item i of
%            the current confirm step), 'setParam' (set a parameter the
%            current param step lists), 'pause' (running or waitOperator),
%            'resume' (paused), 'abort' (running, waitOperator or paused)
%            and 'reset' (done: back to idle, parameter values kept)
%
%   Nothing here touches the machine. The action is recorded as an intent
%   (or, for check, setParam and reset, applied to the state) and procStep
%   acts on the intent at its next call. When the action is not valid in the
%   current state, ok is false, msg says why and p is returned unchanged.

ok = false;
msg = '';
active = any(strcmp(p.state, {'running', 'waitOperator', 'paused'}));
if p.k >= 1
    st = p.def.steps(p.k);
else
    st = [];
end
if ~isempty(p.intent) && ~strcmp(action, 'abort') && ~strcmp(action, 'reset')
    msg = 'Another action is waiting to be processed';
    return
end

switch action
    case 'start'
        if strcmp(p.state, 'idle')
            p.intent = 'start';
            ok = true;
        else
            msg = 'The procedure is already started';
        end
    case 'confirm'
        if ~strcmp(p.state, 'waitOperator')
            msg = 'Nothing to confirm now';
        elseif strcmp(st.type, 'confirm') && ~all(p.checks)
            msg = 'Tick every item first';
        else
            bad = '';
            if strcmp(st.type, 'param')
                bad = outOfRange(p, st);
            end
            if isempty(bad)
                p.intent = 'confirm';
                ok = true;
            else
                msg = sprintf('%s is out of range', bad);
            end
        end
    case 'repeat'
        if strcmp(p.state, 'waitOperator') && ~isempty(st.repeatFrom)
            p.intent = 'repeat';
            ok = true;
        else
            msg = 'There is nothing to repeat here';
        end
    case 'check'
        i = varargin{1};
        tf = varargin{2};
        if strcmp(p.state, 'waitOperator') && strcmp(st.type, 'confirm') ...
                && isnumeric(i) && isscalar(i) && i >= 1 && i <= numel(p.checks) ...
                && i == fix(i)
            p.checks(i) = logical(tf);
            ok = true;
        else
            msg = 'There is no checklist to tick here';
        end
    case 'setParam'
        key = varargin{1};
        value = varargin{2};
        if ~(strcmp(p.state, 'waitOperator') && strcmp(st.type, 'param') ...
                && ischar(key) && any(strcmp(key, st.params)))
            msg = 'This parameter cannot be changed now';
        elseif ~(isnumeric(value) && isscalar(value) && isreal(value) ...
                && ~isnan(value))
            msg = 'The value must be a number';
        else
            q = p.def.params(strcmp(key, {p.def.params.key}));
            if value < q.min || value > q.max
                if strcmp(q.kind, 'pressure')
                    % The range is in atm; the app checks in its own unit.
                    msg = sprintf('%s is out of range', q.label);
                else
                    msg = sprintf('%s must be between %g and %g', q.label, ...
                        q.min, q.max);
                end
            else
                p.values.(key) = value;
                ok = true;
            end
        end
    case 'pause'
        if any(strcmp(p.state, {'running', 'waitOperator'}))
            p.intent = 'pause';
            ok = true;
        else
            msg = 'The procedure is not running';
        end
    case 'resume'
        if strcmp(p.state, 'paused')
            p.intent = 'resume';
            ok = true;
        else
            msg = 'The procedure is not paused';
        end
    case 'abort'
        if active
            p.intent = 'abort';
            ok = true;
        else
            msg = 'The procedure is not active';
        end
    case 'reset'
        if strcmp(p.state, 'done')
            p = procNew(p.def, p.cfg, p.values);
            ok = true;
        else
            msg = 'The procedure has not ended';
        end
    otherwise
        msg = sprintf('Unknown action "%s"', action);
end
end

function bad = outOfRange(p, st)
% The label of the first parameter of the step outside its range, or ''.
bad = '';
for i = 1:numel(st.params)
    q = p.def.params(strcmp(st.params{i}, {p.def.params.key}));
    v = p.values.(q.key);
    if v < q.min || v > q.max
        bad = q.label;
        return
    end
end
end
