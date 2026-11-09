function v = procView(p, opts)
%PROCVIEW What the procedure panel shows for an engine state.
%   v = procView(p)
%   v = procView(p, opts)
%
%   opts  how pressure parameters and the held pressure are shown: the
%         struct of pressureUnitInfo (label, format, toUnit); the default
%         shows atm absolute
%
%   A pure function: the app copies the fields into its components.
%     stateText      one line on the engine's state
%     heading        'Step 4 of 19 - Saline purge/fill'
%     title, text    the step's title and instruction, with {key} replaced
%                    by the parameter value in its format; a pressure
%                    parameter is converted and followed by its unit label
%     list           cellstr, one entry per step: '[x] title' (done),
%                    '> title' (current) or '  title'
%     listIndex      index of the current step in list (1 when none)
%     checks         labels (cellstr), values (logical) and enable
%     params         keys, kinds, labels (with unit), values, formats, mins,
%                    maxs (a pressure parameter's in the display unit) and
%                    editable (true only for the keys of the current param
%                    step)
%     confirm        visible, enable, label of the Confirm button
%     repeat         visible, label of the Repeat button
%     pause          enable, label ('Pause' or 'Resume'); Resume is
%                    disabled while a pause reason (E-STOP, link lost,
%                    model stopped, slave fault) holds, and enabled once it
%                    clears
%     abortEnable, startEnable, active
%     timerText      elapsed time as m:ss, or '--' when never started
%     volumeText     volume discharged in mL (%.1f), or '--'
%     pressureTarget atm of the pressure the procedure holds, NaN if none
%     pressureTargetText  that pressure in the display unit with its label,
%                    or '--'
%     stepProgress   'm:ss / m:ss' (elapsed of total) for a timed wait and
%                    for a track step once issued, else ''
%     message        why the procedure paused or ended, or the last refusal;
%                    the note of a step that ended on its own when there is
%                    none

if nargin < 2
    opts = struct('label', 'atm abs', 'format', '%.2f', 'toUnit', @(x) x);
end
d = p.def;
n = numel(d.steps);
k = p.k;
active = any(strcmp(p.state, {'running', 'waitOperator', 'paused', 'aborting'}));
operator = any(strcmp(p.state, {'waitOperator', 'paused'}));
if k >= 1
    st = d.steps(k);
else
    st = [];
end

% ----- State and step text -----
switch p.state
    case 'idle',         v.stateText = 'Ready';
    case 'running',      v.stateText = 'Running';
    case 'waitOperator', v.stateText = 'Waiting for the operator';
    case 'paused'
        if isempty(p.reason)
            v.stateText = 'Paused';
        else
            v.stateText = ['Paused: ' p.reason];
        end
    case 'aborting',     v.stateText = 'Stopping';
    case 'done'
        if strcmp(p.outcome, 'completed')
            v.stateText = 'Completed';
        else
            v.stateText = 'Aborted';
        end
    otherwise,           v.stateText = p.state;
end
if active
    v.heading = sprintf('Step %d of %d - %s', k, n, st.section);
    v.title = fillKeys(p, opts, st.title);
    v.text = fillKeys(p, opts, st.text);
elseif k >= 1
    v.heading = sprintf('Stopped at step %d of %d - %s', k, n, st.section);
    v.title = fillKeys(p, opts, st.title);
    v.text = fillKeys(p, opts, st.text);
else
    v.heading = d.name;
    v.title = '';
    v.text = '';
end

% ----- Step list -----
v.list = cell(n, 1);
for i = 1:n
    if active && i == k
        mark = '> ';
    elseif p.stepDone(i)
        mark = '[x] ';
    else
        mark = '  ';
    end
    v.list{i} = [mark d.steps(i).title];
end
v.listIndex = max(k, 1);

% ----- Checklist and parameters -----
isConfirm = ~isempty(st) && strcmp(st.type, 'confirm');
isParam = ~isempty(st) && strcmp(st.type, 'param');
labels = cell(1, 0);
if isConfirm && active
    labels = cellfun(@(s) fillKeys(p, opts, s), st.checks, 'UniformOutput', false);
end
vals = false(1, numel(labels));
if isConfirm && active
    vals = p.checks;
end
v.checks = struct('labels', {labels}, 'values', vals, ...
    'enable', strcmp(p.state, 'waitOperator'));
np = numel(d.params);
keys = cell(1, np);
kinds = cell(1, np);
plabels = cell(1, np);
pvals = zeros(1, np);
formats = cell(1, np);
mins = zeros(1, np);
maxs = zeros(1, np);
editable = false(1, np);
for i = 1:np
    q = d.params(i);
    keys{i} = q.key;
    kinds{i} = q.kind;
    if strcmp(q.kind, 'pressure')
        plabels{i} = sprintf('%s (%s)', q.label, opts.label);
        formats{i} = opts.format;
        pvals(i) = opts.toUnit(p.values.(q.key));
        mins(i) = opts.toUnit(q.min);
        maxs(i) = opts.toUnit(q.max);
    else
        if isempty(q.unit)
            plabels{i} = q.label;
        else
            plabels{i} = sprintf('%s (%s)', q.label, q.unit);
        end
        formats{i} = q.format;
        pvals(i) = p.values.(q.key);
        mins(i) = q.min;
        maxs(i) = q.max;
    end
    editable(i) = isParam && strcmp(p.state, 'waitOperator') && ...
        any(strcmp(q.key, st.params));
end
v.params = struct('keys', {keys}, 'kinds', {kinds}, 'labels', {plabels}, ...
    'values', pvals, 'formats', {formats}, 'mins', mins, 'maxs', maxs, ...
    'editable', editable);

% ----- Buttons -----
enable = false;
if strcmp(p.state, 'waitOperator')
    if isConfirm
        enable = all(p.checks);
    elseif isParam
        enable = true;
        for i = 1:numel(st.params)
            q = d.params(strcmp(st.params{i}, keys));
            value = p.values.(q.key);
            if value < q.min || value > q.max
                enable = false;
            end
        end
    end
end
label = 'Confirm';
if ~isempty(st)
    label = st.confirmLabel;
end
v.confirm = struct('visible', operator && (isConfirm || isParam), ...
    'enable', enable, 'label', label);
hasRepeat = isConfirm && ~isempty(st.repeatFrom);
rlabel = 'Repeat';
if hasRepeat
    rlabel = st.repeatLabel;
end
v.repeat = struct('visible', operator && hasRepeat, 'label', rlabel);
if strcmp(p.state, 'paused')
    v.pause = struct('enable', isempty(p.reason), 'label', 'Resume');
else
    v.pause = struct('enable', any(strcmp(p.state, {'running', 'waitOperator'})), ...
        'label', 'Pause');
end
v.abortEnable = any(strcmp(p.state, {'running', 'waitOperator', 'paused'}));
v.startEnable = any(strcmp(p.state, {'idle', 'done'}));
v.active = active;

% ----- Readouts -----
if isnan(p.timer.elapsed)
    v.timerText = '--';
else
    s = floor(p.timer.elapsed);
    v.timerText = sprintf('%d:%02d', floor(s / 60), mod(s, 60));
end
if isnan(p.volumeRef) || isnan(p.volumeDischarged)
    v.volumeText = '--';
else
    v.volumeText = sprintf('%.1f', p.volumeDischarged);
end
v.pressureTarget = NaN;
v.pressureTargetText = '--';
if p.bg.op == 4 && ~p.bg.lost
    v.pressureTarget = p.bg.target;
    v.pressureTargetText = sprintf([opts.format ' %s'], ...
        opts.toUnit(p.bg.target), opts.label);
end
v.message = p.message;
if isempty(v.message)
    v.message = p.note;
end

% ----- Step progress -----
v.stepProgress = '';
if active && ~isempty(st)
    if strcmp(st.type, 'wait') && isempty(st.until)
        total = st.duration;
        if ischar(total)
            total = p.values.(total);
        end
        v.stepProgress = progressText(p.stepT, total);
    elseif strcmp(st.type, 'machine') && strcmp(st.op, 'track') ...
            && ~isnan(p.trk.duration)
        v.stepProgress = progressText(p.trk.t, p.trk.duration);
    end
end
end

function s = progressText(elapsed, total)
% 'm:ss / m:ss', the elapsed time as the timer shows it (whole seconds
% down) held at the total, the total rounded up.
e = floor(min(elapsed, total));
t = ceil(total);
s = sprintf('%d:%02d / %d:%02d', floor(e / 60), mod(e, 60), ...
    floor(t / 60), mod(t, 60));
end

function s = fillKeys(p, opts, s)
% Replace {key} by the parameter's value in its format; a pressure
% parameter is converted to the display unit and followed by its label.
for i = 1:numel(p.def.params)
    q = p.def.params(i);
    if strcmp(q.kind, 'pressure')
        t = sprintf([opts.format ' %s'], opts.toUnit(p.values.(q.key)), ...
            opts.label);
    else
        t = sprintf(q.format, p.values.(q.key));
    end
    s = strrep(s, ['{' q.key '}'], t);
end
end
