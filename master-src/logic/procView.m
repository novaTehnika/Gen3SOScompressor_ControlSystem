function v = procView(p)
%PROCVIEW What the procedure panel shows for an engine state.
%   v = procView(p)
%
%   A pure function: the app copies the fields into its components.
%     stateText      one line on the engine's state
%     heading        'Step 4 of 19 - Saline purge/fill'
%     title, text    the step's title and instruction, with {key} replaced
%                    by the parameter value in its format
%     list           cellstr, one entry per step: '[x] title' (done),
%                    '> title' (current) or '  title'
%     listIndex      index of the current step in list (1 when none)
%     checks         labels (cellstr), values (logical) and enable
%     params         keys, labels (with unit), values, and editable (true
%                    only for the keys of the current param step)
%     confirm        visible, enable, label of the Confirm button
%     repeat         visible, label of the Repeat button
%     pause          enable, label ('Pause' or 'Resume'); Resume is
%                    disabled while a pause reason (E-STOP, link lost,
%                    model stopped) holds, and enabled once it clears
%     abortEnable, startEnable, active
%     timerText      elapsed time as m:ss, or '--' when never started
%     volumeText     volume discharged in mL (%.1f), or '--'
%     pressureTarget atm of the pressure the procedure holds, NaN if none
%     message        why the procedure paused or ended, or the last refusal

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
    case 'faulted'
        v.stateText = ['Stopped: ' p.reason];
    otherwise,           v.stateText = p.state;
end
if active
    v.heading = sprintf('Step %d of %d - %s', k, n, st.section);
    v.title = fillKeys(p, st.title);
    v.text = fillKeys(p, st.text);
elseif k >= 1
    v.heading = sprintf('Stopped at step %d of %d - %s', k, n, st.section);
    v.title = fillKeys(p, st.title);
    v.text = fillKeys(p, st.text);
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
    labels = cellfun(@(s) fillKeys(p, s), st.checks, 'UniformOutput', false);
end
vals = false(1, numel(labels));
if isConfirm && active
    vals = p.checks;
end
v.checks = struct('labels', {labels}, 'values', vals, ...
    'enable', strcmp(p.state, 'waitOperator'));
np = numel(d.params);
keys = cell(1, np);
plabels = cell(1, np);
pvals = zeros(1, np);
editable = false(1, np);
for i = 1:np
    q = d.params(i);
    keys{i} = q.key;
    if isempty(q.unit)
        plabels{i} = q.label;
    else
        plabels{i} = sprintf('%s (%s)', q.label, q.unit);
    end
    pvals(i) = p.values.(q.key);
    editable(i) = isParam && strcmp(p.state, 'waitOperator') && ...
        any(strcmp(q.key, st.params));
end
v.params = struct('keys', {keys}, 'labels', {plabels}, 'values', pvals, ...
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
v.startEnable = any(strcmp(p.state, {'idle', 'done', 'faulted'}));
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
if p.bg.op == 4 && ~p.bg.lost
    v.pressureTarget = p.bg.target;
end
v.message = p.message;
end

function s = fillKeys(p, s)
% Replace {key} by the parameter's value in its format.
for i = 1:numel(p.def.params)
    q = p.def.params(i);
    s = strrep(s, ['{' q.key '}'], sprintf(q.format, p.values.(q.key)));
end
end
