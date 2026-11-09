function p = procNew(proc, cfg, values)
%PROCNEW Normalize and validate a procedure and return the idle engine state.
%   p = procNew(proc, cfg)
%   p = procNew(proc, cfg, values)
%
%   proc    the struct a procedure file returns (master-src/procedure), or a
%           procedure that procNew has already normalized (p.def)
%   cfg     masterConfig (opConfirmWait, procStatusDelay, procPressureTol,
%           procSettleTime, procEndVolume, procMaxChecks, procMaxParams,
%           tStopMin, tStopMax); kept whole in p.cfg for target functions
%   values  optional struct, parameter key -> value, that replaces the
%           parameters' defaults (the values entered in the previous run)
%
%   proc has name (char), params (cell array of structs), steps (cell array
%   of structs) and optional timerLabel, volumeLabel and record (the trace
%   interval in s, or 'off'; default cfg.procRecordDefault). A parameter gives
%   key and value, and optionally label, unit, min, max, format, sink
%   ('', 'SalineVolume' or 'OxygenRatio') and kind ('' or 'pressure': its
%   value, min and max are atm absolute, shown in the display unit, and it
%   takes no unit of its own). A step gives type ('confirm', 'param',
%   'machine' or 'wait'), title and the fields of its type; see the
%   "Procedure (non-manual mode)" section of master-src/README.md. Fields a
%   step does not give take their defaults. New step fields: speed (mm/s,
%   a position step), pressureMax (atm absolute, a machine step) and
%   alsoWhen ('endOfTravel', a confirm step or a pressure step). A position
%   or pressure target may be a function handle @(values, cfg) returning a
%   finite scalar, evaluated when the step issues its operation. A step may
%   give record (a number above 0, or 'off'; absent: the procedure's), and a
%   timed wait or a track its duration as a parameter key. A track step
%   (op 'track') gives mode ('pressure' or 'position'), signal (a function
%   @(t, values) of the time column or a [t value] matrix), duration and
%   optionally relative; procNew samples the signal on the parameters'
%   defaults and checks it (procTrackSeries, procTrackCheck).
%
%   The engine state p holds the normalized procedure in p.def and the run
%   in the fields documented in procStep.m. procNew raises 'procNew:bad' for
%   a procedure that cannot run: unknown type, op, unit or until, a missing
%   title or target, a target or key that names no parameter, a duplicate
%   id, a repeatFrom that does not name an earlier step, too many checks or
%   parameters, an unknown sink or kind, a pressure parameter with a unit,
%   keep on a step that cannot hold, a speed, pressureMax or alsoWhen where
%   it does not apply, a target function that fails on the parameters'
%   defaults, a settled wait with no earlier pressure step that has keep, a
%   record that is not a number above 0 or 'off', a duration key that names
%   no parameter, a track without mode, signal or duration, a track with
%   target, unit, speed, alsoWhen or keep, mode, signal or relative on
%   another step, or a track signal that fails on the defaults or leaves
%   the pressure or position limits.

if nargin < 3
    values = struct();
end
if isfield(proc, 'normalized') && proc.normalized
    def = proc;
else
    def = normalizeProc(proc, cfg);
end

p.def = def;
p.c = struct('opConfirmWait', cfg.opConfirmWait, ...
    'procStatusDelay', cfg.procStatusDelay, ...
    'procPressureTol', cfg.procPressureTol, ...
    'procSettleTime', cfg.procSettleTime, ...
    'procEndVolume', cfg.procEndVolume, ...
    'tStopMin', cfg.tStopMin, 'tStopMax', cfg.tStopMax);
p.cfg = cfg;
p.state = 'idle';
p.outcome = '';
p.reason = '';
p.faultCode = 0;
p.message = '';
p.note = '';
p.k = 0;
p.stepT = 0;
p.phase = 'issue';
p.track = emptyTrack();
p.trk = emptyTrk();
p.record = '';
p.settleT = 0;
p.settleOk = false;
p.bg = emptyBg();
p.returnTo = 0;
p.ret = struct('state', '', 'checks', false(1, 0), 'stepT', 0, 'phase', 'issue');
p.prevState = '';
p.checks = false(1, 0);
p.values = struct();
for i = 1:numel(def.params)
    q = def.params(i);
    p.values.(q.key) = q.value;
end
vkeys = fieldnames(values);
for i = 1:numel(vkeys)
    v = values.(vkeys{i});
    if isfield(p.values, vkeys{i}) && isnumeric(v) && isscalar(v) && ~isnan(v)
        p.values.(vkeys{i}) = v;
    end
end
p.stepDone = false(1, numel(def.steps));
p.timer = struct('running', false, 'elapsed', NaN);
p.volumeRef = NaN;
p.volumeDischarged = NaN;
p.mixerOn = false;
p.intent = '';
end

function t = emptyTrack()
t = struct('op', 0, 'confirmed', false, 'sinceSent', 0, 'target', NaN);
end

function t = emptyTrk()
t = struct('on', false, 'borrow', false, 't', 0, 'offset', NaN, ...
    'duration', NaN);
end

function b = emptyBg()
b = struct('op', 0, 'target', NaN, 'step', 0, 'confirmed', false, ...
    'sinceSent', 0, 'lost', false);
end

function def = normalizeProc(proc, cfg)
% Fill the defaults of a procedure and check it.
if ~isfield(proc, 'name') || ~ischar(proc.name)
    error('procNew:bad', 'the procedure needs a name');
end
def.name = proc.name;
def.timerLabel = 'Time pressurized';
def.volumeLabel = 'Volume discharged (mL)';
if isfield(proc, 'timerLabel'), def.timerLabel = proc.timerLabel; end
if isfield(proc, 'volumeLabel'), def.volumeLabel = proc.volumeLabel; end
def.record = cfg.procRecordDefault;
if isfield(proc, 'record')
    if ~validRecord(proc.record)
        error('procNew:bad', 'record must be a number above 0 or ''off''');
    end
    def.record = proc.record;
end

% ----- Parameters -----
rawParams = asCell(proc, 'params');
if numel(rawParams) > cfg.procMaxParams
    error('procNew:bad', 'too many parameters (%d, at most %d)', ...
        numel(rawParams), cfg.procMaxParams);
end
tp = struct('key', '', 'label', '', 'unit', '', 'value', NaN, ...
    'min', -Inf, 'max', Inf, 'format', '%g', 'sink', '', 'kind', '');
params = repmat(tp, 1, 0);
for i = 1:numel(rawParams)
    r = rawParams{i};
    q = fillFields(tp, r, sprintf('parameter %d', i));
    if ~isfield(r, 'key') || ~ischar(q.key) || ~isvarname(q.key)
        error('procNew:bad', 'parameter %d: key must be a valid identifier', i);
    end
    if any(strcmp(q.key, {params.key}))
        error('procNew:bad', 'parameter %d: duplicate key %s', i, q.key);
    end
    if ~isfield(r, 'value') || ~isnumeric(q.value) || ~isscalar(q.value) ...
            || isnan(q.value)
        error('procNew:bad', 'parameter %s: value must be a numeric scalar', q.key);
    end
    if ~isfield(r, 'label'), q.label = q.key; end
    if ~any(strcmp(q.sink, {'', 'SalineVolume', 'OxygenRatio'}))
        error('procNew:bad', 'parameter %s: unknown sink %s', q.key, q.sink);
    end
    if ~any(strcmp(q.kind, {'', 'pressure'}))
        error('procNew:bad', 'parameter %s: unknown kind %s', q.key, ...
            num2str(q.kind));
    end
    if strcmp(q.kind, 'pressure') && ~isempty(q.unit)
        error('procNew:bad', ['parameter %s: a pressure parameter takes ' ...
            'its unit from the display'], q.key);
    end
    if q.value < q.min || q.value > q.max
        error('procNew:bad', 'parameter %s: value outside min and max', q.key);
    end
    params(end + 1) = q; %#ok<AGROW>
end
def.params = params;
keys = {params.key};
defaults = struct();
for i = 1:numel(params)
    defaults.(params(i).key) = params(i).value;
end

% ----- Steps -----
rawSteps = asCell(proc, 'steps');
if isempty(rawSteps)
    error('procNew:bad', 'the procedure has no steps');
end
ts = struct('id', '', 'section', '', 'title', '', 'text', '', 'type', '', ...
    'checks', {{}}, 'confirmLabel', 'Confirm', 'repeatFrom', '', ...
    'repeatLabel', 'Repeat', 'params', {{}}, 'op', '', 'target', [], ...
    'unit', '', 'until', '', 'keep', false, 'tolerance', NaN, ...
    'duration', NaN, 'timeout', Inf, 'timerStart', false, ...
    'timerStop', false, 'zeroVolume', false, 'mixer', '', ...
    'speed', NaN, 'pressureMax', Inf, 'alsoWhen', '', ...
    'mode', '', 'signal', [], 'relative', false, 'record', NaN);
steps = repmat(ts, 1, 0);
section = '';
pressureHold = false;   % an earlier pressure step has keep
for k = 1:numel(rawSteps)
    r = rawSteps{k};
    s = fillFields(ts, r, sprintf('step %d', k));
    if ~isfield(r, 'id'), s.id = sprintf('s%d', k); end
    if ~isfield(r, 'section'), s.section = section; end
    section = s.section;
    bad = @(fmt, varargin) error('procNew:bad', ['step %d (%s): ' fmt], ...
        k, s.id, varargin{:});
    if isempty(s.title), bad('missing title'); end
    if any(strcmp(s.id, {steps.id})), bad('duplicate id'); end
    if ~any(strcmp(s.type, {'confirm', 'param', 'machine', 'wait'}))
        bad('unknown type "%s"', s.type);
    end
    if numel(s.checks) > cfg.procMaxChecks
        bad('too many checks (%d, at most %d)', numel(s.checks), cfg.procMaxChecks);
    end
    if ~isempty(s.checks) && ~strcmp(s.type, 'confirm')
        bad('checks only apply to a confirm step');
    end
    for i = 1:numel(s.params)
        if ~any(strcmp(s.params{i}, keys))
            bad('param "%s" names no parameter', s.params{i});
        end
    end
    if ~isempty(s.params) && ~strcmp(s.type, 'param')
        bad('params only apply to a param step');
    end
    if strcmp(s.type, 'param') && isempty(s.params)
        bad('a param step lists at least one parameter');
    end
    if ~isempty(s.repeatFrom)
        if ~strcmp(s.type, 'confirm')
            bad('repeatFrom only applies to a confirm step');
        end
        if isempty(find(strcmp(s.repeatFrom, {steps.id}), 1))
            bad('repeatFrom "%s" is not an earlier step', s.repeatFrom);
        end
    end
    if ~isempty(s.mixer) && ~any(strcmp(s.mixer, [{'on', 'off'}, keys]))
        bad('mixer "%s" is neither on, off nor a parameter', s.mixer);
    end
    texts = [{s.title, s.text}, s.checks];
    for i = 1:numel(texts)
        tok = regexp(texts{i}, '\{(\w+)\}', 'tokens');
        for j = 1:numel(tok)
            if ~any(strcmp(tok{j}{1}, keys))
                bad('{%s} names no parameter', tok{j}{1});
            end
        end
    end
    if s.keep && ~(strcmp(s.type, 'machine') && ...
            any(strcmp(s.op, {'position', 'pressure'})))
        bad('keep only applies to a position or pressure step');
    end
    if ~any(strcmp(s.type, {'machine', 'wait'}))
        if ~isempty(s.until) || ~isequal(s.timeout, Inf)
            bad('until and timeout only apply to a machine or wait step');
        end
        if ~isempty(s.op) || ~isempty(s.target)
            bad('op and target only apply to a machine step');
        end
    end
    if strcmp(s.type, 'machine')
        if ~any(strcmp(s.op, {'home', 'goHome', 'position', 'pressure', ...
                'track', 'stop'}))
            bad('unknown op "%s"', s.op);
        end
        if any(strcmp(s.op, {'position', 'pressure'}))
            if isempty(s.target)
                bad('missing target');
            end
            if ischar(s.target)
                if ~any(strcmp(s.target, keys))
                    bad('target "%s" names no parameter', s.target);
                end
            elseif isa(s.target, 'function_handle')
                try
                    v = s.target(defaults, cfg);
                    good = isnumeric(v) && isscalar(v) && isfinite(v);
                    why = 'the result is not a finite number';
                catch err
                    good = false;
                    why = err.message;
                end
                if ~good
                    bad('target function fails on the default values: %s', why);
                end
            elseif ~(isnumeric(s.target) && isscalar(s.target) && isfinite(s.target))
                bad('target must be a number, a parameter key or a function');
            end
        elseif ~isempty(s.target)
            bad('target only applies to a position or pressure step');
        end
        if isfield(r, 'speed')
            if ~strcmp(s.op, 'position')
                bad('speed only applies to a position step');
            end
            if ~(isnumeric(s.speed) && isscalar(s.speed) && isfinite(s.speed) ...
                    && s.speed > 0)
                bad('speed must be a finite number above 0');
            end
        end
        if strcmp(s.op, 'position')
            if ~isfield(r, 'unit'), s.unit = 'mm'; end
            if ~any(strcmp(s.unit, {'mm', 'mL'}))
                bad('unknown unit "%s"', s.unit);
            end
        elseif ~isempty(s.unit)
            bad('unit only applies to a position step');
        end
        switch s.op
            case 'position', allowed = {'arrive'};
            case {'home', 'goHome'}, allowed = {'ended'};
            case 'pressure', allowed = {'running', 'settled'};
            case 'track', allowed = {'elapsed'};
            otherwise, allowed = {'stopped'};
        end
        if isempty(s.until), s.until = allowed{1}; end
        if ~any(strcmp(s.until, allowed))
            bad('unknown until "%s" for op %s', s.until, s.op);
        end
        if strcmp(s.op, 'pressure') && s.keep
            pressureHold = true;
        end
    elseif strcmp(s.type, 'wait')
        if ~any(strcmp(s.until, {'', 'settled'}))
            bad('unknown until "%s" for a wait', s.until);
        end
        if strcmp(s.until, 'settled') && ~pressureHold
            bad('a settled wait needs an earlier pressure step with keep');
        end
        if isempty(s.until)
            d = s.duration;
            if ischar(d)
                if ~any(strcmp(d, keys))
                    bad('duration "%s" names no parameter', d);
                end
                d = defaults.(d);
            end
            if ~(isnumeric(d) && isscalar(d) && isfinite(d) && d >= 0)
                bad('a timed wait needs a duration');
            end
        end
    end
    if ischar(s.duration) && ~((strcmp(s.type, 'wait') && isempty(s.until)) ...
            || (strcmp(s.type, 'machine') && strcmp(s.op, 'track')))
        bad('a duration key only applies to a timed wait or a track step');
    end
    isTrack = strcmp(s.type, 'machine') && strcmp(s.op, 'track');
    if ~isTrack && (isfield(r, 'mode') || isfield(r, 'signal') ...
            || isfield(r, 'relative'))
        bad('mode, signal and relative only apply to a track step');
    end
    if isTrack
        if ~any(strcmp(s.mode, {'pressure', 'position'}))
            bad('a track needs mode "pressure" or "position"');
        end
        if isempty(s.signal)
            bad('a track needs a signal');
        end
        if ~(islogical(s.relative) || (isnumeric(s.relative) && isscalar(s.relative)))
            bad('relative must be true or false');
        end
        s.relative = logical(s.relative);
        if isnumeric(s.duration) && isnan(s.duration)
            bad('a track needs a duration');
        end
        [raw, why] = procTrackSeries(s, defaults, cfg);
        if isempty(why)
            if strcmp(s.mode, 'position')
                limit = cfg.speedMax;
            else
                limit = cfg.trackPressureSlopeMax;
            end
            why = procTrackCheck(raw, s.mode, s.relative, cfg, limit);
        end
        if ~isempty(why)
            bad('track signal %s', why);
        end
    end
    if ~(isequaln(s.record, NaN) || validRecord(s.record))
        bad('record must be a number above 0 or ''off''');
    end
    if isfield(r, 'speed') && ~strcmp(s.type, 'machine')
        bad('speed only applies to a position step');
    end
    if isfield(r, 'pressureMax')
        if ~strcmp(s.type, 'machine')
            bad('pressureMax only applies to a machine step');
        end
        if ~(isnumeric(s.pressureMax) && isscalar(s.pressureMax) ...
                && ~isnan(s.pressureMax) && s.pressureMax > 0)
            bad('pressureMax must be a number above 0');
        end
    end
    if ~any(strcmp(s.alsoWhen, {'', 'endOfTravel'}))
        bad('unknown alsoWhen "%s"', num2str(s.alsoWhen));
    end
    if ~isempty(s.alsoWhen) && ~(strcmp(s.type, 'confirm') || ...
            (strcmp(s.type, 'machine') && strcmp(s.op, 'pressure')))
        bad('alsoWhen only applies to a confirm step or a pressure step');
    end
    if strcmp(s.until, 'settled')
        if ischar(s.duration)
            bad('a settled step takes its duration as a number');
        end
        if isnan(s.tolerance), s.tolerance = cfg.procPressureTol; end
        if isnan(s.duration), s.duration = cfg.procSettleTime; end
    end
    steps(k) = s;
end
def.steps = steps;
def.normalized = true;
end

function tf = validRecord(v)
% A record interval: a number above 0 (s) or 'off'.
tf = isequal(v, 'off') || (isnumeric(v) && isscalar(v) && isfinite(v) && v > 0);
end

function c = asCell(proc, name)
% A cell array of structs from a cell array or a struct array.
if ~isfield(proc, name)
    c = {};
    return
end
c = proc.(name);
if isstruct(c)
    c = num2cell(c);
end
if ~iscell(c)
    error('procNew:bad', '%s must be a cell array of structs', name);
end
c = c(:)';
end

function s = fillFields(template, raw, what)
% The template with the fields of raw laid over it; unknown fields are errors.
if ~isstruct(raw)
    error('procNew:bad', '%s must be a struct', what);
end
s = template;
f = fieldnames(raw);
for i = 1:numel(f)
    if ~isfield(template, f{i})
        error('procNew:bad', '%s: unknown field %s', what, f{i});
    end
    s.(f{i}) = raw.(f{i});
end
end
