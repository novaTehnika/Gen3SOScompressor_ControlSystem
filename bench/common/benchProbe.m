function [p, caps] = benchProbe(cmd, p, varargin)
%BENCHPROBE Master side of PRG_StepProbe: arm captures, read them out of
%   the slave's ring buffer through its 18-record window, and decode them.
%   Shared by frictionRun and charRun.
%
%   p = benchProbe('new', cfgProbe, scan)
%       cfgProbe is benchConfig().probe, scan the slave's scan period [s],
%       which sets the t column of a capture.
%   p = benchProbe('arm', p, info)
%       Queue a capture. info.step and info.tag label it, info.nRecords is
%       its length in records (default cfgProbe.nScan; Inf for a stream
%       that runs until 'close' or 'drain'), info.nPre the records taken
%       from before the arm (default cfgProbe.nPre): the capture is the
%       records from mark - nPre on, nRecords of them in all.
%   p = benchProbe('close', p)
%       End every open stream at the newest record seen.
%   [p, caps] = benchProbe('slot', p, link, tNow)
%       The work of one gap between protocol exchanges: at most one FC04
%       read of IR401-520 and at most one write (the next arm, a repeat of
%       an unacknowledged one, or the next window request), the read first
%       so the slave has the whole next cycle to act on the write. caps is
%       a cell array of the captures finished by this read.
%   [p, caps] = benchProbe('drain', p, link, env)
%       Blocking: closes the streams, then slots every cfgProbe.drainGap
%       (env.now and env.wait(tTarget) are the run's clock functions)
%       until nothing is pending or cfgProbe.timeout passes without
%       progress.
%
%   p.armBusy is true from 'arm' until the slave has acknowledged the arm
%   (or the attempt was given up), which is when the run applies the
%   command the capture is meant to see. p.nPending counts the captures
%   not yet finished, p.failures the failed link transactions and given-up
%   arms and p.lost the captures lost to overrun.
%
%   A capture struct has step, tag, tArm (when its arm was first written),
%   tRead, mark (record index at the arm), first (index of its first
%   record), lost, pos0 (position at the first record, mm) and data =
%   [t vCmd velocity torque position pressure] with t = (0:N-1)' * scan.
%   A capture is lost when the ring overwrote records before they were
%   read; its data hold the records read before that.
%
%   Window protocol (see PRG_StepProbe.st): the master writes the index of
%   the first record it wants (HR112-113); the slave fills the window from
%   it; a read is accepted when Start equals StartTail (not torn) and
%   Start equals the index requested, otherwise the request is repeated.
%   A partial window (fewer than 18 records, the capture still being
%   recorded) is accepted and the next request advances by its count.
%   The first arm value is the slave's ArmEcho + 1 (wrapping, never 0), so
%   a fresh script cannot collide with the value the slave has latched.

switch cmd
    case 'new'
        p = struct('cfg', p, 'scan', varargin{1}, 'q', {{}}, ...
            'armValue', NaN, 'progress', 0, 'failures', 0, 'lost', 0, ...
            'armBusy', false, 'nPending', 0);
        caps = {};
    case 'arm'
        info = varargin{1};
        it = struct('step', field(info, 'step', 0), 'tag', field(info, 'tag', ''), ...
            'nRecords', field(info, 'nRecords', p.cfg.nScan), ...
            'nPre', field(info, 'nPre', p.cfg.nPre), ...
            'state', 'queued', 'armValue', NaN, 'tSeen', NaN, 'tArm', NaN, ...
            'tWrite', NaN, 'mark', NaN, 'first', NaN, 'next', NaN, ...
            'last', Inf, 'closed', false, 'chunks', {{}});
        p.q{end + 1} = it;
        p = refresh(p);
        caps = {};
    case 'close'
        for k = 1:numel(p.q)
            if isinf(p.q{k}.nRecords)
                p.q{k}.closed = true;
            end
        end
        caps = {};
    case 'slot'
        [p, caps] = slot(p, varargin{1}, varargin{2});
    case 'drain'
        [p, caps] = drain(p, varargin{1}, varargin{2});
    otherwise
        error('benchProbe: unknown command "%s"', cmd);
end
end


% ===========================================================================
function [p, caps] = slot(p, link, tNow)
caps = {};
if isempty(p.q)
    return;
end
pr = p.cfg;

% Arms the slave has not acknowledged in time are given up.
keep = true(1, numel(p.q));
for k = 1:numel(p.q)
    it = p.q{k};
    if isnan(it.tSeen)
        p.q{k}.tSeen = tNow;
    elseif any(strcmp(it.state, {'queued', 'sent'})) && tNow - it.tSeen > pr.timeout
        fprintf('  probe arm for step %d not acknowledged in %g s: given up\n', it.step, pr.timeout);
        p.failures = p.failures + 1;
        keep(k) = false;
    end
end
p.q = p.q(keep);
p = refresh(p);
if isempty(p.q)
    return;
end

% The one read.
try
    regs = link.read(pr.irFirst, pr.nRegs);
    [p, caps, whole] = ingest(p, double(regs(:)'), tNow);
catch err
    note(p, tNow, err);
    p.failures = p.failures + 1;
    return;
end
p = refresh(p);

% The one write: an arm first, else the request for the next records.
try
    a = find(cellfun(@(it) any(strcmp(it.state, {'queued', 'sent'})), p.q), 1);
    if ~isempty(a) && ~isnan(p.armValue)
        it = p.q{a};
        if strcmp(it.state, 'queued')
            link.writeOne(pr.hrArm, p.armValue);
            it.armValue = p.armValue;
            p.armValue = mod(p.armValue, 65535) + 1;
            it.state = 'sent';
            it.tArm = tNow;
            it.tWrite = tNow;
            p.q{a} = it;
            return;
        elseif tNow - it.tWrite >= pr.armRetry
            link.writeOne(pr.hrArm, it.armValue);
            p.q{a}.tWrite = tNow;
            return;
        end
    end
    h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
    if ~isempty(h) && ~whole
        req = p.q{h}.next;
        link.write(pr.hrReq, [mod(req, 65536) floor(req / 65536)]);
    end
catch err
    note(p, tNow, err);
    p.failures = p.failures + 1;
end
end


function [p, caps, whole] = ingest(p, regs, tNow)
% Take one read: the header, the arm acknowledgement, the window. whole
% is false when the read was torn or did not show the window asked for.
caps = {};
whole = false;
pr = p.cfg;
u32 = @(i) regs(i) + 65536 * regs(i + 1);
written = u32(1);
mark = u32(3);
start = u32(5);
tail = u32(pr.nRegs - 1);
if start ~= tail
    return;
end
echo = regs(7);
count = regs(8);
depth = regs(9);
if isnan(p.armValue)
    p.armValue = mod(echo, 65535) + 1;
end

% Acknowledgement of the arm in flight.
a = find(cellfun(@(it) strcmp(it.state, 'sent'), p.q), 1);
if ~isempty(a) && echo == p.q{a}.armValue
    it = p.q{a};
    it.mark = mark;
    it.first = max(mark - it.nPre, 0);
    it.next = it.first;
    it.state = 'read';
    p.q{a} = it;
    p.progress = p.progress + 1;
end
% A closed stream ends at the newest record seen.
for k = 1:numel(p.q)
    it = p.q{k};
    if it.closed && strcmp(it.state, 'read') && isinf(it.last)
        p.q{k}.last = max(written, it.next);
    end
end

% Records, oldest capture first.
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
if isempty(h)
    whole = true;
    return;
end
[p, caps] = consume(p, regs, start, count, tNow);
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
while ~isempty(h) && p.q{h}.next < written - depth
    [p, c] = finish(p, h, tNow, true);          % overwritten before it was read
    caps{end + 1} = c;                          %#ok<AGROW>
    h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
end
whole = isempty(h) || start == p.q{h}.next;
end


function [p, caps] = consume(p, regs, start, count, tNow)
% Hand the window's records to the captures that want them, oldest first.
% The window counts only if it starts where the oldest capture is.
pr = p.cfg;
caps = {};
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
if start ~= p.q{h}.next || count == 0
    return;
end
n = min(count, pr.nWindow);
rec = decodeRecords(reshape(regs(11:10 + pr.nWords * n), pr.nWords, n)', pr);
while ~isempty(h)
    it = p.q{h};
    j = it.next - start;                        % offset of the wanted record in the window
    if j < 0 || j >= n
        break;
    end
    take = min(n - j, endOf(it) - it.next);
    if take > 0
        it.chunks{end + 1} = rec(j + 1:j + take, :);
        it.next = it.next + take;
        p.progress = p.progress + take;
    end
    p.q{h} = it;
    if it.next < endOf(it)
        break;
    end
    [p, c] = finish(p, h, tNow, false);
    caps{end + 1} = c;                          %#ok<AGROW>
    h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
end
end


function e = endOf(it)
% Index after the capture's last record.
e = min(it.first + it.nRecords, it.last);
end


function [p, cap] = finish(p, h, tNow, lost)
it = p.q{h};
d = vertcat(it.chunks{:});
if isempty(d)
    d = zeros(0, 5);
end
N = size(d, 1);
cap = struct('step', it.step, 'tag', it.tag, 'tArm', it.tArm, 'tRead', tNow, ...
    'mark', it.mark, 'first', it.first, 'lost', lost, ...
    'pos0', NaN, 'data', [(0:N - 1)' * p.scan d]);
if N > 0
    cap.pos0 = d(1, 4);
end
if lost
    p.lost = p.lost + 1;
end
p.q(h) = [];
p = refresh(p);
end


function rec = decodeRecords(w, pr)
% Window rows [vCmd vel torque posLo posHi pressure] as
% [vCmd velocity torque position pressure] in engineering units.
pos = w(:, 4) + 65536 * w(:, 5);
pos(pos >= 2^31) = pos(pos >= 2^31) - 2^32;                 % two's complement
rec = [int16Of(w(:, 1)) / pr.velScale, int16Of(w(:, 2)) / pr.velScale, ...
       int16Of(w(:, 3)) / pr.trqScale, pos / pr.posScale, ...   % [counts -> mm]
       int16Of(w(:, 6)) / pr.presScale];
end


function x = int16Of(w)
% Signed value of 16-bit registers.
x = w;
x(x >= 32768) = x(x >= 32768) - 65536;
end


function [p, caps] = drain(p, link, env)
pr = p.cfg;
caps = {};
p = benchProbe('close', p);
tProg = env.now();
progress = p.progress;
while ~isempty(p.q)
    t = env.now();
    [p, c] = slot(p, link, t);
    caps = [caps c];                            %#ok<AGROW>
    if p.progress ~= progress
        progress = p.progress;
        tProg = t;
    end
    if ~isempty(p.q) && t - tProg > pr.timeout
        fprintf('  probe drain: %d captures not read in %g s: given up\n', numel(p.q), pr.timeout);
        p.failures = p.failures + numel(p.q);
        p.q = {};
        break;
    end
    env.wait(env.now() + pr.drainGap);
end
p = refresh(p);
end


function p = refresh(p)
p.nPending = numel(p.q);
p.armBusy = any(cellfun(@(it) any(strcmp(it.state, {'queued', 'sent'})), p.q));
end


function v = field(s, name, default)
if isfield(s, name)
    v = s.(name);
else
    v = default;
end
end


function note(p, tNow, err)
if p.failures < 3
    fprintf('  probe transaction failed at %.2f s: %s\n', tNow, err.message);
end
end
