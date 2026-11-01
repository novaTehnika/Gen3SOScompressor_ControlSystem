function [p, caps] = benchProbe(cmd, p, varargin)
%BENCHPROBE Master side of the slave's step probe (slave-src/FB/FB_Probe.st):
%   arm captures, read them out of the slave's ring buffer through its
%   13-record window, fetch the history before a fault, and decode the
%   records. Shared by frictionRun and charRun.
%
%   p = benchProbe('new', cfgProbe, scan)
%       cfgProbe is benchConfig().probe, scan the slave's scan period [s],
%       which sets the t column of a capture.
%   p = benchProbe('arm', p, info)
%       Queue a capture. info.step and info.tag label it, info.nRecords is
%       its length in records (default cfgProbe.nScan; Inf for a stream
%       that runs until 'close' or 'drain'), info.nPre the records taken
%       from before the arm (default cfgProbe.nPre): the capture is the
%       records from mark - nPre on, nRecords of them in all. The slave is
%       told the length (nRecords - nPre; 0, open ended, for a stream or a
%       length above 65535) so its probe state follows the capture.
%   p = benchProbe('fault', p, info)
%       Queue the history around the slave's latest fault: the records
%       from FaultMark - info.nPre (default 500) on, nPre + info.nPost
%       (default 500) of them, FaultMark being the record written in the
%       scan PRG_Main entered ST_FAULT. The slave is not armed: the ring
%       already holds it, for as long as the ring does. With info.wait
%       true the capture waits for a fault after the first read instead
%       of taking one already latched. A script that wants the history of
%       a fault it provokes queues this before the fault, with wait true.
%   p = benchProbe('close', p)
%       End every open stream at the newest record seen.
%   [p, caps] = benchProbe('slot', p, link, tNow)
%       The work of one gap between protocol exchanges: at most one FC04
%       read of IR401-524 and at most one write (the next arm, a repeat of
%       an unacknowledged one, or the next window request), the read first
%       so the slave has the whole next cycle to act on the write. caps is
%       a cell array of the captures finished by this read.
%   p = benchProbe('poll', p, link, tNow)
%       One read that only updates the status fields below.
%   [p, caps] = benchProbe('drain', p, link, env)
%       Blocking: closes the streams, then slots every cfgProbe.drainGap
%       (env.now and env.wait(tTarget) are the run's clock functions)
%       until nothing is pending or cfgProbe.timeout passes without
%       progress.
%   st = benchProbe('status', [], word)
%       Fields inMotion, performance, fault, homing (logical) and mode of
%       a status column of a capture (data(:, 8)).
%
%   p.armBusy is true from 'arm' until the slave has acknowledged the arm
%   (or the attempt was given up), which is when the run applies the
%   command the capture is meant to see. p.nPending counts the captures
%   not yet finished, p.failures the failed link transactions and given-up
%   arms and p.lost the captures lost to overrun. After a read p.written
%   (records so far), p.depth (the ring), p.probeState (0 off, 1 recording,
%   2 capturing, 3 held), p.xferState (0 idle, 1 filling, 2 pending, 3
%   overrun) and p.faultMark (0 until a fault) hold the slave's registers.
%
%   A capture struct has step, tag, tArm (when its arm was first written),
%   tRead, mark (record index at the arm, or at the fault), first (index of
%   its first record), lost, pos0 (position at the first record, mm) and
%   data = [t vCmd velocity torque position pressure volts status] with
%   t = (0:N-1)' * scan, the velocities in mm/s, torque in % rated,
%   position in mm, pressure in MPa gauge, volts the raw pressure input (V)
%   and status the record's status word (bit 0 in motion, 1 performance
%   status, 2 fault active, 3 homing complete, bits 8-15 the confirmed mode
%   code; see 'status'). A capture is lost when the ring overwrote records
%   before they were read; its data hold the records read before that.
%
%   Window protocol (see FB_Probe.st): the master writes the index of the
%   first record it wants (HR112-113); the slave fills the window from it,
%   a few records per scan, and publishes it when it is complete; a read is
%   accepted when Start equals StartTail (not torn) and Start equals the
%   index requested, otherwise the request is repeated unless the slave
%   reports that it is still filling or waiting for the records (Xfer).
%   A partial window (fewer records than it holds, the capture still being
%   recorded) is accepted and the next request advances by its count. The
%   record's word count is read from the slave (IR410), so a longer record
%   with the same first eight words still decodes.
%   The first arm value is the slave's ArmEcho + 1 (wrapping, never 0), so
%   a fresh script cannot collide with the value the slave has latched.

switch cmd
    case 'new'
        p = struct('cfg', p, 'scan', varargin{1}, 'q', {{}}, ...
            'armValue', NaN, 'progress', 0, 'failures', 0, 'lost', 0, ...
            'armBusy', false, 'nPending', 0, 'req', NaN, 'written', NaN, ...
            'depth', NaN, 'probeState', NaN, 'xferState', NaN, 'faultMark', NaN);
        caps = {};
    case 'arm'
        info = varargin{1};
        it = newItem('arm', info, p.cfg.nScan, p.cfg.nPre);
        p.q{end + 1} = it;
        p = refresh(p);
        caps = {};
    case 'fault'
        info = varargin{1};
        nPre = field(info, 'nPre', 500);
        it = newItem('fault', info, nPre + field(info, 'nPost', 500), nPre);
        it.state = 'faultwait';
        it.wait = logical(field(info, 'wait', false));
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
    case 'poll'
        try
            [p, ~, ~] = ingest(p, double(varargin{1}.read(p.cfg.irFirst, p.cfg.nRegs)), varargin{2});
        catch err
            note(p, varargin{2}, err);
            p.failures = p.failures + 1;
        end
        caps = {};
    case 'drain'
        [p, caps] = drain(p, varargin{1}, varargin{2});
    case 'status'
        w = varargin{1};
        p = struct('inMotion', bitget(w, 1) == 1, 'performance', bitget(w, 2) == 1, ...
            'fault', bitget(w, 3) == 1, 'homing', bitget(w, 4) == 1, ...
            'mode', floor(w / 256));
        caps = {};
    otherwise
        error('benchProbe: unknown command "%s"', cmd);
end
end


% ===========================================================================
function it = newItem(kind, info, nDefault, nPreDefault)
it = struct('kind', kind, 'step', field(info, 'step', 0), 'tag', field(info, 'tag', ''), ...
    'nRecords', field(info, 'nRecords', nDefault), ...
    'nPre', field(info, 'nPre', nPreDefault), 'wait', false, 'fmBase', NaN, ...
    'state', 'queued', 'armValue', NaN, 'tSeen', NaN, 'tArm', NaN, ...
    'tWrite', NaN, 'mark', NaN, 'first', NaN, 'next', NaN, ...
    'last', Inf, 'closed', false, 'chunks', {{}});
end


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
        hint = '';
        if p.probeState == 0
            hint = ' (the probe is disabled: G_cfgProbeEnable)';
        end
        fprintf('  probe arm for step %d not acknowledged in %g s: given up%s\n', it.step, pr.timeout, hint);
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
        if strcmp(it.state, 'queued') || tNow - it.tWrite >= pr.armRetry
            if strcmp(it.state, 'queued')
                it.armValue = p.armValue;
                p.armValue = mod(p.armValue, 65535) + 1;
                it.state = 'sent';
                it.tArm = tNow;
            end
            % The arm, the request in force (so that it does not restart a fill)
            % and the length, in one write.
            req = currentRequest(p);
            link.write(pr.hrArm, [it.armValue, mod(req, 65536), floor(req / 65536), armLength(it)]);
            p.req = req;
            it.tWrite = tNow;
            p.q{a} = it;
            return;
        end
    end
    h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
    if ~isempty(h) && ~whole
        req = p.q{h}.next;
        busy = any(p.xferState == [1 2]);       % filling, or waiting for records
        if ~(req == p.req && busy)
            link.write(pr.hrReq, [mod(req, 65536) floor(req / 65536)]);
            p.req = req;
        end
    end
catch err
    note(p, tNow, err);
    p.failures = p.failures + 1;
end
end


function req = currentRequest(p)
% The window request to leave in force when an arm is written: that of the
% capture being read, else the last one written, else the start.
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
if ~isempty(h)
    req = p.q{h}.next;
elseif ~isnan(p.req)
    req = p.req;
else
    req = 0;
end
end


function n = armLength(it)
% Records the slave is told to expect from the mark on; 0 is open ended.
n = it.nRecords - it.nPre;
if isinf(n) || n > 65535
    n = 0;
else
    n = max(n, 1);
end
end


function [p, caps, whole] = ingest(p, regs, tNow)
% Take one read: the header, the arm acknowledgement, the window. whole
% is false when the read was torn or did not show the window asked for.
caps = {};
whole = false;
pr = p.cfg;
o = pr.off;
u32 = @(i) regs(i) + 65536 * regs(i + 1);
written = u32(o.written);
mark = u32(o.mark);
start = u32(o.start);
tail = u32(o.tail);
if start ~= tail
    return;
end
echo = regs(o.echo);
count = regs(o.count);
depth = regs(o.depth);
nWords = regs(o.words);
if nWords < pr.nWords
    error('benchProbe: the slave''s records have %d words, the decoder needs %d', nWords, pr.nWords);
end
nWin = floor(pr.nWindowWords / nWords);
p.written = written;
p.depth = depth;
p.probeState = regs(o.state);
p.xferState = regs(o.xfer);
p.faultMark = u32(o.faultMark);
if isnan(p.armValue)
    p.armValue = mod(echo, 65535) + 1;
end

% A queued fault capture takes the latched fault mark (a newer one if it
% waits for one).
for k = 1:numel(p.q)
    it = p.q{k};
    if ~strcmp(it.state, 'faultwait')
        continue;
    end
    if it.wait && isnan(it.fmBase)
        p.q{k}.fmBase = p.faultMark;
    elseif p.faultMark ~= 0 && (~it.wait || p.faultMark ~= it.fmBase)
        it.mark = p.faultMark;
        it.first = max(it.mark - it.nPre, 0);
        it.next = it.first;
        it.tArm = tNow;
        it.state = 'read';
        p.q{k} = it;
        p.progress = p.progress + 1;
    end
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
[p, caps] = consume(p, regs, start, count, nWords, nWin, tNow);
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
while ~isempty(h) && (p.q{h}.next < written - depth || ...
        (p.xferState == 3 && p.req == p.q{h}.next && start ~= p.q{h}.next))
    [p, c] = finish(p, h, tNow, true);          % overwritten before it was read
    caps{end + 1} = c;                          %#ok<AGROW>
    h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
end
whole = isempty(h) || start == p.q{h}.next;
end


function [p, caps] = consume(p, regs, start, count, nWords, nWin, tNow)
% Hand the window's records to the captures that want them, oldest first.
% The window counts only if it starts where the oldest capture is.
pr = p.cfg;
caps = {};
h = find(cellfun(@(it) strcmp(it.state, 'read'), p.q), 1);
if start ~= p.q{h}.next || count == 0
    return;
end
n = min(count, nWin);
w0 = pr.off.window;
rec = decodeRecords(reshape(regs(w0:w0 - 1 + nWords * n), nWords, n)', pr);
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
    d = zeros(0, 7);
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
% Window rows [vCmd vel torque posLo posHi pressure raw status ...] as
% [vCmd velocity torque position pressure volts status] in engineering
% units; words past the eighth are not decoded.
pos = w(:, 4) + 65536 * w(:, 5);
pos(pos >= 2^31) = pos(pos >= 2^31) - 2^32;                 % two's complement
rec = [int16Of(w(:, 1)) / pr.velScale, int16Of(w(:, 2)) / pr.velScale, ...
       int16Of(w(:, 3)) / pr.trqScale, pos / pr.posScale, ...   % [counts -> mm]
       int16Of(w(:, 6)) / pr.presScale, ...
       w(:, 7) / pr.voltScale, ...                              % [counts -> V]
       w(:, 8)];
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
