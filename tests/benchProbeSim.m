function benchProbeSim()
%BENCHPROBESIM Test bench/common/benchProbe against the slave's own probe
%   (slave-src/FB/FB_Probe.st, translated into the simulated slave and read
%   through the registers by tests/fakeProbeLink.m). The reader side: an
%   arm whose write is lost is written again and acknowledged; a torn read
%   is discarded and read again; an arm made while an earlier capture is
%   still being read is accepted and both captures come back whole; a
%   stream longer than the ring is read continuously by a reader fast
%   enough and reported lost to a reader that is too slow; drain reads
%   everything pending and gives up on an arm the slave never sees. The
%   record contents, including the raw volts and status columns, are
%   checked against a log of what the probe was given (torque carries white
%   noise, so every record differs).
%   The slave side, read through the registers scan by scan: the recording
%   state machine (an arm with a length goes CAPTURING, HELD, RECORDING at
%   the scans the rules give; an open-ended arm stays CAPTURING; a capture
%   longer than the ring skips HELD), the transfer state machine (a fill
%   spread over several scans never publishes a half window and is not
%   restarted by the same request written again; a request ahead of the
%   records is PENDING and then served; one behind the ring is OVERRUN,
%   with the real ring depth and a start index above 65535), and the fault
%   mark: the history around an injected drive fault read back with
%   benchProbe('fault'), the fault bit of the status word set from the
%   marked record on.
%   Depends on: master-src/logic slave-src slave-sim tools models bench/common tests/fakeProbeLink.m
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath master-src/logic slave-sim slave-sim/generated tests

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'common'));
bcfg = benchConfig();
pr = bcfg.probe;
scan = bcfg.slaveScan;

% --- A lost arm is written again and acknowledged ----------------------------------
[link, env] = freshLink(8192);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 3, 'tag', 'a', 'nRecords', 100));
assert(p.armBusy && p.nPending == 1);
fakeProbeLink('dropArm', 2);
[p, caps, t] = runSlots(p, link, env, 0.05, 3, @(p, caps) p.nPending == 0);
S = fakeProbeLink('state');
assert(S.sp.dropArm == 0, 'the dropped writes were not both made');
assert(numel(caps) == 1 && ~caps{1}.lost && size(caps{1}.data, 1) == 100, 'capture not returned');
assert(p.failures == 0, 'a retried arm counted as a failure');
assert(caps{1}.tRead - caps{1}.tArm > 2 * pr.armRetry, 'arm acknowledged without the retries (%.2f s)', t);
checkAgainstRing(caps{1}, S, scan);
assert(caps{1}.step == 3 && strcmp(caps{1}.tag, 'a'));
fprintf('ok  two lost arm writes retried every %.1f s, capture complete at %.2f s\n', pr.armRetry, t);

% The next arm takes the value after the echo, not the latched one.
p = benchProbe('arm', p, struct('step', 4, 'nRecords', 40));
[p, caps] = runSlots(p, link, env, 0.05, 2, @(p, caps) p.nPending == 0);
assert(numel(caps) == 1 && size(caps{1}.data, 1) == 40 && p.failures == 0);
S = fakeProbeLink('state');
checkAgainstRing(caps{1}, S, scan);
assert(S.sp.echo == S.sp.arm && S.sp.arm == 2, 'arm values %d, %d', S.sp.arm, S.sp.echo);

% --- A torn read is discarded and read again ---------------------------------------
[link, env] = freshLink(8192);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 1, 'nRecords', 300));
[p, caps] = runSlots(p, link, env, 0.05, 3, @(p, caps) ~p.armBusy);
assert(isempty(caps) && p.nPending == 1);
fakeProbeLink('tearRead', 3);
[p, caps] = runSlots(p, link, env, 0.05, 6, @(p, caps) p.nPending == 0);
S = fakeProbeLink('state');
assert(S.sp.tear == 0, 'torn reads were not made');
assert(numel(caps) == 1 && size(caps{1}.data, 1) == 300 && p.failures == 0);
checkAgainstRing(caps{1}, S, scan);
fprintf('ok  three torn reads discarded, capture of 300 records intact\n');

% --- An arm made while an earlier capture is being read ------------------------------
[link, env] = freshLink(8192);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 1, 'tag', 'first', 'nRecords', 500));
[p, caps] = runSlots(p, link, env, 0.05, 3, @(p, caps) ~p.armBusy);
[p, caps] = runSlots(p, link, env, 0.05, 1, @(p, caps) false, 4);
assert(isempty(caps) && p.nPending == 1, 'first capture already complete');
p = benchProbe('arm', p, struct('step', 2, 'tag', 'second', 'nRecords', 200, 'nPre', 10));
[p, caps] = runSlots(p, link, env, 0.05, 6, @(p, caps) p.nPending == 0);
S = fakeProbeLink('state');
assert(numel(caps) == 2 && strcmp(caps{1}.tag, 'first') && strcmp(caps{2}.tag, 'second'), ...
    'captures not returned oldest first');
assert(size(caps{1}.data, 1) == 500 && size(caps{2}.data, 1) == 200 && p.failures == 0);
assert(caps{2}.mark > caps{1}.mark && caps{2}.mark - caps{2}.first == 10, 'marks %d, %d, first %d', ...
    caps{1}.mark, caps{2}.mark, caps{2}.first);
assert(caps{2}.mark < caps{1}.mark + 500, 'the second arm did not come during the first capture');
checkAgainstRing(caps{1}, S, scan);
checkAgainstRing(caps{2}, S, scan);
fprintf('ok  arm during a read accepted: marks %d and %d, both captures correct\n', ...
    caps{1}.mark, caps{2}.mark);

% --- A stream longer than the ring --------------------------------------------------
depth = 512;
[link, env] = freshLink(depth);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 1, 'tag', 'stream', 'nRecords', Inf));
[p, caps, t] = runSlots(p, link, env, 0.01, 3000 * scan, @(p, caps) false);
assert(isempty(caps) && p.nPending == 1 && p.lost == 0, 'a fast reader lost the stream');
[p, caps] = benchProbe('drain', p, link, env);
S = fakeProbeLink('state');
assert(numel(caps) == 1 && ~caps{1}.lost && p.nPending == 0 && p.failures == 0);
N = size(caps{1}.data, 1);
assert(N > 4 * depth && caps{1}.first + N <= S.sp.written, 'stream of %d records', N);
assert(caps{1}.first + N >= S.sp.written - 40, 'stream ends %d records before the newest', ...
    S.sp.written - caps{1}.first - N);
checkAgainstRing(struct('first', caps{1}.first + N - 400, 'data', [(0:399)' * scan caps{1}.data(end - 399:end, 2:8)], 'tag', 'tail'), S, scan);
d = diff(caps{1}.data(:, 1));
assert(all(abs(d - scan) < 1e-9), 'stream time column not continuous');
fprintf('ok  stream of %d records through a ring of %d, read continuously every 10 ms\n', N, depth);

[link, env] = freshLink(depth);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 1, 'tag', 'stream', 'nRecords', Inf));
[p, caps, t] = runSlots(p, link, env, 25 * scan, 12, @(p, caps) ~isempty(caps));
assert(numel(caps) == 1 && caps{1}.lost && p.lost == 1, 'slow reader not reported lost');
N = size(caps{1}.data, 1);
assert(N > 0 && N < 3 * depth, 'lost stream holds %d records', N);
assert(t < 12, 'overrun found only at %.1f s', t);
fprintf('ok  stream lost to a %.0f ms reader after %.1f s (%d records kept)\n', 25 * scan * 1000, t, N);

% --- Drain reads everything pending ---------------------------------------------------
[link, env] = freshLink(8192);
p = benchProbe('new', pr, scan);
n = [300 500 100];
for k = 1:3
    p = benchProbe('arm', p, struct('step', k, 'nRecords', n(k)));
end
assert(p.nPending == 3);
[p, caps] = benchProbe('drain', p, link, env);
S = fakeProbeLink('state');
assert(numel(caps) == 3 && p.nPending == 0 && ~p.armBusy && p.failures == 0, '%d captures from drain', numel(caps));
assert(all(cellfun(@(c) c.step, caps) == 1:3) && all(cellfun(@(c) size(c.data, 1), caps) == n));
for k = 1:3
    checkAgainstRing(caps{k}, S, scan);
end
assert(caps{2}.mark > caps{1}.mark && caps{3}.mark > caps{2}.mark);
fprintf('ok  drain read three queued captures (marks %d, %d, %d)\n', ...
    caps{1}.mark, caps{2}.mark, caps{3}.mark);

% An arm the slave never sees is given up after the timeout and counted.
[link, env] = freshLink(8192);
pr2 = pr;
pr2.timeout = 1;
p = benchProbe('new', pr2, scan);
fakeProbeLink('dropArm', 1000);
p = benchProbe('arm', p, struct('step', 1, 'nRecords', 100));
[p, caps] = benchProbe('drain', p, link, env);
assert(isempty(caps) && p.nPending == 0 && ~p.armBusy && p.failures == 1, 'given-up arm: %d failures', p.failures);
fprintf('ok  arm never acknowledged given up and counted\n');

% --- Recording state: CAPTURING, HELD, RECORDING ------------------------------------
freshLink(64);
step(50);
r = probeRegs();
assert(r.state == 1 && r.depth == 64 && r.words == 8 && r.xfer == 0, 'power-up: state %d, depth %d, words %d', ...
    r.state, r.depth, r.words);
[seq, at, r] = watchStates([1 0 0 20], 200);
assert(isequal(seq, [2 3 1]) && isequal(at, [1 20 65]), 'length 20: states %s at %s', mat2str(seq), mat2str(at));
assert(r.echo == 1, 'arm not echoed');
[seq, at] = watchStates([2 0 0 1], 200);
assert(isequal(seq, [3 1]) && isequal(at, [1 65]), 'length 1: states %s at %s', mat2str(seq), mat2str(at));
[seq, at] = watchStates([3 0 0 100], 200);
assert(isequal(seq, [2 1]) && isequal(at, [1 100]), 'length 100 in a ring of 64: states %s at %s', mat2str(seq), mat2str(at));
[seq, ~] = watchStates([4 0 0 0], 200);
assert(isequal(seq, 2), 'an open-ended arm left CAPTURING: %s', mat2str(seq));
[seq, at] = watchStates([5 0 0 10], 100);
assert(isequal(seq, [2 3 1]) && isequal(at, [1 10 65]), 'the next arm after an open-ended one: %s at %s', mat2str(seq), mat2str(at));
% An arm during a held capture starts a new one.
step(20);
[seq, ~] = watchStates([6 0 0 30], 10);
assert(isequal(seq, 2), 'arm during a held capture: %s', mat2str(seq));
% Disabled: nothing is recorded and an arm made meanwhile is not acted on later.
fakeProbeLink('slave', @(sl) setCfg(sl, 'G_cfgProbeEnable', false));
step(3);
r = probeRegs();
w = r.written;
fakeProbeLink('write', 111, [9 0 0 5]);
step(5);
r = probeRegs();
assert(r.state == 0 && r.xfer == 0 && r.written == w, 'disabled probe: state %d, %d records written', r.state, r.written - w);
fakeProbeLink('slave', @(sl) setCfg(sl, 'G_cfgProbeEnable', true));
step(3);
r = probeRegs();
assert(r.state == 1 && r.echo ~= 9 && r.written == w + 3, 'enabled again: state %d, echo %d', r.state, r.echo);
fprintf('ok  recording states: CAPTURING, HELD at the last record, RECORDING once the ring overwrites the mark (65 scans on a 64 ring); length 1, longer than the ring (no HELD), open ended; off\n');

% --- Transfer: a fill spread over several scans --------------------------------------
freshLink(8192);
step(250);
r0 = probeRegs();
assert(r0.depth == 8192 && r0.xfer == 0 && r0.count == 13 && r0.start == 0, 'start: depth %d, count %d', r0.depth, r0.count);
[seen, wins] = watchWindow([120 0], 5, false);
assert(all(seen(1:2, 1) == 1) && all(all(seen(1:2, 2:4) == repmat([0 0 13], 2, 1))), 'half window published: %s', mat2str(seen));
assert(isequal(wins{1}, r0.window) && isequal(wins{2}, r0.window), 'the window changed before the fill completed');
assert(isequal(seen(3, :), [0 120 120 13]) && isequal(seen(5, :), [0 120 120 13]), 'fill not complete at scan 3: %s', mat2str(seen));
S = fakeProbeLink('state');
assert(isequal(wins{3}, expectedWindow(S.ring, 120, 13)), 'window records differ from the log');
% The same request written every scan does not restart the fill.
[seen, ~] = watchWindow([140 0], 4, true);
assert(isequal(seen(:, 1)', [1 1 0 0]) && isequal(seen(3, :), [0 140 140 13]) && seen(1, 2) == 120, 'rewritten request: %s', mat2str(seen));
% One record per scan: thirteen scans, the old window all the while.
fakeProbeLink('slave', @(sl) setCfg(sl, 'G_cfgProbeFillPerScan', 1));
[seen, wins] = watchWindow([160 0], 15, false);
assert(all(seen(1:12, 1) == 1) && all(seen(1:12, 2) == 140) && all(seen(1:12, 3) == 140) && all(seen(13:15, 1) == 0) ...
    && all(seen(13:15, 2) == 160) && all(seen(13:15, 3) == 160), 'one record per scan: %s', mat2str(seen(:, 1:3)));
S = fakeProbeLink('state');
assert(isequal(wins{12}, expectedWindow(S.ring, 140, 13)) && isequal(wins{13}, expectedWindow(S.ring, 160, 13)));
fakeProbeLink('slave', @(sl) setCfg(sl, 'G_cfgProbeFillPerScan', 6));
fprintf('ok  fill of 13 records: 3 scans at 6 per scan, 13 at 1 per scan; Start, Count and tail change only when it completes, the window not before\n');

% --- Transfer: PENDING, then a window that grows with the records ----------------------
r = probeRegs();
prev = r.start;
req = r.written + 40;
fakeProbeLink('write', 112, [mod(req, 65536) floor(req / 65536)]);
step(1);
r = probeRegs();
assert(r.xfer == 2 && r.start == prev, 'request ahead of the records: xfer %d', r.xfer);
counts = [];
for k = 1:80
    step(1);
    r = probeRegs();
    if r.xfer == 2
        assert(r.start == prev && r.tail == prev, 'published while pending');
    else
        assert(r.xfer == 0 && r.start == req && r.tail == req, 'xfer %d, start %d', r.xfer, r.start);
        counts(end + 1) = r.count;                  %#ok<AGROW>
    end
end
assert(isequal(counts(1:13), 1:13) && all(counts(14:end) == 13), 'window growth: %s', mat2str(counts));
S = fakeProbeLink('state');
assert(isequal(r.window, expectedWindow(S.ring, req, 13)), 'grown window differs from the log');
fprintf('ok  request %d records ahead is PENDING, then served from the first record written; the window grows 1..13 records\n', 40);

% --- Transfer: OVERRUN with the real ring and indices above 65535 ------------------------
fakeProbeLink('slave', @(sl) setWritten(sl, 100000));
step(3);
r = probeRegs();
assert(r.written == 100003 && r.depth == 8192);
prev = r.start;
gone = r.written - 8192;                            % the scan that serves it writes one more record
fakeProbeLink('write', 112, [mod(gone, 65536) floor(gone / 65536)]);
step(1);
r = probeRegs();
assert(gone > 65535 && r.xfer == 3 && r.start == prev && r.tail == prev, 'start overwritten: xfer %d', r.xfer);
late = r.written - 8192;
fakeProbeLink('write', 112, [mod(late, 65536) floor(late / 65536)]);
step(1);
r = probeRegs();
assert(r.xfer == 3, 'a start one record past the ring''s oldest was served');
ok = r.written + 1 - 8192;
fakeProbeLink('write', 112, [mod(ok, 65536) floor(ok / 65536)]);
step(3);
r = probeRegs();
assert(r.xfer == 0 && r.start == ok && r.count == 13, 'oldest record in the ring not served: xfer %d', r.xfer);
req = r.written + 10;
fakeProbeLink('write', 112, [mod(req, 65536) floor(req / 65536)]);
step(30);
r = probeRegs();
S = fakeProbeLink('state');
assert(r.xfer == 0 && r.start == req && r.count == 13 && isequal(r.window, expectedWindow(S.ring, req, 13)), 'start above 65535');
% A fill whose rest is overwritten while it runs.
fakeProbeLink('slave', @(sl) setCfg(sl, 'G_cfgProbeFillPerScan', 1));
prev = r.start;
req = r.written - 100;
fakeProbeLink('write', 112, [mod(req, 65536) floor(req / 65536)]);
step(2);
r = probeRegs();
assert(r.xfer == 1 && r.start == prev);
fakeProbeLink('slave', @(sl) setWritten(sl, sl.s.fbProbe.Written + 20000));
step(1);
r = probeRegs();
assert(r.xfer == 3 && r.start == prev && r.tail == prev, 'overwritten during the fill: xfer %d', r.xfer);
step(5);
r = probeRegs();
assert(r.xfer == 3 && r.start == prev, 'overrun not held');
fprintf('ok  start behind the ring is OVERRUN (exact boundary, index %d above 65535, ring 8192), also when overwritten mid-fill; nothing published\n', gone);

% --- Fault mark, the history around a fault, the new columns --------------------------------
[link, env] = freshLink(8192);
p = benchProbe('new', pr, scan);
p = benchProbe('poll', p, link, env.now());
assert(p.faultMark == 0 && p.probeState == 1 && p.depth == 8192 && p.xferState == 0, 'status fields after a poll');
p = benchProbe('arm', p, struct('step', 1, 'tag', 'run', 'nRecords', Inf));
p = benchProbe('fault', p, struct('step', 2, 'tag', 'fault', 'nPre', 100, 'nPost', 60, 'wait', true));
t0 = env.now();
fakeProbeLink('faultAt', t0 + 3.6);
hb = 0;
enable = 0;
caps = {};
while env.now() < t0 + 4.4
    env.wait(env.now() + 0.02);
    t = env.now();
    hb = mod(hb + 1, 65536);
    conf = fakeProbeLink('read', 3, 1);
    if conf == 1
        enable = 1;
    end
    fakeProbeLink('write', 1, [hb 1 enable 0 0 0 0 0]);    % HR1-8, position speed 0
    [p, c] = benchProbe('slot', p, link, t);
    caps = [caps c];                                %#ok<AGROW>
end
assert(isempty(caps) && p.nPending == 2, 'the stream and the fault capture should both be pending');
[p, c] = benchProbe('drain', p, link, env);
caps = [caps c];
S = fakeProbeLink('state');
assert(S.sl.G.G_sysCurrentState == S.sl.E.E_SystemState.ST_FAULT, 'the drive fault did not put the slave in ST_FAULT');
assert(numel(caps) == 2 && p.failures == 0 && p.lost == 0, '%d captures, %d failures', numel(caps), p.failures);
stream = caps{1};
hist = caps{2};
assert(strcmp(hist.tag, 'fault') && ~hist.lost && size(hist.data, 1) == 160, 'fault capture: %d records', size(hist.data, 1));
assert(hist.mark == S.sp.faultMark && p.faultMark == S.sp.faultMark && hist.first == hist.mark - 100, 'fault mark %d, slave %d', hist.mark, S.sp.faultMark);
st = benchProbe('status', [], hist.data(:, 8));
assert(~any(st.fault(1:100)) && all(st.fault(101:160)), 'fault bit of the status word does not start at the marked record');
assert(~any(st.homing) && all(st.mode == 0));
checkAgainstRing(hist, S, scan);
checkAgainstRing(stream, S, scan);
ss = benchProbe('status', [], stream.data(:, 8));
modes = unique(ss.mode)';
assert(isequal(modes, [0 1]), 'confirmed mode codes in the status column: %s', mat2str(modes));
rowsMode1 = find(ss.mode == 1);
assert(all(diff(rowsMode1) == 1), 'the handshake confirmation is not one block of records');
assert(find(ss.fault, 1) == hist.mark - stream.first + 1, 'the stream''s first fault record is not the marked one');
% The plant's chamber pressure drifts as its oxygen charge dissolves (3 mV in
% the 4 s of the stream), so the raw volts agree with the input to 5 mV.
assert(max(abs(stream.data(:, 7) - S.sl.IO.MO1_AI_01)) < 5e-3 && any(stream.data(:, 7) > 0.5), 'raw volts column');
p = benchProbe('fault', p, struct('step', 3, 'tag', 'again', 'nPre', 10, 'nPost', 10));
[p, c] = benchProbe('drain', p, link, env);
assert(numel(c) == 1 && c{1}.mark == hist.mark && size(c{1}.data, 1) == 20, 'a fault capture without wait did not take the latched mark');
fprintf('ok  drive fault at record %d: history of %d records read back, fault bit set from the marked record, mode code 1 and %.2f V in the new columns\n', ...
    hist.mark, size(hist.data, 1), mean(stream.data(:, 7)));

fprintf('all passed\n');
end


function [link, env] = freshLink(depth)
% A new slave whose records differ from scan to scan (torque noise) and a
% ring of the given depth.
clear fakeProbeLink
fakeProbeLink('init');
fakeProbeLink('ringDepth', depth);
randn('seed', 7);
fakeProbeLink('torqueModel', struct('F0', 0, 'Fc', 0, 'dFs', 0, 'vs', 1, 'b', 0, 'mu', 0, ...
    'noisePct', 5, 'tStick', 0, 'newtonPerPct', 1e9));
link.read     = @(start, n) fakeProbeLink('read', start, n);
link.write    = @(start, v) fakeProbeLink('write', start, v);
link.writeOne = @(addr, v) fakeProbeLink('writeOne', addr, v);
link.close    = @() [];
env.now = @() fakeProbeLink('now');
env.wait = @(t) fakeProbeLink('wait', t);
fakeProbeLink('wait', 0.1);
end


function [p, caps, t] = runSlots(p, link, env, period, tMax, done, nSlots)
% One slot every period seconds until done(p, caps) or tMax seconds (or
% nSlots slots), as the run scripts do in the gaps between exchanges.
if nargin < 7
    nSlots = Inf;
end
caps = {};
t0 = env.now();
k = 0;
while k < nSlots
    env.wait(env.now() + period);
    t = env.now();
    [p, c] = benchProbe('slot', p, link, t);
    caps = [caps c];                            %#ok<AGROW>
    k = k + 1;
    if done(p, caps) || t - t0 > tMax
        break;
    end
end
t = env.now() - t0;
end


function checkAgainstRing(cap, S, scan)
% The capture's records are the log's, converted to engineering units.
R = S.ring;
depth = S.sp.depth;
N = size(cap.data, 1);
idx = cap.first + (0:N - 1)';
assert(all(idx >= S.sp.written - depth), 'capture %s is older than the ring', cap.tag);
rows = R(idx + 1, :);
want = [rows(:, 1) / 1000, rows(:, 2) / 1000, rows(:, 3) / 100, rows(:, 4) / 1e5, ...
    rows(:, 5) / 1000, rows(:, 6) / 5000, rows(:, 7)];
got = cap.data(:, 2:8);
assert(size(got, 2) == 7 && max(abs(got(:) - want(:))) < 1e-9, 'capture %s differs from the log', cap.tag);
assert(max(abs(cap.data(:, 1) - (0:N - 1)' * scan)) < 1e-12);
assert(numel(unique(cap.data(:, 4))) > N / 2, 'records of capture %s are not distinct', cap.tag);
end


function r = probeRegs()
% The probe's registers IR401-524 as one read returns them.
x = fakeProbeLink('read', 401, 124);
u32 = @(i) x(i) + 65536 * x(i + 1);
r = struct('written', u32(1), 'mark', u32(3), 'start', u32(5), 'echo', x(7), 'count', x(8), ...
    'depth', x(9), 'words', x(10), 'window', x(11:114), 'tail', u32(119), 'state', x(121), ...
    'xfer', x(122), 'faultMark', u32(123));
end


function step(n)
% n slave scans.
for k = 1:n
    fakeProbeLink('wait', fakeProbeLink('now') + benchConfig().slaveScan);
end
end


function [seq, at, r] = watchStates(arm, nScans)
% Write HR111-114 and step scan by scan: the probe states in the order they
% appear, and the records written since the mark (Written - Mark) when each
% appeared.
fakeProbeLink('write', 111, arm);
seq = [];
at = [];
for k = 1:nScans
    step(1);
    r = probeRegs();
    if isempty(seq) || r.state ~= seq(end)
        seq(end + 1) = r.state;                     %#ok<AGROW>
        at(end + 1) = r.written - r.mark;           %#ok<AGROW>
    end
end
end


function [seen, wins] = watchWindow(req, n, rewrite)
% Write the window request and step n scans, reading the registers after
% each: [Xfer Start StartTail Count] per scan and the window as read.
seen = zeros(n, 4);
wins = cell(1, n);
fakeProbeLink('write', 112, req);
for k = 1:n
    if rewrite
        fakeProbeLink('write', 112, req);
    end
    step(1);
    r = probeRegs();
    seen(k, :) = [r.xfer r.start r.tail r.count];
    wins{k} = r.window;
end
end


function w = expectedWindow(R, first, count)
% The window words for records first.. of the log R (counts), padded with
% the words the slave has not written.
w = zeros(1, 104);
for i = 0:count - 1
    row = R(first + i + 1, :);
    pos = mod(row(4), 2^32);
    w(8 * i + (1:8)) = [mod(row(1:3), 65536), mod(pos, 65536), floor(pos / 65536), ...
        mod(row(5), 65536), row(6), row(7)];
end
end


function sl = setCfg(sl, name, value)
sl.G.(name) = value;
end


function sl = setWritten(sl, value)
% Test hook: the probe's count of records written (as after a long uptime).
sl.s.fbProbe.Written = value;
end
