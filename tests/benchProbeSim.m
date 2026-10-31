function benchProbeSim()
%BENCHPROBESIM Test bench/common/benchProbe against the emulation of
%   PRG_StepProbe in fakeProbeLink: an arm whose write is lost is written
%   again and acknowledged; a torn read is discarded and read again; an
%   arm made while an earlier capture is still being read is accepted and
%   both captures come back whole; a stream longer than the ring is read
%   continuously by a reader fast enough and reported lost to a reader
%   that is too slow; drain reads everything pending and gives up on an
%   arm the slave never sees. The record contents are checked against the
%   ring behind the emulation (torque carries white noise, so every
%   record differs).
%   Depends on: master-src/logic slave-src slave-sim tools bench/common tests/fakeProbeLink.m
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'common'));
bcfg = benchConfig();
pr = bcfg.probe;
scan = 0.002;

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
[p, caps, t] = runSlots(p, link, env, 0.01, 6, @(p, caps) false);
assert(isempty(caps) && p.nPending == 1 && p.lost == 0, 'a fast reader lost the stream');
[p, caps] = benchProbe('drain', p, link, env);
S = fakeProbeLink('state');
assert(numel(caps) == 1 && ~caps{1}.lost && p.nPending == 0 && p.failures == 0);
N = size(caps{1}.data, 1);
assert(N > 4 * depth && caps{1}.first + N <= S.sp.written, 'stream of %d records', N);
assert(caps{1}.first + N >= S.sp.written - 40, 'stream ends %d records before the newest', ...
    S.sp.written - caps{1}.first - N);
checkAgainstRing(struct('first', caps{1}.first + N - 400, 'data', [(0:399)' * scan caps{1}.data(end - 399:end, 2:6)], 'tag', 'tail'), S, scan);
d = diff(caps{1}.data(:, 1));
assert(all(abs(d - scan) < 1e-9), 'stream time column not continuous');
fprintf('ok  stream of %d records through a ring of %d, read continuously every 10 ms\n', N, depth);

[link, env] = freshLink(depth);
p = benchProbe('new', pr, scan);
p = benchProbe('arm', p, struct('step', 1, 'tag', 'stream', 'nRecords', Inf));
[p, caps, t] = runSlots(p, link, env, 0.05, 12, @(p, caps) ~isempty(caps));
assert(numel(caps) == 1 && caps{1}.lost && p.lost == 1, 'slow reader not reported lost');
N = size(caps{1}.data, 1);
assert(N > 0 && N < 3 * depth, 'lost stream holds %d records', N);
assert(t < 12, 'overrun found only at %.1f s', t);
fprintf('ok  stream lost to a 50 ms reader after %.1f s (%d records kept)\n', t, N);

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
% The capture's records are the ring's, converted to engineering units.
R = S.ring;
depth = S.sp.depth;
N = size(cap.data, 1);
idx = cap.first + (0:N - 1)';
assert(all(idx >= S.sp.written - depth), 'capture %s is older than the ring', cap.tag);
rows = R(mod(idx, depth) + 1, :);
want = [rows(:, 1) / 1000, rows(:, 2) / 1000, rows(:, 3) / 100, rows(:, 4) / 1e5, rows(:, 5) / 1000];
got = cap.data(:, 2:6);
assert(max(abs(got(:) - want(:))) < 1e-9, 'capture %s differs from the ring', cap.tag);
assert(max(abs(cap.data(:, 1) - (0:N - 1)' * scan)) < 1e-12);
assert(numel(unique(cap.data(:, 4))) > N / 2, 'records of capture %s are not distinct', cap.tag);
end
