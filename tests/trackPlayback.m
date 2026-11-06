function trackPlayback()
%TRACKPLAYBACK The shell's track playback (trackStep) on its own: the table
%   index against the model step count, the status gate, the restart on a
%   new sequence number and the pass-through outside a track.
%
%   Run from the repository root:  addpath master-src/logic tests; trackPlayback
%   Depends on: master-src/logic

cfg = masterConfig();
spe = round(cfg.exchangePeriod / cfg.dt);   % model steps per table value
assert(spe == 2);
tbl = 1:10;

% ----- Pass-through and the start gate -----
clear trackStep
[xp, xq, idx] = trackStep(111, 222, tbl, 0, 0, 5);
assert(xp == 111 && xq == 222 && idx == 0, 'mode 0 passes through');
for i = 1:10
    [xp, xq, idx] = trackStep(111, 222, tbl, 0, 0, 5);
    assert(xp == 111 && xq == 222 && idx == 0, 'before a track the output is the input');
end
% A track whose mode lands one step before its sequence number restarts at
% the first value when the number changes.
for i = 1:6
    trackStep(111, 222, tbl, 4, 0, 5);
end
[~, xq] = trackStep(111, 222, tbl, 4, 1, 5);
assert(xq == 1, 'the sequence number restarts the series');

% ----- Status gate: the index holds at 1 until the mode's status -----
clear trackStep
trackStep(0, 0, tbl, 0, 0, 0);
for i = 1:6
    [~, xq, idx] = trackStep(111, 222, tbl, 4, 1, 7);
    assert(xq == 1 && idx == 1, 'starting (7) does not advance');
end
for i = 1:4
    [~, xq] = trackStep(111, 222, tbl, 4, 1, 2);
    assert(xq == 1, 'a position status does not advance a pressure track');
end

% ----- Pressure: one value per two calls, holding the end -----
seen = zeros(1, 30);
for i = 1:30
    [~, xq, idx] = trackStep(111, 222, tbl, 4, 1, 5);
    assert(xq == tbl(idx));
    seen(i) = xq;
end
want = min(floor((0:29) / spe) + 1, 10);
assert(isequal(seen, want), 'the index advances every second call and holds 10');
[~, xq, idx] = trackStep(111, 222, tbl, 4, 1, 5);
assert(xq == 10 && idx == 10);
% The count stopped at the end: leaving the status and returning does not
% change what is held.
[~, xq] = trackStep(111, 222, tbl, 4, 1, 7);
assert(xq == 10);

% ----- A new sequence number restarts -----
[~, xq] = trackStep(111, 222, tbl, 4, 2, 5);
assert(xq == 1, 'restart at the first value');
[~, xq] = trackStep(111, 222, tbl, 4, 2, 5);
assert(xq == 1);
[~, xq] = trackStep(111, 222, tbl, 4, 2, 5);
assert(xq == 2);
% The track ends: the targets pass through again and the index reads 0.
[xp, xq, idx] = trackStep(111, 222, tbl, 0, 2, 5);
assert(xp == 111 && xq == 222 && idx == 0);

% ----- Position mode drives the position and counts on 2 and 3 -----
clear trackStep
trackStep(0, 0, tbl, 0, 0, 0);
[xp, xq, idx] = trackStep(111, 222, tbl, 2, 1, 5);
assert(xp == 1 && xq == 222 && idx == 1);
for i = 1:6
    trackStep(111, 222, tbl, 2, 1, 5);
end
[xp] = trackStep(111, 222, tbl, 2, 1, 5);
assert(xp == 1, 'status 5 does not advance a position track');
got = zeros(1, 8);
for i = 1:8
    status = 2 + mod(i, 2);          % 2 and 3 alternate
    got(i) = trackStep(111, 222, tbl, 2, 1, status);
end
assert(isequal(got, [1 1 2 2 3 3 4 4]), mat2str(got));

% ----- The first call takes the sequence number as it is -----
clear trackStep
[xp, xq, idx] = trackStep(111, 222, tbl, 0, 7, 5);
assert(xp == 111 && xq == 222 && idx == 0, 'a nonzero counter at model start passes through');
for i = 1:4
    [~, xq] = trackStep(111, 222, tbl, 4, 7, 7);
    assert(xq == 1, 'no start without the status');
end
[~, xq] = trackStep(111, 222, tbl, 4, 8, 5);
[~, xq] = trackStep(111, 222, tbl, 4, 8, 5);
[~, xq] = trackStep(111, 222, tbl, 4, 8, 5);
assert(xq == 2, 'a change after that plays');

% ----- A long table is indexed in place and ends at its last value -----
clear trackStep
big = (1:cfg.trackTableLength) * 1e-3;
trackStep(0, 0, big, 0, 0, 0);
[~, xq, idx] = trackStep(0, 0, big, 4, 1, 5);
assert(idx == 1 && xq == big(1));
fprintf('ok  trackStep: pass-through, start gate, status gate, pace, hold, restart, position mode\n');
clear trackStep
end
