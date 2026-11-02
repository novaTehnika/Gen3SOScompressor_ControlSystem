function modbusLinkTiming()
%MODBUSLINKTIMING Check the Modbus link timing model (slave-sim/modbusLink,
%   modbusLinkConfig) against the bench measurements it encodes
%   (bench/modbus/README.md, section 5) and the effect of register count:
%   requests served on the next tick and one per tick, 81 % of responses in
%   the same tick, a written value read back two ticks after its write is
%   served (on the compiled slave), transfers split by the Modbus limits so
%   that their time grows with the register count, the per-register term,
%   and a repeatable stream; and what the Simulink model adds around the
%   link (the bench's Modbus Client block runs, bench/modbus/README.md
%   section 7): steps delayed by the blocking calls (the share of late
%   steps at a 25 ms and a 5 ms step), an exchange now and then a period
%   late, and the model stopping when a call fails on a lost link (bench
%   case D), each with its switch.
%   Depends on: slave-sim slave-src tools models

cfg = modbusLinkConfig();
T = 0.004;                                      % s, the slave's tick

% --- Service tick and same-tick share: 2000 single-register reads -----------
L = modbusLink('init', cfg, T);
t = 0;
wait = zeros(1, 2000);
tDone = 0;
for i = 1:2000
    % Calls spread over the tick, each after the previous one returned
    t = max(t + 0.0173, tDone);
    [L, ~] = modbusLink('submit', L, t, 'read', 1, 1);
    [L, d] = serveUntilDone(L, t, @(q) zeros(1, q.n));
    tDone = d.tDone;
    wait(i) = d.ticks(1) * T - t;               % request -> service (the ACK)
end
assert(all(wait > 0 & wait <= T + 1e-12), 'a request waited %.2f ms', 1000 * max(wait));
assert(abs(median(wait) - T / 2) < 0.15 * T, 'median wait %.2f ms, half a tick expected', 1000 * median(wait));
fprintf('ok  served on the next tick: wait median %.2f ms, max %.2f ms (bench: even over 0.5-4 ms)\n', ...
    1000 * median(wait), 1000 * max(wait));

% Same-tick share, from the responses alone (no client delay)
cfg0 = cfg;
cfg0.clientMedian = 1e-9;
cfg0.clientP99 = 1e-9;
L = modbusLink('init', cfg0, T);
same = 0;
for i = 1:2000
    t = i * 0.05;
    [L, ~] = modbusLink('submit', L, t, 'read', 1, 1);
    [L, d] = serveUntilDone(L, t, @(q) zeros(1, q.n));
    same = same + (d.tDone - d.ticks(1) * T < T / 2);
end
assert(abs(same / 2000 - 0.81) < 0.03, 'same-tick share %.3f', same / 2000);
fprintf('ok  responses in the same tick: %.1f %% (bench: 81 %%)\n', 100 * same / 2000);

% --- One request per tick: a pipelined client's five requests ---------------
cfgP = cfg;
cfgP.pipelined = true;
L = modbusLink('init', cfgP, T);
for i = 1:5
    [L, ~] = modbusLink('submit', L, 0.001, 'read', 1, 1);
end
ticks = [];
for k = 1:20
    [L, reqs] = modbusLink('due', L, k, 'read');
    for q = reqs
        L = modbusLink('answer', L, q.slot, 0);
        ticks(end + 1) = k; %#ok<AGROW>
    end
end
assert(isequal(ticks, 1:5), 'pipelined requests served at ticks %s', mat2str(ticks));
fprintf('ok  one request per tick: five pipelined requests in ticks 1-5\n');

% --- A written value is read back two ticks after its write is served -------
sl = slaveInit();
L = modbusLink('init', cfgP, sl.G.G_cfgScanTime);
[L, ~] = modbusLink('submit', L, 0.0005, 'write', 1, 4242);   % HR1 heartbeat
for k = 1:3
    [L, idR(k)] = modbusLink('submit', L, 0.0005 + (k - 1) * 1e-6, 'read', 1, 1); %#ok<AGROW>
end
echo = zeros(1, 3);
for k = 1:8
    [L, reqs] = modbusLink('due', L, k, 'read');
    for q = reqs
        L = modbusLink('answer', L, q.slot, readInputRegisters(sl, q.start, q.n));
    end
    scan(sl);
    [L, reqs] = modbusLink('due', L, k, 'write');
    for q = reqs
        writeHoldingRegisters(sl, q.start, q.words);
        L = modbusLink('answer', L, q.slot, []);
    end
end
[L, done] = modbusLink('poll', L, 1);
for k = 1:3
    d = done([done.id] == idR(k));
    echo(k) = d.data;
    tk(k) = d.ticks; %#ok<AGROW>
end
assert(echo(1) ~= 4242 && echo(2) == 4242, ...
    'echo %s at ticks %s after a write served at tick 1', mat2str(echo), mat2str(tk));
fprintf('ok  write served at tick 1: IR1 reads %d at tick 2, %d at tick 3 (bench: two ticks later)\n', ...
    echo(1), echo(2));

% --- Register count: transfers split by the Modbus limits --------------------
% Each count is read 20 times; a transfer of r requests is served in r
% ticks, one after another, so its mean time grows by a tick and a client
% delay for each request beyond the first.
counts = [1 20 125 126 250 3000];
L = modbusLink('init', cfg, T);
t = 0;
for c = counts
    for rep = 1:20
        t = t + 1;
        [L, ~] = modbusLink('submit', L, t, 'read', 1, c);
        [L, d] = serveUntilDone(L, t, @(q) zeros(1, q.n));
        assert(numel(d.data) == c && all(diff(d.ticks) >= 1) && numel(d.ticks) == d.requests, ...
            'read of %d: %d words, ticks %s', c, numel(d.data), mat2str(d.ticks));
        dur(rep, counts == c) = d.tDone - t; %#ok<AGROW>
    end
    nReq(counts == c) = d.requests; %#ok<AGROW>
end
mdur = mean(dur);
assert(isequal(nReq, ceil(counts / cfg.maxRead)), 'requests %s', mat2str(nReq));
one = mdur(nReq == 1);
two = mdur(nReq == 2);
assert(max(one) - min(one) < 0.005 && min(two) > max(one) + T, ...
    'mean times %s ms', mat2str(1000 * mdur, 3));
assert(min(dur(:, end)) > 24 * T, '3000 registers in %.1f ms', 1000 * min(dur(:, end)));
[L, ~] = modbusLink('submit', L, 100, 'write', 1, zeros(1, 124));
[L, d] = serveUntilDone(L, 100, @(q) []);
assert(d.requests == 2, 'a 124-register write took %d requests', d.requests);
fprintf('ok  register count: %s registers in %s requests, mean %s ms\n', mat2str(counts), ...
    mat2str(nReq), mat2str(round(1000 * mdur)));

% --- The per-register term ---------------------------------------------------
cfgR = cfg0;
cfgR.pNextTick = 0;
cfgR.perRegister = 1e-4;                        % 0.1 ms per register
L = modbusLink('init', cfgR, T);
[L, ~] = modbusLink('submit', L, 0.001, 'read', 1, 1);
[L, d1] = serveUntilDone(L, 0.001, @(q) zeros(1, q.n));
[L, ~] = modbusLink('submit', L, 1.001, 'read', 1, 101);
[L, d2] = serveUntilDone(L, 1.001, @(q) zeros(1, q.n));
extra = (d2.tDone - 1) - (d1.tDone - 0);
assert(abs(extra - 100 * cfgR.perRegister) < 1e-6, 'per-register term %.3f ms', 1000 * extra);
fprintf('ok  per-register term: 100 more registers at 0.1 ms each add %.1f ms\n', 1000 * extra);

% --- Steps delayed by the blocking calls (the model's overrun) --------------
% 60 s runs of the Read, masterStep, Write exchange every 50 ms against a
% slave that answers at once. Bench, Modbus Client blocks: at a 25 ms step
% 0.21 % of the steps start a tick or more late (2 ticks at most); at 5 ms
% 36.3 % do. The lognormal client delay has a lighter tail than the bench's
% (61 ms at most), so the model is checked to be below the bench at 25 ms
% and near it at 5 ms.
cfgN = cfg;
cfgN.lateExchangeRate = 0;
[~, log25] = runModel(cfgN, 60, 0.025, 0.05);
s25 = log25.stats;
assert(s25.steps >= 2390 && s25.lagging / s25.steps < 0.01 && s25.maxLag <= 2, ...
    '25 ms step: %d of %d steps late, up to %d ticks', s25.lagging, s25.steps, s25.maxLag);
[~, log5] = runModel(cfgN, 60, 0.005, 0.05);
s5 = log5.stats;
share = s5.lagging / s5.steps;
assert(share > 0.25 && share < 0.45 && s5.maxLag >= 5, ...
    '5 ms step: %.1f %% of the steps late, up to %d ticks', 100 * share, s5.maxLag);
dur = [log5.ev([log5.ev.kind] == 'x').dur];
assert(median(dur) > 0.006 && median(dur) < 0.014, 'Read call median %.1f ms', 1000 * median(dur));
fprintf('ok  overrun: %.2f %% of 25 ms steps late (bench 0.21 %%, max %d ticks); %.1f %% of 5 ms steps (bench 36.3 %%, max %d ticks)\n', ...
    100 * s25.lagging / s25.steps, s25.maxLag, 100 * share, s5.maxLag);

% Steps keep their order, none starts before its nominal time or inside a
% Read call, and the model catches up: with 30 ms client delays each
% exchange (every 100 ms here) holds the model for about 66 ms.
cfgS = cfgN;
cfgS.clientMedian = 0.03;
cfgS.clientP99 = 0.03;
[~, logS] = runModel(cfgS, 10, 0.025, 0.1);
ev = logS.ev(ismember([logS.ev.kind], 'pr'));   % steps and exchange steps
assert(isequal([ev.n], 1:numel(ev)), 'steps out of order');
assert(all([ev.lag] >= 0) && all([ev.t] >= [ev.n] * 0.025 - 1e-9), 'a step started before its time');
assert(logS.stats.maxLag >= 1 && logS.stats.maxLag <= 2 && numel(ev) >= 10 / 0.025 - 3, ...
    '%d steps, lag up to %d', numel(ev), logS.stats.maxLag);
x = find([logS.ev.kind] == 'x');
for i = x(1:end - 1)
    nxt = logS.ev(i + 1);                       % what follows the Read call
    assert(nxt.t >= logS.ev(i).t, 'events out of order');
end
for i = find([logS.ev.kind] == 'r')
    j = find([logS.ev(i + 1:end).kind] == 'p', 1) + i;
    k = find([logS.ev(i + 1:end).kind] == 'x', 1) + i;
    assert(isempty(j) || isempty(k) || j > k, 'a step started inside a Read call');
end
fprintf('ok  overrun: with 66 ms exchanges every 100 ms the steps behind run late (up to %d ticks), in order, and the model catches up\n', ...
    logS.stats.maxLag);

% Switched off, the calls do not stall the model
cfgO = cfgN;
cfgO.overrun = false;
[~, logO] = runModel(cfgO, 10, 0.005, 0.05);
assert(logO.stats.lagging == 0 && logO.stats.steps == 2000 && logO.stats.exchanges > 190, ...
    'overrun off: %d steps, %d late', logO.stats.steps, logO.stats.lagging);
fprintf('ok  overrun off: %d steps on the nominal clock, none late\n', logO.stats.steps);

% --- Exchanges that arrive a period late ----------------------------------
assert(abs(cfg.lateExchangeRate * 60 - 1) < 1e-12, 'default: one late exchange a minute');
cfgL = cfg;
cfgL.lateExchangeRate = 20 / 60;                % twenty times the bench rate
[~, logL] = runModel(cfgL, 60, 0.025, 0.05);
sL = logL.stats;
gap = diff(sL.tExchange);
nGap = sum(gap > 1.7 * 0.05);                  % (the last one may fall after the run)
assert(sL.late >= 8 && sL.late <= 35, '%d late exchanges in 60 s at 20 a minute', sL.late);
assert(nGap >= sL.late - 1 && nGap <= sL.late && max(gap) < 2.5 * 0.05, ...
    '%d late exchanges, %d gaps of two periods, longest %.0f ms', sL.late, nGap, 1000 * max(gap));
assert(sL.exchanges + sL.late >= 1198 && sL.exchanges + sL.late <= 1200, ...
    '%d exchanges and %d late of 1200 due', sL.exchanges, sL.late);
cfgL.lateExchangeRate = 0;
[~, logL0] = runModel(cfgL, 20, 0.025, 0.05);
assert(logL0.stats.late == 0 && max(diff(logL0.stats.tExchange)) < 1.7 * 0.05, 'late exchange with the rate at 0');
fprintf('ok  late exchange: %d of 1200 at 20 a minute, each a gap of two periods (longest %.0f ms); none at rate 0\n', ...
    sL.late, 1000 * max(gap));

% --- A lost link stops the model ------------------------------------------
% The connection drops while the model is between calls: the next Read
% call fails cfg.lostLinkErrorDelay after it was made and the model stops
% there, with no further step.
cfgD = cfg;
cfgD.lateExchangeRate = 0;
cfgD.lostLinkErrorDelay = 0.02;
L = modbusLink('init', cfgD, T);
L = modbusLink('modelInit', L, 0.025, 0.05, 20);
[L, lg] = runTicks(L, 1, 90);                   % exchange at 50 ms, done by 90 ms
assert(~L.model.stopped && any([lg.ev.kind] == 'x') && strcmp(L.model.x, 'none'));
L = modbusLink('cut', L, 0.090);
[L, lg] = runTicks(L, 91, 400);
stopEv = lg.ev([lg.ev.kind] == 's');
callEv = lg.ev([lg.ev.kind] == 'r');
assert(numel(stopEv) == 1 && numel(callEv) == 1, 'stop events: %d', numel(stopEv));
assert(abs(stopEv.t - (callEv.t + 0.02)) < 1e-9, 'stopped at %.3f, call made at %.3f', stopEv.t, callEv.t);
assert(callEv.t >= 0.100 - 1e-9 && callEv.t < 0.110, 'the call was made at %.3f s', callEv.t);
iStop = find([lg.ev.kind] == 's');
assert(iStop == numel(lg.ev), 'events after the stop');
st = modbusLink('modelStats', L);
assert(st.stopped && abs(st.tStop - stopEv.t) < 1e-9);
nSteps = st.steps;
L = modbusLink('restore', L);
[L, lg] = runTicks(L, 401, 700);
assert(isempty(lg.ev) && modbusLink('modelStats', L).steps == nSteps, ...
    'a stopped model went on after the link was restored');
fprintf('ok  lost link: the Read call made at %.0f ms failed %.0f ms later and stopped the model; it stays stopped\n', ...
    1000 * callEv.t, 1000 * (stopEv.t - callEv.t));

% A call in flight when the connection drops fails then.
L = modbusLink('init', cfgD, T);
L = modbusLink('modelInit', L, 0.025, 0.05, 20);
[L, lg] = runTicks(L, 1, 52);                   % the Read call was sent at 50 ms
assert(strcmp(L.model.x, 'read'), 'no call in flight');
L = modbusLink('cut', L, 0.052);
[L, lg] = runTicks(L, 53, 200);
stopEv = lg.ev([lg.ev.kind] == 's');
assert(numel(stopEv) == 1 && abs(stopEv.t - 0.072) < 1e-9, 'the call in flight failed at %.3f', stopEv.t);
fprintf('ok  lost link: a call in flight when the connection dropped (52 ms) failed at %.0f ms\n', 1000 * stopEv.t);

% Switched off, a failed call returns no data and the model goes on; the
% calls work again once the connection is back.
cfgD.stopOnLostLink = false;
L = modbusLink('init', cfgD, T);
L = modbusLink('modelInit', L, 0.025, 0.05, 20);
[L, lg1] = runTicks(L, 1, 120);
L = modbusLink('cut', L, 0.120);
[L, lg2] = runTicks(L, 121, 400);
L = modbusLink('restore', L);
[L, lg3] = runTicks(L, 401, 800);
bad = lg2.ev([lg2.ev.kind] == 'x');
assert(~isempty(bad) && all([bad.failed]), 'no failed exchange while cut');
good = lg3.ev([lg3.ev.kind] == 'x' & ~[lg3.ev.failed]);
assert(numel(good) >= 6 && ~L.model.stopped && ~any([lg2.ev.kind] == 's'), ...
    'the model did not carry on: %d good exchanges, stopped %d', numel(good), L.model.stopped);
fprintf('ok  lost link with stopOnLostLink off: %d failed exchanges, then %d good ones; no stop\n', ...
    numel(bad), numel(good));

% --- The stream is the link's own and repeats with the seed -------------------
a = durations(cfg);
rand('seed', 7); randn('seed', 7); %#ok<RAND>
b = durations(cfg);
assert(isequal(a, b), 'the link timing depends on the global random state');
cfgL.lateExchangeRate = 5;
[~, m1] = runModel(cfgL, 5, 0.005, 0.05);
rand('seed', 8); randn('seed', 8); %#ok<RAND>
[~, m2] = runModel(cfgL, 5, 0.005, 0.05);
assert(isequal(m1.ev, m2.ev) && m1.stats.late > 0, 'the model run depends on the global random state');
fprintf('ok  repeatable: the same seed gives the same timing and the same model run\n');

fprintf('all passed\n');
end


function [L, d] = serveUntilDone(L, t, readData)
% Serve the link's requests tick by tick from time t until the transfer
% submitted last is done; readData(q) gives the words of a read.
k = floor(t / L.T);
d = [];
while isempty(d)
    k = k + 1;
    for phase = {'read', 'write'}
        [L, reqs] = modbusLink('due', L, k, phase{1});
        for q = reqs
            if strcmp(q.kind, 'read')
                L = modbusLink('answer', L, q.slot, readData(q));
            else
                L = modbusLink('answer', L, q.slot, []);
            end
        end
    end
    [L, done] = modbusLink('poll', L, k * L.T + 1);
    if ~isempty(done)
        d = done(end);
    end
end
end


function dur = durations(cfg)
L = modbusLink('init', cfg, 0.004);
dur = zeros(1, 50);
for i = 1:50
    t = i * 0.1;
    [L, ~] = modbusLink('submit', L, t, 'read', 1, 20);
    [L, d] = serveUntilDone(L, t, @(q) zeros(1, q.n));
    dur(i) = d.tDone - t;
end
end


function [L, lg] = runTicks(L, tick0, tick1)
% Run the model and an all-zero slave on the 1 ms tick from tick0 to tick1:
% the model's events as they come (lg.ev: t, kind 'p' step, 'r' Read sent,
% 'x' exchange, 's' stopped, with n, lag, dur, failed) and the link's
% requests served around each slave scan, a 4 ms tick, reads before it
% and writes after it. The model answers each exchange with a write.
ev = struct('t', {}, 'kind', {}, 'n', {}, 'lag', {}, 'dur', {}, 'failed', {});
code = struct('step', 'p', 'read', 'r', 'exchange', 'x', 'stopped', 's');
for tick = tick0:tick1
    t = tick / 1000;
    e.kind = '';
    while ~strcmp(e.kind, 'none')
        [L, e] = modbusLink('modelPoll', L, t);
        if ~strcmp(e.kind, 'none')
            ev(end + 1) = struct('t', e.t, 'kind', code.(e.kind), 'n', e.n, 'lag', e.lag, ...
                'dur', e.dur, 'failed', e.failed); %#ok<AGROW>
        end
        if strcmp(e.kind, 'exchange')
            L = modbusLink('modelWrite', L, t, zeros(1, 8));
        end
    end
    if mod(tick, 4) == 0
        k = tick / 4;
        [L, reqs] = modbusLink('due', L, k, 'read');
        for q = reqs
            L = modbusLink('answer', L, q.slot, zeros(1, q.n));
        end
        [L, reqs] = modbusLink('due', L, k, 'write');
        for q = reqs
            L = modbusLink('answer', L, q.slot, []);
        end
    end
end
lg.ev = ev;
lg.stats = modbusLink('modelStats', L);
end


function [L, lg] = runModel(cfg, T, dt, period)
% A run of the model for T seconds with 20 registers read and 8 written.
L = modbusLink('init', cfg, 0.004);
L = modbusLink('modelInit', L, dt, period, 20);
[L, lg] = runTicks(L, 1, round(T * 1000));
end
