function modbusLinkTiming()
%MODBUSLINKTIMING Check the Modbus link timing model (slave-sim/modbusLink,
%   modbusLinkConfig) against the bench measurements it encodes
%   (bench/modbus/README.md, section 5) and the effect of register count:
%   requests served on the next tick and one per tick, 81 % of responses in
%   the same tick, a written value read back two ticks after its write is
%   served (on the compiled slave), transfers split by the Modbus limits so
%   that their time grows with the register count, the per-register term,
%   and a repeatable stream.
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

% --- The stream is the link's own and repeats with the seed -------------------
a = durations(cfg);
rand('seed', 7); randn('seed', 7); %#ok<RAND>
b = durations(cfg);
assert(isequal(a, b), 'the link timing depends on the global random state');
fprintf('ok  repeatable: the same seed gives the same timing\n');

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
