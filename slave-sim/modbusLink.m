function varargout = modbusLink(cmd, varargin)
%MODBUSLINK Timing model of a Modbus TCP connection to the controller
%   (modbusLinkConfig gives the measured figures). A transfer (one call of
%   the client: a read or a write of n registers) is split into requests of
%   at most cfg.maxRead / cfg.maxWrite registers. The controller serves one
%   request per tick of its task, in order; a request is served at the
%   first tick after it arrives. Its response leaves in that tick, or one
%   tick later with probability cfg.pNextTick, plus cfg.perRegister per
%   register. The client sends its next request, or its call returns, a
%   client delay after the response.
%
%   The link only keeps time; the caller serves the requests on the slave's
%   registers when the link says they are due, around each scan k (at time
%   k * tick): reads before the scan, so they return what the last scan
%   left, and writes after it, so the program sees them at the next scan.
%   A written value is therefore read back two ticks after its write is
%   served.
%
%   What the Simulink shell adds around the link is in the model commands
%   below (the last block of modbusLinkConfig; each effect has a switch):
%   the shell's step calls the Read block, masterStep and the Write block
%   in that order at the exchange period, and each Modbus call blocks the
%   model. The model then
%     - stalls for the length of the calls: a step that falls due while a
%       call is in flight starts when the call returns, and the steps
%       behind it run back to back until the model is on its clock again
%       (a step's lag is the whole ticks it started late, the bench's
%       Missed Ticks); cfg.overrun;
%     - receives an exchange a period late now and then (cfg.lateExchangeRate
%       per second of model time, a draw per exchange from the link's
%       stream): the exchange due at that step is held back one exchange
%       period and the one due then merges with it, so the read results
%       have a gap of two periods; cfg.lateExchangeRate;
%     - stops when a call fails on a lost link: 'cut' drops the
%       connection, the unanswered calls and every later one fail
%       cfg.lostLinkErrorDelay after they were made, and the failure of a
%       Read or Write call ends the run (the Modbus Client blocks raise an
%       error, with no retry; bench case D); cfg.stopOnLostLink. A stopped
%       model makes no more steps, and its DAQ outputs go to their final
%       values (the caller applies them).
%   With cfg.overrun off the calls do not stall the model (an exchange due
%   while the previous one is in flight is skipped); with cfg.stopOnLostLink
%   off a failed Read returns no data (ev.failed) and the model goes on. The
%   late exchanges and the link's delays come from the same stream, so a
%   run repeats from cfg.seed.
%
%   L = modbusLink('init', cfg, tick)        cfg from modbusLinkConfig, tick
%                                            in s (cfg.tick if not empty)
%   [L, id] = modbusLink('submit', L, t, 'read', start, n)
%   [L, id] = modbusLink('submit', L, t, 'write', start, words)
%                                            a call made at time t (s)
%   [L, reqs] = modbusLink('due', L, k, phase)
%                                            the requests to serve at scan k:
%                                            phase 'read' before the scan,
%                                            'write' after it; reqs(i) has
%                                            .slot, .kind, .start, .n, .words
%   L = modbusLink('answer', L, slot, data)  a due request served (data: the
%                                            words read, [] for a write)
%   [L, done] = modbusLink('poll', L, t)     the transfers finished by t:
%                                            .id, .kind, .start, .data,
%                                            .tCall, .tDone, .ticks (served),
%                                            .requests, .failed (the link
%                                            was cut; no data)
%   busy = modbusLink('busy', L)             a request not yet answered
%   L = modbusLink('cut', L, t)              the connection drops at t (s)
%   L = modbusLink('restore', L)             it is back; new calls work (a
%                                            stopped model stays stopped)
%
%   The model, driven by the caller on its tick:
%   L = modbusLink('modelInit', L, dt, period, nRead, t0)
%                                            starts a run at t0 (s, default
%                                            0): a step every dt (s), the
%                                            first due at t0 + dt; every
%                                            period (s, a multiple of dt)
%                                            the step reads nRead registers
%                                            from 1 and then writes
%   [L, ev] = modbusLink('modelPoll', L, t)  what happens at time t; call it
%                                            until ev.kind is 'none'.
%                                            ev.kind:
%                                            'step'     a step without a call:
%                                                       run masterStep on the
%                                                       last read
%                                            'read'     the exchange step has
%                                                       sent its Read call
%                                            'exchange' the Read returned
%                                                       (ev.data; empty with
%                                                       ev.failed when the
%                                                       call failed): run
%                                                       masterStep, then call
%                                                       modelWrite
%                                            'stopped'  the model stopped
%                                                       (ev.t, once)
%                                            'none'     nothing is due
%                                            ev.n is the step's index and
%                                            ev.lag its lag in ticks; for
%                                            'exchange' ev.dur is the Read
%                                            call's duration (s)
%   L = modbusLink('modelWrite', L, t, words)
%                                            the exchange's Write call; empty
%                                            words make no call
%   L = modbusLink('late', L)                the next exchange arrives late
%   s = modbusLink('modelStats', L)          the model's state: .steps,
%                                            .lagging (lag of 1 tick or more),
%                                            .lagOver5, .maxLag, .exchanges,
%                                            .late, .tExchange (times of the
%                                            read results), .stopped, .tStop

switch cmd
    case 'init'
        cfg = varargin{1};
        L.cfg = cfg;
        L.T = cfg.tick;
        if isempty(L.T)
            L.T = varargin{2};
        end
        L.rng = cfg.seed;
        L.cut = false;          % the connection is down
        L.model = [];
        L.req = struct('id', {}, 'kind', {}, 'start', {}, 'n', {}, 'words', {}, ...
            'tSend', {}, 'kServe', {}, 'tResp', {}, 'data', {});
        L.xfer = struct('id', {}, 'kind', {}, 'start', {}, 'tCall', {}, 'nReq', {}, ...
            'nDone', {}, 'tDone', {}, 'data', {}, 'ticks', {}, 'failed', {});
        L.nextId = 1;
        L.lastServe = -Inf;     % the tick of the last request served or booked
        L.tFree = -Inf;         % when the client may send its next request
        varargout = {L};

    case 'submit'
        [L, t, kind, start, arg] = varargin{:};
        if strcmp(kind, 'read')
            n = arg;
            words = [];
            limit = L.cfg.maxRead;
        else
            words = arg(:)';
            n = numel(words);
            limit = L.cfg.maxWrite;
        end
        id = L.nextId;
        L.nextId = id + 1;
        nReq = ceil(n / limit);
        if L.cut
            % No connection: the call fails without a request
            L.xfer(end + 1) = struct('id', id, 'kind', kind, 'start', start, 'tCall', t, ...
                'nReq', 0, 'nDone', 0, 'tDone', t + L.cfg.lostLinkErrorDelay, 'data', [], ...
                'ticks', [], 'failed', true);
            varargout = {L, id};
            return;
        end
        for r = 1:nReq
            k0 = (r - 1) * limit;
            m = min(limit, n - k0);
            w = [];
            if ~isempty(words)
                w = words(k0 + 1:k0 + m);
            end
            L.req(end + 1) = struct('id', id, 'kind', kind, 'start', start + k0, 'n', m, ...
                'words', w, 'tSend', NaN, 'kServe', NaN, 'tResp', NaN, 'data', []);
        end
        L.xfer(end + 1) = struct('id', id, 'kind', kind, 'start', start, 'tCall', t, ...
            'nReq', nReq, 'nDone', 0, 'tDone', NaN, 'data', [], 'ticks', [], 'failed', false);
        L = sendReady(L, t);
        varargout = {L, id};

    case 'due'
        [L, k, phase] = varargin{:};
        slots = find([L.req.kServe] == k & strcmp({L.req.kind}, phase) & isnan([L.req.tResp]));
        reqs = struct('slot', {}, 'kind', {}, 'start', {}, 'n', {}, 'words', {});
        for i = slots
            q = L.req(i);
            reqs(end + 1) = struct('slot', i, 'kind', q.kind, 'start', q.start, 'n', q.n, ...
                'words', q.words); %#ok<AGROW>
        end
        varargout = {L, reqs};

    case 'answer'
        [L, slot, data] = varargin{:};
        q = L.req(slot);
        [u, L.rng] = uniform(L.rng);
        tResp = q.kServe * L.T + (u < L.cfg.pNextTick) * L.T + L.cfg.perRegister * q.n;
        [d, L.rng] = clientDelay(L.rng, L.cfg);
        L.req(slot).tResp = tResp;
        L.req(slot).data = data(:)';
        L.tFree = max(L.tFree, tResp + d);
        x = find([L.xfer.id] == q.id);
        L.xfer(x).nDone = L.xfer(x).nDone + 1;
        L.xfer(x).ticks(end + 1) = q.kServe;
        if L.xfer(x).nDone == L.xfer(x).nReq
            mine = find([L.req.id] == q.id);
            L.xfer(x).data = [L.req(mine).data];
            L.xfer(x).tDone = tResp + d;
        end
        L = sendReady(L, tResp + d);
        varargout = {L};

    case 'poll'
        [L, t] = varargin{:};
        [L, done] = collect(L, find([L.xfer.tDone] <= t));
        varargout = {L, done};

    case 'busy'
        L = varargin{1};
        varargout = {any(isnan([L.req.tResp]))};

    case 'cut'
        [L, t] = varargin{:};
        L.cut = true;
        open = find(isnan([L.xfer.tDone]) | [L.xfer.tDone] > t);   % not yet back at the client
        for x = open
            L.xfer(x).failed = true;
            L.xfer(x).tDone = t + L.cfg.lostLinkErrorDelay;
        end
        L.req(ismember([L.req.id], [L.xfer(open).id]) & isnan([L.req.tResp])) = [];
        varargout = {L};

    case 'restore'
        L = varargin{1};
        L.cut = false;
        varargout = {L};

    case 'modelInit'
        [L, dt, period, nRead] = varargin{1:4};
        t0 = 0;
        if numel(varargin) > 4
            t0 = varargin{5};
        end
        L.model = struct('t0', t0, 'dt', dt, 'period', round(period / dt), 'nRead', nRead, ...
            'n', 1, 'x', 'none', 'id', 0, 'deferred', false, 'lateNext', false, ...
            'stopped', false, 'reported', false, 'tStop', NaN, 'steps', 0, 'lagging', 0, ...
            'lagOver5', 0, 'maxLag', 0, 'exchanges', 0, 'late', 0, 'tExchange', []);
        varargout = {L};

    case 'late'
        L = varargin{1};
        L.model.lateNext = true;
        varargout = {L};

    case 'modelPoll'
        [L, t] = varargin{:};
        [L, ev] = modelPoll(L, t);
        varargout = {L, ev};

    case 'modelWrite'
        [L, t, words] = varargin{:};
        if isempty(words)
            L.model.x = 'none';
        else
            [L, id] = modbusLink('submit', L, t, 'write', 1, words);
            L.model.id = id;
            L.model.x = 'write';
        end
        varargout = {L};

    case 'modelStats'
        L = varargin{1};
        varargout = {L.model};

    otherwise
        error('modbusLink:cmd', 'unknown command %s', cmd);
end
end


function [L, done] = collect(L, fin)
% The finished transfers fin (indices in L.xfer) as call records, removed
% from the link together with their requests.
done = struct('id', {}, 'kind', {}, 'start', {}, 'data', {}, 'tCall', {}, ...
    'tDone', {}, 'ticks', {}, 'requests', {}, 'failed', {});
for x = fin
    X = L.xfer(x);
    done(end + 1) = struct('id', X.id, 'kind', X.kind, 'start', X.start, 'data', X.data, ...
        'tCall', X.tCall, 'tDone', X.tDone, 'ticks', X.ticks, 'requests', X.nReq, ...
        'failed', X.failed); %#ok<AGROW>
end
if ~isempty(fin)
    gone = [L.xfer(fin).id];
    L.req(ismember([L.req.id], gone)) = [];
    L.xfer(fin) = [];
end
end


function [L, ev] = modelPoll(L, t)
% One event of the Simulink model at time t (see the header). A step runs
% when the model is not blocked in a call and the step's nominal time
% n * dt has come; a late step finds t past n * dt.
M = L.model;
cfg = L.cfg;
ev = struct('kind', 'none', 'n', M.n, 'lag', 0, 'data', [], 'failed', false, 't', t, 'dur', 0);
if M.stopped
    if ~M.reported
        M.reported = true;
        ev.kind = 'stopped';
        ev.t = M.tStop;
    end
    L.model = M;
    return;
end

% The call in flight: its return ends the Read block (the caller then
% steps) or the Write block (the exchange is over).
if strcmp(M.x, 'read') || strcmp(M.x, 'write')
    x = find([L.xfer.id] == M.id & [L.xfer.tDone] <= t, 1);
    if ~isempty(x)
        [L, d] = collect(L, x);
        if d.failed && cfg.stopOnLostLink
            M.stopped = true;
            M.reported = true;
            M.tStop = d.tDone;
            ev.kind = 'stopped';
            ev.t = d.tDone;
        elseif strcmp(M.x, 'read')
            M.x = 'master';
            M.exchanges = M.exchanges + 1;
            M.tExchange(end + 1) = t;
            ev.kind = 'exchange';
            ev.data = d.data;
            ev.failed = d.failed;
            ev.dur = d.tDone - d.tCall;
        else
            M.x = 'none';
        end
        if ~strcmp(ev.kind, 'none')
            L.model = M;
            return;
        end
    end
end

% A step falls due unless a call blocks the model or the caller still owes
% the Write
if strcmp(M.x, 'master') || (cfg.overrun && ~strcmp(M.x, 'none'))
    L.model = M;
    return;
end
tn = M.t0 + M.n * M.dt;                 % [s] nominal time of the step
if t < tn - 1e-9
    L.model = M;
    return;
end
lag = floor((t - tn) / M.dt + 1e-9);    % [ticks] started late
M.steps = M.steps + 1;
M.lagging = M.lagging + (lag >= 1);
M.lagOver5 = M.lagOver5 + (lag > 5);
M.maxLag = max(M.maxLag, lag);
ev.n = M.n;
ev.lag = lag;

% An exchange step: the period's exchange, held back one period when it
% arrives late
exchange = mod(M.n, M.period) == 0;
if exchange
    if M.deferred
        M.deferred = false;
    else
        late = M.lateNext;
        if ~late && cfg.lateExchangeRate > 0
            [u, L.rng] = uniform(L.rng);
            late = u < cfg.lateExchangeRate * M.period * M.dt;
        end
        if late
            M.lateNext = false;
            M.deferred = true;
            M.late = M.late + 1;
            exchange = false;
        end
    end
end
if exchange && strcmp(M.x, 'none')
    [L, M.id] = modbusLink('submit', L, t, 'read', 1, M.nRead);
    M.x = 'read';
    ev.kind = 'read';
else
    ev.kind = 'step';
end
M.n = M.n + 1;
L.model = M;
end


function L = sendReady(L, t)
% Send the requests the client may send at time t: with a sequential
% client the next unsent one once nothing is in flight; with a pipelined
% one every unsent one. Each is booked for the first tick after it arrives
% and after the tick of the request before it (one request per tick).
while true
    unsent = find(isnan([L.req.tSend]), 1);
    inFlight = any(~isnan([L.req.tSend]) & isnan([L.req.tResp]));
    if isempty(unsent) || (inFlight && ~L.cfg.pipelined)
        return;
    end
    tSend = max(t, L.tFree);
    k = max(floor(tSend / L.T + 1e-9) + 1, L.lastServe + 1);
    L.req(unsent).tSend = tSend;
    L.req(unsent).kServe = k;
    L.lastServe = k;
end
end


function [u, s] = uniform(s)
% A uniform draw on [0, 1) from a 31-bit linear congruential stream.
s = mod(1103515245 * s + 12345, 2^31);
u = s / 2^31;
end


function [d, s] = clientDelay(s, cfg)
% A lognormal draw with the configured median and 99th percentile.
[u1, s] = uniform(s);
[u2, s] = uniform(s);
z = sqrt(-2 * log(max(u1, 1e-12))) * cos(2 * pi * u2);
sigma = log(cfg.clientP99 / cfg.clientMedian) / 2.3263;
d = cfg.clientMedian * exp(sigma * z);
end
