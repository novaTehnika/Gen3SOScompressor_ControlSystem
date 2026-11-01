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
%                                            .requests
%   busy = modbusLink('busy', L)             a request not yet answered

switch cmd
    case 'init'
        cfg = varargin{1};
        L.cfg = cfg;
        L.T = cfg.tick;
        if isempty(L.T)
            L.T = varargin{2};
        end
        L.rng = cfg.seed;
        L.req = struct('id', {}, 'kind', {}, 'start', {}, 'n', {}, 'words', {}, ...
            'tSend', {}, 'kServe', {}, 'tResp', {}, 'data', {});
        L.xfer = struct('id', {}, 'kind', {}, 'start', {}, 'tCall', {}, 'nReq', {}, ...
            'nDone', {}, 'tDone', {}, 'data', {}, 'ticks', {});
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
            'nReq', nReq, 'nDone', 0, 'tDone', NaN, 'data', [], 'ticks', []);
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
        fin = find([L.xfer.tDone] <= t);
        done = struct('id', {}, 'kind', {}, 'start', {}, 'data', {}, 'tCall', {}, ...
            'tDone', {}, 'ticks', {}, 'requests', {});
        for x = fin
            X = L.xfer(x);
            done(end + 1) = struct('id', X.id, 'kind', X.kind, 'start', X.start, 'data', X.data, ...
                'tCall', X.tCall, 'tDone', X.tDone, 'ticks', X.ticks, 'requests', X.nReq); %#ok<AGROW>
        end
        if ~isempty(fin)
            gone = [L.xfer(fin).id];
            L.req(ismember([L.req.id], gone)) = [];
            L.xfer(fin) = [];
        end
        varargout = {L, done};

    case 'busy'
        L = varargin{1};
        varargout = {any(isnan([L.req.tResp]))};

    otherwise
        error('modbusLink:cmd', 'unknown command %s', cmd);
end
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
