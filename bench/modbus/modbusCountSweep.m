function result = modbusCountSweep(nCycles, opts)
%MODBUSCOUNTSWEEP Time Modbus requests against their register count, for
%   the link timing model (slave-sim/modbusLinkConfig.m): whether the
%   controller takes longer to serve a request with more registers, on top
%   of its one-request-per-tick service.
%
%   result = modbusCountSweep              500 cycles, reads only
%   result = modbusCountSweep(nCycles, opts)
%
%   opts.readStart     first input register read (default 1)
%   opts.readCounts    registers per read, FC04 (default [1 2 20 60 125])
%   opts.writeStart    first holding register written, FC16. Default []:
%                      no writes. It must lie inside the holding register
%                      group (opts.holdingLast) and outside every register
%                      the program uses (check the Global Variables
%                      worksheet); HR1-12 (the protocol) and HR111-114 (the
%                      step probe) are refused. HR101-110 belong to the
%                      bench programs: stay below them (writeStart 13 with
%                      the default counts ends at HR100).
%                      The block is read once (FC03) and the same values
%                      are written back, so its content does not change.
%   opts.writeCounts   registers per write (default [1 6 30 88]). The
%                      holding group is shorter than the 123 registers of
%                      the Modbus limit, so the sweep stops at the free
%                      block's length; the model's split at 123 does not
%                      depend on a measurement.
%   opts.holdingLast   last register of the holding group (default 114,
%                      slave-src/GVL/GlobalVariables_Reference.st)
%   opts.address, opts.port   the server (default bench/common/benchConfig)
%
%   Each cycle sends one request of every case, in turn, so drift affects
%   them alike. Requests are sequential (the next is sent once the last is
%   answered), as the protocol's client sends them. The times are MATLAB's
%   and include the client's own delay; capture the run in Wireshark
%   (tcp.port == 502) and run
%       python3 modbusCaptureTiming.py captures/<file>.pcapng
%   for the controller's share per function code and register count, the
%   same-tick share and the per-register slope to set in
%   modbusLinkConfig.perRegister.
%
%   Run with MotionWorks offline, PRG_Main loaded and the axis idle (case A
%   of README.md). The result is also saved under bench/common/captures.

if nargin < 1 || isempty(nCycles)
    nCycles = 500;
end
if nargin < 2
    opts = struct();
end
opts = withDefault(opts, 'readStart', 1);
opts = withDefault(opts, 'readCounts', [1 2 20 60 125]);
opts = withDefault(opts, 'writeStart', []);
opts = withDefault(opts, 'writeCounts', [1 6 30 88]);
opts = withDefault(opts, 'holdingLast', 114);
cfg = modbusBenchConfig();
opts = withDefault(opts, 'address', cfg.address);
opts = withDefault(opts, 'port', cfg.port);
assert(all(opts.readCounts >= 1 & opts.readCounts <= 125), 'a read takes 1 to 125 registers');
assert(all(opts.writeCounts >= 1 & opts.writeCounts <= 123), 'a write takes 1 to 123 registers');

% Refuse writes into the protocol's and the step probe's holding registers
% and past the end of the holding group, before connecting
if ~isempty(opts.writeStart)
    last = opts.writeStart + max(opts.writeCounts) - 1;
    assert(last <= opts.holdingLast, ...
        'writes to HR%d-%d run past the holding group (HR1-%d)', opts.writeStart, last, opts.holdingLast);
    protected = [1 12; 111 114];
    for p = 1:size(protected, 1)
        assert(last < protected(p, 1) || opts.writeStart > protected(p, 2), ...
            'writes to HR%d-%d would overwrite HR%d-%d', opts.writeStart, last, protected(p, :));
    end
end

shared = benchConfig();
uid = uint8(cfg.serverId);
t = tcpclient(opts.address, opts.port, 'Timeout', 1);
cleanup = onCleanup(@() clear('t'));
tid = 0;

% The cases: [function code, first register, count]
cases = [4 * ones(numel(opts.readCounts), 1), opts.readStart * ones(numel(opts.readCounts), 1), ...
         opts.readCounts(:)];
values = [];
if ~isempty(opts.writeStart)
    % The block's present content, written back unchanged
    [values, tid] = transact(t, uid, tid, 3, opts.writeStart, max(opts.writeCounts), []);
    cases = [cases; 16 * ones(numel(opts.writeCounts), 1), ...
             opts.writeStart * ones(numel(opts.writeCounts), 1), opts.writeCounts(:)];
end

nCases = size(cases, 1);
times = nan(nCycles, nCases);
failures = zeros(1, nCases);
firstError = repmat({''}, 1, nCases);
for k = 1:nCycles
    for c = 1:nCases
        fc = cases(c, 1);
        n = cases(c, 3);
        w = [];
        if fc == 16
            w = values(1:n);
        end
        tk = tic;
        try
            [~, tid] = transact(t, uid, tid, fc, cases(c, 2), n, w);
            times(k, c) = toc(tk);
        catch err
            failures(c) = failures(c) + 1;
            if isempty(firstError{c})
                firstError{c} = err.message;
            end
            flush(t);
        end
    end
end

% Report: per case, and the slope of the median against the count
ms = 1000;
fprintf('\n%-6s %6s %6s %8s %8s %8s %8s\n', 'func', 'count', 'fail', 'median', 'p95', 'p99', 'max');
med = nan(1, nCases);
for c = 1:nCases
    x = sort(times(~isnan(times(:, c)), c)) * ms;
    if isempty(x)
        fprintf('FC%02d   %6d %6d   no data: %s\n', cases(c, 1), cases(c, 3), failures(c), firstError{c});
        continue;
    end
    pct = @(p) x(max(1, ceil(p / 100 * numel(x))));
    med(c) = pct(50);
    fprintf('FC%02d   %6d %6d %8.2f %8.2f %8.2f %8.2f ms\n', cases(c, 1), cases(c, 3), failures(c), ...
        med(c), pct(95), pct(99), x(end));
end
for fc = unique(cases(:, 1))'
    sel = cases(:, 1) == fc & ~isnan(med(:));
    if sum(sel) >= 2
        b = [ones(sum(sel), 1), cases(sel, 3)] \ med(sel)';
        fprintf('FC%02d: median %.2f ms + %.4f ms per register (MATLAB side, client delay included)\n', ...
            fc, b(1), b(2));
    end
end

result = struct('cases', cases, 'times', times, 'failures', failures, 'firstError', {firstError}, ...
    'opts', opts, 'nCycles', nCycles);
if ~exist(shared.captures, 'dir')
    mkdir(shared.captures);
end
file = fullfile(shared.captures, sprintf('countSweep_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(file, 'result');
fprintf('saved %s\n', file);
end


function [regs, tid] = transact(t, uid, tid, fc, first, n, w)
% One request and its response. FC03/FC04 return the registers read; FC16
% writes w. Errors on an exception response or a malformed one.
tid = mod(tid + 1, 65536);
switch fc
    case {3, 4}
        pdu = [fc, u16(first - 1), u16(n)];
    case 16
        pdu = [16, u16(first - 1), u16(n), 2 * n];
        for v = w(:)'
            pdu = [pdu, u16(v)]; %#ok<AGROW>
        end
end
write(t, frame(tid, uid, pdu));
head = double(read(t, 7, 'uint8'));         % MBAP header
len = head(5) * 256 + head(6);
body = double(read(t, len - 1, 'uint8'));   % function code and data
rtid = head(1) * 256 + head(2);
if rtid ~= tid
    error('response to transaction %d, expected %d', rtid, tid);
end
if body(1) >= 128
    error('exception response: function %d, code %d', body(1) - 128, body(2));
end
regs = [];
if fc == 3 || fc == 4
    bytes = body(3:2 + body(2));
    regs = bytes(1:2:end) * 256 + bytes(2:2:end);
end
end


function b = u16(v)
% Big-endian 16-bit value as two bytes.
b = uint8([floor(v / 256), mod(v, 256)]);
end


function f = frame(tid, uid, pdu)
% MBAP header (transaction ID, protocol 0, length, unit ID) + PDU.
pdu = uint8(pdu);
f = [u16(tid), uint8([0, 0]), u16(numel(pdu) + 1), uid, pdu];
end


function s = withDefault(s, name, value)
if ~isfield(s, name)
    s.(name) = value;
end
end
