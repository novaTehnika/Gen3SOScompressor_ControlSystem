function result = modbusRawProbe(nCycles)
%MODBUSRAWPROBE Time the heartbeat exchange with hand-built Modbus TCP
%   frames over a plain tcpclient, bypassing the modbus object. Measures
%   the client-side floor and whether the controller accepts the write and
%   the read back to back (pipelined) in one service tick.
%
%   result = modbusRawProbe(nCycles)   nCycles per variant (default 1000)
%
%   Variants:
%     sequential   FC06 write, wait for its response, FC04 read, wait.
%                  Same traffic as modbusProbe.
%     two writes   both requests sent as two write() calls, then both
%                  responses read. Nagle may hold the second request until
%                  the first is acknowledged.
%     one write    both requests in one write() call, so they share one TCP
%                  segment. Needs a server that handles two ADUs per segment.
%
%   Uses HR1 (G_mbHeartbeat) and IR1-2 (G_mbHeartbeatEcho, G_mbSlaveCounter)
%   from slave/PRG_ModbusBench.st.

if nargin < 1
    nCycles = 1000;
end
cfg = modbusBenchConfig();
uid = uint8(cfg.serverId);

t = tcpclient(cfg.address, cfg.port, 'Timeout', 1);
% Response lengths: FC06 echoes the request (6 + 6), FC04 for 2 registers
% is 6 + 7.
nWriteResp = 12;
nReadResp = 13;

variants = {'sequential', 'two writes', 'one write'};
result = struct();
for v = 1:numel(variants)
    name = variants{v};
    tTotal = nan(1, nCycles);
    lag = nan(1, nCycles);
    failures = 0;
    firstError = '';
    tid = 0;
    for k = 1:nCycles
        seq = mod(k, 65536);
        tid = mod(tid + 2, 65536);
        reqW = frame(tid, uid, [6, u16(cfg.hrHeartbeat - 1), u16(seq)]);
        reqR = frame(tid + 1, uid, [4, u16(cfg.irHeartbeatEcho - 1), u16(2)]);
        flush(t);
        tk = tic;
        try
            switch name
                case 'sequential'
                    write(t, reqW);
                    rW = read(t, nWriteResp, 'uint8');
                    write(t, reqR);
                    rR = read(t, nReadResp, 'uint8');
                    raw = [rW, rR];
                case 'two writes'
                    write(t, reqW);
                    write(t, reqR);
                    raw = read(t, nWriteResp + nReadResp, 'uint8');
                case 'one write'
                    write(t, [reqW, reqR]);
                    raw = read(t, nWriteResp + nReadResp, 'uint8');
            end
            tTotal(k) = toc(tk);
            regs = parseRead(raw, tid + 1);
            lag(k) = mod(seq - regs(1), 65536);
        catch err
            failures = failures + 1;
            if isempty(firstError)
                firstError = err.message;
            end
        end
    end

    ms = 1000;
    ok = ~isnan(tTotal);
    fprintf('\n%s: %d cycles, %d failed\n', name, nCycles, failures);
    if failures > 0
        fprintf('  first failure: %s\n', firstError);
    end
    printStats('  exchange (write + read)', tTotal(ok) * ms);
    if any(ok)
        fprintf('  echo matched in the same cycle: %.2f %%\n', 100 * mean(lag(ok) == 0));
    end
    field = strrep(name, ' ', '_');
    result.(field).totalMs = tTotal * ms;
    result.(field).failures = failures;
    result.(field).firstError = firstError;
end
clear t
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


function regs = parseRead(raw, tidRead)
% Walk the ADUs in raw and return the two registers of the FC04 response
% with transaction ID tidRead. Errors if it is missing or is an exception.
raw = double(raw(:)');
p = 1;
while p + 6 <= numel(raw)
    tid = raw(p) * 256 + raw(p + 1);
    len = raw(p + 4) * 256 + raw(p + 5);
    adu = raw(p:min(numel(raw), p + 5 + len));
    if numel(adu) < 6 + len
        error('short response (%d of %d bytes)', numel(adu), 6 + len);
    end
    fc = adu(8);
    if fc >= 128
        error('exception response: function %d, code %d', fc - 128, adu(9));
    end
    if tid == tidRead && fc == 4
        regs = [adu(10) * 256 + adu(11), adu(12) * 256 + adu(13)];
        return;
    end
    p = p + 6 + len;
end
error('no FC04 response with transaction ID %d', tidRead);
end


function printStats(label, x)
if isempty(x)
    fprintf('%s no data\n', label);
    return;
end
x = sort(x);
pct = @(p) x(max(1, ceil(p / 100 * numel(x))));
fprintf('%s median %6.2f  p95 %6.2f  p99 %6.2f  max %6.2f ms\n', ...
    label, pct(50), pct(95), pct(99), x(end));
end
