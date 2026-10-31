function link = benchLink(address, port, serverId)
%BENCHLINK Raw Modbus TCP link over tcpclient for the bench runs: FC04 read,
%   FC06 single write, FC16 write. The controller answers in about 4 ms;
%   the Industrial Communication Toolbox modbus object adds some 7 ms per
%   transaction, which the 25 ms gaps between master steps cannot spare.
%
%   link = benchLink()                      address, port and unit from benchConfig
%   link = benchLink(address, port, serverId)
%
%   link.read(start, n)        registers start..start+n-1 (1-based) as doubles
%   link.write(start, vals)    FC16 from register start
%   link.writeOne(addr, val)   FC06
%   link.close()

if nargin < 3
    bcfg = benchConfig();
    address = bcfg.address;
    port = bcfg.port;
    serverId = bcfg.serverId;
end
% Without a transfer delay each request leaves at once; with Nagle's
% algorithm on, a request can wait for the acknowledgement of the one
% before it, which on this link showed as tens of milliseconds a
% transaction.
t = tcpclient(address, port, 'Timeout', 0.5, 'EnableTransferDelay', false);
uid = uint8(serverId);
link.t = t;
link.read = @(start, n) rawTxn(t, uid, [4, u16(start - 1), u16(n)], 4, n);
link.write = @(start, vals) rawTxn(t, uid, ...
    [16, u16(start - 1), u16(numel(vals)), 2 * numel(vals), u16s(vals)], 16, 0);
link.writeOne = @(addr, val) rawTxn(t, uid, [6, u16(addr - 1), u16(val)], 6, 0);
link.close = @() delete(t);
end


function regs = rawTxn(t, uid, pdu, fc, nRegs)
% Send one request and parse its response. Returns the registers of an
% FC04 read, or [] for a write.
persistent tid
if isempty(tid)
    tid = 0;
end
tid = mod(tid + 1, 65536);
flush(t);
pdu = uint8(pdu);
write(t, [u16(tid), uint8([0, 0]), u16(numel(pdu) + 1), uid, pdu]);
hdr = double(read(t, 7, 'uint8'));
if numel(hdr) < 7
    error('no response (%d of 7 header bytes)', numel(hdr));
end
len = hdr(5) * 256 + hdr(6);
body = double(read(t, len - 1, 'uint8'));
if numel(body) < len - 1
    error('short response (%d of %d bytes)', numel(body), len - 1);
end
if hdr(1) * 256 + hdr(2) ~= tid
    error('transaction ID %d, expected %d', hdr(1) * 256 + hdr(2), tid);
end
if body(1) >= 128
    error('exception response: function %d, code %d', body(1) - 128, body(2));
end
if body(1) ~= fc
    error('function %d in response, expected %d', body(1), fc);
end
if fc == 4
    if body(2) < 2 * nRegs
        error('%d bytes of registers, expected %d', body(2), 2 * nRegs);
    end
    regs = body(3:2:2 + 2 * nRegs) * 256 + body(4:2:3 + 2 * nRegs);
else
    regs = [];
end
end


function b = u16(v)
% Big-endian 16-bit value as two bytes.
v = mod(round(v), 65536);
b = uint8([floor(v / 256), mod(v, 256)]);
end


function b = u16s(vals)
b = uint8(zeros(1, 2 * numel(vals)));
for k = 1:numel(vals)
    b(2 * k - 1:2 * k) = u16(vals(k));
end
end
