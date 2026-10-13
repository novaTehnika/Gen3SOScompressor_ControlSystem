function result = modbusProbe(nCycles)
%MODBUSPROBE Check the Modbus register mapping and word order against
%   PRG_ModbusBench, then time heartbeat write/read transactions.
%   Runs from the MATLAB command line without Simulink.
%
%   result = modbusProbe(nCycles)   nCycles timed transactions (default 2000)

if nargin < 1
    nCycles = 2000;
end
cfg = modbusBenchConfig();

m = modbus('tcpip', cfg.address, cfg.port);
m.Timeout = 1;    % the connection closes when m goes out of scope

% --- Slave -> master: fixed patterns, read as raw 16-bit registers ---------
raw = read(m, 'inputregs', cfg.irProbe, 8, cfg.serverId, 'uint16');
dintRaw = raw(1:2);
realRaw = raw(3:4);
uintRaw = raw(8);

fprintf('\nSlave -> master (IR%d-%d)\n', cfg.irProbe, cfg.irProbe + 7);
fprintf('  UINT 16#1234      read %s\n', hexWords(uintRaw));
fprintf('  DINT 16#12345678  read %s\n', hexWords(dintRaw));
fprintf('  REAL 1234.5       read %s (16#449A 16#5000 expected in some order)\n', ...
    hexWords(realRaw));

result.byteOrderOk = uintRaw == hex2dec('1234');
result.slaveWordOrder = wordOrderOf(dintRaw, [hex2dec('1234') hex2dec('5678')]);
realOrder = wordOrderOf(realRaw, [hex2dec('449A') hex2dec('5000')]);
fprintf('  byte order within a register: %s\n', okText(result.byteOrderOk));
fprintf('  word order (DINT): %s, (REAL): %s\n', result.slaveWordOrder, realOrder);

% --- Master -> slave: write a DINT pattern as two raw registers ----------
write(m, 'holdingregs', cfg.hrProbe, [hex2dec('1234') hex2dec('5678')], ...
    cfg.serverId, 'uint16');
pause(0.05);
raw = read(m, 'inputregs', cfg.irProbe + 4, 3, cfg.serverId, 'uint16');
codes = {'other (check address mapping)', 'high word first', 'low word first'};
result.masterWordOrderCode = raw(3);
fprintf('\nMaster -> slave (HR%d-%d)\n', cfg.hrProbe, cfg.hrProbe + 1);
fprintf('  wrote 16#1234 16#5678, slave echoes %s\n', hexWords(raw(1:2)));
fprintf('  slave decoded the DINT as: %s\n', codes{min(raw(3), 2) + 1});

% --- Heartbeat round trips ------------------------------------------------
% A long timeout here so that a slow server shows up as a large transaction
% time rather than as a failure. On a TCP timeout the client closes and
% reopens the connection, which would slow every later cycle as well.
m.Timeout = 1;
tWrite = nan(1, nCycles);
tTotal = nan(1, nCycles);
lag = nan(1, nCycles);
counter = nan(1, nCycles);
wall = nan(1, nCycles);
failures = 0;
firstError = '';
t0 = tic;
for k = 1:nCycles
    seq = mod(k, 65536);
    tk = tic;
    try
        write(m, 'holdingregs', cfg.hrHeartbeat, seq, cfg.serverId, 'uint16');
        tWrite(k) = toc(tk);
        r = read(m, 'inputregs', cfg.irHeartbeatEcho, 2, cfg.serverId, 'uint16');
        tTotal(k) = toc(tk);
        lag(k) = mod(seq - r(1), 65536);
        counter(k) = r(2);
    catch err
        failures = failures + 1;
        if isempty(firstError)
            firstError = err.message;
        end
    end
    wall(k) = toc(t0);
end

ms = 1000;
ok = ~isnan(tTotal);
fprintf('\nHeartbeat round trips: %d cycles, %d failed (timeout %.0f ms)\n', ...
    nCycles, failures, m.Timeout * ms);
if failures > 0
    fprintf('  first failure: %s\n', firstError);
end
printStats('  write             ', tWrite(ok) * ms);
printStats('  read              ', (tTotal(ok) - tWrite(ok)) * ms);
printStats('  write + read      ', tTotal(ok) * ms);
fprintf('  bench timeout %.0f ms exceeded in %.2f %% of cycles\n', ...
    cfg.timeout * ms, 100 * mean(tTotal(ok) > cfg.timeout));
if any(ok)
    fprintf('  echo matched in the same cycle: %.2f %%\n', 100 * mean(lag(ok) == 0));
end

% Slave scan period from its counter against PC wall time.
idx = find(ok);
if numel(idx) >= 2
    scans = mod(counter(idx(end)) - counter(idx(1)), 65536);
    elapsed = wall(idx(end)) - wall(idx(1));
    if scans > 0
        fprintf('  slave scan period: %.2f ms (expected %.1f ms; counter wraps every %.0f s)\n', ...
            elapsed / scans * ms, cfg.slaveScan * ms, 65536 * cfg.slaveScan);
    end
end

result.failures = failures;
result.firstError = firstError;
result.totalMs = tTotal * ms;
result.writeMs = tWrite * ms;
result.lag = lag;
end


function order = wordOrderOf(raw, expectedHighFirst)
if isequal(raw(:)', expectedHighFirst)
    order = 'big-endian (high word first)';
elseif isequal(raw(:)', fliplr(expectedHighFirst))
    order = 'little-endian (low word first)';
else
    order = 'unexpected - check address mapping';
end
end


function s = hexWords(words)
s = strjoin(arrayfun(@(w) sprintf('16#%04X', w), words, 'UniformOutput', false), ' ');
end


function s = okText(flag)
if flag
    s = 'ok';
else
    s = 'SWAPPED or wrong address';
end
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
