function [echo, slaveCounter, txnMs, stepMs, ok] = modbusBenchStep(seq)
%MODBUSBENCHSTEP One bench-model step: write the heartbeat seq to HR1, then
%   read the echo and slave scan counter from IR1-2. Called extrinsically
%   from the ModbusBench MATLAB Function block, so it runs as interpreted
%   MATLAB on the host. The connection is opened on the first call and
%   closed by the model's StopFcn (clear modbusBenchStep).
%
%   echo          IR1, last heartbeat the slave saw (-1 on failure)
%   slaveCounter  IR2, slave scan counter (-1 on failure)
%   txnMs         time spent in the write and read (ms)
%   stepMs        host time since the previous call (ms)
%   ok            1 when both transactions succeeded

persistent m cfg tStep

if isempty(m)
    cfg = modbusBenchConfig();
    m = modbus('tcpip', cfg.address, cfg.port);
    m.Timeout = cfg.timeout;
    tStep = tic;
end

stepMs = toc(tStep) * 1000;
tStep = tic;

t0 = tic;
try
    write(m, 'holdingregs', cfg.hrHeartbeat, seq, cfg.serverId, 'uint16');
    r = read(m, 'inputregs', cfg.irHeartbeatEcho, 2, cfg.serverId, 'uint16');
    echo = r(1);
    slaveCounter = r(2);
    ok = 1;
catch
    echo = -1;
    slaveCounter = -1;
    ok = 0;
end
txnMs = toc(t0) * 1000;
end
