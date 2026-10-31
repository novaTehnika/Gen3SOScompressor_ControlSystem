function cfg = modbusLinkConfig()
%MODBUSLINKCONFIG Timing of the Modbus TCP link between a client and the
%   controller, as measured on the Modbus bench (bench/modbus/README.md,
%   section 5, case A captures). modbusLink uses it.
%
%   The controller's Modbus server runs on the slave's task: it answers
%   one request per task tick on a connection, a written value reaches the
%   program at the next scan, and a read returns the registers as the last
%   finished scan left them.

% Requests are served on the slave's task tick, G_cfgScanTime (4 ms): the
% request -> TCP ACK delay is spread evenly over 0.5 to 4 ms.
cfg.tick = [];                  % s; empty takes the slave's scan time

% Share of responses that leave one tick after the request was served
% rather than in the same tick: 81 % in the same tick.
cfg.pNextTick = 0.19;

% Controller time per register in a request, on top of the tick. Not yet
% measured: bench/modbus/modbusCountSweep.m measures it.
cfg.perRegister = 0;            % s per register

% Largest request (Modbus application protocol): 125 registers in a read
% (FC04), 123 in a write (FC16). A longer transfer is split into requests,
% each served in its own tick.
cfg.maxRead = 125;
cfg.maxWrite = 123;

% Client time from a response to its next request (or to the blocking call
% returning): median 6.9 ms, p99 17.2 ms with MATLAB's modbus object and
% tcpclient; drawn from the lognormal with those quantiles.
cfg.clientMedian = 6.9e-3;      % s
cfg.clientP99 = 17.2e-3;        % s

% A client sends a request only once the previous one is answered (the
% Simulink Modbus Client blocks, the modbus object, benchLink). True sends
% a transfer's requests at once, as modbusRawProbe's pipelined variants do.
cfg.pipelined = false;

% Seed of the link's own random stream (independent of rand/randn).
cfg.seed = 1;
end
