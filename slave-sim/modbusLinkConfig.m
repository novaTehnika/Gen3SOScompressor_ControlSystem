function cfg = modbusLinkConfig()
%MODBUSLINKCONFIG Timing of the Modbus TCP link between a client and the
%   controller, as measured on the Modbus bench (bench/modbus/README.md,
%   sections 5 and 7). modbusLink uses it.
%
%   The controller's Modbus server runs on the slave's task: it answers
%   one request per task tick on a connection, a written value reaches the
%   program at the next scan, and a read returns the registers as the last
%   finished scan left them. The last block of fields is what the Simulink
%   shell adds to that (the model commands of modbusLink); each is
%   switched off by the value given with it, which leaves the model
%   running on its nominal clock with every exchange completing, for
%   scan-exact checks.

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

% --- The Simulink model around the link (modbusLink 'model...') -------------
% The Modbus Client blocks of the shell make blocking calls inside the
% model step (Read, masterStep, Write, in that order, at the exchange
% period), so a call that outlasts the step delays the steps behind it,
% which then run back to back until the model has caught up with its
% clock. Bench: Modbus Client blocks at a 50 ms block sample time, 60 s
% runs, section 7.

% The step stalls for the blocking Read and Write calls. At a 25 ms step
% 0.21 % of the steps start at least one tick late (2 ticks at most); at 5
% ms 36.3 % do (17 ticks at most). False: the calls do not stall the
% model; steps stay on the nominal clock and an exchange is skipped while
% the previous one is in flight.
cfg.overrun = true;

% Exchanges that arrive a period late: the exchange due at a step is held
% back one exchange period and the one due then is merged with it, so the
% read results have a gap of two periods. Bench: one a minute at the
% 50 ms period (longest gap between read results 24 and 26 scans, 4 ms
% each). 0 switches it off.
cfg.lateExchangeRate = 1 / 60;  % per s of model time

% A lost link stops the model: the Read block raises an error on the first
% exchange after the connection drops (bench case D, cable unplugged for
% about 3 s; the blocks have no status output, retry or reconnect) and the
% simulation ends, which sets the DAQ outputs to their final values. The
% call fails this long after it was made; the bench reports the error on
% the exchange itself, with no measured delay. False: a failed call
% returns no data and the model goes on.
cfg.stopOnLostLink = true;
cfg.lostLinkErrorDelay = 0;     % s

% Seed of the link's own random stream (independent of rand/randn).
cfg.seed = 1;
end
