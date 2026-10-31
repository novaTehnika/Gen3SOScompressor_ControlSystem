function cfg = modbusBenchConfig()
%MODBUSBENCHCONFIG Constants for the Modbus TCP bench test.
%   Register numbers are 1-based, as used by the MATLAB modbus object and
%   the Simulink Modbus Client blocks. They must match the addresses in
%   slave/PRG_ModbusBench.st.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'), fullfile(here, '..', 'common'));
master = masterConfig();
shared = benchConfig();

% Connection (bench/common/benchConfig.m)
cfg.address   = shared.address;   % controller IP: the address MotionWorks connects to
cfg.port      = shared.port;
cfg.serverId  = shared.serverId;
cfg.timeout   = 0.020;            % s, per transaction in the bench model; modbusProbe reports the share of cycles slower than this
cfg.wordOrder = 'little-endian';  % low word first, as measured by modbusProbe

% Timing
cfg.dt        = master.dt;        % model step (s), same as the shell
cfg.duration  = 60;               % s, length of a Simulink bench run
cfg.skip      = 20;               % steps discarded at the start (connection set-up)

% Registers
cfg.hrHeartbeat     = 1;          % HR1      G_mbHeartbeat
cfg.irHeartbeatEcho = 1;          % IR1      G_mbHeartbeatEcho, IR2 G_mbSlaveCounter
cfg.hrProbe         = 101;        % HR101-102 G_mbProbeDintIn
cfg.irProbe         = 101;        % IR101-108 probe block

% Slave
cfg.slaveScan = shared.slaveScan; % s, cyclic task interval
end
