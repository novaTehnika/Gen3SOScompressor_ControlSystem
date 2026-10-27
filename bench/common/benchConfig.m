function cfg = benchConfig()
%BENCHCONFIG Settings shared by every bench: the controller connection, the
%   slave scan, where runs are logged, the safety-hold model and the step
%   probe's registers. Each bench keeps its own constants in its own
%   config (modbusBenchConfig, frictionBenchConfig, charConfig) and takes
%   these from here.

here = fileparts(mfilename('fullpath'));

% Connection: the MP2600iec's Modbus TCP server, the address MotionWorks
% connects to.
cfg.address   = '192.168.1.1';
cfg.port      = 502;
cfg.serverId  = 1;
cfg.slaveScan = 0.002;            % s, cyclic task interval (G_cfgScanTime)

% Where the run scripts write their logs and captures (gitignored), and
% the Desktop Real-Time model that holds the safety relay while a script
% takes the shell's place as Modbus master (buildSafetyHold).
cfg.captures    = fullfile(here, 'captures');
cfg.safetyModel = 'SafetyHold';
cfg.safetyDir   = here;

% Scan-rate capture by PRG_StepProbe (slave/PRG_StepProbe.st): registers
% (1-based), length and scales. The five arrays are contiguous from irFirst:
% three INT, one DINT and one INT, 6 * nScan registers in all.
cfg.probe.hrArm     = 111;      % HR111 G_mbStepArm
cfg.probe.irState   = 401;      % IR401 state, 402 arm echo, 403 count, 404-405 Pos0 (REAL)
cfg.probe.irFirst   = 411;      % IR411-910 vCmd, 911-1410 vel, 1411-1910 torque, 1911-2910 position, 2911-3410 pressure
cfg.probe.nScan     = 500;      % scans per capture
cfg.probe.velScale  = 1000;     % counts per mm/s
cfg.probe.trqScale  = 100;      % counts per % rated
cfg.probe.posScale  = 1e5;      % counts per mm
cfg.probe.presScale = 1000;     % counts per MPa
cfg.probe.blockRegs = 125;      % registers per FC04 read, one read per exchange gap
cfg.probe.timeout   = 4;        % s from arm to ready before the capture is given up
end
