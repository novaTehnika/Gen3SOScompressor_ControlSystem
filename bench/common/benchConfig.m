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
cfg.slaveScan = 0.004;            % s, cyclic task interval (G_cfgScanTime)

% Where the run scripts write their logs and captures (gitignored), and
% the Desktop Real-Time model that holds the safety relay while a script
% takes the shell's place as Modbus master (buildSafetyHold).
cfg.captures    = fullfile(here, 'captures');
cfg.safetyModel = 'SafetyHold';
cfg.safetyDir   = here;

% Scan-rate capture by PRG_StepProbe (slave/PRG_StepProbe.st), read by
% benchProbe: registers (1-based), scales and timing. The slave keeps a ring
% of records; the master asks for a start index (HR112-113) and reads the
% header and an 18-record window (IR401-520) in one FC04 read.
cfg.probe.hrArm     = 111;      % HR111 G_mbStepArm
cfg.probe.hrReq     = 112;      % HR112-113 requested start index of the window, low word first
cfg.probe.irFirst   = 401;      % IR401-520: Written, Mark, Start (2 each), ArmEcho, Count, Depth, Words, window, StartTail
cfg.probe.nRegs     = 120;      % registers of that read
cfg.probe.nWindow   = 18;       % records the window holds
cfg.probe.nWords    = 6;        % words per record: vCmd vel torque posLo posHi pressure
cfg.probe.nScan     = 500;      % records per capture by default (1 s at the 2 ms scan)
cfg.probe.nPre      = 0;        % records taken before the arm's mark
cfg.probe.velScale  = 1000;     % counts per mm/s
cfg.probe.trqScale  = 100;      % counts per % rated
cfg.probe.posScale  = 1e5;      % counts per mm
cfg.probe.presScale = 1000;     % counts per MPa
cfg.probe.armRetry  = 0.3;      % s before an unacknowledged arm is written again
cfg.probe.timeout   = 4;        % s without an acknowledgement (or, when draining, without a new record) before a capture is given up
cfg.probe.drainGap  = 0.02;     % s between transactions while draining
end
