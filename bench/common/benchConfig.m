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

% Scan-rate capture by the slave's step probe (slave-src/FB/FB_Probe.st),
% read by benchProbe: registers (1-based), scales and timing. The slave keeps
% a ring of records; the master asks for a start index (HR112-113) and reads
% the header, a 13-record window and the state registers (IR401-524) in one
% FC04 read. An arm is written with the current request and the capture
% length in one FC16 write of HR111-114.
cfg.probe.hrArm     = 111;      % HR111 G_mbProbeArm, then HR112-113 the request and HR114 the length
cfg.probe.hrReq     = 112;      % HR112-113 requested start index of the window, low word first
cfg.probe.hrLength  = 114;      % HR114 G_mbProbeLength: records the arm wants, 0 = open ended
cfg.probe.irFirst   = 401;      % first register of the read
cfg.probe.nRegs     = 124;      % registers of that read: IR401-524
cfg.probe.off.written   = 1;    % offsets in that read (1-based): Written, Mark, Start (2 words each)
cfg.probe.off.mark      = 3;
cfg.probe.off.start     = 5;
cfg.probe.off.echo      = 7;    % ArmEcho
cfg.probe.off.count     = 8;    % records in the window
cfg.probe.off.depth     = 9;    % ring depth in records
cfg.probe.off.words     = 10;   % words per record
cfg.probe.off.window    = 11;   % the window, nWindowWords registers
cfg.probe.off.tail      = 119;  % StartTail (2 words), IR519-520
cfg.probe.off.state     = 121;  % E_ProbeState code, IR521
cfg.probe.off.xfer      = 122;  % E_ProbeXferState code, IR522
cfg.probe.off.faultMark = 123;  % FaultMark (2 words), IR523-524
cfg.probe.nWindowWords = 104;   % registers of the window (13 records of 8 words)
cfg.probe.nWords    = 8;        % words per record the decoder knows: vCmd vel torque posLo posHi pressure raw status
cfg.probe.nScan     = 500;      % records per capture by default (1 s at the 2 ms scan)
cfg.probe.nPre      = 0;        % records taken before the arm's mark
cfg.probe.velScale  = 1000;     % counts per mm/s
cfg.probe.trqScale  = 100;      % counts per % rated
cfg.probe.posScale  = 1e5;      % counts per mm
cfg.probe.presScale = 1000;     % counts per MPa
cfg.probe.voltScale = 5000;     % counts per V of the raw pressure input (0.2 mV per count)
cfg.probe.armRetry  = 0.3;      % s before an unacknowledged arm is written again
cfg.probe.timeout   = 4;        % s without an acknowledgement (or, when draining, without a new record) before a capture is given up
cfg.probe.drainGap  = 0.02;     % s between transactions while draining
end
