function out = fakeProbeLink(cmd, varargin)
%FAKEPROBELINK Stand-in for the bench runs' Modbus link and clock: the
%   simulated slave behind the protocol registers (IR1-20, HR1-8), an
%   emulation of PRG_PressureProbe behind the noise registers (HR101,
%   IR101-113, IR121-370) and, behind the step probe registers (HR111-114,
%   IR401-524), the slave's own FB_Probe: the registers are read from and
%   written to the generated slave's globals (G_mbProbe*), so the ring,
%   the arm latch, the recording and transfer state machines, the window
%   and the fault mark are the slave source's. Only the link is emulated:
%   the hooks below lose an arm write or tear a read. The test keeps a log
%   of what the probe was given each scan, converted to counts here and
%   not by the slave, to check the ring's records against.
%   Time advances only in 'wait'.
%
%   fakeProbeLink('init')               fresh slave, axis 40 mm into the chamber
%   fakeProbeLink('faultAt', t)         drive fault injected at t seconds
%   fakeProbeLink('faultAt', t, until)  the fault clears again at until
%   fakeProbeLink('slave', fn)          sl = fn(sl): edit the simulated slave
%   fakeProbeLink('dropArm', n)         lose the next n writes that include HR111
%   fakeProbeLink('tearRead', n)        the next n reads that include IR519-520
%                                       find the window changed under them
%   fakeProbeLink('ringDepth', n)       use a ring of n records: sets the probe
%                                       block's constant cDepth in the simulated
%                                       instance (the ST keeps 8192, the array
%                                       its full size), so overrun is reached in
%                                       a few hundred scans
%   fakeProbeLink('torqueModel', p)     report a friction torque (below)
%   fakeProbeLink('valveAt', t, Cd)     outlet valve opens at t seconds: the
%                                       chamber drains through the orifice
%                                       bank, Q = Cd A_o sqrt(2P/rho), after
%                                       losing the tube's 0.236 mL at once
%   t    = fakeProbeLink('now')
%          fakeProbeLink('wait', t)     run the slave until t
%   regs = fakeProbeLink('read', start, n)
%          fakeProbeLink('write', start, values)
%          fakeProbeLink('writeOne', address, value)
%   S    = fakeProbeLink('state')       S.collectLog is [t, mm^3 that left]
%                                       while the valve is open; S.ring has the
%                                       log of records, one row per record index
%                                       from 0: [vCmd vel torque pos pressure
%                                       raw status] in counts; S.sp the probe's
%                                       written, depth, arm, echo, mark, state,
%                                       xfer, faultMark and the hooks' counts
%
%   The torque model stands in for the drive's torque feedback, which the
%   axis model reports as zero. With p a struct of F0, Fc, dFs, vs, b, mu,
%   noisePct and tStick, the reported torque (% rated) is the piston force
%     F0 + sign(v) (Fc + dFs exp(-(|v|/vs)^2) + mu P A) + b v + P A
%   in N divided by newtonPerPct, plus white noise, with v the axis
%   velocity (mm/s), P the chamber pressure (MPa) and A the piston area
%   (mm^2). A nonzero tStick holds the axis still for that long after each
%   command from rest while the torque ramps to F0 + sign (Fc + dFs) + P A,
%   which is what breakaway looks like in the log. A field tauV lags the
%   velocity the registers report behind the axis by that time constant,
%   standing in for the drive's velocity loop, which the axis model does
%   not have.

persistent S R
out = [];
switch cmd
    case 'init'
        S.map = modbusMap();
        S.cfg = masterConfig();
        S.sl = slaveInit();
        S.sl.G.G_cfgPosSoftLimitMax = S.cfg.posMax + 5;
        S.sl.sim.switchPosPos = S.cfg.posMax + 10;
        S.sl.ax.x = S.sl.sim.chamberContactPos + 40;
        S.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
            'faultReset', 0, 'reference', 0, 'positionSpeed', 0);
        [S.sl, S.y] = slaveStep(S.sl, S.u);
        S.tick = 0;
        S.faultAt = Inf;
        S.faultUntil = Inf;
        S.torque = [];
        S.valve = struct('at', Inf, 'Cd', 0.7, 'open', false, 'collected', 0);
        S.collectLog = zeros(0, 2);
        S.stick = struct('until', -Inf, 'sign', 0, 'armed', true);
        S.vLag = 0;
        S.hook = struct('dropArm', 0, 'tear', 0);
        S.nT = 0;
        R = zeros(4096, 7);
        S.noise = 0.001;        % V rms added to the sampled input
        S.p = struct('window', 0, 'state', 0, 'echo', 0, 'arm', 0, 'lastArm', 0, ...
            'idx', 0, 'samples', zeros(1, 250), 'sum', 0, 'sumSq', 0, ...
            'min', Inf, 'max', -Inf, 'n', 0, 'stats', zeros(1, 4), 'live', 0);
    case 'faultAt'
        S.faultAt = varargin{1};
        S.faultUntil = Inf;
        if numel(varargin) > 1
            S.faultUntil = varargin{2};
        end
    case 'slave'
        S.sl = varargin{1}(S.sl);
    case 'torqueModel'
        S.torque = varargin{1};
    case 'valveAt'
        S.valve.at = varargin{1};
        if numel(varargin) > 1
            S.valve.Cd = varargin{2};
        end
    case 'dropArm'
        S.hook.dropArm = varargin{1};
    case 'tearRead'
        S.hook.tear = varargin{1};
    case 'ringDepth'
        S.sl.s.fbProbe.cDepth = varargin{1};
    case 'now'
        out = S.tick / 1000;
    case 'wait'
        while S.tick / 1000 < varargin{1}
            S.tick = S.tick + 1;
            if S.tick / 1000 >= S.faultAt && S.tick / 1000 < S.faultUntil
                S.sl.hook.driveFault = true;
            elseif S.tick / 1000 >= S.faultUntil
                S.sl.hook.driveFault = false;
            end
            if mod(S.tick, 2) == 0
                if ~isempty(S.torque)
                    S = torqueScan(S);
                end
                S = valveScan(S);
                Gpre = S.sl.G;                      % what the probe is given
                [S.sl, S.y] = slaveStep(S.sl, S.u);
                S = lagScan(S);
                S = probeScan(S);
                [S, R] = logScan(S, R, Gpre);
            end
        end
    case 'read'
        start = varargin{1};
        n = varargin{2};
        regs = zeros(1, 524);
        regs(1:S.map.nRead) = packRead(S.y, S.map);
        if ~isempty(S.torque)
            regs(S.map.ir.velocity:S.map.ir.velocity + 1) = modbusPack(S.vLag);
        end
        regs(101:102) = modbusPack(S.p.stats(1));
        regs(103:104) = modbusPack(S.p.stats(2));
        regs(105:106) = modbusPack(S.p.stats(3));
        regs(107:108) = modbusPack(S.p.stats(4));
        regs(109) = S.p.window;
        regs(110) = S.p.state;
        regs(111:112) = modbusPack(S.p.live);
        regs(113) = S.p.echo;
        regs(121:370) = S.p.samples;
        G = S.sl.G;
        regs(401:402) = u32Words(G.G_mbProbeWritten);
        regs(403:404) = u32Words(G.G_mbProbeMark);
        regs(405:406) = u32Words(G.G_mbProbeStart);
        regs(407) = G.G_mbProbeArmEcho;
        regs(408) = G.G_mbProbeCount;
        regs(409) = G.G_mbProbeDepth;
        regs(410) = G.G_mbProbeWords;
        regs(411:514) = G.G_mbProbeWindow;
        regs(519:520) = u32Words(G.G_mbProbeStartTail);
        regs(521) = G.G_mbProbeState;
        regs(522) = G.G_mbProbeXfer;
        regs(523:524) = u32Words(G.G_mbProbeFaultMark);
        assert(start >= 1 && start + n - 1 <= 524, 'read outside the register groups');
        if start <= 519 && start + n - 1 >= 520 && S.hook.tear > 0
            regs(519:520) = u32Words(G.G_mbProbeStartTail + 1);     % a window that changed under the read
            S.hook.tear = S.hook.tear - 1;
        end
        out = regs(start:start + n - 1);
    case 'write'
        w = varargin{2};
        first = varargin{1};
        if first >= 111 && first <= 114
            assert(first + numel(w) - 1 <= 114, 'write past HR114');
            if first == 111 && S.hook.dropArm > 0
                S.hook.dropArm = S.hook.dropArm - 1;
                return;
            end
            for k = 1:numel(w)
                S = probeRegister(S, first + k - 1, w(k));
            end
            return;
        end
        assert(first == 1 && numel(w) == S.map.nWrite, 'write is not HR1-8 or HR111-114');
        S.u.heartbeat    = w(S.map.hr.heartbeat);
        S.u.modeCode     = w(S.map.hr.modeCode);
        S.u.motionEnable = w(S.map.hr.motionEnable);
        S.u.faultReset   = w(S.map.hr.faultReset);
        S.u.reference    = modbusUnpack(w(S.map.hr.reference), w(S.map.hr.reference + 1));
        S.u.positionSpeed = modbusUnpack(w(S.map.hr.positionSpeed), w(S.map.hr.positionSpeed + 1));
    case 'writeOne'
        switch varargin{1}
            case 101
                S.p.arm = varargin{2};
            case 111
                if S.hook.dropArm > 0
                    S.hook.dropArm = S.hook.dropArm - 1;
                else
                    S = probeRegister(S, 111, varargin{2});
                end
            case {112, 113, 114}
                S = probeRegister(S, varargin{1}, varargin{2});
            otherwise
                error('single write is not HR101 or HR111-114');
        end
    case 'state'
        out = S;
        out.ring = R(1:S.nT, :);        % the log of records, one row per index from 0, counts
        G = S.sl.G;
        out.sp = struct('written', G.G_mbProbeWritten, 'depth', G.G_mbProbeDepth, ...
            'arm', G.G_mbProbeArm, 'echo', G.G_mbProbeArmEcho, 'mark', G.G_mbProbeMark, ...
            'state', G.G_mbProbeState, 'xfer', G.G_mbProbeXfer, ...
            'faultMark', G.G_mbProbeFaultMark, 'dropArm', S.hook.dropArm, 'tear', S.hook.tear);
end
end


function S = torqueScan(S)
% Torque feedback from the friction model, and the stick after a command
% from rest.
p = S.torque;
sl = S.sl;
t = S.tick / 1000;
cmd = sl.G.G_cmdDirectControl;
commanded = cmd.Enable && cmd.ControlMode == sl.E.Y_ControlMode.VelocityTLMode && cmd.Velocity ~= 0;
if ~commanded
    S.stick.armed = true;
end
if commanded && S.stick.armed && abs(sl.ax.v) < 0.05 && p.tStick > 0
    S.stick.armed = false;
    S.stick.until = t + p.tStick;
    S.stick.sign = sign(cmd.Velocity);
end
v = sl.ax.v;
PA = sl.ax.P * sl.sim.pistonArea;               % MPa * mm^2 = N
if t < S.stick.until
    sl.hook.blocked = true;
    frac = 1 - (S.stick.until - t) / p.tStick;
    F = PA + p.F0 + S.stick.sign * frac * (p.Fc + p.dFs);
else
    sl.hook.blocked = false;
    s = sign(v);
    F = PA + p.F0 + s * (p.Fc + p.dFs * exp(-(abs(v) / p.vs)^2) + p.mu * PA) + p.b * v;
end
sl.hook.torque = F / p.newtonPerPct + p.noisePct * randn();
S.sl = sl;
end


function S = valveScan(S)
% Orifice outflow once the valve is open: 32 orifices of 40 um discharging
% to atmosphere, after the air in the 1 mm bore tube has left.
if isinf(S.valve.at)
    return;
end
t = S.tick / 1000;
if ~S.valve.open && t >= S.valve.at
    S.valve.open = true;
    S.sl.ax.vOut = S.sl.ax.vOut + 236;                        % mm^3, the tube
end
if S.valve.open
    Ao = 32 * pi / 4 * (40e-6)^2;                             % m^2
    Q = S.valve.Cd * Ao * sqrt(2 * max(S.sl.ax.P, 0) * 1e6 / 1000) * 1e9;    % mm^3/s
    S.sl.hook.outflow = Q;
    S.valve.collected = S.valve.collected + Q * S.sl.G.G_cfgScanTime;
    S.collectLog(end + 1, :) = [t S.valve.collected];
end
end


function S = lagScan(S)
% The velocity the registers report: the axis velocity through a
% first-order lag when the torque model asks for one.
dt = S.sl.G.G_cfgScanTime;
if ~isempty(S.torque) && isfield(S.torque, 'tauV') && S.torque.tauV > 0
    S.vLag = S.vLag + (S.sl.ax.v - S.vLag) * dt / (S.torque.tauV + dt);
else
    S.vLag = S.sl.ax.v;
end
S.sl.G.G_sysActualVelocity = S.vLag;            % the feedback the next scan sees
end


function S = probeRegister(S, hr, value)
% A write to one of the step probe's holding registers (HR111-114).
switch hr
    case 111
        S.sl.G.G_mbProbeArm = value;
    case 112
        S.sl.G.G_mbProbeReqLo = value;
    case 113
        S.sl.G.G_mbProbeReqHi = value;
    case 114
        S.sl.G.G_mbProbeLength = value;
end
end


function [S, R] = logScan(S, R, Gpre)
% Log the record the probe wrote in the scan just run, as counts: the
% values it was given (the axis feedback as it stood before the scan, the
% rest as the scan left them) converted here, independently of the slave.
w = S.sl.s.fbProbe.Written;
if w <= S.nT
    return;
end
G = S.sl.G;
lim = @(x, lo, hi) min(max(round(x), lo), hi);
status = G.G_doInMotion + 2 * G.G_doPerformanceStatus + 4 * G.G_doFaultActive ...
    + 8 * G.G_doHomingComplete + 256 * min(G.G_sysConfirmedModeCode, 255);
row = [lim(G.G_sysCommandedVelocity * 1000, -32767, 32767), ...
       lim(Gpre.G_sysActualVelocity * 1000, -32767, 32767), ...
       lim(Gpre.G_sysActualTorque * 100, -32767, 32767), ...
       round(Gpre.G_sysActualPosition * 1e5), ...
       lim(G.G_sysActualPressure * 1000, -32767, 32767), ...
       lim(G.G_aiPressure * 5000, 0, 65535), double(status)];
while w > size(R, 1)
    R(end + size(R, 1), 1) = 0;                 % twice the rows
end
R(w, :) = row;
S.nT = w;
end


function w = u32Words(x)
% An unsigned 32-bit value as two registers, low word first.
w = [mod(x, 65536), floor(x / 65536)];
end


function w = u16s(x)
% Signed integers as 16-bit register values.
w = mod(x, 65536);
end


function S = probeScan(S)
% One scan of PRG_PressureProbe on the slave's analog input.
p = S.p;
v = S.sl.IO.MO1_AI_01 + S.noise * randn();
p.live = v;
p.sum = p.sum + v;
p.sumSq = p.sumSq + v * v;
p.min = min(p.min, v);
p.max = max(p.max, v);
p.n = p.n + 1;
if p.n >= 500
    m = p.sum / p.n;
    p.stats = [m sqrt(max(p.sumSq / p.n - m * m, 0)) p.min p.max];
    p.window = mod(p.window + 1, 65536);
    p.sum = 0; p.sumSq = 0; p.min = Inf; p.max = -Inf; p.n = 0;
end
if p.arm ~= p.lastArm && p.state ~= 1
    p.lastArm = p.arm;
    p.idx = 0;
    p.state = 1;
end
if p.state == 1
    p.idx = p.idx + 1;
    p.samples(p.idx) = min(max(round(v * 5000), 0), 65535);
    if p.idx >= 250
        p.echo = p.lastArm;
        p.state = 2;
    end
end
S.p = p;
end


function regs = packRead(y, map)
regs = zeros(1, map.nRead);
regs(map.ir.heartbeatEcho)     = y.heartbeatEcho;
regs(map.ir.slaveCounter)      = y.slaveCounter;
regs(map.ir.confCode)          = y.confCode;
regs(map.ir.faultCode)         = y.faultCode;
regs(map.ir.faultActive)       = y.faultActive;
regs(map.ir.inMotion)          = y.inMotion;
regs(map.ir.performanceStatus) = y.performanceStatus;
regs(map.ir.homingComplete)    = y.homingComplete;
regs(map.ir.position:map.ir.position + 1)   = modbusPack(y.actualPosition);
regs(map.ir.velocity:map.ir.velocity + 1)   = modbusPack(y.actualVelocity);
regs(map.ir.torque:map.ir.torque + 1)       = modbusPack(y.actualTorque);
regs(map.ir.commandedPosition:map.ir.commandedPosition + 1) = ...
    modbusPack(y.commandedPosition);
regs(map.ir.pressure:map.ir.pressure + 1) = modbusPack(y.actualPressure);
end
