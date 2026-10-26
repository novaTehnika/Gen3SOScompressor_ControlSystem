function out = fakeProbeLink(cmd, varargin)
%FAKEPROBELINK Stand-in for pressureNoiseRun's Modbus link and clock: the
%   simulated slave behind the protocol registers (IR1-20, HR1-6) and an
%   emulation of PRG_PressureProbe behind the probe registers (HR101,
%   IR101-113, IR121-370). Time advances only in 'wait'.
%
%   fakeProbeLink('init')               fresh slave, axis 40 mm into the chamber
%   fakeProbeLink('faultAt', t)         drive fault injected at t seconds
%   fakeProbeLink('slave', fn)          sl = fn(sl): edit the simulated slave
%   fakeProbeLink('torqueModel', p)     report a friction torque (below)
%   t    = fakeProbeLink('now')
%          fakeProbeLink('wait', t)     run the slave until t
%   regs = fakeProbeLink('read', start, n)
%          fakeProbeLink('write', start, values)
%          fakeProbeLink('writeOne', address, value)
%   S    = fakeProbeLink('state')
%
%   The torque model stands in for the drive's torque feedback, which the
%   axis model reports as zero. With p a struct of F0, Fc, dFs, vs, b, mu,
%   noisePct and tStick, the reported torque (% rated) is the piston force
%     F0 + sign(v) (Fc + dFs exp(-(|v|/vs)^2) + mu P A) + b v + P A
%   in N divided by newtonPerPct, plus white noise, with v the axis
%   velocity (mm/s), P the chamber pressure (MPa) and A the piston area
%   (mm^2). A nonzero tStick holds the axis still for that long after each
%   command from rest while the torque ramps to F0 + sign (Fc + dFs) + P A,
%   which is what breakaway looks like in the log.

persistent S
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
            'faultReset', 0, 'reference', 0);
        [S.sl, S.y] = slaveStep(S.sl, S.u);
        S.tick = 0;
        S.faultAt = Inf;
        S.torque = [];
        S.stick = struct('until', -Inf, 'sign', 0, 'armed', true);
        S.noise = 0.001;        % V rms added to the sampled input
        S.p = struct('window', 0, 'state', 0, 'echo', 0, 'arm', 0, 'lastArm', 0, ...
            'idx', 0, 'samples', zeros(1, 250), 'sum', 0, 'sumSq', 0, ...
            'min', Inf, 'max', -Inf, 'n', 0, 'stats', zeros(1, 4), 'live', 0);
    case 'faultAt'
        S.faultAt = varargin{1};
    case 'slave'
        S.sl = varargin{1}(S.sl);
    case 'torqueModel'
        S.torque = varargin{1};
    case 'now'
        out = S.tick / 1000;
    case 'wait'
        while S.tick / 1000 < varargin{1}
            S.tick = S.tick + 1;
            if S.tick / 1000 >= S.faultAt
                S.sl.hook.driveFault = true;
            end
            if mod(S.tick, 2) == 0
                if ~isempty(S.torque)
                    S = torqueScan(S);
                end
                [S.sl, S.y] = slaveStep(S.sl, S.u);
                S = probeScan(S);
            end
        end
    case 'read'
        start = varargin{1};
        n = varargin{2};
        regs = zeros(1, 370);
        regs(1:S.map.nRead) = packRead(S.y, S.map);
        regs(101:102) = modbusPack(S.p.stats(1));
        regs(103:104) = modbusPack(S.p.stats(2));
        regs(105:106) = modbusPack(S.p.stats(3));
        regs(107:108) = modbusPack(S.p.stats(4));
        regs(109) = S.p.window;
        regs(110) = S.p.state;
        regs(111:112) = modbusPack(S.p.live);
        regs(113) = S.p.echo;
        regs(121:370) = S.p.samples;
        assert(start >= 1 && start + n - 1 <= 370, 'read outside the register groups');
        out = regs(start:start + n - 1);
    case 'write'
        w = varargin{2};
        assert(varargin{1} == 1 && numel(w) == S.map.nWrite, 'write is not HR1-6');
        S.u.heartbeat    = w(S.map.hr.heartbeat);
        S.u.modeCode     = w(S.map.hr.modeCode);
        S.u.motionEnable = w(S.map.hr.motionEnable);
        S.u.faultReset   = w(S.map.hr.faultReset);
        S.u.reference    = modbusUnpack(w(S.map.hr.reference), w(S.map.hr.reference + 1));
    case 'writeOne'
        assert(varargin{1} == 101, 'single write is not HR101');
        S.p.arm = varargin{2};
    case 'state'
        out = S;
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
