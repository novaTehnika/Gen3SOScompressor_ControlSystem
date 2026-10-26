function out = fakeAxisLink(cmd, varargin)
%FAKEAXISLINK Stand-in for charRun's Modbus link and clock: the simulated
%   slave behind the protocol registers (IR1-20, HR1-6), an emulation of
%   PRG_AxisProbe behind the probe registers (HR101, IR101-104, IR121-1120),
%   and an outlet valve that drains the simulated chamber through the
%   orifice bank. Time advances only in 'wait'.
%
%   fakeAxisLink('init', x0)            fresh slave with the axis at x0 mm
%   fakeAxisLink('faultAt', t)          drive fault injected at t seconds
%   fakeAxisLink('valveAt', t, Cd)      outlet opens at t seconds, Q = Cd A_o sqrt(2P/rho)
%   t    = fakeAxisLink('now')
%          fakeAxisLink('wait', t)      run the slave until t
%   regs = fakeAxisLink('read', start, n)
%          fakeAxisLink('write', start, values)
%          fakeAxisLink('writeOne', address, value)
%   S    = fakeAxisLink('state')        S.collected is the volume that left (mm^3),
%                                       S.collectLog its history [t, volume]

persistent S
out = [];
switch cmd
    case 'init'
        S.map = modbusMap();
        S.cfg = masterConfig();
        S.ccfg = charConfig();
        S.sl = slaveInit();
        S.sl.ax.x = varargin{1};
        S.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
            'faultReset', 0, 'reference', 0);
        [S.sl, S.y] = slaveStep(S.sl, S.u);
        S.tick = 0;
        S.faultAt = Inf;
        S.valveAt = Inf;
        S.Cd = 0.7;
        S.valveOpen = false;
        S.collected = 0;
        S.collectLog = zeros(0, 2);     % [t collected] every scan while the valve is open
        pr = S.ccfg.probe;
        S.p = struct('state', 0, 'echo', 0, 'trig', 0, 'arm', 0, 'lastArm', 0, ...
            'idx', 0, 'post', 0, 'lastCmd', 0, 'ring', zeros(pr.nSamples, 4));
    case 'faultAt'
        S.faultAt = varargin{1};
    case 'valveAt'
        S.valveAt = varargin{1};
        if numel(varargin) > 1
            S.Cd = varargin{2};
        end
    case 'now'
        out = S.tick / 1000;
    case 'wait'
        while S.tick / 1000 < varargin{1}
            S.tick = S.tick + 1;
            if S.tick / 1000 >= S.faultAt
                S.sl.hook.driveFault = true;
            end
            if mod(S.tick, 2) == 0
                S = valve(S, S.sl.G.G_cfgScanTime);
                [S.sl, S.y] = slaveStep(S.sl, S.u);
                S = probeScan(S);
            end
        end
    case 'read'
        start = varargin{1};
        n = varargin{2};
        pr = S.ccfg.probe;
        last = pr.irSamples + 4 * pr.nSamples - 1;
        regs = zeros(1, last);
        regs(1:S.map.nRead) = packRead(S.y, S.map);
        regs(pr.irState) = S.p.state;
        regs(pr.irArmEcho) = S.p.echo;
        regs(pr.irTrigIdx) = S.p.trig;
        regs(pr.irCount) = pr.nSamples;
        ring = S.p.ring(:);
        ring(ring < 0) = ring(ring < 0) + 65536;
        regs(pr.irSamples:last) = ring';
        assert(start >= 1 && start + n - 1 <= last, 'read outside the register groups');
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
        assert(varargin{1} == S.ccfg.probe.hrArm, 'single write is not the arm register');
        S.p.arm = varargin{2};
    case 'state'
        out = S;
end
end


function S = valve(S, dt)
% Orifice outflow once the valve is open; the tube's air leaves at once.
if ~S.valveOpen && S.tick / 1000 >= S.valveAt
    S.valveOpen = true;
    S.sl.ax.vOut = S.sl.ax.vOut + S.ccfg.tubeVolume_mL * 1000;
end
if S.valveOpen
    P = max(S.sl.ax.P, 0) * 1e6;
    Q = S.Cd * S.ccfg.orificeArea_m2 * sqrt(2 * P / S.ccfg.rho) * 1e9;    % mm^3/s
    S.sl.hook.outflow = Q;
    S.collected = S.collected + Q * dt;
    S.collectLog(end + 1, :) = [S.tick / 1000, S.collected];
else
    S.sl.hook.outflow = 0;
end
end


function S = probeScan(S)
% One scan of PRG_AxisProbe.
pr = S.ccfg.probe;
p = S.p;
G = S.sl.G;
if p.arm ~= p.lastArm && p.state ~= 1 && p.state ~= 2
    p.lastArm = p.arm;
    p.state = 1;
end
if p.state ~= 3
    i = p.idx + 1;
    p.ring(i, :) = round([G.G_sysActualVelocity * pr.scale(1), G.G_sysCommandedVelocity * pr.scale(2), ...
        G.G_sysActualTorque * pr.scale(3), G.G_sysActualPressure * pr.scale(4)]);
    if p.state == 1 && abs(G.G_sysCommandedVelocity - p.lastCmd) > pr.trigger
        p.trig = p.idx;
        p.post = pr.nSamples - pr.preTrigger - 1;
        p.state = 2;
    elseif p.state == 2
        p.post = p.post - 1;
        if p.post <= 0
            p.echo = p.lastArm;
            p.state = 3;
        end
    end
    p.idx = mod(p.idx + 1, pr.nSamples);
end
p.lastCmd = G.G_sysCommandedVelocity;
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
