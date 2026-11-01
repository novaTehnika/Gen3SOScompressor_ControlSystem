function out = fakeProbeLink(cmd, varargin)
%FAKEPROBELINK Stand-in for the bench runs' Modbus link and clock: the
%   simulated slave's own registers behind the protocol registers (IR1-20,
%   HR1-12) and, behind the step probe registers (HR111-114, IR401-524),
%   the slave's own FB_Probe. The slave's
%   registers are packed and unpacked by its register map (the GVL's AT
%   addresses, readInputRegisters / writeHoldingRegisters), so the ring,
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
%   fakeProbeLink('ringDepth', n)       use a ring of n records (see below)
%   fakeProbeLink('torqueModel', p)     report a friction torque (below)
%   fakeProbeLink('inputNoise', v)      white noise of v volts rms on the
%                                       slave's analog input (the transducer
%                                       model plus noise, through the
%                                       pressureVolt hook); 0, the default,
%                                       leaves the model's own voltage
%   fakeProbeLink('valveAt', t, Cd)     outlet valve opens at t seconds: the
%                                       plant's orifice bank (32 of 40 um,
%                                       discharge coefficient Cd) drains the
%                                       chamber, after the tube fills
%   fakeProbeLink('linkTiming', cfg)    the link's timing (modbusLinkConfig,
%                                       the default); [] makes calls instant
%   t    = fakeProbeLink('now')
%          fakeProbeLink('wait', t)     run the slave until t
%   regs = fakeProbeLink('read', start, n)
%          fakeProbeLink('write', start, values)
%          fakeProbeLink('writeOne', address, value)
%
%   A read or write is a blocking call through the Modbus link model
%   (modbusLink): it is split into requests by the Modbus limits, served
%   one per slave tick around the scans (reads before a scan, writes after
%   it) and returns after the client delay, with the slave running all the
%   while. S.calls logs every call as [t called, t returned, requests,
%   registers, is a read].
%   S    = fakeProbeLink('state')       S.collectLog is [t, mm^3 collected at
%                                       the orifices, mol of free gas in the
%                                       chamber] while the valve is open;
%                                       S.ring has the log of records, one row
%                                       per record index from 0: [vCmd vel
%                                       torque pos pressure raw status] in
%                                       counts; S.sp the probe's written,
%                                       depth, arm, echo, mark, state, xfer,
%                                       faultMark and the hooks' counts
%
%   The simulated chamber, outlet and drive are the design model's plant
%   (models/c/plant.c), so the drive's dead time and lag and the
%   chamber's stiffness are its own (sl.plant.par). The torque model stands
%   in for the drive's torque feedback, which the simulated slave reports as
%   zero: it is a friction law chosen by the test, for the friction fit to
%   recover. With p a struct of F0, Fc, dFs, vs, b, mu,
%   noisePct and tStick, the reported torque (% rated) is the piston force
%     F0 + sign(v) (Fc + dFs exp(-(|v|/vs)^2) + mu P A) + b v + P A
%   in N divided by newtonPerPct, plus white noise, with v the axis
%   velocity (mm/s), P the chamber pressure (MPa) and A the piston area
%   (mm^2). A nonzero tStick holds the axis still for that long after each
%   command from rest while the torque ramps to F0 + sign (Fc + dFs) + P A,
%   which is what breakaway looks like in the log.

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
        S.y = scan(S.sl);
        S.tick = 0;
        S.faultAt = Inf;
        S.faultUntil = Inf;
        S.torque = [];
        S.valve = struct('at', Inf, 'Cd', 0.7, 'open', false, 'vOut0', 0, 'tube', 0);
        S.collectLog = zeros(0, 3);
        S.stick = struct('until', -Inf, 'sign', 0, 'armed', true);
        S.hook = struct('dropArm', 0, 'tear', 0);
        S.linkCfg = modbusLinkConfig();
        S.link = modbusLink('init', S.linkCfg, S.sl.G.G_cfgScanTime);
        S.calls = zeros(0, 5);
        S.nT = 0;
        R = zeros(4096, 7);
        S.noise = 0;            % V rms added to the analog input
    case 'faultAt'
        S.faultAt = varargin{1};
        S.faultUntil = Inf;
        if numel(varargin) > 1
            S.faultUntil = varargin{2};
        end
    case 'slave'
        S.sl = varargin{1}(S.sl);
    case 'inputNoise'
        S.noise = varargin{1};
        if S.noise == 0
            S.sl.hook.pressureVolt = NaN;
        end
    case 'torqueModel'
        S.torque = varargin{1};
    case 'dropArm'
        S.hook.dropArm = varargin{1};
    case 'tearRead'
        S.hook.tear = varargin{1};
    case 'ringDepth'
        S.sl.s.fbProbe.cDepth = varargin{1};
    case 'linkTiming'
        S.linkCfg = varargin{1};
        if ~isempty(S.linkCfg)
            S.link = modbusLink('init', S.linkCfg, S.sl.G.G_cfgScanTime);
        end
    case 'valveAt'
        S.valve.at = varargin{1};
        if numel(varargin) > 1
            S.valve.Cd = varargin{2};
        end
    case 'now'
        out = S.tick / 1000;
    case 'wait'
        [S, R] = advance(S, R, varargin{1}, []);
    case 'read'
        start = varargin{1};
        n = varargin{2};
        assert(start >= 1 && start + n - 1 <= 524, 'read outside the register groups');
        [S, R, out] = call(S, R, 'read', start, n);
    case 'write'
        w = varargin{2};
        first = varargin{1};
        inProbe = first >= 111 && first <= 114;
        assert((inProbe && first + numel(w) - 1 <= 114) || (first == 1 && numel(w) == S.map.nWrite), ...
            'write is not HR1-nWrite or HR111-114');
        if first == 1
            % The master's last write, decoded, for the tests' checks
            S.u.heartbeat    = w(S.map.hr.heartbeat);
            S.u.modeCode     = w(S.map.hr.modeCode);
            S.u.motionEnable = w(S.map.hr.motionEnable);
            S.u.faultReset   = w(S.map.hr.faultReset);
            S.u.reference    = modbusUnpack(w(S.map.hr.reference), w(S.map.hr.reference + 1));
            S.u.positionSpeed = modbusUnpack(w(S.map.hr.positionSpeed), w(S.map.hr.positionSpeed + 1));
        end
        [S, R] = call(S, R, 'write', first, w);
    case 'writeOne'
        assert(any(varargin{1} == [111 112 113 114]), 'single write is not HR111-114');
        [S, R] = call(S, R, 'write', varargin{1}, varargin{2});
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


function [S, R, data] = call(S, R, kind, start, arg)
% A blocking call: through the link model, the slave running until it
% returns; instant (served at once, no time passing) without link timing.
data = [];
t0 = S.tick / 1000;
if isempty(S.linkCfg)
    if strcmp(kind, 'read')
        [S, data] = serveRead(S, start, arg);
    else
        S = serveWrite(S, start, arg);
    end
    S.calls(end + 1, :) = [t0, t0, 1, numel(data) + numel(arg) * strcmp(kind, 'write'), strcmp(kind, 'read')];
    return;
end
[S.link, id] = modbusLink('submit', S.link, t0, kind, start, arg);
[S, R, d] = advance(S, R, Inf, id);
data = d.data;
S.calls(end + 1, :) = [d.tCall, d.tDone, d.requests, ...
    ifelse(strcmp(kind, 'read'), arg, numel(arg)), strcmp(kind, 'read')];
end


function v = ifelse(c, a, b)
if c
    v = a;
else
    v = b;
end
end


function [S, R, done] = advance(S, R, tEnd, id)
% Run the slave on the 1 ms tick until tEnd or, with id, until the link
% returns that call (done: its record). Around each scan the link's due
% requests are served: reads before the scan, writes after it.
done = [];
scanTicks = round(1000 * S.sl.G.G_cfgScanTime);
while isempty(id) && S.tick / 1000 < tEnd || ~isempty(id)
    S.tick = S.tick + 1;
    if S.tick / 1000 >= S.faultAt && S.tick / 1000 < S.faultUntil
        S.sl.hook.driveFault = true;
    elseif S.tick / 1000 >= S.faultUntil
        S.sl.hook.driveFault = false;
    end
    if mod(S.tick, scanTicks) == 0
        k = S.tick / scanTicks;
        if ~isempty(S.torque)
            S = torqueScan(S);
        end
        S = valveScan(S);
        if ~isempty(S.linkCfg)
            [S.link, reqs] = modbusLink('due', S.link, k, 'read');
            for q = reqs
                [S, words] = serveRead(S, q.start, q.n);
                S.link = modbusLink('answer', S.link, q.slot, words);
            end
        end
        pre = [S.sl.G.G_sysActualVelocity, S.sl.G.G_sysActualTorque, ...
               S.sl.G.G_sysActualPosition];     % what the probe is given
        if S.noise > 0
            S = inputScan(S);
        end
        S.y = scan(S.sl);
        [S, R] = logScan(S, R, pre);
        if ~isempty(S.linkCfg)
            [S.link, reqs] = modbusLink('due', S.link, k, 'write');
            for q = reqs
                S = serveWrite(S, q.start, q.words);
                S.link = modbusLink('answer', S.link, q.slot, []);
            end
        end
    end
    if ~isempty(id)
        [S.link, fin] = modbusLink('poll', S.link, S.tick / 1000);
        if ~isempty(fin) && any([fin.id] == id)
            done = fin([fin.id] == id);
            return;
        end
    end
end
end


function [S, words] = serveRead(S, start, n)
% The registers start..start+n-1 as the controller answers a read now: the
% slave's own (protocol and step probe).
regs = zeros(1, 524);
regs(1:S.map.nRead) = readInputRegisters(S.sl, 1, S.map.nRead);
if start + n - 1 >= 401
    regs(401:524) = readInputRegisters(S.sl, 401, 124);
end
if start <= 519 && start + n - 1 >= 520 && S.hook.tear > 0
    tail = regs(519) + 65536 * regs(520) + 1;       % a window that changed under the read
    regs(519:520) = [mod(tail, 65536), floor(tail / 65536)];
    S.hook.tear = S.hook.tear - 1;
end
words = regs(start:start + n - 1);
end


function S = serveWrite(S, start, words)
% A write as the controller takes it, to the slave's registers; a lost arm
% write (the dropArm hook) never arrives.
if start <= 111 && start + numel(words) - 1 >= 111 && S.hook.dropArm > 0
    S.hook.dropArm = S.hook.dropArm - 1;
    return;
end
writeHoldingRegisters(S.sl, start, words);
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
PA = sl.ax.P * 1e6 * sl.plant.par.A;            % MPa * m^2 -> N
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
% The outlet valve: from its opening, the plant's orifice bank at the
% valve's discharge coefficient, and in the opening scan the flow that
% fills the tube, eq. (37) of models/README.md.
if isinf(S.valve.at)
    return;
end
t = S.tick / 1000;
dt = S.sl.G.G_cfgScanTime;
if ~S.valve.open && t >= S.valve.at
    S.valve.open = true;
    Ao = 32 * pi / 4 * (40e-6)^2;                             % m^2
    S.sl.plant.par.k_orifice = S.valve.Cd * Ao * sqrt(2 / 1000);
    S.valve.vOut0 = S.sl.ax.vOut;
    S.valve.tube = 236;                                       % mm^3
    S.sl.hook.valve = 1;
    S.sl.hook.outflow = S.valve.tube / dt;
elseif S.valve.open
    S.sl.hook.outflow = 0;
    S.collectLog(end + 1, :) = [t, S.sl.ax.vOut - S.valve.vOut0 - S.valve.tube, ...
        S.sl.plant.n_tot - S.sl.plant.n_dis];
end
end


function [S, R] = logScan(S, R, pre)
% Log the record the probe wrote in the scan just run, as counts: the
% values it was given (the axis feedback as it stood before the scan, the
% rest as the scan left them) converted here, independently of the slave.
w = S.sl.s.fbProbe.Written;
if w <= S.nT
    return;
end
% Single-field reads: a read of S.sl.G builds a snapshot of every global.
lim = @(x, lo, hi) min(max(round(x), lo), hi);
status = S.sl.G.G_sysInMotion + 2 * S.sl.G.G_sysPerformanceStatus ...
    + 4 * S.sl.G.G_sysFaultActive + 8 * S.sl.G.G_sysHomingComplete ...
    + 256 * min(S.sl.G.G_sysConfirmedModeCode, 255);
row = [lim(S.sl.G.G_sysCommandedVelocity * 1000, -32767, 32767), ...
       lim(pre(1) * 1000, -32767, 32767), ...
       lim(pre(2) * 100, -32767, 32767), ...
       round(pre(3) * 1e5), ...
       lim(S.sl.G.G_sysActualPressure * 1000, -32767, 32767), ...
       lim(S.sl.G.G_aiPressure * 5000, 0, 65535), double(status)];
while w > size(R, 1)
    R(end + size(R, 1), 1) = 0;                 % twice the rows
end
R(w, :) = row;
S.nT = w;
end


function w = u16s(x)
% Signed integers as 16-bit register values.
w = mod(x, 65536);
end


function S = inputScan(S)
% The transducer model's voltage plus white noise, on the analog input of
% the scan about to run (the pressureVolt hook replaces the model's value).
mA = S.sl.sim.sensorZeroMa + S.sl.sim.sensorSpanMa * S.sl.ax.P / S.sl.sim.sensorRangeMPa;
S.sl.hook.pressureVolt = mA * S.sl.sim.sensorShuntOhm / 1000 + S.noise * randn();
end
