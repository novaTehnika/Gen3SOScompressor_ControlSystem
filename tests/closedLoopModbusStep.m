function closedLoopModbusStep()
%CLOSEDLOOPMODBUSSTEP Run masterStep (the Simulink block's entry point)
%   against the simulated slave through the Modbus registers and the link
%   timing model (modbusLink, modbusLinkConfig: the bench's measurements):
%   the master steps every cfg.dt; every cfg.exchangePeriod it reads the
%   slave's input registers, steps on what it read and writes its holding
%   registers, as 16-bit words packed and unpacked by the slave's register
%   map (the GVL's AT addresses); and the slave scans every G_cfgScanTime
%   (4 ms), serving the link's requests around its scans, on a 1 ms tick. Checks that the master's
%   register map (modbusMap) agrees with the slave's, then covers homing,
%   position, pressure, fault reset and link loss on either side.
%
%   masterStep uses masterConfig as it is, so the slave's soft limit is
%   widened to the bench value to match.
%   Run from the repository root after  octave-cli --eval "addpath slave-sim; buildSlaveSim"
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools models

clear masterStep masterStateMachine
cfg = masterConfig();
map = modbusMap();

h.sl = slaveInit();
h.sl.G.G_cfgPosSoftLimitMax = cfg.posMax + 5;
h.sl.sim.switchPosPos = cfg.posMax + 10;
h.map = map;
h.cfg = cfg;

h.app = struct('op', 0, 'cmdSeq', 0, 'targetPosition', 100, 'targetVelocity', 0, ...
    'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0, ...
    'positionSpeed', cfg.positionSpeed);
h.y = scan(h.sl);
h.mbRead = zeros(1, map.nRead);
% The Modbus link with the bench's timing (modbusLinkConfig)
h.link = modbusLink('init', modbusLinkConfig(), h.sl.G.G_cfgScanTime);
h.readId = 0;
h.pending = false;
h.log = struct('read', zeros(0, 2), 'written', zeros(1, 0));
h.tick = 0;
h.m = [];
h.writeLost = false;    % master's registers do not reach the slave
h.readLost = false;     % slave's registers do not reach the master

% The master's register map is the slave's: every register modbusMap
% names is where the GVL's AT address puts the same variable, and the
% exchange covers every register the slave has.
R = slaveSimMex('registers');
pairs = {'hr', 'heartbeat', 'G_mbHeartbeat'; 'hr', 'modeCode', 'G_mbModeCode';
         'hr', 'motionEnable', 'G_mbMotionEnable'; 'hr', 'faultReset', 'G_mbFaultReset';
         'hr', 'reference', 'G_mbReference'; 'hr', 'positionSpeed', 'G_mbPositionSpeed';
         'ir', 'heartbeatEcho', 'G_mbHeartbeatEcho'; 'ir', 'slaveCounter', 'G_mbSlaveCounter';
         'ir', 'confCode', 'G_mbConfCode'; 'ir', 'faultCode', 'G_mbFaultCode';
         'ir', 'faultActive', 'G_mbFaultActive'; 'ir', 'inMotion', 'G_mbInMotion';
         'ir', 'performanceStatus', 'G_mbPerformanceStatus';
         'ir', 'homingComplete', 'G_mbHomingComplete'; 'ir', 'position', 'G_mbActualPosition';
         'ir', 'velocity', 'G_mbActualVelocity'; 'ir', 'torque', 'G_mbActualTorque';
         'ir', 'commandedPosition', 'G_mbCommandedPosition';
         'ir', 'pressure', 'G_mbActualPressure'};
% Every register of the slave inside the exchange is one modbusMap names
% (the registers beyond it are the step probe's, read separately).
inside = arrayfun(@(r) r.first <= map.(groupOf(sideOf(r.group), 'nWrite', 'nRead')), R);
assert(sum(inside) == rows(pairs), 'the exchange covers %d of the slave''s registers, modbusMap names %d', ...
    sum(inside), rows(pairs));
for k = 1:rows(pairs)
    r = R(strcmp({R.name}, pairs{k, 3}));
    assert(numel(r) == 1, '%s has no register', pairs{k, 3});
    assert(strcmp(r.group, groupOf(pairs{k, 1})) && r.first == map.(pairs{k, 1}).(pairs{k, 2}), ...
        'modbusMap.%s.%s is %d; %s is %s register %d', pairs{k, 1}, pairs{k, 2}, ...
        map.(pairs{k, 1}).(pairs{k, 2}), pairs{k, 3}, r.group, r.first);
    last = r.first + r.words - 1;
    assert(last <= map.(groupOf(pairs{k, 1}, 'nWrite', 'nRead')), '%s beyond the exchange', pairs{k, 3});
end
fprintf('ok  register map: modbusMap agrees with the GVL for %d registers\n', rows(pairs));

% Until the slave's counter is seen the link is not up: safety relay off,
% waiting status, no latch once it appears.
h.readLost = true;
h = runUntil(h, 0.5, []);
assert(h.m.statusCode == 35 && h.m.doSafety == 0 && h.m.linkOk == 0);
h.readLost = false;
h = runUntil(h, 0.5, []);
assert(h.m.statusCode == 0 && h.m.doSafety == 1 && h.m.linkOk == 1);
assert(h.y.heartbeatEcho > 0, 'heartbeat not echoed');
fprintf('ok  start-up: waits for the slave, then idle; heartbeat echoed\n');

% With the link's timing the slave's reply to a write appears in the
% following exchange: the heartbeat echo each read returns is the
% heartbeat the master wrote in the exchange before.
n0 = rows(h.log.read);
h = runUntil(h, 3, []);
echo = h.log.read(n0 + 1:end, 2)';
written = h.log.written(n0:end - 1);
nPair = min(numel(echo), numel(written));
assert(isequal(echo(1:nPair), written(1:nPair)), 'echoes %s for writes %s', ...
    mat2str(echo(1:min(nPair, 8))), mat2str(written(1:min(nPair, 8))));
tr = sort(h.log.read(n0 + 1:end, 1));
fprintf('ok  echo in the following exchange over %d exchanges; read call median %.1f ms, max %.1f ms\n', ...
    nPair, 1000 * median(tr), 1000 * tr(end));

% Homing from close to the negative overtravel switch.
h.sl.ax.x = 10;
h = command(h, 1);
h = runUntil(h, 60, @(h) h.sl.G.G_flagHomingComplete && h.m.state == 1);
assert(h.sl.G.G_flagHomingComplete && h.m.state == 1, 'homing did not complete');
fprintf('ok  homing through the registers\n');

h.app.targetPosition = 5;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.m.statusCode == 3);
assert(h.m.statusCode == 3 && abs(h.sl.ax.x - 5) <= cfg.posTolerance);
assert(abs(h.m.position - h.sl.ax.x) < 0.1);
fprintf('ok  position move, displayed position %.3f mm\n', h.m.position);

% Position speed through HR7-8: the packed value reaches the slave, a move
% runs at it, 0 mid-move holds position and the speed above the ceiling is
% limited.
assert(abs(h.sl.G.G_mbPositionSpeed - cfg.positionSpeed) < 1e-6);
h.app.positionSpeed = 2;
h.app.targetPosition = 25;
h = command(h, 2);
% (The new speed reaches the slave an exchange or more after the command,
% so the condition waits for it as well as for the motion.)
h = runUntil(h, 30, @(h) abs(h.sl.G.G_mbPositionSpeed - 2) < 1e-6 && abs(h.sl.ax.v) > 1.9);
h = runUntil(h, 0.3, []);
assert(abs(h.sl.G.G_mbPositionSpeed - 2) < 1e-6 && abs(h.sl.ax.v - 2) < 0.1);
h.app.positionSpeed = 0;
h = runUntil(h, 3, []);
xHold = h.sl.ax.x;
h = runUntil(h, 1, []);
assert(h.sl.G.G_mbPositionSpeed == 0 && abs(h.sl.ax.v) < 0.01 && abs(h.sl.ax.x - xHold) < 0.01);
assert(h.y.faultActive == 0 && h.m.statusCode == 2 && xHold < 24);
h.app.positionSpeed = 100;
h = runUntil(h, 30, @(h) h.m.statusCode == 3);
assert(h.m.statusCode == 3 && abs(h.sl.G.G_mbPositionSpeed - cfg.positionSpeedMax) < 1e-6);
h.app.positionSpeed = cfg.positionSpeed;
fprintf('ok  position speed through HR7-8: 2 mm/s, 0 holds at %.2f mm, 100 limited to %g mm/s\n', ...
    xHold, h.sl.G.G_mbPositionSpeed);

% Go Home with a slow drive start. The link carries no BrakeDisengage, so
% the master enters the operation tEnableMax after raising MotionEnable,
% possibly before the axis moves; rest before any motion must not end it.
% Go Home from home never moves and ends after tGoHomeStartMax.
h.app.op = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE, 'slave did not return to idle');
powerOnDelay = h.sl.sim.powerOnDelay;
h.sl.sim.powerOnDelay = 2.5;
h = command(h, 6);
h = runUntil(h, 1, []);
h = runUntil(h, 30, @(h) h.m.state == 1);
assert(abs(h.sl.ax.x - h.sl.G.G_cfgGoHomePosition) < 0.1, 'Go Home ended before arriving');
h.sl.sim.powerOnDelay = powerOnDelay;
t0 = h.tick / 1000;
h = command(h, 6);
h = runUntil(h, 1, []);
h = runUntil(h, 15, @(h) h.m.state == 1);
assert(h.m.state == 1 && h.tick / 1000 - t0 < cfg.tGoHomeStartMax + 3, 'Go Home from home did not end');
fprintf('ok  go home with a slow drive start, and from home (%.1f s)\n', h.tick / 1000 - t0);

% Pressure operation: the target goes to the slave in MPa gauge as mode 7,
% the slave's transducer reading comes back in IR19-20 and is shown in atm
% absolute. The axis is placed in the chamber below the target once the
% slave has released it.
h = command(h, 0);
h = runUntil(h, 5, @(h) h.m.state == 1 && ...
    h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE);
h.sl.ax.x = chamberPosition(h.sl, 1);
h.app.targetPressure = 20 + cfg.atmAmbient;     % atm absolute
h = command(h, 4);
h = runUntil(h, 10, @(h) h.m.statusCode == 5);
assert(h.m.statusCode == 5 && h.sl.G.G_mbModeCode == 7);
assert(abs(h.sl.G.G_mbReference - 20 / cfg.atmPerMPa) < 1e-3);
h = runUntil(h, 30, @(h) h.y.performanceStatus == 1);
assert(h.y.performanceStatus == 1, 'pressure target not reached');
% Let the loop settle, so that the display (filtered on the master and
% read every 50 ms) is compared with the true pressure at rest
h = runUntil(h, 2, []);
assert(abs(h.m.pressure - (h.sl.ax.P * cfg.atmPerMPa + cfg.atmAmbient)) < 0.5);
fprintf('ok  pressure operation (%.1f atm absolute shown)\n', h.m.pressure);

% Drive fault: latched, the full code mirrored in the mode register, and
% cleared by the operator reset.
h.sl.hook.driveFault = true;
h = runUntil(h, 0.5, []);
h.sl.hook.driveFault = false;
assert(h.m.statusCode == 10 && h.m.faultCode == 2);
assert(h.sl.G.G_mbModeCode == 2 && h.sl.G.G_mbMotionEnable == 0);
h = command(h, 5);
h = runUntil(h, 1.5, []);
assert(h.y.faultActive == 0 && h.m.state == 1);
fprintf('ok  fault, mirror and reset\n');
h.sl.ax.x = 25;                     % back out of the chamber

% Link loss seen by the slave while quiescent is not a fault: after the
% reset it waits in brake hold, the master stopping is routine, and the
% link resumes without any reset.
E = h.sl.E.E_SystemState;
assert(h.sl.G.G_sysCurrentState == E.ST_BRAKE_HOLD);
h.writeLost = true;
h = runUntil(h, 0.5, []);
assert(h.y.faultActive == 0 && h.sl.G.G_sysCurrentState == E.ST_BRAKE_HOLD);
h.writeLost = false;
h = runUntil(h, 0.3, []);
assert(h.y.faultActive == 0 && h.m.statusCode == 0);
fprintf('ok  slave link loss while quiescent -> no fault\n');

% Link loss seen by the slave: the heartbeat freezes during a jog, the
% slave raises FAULT_COMM and stops, the master shows the fault.
h.app.targetVelocity = 2;
h.app.jogDirection = 1;
h = command(h, 3);
h = runUntil(h, 10, @(h) h.m.statusCode == 4);
h = runUntil(h, 2, @(h) h.sl.ax.v > 1);
assert(h.m.statusCode == 4 && h.sl.ax.v > 1);
h.writeLost = true;
h = runUntil(h, 0.5, []);
assert(h.y.faultActive == 1 && h.y.faultCode == 4, 'slave did not raise FAULT_COMM');
assert(h.m.statusCode == 10 && h.m.faultCode == 4);
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.v == 0, 'axis did not stop after FAULT_COMM');
h.writeLost = false;
h.app.op = 0;
h = runUntil(h, 0.5, []);
h = command(h, 5);
h = runUntil(h, 1.5, []);
assert(h.y.faultActive == 0 && h.m.state == 1 && h.m.linkOk == 1);
fprintf('ok  slave link loss -> FAULT_COMM, reset\n');

% Link loss seen by the master: the slave's registers freeze during a jog.
% The master drops MotionEnable and the safety relay, shows LINK_LOST, and
% stays latched after the registers resume until the operator sends Stop.
h = command(h, 3);
h = runUntil(h, 10, @(h) h.m.statusCode == 4);
h = runUntil(h, 2, @(h) h.sl.ax.v > 1);
assert(h.m.statusCode == 4 && h.sl.ax.v > 1);
h.readLost = true;
h = runUntil(h, 0.5, []);
assert(h.m.linkOk == 0 && h.m.statusCode == 34 && h.m.doSafety == 0);
assert(h.sl.G.G_mbMotionEnable == 0);
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.v == 0, 'axis did not stop after MotionEnable dropped');
h.readLost = false;
h = runUntil(h, 0.5, []);
assert(h.m.linkOk == 0 && h.m.statusCode == 34, 'link loss did not stay latched');
h = command(h, 0);
h = runUntil(h, 0.5, []);
assert(h.m.linkOk == 1 && h.m.doSafety == 1 && h.m.state == 1);
h.app.targetPosition = round(h.m.position) + 3;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.m.statusCode == 3);
assert(h.m.statusCode == 3, 'position move after link loss did not reach target');
fprintf('ok  master link loss -> latched until Stop\n');

% E-stop drops the safety output and the mixer.
h.app.mixer = 1;
h = runUntil(h, 0.1, []);
assert(h.m.doMixer == 1);
h.app.estop = 1;
h = runUntil(h, 0.1, []);
assert(h.m.doSafety == 0 && h.m.doMixer == 0 && h.m.statusCode == 20);
h.app.estop = 0;
h = runUntil(h, 0.5, []);
assert(h.m.doSafety == 1 && h.m.state == 1);
fprintf('ok  e-stop\n');

% Displayed volume: dead volume at the end of travel, bore area per mm.
assert(abs(x2mL(cfg.posEOT, cfg) - cfg.deadVolume) < 1e-9);
assert(abs(x2mL(cfg.posEOT - 100, cfg) - cfg.deadVolume ...
           - 100 * pi / 4 * (2.602 * 25.4)^2 / 1000) < 1e-6);
fprintf('ok  volume scaling (%.3f mL/mm)\n', mLPerMm(cfg));

fprintf('all passed\n');
end


function h = command(h, op)
h.app.op = op;
h.app.cmdSeq = h.app.cmdSeq + 1;
end


function side = sideOf(group)
% The modbusMap side of a slave register group.
if strcmp(group, 'holding')
    side = 'hr';
else
    side = 'ir';
end
end


function g = groupOf(side, hr, ir)
% The slave's register group of a modbusMap side, or the given names.
if nargin < 2
    hr = 'holding';
    ir = 'input';
end
if strcmp(side, 'hr')
    g = hr;
else
    g = ir;
end
end


function h = runUntil(h, T, done)
% On a 1 ms tick: the master steps every cfg.dt; at an exchange step it
% reads the input registers through the link, steps when the read returns
% and then writes its holding registers (the Read block, masterStep, the
% Write block). The slave scans every G_cfgScanTime and serves the link's
% requests around each scan: reads before it, writes after it.
stepTicks = round(h.cfg.dt * 1000);
exchangeTicks = round(h.cfg.exchangePeriod * 1000);
scanTicks = round(1000 * h.sl.G.G_cfgScanTime);
for n = 1:round(T * 1000)
    h.tick = h.tick + 1;
    t = h.tick / 1000;

    if mod(h.tick, stepTicks) == 0 && ~h.pending
        if mod(h.tick, exchangeTicks) ~= 0 || isempty(h.m)
            h = stepMaster(h);
        elseif h.readLost
            h = stepMaster(h);
            h = submitWrite(h, t);
        else
            [h.link, h.readId] = modbusLink('submit', h.link, t, 'read', 1, h.map.nRead);
            h.pending = true;
        end
    end

    [h.link, fin] = modbusLink('poll', h.link, t);
    for d = fin
        if strcmp(d.kind, 'read') && d.id == h.readId
            h.mbRead = d.data;
            h.log.read(end + 1, :) = [d.tDone - d.tCall, d.data(h.map.ir.heartbeatEcho)];
            h = stepMaster(h);
            h = submitWrite(h, t);
            h.pending = false;
        end
    end

    if mod(h.tick, scanTicks) == 0
        k = h.tick / scanTicks;
        [h.link, reqs] = modbusLink('due', h.link, k, 'read');
        for q = reqs
            h.link = modbusLink('answer', h.link, q.slot, readInputRegisters(h.sl, q.start, q.n));
        end
        h.y = scan(h.sl);
        [h.link, reqs] = modbusLink('due', h.link, k, 'write');
        for q = reqs
            writeHoldingRegisters(h.sl, q.start, q.words);
            h.link = modbusLink('answer', h.link, q.slot, []);
        end
    end

    if ~isempty(done) && ~isempty(h.m) && done(h)
        return;
    end
end
end


function h = stepMaster(h)
a = h.app;
[m.mbWrite, m.doSafety, m.doMixer, m.position, m.velocity, m.pressure, ...
    m.statusCode, m.faultCode, m.state, m.linkOk] = masterStep(h.mbRead, ...
    a.op, a.cmdSeq, a.targetPosition, ...
    a.targetVelocity, a.jogDirection, a.targetPressure, a.estop, a.mixer, ...
    a.positionSpeed);
h.m = m;
end


function h = submitWrite(h, t)
% The master's write of this exchange, unless the link loses it; the
% heartbeat written is logged against the read that preceded it.
if ~h.writeLost
    [h.link, ~] = modbusLink('submit', h.link, t, 'write', 1, h.m.mbWrite);
    h.log.written(end + 1) = h.m.mbWrite(h.map.hr.heartbeat);
else
    h.log.written(end + 1) = NaN;
end
if h.readLost
    h.log.read(end + 1, :) = [NaN, NaN];
end
end
