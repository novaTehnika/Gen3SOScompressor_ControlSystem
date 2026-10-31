function closedLoopModbusStep()
%CLOSEDLOOPMODBUSSTEP Run masterStep (the Simulink block's entry point)
%   against the simulated slave through the Modbus register vectors: the
%   master steps every cfg.dt, the registers are exchanged every
%   cfg.exchangePeriod as packed 16-bit words, and the slave scans every
%   G_cfgScanTime (4 ms), on a 1 ms tick. Covers homing, position, pressure,
%   fault reset
%   and link loss on either side.
%
%   masterStep uses masterConfig as it is, so the slave's soft limit is
%   widened to the bench value to match.
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools

clear masterStep masterStateMachine
cfg = masterConfig();
map = modbusMap();

h.sl = slaveInit();
h.sl.G.G_cfgPosSoftLimitMax = cfg.posMax + 5;
h.sl.sim.switchPosPos = cfg.posMax + 10;
h.map = map;
h.cfg = cfg;

h.app = struct('op', 0, 'cmdSeq', 0, 'targetPosition', 100, 'targetVelocity', 0, ...
    'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0);
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0, 'salineVolume', 0, 'oxygenRatio', 0);
[h.sl, h.y] = slaveStep(h.sl, h.u);
h.mbRead = zeros(1, map.nRead);
h.tick = 0;
h.m = [];
h.writeLost = false;    % master's registers do not reach the slave
h.readLost = false;     % slave's registers do not reach the master

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
h.sl.ax.x = chamberPosition(h.sl.sim, 1);
h.app.targetPressure = 20 + cfg.atmAmbient;     % atm absolute
h = command(h, 4);
h = runUntil(h, 10, @(h) h.m.statusCode == 5);
assert(h.m.statusCode == 5 && h.u.modeCode == 7);
assert(abs(h.u.reference - 20 / cfg.atmPerMPa) < 1e-3);
% The fill description rides in the same exchange and is latched by the
% slave on entry to the mode, as the REAL it was sent as.
assert(h.u.salineVolume == single(cfg.salineVolume) && h.u.oxygenRatio == single(cfg.oxygenRatio));
assert(h.sl.s.rSchedSaline == single(cfg.salineVolume) && h.sl.s.rSchedOxygenRatio == single(cfg.oxygenRatio), ...
    'fill not latched by the slave');
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
assert(h.u.modeCode == 2 && h.u.motionEnable == 0);
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
assert(h.u.motionEnable == 0);
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


function regs = packRead(y, map)
% The slave's registers as the Modbus Client Read block delivers them.
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


function u = unpackWrite(w, map)
% The master's registers as the slave sees them.
u.heartbeat    = w(map.hr.heartbeat);
u.modeCode     = w(map.hr.modeCode);
u.motionEnable = w(map.hr.motionEnable);
u.faultReset   = w(map.hr.faultReset);
u.reference    = modbusUnpack(w(map.hr.reference), w(map.hr.reference + 1));
u.salineVolume = modbusUnpack(w(map.hr.salineVolume), w(map.hr.salineVolume + 1));
u.oxygenRatio  = modbusUnpack(w(map.hr.oxygenRatio), w(map.hr.oxygenRatio + 1));
end


function h = runUntil(h, T, done)
stepTicks = round(h.cfg.dt * 1000);
exchangeTicks = round(h.cfg.exchangePeriod * 1000);
for n = 1:round(T * 1000)
    h.tick = h.tick + 1;

    if mod(h.tick, stepTicks) == 0
        a = h.app;
        [m.mbWrite, m.doSafety, m.doMixer, m.position, m.velocity, m.pressure, ...
            m.statusCode, m.faultCode, m.state, m.linkOk] = masterStep(h.mbRead, ...
            a.op, a.cmdSeq, a.targetPosition, ...
            a.targetVelocity, a.jogDirection, a.targetPressure, a.estop, a.mixer);
        h.m = m;
    end

    if mod(h.tick, exchangeTicks) == 0 && ~isempty(h.m)
        if ~h.writeLost
            h.u = unpackWrite(h.m.mbWrite, h.map);
        end
        if ~h.readLost
            h.mbRead = packRead(h.y, h.map);
        end
    end

    if mod(h.tick, round(1000 * h.sl.G.G_cfgScanTime)) == 0
        [h.sl, h.y] = slaveStep(h.sl, h.u);
    end

    if ~isempty(done) && ~isempty(h.m) && done(h)
        return;
    end
end
end
