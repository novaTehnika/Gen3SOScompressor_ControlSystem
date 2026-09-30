function closedLoopMasterStep()
%CLOSEDLOOPMASTERSTEP Run masterStep (the Simulink block's entry point)
%   against the simulated slave through the bench DAQ wiring: slave outputs
%   sink the master's inputs (inverted), master outputs in DAQ channel order.
%
%   masterStep uses masterConfig as it is, so the slave's position map and
%   soft limit are set to the bench values to match.
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests

clear masterStep masterStateMachine
cfg = masterConfig();

h.sl = slaveInit();
h.sl.G.G_cfgHandshakeTimeout = 0.5;
h.sl.G.G_cfgPosMapStage2PosMax = cfg.posMapXmax;
h.sl.G.G_cfgPosSoftLimitMax = cfg.posMapXmax;
h.sl.sim.switchPosPos = cfg.posMapXmax + 5;
E = h.sl.E;

h.app = struct('op', 0, 'cmdSeq', 0, 'targetPosition', 100, 'targetVelocity', 0, ...
    'jogDirection', 0, 'targetPressure', 0, 'estop', 0, 'mixer', 0);
h.u = struct('modeBit0', 0, 'modeBit1', 0, 'modeBit2', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'refVoltage', 0);
[h.sl, h.y] = slaveStep(h.sl, h.u);
h.pressureVolt = P2V(1, cfg);      % atmosphere
h.tick = 0;
h.m = [];

h = runUntil(h, 0.5, []);
assert(h.m.statusCode == 0 && h.m.doSlave(1) == 1);

h = command(h, 1);
h = runUntil(h, 60, @(h) h.sl.G.G_flagHomingComplete && h.m.state == 1);
assert(h.sl.G.G_flagHomingComplete && h.m.state == 1, 'homing did not complete');
fprintf('ok  homing through DAQ wiring\n');

h.app.targetPosition = 25;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.m.statusCode == 3);
assert(h.m.statusCode == 3 && abs(h.sl.ax.x - 25) <= cfg.posTolerance);
assert(abs(h.m.position - h.sl.ax.x) < 0.1);
fprintf('ok  position move, displayed position %.2f mm\n', h.m.position);

% Pressure operation: below target the loop drives into the cylinder.
h.app.targetPressure = 50;
h.pressureVolt = P2V(20, cfg);
h = command(h, 4);
h = runUntil(h, 10, @(h) h.m.statusCode == 5);
h = runUntil(h, 3, []);
assert(h.m.statusCode == 5 && h.sl.ax.v > 0.5 && h.m.velocity > 0.5);
assert(h.sl.ax.v <= cfg.pressureVelMax + 0.01);
h.pressureVolt = P2V(50, cfg);     % at target
h = runUntil(h, 3, []);
assert(abs(h.sl.ax.v) < 0.05);
fprintf('ok  pressure operation\n');

% Drive fault: latched and mirrored; operator reset clears it.
h.sl.hook.driveFault = true;
h = runUntil(h, 0.5, []);
h.sl.hook.driveFault = false;
assert(h.m.statusCode == 12 && h.m.faultCode == 2);
h = command(h, 5);
h = runUntil(h, 1.5, []);
assert(h.y.faultActive == 0 && h.m.state == 1);
fprintf('ok  fault and reset\n');

% E-stop drops the safety output and the mixer.
h.app.mixer = 1;
h = runUntil(h, 0.1, []);
assert(h.m.doMixer == 1);
h.app.estop = 1;
h = runUntil(h, 0.1, []);
assert(h.m.doSlave(1) == 0 && h.m.doMixer == 0 && h.m.statusCode == 20);
h.app.estop = 0;
h = runUntil(h, 0.5, []);
assert(h.m.doSlave(1) == 1 && h.m.state == 1);
fprintf('ok  e-stop\n');

% Displayed volume: dead volume at the end of travel, bore area per mm.
assert(abs(x2mL(cfg.posEOT, cfg) - cfg.deadVolume) < 1e-9);
assert(abs(x2mL(cfg.posEOT - 100, cfg) - cfg.deadVolume ...
           - 100 * pi / 4 * (2.602 * 25.4)^2 / 1000) < 1e-6);
% Pressure: 4 mA reads the range minimum plus one atmosphere (gauge),
% 20 mA the range maximum plus one atmosphere.
assert(abs(V2P(0.004 * cfg.pressureShuntOhms, cfg) - ...
           (cfg.pressureRangeMin * cfg.atmPerPsi + cfg.pressureIsGauge)) < 1e-9);
assert(abs(V2P(0.020 * cfg.pressureShuntOhms, cfg) - ...
           (cfg.pressureRangeMax * cfg.atmPerPsi + cfg.pressureIsGauge)) < 1e-9);
fprintf('ok  pressure scaling\n');
fprintf('ok  volume scaling (%.3f mL/mm)\n', mLPerMm(cfg));

fprintf('all passed\n');
end


function h = command(h, op)
h.app.op = op;
h.app.cmdSeq = h.app.cmdSeq + 1;
end


function h = runUntil(h, T, done)
for n = 1:round(T * 1000)
    h.tick = h.tick + 1;

    if mod(h.tick, 3) == 0
        % Slave outputs sink the DAQ inputs: asserted reads 0.
        y = h.y;
        di = 1 - [y.confBit0, y.confBit1, y.confBit2, y.performanceStatus, ...
                  y.faultActive, y.homingComplete, y.inMotion];
        a = h.app;
        [m.doSlave, m.doMixer, m.refVoltage, m.position, m.velocity, m.pressure, ...
            m.statusCode, m.faultCode, m.state] = masterStep(di, ...
            y.positionVolt, h.pressureVolt, a.op, a.cmdSeq, a.targetPosition, ...
            a.targetVelocity, a.jogDirection, a.targetPressure, a.estop, a.mixer);
        h.m = m;
        % DAQ output order: [safety modeBit1 motionEnable modeBit0 modeBit2 faultReset]
        h.u.modeBit1     = m.doSlave(2);
        h.u.motionEnable = m.doSlave(3);
        h.u.modeBit0     = m.doSlave(4);
        h.u.modeBit2     = m.doSlave(5);
        h.u.faultReset   = m.doSlave(6);
        h.u.refVoltage   = m.refVoltage;
    end

    if mod(h.tick, 2) == 0
        [h.sl, h.y] = slaveStep(h.sl, h.u);
    end

    if ~isempty(done) && ~isempty(h.m) && done(h)
        return;
    end
end
end
