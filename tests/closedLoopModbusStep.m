function closedLoopModbusStep()
%CLOSEDLOOPMODBUSSTEP Run masterStep (the Simulink block's entry point)
%   against the simulated slave through the Modbus registers and the link
%   timing model (modbusLink, modbusLinkConfig: the bench's measurements):
%   the master steps every cfg.dt; every cfg.exchangePeriod it reads the
%   slave's input registers, steps on what it read and writes its holding
%   registers, as 16-bit words packed and unpacked by the slave's register
%   map (the GVL's AT addresses); and the slave scans every G_cfgScanTime
%   (4 ms), serving the link's requests around its scans, on a 1 ms tick.
%   The master runs as the Simulink model does (the model commands of
%   modbusLink): the Read and Write calls block its step, so a slow
%   exchange delays the steps behind it, an exchange now and then arrives a
%   period late, and a call that fails on a lost link stops the model, with
%   the DAQ outputs at their final values. Checks that the master's
%   register map (modbusMap) agrees with the slave's, then covers homing,
%   position, pressure, fault reset, link loss on either side, a late
%   exchange and the model stopping on a lost link, and the slave's
%   per-exchange summary (IR21-34): its windows follow the master's
%   heartbeat, are the mean, minimum and maximum of the scans' own inputs,
%   survive link loss and the length cap, and reach the master intact.
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
    'positionSpeed', cfg.speedMax);
h.y = scan(h.sl);
h.mbRead = zeros(1, map.nRead);
% The Modbus link with the bench's timing (modbusLinkConfig)
h.link = modbusLink('init', modbusLinkConfig(), h.sl.G.G_cfgScanTime);
h.link = modbusLink('modelInit', h.link, cfg.dt, cfg.exchangePeriod, map.nRead);
h.stopped = false;      % the Simulink model has stopped on a lost link
h.tStop = NaN;
h.nStep = 0;            % masterStep calls
h.log = struct('read', zeros(0, 2), 'written', zeros(1, 0));
h.tick = 0;
h.m = [];
h.writeLost = false;    % master's registers do not reach the slave
h.logScans = false;     % record the exchange summary's inputs and outputs every scan
h.scanLog = zeros(0, 19);
h.inputFcn = [];        % @(h) volts for the slave's analog input before each scan
h.log.words = zeros(0, map.nRead);
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
         'ir', 'homingComplete', 'G_mbHomingComplete';
         'ir', 'pressureBlock', 'G_mbPressureBlock'; 'ir', 'atEndOfTravel', 'G_mbAtEndOfTravel';
         'ir', 'position', 'G_mbActualPosition';
         'ir', 'velocity', 'G_mbActualVelocity'; 'ir', 'torque', 'G_mbActualTorque';
         'ir', 'commandedPosition', 'G_mbCommandedPosition';
         'ir', 'pressure', 'G_mbActualPressure';
         'ir', 'pressureMean', 'G_mbSumPressureMean'; 'ir', 'pressureMin', 'G_mbSumPressureMin';
         'ir', 'pressureMax', 'G_mbSumPressureMax'; 'ir', 'velocityMean', 'G_mbSumVelocityMean';
         'ir', 'torqueMean', 'G_mbSumTorqueMean'; 'ir', 'positionMean', 'G_mbSumPositionMean';
         'ir', 'summaryScans', 'G_mbSumScans'; 'ir', 'summarySeq', 'G_mbSumSeq'};
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
assert(abs(h.sl.G.G_mbPositionSpeed - cfg.speedMax) < 1e-6);
h.app.positionSpeed = 2;
h.app.targetPosition = 25;
h = command(h, 2);
% (The new speed reaches the slave an exchange or more after the command,
% so the condition waits for it as well as for the motion.)
h = runUntil(h, 30, @(h) abs(h.sl.G.G_mbPositionSpeed - 2) < 1e-6 && abs(h.sl.ax.v) > 1.9);
nEx0 = rows(h.log.words);
h.logScans = true;
h.scanLog = zeros(0, 19);
h = runUntil(h, 0.3, []);
assert(abs(h.sl.G.G_mbPositionSpeed - 2) < 1e-6 && abs(h.sl.ax.v - 2) < 0.1);
h.app.positionSpeed = 0;
h = runUntil(h, 3, []);
h.logScans = false;
xHold = h.sl.ax.x;
h = runUntil(h, 1, []);
assert(h.sl.G.G_mbPositionSpeed == 0 && abs(h.sl.ax.v) < 0.01 && abs(h.sl.ax.x - xHold) < 0.01);
assert(h.y.faultActive == 0 && h.m.statusCode == 2 && xHold < 24);
h.app.positionSpeed = 100;
h = runUntil(h, 30, @(h) h.m.statusCode == 3);
assert(h.m.statusCode == 3 && abs(h.sl.G.G_mbPositionSpeed - cfg.speedMax) < 1e-6);
h.app.positionSpeed = cfg.speedMax;
fprintf('ok  position speed through HR7-8: 2 mm/s, 0 holds at %.2f mm, 100 limited to %g mm/s\n', ...
    xHold, h.sl.G.G_mbPositionSpeed);

% The summary through the move at 2 mm/s and its stop: the windows are the
% scans' own means, and the velocity mean follows the stop.
Sm = summaryWindows(h.scanLog, 65535);
cr = checkReads(h, Sm, h.log.words(nEx0 + 1:end, :), 0);
w = Sm.win;
assert(w(2).velocityMean > 1.9 && min([w.velocityMean]) < 0.5 && any(diff([w.positionMean]) > 0.05), ...
    'the summary did not follow the move');
fprintf('ok  summary through a move at 2 mm/s and its stop: %d windows, velocity mean %.2f to %.2f mm/s, Scans %d to %d\n', ...
    numel(w), max([w.velocityMean]), min([w.velocityMean]), Sm.scansMin, Sm.scansMax);

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

% A late exchange: the exchange due at one step arrives a period later
% (the bench sees one a minute at 50 ms). The read results have one gap of
% two periods, the master rides it out in a jog, and the slave's 300 ms
% heartbeat timeout is not reached.
h.app.targetVelocity = 2;
h.app.jogDirection = 1;
h = command(h, 3);
h = runUntil(h, 10, @(h) h.m.statusCode == 4);
h = runUntil(h, 2, @(h) h.sl.ax.v > 1);
assert(h.m.statusCode == 4 && h.sl.ax.v > 1);
st0 = modbusLink('modelStats', h.link);
h.link = modbusLink('late', h.link);
h = runUntil(h, 1, []);
st = modbusLink('modelStats', h.link);
gap = diff(st.tExchange(numel(st0.tExchange):end));
assert(st.late == st0.late + 1, 'late exchanges: %d', st.late - st0.late);
assert(sum(gap > 1.7 * cfg.exchangePeriod) == 1 && max(gap) < 2.5 * cfg.exchangePeriod, ...
    'gaps between read results (ms): %s', mat2str(round(1000 * gap)));
assert(numel(gap) >= 18 && numel(gap) <= 19, ...
    '%d exchanges in 1 s, 19 or 20 expected less the merged one', numel(gap));
assert(h.y.faultActive == 0 && h.m.linkOk == 1 && h.m.statusCode == 4 && h.sl.ax.v > 1);
fprintf('ok  late exchange: gap of %.0f ms between read results, nominal %.0f ms; jog and link unaffected\n', ...
    1000 * max(gap), 1000 * cfg.exchangePeriod);
h.app.op = 0;
h = runUntil(h, 0.5, []);
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);

% The model stops on a lost link: the connection drops during a jog, the
% next Modbus call (the next exchange step's Read) fails, the model ends (no more masterStep calls) and
% the DAQ outputs fall to their final values, safety relay off. The slave
% sees the heartbeat stop, raises FAULT_COMM and stops the axis. A restarted
% model finds the fault and the operator resets it.
h.app.targetVelocity = 2;
h.app.jogDirection = 1;
h = command(h, 3);
h = runUntil(h, 10, @(h) h.m.statusCode == 4);
h = runUntil(h, 2, @(h) h.sl.ax.v > 1);
assert(h.m.statusCode == 4 && h.sl.ax.v > 1 && ~h.stopped);
h = runUntil(h, 1, @(h) strcmp(h.link.model.x, 'none'));     % between two calls
tCut = h.tick / 1000;
h.link = modbusLink('cut', h.link, tCut);
h = runUntil(h, 1, @(h) h.stopped);
assert(h.stopped, 'the model did not stop on the lost link');
assert(h.tStop > tCut && h.tStop - tCut <= cfg.exchangePeriod + 1e-9, 'the model stopped %.0f ms after the cut', ...
    1000 * (h.tStop - tCut));
nStep = h.nStep;
h = runUntil(h, 0.3, []);
assert(h.nStep == nStep, 'masterStep ran after the model stopped');
assert(h.m.doSafety == 0 && h.m.doMixer == 0, 'DAQ outputs not at their final values');
assert(h.y.faultActive == 1 && h.y.faultCode == 4, 'slave did not raise FAULT_COMM');
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.v == 0, 'axis did not stop after the model stopped');
fprintf('ok  lost link: model stopped %.0f ms after the cut, outputs off, slave FAULT_COMM and stopped\n', ...
    1000 * (h.tStop - tCut));
h.link = modbusLink('restore', h.link);
h.app.op = 0;
h = restartModel(h);
h = runUntil(h, 0.5, []);
assert(~h.stopped && h.m.linkOk == 1 && h.y.faultActive == 1);
h = command(h, 5);
h = runUntil(h, 1.5, []);
assert(h.y.faultActive == 0 && h.m.state == 1 && h.m.linkOk == 1 && h.m.doSafety == 1);
fprintf('ok  restarted model resets the fault\n');

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

% Exchange summary (IR21-34), windows follow the heartbeat: the axis rests
% in the chamber at 1 MPa and a 15 Hz tone of 0.1 MPa rides on the
% transducer's voltage, so the pressure moves within a window and the
% snapshot in IR19-20 (one scan in twelve) aliases the tone to 5 Hz. With
% no late exchange, each master write closes one window: 12.5 scans on
% average, 5 to 20 with the read call's jitter (up to 25 ms) in the write time.
h.sl.ax.x = chamberPosition(h.sl, 1);
h.link.cfg.lateExchangeRate = 0;
tone = 0.1;                                     % [MPa]
h.inputFcn = @(h) pressureVolts(h.sl, h.sl.ax.P + tone * sin(2 * pi * 15 * h.tick / 1000));
h = runUntil(h, 1, []);
nEx0 = rows(h.log.words);
h.logScans = true;
h.scanLog = zeros(0, 19);
h = runUntil(h, 3, []);
h.logScans = false;
Sm = summaryWindows(h.scanLog, 65535);
assert(Sm.nBoundary >= 55 && Sm.scansMin >= 5 && Sm.scansMax <= 20 && ...
    abs(mean([Sm.win.scans]) - 12.5) < 0.5, '%d windows of %d to %d scans, mean %.2f', ...
    Sm.nBoundary, Sm.scansMin, Sm.scansMax, mean([Sm.win.scans]));
cr = checkReads(h, Sm, h.log.words(nEx0 + 1:end, :), 0.5);
assert(cr.scansMin >= 5 && cr.scansMax <= 20);
fprintf('ok  summary windows follow the heartbeat: %d windows, Scans %d to %d (mean %.2f; reads %d to %d), Seq +1 per read over %d reads\n', ...
    Sm.nBoundary, Sm.scansMin, Sm.scansMax, mean([Sm.win.scans]), cr.scansMin, cr.scansMax, cr.nRead);
assert(cr.stdMean < 0.5 * cr.stdSnap && cr.minRange >= 0.5 * cr.ptpLogged, ...
    'window means std %.4f vs snapshots %.4f MPa; window range %.3f, logged pk-pk %.3f', ...
    cr.stdMean, cr.stdSnap, cr.minRange, cr.ptpLogged);
fprintf('ok  summary averages the tone: std of window means %.4f MPa against %.4f MPa of the snapshots (ratio %.2f), smallest window range %.3f of %.3f MPa pk-pk\n', ...
    cr.stdMean, cr.stdSnap, cr.stdMean / cr.stdSnap, cr.minRange, cr.ptpLogged);
h.inputFcn = [];
h.sl.hook.pressureVolt = NaN;

% Link loss: with no write the heartbeat is frozen and the window stays
% open, to be closed by the first write after the link returns. With the
% length cap at 25 scans the windows close without a write.
nEx0 = rows(h.log.words);
h.logScans = true;
h.scanLog = zeros(0, 19);
h = runUntil(h, 0.3, []);
h.writeLost = true;
h = runUntil(h, 0.5, []);
h.writeLost = false;
h = runUntil(h, 0.3, []);
h.logScans = false;
Sm = summaryWindows(h.scanLog, 65535);
assert(Sm.scansMax >= 125 && sum([Sm.win.scans] >= 125) == 1, ...
    'longest window %d scans after 0.5 s of lost writes', Sm.scansMax);
fprintf('ok  summary through a write loss: one window of %d scans, Seq repeats, closed by the first write after it\n', Sm.scansMax);

h.sl.s.fbExchangeSummary.cMaxScans = 25;
h.logScans = true;
h.scanLog = zeros(0, 19);
h = runUntil(h, 0.3, []);
h.writeLost = true;
h = runUntil(h, 0.4, []);
h.writeLost = false;
h = runUntil(h, 0.3, []);
h.logScans = false;
h.sl.s.fbExchangeSummary.cMaxScans = 65535;
Sm = summaryWindows(h.scanLog, 25);
lost = Sm.win(Sm.winTick > Sm.scanTick(1) + 300 & Sm.winTick <= Sm.scanTick(1) + 700);
assert(all([lost.scans] == 25) && numel(lost) >= 3 && numel(lost) <= 5, ...
    '%d windows in the lost-write stretch, Scans %s', numel(lost), mat2str([lost.scans]));
fprintf('ok  summary length cap: %d windows of %d scans without a write, Seq +%d\n', ...
    numel(lost), lost(1).scans, numel(lost));

% Swept volume: none at the end of travel, bore area per mm.
assert(abs(sweptVolume(cfg.posEOT, cfg)) < 1e-9);
assert(abs(sweptVolume(cfg.posEOT - 100, cfg) ...
           - 100 * pi / 4 * (2.602 * 25.4)^2 / 1000) < 1e-6);
fprintf('ok  volume scaling (%.3f mL/mm)\n', mLPerMm(cfg));

fprintf('all passed\n');
end


function row = scanRow(sl, tick, pre)
% One scan's log of the exchange summary: the block's inputs, the globals
% they come from (pre: velocity, torque and position as the scan found
% them), the pressure register and the summary registers.
fb = sl.s.fbExchangeSummary;
row = [fb.Heartbeat, fb.Pressure, fb.Velocity, fb.Torque, fb.Position, ...
    sl.G.G_sysActualPressure, pre, sl.G.G_mbActualPressure, ...
    sl.G.G_mbSumPressureMean, sl.G.G_mbSumPressureMin, sl.G.G_mbSumPressureMax, ...
    sl.G.G_mbSumVelocityMean, sl.G.G_mbSumTorqueMean, sl.G.G_mbSumPositionMean, ...
    sl.G.G_mbSumScans, sl.G.G_mbSumSeq, tick];
end


function v = pressureVolts(sl, P)
% The analog input voltage of the transducer at gauge pressure P (MPa): the
% simulator's 4-20 mA loop over its range, through the shunt.
mA = sl.sim.sensorZeroMa + sl.sim.sensorSpanMa * P / sl.sim.sensorRangeMPa;
v = mA * sl.sim.sensorShuntOhm / 1000;
end


function S = summaryWindows(L, cap)
% Check the slave's summary against its own log of inputs, scan by scan
% (columns of scanRow). A window is closed by the scan whose heartbeat
% differs from the previous scan's, or when it holds cap scans: Seq then
% advances by one in that scan and the published values are the mean,
% minimum and maximum of the window's scans, as single (the registers are
% REAL). Between closing scans the registers do not change. The window
% that was open when the log began is not checked, as its early scans are
% not in the log. Returns S.win, one element per checked window (scans,
% the means, tick of its closing scan), the counts and extremes.
hb = L(:, 1); p = L(:, 2); v = L(:, 3); q = L(:, 4); x = L(:, 5);
n = rows(L);
% The block's inputs are the globals it is wired to, and the pressure
% register is the same value as single
assert(isequal(L(:, 2:5), L(:, 6:9)), 'block inputs differ from the globals in columns %s, max %g', mat2str(find(any(L(:, 2:5) ~= L(:, 6:9)))), max(max(abs(L(:, 2:5) - L(:, 6:9)))));
assert(isequal(L(:, 10), double(single(p))), 'IR19-20 differs from the block''s pressure');
seq = L(:, 18);
dSeq = mod(diff(seq), 65536);
first = find(dSeq > 0, 1) + 1;          % first closing scan in the log
assert(~isempty(first), 'no window closed in the log');
assert(all(diff(hb(1:first - 1)) == 0), 'the heartbeat changed without a window closing');
reg = L(:, 11:18);
changed = [false; any(diff(reg) ~= 0, 2)];
count = 0;                              % scans in the open window
S.win = struct('scans', {}, 'pressureMean', {}, 'velocityMean', {}, 'positionMean', {}, 'tick', {});
open = first;                           % first scan of the open window
nClose = 0;
for i = first:n
    closes = (i > first && (hb(i) ~= hb(i - 1) || count >= cap)) || i == first;
    if i == first
        assert(dSeq(i - 1) == 1, 'Seq advanced by %d', dSeq(i - 1));
    else
        assert(dSeq(i - 1) == closes, 'scan %d: Seq advanced by %d, expected %d', i, dSeq(i - 1), closes);
        assert(changed(i) == closes, 'scan %d: registers %s', i, ternary(changed(i), 'changed', 'held'));
    end
    if closes && i > first
        k = open:i - 1;
        e = [mean(p(k)), min(p(k)), max(p(k)), mean(v(k)), mean(q(k)), mean(x(k))];
        got = reg(i, 1:6);
        assert(all(abs(got - double(single(e))) <= 2 * eps(single(e)) + 1e-12), ...
            'scan %d: published %s, window %s', i, mat2str(got, 7), mat2str(e, 7));
        assert(reg(i, 7) == numel(k), 'scan %d: Scans %d for a window of %d', i, reg(i, 7), numel(k));
        S.win(end + 1) = struct('scans', numel(k), 'pressureMean', got(1), ...
            'velocityMean', got(4), 'positionMean', got(6), 'tick', L(i, 19));
        nClose = nClose + 1;
    end
    if closes
        open = i;
        count = 0;
    end
    count = count + 1;
end
S.nBoundary = nClose;
S.scansMin = min([S.win.scans]);
S.scansMax = max([S.win.scans]);
S.winTick = [S.win.tick];
S.scanTick = L(:, 19);
S.log = L;
end


function r = checkReads(h, S, W, tSkip)
% Check the master's reads (the rows of W, one per exchange, as the Read
% block returned them) against the slave's summary. Each read carries the
% summary registers of the scan it was served before: Seq is one more than
% the read before (no write was lost, no exchange late), IR21-33 equal the
% slave's registers of that Seq in the scan log, and Scans is in range. A
% read before the log began is skipped. Then, from tSkip seconds on and for
% windows of at least 10 scans: min < mean < max, and the means vary less
% than the snapshots (IR19-20) do while the window range keeps a good part
% of the logged pressure swing.
map = h.map;
L = S.log;
ok = ~any(isnan(W), 2);
W = W(ok, :);
seq = W(:, map.ir.summarySeq);
inLog = ismember(seq, L(:, 18));
W = W(inLog, :);
seq = seq(inLog);
assert(rows(W) > 5, 'only %d reads fall in the log', rows(W));
assert(all(mod(diff(seq), 65536) == 1), 'Seq between reads: %s', mat2str(mod(diff(seq), 65536)'));
for j = 1:rows(W)
    i = find(L(:, 18) == seq(j), 1);
    u = @(k) modbusUnpack(W(j, k), W(j, k + 1));
    got = [u(map.ir.pressureMean), u(map.ir.pressureMin), u(map.ir.pressureMax), ...
        u(map.ir.velocityMean), u(map.ir.torqueMean), u(map.ir.positionMean), ...
        W(j, map.ir.summaryScans)];
    assert(isequal(got, L(i, 11:17)), 'read of Seq %d differs from the slave''s registers', seq(j));
end
r.nRead = rows(W);
r.scansMin = min(W(:, map.ir.summaryScans));
r.scansMax = max(W(:, map.ir.summaryScans));
% Dynamics, once the window is a full one
tRead = L(arrayfun(@(s) find(L(:, 18) == s, 1), seq), 19) / 1000;
sel = tRead >= L(1, 19) / 1000 + tSkip & W(:, map.ir.summaryScans) >= 10;
pm = arrayfun(@(j) modbusUnpack(W(j, map.ir.pressureMean), W(j, map.ir.pressureMean + 1)), find(sel));
pn = arrayfun(@(j) modbusUnpack(W(j, map.ir.pressureMin), W(j, map.ir.pressureMin + 1)), find(sel));
px = arrayfun(@(j) modbusUnpack(W(j, map.ir.pressureMax), W(j, map.ir.pressureMax + 1)), find(sel));
ps = arrayfun(@(j) modbusUnpack(W(j, map.ir.pressure), W(j, map.ir.pressure + 1)), find(sel));
r.stdMean = std(pm);
r.stdSnap = std(ps);
r.ptpLogged = max(L(:, 2)) - min(L(:, 2));
r.minRange = min(px - pn);
if tSkip > 0
    assert(all(pn < pm & pm < px), 'min < mean < max fails');
end
end


function s = ternary(c, a, b)
if c
    s = a;
else
    s = b;
end
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
% On a 1 ms tick: the Simulink model steps as modbusLink's model commands
% say (the master steps every cfg.dt; at an exchange step the Read block
% returns the input registers through the link, masterStep runs and the
% Write block sends the holding registers, each call blocking the model).
% The slave scans every G_cfgScanTime and serves the link's requests
% around each scan: reads before it, writes after it. A stopped model
% steps no more and its DAQ outputs hold their final values.
scanTicks = round(1000 * h.sl.G.G_cfgScanTime);
for n = 1:round(T * 1000)
    h.tick = h.tick + 1;
    t = h.tick / 1000;

    ev.kind = '';
    while ~strcmp(ev.kind, 'none')
        [h.link, ev] = modbusLink('modelPoll', h.link, t);
        switch ev.kind
            case 'step'
                h = stepMaster(h);
            case 'exchange'
                if h.readLost || ev.failed
                    h.log.read(end + 1, :) = [NaN, NaN];
                    h.log.words(end + 1, :) = NaN;
                else
                    h.mbRead = ev.data;
                    h.log.read(end + 1, :) = [ev.dur, ev.data(h.map.ir.heartbeatEcho)];
                    h.log.words(end + 1, :) = ev.data;
                end
                h = stepMaster(h);
                words = [];
                if ~h.writeLost
                    words = h.m.mbWrite;
                    h.log.written(end + 1) = words(h.map.hr.heartbeat);
                else
                    h.log.written(end + 1) = NaN;
                end
                h.link = modbusLink('modelWrite', h.link, t, words);
            case 'stopped'
                h.stopped = true;
                h.tStop = ev.t;
                if ~isempty(h.m)
                    h.m.doSafety = 0;       % the DAQ outputs' final values
                    h.m.doMixer = 0;
                end
        end
    end

    if mod(h.tick, scanTicks) == 0
        k = h.tick / scanTicks;
        [h.link, reqs] = modbusLink('due', h.link, k, 'read');
        for q = reqs
            h.link = modbusLink('answer', h.link, q.slot, readInputRegisters(h.sl, q.start, q.n));
        end
        if ~isempty(h.inputFcn)
            h.sl.hook.pressureVolt = h.inputFcn(h);
        end
        if h.logScans
            % (the axis model sets these after a scan, for the next one)
            pre = [h.sl.G.G_sysActualVelocity, h.sl.G.G_sysActualTorque, h.sl.G.G_sysActualPosition];
        end
        h.y = scan(h.sl);
        if h.logScans
            h.scanLog(end + 1, :) = scanRow(h.sl, h.tick, pre);
        end
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


function h = restartModel(h)
% A new run of the model: the master's state is cleared, the link's model
% starts from now.
clear masterStep masterStateMachine
h.link = modbusLink('modelInit', h.link, h.cfg.dt, h.cfg.exchangePeriod, h.map.nRead, h.tick / 1000);
h.stopped = false;
h.tStop = NaN;
h.m = [];
h.mbRead = zeros(1, h.map.nRead);
end


function h = stepMaster(h)
a = h.app;
[m.mbWrite, m.doSafety, m.doMixer, m.position, m.velocity, m.pressure, ...
    m.statusCode, m.faultCode, m.state, m.linkOk] = masterStep(h.mbRead, ...
    a.op, a.cmdSeq, a.targetPosition, ...
    a.targetVelocity, a.jogDirection, a.targetPressure, a.estop, a.mixer, ...
    a.positionSpeed);
h.m = m;
h.nStep = h.nStep + 1;
end
