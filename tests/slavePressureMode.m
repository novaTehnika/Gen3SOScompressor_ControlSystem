function slavePressureMode()
%SLAVEPRESSUREMODE Run the simulated slave (slave-sim, generated from
%   slave-src) in pressure mode through its holding registers, standing in
%   for the master: handshake into mode 7, hold and step the target, ride
%   through an outflow disturbance, clamp an excessive target, leave the
%   mode, fault on a broken transducer loop only while the loop is
%   closed on it and, with the stop-law floor (G_cfgPressureFloorEnable),
%   latch on outflow and release at a closing. It ends with PRG_Main's scan
%   order: a step at the transducer reaches the velocity command behind the
%   median alone, and a drive fault commands the stop in the scan the fault
%   state is entered.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath slave-sim slave-sim/generated tests
%   Depends on: slave-src slave-sim tools models

MODE_PRESSURE = 7;
FAULT_PRESSURE_SENSOR = 8;

h = harnessInit();
E = h.sl.E;
G = h.sl.G;
h.sl.ax.x = h.sl.sim.chamberContactPos - 5;      % just short of the chamber

% Loop open at the sensor while idle: no fault.
h.sl.hook.pressureVolt = 0;
h = runFor(h, 1);
assert(h.y.faultActive == 0 && h.y.actualPressure < 0);
h.sl.hook.pressureVolt = NaN;
fprintf('ok  broken loop while idle does not fault\n');

% The simulated chamber is the plant model (models/), and the criteria
% are the design studies': a compression ends within the status band, and
% the overshoot is judged only against the pressure ceiling.
P_CEILING = 12;                         % MPa, StudyParams.P_ceiling
settled = @(h) h.y.performanceStatus == 1 && abs(h.sl.ax.v) < 0.05;

% Enter pressure mode with a 10 MPa target: compresses within the velocity
% clamp and ends in band, below the ceiling. The axis starts in the
% chamber at 0.5 MPa.
h.sl.ax.x = chamberPosition(h.sl, 0.5);
h = enterMode(h, MODE_PRESSURE, 10);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_PRESSURE_CTRL);
h.vMax = 0; h.pMax = 0;
t0 = h.t;
h = runUntil(h, 30, settled);
assert(settled(h), 'pressure did not settle in the target band');
t1 = h.t - t0;
h = runFor(h, 2);
assert(abs(h.sl.ax.P - 10) <= G.G_cfgPressureInBand && h.y.performanceStatus == 1);
assert(h.vMax <= G.G_cfgPressureVelMax + 0.01, 'velocity %.3f mm/s past the clamp', h.vMax);
assert(h.pMax < P_CEILING, 'peak %.3f MPa', h.pMax);
assert(abs(h.y.actualPressure - h.sl.ax.P) < 0.05);
fprintf('ok  compress to 10 MPa (settled in %.1f s, peak %.2f mm/s, max %.3f MPa)\n', ...
    t1, h.vMax, h.pMax);

% Target step to 11 MPa.
h.u.reference = 11;
h.pMax = 0;
h = runFor(h, 0.5);
h = runUntil(h, 20, settled);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - 11) <= G.G_cfgPressureInBand && h.pMax < P_CEILING, ...
    'at %.3f MPa, peak %.3f MPa', h.sl.ax.P, h.pMax);
fprintf('ok  step to 11 MPa (max %.3f MPa)\n', h.pMax);

% Outflow disturbance: the loop holds the pressure up while liquid leaves
% and recovers once it stops.
h.sl.hook.outflow = 300;                 % mm^3/s
h.pMin = inf;
h = runFor(h, 3);
assert(h.pMin > 10 && h.sl.ax.v > 0, 'loop did not follow the outflow');
h.sl.hook.outflow = 0;
h = runFor(h, 0.5);
h = runUntil(h, 20, settled);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - 11) <= G.G_cfgPressureInBand, 'at %.3f MPa', h.sl.ax.P);
fprintf('ok  outflow disturbance (dip to %.3f MPa)\n', h.pMin);

% A target above the effective maximum, the lower of the configured
% maximum and the pressure ceiling less its hysteresis, is clamped to it.
% The ceiling's block never engages.
tMax = min(G.G_cfgPressureTargetMax, ...
    G.G_cfgPressureCeiling - G.G_cfgPressureCeilingHyst);
h.u.reference = 20;
h.pMax = 0;
h = runFor(h, 0.5);
h = runUntil(h, 20, settled);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - tMax) <= G.G_cfgPressureInBand, 'at %.3f MPa', h.sl.ax.P);
assert(h.pMax < G.G_cfgPressureCeiling && h.y.pressureBlock == 0, ...
    'peak %.3f MPa, block %d', h.pMax, h.y.pressureBlock);
fprintf('ok  target clamped to %.1f MPa (ceiling %.0f MPa not reached)\n', ...
    tMax, G.G_cfgPressureCeiling);

% Dropping MotionEnable holds, then mode 0 goes to idle with no fault.
h.u.motionEnable = 0;
h = runFor(h, 1);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_HOLD_POSITION && h.sl.ax.v == 0);
h.u.modeCode = 0;
h.u.reference = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE && h.faultSeen == 0);
assert(h.y.performanceStatus == 0);
fprintf('ok  leave pressure mode\n');

% Master heartbeat frozen while the loop is closed: FAULT_COMM, as for
% any commanded motion, and the axis stops.
h.sl.ax.x = h.sl.sim.chamberContactPos + 10;
h = enterMode(h, MODE_PRESSURE, 10);
h = runFor(h, 1);
assert(h.sl.ax.v > 0.5);
h.freezeHeartbeat = true;
h = runUntil(h, 1, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == 4, 'no comm fault');
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.v == 0);
h.freezeHeartbeat = false;
h.u.motionEnable = 0;
h.u.modeCode = 4;
h = runFor(h, 0.2);
h.u.faultReset = 1;
h = runUntil(h, 2, @(h) h.y.faultActive == 0);
assert(h.y.faultActive == 0, 'comm fault did not clear');
h.u.faultReset = 0;
h.u.modeCode = 0;
h = runFor(h, 1);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_BRAKE_HOLD);
fprintf('ok  link loss in pressure mode -> FAULT_COMM, reset\n');

% Broken loop while the loop is closed: FAULT_PRESSURE_SENSOR, axis stops,
% and the register reset handshake clears it.
h.sl.ax.x = h.sl.sim.chamberContactPos + 10;
h = enterMode(h, MODE_PRESSURE, 10);
h = runFor(h, 1);
assert(h.sl.ax.v > 0.5);
h.sl.hook.pressureVolt = 0;
h = runUntil(h, 1, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PRESSURE_SENSOR, 'no sensor fault');
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_FAULT);
h = runUntil(h, 3, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.v == 0);
h.sl.hook.pressureVolt = NaN;
h.u.motionEnable = 0;
h.u.modeCode = FAULT_PRESSURE_SENSOR;    % the master mirrors the full code
h = runFor(h, 0.2);
h.u.faultReset = 1;
h = runUntil(h, 2, @(h) h.y.faultActive == 0);
assert(h.y.faultActive == 0 && h.y.faultCode == 0, 'fault did not clear');
h.u.faultReset = 0;
h.u.modeCode = 0;
h = runFor(h, 1);
% A fast reset leaves the slave in brake hold with the drive on, awaiting
% the next mode request.
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_BRAKE_HOLD && h.sl.ax.v == 0);
assert(h.y.faultActive == 0 && h.y.brakeDisengage == 0);
fprintf('ok  sensor fault in pressure mode, mirror and reset\n');

% Stop-law floor (G_cfgPressureFloorEnable): at the target with the outlet
% shut the floor is G_cfgPressureFloorVelLow; an outflow the piston must
% follow (4.8 mL/s, the orifice bank at 10 MPa) latches it at
% G_cfgPressureStopVelMin; shutting the outlet abruptly under the moving
% piston releases it, and the forward stop holds the pressure down. With
% the floor off the same closing peaks higher.
peak = zeros(1, 2);
for on = [true false]
    h.sl.G.G_cfgPressureFloorEnable = on;
    h.sl.ax.x = chamberPosition(h.sl, 0.5);
    h = enterMode(h, MODE_PRESSURE, 10);
    h = runUntil(h, 30, settled);
    h = runFor(h, 2);
    pc = h.sl.s.fbPressureCtrl;
    assert(pc.FloorEnable == on && ~pc.FloorError && ~pc.Latched);
    if on
        assert(pc.rFloor == h.sl.G.G_cfgPressureFloorVelLow);
    else
        assert(pc.rFloor == h.sl.G.G_cfgPressureStopVelMin);
    end
    h.sl.hook.outflow = 4800;            % mm^3/s
    h = runFor(h, 4);
    if on
        pc = h.sl.s.fbPressureCtrl;
        assert(pc.Latched && pc.rFloor == h.sl.G.G_cfgPressureStopVelMin, 'outflow not latched');
    end
    assert(abs(h.sl.ax.P - 10) < 1.5 && h.sl.ax.v > 1, 'loop did not follow the outflow');
    h.sl.hook.outflow = 0;               % the outlet shuts under the moving piston
    h.pMax = 0;
    h = runFor(h, 3);
    peak(2 - on) = h.pMax;
    if on
        assert(~h.sl.s.fbPressureCtrl.Latched, 'latch not released at the closing');
    end
    h.u.motionEnable = 0;
    h = runFor(h, 1);
    h.u.modeCode = 0;
    h.u.reference = 0;
    h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
end
h.sl.G.G_cfgPressureFloorEnable = false;
assert(peak(1) < peak(2) && peak(1) < 10.6, 'closing peak %.2f MPa with the floor, %.2f without', ...
    peak(1), peak(2));
fprintf('ok  stop-law floor: latched on outflow, released at the closing, peak %.2f MPa (%.2f without)\n', ...
    peak(1), peak(2));

% Scan order (PRG_Main's phases). The reading is conditioned, the loop
% run and the command written in one scan, so a step at the transducer
% moves the velocity command as soon as the median of five passes it:
% in the third scan that samples the step, two scans after the first.
% Any scan between the reading and the command would add to that.
h.sl.ax.x = chamberPosition(h.sl, 0.5);
h = enterMode(h, MODE_PRESSURE, 5);
h = runUntil(h, 30, settled);
h = runFor(h, 2);
dt = h.sl.G.G_cfgScanTime;
voltsAt = @(P) (G.G_cfgPressureZeroMa + G.G_cfgPressureSpanMa * ...
    (P - G.G_cfgPressureRangeMin) / (G.G_cfgPressureRangeMax - G.G_cfgPressureRangeMin)) ...
    * G.G_cfgPressureShuntOhms / 1000;
h.sl.hook.pressureVolt = voltsAt(5);            % hold the reading at the target
h = runFor(h, 1);
cmd0 = h.sl.G.G_cmdDirectControl.Velocity;       % steady: the error is near zero
h = runFor(h, dt);
assert(abs(h.sl.G.G_cmdDirectControl.Velocity - cmd0) < 1e-3, 'command not steady');
cmd0 = h.sl.G.G_cmdDirectControl.Velocity;
h.sl.hook.pressureVolt = voltsAt(4);            % 1 MPa below the target
first = 0;
for k = 0:9
    h = runFor(h, dt);
    if first == 0 && abs(h.sl.G.G_cmdDirectControl.Velocity - cmd0) > 0.05
        first = k;
    end
end
cmd1 = h.sl.G.G_cmdDirectControl.Velocity;
assert(first == 2, 'the command moved %d scans after the step, expected 2', first);
assert(cmd1 > cmd0 + 0.2, 'command %.3f mm/s after the step', cmd1);
h.sl.hook.pressureVolt = NaN;
fprintf(['ok  scan order: a transducer step reaches the velocity command %d scans later ' ...
    '(the median''s delay alone)\n'], first);

% A drive fault raised by the axis in one scan is seen by the safety
% monitor at the top of the next, which enters ST_FAULT before the state
% machine runs: the stop is commanded in that same scan.
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_PRESSURE_CTRL);
h.sl.hook.driveFault = true;
h = runFor(h, dt);                               % the axis sets the alarm flag
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_PRESSURE_CTRL && ...
    ~h.sl.G.G_cmdStop.Execute);
h = runFor(h, dt);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_FAULT, 'not in ST_FAULT');
assert(h.sl.G.G_cmdStop.Execute && h.sl.G.G_cmdDirectControl.Velocity == 0 && ...
    h.y.faultActive == 1, 'the stop was not commanded in the scan of the fault');
h.sl.hook.driveFault = false;
fprintf('ok  scan order: a drive fault commands the stop in the scan it enters ST_FAULT\n');

fprintf('all passed\n');
end


function h = harnessInit()
h.sl = slaveInit();
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0, 'positionSpeed', 0);
[h.sl, h.y] = slaveStep(h.sl, h.u);
h.t = 0;
h.vMax = 0;
h.pMax = 0;
h.pMin = inf;
h.faultSeen = 0;
h.freezeHeartbeat = false;
end


function h = enterMode(h, mode, reference)
% The master's handshake: command the mode with MotionEnable low, wait
% for the confirmation, then raise MotionEnable.
h.u.modeCode = mode;
h.u.motionEnable = 0;
h.u.reference = reference;
h = runUntil(h, 2, @(h) h.y.confCode == mode);
assert(h.y.confCode == mode, 'mode %d not confirmed', mode);
h = runFor(h, 0.05);
h.u.motionEnable = 1;
h = runUntil(h, 5, @(h) h.sl.G.G_sysConfirmedMode == mode && h.y.brakeDisengage == 1 ...
    && h.sl.G.G_sysCurrentState ~= h.sl.E.E_SystemState.ST_BRAKE_RELEASE);
end


function h = runFor(h, T)
h = runUntil(h, T, []);
end


function h = runUntil(h, T, done)
% The master's heartbeat advances every 25 ms; the slave scans every 4 ms.
dt = h.sl.G.G_cfgScanTime;
for k = 1:round(T / dt)
    h.t = h.t + dt;
    if mod(k, round(0.025 / dt)) == 0 && ~h.freezeHeartbeat
        h.u.heartbeat = mod(h.u.heartbeat + 1, 65536);
    end
    [h.sl, h.y] = slaveStep(h.sl, h.u);
    h.vMax = max(h.vMax, abs(h.sl.ax.v));
    h.pMax = max(h.pMax, h.sl.ax.P);
    h.pMin = min(h.pMin, h.sl.ax.P);
    h.faultSeen = max(h.faultSeen, h.y.faultActive);
    if ~isempty(done) && done(h)
        return;
    end
end
end
