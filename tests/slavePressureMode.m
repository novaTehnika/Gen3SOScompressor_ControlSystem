function slavePressureMode()
%SLAVEPRESSUREMODE Run the simulated slave (slave-sim, generated from
%   slave-src) in pressure mode through its holding registers, standing in
%   for the master: handshake into mode 7, hold and step the target, ride
%   through an outflow disturbance, clamp an excessive target, leave the
%   mode, and fault on a broken transducer loop only while the loop is
%   closed on it.
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath slave-sim slave-sim/generated tests

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

% Enter pressure mode with a 10 MPa target: compresses, settles in band,
% stays within the velocity clamp and does not overshoot the target. The
% axis starts in the chamber at 0.5 MPa: below that the gas spring is soft
% and compressing it only adds travel at the clamp.
h.sl.ax.x = chamberPosition(h.sl.sim, 0.5);
h = enterMode(h, MODE_PRESSURE, 10);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_PRESSURE_CTRL);
h.vMax = 0; h.pMax = 0;
h = runUntil(h, 30, @(h) h.y.performanceStatus == 1);
assert(h.y.performanceStatus == 1, 'pressure did not reach the target band');
t1 = h.t;
h = runFor(h, 3);
assert(abs(h.sl.ax.P - 10) <= G.G_cfgPressureInBand && h.y.performanceStatus == 1);
assert(h.vMax <= G.G_cfgPressureVelMax + 0.01 && h.pMax <= 10.5, 'overshoot %.3f MPa', h.pMax - 10);
assert(abs(h.y.actualPressure - h.sl.ax.P) < 0.05);
fprintf('ok  compress to 10 MPa (%.1f s, peak %.2f mm/s, max %.3f MPa)\n', t1, h.vMax, h.pMax);

% Target step to 12 MPa.
h.u.reference = 12;
h.pMax = 0;
h = runUntil(h, 10, @(h) abs(h.sl.ax.P - 12) <= 0.05);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - 12) <= G.G_cfgPressureInBand && h.pMax <= 12.3, 'overshoot %.3f MPa', h.pMax - 12);
fprintf('ok  step to 12 MPa (max %.3f MPa)\n', h.pMax);

% Outflow disturbance: the loop holds the pressure up while liquid leaves
% and recovers once it stops.
h.sl.hook.outflow = 300;                 % mm^3/s
h.pMin = inf;
h = runFor(h, 3);
assert(h.pMin > 11 && h.sl.ax.v > 0, 'loop did not follow the outflow');
h.sl.hook.outflow = 0;
h = runFor(h, 4);
assert(abs(h.sl.ax.P - 12) <= G.G_cfgPressureInBand);
fprintf('ok  outflow disturbance (dip to %.3f MPa)\n', h.pMin);

% A target above the configured maximum is clamped to it.
h.u.reference = 20;
h = runUntil(h, 10, @(h) abs(h.sl.ax.P - G.G_cfgPressureTargetMax) <= 0.05);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - G.G_cfgPressureTargetMax) <= G.G_cfgPressureInBand);
assert(h.sl.ax.P < 16, 'pressure beyond the transducer range');
fprintf('ok  target clamped to %.0f MPa\n', G.G_cfgPressureTargetMax);

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

fprintf('all passed\n');
end


function h = harnessInit()
h.sl = slaveInit();
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0);
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
% The master's heartbeat advances every 25 ms; the slave scans every 2 ms.
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
