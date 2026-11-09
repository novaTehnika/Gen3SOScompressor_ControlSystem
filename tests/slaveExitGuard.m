function slaveExitGuard()
%SLAVEEXITGUARD Run the simulated slave (slave-sim, generated from
%   slave-src) against its piston exit guard (FB_PistonExitGuard),
%   standing in for the master: a retract into the guard band under
%   pressure faults and stops short of the cylinder entry at any speed,
%   the fault resets and the piston moves in freely, a vented chamber
%   retracts through the band whatever the torque, a failed transducer
%   counts as pressure while the piston is in the bore, and a charged
%   chamber stops Go Home and a pressure loop that chases a low target.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath slave-sim slave-sim/generated tests
%   Depends on: slave-src slave-sim tools models

MODE_VELOCITY = 3;
MODE_GO_HOME = 5;
MODE_PRESSURE = 7;
FAULT_PISTON_EXIT = 5;
FAULT_PRESSURE_SENSOR = 8;

h = harnessInit();
E = h.sl.E;
G = h.sl.G;
entry = G.G_cfgPosCylinderEntry;
boundary = entry + G.G_cfgPosExitGuardMargin;
retreat = G.G_cfgExitGuardRetreat;
% Travel a trip allows: the retreat, the scans until MC_Stop has the
% axis, and the stop itself.
stopDist = @(v) retreat + 0.05 * v + v^2 / (2 * G.G_cfgStopDecel);

% The margin holds the worst trip: the fastest retract stops inside it.
assert(stopDist(G.G_cfgVelLimitMax) < G.G_cfgPosExitGuardMargin / 2, ...
    'a trip at %.1f mm/s travels %.2f mm of the %.1f mm margin', ...
    G.G_cfgVelLimitMax, stopDist(G.G_cfgVelLimitMax), G.G_cfgPosExitGuardMargin);
assert(G.G_cfgExitGuardPressure > 0 && retreat > 0);
fprintf('ok  margin %.0f mm against %.2f mm of travel after a trip at %.0f mm/s\n', ...
    G.G_cfgPosExitGuardMargin, stopDist(G.G_cfgVelLimitMax), G.G_cfgVelLimitMax);

% Retract into the band under pressure: FAULT_PISTON_EXIT once the piston
% has backed out the retreat past the boundary, and the stop leaves most
% of the margin.
h = place(h, boundary + 4, 0.5);
assert(~h.sl.G.G_sysExitGuardActive && h.sl.G.G_sysInCylinderZone);
h = enterMode(h, MODE_VELOCITY, -3);
h = runUntil(h, 10, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PISTON_EXIT, 'no exit fault');
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_FAULT);
xTrip = h.y.actualPosition;
assert(xTrip < boundary - retreat && xTrip > boundary - retreat - 0.1, 'tripped at %.3f mm', xTrip);
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
assert(h.sl.G.G_sysLastFaultState == E.E_SystemState.ST_VELOCITY_CTRL);
assert(h.sl.ax.v == 0 && h.sl.ax.x > boundary - stopDist(3), 'rest at %.3f mm', h.sl.ax.x);
assert(h.sl.G.G_sysExitGuardActive);
fprintf('ok  pressurised retract into the band: fault at %.2f mm, at rest at %.2f mm (entry %.0f mm)\n', ...
    xTrip, h.sl.ax.x, entry);

% The fault is an event: at rest it resets with the chamber still
% pressurised, and the piston moves in with no fault.
h = resetFault(h, FAULT_PISTON_EXIT);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_BRAKE_HOLD);
x0 = h.sl.ax.x;
h = enterMode(h, MODE_VELOCITY, 1);
h = runFor(h, 0.5);
assert(h.y.faultActive == 0 && h.sl.ax.x > x0 + 0.3 && h.sl.ax.x < boundary, ...
    'moved in to %.3f mm', h.sl.ax.x);
fprintf('ok  reset under pressure, then moving in is free\n');

% A back-off inside the retreat does not trip. A creep below the in-motion
% speed does, once it has covered the retreat from the innermost position.
h.u.reference = 0;
h = runUntil(h, 2, @(h) h.sl.ax.v == 0);
xIn = h.sl.ax.x;
h.u.reference = -0.5;
h = runFor(h, 0.4);
h.u.reference = 0;
h = runUntil(h, 2, @(h) h.sl.ax.v == 0);
assert(h.y.faultActive == 0 && h.sl.ax.x < xIn - 0.1 && h.sl.ax.x > xIn - retreat, ...
    'back-off of %.3f mm', xIn - h.sl.ax.x);
vCreep = 0.5 * G.G_cfgInMotionVelThreshold;
h.u.reference = -vCreep;
h.vMax = 0;
h = runUntil(h, 2 * retreat / vCreep, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PISTON_EXIT, 'creep not caught');
assert(h.vMax < G.G_cfgInMotionVelThreshold, 'crept at %.3f mm/s', h.vMax);
assert(xIn - h.y.actualPosition < retreat + 0.05, 'creep of %.3f mm', xIn - h.y.actualPosition);
h = runUntil(h, 2, @(h) h.sl.ax.v == 0);
h = resetFault(h, FAULT_PISTON_EXIT);
fprintf('ok  back-off inside the retreat passes; a %.2f mm/s creep trips after %.2f mm\n', ...
    vCreep, retreat);

% Each acknowledged fault allows one more trip's travel, no more.
x0 = h.sl.ax.x;
h = enterMode(h, MODE_VELOCITY, -3);
h = runUntil(h, 5, @(h) h.y.faultActive == 1);
assert(h.y.faultCode == FAULT_PISTON_EXIT);
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
assert(x0 - h.sl.ax.x < stopDist(3), 'retry travelled %.3f mm', x0 - h.sl.ax.x);
h = resetFault(h, FAULT_PISTON_EXIT);
fprintf('ok  a retry after the reset trips again within %.2f mm\n', x0 - h.sl.ax.x);

% A vented chamber retracts through the band and out of the cylinder with
% no fault, whatever torque the motor reports.
h = place(h, boundary + 4, 0.5 * G.G_cfgExitGuardPressure);
h.sl.hook.torque = 90;
h = enterMode(h, MODE_VELOCITY, -5);
h = runUntil(h, 10, @(h) h.sl.ax.x < entry - 4);
assert(h.sl.ax.x < entry - 4 && h.faultSeen == 0);
assert(~h.sl.G.G_sysExitGuardActive && ~h.sl.G.G_sysInCylinderZone);
h.sl.hook.torque = 0;
h = stopAxis(h);
fprintf('ok  vented retract through the band and past the entry (torque 90 %%)\n');

% A pressure measured below the configured entry is acted on: the band
% has no lower edge.
h = place(h, entry - 20, 0.5);
h = enterMode(h, MODE_VELOCITY, -3);
h = runUntil(h, 5, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PISTON_EXIT);
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
h = resetFault(h, FAULT_PISTON_EXIT);
fprintf('ok  pressure below the configured entry still trips\n');

% Failed transducer loop (0 mA): in the bore the pressure is unknown and
% counts as present. At rest there is no fault; a retract trips as
% FAULT_PRESSURE_SENSOR, which resets with the loop still open, and the
% piston moves in freely.
h = place(h, boundary - 6, 0);
h.sl.hook.pressureVolt = 0;
h = runFor(h, 1);
assert(h.y.faultActive == 0 && h.y.actualPressure < 0 && h.sl.G.G_sysExitGuardActive);
h = enterMode(h, MODE_VELOCITY, -3);
h = runUntil(h, 5, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PRESSURE_SENSOR, 'no sensor fault');
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
assert(h.sl.ax.x > boundary - 6 - stopDist(3));
h = resetFault(h, FAULT_PRESSURE_SENSOR);
h = enterMode(h, MODE_VELOCITY, 2);
h = runFor(h, 1);
assert(h.y.faultActive == 0);
h = stopAxis(h);
fprintf('ok  open transducer loop in the bore: retract faults as the sensor, moving in is free\n');

% Below the entry the cylinder is open and nothing is assumed: the axis
% moves out with the loop still open.
h = place(h, entry - 4, []);
h.faultSeen = 0;
h = enterMode(h, MODE_VELOCITY, -5);
h = runFor(h, 3);
assert(h.faultSeen == 0 && h.sl.ax.x < entry - 15 && ~h.sl.G.G_sysExitGuardActive);
h = stopAxis(h);
h.sl.hook.pressureVolt = NaN;
h = runFor(h, 0.5);
fprintf('ok  open transducer loop below the entry: the axis still retracts\n');

% The simulated chamber, with the cylinder entry where the slave has it
% and a fill large enough to hold the supply pressure 20 mm above the
% boundary (the simulator's default fill is near ambient by the boundary
% and does not arm the guard): its charge is still well above the
% threshold at the boundary.
h.sl.sim.chamberContactPos = entry;
h.sl.sim.plantStartPos = boundary + 20;
h = place(h, boundary + 15, []);
assert(chamberPosition(h.sl, 2 * G.G_cfgExitGuardPressure) < boundary - 1, ...
    'the charge is below %.2f MPa at the boundary', 2 * G.G_cfgExitGuardPressure);

% Go Home with the chamber charged: the move toward position 0 is stopped
% in the band.
h.u.positionSpeed = G.G_cfgGoHomeVelocity;
h = enterMode(h, MODE_GO_HOME, 0);
h = runUntil(h, 20, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PISTON_EXIT, 'Go Home not stopped');
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
assert(h.sl.G.G_sysLastFaultState == E.E_SystemState.ST_GO_HOME);
assert(h.sl.ax.v == 0 && h.sl.ax.x > boundary - stopDist(G.G_cfgGoHomeVelocity), ...
    'rest at %.3f mm', h.sl.ax.x);
assert(h.sl.ax.P > G.G_cfgExitGuardPressure);
h = resetFault(h, FAULT_PISTON_EXIT);
fprintf('ok  Go Home with a charged chamber: stopped at %.2f mm with %.2f MPa\n', ...
    h.sl.ax.x, h.sl.ax.P);

% Pressure mode with a target below what the charge holds at the entry:
% the loop retracts to lower the pressure and is stopped in the band.
h = place(h, boundary + 10, []);
h = enterMode(h, MODE_PRESSURE, 0);
h = runUntil(h, 20, @(h) h.y.faultActive == 1);
assert(h.y.faultActive == 1 && h.y.faultCode == FAULT_PISTON_EXIT, 'pressure loop not stopped');
h = runUntil(h, 5, @(h) h.sl.ax.v == 0);
assert(h.sl.G.G_sysLastFaultState == E.E_SystemState.ST_PRESSURE_CTRL);
assert(h.sl.ax.v == 0 && h.sl.ax.x > boundary - stopDist(G.G_cfgPressureVelMax), ...
    'rest at %.3f mm', h.sl.ax.x);
h = resetFault(h, FAULT_PISTON_EXIT);
fprintf('ok  pressure mode chasing 0 MPa on a charged chamber: stopped at %.2f mm\n', h.sl.ax.x);

fprintf('all passed\n');
end


function h = harnessInit()
h.sl = slaveInit();
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0, 'positionSpeed', 5);
[h.sl, h.y] = slaveStep(h.sl, h.u);
h.t = 0;
h.vMax = 0;
h.faultSeen = 0;
end


function v = sensorVolts(sim, P)
% Voltage across the shunt at P [MPa gauge].
mA = sim.sensorZeroMa + P / sim.sensorRangeMPa * sim.sensorSpanMa;
v = mA * sim.sensorShuntOhm / 1000;
end


function h = place(h, x, P)
% Put the piston at x [mm] at rest, the slave idle, and, unless P is
% empty, the transducer at P [MPa gauge]. The chamber is read as vented
% while the piston is placed, so the jump in position is not a retreat
% under pressure.
volt = h.sl.hook.pressureVolt;
h.sl.hook.pressureVolt = sensorVolts(h.sl.sim, 0);
h = runFor(h, 0.3);
h.sl.ax.x = x;
h = runFor(h, 0.1);
if isempty(P)
    h.sl.hook.pressureVolt = volt;
else
    h.sl.hook.pressureVolt = sensorVolts(h.sl.sim, P);
end
h = runFor(h, 0.5);
assert(h.y.faultActive == 0, 'fault %d after placing the piston', h.y.faultCode);
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


function h = stopAxis(h)
% Drop MotionEnable, wait for the hold, then go to idle.
h.u.motionEnable = 0;
h.u.reference = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_HOLD_POSITION ...
    && h.sl.G.G_sysHalted);
h.u.modeCode = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE && h.y.faultActive == 0);
end


function h = resetFault(h, code)
% The master's reset: MotionEnable low, the fault code mirrored in the
% mode register, then FaultReset held until the fault clears.
h.u.motionEnable = 0;
h.u.reference = 0;
h.u.modeCode = code;
h = runFor(h, 0.2);
h.u.faultReset = 1;
h = runUntil(h, 2, @(h) h.y.faultActive == 0);
assert(h.y.faultActive == 0 && h.y.faultCode == 0, 'fault %d did not clear', code);
h.u.faultReset = 0;
h.u.modeCode = 0;
h = runFor(h, 0.5);
assert(h.y.faultActive == 0, 'fault %d came back', h.y.faultCode);
h.faultSeen = 0;
end


function h = runFor(h, T)
h = runUntil(h, T, []);
end


function h = runUntil(h, T, done)
% The master's heartbeat advances every 25 ms; the slave scans every 4 ms.
dt = h.sl.G.G_cfgScanTime;
for k = 1:round(T / dt)
    h.t = h.t + dt;
    if mod(k, round(0.025 / dt)) == 0
        h.u.heartbeat = mod(h.u.heartbeat + 1, 65536);
    end
    [h.sl, h.y] = slaveStep(h.sl, h.u);
    h.vMax = max(h.vMax, abs(h.sl.ax.v));
    h.faultSeen = max(h.faultSeen, h.y.faultActive);
    if ~isempty(done) && done(h)
        return;
    end
end
end
