function slavePressureMode()
%SLAVEPRESSUREMODE Run the simulated slave (slave-sim, generated from
%   slave-src) in pressure mode through its holding registers, standing in
%   for the master: handshake into mode 7, hold and step the target, ride
%   through an outflow disturbance, clamp an excessive target, leave the
%   mode, fault on a broken transducer loop only while the loop is
%   closed on it and, with the gain schedule
%   (G_cfgPressureScheduleEnable), compress to 10 MPa with the gains
%   following the stiffness bound.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath slave-sim slave-sim/generated tests
%   Depends on: slave-src slave-sim tools models bench/characterisation

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'));

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

% A target above the configured maximum is clamped to it.
h.u.reference = 20;
h = runFor(h, 0.5);
h = runUntil(h, 20, settled);
h = runFor(h, 2);
assert(abs(h.sl.ax.P - G.G_cfgPressureTargetMax) <= G.G_cfgPressureInBand, 'at %.3f MPa', h.sl.ax.P);
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

% Gain schedule on the simulated chamber (the design model's plant, 343 mL
% of saline with a 1:1 oxygen charge): the master describes that fill in
% holding registers HR9-12, the gains follow wc / k_est, the integrator is
% held until the first arrival, the pressure settles in band below the
% ceiling, and the switch and the fill are latched on entry.
h.sl.G.G_cfgPressureScheduleEnable = true;
h.sl.G.G_mbSalineVolume = 343;                        % mL
h.sl.G.G_mbOxygenRatio = 1;
h.sl.ax.x = chamberPosition(h.sl, 0.5);
h = enterMode(h, MODE_PRESSURE, 10);
assert(h.sl.s.bPressureSchedActive && h.sl.s.fbPressureSched.IntegHold);
assert(h.sl.s.rSchedSaline == 343 && h.sl.s.rSchedOxygenRatio == 1);
h.pMax = 0;
t0 = h.t;
% The bench's copy of the bound (bench/characterisation/charScheduleBound,
% which charScheduleCheck compares with a log) is held equal to the block's
% KEst and KEstTarget, sampled every 100 ms through the compression
gSched = h.sl.G;
fillSched = struct('saline_mL', 343, 'oxygenRatio', 1, 'hoseOpen', logical(gSched.G_cfgPressureSchedHoseOpen));
trackBound([], [], [], true);
h = runUntil(h, 30, @(h) trackBound(h, gSched, fillSched, false) || settled(h));
assert(settled(h), 'scheduled loop did not settle in the target band');
[~, boundGap, boundSamples] = trackBound([], [], [], false);
assert(boundSamples > 20 && boundGap < 1e-9, 'the bench''s bound differs from the block''s by %.2g (%d samples)', ...
    boundGap, boundSamples);
fprintf('ok  bench stiffness bound equals the block''s KEst and KEstTarget (%d samples, relative difference %.1e)\n', ...
    boundSamples, boundGap);
t1 = h.t - t0;
h = runFor(h, 3);
sc = h.sl.s.fbPressureSched;
assert(isfinite(sc.KEst) && sc.KEst > 0 && sc.KEstTarget >= sc.KEst);
wc = h.sl.G.G_cfgPressureSchedCrossover;
kSched = max(sc.KEst, sc.KEstTarget);
assert(abs(sc.Kp - min(max(wc / kSched, h.sl.G.G_cfgPressureSchedKpMin), ...
    h.sl.G.G_cfgPressureSchedKpMax)) < 1e-12);
assert(h.sl.s.fbPressureCtrl.Kp == sc.Kp && h.sl.s.fbPressureCtrl.Stiffness == sc.Stiffness);
assert(~sc.IntegHold && ~sc.ParamError);
assert(abs(h.sl.ax.P - 10) <= G.G_cfgPressureInBand && h.y.performanceStatus == 1);
assert(h.pMax < P_CEILING, 'peak %.3f MPa', h.pMax);
% The switch and the fill are latched on entry: turning the switch off or
% rewriting the registers mid-run changes nothing.
kp = sc.Kp;
h.sl.G.G_cfgPressureScheduleEnable = false;
h.sl.G.G_mbSalineVolume = 100;
h.sl.G.G_mbOxygenRatio = 2;
h = runFor(h, 0.5);
assert(h.sl.s.bPressureSchedActive && abs(h.sl.s.fbPressureCtrl.Kp - kp) < 0.05 * kp);
assert(h.sl.s.rSchedSaline == 343 && h.sl.s.rSchedOxygenRatio == 1);
h.u.motionEnable = 0;
h = runFor(h, 1);
h.u.modeCode = 0;
h.u.reference = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE && h.y.faultActive == 0);
fprintf('ok  scheduled gains: 10 MPa in %.1f s, k_est %.2f MPa/mm, Kp %.3f, max %.3f MPa\n', ...
    t1, sc.KEst, sc.Kp, h.pMax);

% A fill that was never written (zero saline volume) is a ParamError of the
% schedule, and the loop runs on the configured gains.
h.sl.G.G_cfgPressureScheduleEnable = true;
h.sl.G.G_mbSalineVolume = 0;
h.sl.G.G_mbOxygenRatio = 0;
h.sl.ax.x = chamberPosition(h.sl, 0.5);
h = enterMode(h, MODE_PRESSURE, 10);
h = runFor(h, 1);
assert(h.sl.s.bPressureSchedActive && h.sl.s.fbPressureSched.ParamError && ~h.sl.s.bPressureSchedUsed);
assert(h.sl.s.fbPressureCtrl.Kp == h.sl.G.G_cfgPressureKp && h.sl.s.fbPressureCtrl.Ki == h.sl.G.G_cfgPressureKi);
h.u.motionEnable = 0;
h = runFor(h, 1);
h.u.modeCode = 0;
h.u.reference = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
fprintf('ok  unwritten fill -> schedule ParamError, configured gains\n');

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


function [done, worst, count] = trackBound(h, g, fill, reset)
% Compare charScheduleBound with the block's KEst and KEstTarget on the
% scan just run, one scan in 25; returns the largest relative difference
% and the number of scans compared so far. Never ends a run (done false).
persistent w n k
done = false;
if reset
    w = 0; n = 0; k = 0;
elseif ~isempty(h)
    k = k + 1;
    if mod(k, 25) == 0
        sc = h.sl.s.fbPressureSched;
        if ~sc.ParamError && sc.KEst > 0
            % The block's own inputs of the scan: the formula is held equal,
            % not the signals that feed it
            pRef = (sc.rPRef - g.G_cfgPressureSchedPatm) / 1e6;     % [Pa -> MPa]
            [kEst, kEstTarget] = charScheduleBound(sc.Actual, sc.Position, pRef, sc.Target, fill, g);
            w = max([w, abs(kEst - sc.KEst) / sc.KEst, abs(kEstTarget - sc.KEstTarget) / sc.KEstTarget]);
            n = n + 1;
        end
    end
end
worst = w;
count = n;
end
