function charRunSim()
%CHARRUNSIM Run bench/characterisation/charRun against the simulated slave
%   through fakeProbeLink (the friction bench's stand-in, with its step
%   probe emulation and an outlet valve), then the three fits on what it
%   logged: pressure-ended ramps, a dwell, velocity steps and a hold in one
%   velocity session with captures; the one-pass stopping-distance path of
%   test 4 (charStopPath) and its fit (charStopDistance); a delivery
%   through the orifice bank in pressure mode; the pressure and position
%   guards and a slave fault.
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools bench/characterisation bench/common tests/fakeProbeLink.m

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'), fullfile(here, '..', 'bench', 'common'));
saveDir = tempname();
ccfg = charConfig();
T = 1; STEP = 2; STATUS = 3; MODE = 4; REF = 5; POS = 6; P = 9;    % log columns

% --- Velocity session: ramps on the pressure, dwell, steps, then a hold ---------
% The simulated chamber is a gas spring: 1.7 MPa at 255 mm, 3 MPa at 256.4 mm.
opts = linkOpts(saveDir, 255);
opts.probe = true;
path = {{'ramp', 0.1, 3.0, 40}, {'dwell', 2.5}, {'ramp', -0.1, 2.0, 40}, ...
        {'velstep', [0.5 0], [2.5 2.5]}, {'hold', 1}};
r = charRun('sim', path, opts);
assert(isempty(r.abort), r.abort);
L = r.runLog;
s1 = L(L(:, STEP) == 1, :);
assert(s1(end, P) >= 3.0 && s1(end, P) < 3.3, 'ramp did not end at 3 MPa (%.2f)', s1(end, P));
assert(sum(s1(:, STATUS) == 4) > 100 && all(s1(s1(:, STATUS) == 4, MODE) == 3), 'ramp not in velocity mode');
s3 = L(L(:, STEP) == 3, :);
assert(s3(end, P) <= 2.0 && s3(end, P) > 1.7, 'down ramp did not end at 2 MPa (%.2f)', s3(end, P));
session = L(L(:, STEP) >= 2 & L(:, STEP) <= 4, :);
assert(~any(session(:, STATUS) == 7) && all(session(session(:, STEP) <= 3, STATUS) == 4), ...
    'velocity session left the mode');
s5 = L(L(:, STEP) == 5, :);
assert(any(s5(:, MODE) == 2) && any(s5(:, STATUS) == 3), 'hold did not run in position mode');
assert(numel(r.captures) == 5, 'expected one capture per command change, got %d', numel(r.captures));
cap = r.captures{1};
assert(cap.step == 1 && size(cap.data, 1) == 500 && size(cap.data, 2) == 6);
assert(abs(cap.data(1, 6) - s1(1, P)) < 0.3, 'capture pressure %.2f, log %.2f MPa', cap.data(1, 6), s1(1, P));
j = find(cap.data(:, 2) ~= 0, 1);
assert(~isempty(j) && abs(cap.data(j, 2) - 0.1) < 1e-3 && j > 5, 'first capture not around the 0.1 mm/s command');
assert(cap.data(min(j + 60, 500), 3) > 0.09, 'velocity did not follow the command in the capture');
cap = r.captures{4};
j = find(abs(cap.data(:, 2) - 0.5) < 1e-3, 1);
assert(~isempty(j) && abs(cap.data(j - 1, 2) + 0.1) < 1e-3, 'velstep capture command');
assert(exist(r.file, 'file') == 2 && numel(r.capFiles) == 5);
assert(r.exchangeFailures == 0 && r.probeFailures == 0);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  velocity session: two ramps, dwell, steps, hold; five captures\n');

% The simulated axis follows its slewed command exactly: the lag fit must
% find a lag of at most a few scans and a small residual, from the
% captures and from the 50 ms log, velocity and pressure.
lag = charDriveLag([r.capFiles {r.file}], struct('quiet', true));
assert(all([lag.captures.tauV] < 0.006), 'lag %s s on a lag-free axis', mat2str([lag.captures.tauV], 3));
assert(all([lag.captures.residual] < 0.03), 'lag fit residual %s', mat2str([lag.captures.residual], 3));
assert(~isempty(lag.runLog) && lag.runLog.nSteps >= 2, 'run log steps not found');   % the 0.1 mm/s commands are below minStep
assert(lag.runLog.nStepsP >= 1, 'no pressurised step in the run log');
% The gas spring stiffens by a fifth over the 0.5 mm/s capture, which the
% constant-stiffness model absorbs as a few tens of milliseconds; a liquid
% chamber holds its stiffness to a percent over a capture.
capsP = lag.captures(~isnan([lag.captures.tauP]));
assert(numel(capsP) >= 1 && all([capsP.tauP] + [capsP.TdP] < 0.03), ...
    'piston lag %s s from the captures of a lag-free axis', mat2str([capsP.tauP] + [capsP.TdP], 3));
assert(lag.tauP_s < 0.03, 'piston lag %.3f s on a lag-free axis', lag.tauP_s);
fprintf('ok  drive lag fit: captures %.1f ms, run log %.0f ms, pressure %.1f ms from %d captures\n', ...
    1000 * lag.tauV_s, 1000 * (lag.runLog.tauV + lag.runLog.Td), 1000 * lag.tauP_s, numel(capsP));

% The gas spring is not a liquid: the bulk modulus fit must reject it and
% attribute the compliance to gas of about the chamber volume.
bk = charBulkModulus(r.file, struct('quiet', true, 'deadVolume_mL', 10, 'pressureRange', [2.0 3.1]));
assert(~bk.pass && bk.K_lin_GPa < 0.5, 'gas spring accepted as a liquid (K %.3f GPa)', bk.K_lin_GPa);
vGas = S.sl.sim.chamberGasVolume * S.sl.sim.atmMPa / (2.5 + S.sl.sim.atmMPa) / 1000;
assert(abs(bk.Vg_mL - vGas) / vGas < 0.3, 'gas volume %.2f mL, chamber holds about %.2f mL', bk.Vg_mL, vGas);
fprintf('ok  bulk modulus fit rejects the gas spring (K %.4f GPa, gas %.1f mL of %.1f)\n', ...
    bk.K_lin_GPa, bk.Vg_mL, vGas);

% --- One pass: levels, excursions in both directions, stops ---------------------
% The path is planned on the simulated gas spring's own stiffness, so that
% each excursion stays within its budget, and the fit must find the
% deceleration the simulated axis slews at (G_cfgDirectCtrlDecel) in both
% directions with no delay term, since the axis follows its command at
% once.
opts = linkOpts(saveDir, 255);
opts.probe = 'stops';
S = fakeProbeLink('state');
sim = S.sl.sim;
kGas = @(Pq) (Pq + sim.atmMPa).^2 * sim.pistonArea / (sim.atmMPa * sim.chamberGasVolume);
[path, plan] = charStopPath(struct('levels_MPa', [2 3], 'speeds', [0.5 2], 'budget_MPa', 1.0, ...
    'rampSpeed', 0.5, 'stiffness', kGas, 'T0_s', 0.06, 'latency_s', 0.05, 'D_mm_s2', ccfg.velDecel, ...
    'hold_s', 1, 'pause_s', 3, 'quiet', true));
nStops = plan(1).nStops;
types = cellfun(@(s) s{1}, path, 'UniformOutput', false);
assert(nStops >= 6 && sum(strcmp(types, 'move')) == nStops && sum(strcmp(types, 'settle')) == nStops / 2, ...
    'plan: %d stops, %d steps', nStops, numel(path));
r = charRun('simstop', path, opts);
assert(isempty(r.abort), r.abort);
L = r.runLog;
% Every excursion stays within its budget (plus the stop beyond the leg)
% and each pair of legs returns the piston to where the first began
level = 0;
for s = 1:numel(path)
    rows = L(L(:, STEP) == s, :);
    switch path{s}{1}
        case 'ramp'
            level = level + (path{s}{2} > 0);
        case 'move'
            if path{s}{2} > 0
                xPair = rows(1, POS);
                Plevel = plan(min(level, numel(plan))).P_MPa;
                assert(max(rows(:, P)) < Plevel + 1.0 + 0.6, ...
                    'step %d reached %.2f MPa at a %.0f MPa level', s, max(rows(:, P)), Plevel);
            else
                after = L(L(:, STEP) == s + 1, :);
                assert(abs(after(end, POS) - xPair) < 0.15, ...
                    'pair ending at step %d returned to %.3f mm from %.3f', s, after(end, POS), xPair);
            end
    end
end
rs = charStopDistance([r.capFiles {r.file}], struct('quiet', true, 'D_cfg', ccfg.velDecel, 'lag_s', 0.06));
assert(numel(rs.stops) >= nStops, 'only %d stops found', numel(rs.stops));
assert(sum(strcmp({rs.stops.source}, 'capture')) >= nStops - 1, 'only %d stops in the captures', ...
    sum(strcmp({rs.stops.source}, 'capture')));
for f = {rs.fit.comp, rs.fit.ret}
    assert(abs(f{1}.D_mm_s2 / ccfg.velDecel - 1) < 0.05 && f{1}.T0_s < 0.01, ...
        'fit D %.1f mm/s^2 (axis slews at %.0f), T0 %.3f s', f{1}.D_mm_s2, ccfg.velDecel, f{1}.T0_s);
end
assert(rs.pass, 'stopping-distance checks failed');
fprintf('ok  one-pass path: %d steps, %d stops, D %.1f / %.1f mm/s^2 (axis %.0f), T0 %.0f ms\n', ...
    numel(path), numel(rs.stops), rs.fit.comp.D_mm_s2, rs.fit.ret.D_mm_s2, ccfg.velDecel, 1000 * rs.T0_s);

% --- Delivery: pressure mode, valve opens, the loop balances the outflow --------
opts = linkOpts(saveDir, 250);
fakeProbeLink('valveAt', 12, 0.68);
r = charRun('simdeliver', {{'pressure', 2.0, 30}}, opts);
assert(isempty(r.abort), r.abort);
L = r.runLog;
inStep = L(:, STEP) == 1 & L(:, STATUS) == 5;
% The volume that left while the step ran (the outflow goes on after the
% stop). The simulated chamber is a gas spring, so what left exceeds the
% swept volume by the gas expansion as the pressure settles, a term the
% script's liquid correction cannot see; the check allows for it.
S = fakeProbeLink('state');
tEnd = max(L(inStep, T));
collected = interp1(S.collectLog(:, 1), S.collectLog(:, 2), tEnd) / 1000;
vGasAt = @(Pq) S.sl.sim.chamberGasVolume * S.sl.sim.atmMPa ./ (Pq + S.sl.sim.atmMPa) / 1000;   % mL
fl = charOrificeFlow(r.file, struct('quiet', true, 'tOpen', 12, 'collected_mL', collected));
assert(abs(fl.Cd - 0.68) / 0.68 < 0.06, 'Cd %.3f, valve set to 0.68', fl.Cd);
% The loop pulls the pressure back up during the window, so the piston also
% compresses the gas; with the gas spring's equivalent bulk modulus the
% correction removes that and Cd comes back within 2 %.
late = inStep & L(:, T) > 15;
Pm = mean(L(late, P)); xm = mean(L(late, POS));
Kgas = (ccfg.pistonArea_mm2 * (ccfg.posEOT - xm) + ccfg.deadVolume_mL * 1000) * (Pm + S.sl.sim.atmMPa) ...
    / (vGasAt(Pm) * 1000);                                                 % MPa
flk = charOrificeFlow(r.file, struct('quiet', true, 'tOpen', 12, 'K_Pa', Kgas * 1e6));
assert(abs(flk.Cd - 0.68) / 0.68 < 0.02, 'Cd %.3f with the gas correction, valve set to 0.68', flk.Cd);
pAt = @(tq) interp1(L(inStep, T), L(inStep, P), tq);
expansion = vGasAt(pAt(tEnd)) - vGasAt(pAt(12));
expected = collected / (collected - expansion);
assert(abs(fl.gravimetric.ratio - expected) < 0.02, 'collected %.3f of swept, %.3f expected with %.1f mL of gas expansion', ...
    fl.gravimetric.ratio, expected, expansion);
assert(all(cellfun(@(c) c.pass, fl.checks(1:2))), 'Cd or its spread failed');
fl2 = charOrificeFlow(r.file, struct('quiet', true));
assert(abs(fl2.windows(1).t1 - fl.windows(1).t1) < 2, 'valve opening detected at %.1f s, set at 12 s', fl2.windows(1).t1);
fprintf('ok  delivery: Cd %.3f (%.3f with the gas correction), collected %.1f %% of swept (%.1f expected), opening found at %.1f s\n', ...
    fl.Cd, flk.Cd, 100 * fl.gravimetric.ratio, 100 * expected, fl2.windows(1).t1);

% --- Guards ----------------------------------------------------------------------
opts = linkOpts(saveDir, 257);                 % about 10 MPa in the gas spring
opts.pressureStop = 10.5;
r = charRun('simguard', {{'ramp', 0.1, 11.5, 20}}, opts);
assert(~isempty(strfind(r.abort, 'pressure guard')), 'pressure guard did not trip: %s', r.abort);
assert(max(r.runLog(:, P)) < 11.0, 'pressure reached %.2f MPa after the guard', max(r.runLog(:, P)));
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0);
opts = linkOpts(saveDir, 150);                 % outside the chamber
opts.posStop = 160;
r = charRun('simposguard', {{'jog', 2, 10}}, opts);
assert(~isempty(strfind(r.abort, 'position guard')), 'position guard did not trip: %s', r.abort);
S = fakeProbeLink('state');
assert(S.sl.ax.x < opts.posStop + 1 && S.sl.ax.v == 0);
fprintf('ok  pressure and position guards stop the run\n');

% A fault latched before the run is cleared with the operator reset and
% the run goes on; the drive's own fault is gone before the master is up.
opts = linkOpts(saveDir, 240);
fakeProbeLink('faultAt', 0.02, 0.08);
r = charRun('simlatched', {{'jog', 1, 3}}, opts);
assert(isempty(r.abort), 'latched fault not cleared: %s', r.abort);
L = r.runLog;
assert(any(L(:, STATUS) == 10) && any(L(:, STATUS) == 9) && any(L(:, STEP) == 1 & L(:, STATUS) == 4), ...
    'reset not seen in the log');
S = fakeProbeLink('state');
assert(S.sl.ax.x > 241, 'jog did not run after the reset');
fprintf('ok  latched fault cleared by the operator reset before the run\n');

% A drive fault ends the run with the stop sent.
opts = linkOpts(saveDir, 240);
fakeProbeLink('faultAt', 4);
r = charRun('simfault', {{'jog', 1, 8}}, opts);
assert(~isempty(strfind(r.abort, 'status 10')), 'fault did not abort: %s', r.abort);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.u.modeCode ~= 3, 'motion still commanded after the fault');
fprintf('ok  slave fault aborts the run\n');

fprintf('all passed\n');
end


function opts = linkOpts(saveDir, x0)
clear fakeProbeLink
fakeProbeLink('init');
fakeProbeLink('slave', @(sl) setfield(sl, 'ax', setfield(sl.ax, 'x', x0)));
link.read     = @(start, n) fakeProbeLink('read', start, n);
link.write    = @(start, v) fakeProbeLink('write', start, v);
link.writeOne = @(addr, v) fakeProbeLink('writeOne', addr, v);
link.close    = @() [];
opts.link = link;
opts.now = @() fakeProbeLink('now');
opts.wait = @(t) fakeProbeLink('wait', t);
opts.safetyModel = '';
opts.saveDir = saveDir;
end
