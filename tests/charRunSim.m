function charRunSim()
%CHARRUNSIM Run bench/characterisation/charRun against the simulated slave
%   through fakeProbeLink (the friction bench's stand-in: the slave's own
%   step probe behind the link, and an outlet valve), then the three fits on what it
%   logged: pressure-ended ramps, a dwell, velocity steps and a hold in one
%   velocity session with captures; the one-pass stopping-distance path of
%   test 4 (charStopPath) and its fit (charStopDistance); a delivery
%   through the orifice bank in pressure mode; the pressure and position
%   guards and a slave fault.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools models bench/characterisation bench/common tests/fakeProbeLink.m

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'), fullfile(here, '..', 'bench', 'common'));
saveDir = tempname();
ccfg = charConfig();
T = 1; STEP = 2; STATUS = 3; MODE = 4; REF = 5; POS = 6; P = 9;    % log columns

% --- Velocity session: ramps on the pressure, dwell, steps, then a hold ---------
% The simulated chamber is the plant model's (models/): the
% 1:1 oxygen charge over saline, about 0.5 MPa/mm at 2.5 MPa, and the
% charge dissolves as the pressure rises, so the ramps run at 0.3 mm/s.
opts = linkOpts(saveDir, @(sl) chamberPosition(sl, 1.7));
opts.probe = true;
% A capture is 500 scans, 2 s at the 4 ms scan, and its readout takes
% about as long again, so each dwell outlasts both.
path = {{'ramp', 0.3, 3.0, 40}, {'dwell', 4}, {'ramp', -0.3, 2.0, 40}, ...
        {'velstep', [0.2 0], [4 4]}, {'hold', 1}};
S = fakeProbeLink('state');
pl0 = S.sl.plant;                       % a snapshot: the chamber before the session
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
assert(cap.step == 1 && size(cap.data, 1) == 500 && size(cap.data, 2) == 8);
csv = benchReadCsv(r.capFiles{1});
assert(isequal(fieldnames(csv)', {'t', 'vCmd', 'velocity', 'torque', 'position', 'pressure', 'volts', 'status', 'file'}) ...
    && abs(csv.volts(1) - cap.data(1, 7)) < 1e-4 && csv.status(1) == cap.data(1, 8), 'capture file columns');
assert(abs(cap.data(1, 6) - s1(1, P)) < 0.3, 'capture pressure %.2f, log %.2f MPa', cap.data(1, 6), s1(1, P));
j = find(cap.data(:, 2) ~= 0, 1);
assert(~isempty(j) && abs(cap.data(j, 2) - 0.3) < 1e-3 && j > 5, 'first capture not around the 0.3 mm/s command');
assert(cap.data(min(j + 60, 500), 3) > 0.27, 'velocity did not follow the command in the capture');
cap = r.captures{4};
j = find(abs(cap.data(:, 2) - 0.2) < 1e-3, 1);
assert(~isempty(j) && abs(cap.data(j - 1, 2) + 0.3) < 1e-3, 'velstep capture command');
assert(exist(r.file, 'file') == 2 && numel(r.capFiles) == 5);
assert(r.exchangeFailures == 0 && r.probeFailures == 0);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  velocity session: two ramps, dwell, steps, hold; five captures\n');

% The simulated drive is the plant's: the command after a dead time of
% n_dead scans, through a first-order lag tauV, unslewed. The lag fit must
% find both from the captures, the dead time to within a scan (the
% capture's command column leads the drive by one), and their sum from the
% 50 ms log to within half a log interval.
S = fakeProbeLink('state');
tauV = S.sl.plant.par.tauV;                                         % [s]
Td = double(S.sl.plant.n_dead) * S.sl.plant.par.Ts;                % [s]
lag = charDriveLag([r.capFiles {r.file}], struct('quiet', true, 'accel', 1e6, 'decel', 1e6));
assert(all(abs([lag.captures.tauV] - tauV) < 0.003), 'lag %s s, plant %.4f', mat2str([lag.captures.tauV], 3), tauV);
assert(all(abs([lag.captures.Td] - Td) <= ccfg.scanTime + 1e-3), 'dead time %s s, plant %.3f', ...
    mat2str([lag.captures.Td], 3), Td);
assert(all([lag.captures.residual] < 0.03), 'lag fit residual %s', mat2str([lag.captures.residual], 3));
assert(~isempty(lag.runLog) && lag.runLog.nSteps >= 2, 'run log steps not found');
assert(abs(lag.runLog.tauV + lag.runLog.Td - (tauV + Td)) < 0.025, 'run log lag %.3f s, plant %.3f', ...
    lag.runLog.tauV + lag.runLog.Td, tauV + Td);
assert(lag.runLog.nStepsP >= 1, 'no pressurised step in the run log');
% The pressure follows the piston with no lag of its own, but the gas
% stiffens and dissolves over a capture, which the constant-stiffness model
% absorbs as a lag of either sign: about 30 ms for each second of capture.
% A capture is 500 scans.
biasMax = 0.03 * 500 * ccfg.scanTime;                              % [s]
capsP = lag.captures(~isnan([lag.captures.tauP]));
assert(numel(capsP) >= 1 && all([capsP.tauP] + [capsP.TdP] < tauV + Td + biasMax), ...
    'piston lag %s s, drive %.3f s', mat2str([capsP.tauP] + [capsP.TdP], 3), tauV + Td);
fprintf('ok  drive lag fit: captures %.1f ms and %.1f ms dead (plant %.1f, %.1f), run log %.0f ms, pressure %.1f ms\n', ...
    1000 * lag.tauV_s, 1000 * lag.Td_s, 1000 * tauV, 1000 * Td, ...
    1000 * (lag.runLog.tauV + lag.runLog.Td), 1000 * lag.tauP_s);

% The charge is gas, not liquid: the bulk modulus fit must reject it and
% attribute the compliance to gas. Its gas stands for all of the chamber's
% compliance, eqs. (4) and (42): the free gas (as it was before the
% ramps; it dissolves as the session runs), the saline and the hose, as
% the gas volume with that compliance at 2.5 MPa.
bk = charBulkModulus(r.file, struct('quiet', true, 'deadVolume_mL', 10, 'pressureRange', [2.0 3.1]));
Pabs = 2.5e6 + pl0.par.Patm;                                        % [Pa]
C = (pl0.n_tot - pl0.n_dis) * pl0.par.RT / Pabs^2 + (pl0.V_l + pl0.V_vir) / pl0.par.K ...
    + pl0.par.C_hose;                                               % [m^3/Pa]
vGas = C * Pabs * 1e6;                                              % [m^3 -> mL]
assert(~bk.pass && bk.K_lin_GPa < 0.5, 'gas charge accepted as a liquid (K %.3f GPa)', bk.K_lin_GPa);
assert(abs(bk.Vg_mL - vGas) / vGas < 0.3, 'gas volume %.2f mL, chamber holds about %.2f mL', bk.Vg_mL, vGas);
fprintf('ok  bulk modulus fit rejects the gas charge (K %.4f GPa, gas %.1f mL of %.1f)\n', ...
    bk.K_lin_GPa, bk.Vg_mL, vGas);

% --- One pass: levels, excursions in both directions, stops ---------------------
% The path is planned on the simulated chamber's own stiffness (its free
% gas and its liquid, eqs. (4) and (10) of models/README.md),
% so that each excursion stays within its budget.
opts = linkOpts(saveDir, @(sl) chamberPosition(sl, 1.7));
opts.probe = 'stops';
S = fakeProbeLink('state');
pl = S.sl.plant;
kPlant = @(Pq) pl.par.A ./ ((pl.n_tot - pl.n_dis) * pl.par.RT ./ (Pq * 1e6 + pl.par.Patm).^2 ...
    + pl.V_l / pl.par.K) * 1e-9;                                    % [Pa/m -> MPa/mm]
[path, plan] = charStopPath(struct('levels_MPa', [2 3], 'speeds', [0.5 2], 'budget_MPa', 1.0, ...
    'rampSpeed', 0.5, 'stiffness', kPlant, 'T0_s', tauV + Td, 'latency_s', 0.05, 'D_mm_s2', ccfg.velDecel, ...
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
    % The plant's drive does not slew: a stop is the dead time and the lag,
    % a distance linear in the speed, so the fit's deceleration term goes
    % large and its delay is the drive's: the lag plus the dead time less
    % the scan by which the logged command leads the drive, plus the scan
    % by which the probe's velocity feedback is older than its command (the
    % probe runs in PRG_Main, before the axis updates the feedback), less the
    % half scan the log's sum of v*Ts loses.
    T0 = tauV + Td - 0.5 * ccfg.scanTime;                           % [s]
    assert(abs(f{1}.T0_s - T0) < 0.003 && f{1}.D_mm_s2 > 5 * ccfg.velDecel, ...
        'fit D %.1f mm/s^2, T0 %.4f s; the drive stops in %.4f s, unslewed', f{1}.D_mm_s2, f{1}.T0_s, T0);
end
assert(rs.pass, 'stopping-distance checks failed');
fprintf('ok  one-pass path: %d steps, %d stops, D %.0f / %.0f mm/s^2 (unslewed), T0 %.1f ms (drive %.1f)\n', ...
    numel(path), numel(rs.stops), rs.fit.comp.D_mm_s2, rs.fit.ret.D_mm_s2, 1000 * rs.T0_s, ...
    1000 * (tauV + Td - 0.5 * ccfg.scanTime));

% --- Delivery: pressure mode, valve opens, the loop balances the outflow --------
% A delivery follows the hold that dissolves the charge, so the saline
% starts saturated at the delivery pressure and the gas no longer
% dissolves while the orifices are timed.
opts = linkOpts(saveDir, @(sl) dissolvedAt(sl, 2.0));
fakeProbeLink('valveAt', 12, 0.68);
r = charRun('simdeliver', {{'pressure', 2.0, 30}}, opts);
assert(isempty(r.abort), r.abort);
L = r.runLog;
inStep = L(:, STEP) == 1 & L(:, STATUS) == 5;
% The volume that left while the step ran (the outflow goes on after the
% stop). The simulated chamber holds a gas charge, so what left differs
% from the swept volume by the change of the free gas's volume, as the
% pressure settles and as the gas dissolves, a term the script's liquid
% correction cannot see; the check allows for it.
S = fakeProbeLink('state');
pl = S.sl.plant;
tEnd = max(L(inStep, T));
collected = interp1(S.collectLog(:, 1), S.collectLog(:, 2), tEnd) / 1000;
nFree = @(tq) interp1(S.collectLog(:, 1), S.collectLog(:, 3), tq, 'linear', 'extrap');   % [mol]
vGasAt = @(Pq, tq) nFree(tq) * pl.par.RT ./ (Pq * 1e6 + pl.par.Patm) * 1e6;          % [m^3 -> mL]
fl = charOrificeFlow(r.file, struct('quiet', true, 'tOpen', 12, 'collected_mL', collected));
assert(abs(fl.Cd - 0.68) / 0.68 < 0.06, 'Cd %.3f, valve set to 0.68', fl.Cd);
% The loop pulls the pressure back up during the window, so the piston also
% compresses the gas; with the chamber's equivalent bulk modulus (its
% volume over its compliance, eqs. (4) and (42)) the correction removes
% that and Cd comes back within 2 %.
late = inStep & L(:, T) > 15;
Pm = mean(L(late, P)); xm = mean(L(late, POS)); tm = mean(L(late, T));
Cm = nFree(tm) * pl.par.RT / (Pm * 1e6 + pl.par.Patm)^2 + (pl.V_l + pl.V_vir) / pl.par.K ...
    + pl.par.C_hose;                                                       % [m^3/Pa]
Kgas = (ccfg.pistonArea_mm2 * (ccfg.posEOT - xm) + ccfg.deadVolume_mL * 1000) * 1e-9 ...
    / Cm * 1e-6;                                                           % [mm^3 -> m^3, Pa -> MPa]
flk = charOrificeFlow(r.file, struct('quiet', true, 'tOpen', 12, 'K_Pa', Kgas * 1e6));
assert(abs(flk.Cd - 0.68) / 0.68 < 0.02, 'Cd %.3f with the gas correction, valve set to 0.68', flk.Cd);
pAt = @(tq) interp1(L(inStep, T), L(inStep, P), tq);
expansion = vGasAt(pAt(tEnd), tEnd) - vGasAt(pAt(12), 12);
expected = collected / (collected - expansion);
assert(abs(fl.gravimetric.ratio - expected) < 0.02, 'collected %.3f of swept, %.3f expected with %.2f mL of gas expansion', ...
    fl.gravimetric.ratio, expected, expansion);
assert(all(cellfun(@(c) c.pass, fl.checks(1:2))), 'Cd or its spread failed');
fl2 = charOrificeFlow(r.file, struct('quiet', true));
assert(abs(fl2.windows(1).t1 - fl.windows(1).t1) < 2, 'valve opening detected at %.1f s, set at 12 s', fl2.windows(1).t1);
fprintf('ok  delivery: Cd %.3f (%.3f with the gas correction), collected %.1f %% of swept (%.1f expected), opening found at %.1f s\n', ...
    fl.Cd, flk.Cd, 100 * fl.gravimetric.ratio, 100 * expected, fl2.windows(1).t1);

% --- Guards ----------------------------------------------------------------------
% From 9 MPa, ramping at 0.3 mm/s: the undersaturated charge dissolves
% fast at that pressure, which a slower ramp barely outruns.
opts = linkOpts(saveDir, @(sl) chamberPosition(sl, 9));
opts.pressureStop = 10.5;
r = charRun('simguard', {{'ramp', 0.3, 11.5, 20}}, opts);
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
if isa(x0, 'function_handle')
    fakeProbeLink('slave', @(sl) setfield(sl, 'ax', setfield(sl.ax, 'x', x0(sl))));
else
    fakeProbeLink('slave', @(sl) setfield(sl, 'ax', setfield(sl.ax, 'x', x0)));
end
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


function x = dissolvedAt(sl, P)
% The axis position at which the chamber holds P MPa with its saline
% saturated at P (Henry's law, eqs. (2) and (3) of
% models/README.md), after setting the dissolved gas so.
pl = sl.plant;
sl.plant.n_dis = min(pl.par.kH * (P * 1e6 + pl.par.Patm) * pl.V_l, pl.n_tot);   % [mol]
x = chamberPosition(sl, P);
end

