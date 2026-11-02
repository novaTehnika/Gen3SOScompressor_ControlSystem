function charFitsSynthetic()
%CHARFITSSYNTHETIC The characterisation fits on synthetic logs with
%   known parameters: charBulkModulus recovers K, the air pocket at the
%   pressure transducer (a fixed isothermal gas of Vair / 1000 at
%   atmosphere in every chamber), a residual gas bubble beyond it and
%   the split between liquid and series compliance from two fills;
%   charOrificeFlow recovers Cd and the square-root exponent from
%   deliveries at three pressures; charDriveLag recovers tauV and the dead
%   time from step probe captures, and roughly from 50 ms run logs,
%   including the piston-side lag from the pressure column through the
%   sensor chain; charScheduleCheck compares the schedule's stiffness bound
%   with a known stiffness; charStopDistance recovers the delay and the
%   deceleration of the stopping-distance law from captures and a run
%   log of stops in both directions; charDrivePath recovers the inertia,
%   the friction reversal, the backlash, the windup and the stiffness
%   (or its bound) of a synthetic two-mass drive, and gives the October
%   numbers on the committed logs. The logs are in frictionRun's
%   formats.
%   Run from the repository root:  addpath master-src/logic tests
%   Depends on: master-src/logic slave-src bench/characterisation bench/common bench/friction bench/captures

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'), fullfile(here, '..', 'bench', 'common'), fullfile(here, '..', 'bench', 'friction'));
cfg = charConfig();
A = cfg.pistonArea_mm2;
Patm = cfg.Patm_MPa;
dir = tempname();
mkdir(dir);
randn('seed', 7);

% --- Bulk modulus ---------------------------------------------------------------
K = 1300;                       % MPa
Vdead = 25e3;                   % mm^3
x0 = 285;                       % liquid-full at zero gauge
V0 = A * (cfg.posEOT - x0) + Vdead;
% Every chamber holds the air pocket at the transducer, a fixed isothermal
% gas of the configured volume at atmosphere (params.py V_air); a bubble
% of free gas comes on top of it where a case sets one.
Vair = 650;                     % mm^3 of air at atmosphere in the transducer passage: larger than
                                % the configured pocket so the fit resolves it, passed to every fit
VgBubble = 1000;                % mm^3 of free gas at atmosphere beyond the pocket
% Up and down ramps at 0.1 mm/s; x(P) from the compliances, then t from x.
% The pocket alone: the fit recovers the liquid K, and its gas term is the
% pocket, within the tolerance of the acceptance.
f1 = fullfile(dir, 'k_run_1.csv');
writeRamps(f1, K, V0, x0, Vair, 0, A, Patm);
r = charBulkModulus(f1, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
assert(abs(r.K_liq_GPa - K / 1000) / (K / 1000) < 0.02, 'liquid K %.3f GPa, set %.3f', r.K_liq_GPa, K / 1000);
assert(abs(r.K_lin_GPa - K / 1000) / (K / 1000) < 0.04, 'effective K %.3f GPa, set %.3f', r.K_lin_GPa, K / 1000);
assert(abs(r.K_Pa - r.K_liq_GPa * 1e9) < 1, 'r.K_Pa is not the liquid K');
P0 = r.segments(1).P0;
VgAtP0 = Vair * Patm / (P0 + Patm) / 1000;
assert(abs(r.segments(1).airAtP0_mL - VgAtP0) < 1e-12, 'the pocket at P0 is %.5f mL, expected %.5f', ...
    r.segments(1).airAtP0_mL, VgAtP0);
assert(abs(r.Vg_mL - VgAtP0) < 0.5 * VgAtP0, 'gas %.4f mL fitted, pocket %.4f mL at %.1f MPa', r.Vg_mL, VgAtP0, P0);
assert(mean([r.segments.Vair_fit_mL]) > 0.5 * Vair / 1000 && mean([r.segments.Vair_fit_mL]) < 1.5 * Vair / 1000, ...
    'pocket at atmosphere %.3f mL fitted, %.3f mL set', mean([r.segments.Vair_fit_mL]), Vair / 1000);
assert(r.gasExcess_mL < 0.3 && r.pass, 'the pocket alone failed the checks (excess %.3f mL)', r.gasExcess_mL);
assert(numel(r.segments) == 2 && r.hysteresis < 0.02);
fprintf('ok  bulk modulus: liquid K %.3f GPa (set %.3f), pocket %.3f mL at atmosphere fitted (set %.3f), gas %.4f mL at %.1f MPa (set %.4f)\n', ...
    r.K_liq_GPa, K / 1000, mean([r.segments.Vair_fit_mL]), Vair / 1000, r.Vg_mL, P0, VgAtP0);

% A bubble of free gas beyond the pocket: the fit finds the sum, the
% liquid K is still recovered, and the gas excess over the pocket is the
% bubble.
f1b = fullfile(dir, 'k_run_1b.csv');
writeRamps(f1b, K, V0, x0, Vair + VgBubble, 0, A, Patm);
r = charBulkModulus(f1b, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
bubbleAtP0 = VgBubble * Patm / (P0 + Patm) / 1000;
assert(abs(r.K_liq_GPa - K / 1000) / (K / 1000) < 0.03, 'liquid K %.3f GPa with a bubble, set %.3f', r.K_liq_GPa, K / 1000);
assert(abs(mean([r.segments.VgExcess_mL]) - bubbleAtP0) < 0.5 * bubbleAtP0, ...
    'gas beyond the pocket %.4f mL, bubble %.4f mL at %.1f MPa', mean([r.segments.VgExcess_mL]), bubbleAtP0, P0);
fprintf('ok  bulk modulus: a bubble of %.4f mL at %.1f MPa beyond the pocket is found as %.4f mL, liquid K %.3f GPa\n', ...
    bubbleAtP0, P0, mean([r.segments.VgExcess_mL]), r.K_liq_GPa);

% Gas well beyond the pocket (30 mL at atmosphere) fails the gas check;
% the 0.3 mL tolerance is an option.
f1c = fullfile(dir, 'k_run_1c.csv');
writeRamps(f1c, K, V0, x0, Vair + 30e3, 0, A, Patm);
r = charBulkModulus(f1c, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
failedNames = cellfun(@(c) c.name, r.checks(cellfun(@(c) ~c.pass, r.checks)), 'UniformOutput', false);
assert(any(strcmp(failedNames, 'gasExcess_mL')), 'failed checks: %s', strjoin(failedNames, ', '));
r2 = charBulkModulus(f1c, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000, 'airTolerance_mL', 5));
assert(all(cellfun(@(c) c.pass, r2.checks(strcmp(cellfun(@(c) c.name, r2.checks, 'UniformOutput', false), 'gasExcess_mL')))), ...
    'a gas excess inside airTolerance_mL = 5 failed (%.3f mL)', r2.gasExcess_mL);
fprintf('ok  bulk modulus: %.2f mL of gas beyond the pocket at %.1f MPa fails the gas check (%.2f mL excess), passes with a 5 mL tolerance\n', ...
    30e3 * Patm / (P0 + Patm) / 1000, P0, r.gasExcess_mL);

% The pocket alone and the clean-run checks pass.
f2 = fullfile(dir, 'k_run_2.csv');
writeRamps(f2, K, V0, x0, Vair, 0, A, Patm);
r = charBulkModulus(f2, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
assert(r.pass, 'clean liquid run failed the checks');
assert(abs(r.K_Pa - K * 1e6) / (K * 1e6) < 0.02);
fprintf('ok  bulk modulus: clean run passes, K %.3g Pa\n', r.K_Pa);

% A log recorded with another shunt setting (434 ohm; the present GVL has
% 421.4) reads its pressure scaled by the ratio of the two: opts.logShuntOhms
% rescales it back, and without it K is off by that ratio.
gv0 = benchGvl();
mAperMPa = gv0.G_cfgPressureSpanMa / (gv0.G_cfgPressureRangeMax - gv0.G_cfgPressureRangeMin);
d2 = benchReadCsv(f2);
mAtrue = gv0.G_cfgPressureZeroMa + (d2.pressure - gv0.G_cfgPressureRangeMin) * mAperMPa;      % [mA]
mAlogged = mAtrue * gv0.G_cfgPressureShuntOhms / 434;                                          % [mA]
Plogged = gv0.G_cfgPressureRangeMin + (mAlogged - gv0.G_cfgPressureZeroMa) / mAperMPa;        % [MPa]
f2s = fullfile(dir, 'k_run_2_shunt434.csv');
writeRun(f2s, [d2.t d2.step d2.status d2.mode d2.ref d2.position d2.velocity d2.torque Plogged ...
    d2.inMotion d2.inBand d2.exchangeMs]);
rs = charBulkModulus(f2s, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000, 'logShuntOhms', 434));
ru = charBulkModulus(f2s, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
assert(abs(rs.K_Pa - K * 1e6) / (K * 1e6) < 0.02, 'rescaled K %.3g Pa, set %.3g', rs.K_Pa, K * 1e6);
assert(abs(ru.K_Pa - K * 1e6) / (K * 1e6) > 0.5 * abs(1 - gv0.G_cfgPressureShuntOhms / 434), ...
    'K without the rescale %.3g Pa is not off by the shunt ratio', ru.K_Pa);
fprintf('ok  bulk modulus: a log from the 434 ohm shunt, K %.3g Pa rescaled (set %.3g), %.3g without\n', ...
    rs.K_Pa, K * 1e6, ru.K_Pa);

% Two fills with a series compliance of 200 N/um split liquid and structure.
ks = 200e3;                     % N/mm
f3 = fullfile(dir, 'k_fill300.csv');
f4 = fullfile(dir, 'k_fill150.csv');
writeRamps(f3, K, V0, x0, Vair, ks, A, Patm);
x1 = cfg.posEOT - (150e3 - Vdead) / A;
writeRamps(f4, K, 150e3, x1, Vair, ks, A, Patm);
r = charBulkModulus({f3, f4}, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
Keff = 1 / (1 / K + A^2 / (ks * V0));
assert(abs(r.segments(1).K_lin_GPa - Keff / 1000) / (Keff / 1000) < 0.03, ...
    'effective K at 300 mL %.3f GPa, expected %.3f with the series compliance', r.segments(1).K_lin_GPa, Keff / 1000);
assert(~isempty(r.split), 'two fills gave no split');
assert(abs(r.split.K_GPa - K / 1000) / (K / 1000) < 0.05, 'split liquid K %.3f GPa, set %.3f', r.split.K_GPa, K / 1000);
assert(abs(r.split.ks_N_per_um - ks / 1000) / (ks / 1000) < 0.1, 'series stiffness %.0f N/um, set %.0f', ...
    r.split.ks_N_per_um, ks / 1000);
fprintf('ok  bulk modulus: two fills give liquid %.2f GPa and series %.0f N/um (effective %.2f GPa at 300 mL)\n', ...
    r.split.K_GPa, r.split.ks_N_per_um, r.segments(1).K_lin_GPa);
% The pocket is the same volume at both fills: it leaves the slope of the
% compliance against V (the liquid K) alone and adds to the intercept,
% which the fit corrects for; without the correction the series stiffness
% reads low by the pocket's share of the intercept.
r0 = charBulkModulus({f3, f4}, struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', 0));
assert(abs(r0.split.K_GPa - r.split.K_GPa) < 0.01 * r.split.K_GPa, ...
    'the pocket moved the split liquid K from %.3f to %.3f GPa', r0.split.K_GPa, r.split.K_GPa);
assert(r0.split.ks_N_per_um < r.split.ks_N_per_um - 3 && ...
    abs(r.split.ks_N_per_um - ks / 1000) < abs(r0.split.ks_N_per_um - ks / 1000), ...
    'series stiffness %.0f N/um corrected, %.0f uncorrected, set %.0f', r.split.ks_N_per_um, r0.split.ks_N_per_um, ks / 1000);
fprintf('ok  bulk modulus: the pocket leaves the two-fill liquid K at %.3f GPa; series %.0f N/um corrected, %.0f without (set %.0f)\n', ...
    r.split.K_GPa, r.split.ks_N_per_um, r0.split.ks_N_per_um, ks / 1000);

% File patterns: the fits take a pattern like a list of names
rp = charBulkModulus(fullfile(dir, 'k_fill*.csv'), struct('quiet', true, 'deadVolume_mL', Vdead / 1000, 'airVolume_mL', Vair / 1000));
assert(numel(rp.segments) == numel(r.segments) && abs(rp.split.K_GPa - r.split.K_GPa) < 1e-9, 'pattern differs from the list');
failed = false;
try
    benchFiles(fullfile(dir, 'no_such_*.csv'));
catch
    failed = true;
end
assert(failed, 'a pattern that matches nothing was accepted');
fprintf('ok  file patterns: k_fill*.csv gives the two fills, a pattern with no match is an error\n');

% --- Schedule check ---------------------------------------------------------------
% The slave's stiffness bound against a liquid-only run of known stiffness
% (the clean run above, K 1.3 GPa, the fill as the saline volume): with a
% largest plausible bulk modulus of 2 GPa the bound is 2/1.3 of the
% measured stiffness and the check passes; with the GVL's 1 GPa, below the
% run's own, the bound is below the measured stiffness and the check fails.
fillSaline = V0 / 1000;                                             % [mm^3 -> mL]
bulk = struct('deadVolume_mL', Vdead / 1000);
sc = charScheduleCheck(f2, struct('quiet', true, 'salineVolume_mL', fillSaline, 'bulk', bulk, ...
    'settings', struct('G_cfgPressureSchedBulkModulus', 2e9)));
assert(sc.pass && numel(sc.rows) == 2, 'bound below the stiffness with K_cfg above the true K');
assert(all(abs([sc.rows.ratio] - 2000 / K) < 0.04 * 2000 / K), 'bound / measured %s, expected %.3f', ...
    mat2str([sc.rows.ratio], 4), 2000 / K);
gv = benchGvl();
wcExpected = gv.G_cfgPressureSchedCrossover * 1300 / 2000;         % [rad/s] Kp k_true = wc k_true / k_bound
assert(all(abs([sc.rows.wc] - wcExpected) < 0.04 * wcExpected), 'crossover %s rad/s, expected %.2f', ...
    mat2str([sc.rows.wc], 4), wcExpected);
sc = charScheduleCheck(f2, struct('quiet', true, 'salineVolume_mL', fillSaline, 'bulk', bulk, ...
    'settings', struct('G_cfgPressureSchedBulkModulus', 1e9)));
assert(~sc.pass && all([sc.rows.ratio] < 1) && abs(sc.minRatio - 1000 / K) < 0.04 * 1000 / K, ...
    'a bound below the stiffness was accepted (ratio %.3f)', sc.minRatio);
fprintf('ok  schedule check: bound / measured %.2f with 2 GPa (set 1.3 GPa), %.2f with 1 GPa fails\n', ...
    2000 / K, sc.minRatio);

% The committed K300 logs (proximal valve shut, 343 mL, logged at 434 ohm):
% the GVL's bound, with its 1 GPa bulk modulus above the measured 0.72 GPa,
% stands above the measured stiffness of every ramp.
cap = fullfile(here, '..', 'bench', 'captures');
k300 = {fullfile(cap, 'K300_char_20261026_192617.csv'), fullfile(cap, 'K300_char_20261026_192751.csv'), ...
        fullfile(cap, 'K300_char_20261026_193002.csv')};
sc = charScheduleCheck(k300, struct('quiet', true, 'bulk', struct('logShuntOhms', 434, 'deadVolume_mL', 50)));
assert(sc.pass && numel(sc.rows) == 24 && sc.minRatio > 1.15 && max([sc.rows.ratio]) < 1.6, ...
    'K300 logs: bound / measured %.2f to %.2f over %d ramps', sc.minRatio, max([sc.rows.ratio]), numel(sc.rows));
fprintf('ok  schedule check on the K300 logs: bound %.1f MPa/mm over measured %.1f to %.1f, ratio %.2f to %.2f\n', ...
    sc.rows(1).kBound, min([sc.rows.dPdx]), max([sc.rows.dPdx]), sc.minRatio, max([sc.rows.ratio]));

% --- Orifice flow -------------------------------------------------------------------
Cd = 0.68;
files = {};
collected = [];
for P = [4 7 10]
    f = fullfile(dir, sprintf('q_run_%d.csv', P));
    collected(end + 1) = writeDelivery(f, P, Cd, K, V0, x0, cfg); %#ok<AGROW>
    files{end + 1} = f; %#ok<AGROW>
end
r = charOrificeFlow(files, struct('quiet', true, 'K_Pa', K * 1e6, 'deadVolume_mL', Vdead / 1000, ...
    'collected_mL', collected));
assert(abs(r.Cd - Cd) / Cd < 0.02, 'Cd %.3f, set %.3f', r.Cd, Cd);
assert(abs(r.exponent - 0.5) < 0.03, 'exponent %.3f', r.exponent);
assert(all(abs([r.gravimetric.ratio] - 1) < 0.02), 'collected against swept: %s', mat2str([r.gravimetric.ratio], 3));
assert(r.pass, 'delivery checks failed');
fprintf('ok  orifice flow: Cd %.3f (set %.3f), exponent %.3f, collected within %.1f %%\n', r.Cd, Cd, ...
    r.exponent, 100 * max(abs([r.gravimetric.ratio] - 1)));

% --- Drive lag ---------------------------------------------------------------------
tau = 0.020; Td = 0.006; accel = 10;
c1 = fullfile(dir, 'lag_cap_1_up.csv');
c2 = fullfile(dir, 'lag_cap_2_down.csv');
writeCapture(c1, 0, 1.0, tau, Td, accel, 0, 0, cfg);
writeCapture(c2, 1.0, 0, tau, Td, accel, 0, 0, cfg);
r = charDriveLag({c1, c2}, struct('quiet', true, 'accel', accel, 'decel', accel));
assert(all(abs([r.captures.tauV] - tau) < 0.002), 'tauV %s, set %.3f', mat2str([r.captures.tauV], 3), tau);
assert(all(abs([r.captures.Td] - Td) < 0.002), 'Td %s, set %.3f', mat2str([r.captures.Td], 3), Td);
assert(all(isnan([r.captures.tauP])) && isnan(r.tauP_s), 'a pressure fit on a dry capture');
assert(r.pass);
fprintf('ok  drive lag: tauV %.1f / %.1f ms (set %.0f), Td %.1f / %.1f ms (set %.0f) from scan-spaced captures\n', ...
    1000 * [r.captures.tauV], 1000 * tau, 1000 * [r.captures.Td], 1000 * Td);

% A capture whose t column is written at half the real row spacing, as
% the 26 October captures were, is fitted at the spacing
% opts.rowPeriod_s gives.
c4 = fullfile(dir, 'lag_cap_4_mislabelled.csv');
lines = strsplit(strtrim(fileread(c1)), "\n");
fid = fopen(c4, 'w');
fprintf(fid, '%s\n', lines{1});
for i = 2:numel(lines)
    f = strsplit(lines{i}, ',');
    f{1} = sprintf('%.3f', (i - 2) * cfg.scanTime / 2);
    fprintf(fid, '%s\n', strjoin(f, ','));
end
fclose(fid);
r = charDriveLag(c4, struct('quiet', true, 'accel', accel, 'decel', accel, 'rowPeriod_s', cfg.scanTime));
assert(abs(r.captures.tauV - tau) < 0.002 && abs(r.captures.Td - Td) < 0.002, ...
    'override: tauV %.4f, Td %.4f, set %.3f, %.3f', r.captures.tauV, r.captures.Td, tau, Td);
fprintf('ok  drive lag: rows labelled at half spacing fitted with rowPeriod_s: tauV %.1f ms, Td %.1f ms (set %.0f, %.0f)\n', ...
    1000 * r.captures.tauV, 1000 * r.captures.Td, 1000 * tau, 1000 * Td);

% Under pressure the capture's pressure channel gives the lag at the
% piston through the median, low-pass and pipeline delay, here set larger
% than the motor's.
c3 = fullfile(dir, 'lag_cap_3_wet.csv');
writeCapture(c3, 0, 0.3, 0.035, Td, accel, 11.0, 4.0, cfg);
r = charDriveLag(c3, struct('quiet', true, 'accel', accel, 'decel', accel));
% Behind the sensor chain the lag and the dead time trade against each
% other; their sum is what the fit determines.
assert(abs(r.captures.tauP + r.captures.TdP - (0.035 + Td)) < 0.004, 'tauP + TdP %.4f, set %.3f', ...
    r.captures.tauP + r.captures.TdP, 0.035 + Td);
assert(abs(r.captures.k_MPa_mm - 11.0) / 11 < 0.05, 'stiffness %.2f, set 11', r.captures.k_MPa_mm);
assert(abs(r.tauP_s - (0.035 + Td)) < 0.004 && r.pass);
fprintf('ok  drive lag at the piston from the capture pressure: %.1f ms incl. dead time (set 41), stiffness %.1f MPa/mm (set 11)\n', ...
    1000 * r.tauP_s, r.captures.k_MPa_mm);

% The 50 ms run log alone: lag plus dead time to about 25 ms, from the
% velocity unpressurised and from the pressure of pressurised steps, where
% the piston lags more than the motor.
f5 = fullfile(dir, 'lag_run.csv');
writeStepLog(f5, tau, Td, accel, 0, 0, cfg);
r = charDriveLag(f5, struct('quiet', true, 'accel', accel, 'decel', accel));
assert(~isempty(r.runLog) && r.runLog.nSteps == 6);
assert(abs(r.tauV_s - (tau + Td)) < 0.025, 'run log lag %.3f s, set %.3f', r.tauV_s, tau + Td);
assert(isnan(r.tauP_s), 'a pressure lag from an unpressurised log');
f6 = fullfile(dir, 'lag_run_wet.csv');
writeStepLog(f6, 0.040, Td, accel, 11.0, 4.0, cfg);
r = charDriveLag(f6, struct('quiet', true, 'accel', accel, 'decel', accel));
assert(r.runLog.nStepsP == 6, '%d pressurised steps found', r.runLog.nStepsP);
assert(abs(r.tauP_s - 0.046) < 0.025, 'piston lag %.3f s, set 0.046', r.tauP_s);
assert(abs(r.runLog.k_MPa_mm - 11) / 11 < 0.1, 'stiffness %.2f, set 11', r.runLog.k_MPa_mm);
fprintf('ok  drive lag from the 50 ms log: dry %.0f ms (set %.0f), piston side %.0f ms (set 46), stiffness %.1f MPa/mm\n', ...
    1000 * r.tauV_s, 1000 * (tau + Td), 1000 * r.tauP_s, r.runLog.k_MPa_mm);

% --- Stopping distance ------------------------------------------------------------
% Stops from three speeds in both directions, the piston decelerating at D
% after a delay T0, in captures and in one run log; the fit recovers both.
T0 = 0.03;
D = 80;
speeds = [1 2 5];
files = {};
for v = speeds
    for dirSign = [1 -1]
        f = fullfile(dir, sprintf('stop_%g_%d.csv', v, dirSign));
        writeStopCapture(f, dirSign * v, T0, D, 5.0, cfg);
        files{end + 1} = f;                                             %#ok<AGROW>
    end
end
fl = fullfile(dir, 'stop_log.csv');
writeStopLog(fl, speeds, T0, D, 5.0, cfg);
files{end + 1} = fl;
rs = charStopDistance(files, struct('quiet', true));
assert(numel(rs.stops) == 12, '%d stops found, 12 written', numel(rs.stops));
for f = {rs.fit.comp, rs.fit.ret}
    assert(abs(f{1}.D_mm_s2 / D - 1) < 0.1 && abs(f{1}.T0_s - T0) < 0.01, ...
        'fit D %.1f (set %.0f), T0 %.3f (set %.3f)', f{1}.D_mm_s2, D, f{1}.T0_s, T0);
end
caps = rs.stops(strcmp({rs.stops.source}, 'capture'));
for s = caps
    v = abs(s.v_mm_s);
    % The trace is summed as rectangles of one scan, which counts the
    % travel v*Ts/2 short of the continuous ramp.
    xSet = v * T0 + v^2 / (2 * D) - v * cfg.scanTime / 2;
    assert(abs(s.x_mm - xSet) < 0.01, 'capture stop %.3f mm at %g mm/s, expected %.3f', s.x_mm, v, xSet);
end
assert(rs.pass, 'stopping-distance checks failed on the synthetic stops');
fprintf('ok  stopping distance: D %.1f / %.1f mm/s^2 (set %.0f), T0 %.0f / %.0f ms (set %.0f)\n', ...
    rs.fit.comp.D_mm_s2, rs.fit.ret.D_mm_s2, D, 1000 * rs.fit.comp.T0_s, 1000 * rs.fit.ret.T0_s, 1000 * T0);

% --- Drive path: the October analyses --------------------------------------------
% The three analyses on the committed logs, with the numbers the README
% reports: the inertia from the open-chamber velocity steps, the friction
% reversal under load and the stiffness bound from the K300 ramps, and the
% hose-open run as the contrast.
steps26 = {fullfile(cap, 's1_cap_steps_*.csv'), fullfile(cap, 's2_cap_steps_*.csv')};
rp = charDrivePath(steps26, struct('quiet', true));
assert(rp.inertia.n == 74, '%d steps', rp.inertia.n);
assert(abs(rp.inertia.J_pct_per_mm_s2 - 0.418) < 0.005 && abs(rp.inertia.mass_kg / 4.6e5 - 1) < 0.01, ...
    'J %.4f %% per mm/s^2, %.3g kg', rp.inertia.J_pct_per_mm_s2, rp.inertia.mass_kg);
Jsize = [rp.inertia.sizes.J];
assert(isequal([rp.inertia.sizes.size], [0.3 1 2.5]) && all(abs(Jsize - [0.379 0.412 0.440]) < 0.005), ...
    'J by step size %s', mat2str(Jsize, 3));
assert(rp.pass, 'the October steps failed the inertia checks');
fprintf('ok  drive path, October steps: J %.3f / %.3f / %.3f %%/(mm/s^2) at 0.3 / 1 / 2.5 mm/s, %.3f overall = %.2g kg (lossless)\n', ...
    Jsize, rp.inertia.J_pct_per_mm_s2, rp.inertia.mass_kg);

rk = charDrivePath(k300, struct('quiet', true));
assert(numel(rk.friction.logs) == 3 && all([rk.friction.logs.nPairs] == 7));
dts = [rk.friction.logs.medDT_pct];
assert(all(abs(dts - [25.1 21.6 22.5]) < 0.2), 'median torque reversal %s', mat2str(dts, 3));
i4 = find(rk.friction.P_MPa == 4);
i9 = find(rk.friction.P_MPa == 9);
dT4 = arrayfun(@(l) l.dT_pct(i4), rk.friction.logs);
dT9 = arrayfun(@(l) l.dT_pct(i9), rk.friction.logs);
assert(all(dT4 > 16.5 & dT4 < 18.5) && all(dT9 > 28.5 & dT9 < 32.5), ...
    'torque reversal %s %% at 4 MPa, %s %% at 9', mat2str(dT4, 3), mat2str(dT9, 3));
dF = [dT4 dT9] * rk.newtonPerPct / 1e3;
assert(min(dF(1:3)) > 18 && max(dF(1:3)) < 21 && min(dF(4:6)) > 31 && max(dF(4:6)) < 36, ...
    'force reversal %s kN', mat2str(dF, 3));
bnd = [rk.friction.logs.bound_N_per_um];
assert(all(abs(bnd ./ [5513 4747 4948] - 1) < 0.01), 'stiffness bounds %s N/um', mat2str(bnd, 4));
assert(rk.stiffness.bound_N_per_um > 4500 && rk.stiffness.bound_N_per_um < 5000 && ~rk.stiffness.resolved, ...
    'bound %.0f N/um', rk.stiffness.bound_N_per_um);
assert(all([rk.friction.logs.medDX_um] < 3), 'offsets %s um', mat2str([rk.friction.logs.medDX_um], 2));
assert(abs(rk.friction.mu - 0.41) < 0.03 && abs(rk.friction.Fc_N / 1e3 - 3.8) < 0.3, ...
    'mu %.2f, Fc %.2f kN', rk.friction.mu, rk.friction.Fc_N / 1e3);
assert(rk.pass);
fprintf('ok  drive path, October K300: torque reversal %.1f..%.1f %% at 4 MPa and %.1f..%.1f %% at 9 (%.0f..%.0f kN and %.0f..%.0f kN lossless), k_s > %.0f N/um\n', ...
    min(dT4), max(dT4), min(dT9), max(dT9), min(dF(1:3)), max(dF(1:3)), min(dF(4:6)), max(dF(4:6)), rk.stiffness.bound_N_per_um);
rh = charDrivePath(fullfile(cap, 'K300_char_20261026_193603.csv'), struct('quiet', true));
assert(rh.friction.logs.medDX_um > 30 && rh.friction.logs.medDX_um < 50 && rh.stiffness.resolved && ~rh.pass, ...
    'hose-open offset %.1f um', rh.friction.logs.medDX_um);
assert(abs(rh.friction.logs.medDT_pct / 23.4 - 1) < 0.02);
fprintf('ok  drive path, October hose-open run: offset %.0f um, the hose and not the drive; it fails the offset check\n', ...
    rh.friction.logs.medDX_um);

% --- Drive path: synthetic two-mass drive ----------------------------------------
% A velocity-controlled motor (dead time Td, lag tauV, reflected inertia mr)
% drives a piston through a backlash and a spring ks; the piston sticks
% unless the force through the path exceeds the force it feels, the chamber
% pressure and its friction (Coulomb plus load-dependent mu P A); the
% motor has friction of its own. The chamber is liquid, at the stiffness of
% the K300 runs. The torque has white noise and a ripple that goes with the
% motor position, the pressure goes through the slave's sensor chain, and
% the velocity and position carry the drive's resolution.
fcfg = frictionBenchConfig();
nPP = fcfg.newtonPerPct;                                    % [N per % rated]
mrTrue = 0.42;                                              % [% per mm/s^2]
base = struct('Ts', cfg.scanTime, 'sub', 8, 'nPP', nPP, 'areaN', 3431, 'tau', 0.0182, 'Td', 0.0228, ...
    'kch', 0, 'P0', 0, 'b_um', 0, 'ks_N_um', 8000, 'Fcp', 2500, 'mu', 0.2, 'Fcm', 3000, ...
    'J', mrTrue, 'noiseV', 0.0005, 'noiseT', 0.3, 'ripple', 0.8, 'noiseX', 0.0003, 'noiseP', 0.002, ...
    'x0', 280, 'filterTau', cfg.filterTau, 'medianSize', cfg.medianSize, 'pipeline', cfg.pipelineScans);
fmTrue = base.Fcm / nPP;                                    % [%] motor-side friction

% Inertia from steps: 0 -> v -> 2v -> v -> 0 and the same downward, at three
% sizes, as the friction bench's steps set.
randn('seed', 21);
fs = {};
for v = [0.3 1 2.5]
    seqs = v * [0 1; 1 2; 2 1; 1 0; 0 -1; -1 -2; -2 -1; -1 0];
    for q = 1:8
        d = driveCapture(base, seqs(q, :), [0 0.1], 500);
        fs{end + 1} = writeDriveCapture(dir, sprintf('step_%g_%d', v, q), d); %#ok<AGROW>
    end
end
rp = charDrivePath(fs, struct('quiet', true));
assert(rp.inertia.n == 24 && abs(rp.inertia.J_pct_per_mm_s2 / mrTrue - 1) < 0.06, ...
    'inertia %.4f %% per mm/s^2 from %d steps, set %.2f', rp.inertia.J_pct_per_mm_s2, rp.inertia.n, mrTrue);
big = rp.inertia.sizes([rp.inertia.sizes.size] >= 1);
assert(all(abs([big.J] / mrTrue - 1) < 0.06), 'J at the larger steps %s', mat2str([big.J], 3));
assert(abs(rp.inertia.mass_kg / (mrTrue * nPP * 1e3) - 1) < 0.06 && rp.pass);
fprintf('ok  drive path, synthetic steps: J %.3f %% per mm/s^2 (set %.2f), %.3g kg (set %.3g)\n', ...
    rp.inertia.J_pct_per_mm_s2, mrTrue, rp.inertia.mass_kg, mrTrue * nPP * 1e3);

% Reversal, open chamber, a stiff path with 12 um of backlash: the
% backlash and the motor-side friction come out of the plateau.
randn('seed', 22);
bTrue = 12;                                                 % [um]
par = base; par.b_um = bTrue;
fr = {};
for v = [0.3 1]
    for q = 1:6
        sgn = 1 - 2 * mod(q, 2);
        d = driveCapture(par, [sgn * v, -sgn * v], [0 0.1], 500);
        fr{end + 1} = writeDriveCapture(dir, sprintf('rev_%g_%d', v, q), d); %#ok<AGROW>
    end
end
rr = charDrivePath(fr, struct('quiet', true, 'inertia_pct', mrTrue));
rv = rr.reversal;
assert(numel(rv.rec) == 12 && all([rv.rec.ok]), 'a reversal was not analysed');
assert(abs(rv.backlash_um - bTrue) < 3, 'backlash %.1f um, set %g', rv.backlash_um, bTrue);
assert(abs(rv.motorFriction_pct - fmTrue) < 0.5, 'motor-side friction %.2f %%, set %.2f', rv.motorFriction_pct, fmTrue);
assert(rv.groups(1).speed < 0.4 && rv.groups(1).nPlateau >= 5 && abs(rv.groups(1).b_um - bTrue) < 3, ...
    'at 0.3 mm/s: %d of 6 with a plateau, backlash %.1f um', rv.groups(1).nPlateau, rv.groups(1).b_um);
assert(abs(rv.motorFriction_kN * 1e3 - base.Fcm) < 0.5 * nPP, 'motor-side friction %.2f kN', rv.motorFriction_kN);
assert(all(abs([rv.rec.dT_pct] - 2 * (base.Fcm + base.Fcp) / nPP) < 1.5), ...
    'torque reversal %s %% against %.1f', mat2str([rv.rec.dT_pct], 3), 2 * (base.Fcm + base.Fcp) / nPP);
fprintf('ok  drive path, synthetic reversals: backlash %.1f um (set %g), motor-side friction %.2f %% = %.2f kN (set %.2f %%), plateau in %d of %d\n', ...
    rv.backlash_um, bTrue, rv.motorFriction_pct, rv.motorFriction_kN, fmTrue, sum([rv.rec.plateau]), numel(rv.rec));

% Without backlash no flat segment is found, and the travel of the
% transition bounds it.
randn('seed', 23);
par = base; par.b_um = 0;
fr0 = {};
for v = [0.3 1]
    for q = 1:6
        sgn = 1 - 2 * mod(q, 2);
        d = driveCapture(par, [sgn * v, -sgn * v], [0 0.1], 500);
        fr0{end + 1} = writeDriveCapture(dir, sprintf('rev0_%g_%d', v, q), d); %#ok<AGROW>
    end
end
rr = charDrivePath(fr0, struct('quiet', true, 'inertia_pct', mrTrue));
rv = rr.reversal;
assert(isnan(rv.backlash_um) && rv.backlashBound_um < 8 && sum([rv.rec.plateau]) <= 2, ...
    'backlash %.1f um, bound %.1f um, %d plateaus without backlash', rv.backlash_um, rv.backlashBound_um, sum([rv.rec.plateau]));
fprintf('ok  drive path, synthetic reversals with no backlash: none reported, the transition is within %.1f um\n', rv.backlashBound_um);

% Dither, closed liquid chamber (7.2 MPa/mm), at 1, 5 and 9 MPa, triangles of
% +-0.15 mm at 0.2 mm/s with legs of 1.5 s: captures at the top and at the
% bottom turn of each level (the next reversal follows 1.5 s later, inside the
% same capture, and is not part of the fit).
%   a soft path, 0.4 kN/um: windup of tens of um at the force reversal
randn('seed', 24);
par = base; par.kch = 7.2; par.ks_N_um = 400; par.b_um = 0;
fdS = ditherSet(dir, 'ditS', par, [1 5 9]);
rd = charDrivePath(fdS, struct('quiet', true, 'inertia_pct', mrTrue));
dt = rd.dither;
assert(all([dt.rec.ok]) && isnan(dt.backlash_um) && all([dt.groups.b_um] == 0), ...
    'backlash in a dither with none: %s', mat2str([dt.groups.b_um], 3));
hi = dt.groups([dt.groups.P_MPa] > 3);
ksRes = [hi.ks_N_per_um];
assert(numel(hi) >= 4 && all(abs(ksRes / par.ks_N_um - 1) < 0.45) && abs(median(ksRes) / par.ks_N_um - 1) < 0.2, ...
    'stiffness %s N/um, set %g', mat2str(ksRes, 3), par.ks_N_um);
assert(all(abs([dt.rec.dP_MPa]) < 0.1), 'the pressure moved by %.3f MPa during a windup', max(abs([dt.rec.dP_MPa])));
[~, w] = max([hi.P_MPa]);
fTrue = base.Fcp + par.mu * hi(w).P_MPa * base.areaN;       % [N] piston-side friction at the turn
wTrue = 2 * fTrue / par.ks_N_um;                            % [N / (N/um) -> um] windup over the force reversal
assert(abs(hi(w).windup_um - wTrue) < 0.4 * wTrue, 'windup %.1f um at %.1f MPa, expected %.1f', ...
    hi(w).windup_um, hi(w).P_MPa, wTrue);
fprintf('ok  drive path, synthetic dither on a soft path: k_s %s N/um (set %g) at %s MPa, windup %.0f um (%.0f expected) at %.1f MPa, no backlash\n', ...
    mat2str(round(ksRes)), par.ks_N_um, mat2str([hi.P_MPa], 3), hi(w).windup_um, wTrue, hi(w).P_MPa);

%   backlash of 20 um on a 2 kN/um path: where the pressure force is below
%   the friction the force through the path passes zero and the flat
%   segment shows (the lowest turn); where it is above, there is none
randn('seed', 25);
par = base; par.kch = 7.2; par.ks_N_um = 2000; par.b_um = 20;
fdB = ditherSet(dir, 'ditB', par, [1 5 9]);
rd = charDrivePath(fdB, struct('quiet', true, 'inertia_pct', mrTrue));
dt = rd.dither;
lo = dt.groups([dt.groups.P_MPa] < 0.5);
hiB = dt.groups([dt.groups.P_MPa] > 3);
assert(numel(lo) == 1 && lo.nPlateau == lo.n && abs(lo.b_um - par.b_um) < 6, ...
    'low-pressure turn: %d plateaus of %d, backlash %.1f um (set %g)', lo.nPlateau, lo.n, lo.b_um, par.b_um);
assert(all([hiB.b_um] == 0), 'backlash %s um in the loaded turns', mat2str([hiB.b_um], 3));
fprintf('ok  drive path, synthetic dither with %g um of backlash: %.1f um at %.2f MPa (%d of %d), none at %s MPa\n', ...
    par.b_um, lo.b_um, lo.P_MPa, lo.nPlateau, lo.n, mat2str([hiB.P_MPa], 3));

%   a path stiffer than the travel resolves: the windup is within
%   opts.res_um, the stiffness is a bound, and the bound is below it
randn('seed', 26);
par = base; par.kch = 7.2; par.ks_N_um = 30000; par.b_um = 0;
fdH = ditherSet(dir, 'ditH', par, [5 9]);
rd = charDrivePath(fdH, struct('quiet', true, 'inertia_pct', mrTrue));
dt = rd.dither;
assert(all(isnan([dt.groups.ks_N_per_um])) && all([dt.groups.ksBound_N_per_um] > 0), ...
    'a stiffness %s N/um from a transition within the resolution', mat2str([dt.groups.ks_N_per_um], 3));
assert(all([dt.groups.ksBound_N_per_um] < par.ks_N_um), 'bound %s N/um against a stiffness of %g', ...
    mat2str([dt.groups.ksBound_N_per_um], 3), par.ks_N_um);
fprintf('ok  drive path, synthetic dither on a stiff path (%g N/um): k_s > %s N/um, the windup is within the %g um resolution\n', ...
    par.ks_N_um, mat2str(round([dt.groups.ksBound_N_per_um])), 3);

% K logs with the friction reversal 2 (Fc + mu P A), the motor's own part
% of Fc not passing the path, and the path's deflection under the rest: a
% stiff path (20 kN/um) deflects less than the logs resolve and gets a
% bound below its stiffness; a soft one (0.8 kN/um) is resolved, and the
% bound with the motor-side friction out is its stiffness.
kStiff = writeKLog(dir, 'kl_stiff', base, 20000);
kSoft = writeKLog(dir, 'kl_soft', base, 800);
rs = charDrivePath(kStiff, struct('quiet', true));
assert(~rs.stiffness.resolved && rs.stiffness.bound_N_per_um < 20000 && rs.stiffness.bound_N_per_um > 1000, ...
    'stiff path: resolved %d, bound %.0f N/um', rs.stiffness.resolved, rs.stiffness.bound_N_per_um);
assert(abs(rs.friction.mu - base.mu) < 0.02 && abs(rs.friction.Fc_N - base.Fcp - base.Fcm) < 300, ...
    'mu %.3f (set %g), Fc %.0f N (set %g)', rs.friction.mu, base.mu, rs.friction.Fc_N, base.Fcp + base.Fcm);
rs2 = charDrivePath(kSoft, struct('quiet', true, 'motorFriction_pct', fmTrue));
assert(rs2.stiffness.resolved && abs(rs2.stiffness.boundPath_N_per_um / 800 - 1) < 0.15, ...
    'soft path: resolved %d, bound %.0f N/um (set 800)', rs2.stiffness.resolved, rs2.stiffness.boundPath_N_per_um);
% The whole reversal is not what passes the path when the motor has
% friction of its own: the bound that takes it so is above the stiffness
assert(rs2.stiffness.bound_N_per_um > 800 * 1.2, 'the whole-reversal bound %.0f N/um is not above the stiffness', ...
    rs2.stiffness.bound_N_per_um);
fprintf('ok  drive path, synthetic K logs: stiff path (20 kN/um) bound %.0f N/um, not resolved; soft path (0.8 kN/um) bound %.0f N/um with the motor-side friction out, resolved; mu %.2f (set %g)\n', ...
    rs.stiffness.bound_N_per_um, rs2.stiffness.boundPath_N_per_um, rs.friction.mu, base.mu);

fprintf('all passed\n');
end


% ===========================================================================
function d = driveCapture(par, cmdSeq, tChange, N)
% Step probe capture of a two-mass drive: cmdSeq(k) [mm/s] from the time
% tChange(k) [s], N rows of one scan. The motor follows the command after a
% dead time Td through a lag tau (the drive's velocity loop and the command
% filter in series, not slewed);
% its torque is mr a + its own friction + the force through the path. The
% path is a spring ks with a gap b (dead zone) between the motor position
% and the piston's; the piston, whose own mass is neglected, stays where it
% is while the force through the path is within its friction
% (Coulomb + mu P A) of the force the chamber puts on it, and otherwise
% moves to where that balance holds (a play operator). The chamber is
% P = P0 + kch x_p; the pressure column goes through the slave's median,
% low-pass and two-scan pipeline. par.Fcm, par.Fcp and par.mu are in N, par.ks_N_um in N/um.
Ts = par.Ts;
dt = Ts / par.sub;                                          % [s] integration step
nF = N * par.sub;
t = (0:nF - 1)' * dt;
cmd = zeros(nF, 1);
for k = 1:numel(cmdSeq)
    cmd(t >= tChange(k)) = cmdSeq(k);
end
nd = round(par.Td / dt);
cd = [cmd(1) * ones(nd, 1); cmd(1:end - nd)];               % dead time
a = exp(-dt / (par.tau / 2));                                % two lags of tau / 2: the command filter and the velocity loop
w = cmd(1);
A = par.areaN;                                              % [N per MPa]
c = par.kch * A;                                            % [N/mm] chamber stiffness
F0 = par.P0 * A;                                            % [N] chamber force at x_p = 0
g = par.b_um / 2000;                                        % [um -> mm] half the gap
ks = par.ks_N_um * 1000;                                    % [N/um -> N/mm]
v = cmd(1);
x = 0;
fp0 = par.Fcp + par.mu * max(par.P0, 0) * A;
xp = playSolve(ks, g, F0, c, sign(cmd(1)) * fp0, x);        % the path loaded in the direction of motion
vm = zeros(nF, 1); xm = zeros(nF, 1); am = zeros(nF, 1); Fs = zeros(nF, 1); Pv = zeros(nF, 1);
for i = 1:nF
    w = a * w + (1 - a) * cd(i);
    vNew = a * v + (1 - a) * w;
    am(i) = (vNew - v) / dt;                                % [mm/s^2]
    v = vNew;
    x = x + v * dt;
    fp = par.Fcp + par.mu * max((F0 + c * xp) / A, 0) * A;  % [N] piston friction
    r = pathForce(ks, g, x - xp) - (F0 + c * xp);
    if r > fp
        xn = playSolve(ks, g, F0, c, fp, x);
        xp = max(xp, xn);
    elseif r < -fp
        xn = playSolve(ks, g, F0, c, -fp, x);
        xp = min(xp, xn);
    end
    vm(i) = v; xm(i) = x;
    Fs(i) = pathForce(ks, g, x - xp);
    Pv(i) = (F0 + c * xp) / A;
end
Tq = par.J * am + (par.Fcm * tanh(vm / 0.01) + Fs) / par.nPP;   % [% rated]
idx = (par.sub:par.sub:nF)';
d.t = (0:N - 1)' * Ts;
d.vCmd = cmd(idx);
d.velocity = vm(idx) + par.noiseV * randn(N, 1);
ripple = par.ripple * sin(2 * pi * xm(idx) / 0.0726) + 0.5 * par.ripple * sin(4 * pi * xm(idx) / 0.0726 + 1);   % [%] per motor turn: 5.08 mm / 70
d.torque = Tq(idx) + par.noiseT * randn(N, 1) + ripple;
d.position = par.x0 + xm(idx) + par.noiseX * randn(N, 1);
Praw = Pv(idx);
med = (par.medianSize - 1) / 2;
Pm = [Praw(1) * ones(med, 1); Praw(1:end - med)];
alpha = Ts / (par.filterTau + Ts);
Pf = Pm;
for i = 2:N
    Pf(i) = Pf(i - 1) + alpha * (Pm(i) - Pf(i - 1));
end
Pp = [Pf(1) * ones(par.pipeline, 1); Pf(1:end - par.pipeline)];
d.pressure = Pp + par.noiseP * randn(N, 1);
end


function F = pathForce(ks, g, dl)
% Force through a spring ks with a gap of 2 g, for the motor ahead of the
% piston by dl.
if dl > g
    F = ks * (dl - g);
elseif dl < -g
    F = ks * (dl + g);
else
    F = 0;
end
end


function xp = playSolve(ks, g, F0, c, tgt, x)
% The piston position at which the force through the path exceeds the
% chamber's by tgt, for the motor at x.
xa = (ks * (x - g) - F0 - tgt) / (ks + c);
if x - xa > g
    xp = xa;
    return;
end
xb = (ks * (x + g) - F0 - tgt) / (ks + c);
if x - xb < -g
    xp = xb;
    return;
end
if c > 0
    xp = -(F0 + tgt) / c;
else
    xp = xb;
end
end


function f = writeDriveCapture(dir, name, d)
f = fullfile(dir, [name '.csv']);
fid = fopen(f, 'w');
fprintf(fid, 't,vCmd,velocity,torque,position,pressure\n');
fprintf(fid, '%.3f,%.3f,%.3f,%.2f,%.5f,%.4f\n', [d.t d.vCmd d.velocity d.torque d.position d.pressure]');
fclose(fid);
end


function files = ditherSet(dir, name, par, levels)
% Dither captures at the pressures in levels (the mean of the triangle):
% two at the top turn and two at the bottom turn, each the first command
% change of a capture, with the next reversal 1.5 s later in the same one.
amp = 0.15;                                                 % [mm] half the triangle
files = {};
for P = levels
    for q = 1:4
        sgn = 1 - 2 * mod(q, 2);                            % +1: moving up, turning at the top
        p = par;
        p.P0 = P + sgn * amp * par.kch - sgn * 0.2;         % [MPa] 0.2 MPa short of the turn at the capture start
        d = driveCapture(p, [sgn * 0.2, -sgn * 0.2, sgn * 0.2], [0 0.1 1.6], 500);
        files{end + 1} = writeDriveCapture(dir, sprintf('%s_%g_%d', name, P, q), d); %#ok<AGROW>
    end
end
end


function file = writeKLog(dir, name, par, ks_N_um)
% 50 ms run log of up and down ramps at 0.1 mm/s between 3 and 10 MPa
% (7.2 MPa/mm): the motor force is P A + sgn (Fc + mu P A), the torque that
% over newtonPerPct, and the motor is ahead of the piston by the force
% through the path over ks_N_um.
file = fullfile(dir, [name '.csv']);
k = 7.2;                                                    % [MPa/mm]
rows = zeros(0, 12);
t = 0;
x0 = 280;
for cycle = 1:2
    for sgn = [1 -1]
        if sgn > 0
            Ps = linspace(3, 10, 700);
        else
            Ps = linspace(10, 3, 700);
        end
        tr = t + (0:numel(Ps) - 1)' * 0.05;
        xp = x0 + (Ps' - 3) / k;                            % [mm]
        PA = Ps' * par.areaN;                               % [N]
        Fm = PA + sgn * (par.Fcp + par.Fcm + par.mu * PA);  % [N] motor force
        Fs = PA + sgn * (par.Fcp + par.mu * PA);            % [N] through the path
        xm = xp + Fs / (ks_N_um * 1000);                    % [N / (N/mm) -> mm]
        rows = [rows; tr ones(numel(tr), 1) 4 * ones(numel(tr), 1) 3 * ones(numel(tr), 1) ...
            0.1 * sgn * ones(numel(tr), 1) xm 0.1 * sgn * ones(numel(tr), 1) Fm / par.nPP ...
            Ps' ones(numel(tr), 1) zeros(numel(tr), 1) 12 * ones(numel(tr), 1)];  %#ok<AGROW>
        t = tr(end) + 0.05;
    end
end
rows(:, 8) = rows(:, 8) + 0.1 * randn(size(rows, 1), 1);   % [%] torque noise
rows(:, 9) = rows(:, 9) + 0.002 * randn(size(rows, 1), 1); % [MPa]
writeRun(file, rows);
end


function [t, cmd, v, x] = stopTrace(Ts, tEnd, v0, tStop, T0, D)
% Velocity at speed v0 until tStop, then, after T0, a ramp to rest at D.
t = (0:Ts:tEnd)';
cmd = v0 * ones(size(t));
cmd(t >= tStop) = 0;
v = v0 * ones(size(t));
after = t >= tStop + T0;
v(after) = sign(v0) * max(abs(v0) - D * (t(after) - tStop - T0), 0);
x = 250 + cumsum(v) * Ts;
end


function writeStopCapture(file, v0, T0, D, P0, cfg)
% step probe capture of a stop: 500 scans, the command to zero
% at 0.1 s, at a stiffness of 3 MPa/mm.
[t, cmd, v, x] = stopTrace(cfg.scanTime, 499 * cfg.scanTime, v0, 0.1, T0, D);
P = P0 + 3 * (x - x(1));
fid = fopen(file, 'w');
fprintf(fid, 't,vCmd,velocity,torque,position,pressure\n');
fprintf(fid, '%.3f,%.3f,%.3f,%.2f,%.5f,%.4f\n', [t cmd v + 0.002 * randn(size(t)) ...
    20 * ones(size(t)) x P]');
fclose(fid);
end


function writeStopLog(file, speeds, T0, D, P0, cfg)
% 50 ms run log of a velocity session: for each speed a leg at +v, a stop,
% a leg at -v and a stop, the reference logged in the row of the exchange
% that sent it and the slave acting a service tick later.
Ts = cfg.scanTime;
dtEx = 0.05;
rows = zeros(0, 12);
tBase = 0;
for v = speeds
    for dirSign = [1 -1]
        [tg, ~, vg, xg] = stopTrace(Ts, 3.0, dirSign * v, 1.5, T0 + 0.004, D);
        tr = (0.013:dtEx:2.9)';
        ref = dirSign * v * ones(size(tr));
        ref(tr >= 1.5) = 0;
        n = numel(tr);
        rows = [rows; tBase + tr ones(n, 1) 4 * ones(n, 1) 3 * ones(n, 1) ref ...
            interp1(tg, xg, tr) interp1(tg, vg, tr) + 0.002 * randn(n, 1) 20 * ones(n, 1) ...
            P0 + 3 * (interp1(tg, xg, tr) - xg(1)) double(abs(interp1(tg, vg, tr)) > 0.1) ...
            zeros(n, 1) 12 * ones(n, 1)];                                  %#ok<AGROW>
        tBase = tBase + 3.0;
    end
end
writeRun(file, rows);
end


% ===========================================================================
function writeRamps(file, K, V0, x0, Vg0, ks, A, Patm)
% Up ramp 0.5 -> 10.3 MPa and down ramp back, at 0.1 mm/s, 50 ms rows.
% Vg0 is the isothermal gas at atmosphere (the air pocket plus any free
% gas), mm^3. x(P) = x0 + V0 P/(K A) + (Vg0 Patm/A)(1/Patm - 1/(P+Patm))
% + P A/ks.
xOf = @(P) x0 + V0 * P / (K * A) + Vg0 * Patm / A * (1 / Patm - 1 ./ (P + Patm)) + ks2c(ks) * P * A;
rows = zeros(0, 12);
t = 0;
for dirn = [1 -1]
    if dirn > 0
        Ps = linspace(0.5, 10.3, 2000);
    else
        Ps = linspace(10.3, 0.5, 2000);
    end
    xs = xOf(Ps);
    ts = t + cumsum([0 abs(diff(xs))]) / 0.1;
    tr = (ts(1):0.05:ts(end))';
    P = interp1(ts, Ps, tr) + 0.002 * randn(size(tr));
    x = interp1(ts, xs, tr) + 0.0005 * randn(size(tr));
    n = numel(tr);
    rows = [rows; tr ones(n, 1) 4 * ones(n, 1) 3 * ones(n, 1) 0.1 * dirn * ones(n, 1) x ...
        0.1 * dirn * ones(n, 1) 20 * ones(n, 1) P ones(n, 1) zeros(n, 1) 12 * ones(n, 1)]; %#ok<AGROW>
    t = ts(end) + 0.05;
end
writeRun(file, rows);
end


function c = ks2c(ks)
if ks == 0
    c = 0;
else
    c = 1 / ks;
end
end


function collected = writeDelivery(file, Ptarget, Cd, K, V0, x0, cfg)
% Pressure mode at Ptarget: compress, hold, valve at 8 s, steady outflow
% with a dip and recovery, 40 s. Returns the volume that left (mL).
A = cfg.pistonArea_mm2;
Ao = cfg.orificeArea_m2;
qOf = @(P) Cd * Ao * sqrt(2 * max(P, 0) * 1e6 / cfg.rho) * 1e9;          % mm^3/s
dt = 0.05;
t = (0:dt:45)';
n = numel(t);
P = zeros(n, 1); x = zeros(n, 1); v = zeros(n, 1); inBand = zeros(n, 1);
xP = x0 + V0 * Ptarget / (K * A);                                       % liquid-full position at the target
tOpen = 8;
collected = 0;
Vl = V0;
for i = 1:n
    if t(i) < 3
        P(i) = Ptarget * t(i) / 3;
        x(i) = x0 + V0 * P(i) / (K * A);
    elseif t(i) < tOpen
        P(i) = Ptarget;
        x(i) = xP;
        inBand(i) = 1;
    else
        tau = t(i) - tOpen;
        P(i) = Ptarget - 0.6 * exp(-tau / 1.5) - 0.15;                   % dip, then a steady offset
        Q = qOf(P(i));
        if i > 1 && t(i - 1) >= tOpen
            dP = P(i) - P(i - 1);
        else
            dP = P(i) - Ptarget;
        end
        out = Q * dt;
        if tau < dt
            out = out + cfg.tubeVolume_mL * 1000;                        % the tube fills first
        else
            collected = collected + Q * dt;
        end
        Vl = Vl - Q * dt;
        x(i) = x(i - 1) + (out + Vl / K * dP) / A;                       % A dx = Q dt + (V/K) dP
        inBand(i) = abs(P(i) - Ptarget) <= 0.1;
    end
end
v = [0; diff(x) ./ dt];
rows = [t ones(n, 1) 5 * ones(n, 1) 7 * ones(n, 1) Ptarget * ones(n, 1) x v 30 * ones(n, 1) ...
    P + 0.002 * randn(n, 1) double(abs(v) > 0.1) inBand 12 * ones(n, 1)];
writeRun(file, rows);
collected = collected / 1000;
end


function writeCapture(file, v0, v1, tau, Td, accel, k, P0, cfg)
% step probe capture: 500 scans from the arm, the command step
% from v0 to v1 at 0.1 s, slew at accel, dead time, first-order lag (the
% piston's when k > 0), and the pressure at stiffness k from P0 through
% the slave's sensor chain.
Ts = cfg.scanTime;
N = 500;
t = (0:N - 1)' * Ts;
cmd = v0 * ones(N, 1);
cmd(t >= 0.1) = v1;
vref = cmd(1) * ones(N, 1);
for i = 2:N
    d = cmd(i) - vref(i - 1);
    vref(i) = vref(i - 1) + min(max(d, -accel * Ts), accel * Ts);
end
vd = interp1(t, vref, t - Td, 'linear', vref(1));
a = exp(-Ts / tau);
v = v0 * ones(N, 1);
for i = 2:N
    v(i) = a * v(i - 1) + (1 - a) * vd(i - 1);
end
x = 250 + cumsum(v) * Ts;
Praw = P0 + k * cumsum(v) * Ts;
med = (cfg.medianSize - 1) / 2;
Pm = [Praw(1) * ones(med, 1); Praw(1:end - med)];
alpha = Ts / (cfg.filterTau + Ts);
Pf = Pm;
for i = 2:N
    Pf(i) = Pf(i - 1) + alpha * (Pm(i) - Pf(i - 1));
end
P = [Pf(1) * ones(cfg.pipelineScans, 1); Pf(1:end - cfg.pipelineScans)];
if k > 0
    % The motor runs ahead of the piston: its feedback shows a shorter lag.
    am = exp(-Ts / 0.015);
    vm = v0 * ones(N, 1);
    for i = 2:N
        vm(i) = am * vm(i - 1) + (1 - am) * vd(i - 1);
    end
    v = vm;
end
fid = fopen(file, 'w');
fprintf(fid, 't,vCmd,velocity,torque,position,pressure\n');
fprintf(fid, '%.3f,%.3f,%.3f,%.2f,%.5f,%.4f\n', [t cmd v + 0.005 * randn(N, 1) 10 * ones(N, 1) x ...
    P + 0.0005 * randn(N, 1)]');
fclose(fid);
end


function writeStepLog(file, tau, Td, accel, k, P0, cfg)
% 50 ms run log of a velocity session: 0 -> v -> 0 -> -v -> 0 -> v -> 0,
% each held 3 s, with v = 1 mm/s unpressurised (k = 0) and 0.2 mm/s at a
% stiffness k (MPa/mm) from P0, the pressure through the slave's sensor
% chain. The reference is logged in the row of the exchange that sent it
% and the slave acts on it a service tick later; the feedback is read at
% the row's time.
Ts = cfg.scanTime;
dtEx = 0.05;
if k > 0
    vStep = 0.2;
else
    vStep = 1;
end
seqV = vStep * [0 1 0 -1 0 1 0];
hold = 3;
tEnd = hold * numel(seqV);
tg = (0:Ts:tEnd)';
cmd = zeros(size(tg));
tr = (0.013:dtEx:tEnd - 0.1)';                     % the exchange is not aligned with the steps
refRow = zeros(size(tr));
for s = 1:numel(seqV)
    tSent = tr(find(tr >= (s - 1) * hold, 1));     % the exchange that carries the new reference
    refRow(tr >= tSent) = seqV(s);
    cmd(tg >= tSent + 0.004) = seqV(s);            % the slave's service tick
end
vref = zeros(size(tg));
for i = 2:numel(tg)
    d = cmd(i) - vref(i - 1);
    vref(i) = vref(i - 1) + min(max(d, -accel * Ts), accel * Ts);
end
vd = interp1(tg, vref, tg - Td, 'linear', 0);
a = exp(-Ts / tau);
v = zeros(size(tg));
for i = 2:numel(tg)
    v(i) = a * v(i - 1) + (1 - a) * vd(i - 1);
end
x = 250 + cumsum(v) * Ts;
Praw = P0 + k * cumsum(v) * Ts;
med = (cfg.medianSize - 1) / 2;
Pm = [Praw(1) * ones(med, 1); Praw(1:end - med)];
alpha = Ts / (cfg.filterTau + Ts);
Pf = Pm;
for i = 2:numel(tg)
    Pf(i) = Pf(i - 1) + alpha * (Pm(i) - Pf(i - 1));
end
P = [Pf(1) * ones(cfg.pipelineScans, 1); Pf(1:end - cfg.pipelineScans)];
n = numel(tr);
vr = interp1(tg, v, tr) + 0.005 * randn(n, 1);
rows = [tr ones(n, 1) 4 * ones(n, 1) 3 * ones(n, 1) refRow interp1(tg, x, tr) ...
    vr 10 * ones(n, 1) interp1(tg, P, tr) + 0.002 * randn(n, 1) double(abs(vr) > 0.1) zeros(n, 1) 12 * ones(n, 1)];
writeRun(file, rows);
end


function writeRun(file, rows)
fid = fopen(file, 'w');
fprintf(fid, 't,step,status,mode,ref,position,velocity,torque,pressure,inMotion,inBand,exchangeMs\n');
fprintf(fid, '%.3f,%d,%d,%d,%.4f,%.4f,%.4f,%.3f,%.4f,%d,%d,%.1f\n', rows');
fclose(fid);
end
