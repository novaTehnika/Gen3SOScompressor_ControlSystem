function charFitsSynthetic()
%CHARFITSSYNTHETIC The three characterisation fits on synthetic logs with
%   known parameters: charBulkModulus recovers K, a residual gas bubble and
%   the split between liquid and series compliance from two fills;
%   charOrificeFlow recovers Cd and the square-root exponent from
%   deliveries at three pressures; charDriveLag recovers tauV and the dead
%   time from 2 ms step probe captures, and roughly from 50 ms run logs,
%   including the piston-side lag from the pressure column through the
%   sensor chain. The logs are in frictionRun's formats.
%   Run from the repository root:  addpath master-src/logic tests

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'), fullfile(here, '..', 'bench', 'friction'), ...
    fullfile(here, '..', 'bench', 'modbus'));
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
Vg0 = 1000;                     % mm^3 of free gas at atmosphere (0.02 mL at 5 MPa)
% Up and down ramps at 0.1 mm/s; x(P) from the compliances, then t from x.
f1 = fullfile(dir, 'k_run_1.csv');
writeRamps(f1, K, V0, x0, Vg0, 0, A, Patm);
r = charBulkModulus(f1, struct('quiet', true, 'deadVolume_mL', Vdead / 1000));
assert(abs(r.K_liq_GPa - K / 1000) / (K / 1000) < 0.02, 'liquid K %.3f GPa, set %.3f', r.K_liq_GPa, K / 1000);
assert(abs(r.K_lin_GPa - K / 1000) / (K / 1000) < 0.04, 'effective K %.3f GPa, set %.3f', r.K_lin_GPa, K / 1000);
P0 = mean([r.segments(1).Pmin r.segments(1).Pmax]);
VgAtP0 = Vg0 * Patm / (P0 + Patm) / 1000;
assert(abs(r.Vg_mL - VgAtP0) / VgAtP0 < 0.3, 'gas %.4f mL fitted, %.4f mL present at %.1f MPa', r.Vg_mL, VgAtP0, P0);
assert(numel(r.segments) == 2 && r.hysteresis < 0.02);
fprintf('ok  bulk modulus: K %.3f GPa (set %.3f), gas %.3f mL (set %.3f at %.1f MPa)\n', ...
    r.K_liq_GPa, K / 1000, r.Vg_mL, VgAtP0, P0);

% No gas, and the K check passes.
f2 = fullfile(dir, 'k_run_2.csv');
writeRamps(f2, K, V0, x0, 0, 0, A, Patm);
r = charBulkModulus(f2, struct('quiet', true, 'deadVolume_mL', Vdead / 1000));
assert(r.pass, 'clean liquid run failed the checks');
assert(abs(r.K_Pa - K * 1e6) / (K * 1e6) < 0.02);
fprintf('ok  bulk modulus: clean run passes, K %.3g Pa\n', r.K_Pa);

% Two fills with a series compliance of 200 N/um split liquid and structure.
ks = 200e3;                     % N/mm
f3 = fullfile(dir, 'k_fill300.csv');
f4 = fullfile(dir, 'k_fill150.csv');
writeRamps(f3, K, V0, x0, 0, ks, A, Patm);
x1 = cfg.posEOT - (150e3 - Vdead) / A;
writeRamps(f4, K, 150e3, x1, 0, ks, A, Patm);
r = charBulkModulus({f3, f4}, struct('quiet', true, 'deadVolume_mL', Vdead / 1000));
Keff = 1 / (1 / K + A^2 / (ks * V0));
assert(abs(r.segments(1).K_lin_GPa - Keff / 1000) / (Keff / 1000) < 0.03, ...
    'effective K at 300 mL %.3f GPa, expected %.3f with the series compliance', r.segments(1).K_lin_GPa, Keff / 1000);
assert(~isempty(r.split), 'two fills gave no split');
assert(abs(r.split.K_GPa - K / 1000) / (K / 1000) < 0.05, 'split liquid K %.3f GPa, set %.3f', r.split.K_GPa, K / 1000);
assert(abs(r.split.ks_N_per_um - ks / 1000) / (ks / 1000) < 0.1, 'series stiffness %.0f N/um, set %.0f', ...
    r.split.ks_N_per_um, ks / 1000);
fprintf('ok  bulk modulus: two fills give liquid %.2f GPa and series %.0f N/um (effective %.2f GPa at 300 mL)\n', ...
    r.split.K_GPa, r.split.ks_N_per_um, r.segments(1).K_lin_GPa);

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
writeCapture(c1, 0, 1.0, tau, Td, accel, cfg);
writeCapture(c2, 1.0, 0, tau, Td, accel, cfg);
r = charDriveLag({c1, c2}, struct('quiet', true, 'accel', accel, 'decel', accel));
assert(all(abs([r.captures.tauV] - tau) < 0.002), 'tauV %s, set %.3f', mat2str([r.captures.tauV], 3), tau);
assert(all(abs([r.captures.Td] - Td) < 0.002), 'Td %s, set %.3f', mat2str([r.captures.Td], 3), Td);
assert(r.pass);
fprintf('ok  drive lag: tauV %.1f / %.1f ms (set %.0f), Td %.1f / %.1f ms (set %.0f) from 2 ms captures\n', ...
    1000 * [r.captures.tauV], 1000 * tau, 1000 * [r.captures.Td], 1000 * Td);

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

fprintf('all passed\n');
end


% ===========================================================================
function writeRamps(file, K, V0, x0, Vg0, ks, A, Patm)
% Up ramp 0.5 -> 10.3 MPa and down ramp back, at 0.1 mm/s, 50 ms rows.
% x(P) = x0 + V0 P/(K A) + (Vg0 Patm/A)(1/Patm - 1/(P+Patm)) + P A/ks.
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


function writeCapture(file, v0, v1, tau, Td, accel, cfg)
% PRG_StepProbe capture: 500 scans at 2 ms from the arm, the command step
% from v0 to v1 at 0.1 s, slew at accel, dead time, first-order lag.
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
x = cumsum(v) * Ts;
fid = fopen(file, 'w');
fprintf(fid, 't,vCmd,velocity,torque,position\n');
fprintf(fid, '%.3f,%.3f,%.3f,%.2f,%.5f\n', [t cmd v + 0.005 * randn(N, 1) 10 * ones(N, 1) x]');
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
