function r = charDriveLag(files, opts)
%CHARDRIVELAG Drive velocity loop lag tauV from velocity command steps
%   (README section 5), as the design model's actuator sees it: the command
%   slewed at the drive's accel and decel limits (eq. 11 of the design
%   README), a dead time Td, and a first-order lag tauV (eq. 12).
%
%   r = charDriveLag(file)
%   r = charDriveLag({capture1, capture2, ..., runLog}, opts)
%
%   Captures (PRG_StepProbe, 2 ms: t, vCmd, velocity, torque, position)
%   give the lag at the motor: tauV and Td are fitted to the velocity
%   feedback by least squares. The feedback is the controller's axis
%   velocity, the motor encoder differentiated and scaled through the gear
%   ratio and screw lead, so gearbox lost motion and screw compliance are
%   invisible to it. bench/friction/stepResponseFit does the same by a grid
%   search; the two should agree.
%
%   Run logs (frictionRun or charRun, 50 ms) are fitted as an ensemble:
%   every reference change in velocity mode is a step, the command reaching
%   the slave within a service tick of the exchange that logged it.
%     velocity   tauV and Td from the velocity column, resolved to about
%                25 ms by the sample period.
%     pressure   for steps whose pressure moves by more than
%                opts.minPressureSwing, the pressure column is fitted to
%                the same lag model integrated at the chamber stiffness and
%                passed through the slave's median, low-pass and pipeline
%                delay (eqs. 16 to 18). Only the piston moves the pressure,
%                so this is the lag at the piston, which is what tauV in
%                the model stands for; the sample period limits it to the
%                same 25 ms. The stiffness of each step comes from its own
%                pressure and position change.
%
%   opts.accel, opts.decel  mm/s^2 (default charConfig: G_cfgDirectCtrlAccel/Decel)
%   opts.fitAccel           also fit a common scale on the slew limits (default false)
%   opts.Ts                 slave scan (default charConfig)
%   opts.filterTau, opts.medianSize, opts.pipelineScans
%                           sensor chain for the pressure fit (default charConfig)
%   opts.minStep            mm/s, smaller command changes are ignored (default 0.2)
%   opts.minPressureSwing   MPa (default 0.1)
%   opts.quiet              no printing
%
%   r.captures   per capture: file, step size, tauV, Td, residual (fraction
%                of the step), t10 and t63 of the feedback from the command
%   r.runLog     per log: the velocity ensemble (tauV, Td, nSteps,
%                residual) and the pressure ensemble (tauP, TdP, nSteps,
%                residualP, k_MPa_mm)
%   r.tauV_s     value for params.py: the mean of the capture fits, or the
%                run-log velocity fit without captures
%   r.tauP_s     the piston-side lag plus dead time from the pressure, NaN
%                without a pressurised step
%   r.pass, r.checks

if ischar(files)
    files = {files};
end
if nargin < 2
    opts = struct();
end
cfg = charConfig();
opts = withDefault(opts, 'accel', cfg.velAccel);
opts = withDefault(opts, 'decel', cfg.velDecel);
opts = withDefault(opts, 'fitAccel', false);
opts = withDefault(opts, 'Ts', cfg.scanTime);
opts = withDefault(opts, 'filterTau', cfg.filterTau);
opts = withDefault(opts, 'medianSize', cfg.medianSize);
opts = withDefault(opts, 'pipelineScans', cfg.pipelineScans);
opts = withDefault(opts, 'minStep', 0.2);
opts = withDefault(opts, 'minPressureSwing', 0.1);
opts = withDefault(opts, 'quiet', false);
say = @(varargin) sayIf(~opts.quiet, varargin{:});

say('\n=== charDriveLag: %d file(s); slew %g / %g mm/s^2, pressure filter %.0f ms + %d scans\n', ...
    numel(files), opts.accel, opts.decel, opts.filterTau * 1000, opts.pipelineScans);
say('Expected: tauV 5..50 ms (20 ms assumed), dead time a few scans. With a %g mm/s^2 slew a\n', opts.accel);
say('  1 mm/s step ramps for %.0f ms; the lag shows as the feedback trailing the ramp by tauV.\n', ...
    1000 / opts.accel);

caps = struct('file', {}, 'stepSize', {}, 'tauV', {}, 'Td', {}, 'accelScale', {}, ...
    'residual', {}, 't10', {}, 't63', {});
logs = [];
for f = 1:numel(files)
    d = charReadCsv(files{f});
    if isfield(d, 'vCmd')
        caps(end + 1) = fitCapture(d, opts, say); %#ok<AGROW>
    else
        rl = fitRunLog(d, opts, say);
        if ~isempty(rl)
            logs = [logs rl]; %#ok<AGROW>
        end
    end
end

if ~isempty(caps)
    say('\n%-28s %7s %8s %7s %6s %6s %6s\n', 'capture', 'step', 'tauV ms', 'Td ms', 'res %', 't10 ms', 't63 ms');
    for k = 1:numel(caps)
        c = caps(k);
        say('%-28s %+7.2f %8.1f %7.1f %6.1f %6.0f %6.0f\n', c.file, c.stepSize, ...
            c.tauV * 1000, c.Td * 1000, 100 * c.residual, c.t10 * 1000, c.t63 * 1000);
    end
end
r.captures = caps;
r.runLog = logs;

% --- Summary and acceptance -------------------------------------------------------
checks = {};
r.tauV_s = NaN; r.Td_s = NaN; r.tauP_s = NaN;
if ~isempty(caps)
    tau = [caps.tauV];
    r.tauV_s = mean(tau);
    r.Td_s = mean([caps.Td]);
    say('\nMotor-side lag from %d captures: tauV %.1f +/- %.1f ms, dead time %.1f ms\n', ...
        numel(tau), 1000 * mean(tau), 1000 * std(tau), 1000 * r.Td_s);
    checks{end + 1} = check('residual', max([caps.residual]), 0, 0.05, 'capture fit residual as a fraction of the step');
    checks{end + 1} = check('tauV_ms', 1000 * [min(tau) max(tau)], 2, 100, 'plausible first-order lag');
    checks{end + 1} = check('Td_ms', 1000 * [caps.Td], 0, 30, 'dead time within a few scans and a Modbus tick');
    if numel(tau) > 1
        checks{end + 1} = check('spread', std(tau) / mean(tau), 0, 0.3, 'capture-to-capture spread of tauV');
    end
end
vel = logs(arrayfun(@(l) ~isnan(l.tauV), logs));
if ~isempty(vel)
    lumped = sum([vel.tauV] .* [vel.nSteps]) / sum([vel.nSteps]) + ...
             sum([vel.Td] .* [vel.nSteps]) / sum([vel.nSteps]);
    say('\nRun log velocity: lag plus dead time %.0f ms from %d steps (50 ms samples, about 25 ms resolution)\n', ...
        1000 * lumped, sum([vel.nSteps]));
    if isempty(caps)
        r.tauV_s = lumped;
        checks{end + 1} = check('residual', max([vel.residual]), 0, 0.1, 'ensemble fit residual as a fraction of the step');
        checks{end + 1} = check('tauV_ms', 1000 * lumped, 2, 100, 'plausible lag plus dead time');
    end
end
pres = logs(arrayfun(@(l) ~isnan(l.tauP), logs));
if ~isempty(pres)
    r.tauP_s = sum(([pres.tauP] + [pres.TdP]) .* [pres.nStepsP]) / sum([pres.nStepsP]);
    say('Run log pressure: piston-side lag plus dead time %.0f ms from %d pressurised steps, stiffness %s MPa/mm\n', ...
        1000 * r.tauP_s, sum([pres.nStepsP]), mat2str([pres.k_MPa_mm], 3));
    checks{end + 1} = check('residualP', max([pres.residualP]), 0, 0.1, 'pressure fit residual as a fraction of the swing');
    checks{end + 1} = check('tauP_ms', 1000 * r.tauP_s, 2, 150, 'plausible piston-side lag plus dead time');
end
if isnan(r.tauV_s) && isnan(r.tauP_s)
    error('no capture and no velocity step in a run log');
end
r.checks = checks;
r.pass = all(cellfun(@(c) c.pass, checks));
printChecks(say, checks);

say('\nparams.py:\n');
if ~isnan(r.tauV_s)
    say('  tauV = %.4f        # [s] drive velocity loop lag', r.tauV_s);
    if ~isempty(caps)
        say(' at the motor encoder, %d captures', numel(caps));
    else
        say(' including dead time, from 50 ms samples');
    end
    say('\n');
end
if ~isnan(r.tauP_s)
    say('  # piston side from the pressure: %.4f s including dead time, to about 0.025 s;\n', r.tauP_s);
    say('  # use it for tauV when it exceeds the motor-side value by more than that\n');
end
end


% ===========================================================================
function c = fitCapture(d, opts, say)
Ts = median(diff(d.t));
cmd = d.vCmd;
v = d.velocity;
[~, c.file] = fileparts(d.file);
if numel(c.file) > 28
    c.file = c.file(1:28);
end
dc = diff(cmd);
j = find(abs(dc) >= opts.minStep, 1);
if isempty(j)
    j = find(abs(dc) > 0, 1);
end
if isempty(j)
    error('%s: the command does not change in the capture', d.file);
end
pre = 1:j;
v0 = mean(v(pre));
c.stepSize = cmd(j + 1) - cmd(j);

th0 = [log(0.02) 0.004];
if opts.fitAccel
    th0(3) = 0;
end
cost = @(th) sum((v - velModel(th, cmd, Ts, v0, opts)).^2);
th = fminsearch(cost, th0, optimset('TolX', 1e-6, 'TolFun', 1e-9, 'MaxFunEvals', 4000, 'MaxIter', 2000));
c.tauV = exp(th(1));
c.Td = max(th(2), 0);
if opts.fitAccel
    c.accelScale = exp(th(3));
else
    c.accelScale = 1;
end
vm = velModel(th, cmd, Ts, v0, opts);
c.residual = sqrt(mean((v - vm).^2)) / max(abs(c.stepSize), 1e-6);

% Model-free: feedback reaching 10 % and 63 % of the first step, measured
% from the scan the command changed.
t = d.t;
after = t >= t(j + 1);
vt = v(after); tt = t(after);
i10 = find(sign(c.stepSize) * (vt - v0) >= 0.10 * abs(c.stepSize), 1);
i63 = find(sign(c.stepSize) * (vt - v0) >= 0.63 * abs(c.stepSize), 1);
c.t10 = pick(tt, i10) - t(j + 1);
c.t63 = pick(tt, i63) - t(j + 1);
say('  %s: step %+.2f mm/s, tauV %.1f ms, Td %.1f ms, residual %.1f %%\n', c.file, c.stepSize, ...
    1000 * c.tauV, 1000 * c.Td, 100 * c.residual);
end


function v = velModel(th, cmd, Ts, v0, opts)
% Slew-limited command, dead time, first-order lag, sampled at Ts.
tau = exp(th(1));
Td = max(th(2), 0);
scale = 1;
if numel(th) >= 3
    scale = exp(th(3));
end
vref = slew(cmd, Ts, opts.accel * scale, opts.decel * scale);
vd = delay(vref, Ts, Td);
v = lag(vd, Ts, tau, v0);
end


function P = pressModel(th, cmd, Ts, v0, k, P0, opts)
% Piston velocity through the lag model, integrated into pressure at
% stiffness k, then the slave's median (a delay of (N-1)/2 scans for a
% smooth signal), first-order filter and two-scan pipeline.
tau = exp(th(1));
Td = max(th(2), 0);
vref = slew(cmd, Ts, opts.accel, opts.decel);
vp = lag(delay(vref, Ts, Td), Ts, tau, v0);
Praw = P0 + k * cumsum(vp) * Ts;
med = (opts.medianSize - 1) / 2;
Pm = shiftBack(Praw, med, Praw(1));
alpha = Ts / (opts.filterTau + Ts);
Pf = zeros(size(Pm));
Pf(1) = Pm(1);
for n = 2:numel(Pm)
    Pf(n) = Pf(n - 1) + alpha * (Pm(n) - Pf(n - 1));
end
P = shiftBack(Pf, opts.pipelineScans, Pf(1));
end


function rl = fitRunLog(d, opts, say)
% Ensemble fits of every velocity-mode reference change in a 50 ms log:
% the velocity column, and the pressure column for the pressurised steps.
rl = [];
inVel = d.mode == 3 & d.status == 4;
idx = find(inVel(1:end - 1) & inVel(2:end) & abs(diff(d.ref)) >= opts.minStep);
if isempty(idx)
    return;
end
[~, rl.file] = fileparts(d.file);
Ts = opts.Ts;
dtEx = median(diff(d.t));
steps = {};
for i = idx'
    t0 = d.t(i + 1);                              % the row whose exchange sent the new reference
    tCmd = t0;                                    % the slave acts on it within a service tick
    tEnd = tCmd + max(1.5, abs(d.ref(i + 1) - d.ref(i)) / opts.accel + 0.6);
    rows = find(d.t >= t0 - 0.3 & d.t <= tEnd & inVel);
    rows = rows(abs(d.ref(rows) - d.ref(i + 1)) < 1e-9 | d.t(rows) < t0);   % until the next change
    if numel(rows) < 10
        continue;
    end
    s.t = d.t(rows) - tCmd;
    s.v = d.velocity(rows);
    s.P = d.pressure(rows);
    s.x = d.position(rows);
    s.ref0 = d.ref(i);
    s.ref1 = d.ref(i + 1);
    preRows = rows(d.t(rows) < t0);
    if isempty(preRows)
        s.v0 = s.ref0;
    else
        s.v0 = mean(d.velocity(preRows));
    end
    s.swing = max(s.P) - min(s.P);
    dx = s.x(end) - s.x(1);
    if abs(dx) > 1e-3
        s.k = (s.P(end) - s.P(1)) / dx;             % MPa/mm over the window
    else
        s.k = NaN;
    end
    steps{end + 1} = s; %#ok<AGROW>
end
if isempty(steps)
    rl = [];
    return;
end
fmo = optimset('TolX', 1e-6, 'TolFun', 1e-9, 'MaxFunEvals', 4000);

% Velocity ensemble.
th = fminsearch(@(th) ensembleCost(th, steps, Ts, opts, 'v'), [log(0.03) 0.0], fmo);
rl.nSteps = numel(steps);
rl.tauV = exp(th(1));
rl.Td = max(th(2), 0);
[~, res, amp] = ensembleCost(th, steps, Ts, opts, 'v');
rl.residual = sqrt(res / amp);
say('  %s: %d steps, velocity lag %.0f ms with dead time %.0f ms, residual %.1f %% (50 ms samples)\n', ...
    rl.file, rl.nSteps, 1000 * rl.tauV, 1000 * rl.Td, 100 * rl.residual);

% Pressure ensemble over the pressurised steps with a usable stiffness.
rl.tauP = NaN; rl.TdP = NaN; rl.nStepsP = 0; rl.residualP = NaN; rl.k_MPa_mm = NaN;
pres = steps(cellfun(@(s) s.swing >= opts.minPressureSwing && ~isnan(s.k) && s.k > 0, steps));
if ~isempty(pres)
    th = fminsearch(@(th) ensembleCost(th, pres, Ts, opts, 'P'), [log(0.03) 0.0], fmo);
    rl.tauP = exp(th(1));
    rl.TdP = max(th(2), 0);
    rl.nStepsP = numel(pres);
    [~, res, amp] = ensembleCost(th, pres, Ts, opts, 'P');
    rl.residualP = sqrt(res / amp);
    rl.k_MPa_mm = mean(cellfun(@(s) s.k, pres));
    say('  %s: %d pressurised steps, piston lag %.0f ms with dead time %.0f ms, stiffness %.1f MPa/mm, residual %.1f %%\n', ...
        rl.file, rl.nStepsP, 1000 * rl.tauP, 1000 * rl.TdP, rl.k_MPa_mm, 100 * rl.residualP);
end
end


function [J, res, amp] = ensembleCost(th, steps, Ts, opts, which)
J = 0; res = 0; amp = 0;
for k = 1:numel(steps)
    s = steps{k};
    tg = (floor(s.t(1) / Ts) * Ts:Ts:s.t(end) + Ts)';
    cmd = s.ref0 * ones(size(tg));
    cmd(tg >= 0) = s.ref1;
    if which == 'v'
        ym = velModel(th, cmd, Ts, s.v0, opts);
        y = s.v;
        a = (s.ref1 - s.ref0)^2;
    else
        % The pressure offset is linear in the model: the best one is the
        % mean residual, so it is not a search parameter.
        ym = pressModel(th, cmd, Ts, s.v0, s.k, 0, opts);
        y = s.P;
        a = s.swing^2;
    end
    yi = interp1(tg, ym, s.t, 'linear', 'extrap');
    if which ~= 'v'
        yi = yi + mean(y - yi);
    end
    e = sum((y - yi).^2);
    J = J + e;
    res = res + e / numel(y);
    amp = amp + a;
end
end


function vref = slew(cmd, Ts, accel, decel)
% Eq. (11): accel while the command grows in magnitude, decel while it
% shrinks or reverses.
vref = zeros(size(cmd));
vref(1) = cmd(1);
for k = 2:numel(cmd)
    d = cmd(k) - vref(k - 1);
    if d * vref(k - 1) >= 0
        rate = accel;
    else
        rate = decel;
    end
    vref(k) = vref(k - 1) + min(max(d, -rate * Ts), rate * Ts);
end
end


function y = delay(u, Ts, Td)
% Fractional delay by linear interpolation, held at the first value.
n = numel(u);
if Td <= 0
    y = u;
    return;
end
t = (0:n - 1)' * Ts;
y = interp1(t, u, t - Td, 'linear', u(1));
end


function y = lag(u, Ts, tau, y0)
% First-order lag, zero-order hold on the input.
a = exp(-Ts / tau);
y = zeros(size(u));
y(1) = y0;
for k = 2:numel(u)
    y(k) = a * y(k - 1) + (1 - a) * u(k - 1);
end
end


function y = shiftBack(u, n, fill)
n = round(n);
if n <= 0
    y = u;
    return;
end
y = [fill * ones(n, 1); u(1:end - n)];
end


function v = pick(t, i)
if isempty(i)
    v = NaN;
else
    v = t(i);
end
end


function c = check(name, value, lo, hi, text)
c.name = name;
c.value = value;
c.limits = [lo hi];
c.text = text;
c.pass = all(value >= lo & value <= hi);
end


function printChecks(say, checks)
say('\nAcceptance:\n');
for k = 1:numel(checks)
    c = checks{k};
    if c.pass
        mark = 'ok  ';
    else
        mark = 'FAIL';
    end
    say('  %s %-14s %-14s in [%g, %g]: %s\n', mark, c.name, mat2str(c.value, 4), c.limits, c.text);
end
end


function sayIf(on, varargin)
if on
    fprintf(varargin{:});
end
end


function opts = withDefault(opts, name, value)
if ~isfield(opts, name)
    opts.(name) = value;
end
end
