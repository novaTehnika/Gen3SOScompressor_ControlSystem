function r = charDriveLag(files, opts)
%CHARDRIVELAG Drive velocity loop lag tauV from velocity command steps
%   (README section 5). Reads axis probe captures (2 ms) written by charRun
%   with opts.probe, and falls back to the 50 ms run log.
%
%   r = charDriveLag(file)
%   r = charDriveLag({capture1, capture2, ..., runLog}, opts)
%
%   Model, as in the design study's velocity actuator: the command is slewed
%   at the drive's accel and decel limits (eq. 11), delayed by a dead time
%   Td and followed through a first-order lag tauV (eq. 12). tauV and Td are
%   fitted to the velocity feedback by least squares; the slew limits are
%   the GVL values (opts.accel, opts.decel) unless opts.fitAccel scales them
%   as a third parameter.
%
%   The feedback is the controller's axis velocity, the motor encoder
%   differentiated and scaled through the gear ratio and screw lead, so it
%   sees the motor, not the piston. When a capture was taken under pressure
%   (pressure range above opts.minPressureSwing) the pressure channel is
%   fitted too: dP/dt = k v_piston, through the slave's median, low-pass
%   and pipeline delay (eqs. 16 to 18), with v_piston the same lag model.
%   That lag is the one at the piston, which is what the design model's
%   tauV stands for.
%
%   Run logs (50 ms) are fitted as an ensemble: every reference change in
%   velocity mode is a step, the command reaching the slave one exchange
%   after it was logged. The sample period cannot resolve Td from tauV;
%   their sum is what the fit returns, to about 25 ms.
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
%                of the step), t10, t63 of the feedback relative to the
%                command, and the pressure fit (tauP, TdP, k) when present
%   r.runLog     ensemble fit of the 50 ms log, when one was given
%   r.tauV_s     value for params.py: the mean of the capture fits, or the
%                pressure fits when present, or the run-log fit
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
    'residual', {}, 't10', {}, 't63', {}, 'tauP', {}, 'TdP', {}, 'k_MPa_mm', {}, 'residualP', {});
runFiles = {};
for f = 1:numel(files)
    fid = fopen(files{f});
    hdr = fgetl(fid);
    fclose(fid);
    if strncmp(hdr, 't,velocity,command', 18)
        c = charReadCapture(files{f});
        caps(end + 1) = fitCapture(c, opts, say); %#ok<AGROW>
    else
        runFiles{end + 1} = files{f}; %#ok<AGROW>
    end
end

if ~isempty(caps)
    say('\n%-24s %7s %8s %7s %6s %6s %6s %8s %7s %8s\n', 'capture', 'step', 'tauV ms', 'Td ms', 'res %', ...
        't10 ms', 't63 ms', 'tauP ms', 'TdP ms', 'k MPa/mm');
    for k = 1:numel(caps)
        c = caps(k);
        say('%-24s %+7.2f %8.1f %7.1f %6.1f %6.0f %6.0f %8s %7s %8s\n', c.file, c.stepSize, ...
            c.tauV * 1000, c.Td * 1000, 100 * c.residual, c.t10 * 1000, c.t63 * 1000, ...
            fmt(c.tauP * 1000, '%.1f'), fmt(c.TdP * 1000, '%.1f'), fmt(c.k_MPa_mm, '%.2f'));
    end
end
r.captures = caps;

r.runLog = [];
for f = 1:numel(runFiles)
    d = charReadRun(runFiles{f});
    rl = fitRunLog(d, opts, say);
    if ~isempty(rl)
        rl.file = shortName(runFiles{f});
        r.runLog = [r.runLog rl];
    end
end

% --- Summary and acceptance -------------------------------------------------------
checks = {};
tauP = [caps.tauP];
tauP = tauP(~isnan(tauP));
if ~isempty(caps)
    tau = [caps.tauV];
    r.tauV_motor_s = mean(tau);
    r.Td_s = mean([caps.Td]);
    say('\nMotor-side lag: tauV %.1f +/- %.1f ms, dead time %.1f ms (%d captures)\n', ...
        1000 * mean(tau), 1000 * std(tau), 1000 * r.Td_s, numel(tau));
    checks{end + 1} = check('residual', max([caps.residual]), 0, 0.05, 'fit residual as a fraction of the step');
    checks{end + 1} = check('tauV_ms', 1000 * [min(tau) max(tau)], 2, 100, 'plausible first-order lag');
    checks{end + 1} = check('Td_ms', 1000 * [caps.Td], 0, 30, 'dead time within a few scans and a Modbus tick');
    if numel(tau) > 1
        checks{end + 1} = check('spread', std(tau) / mean(tau), 0, 0.3, 'capture-to-capture spread of tauV');
    end
    if ~isempty(tauP)
        r.tauV_piston_s = mean(tauP);
        say('Piston-side lag from the pressure: tauP %.1f +/- %.1f ms (%d captures)\n', ...
            1000 * mean(tauP), 1000 * std(tauP), numel(tauP));
        checks{end + 1} = check('residualP', max([caps(~isnan([caps.tauP])).residualP]), 0, 0.1, ...
            'pressure fit residual as a fraction of the swing');
        r.tauV_s = r.tauV_piston_s;
    else
        r.tauV_s = r.tauV_motor_s;
    end
elseif ~isempty(r.runLog)
    r.tauV_s = mean([r.runLog.tauV]);
    say('\nRun log only: tauV + Td %.0f ms from %d steps; the 50 ms log resolves this to about 25 ms\n', ...
        1000 * r.tauV_s, sum([r.runLog.nSteps]));
    checks{end + 1} = check('residual', max([r.runLog.residual]), 0, 0.1, 'ensemble fit residual as a fraction of the step');
    checks{end + 1} = check('tauV_ms', 1000 * r.tauV_s, 2, 100, 'plausible lag plus dead time');
else
    error('no capture and no velocity step in a run log');
end
r.checks = checks;
r.pass = all(cellfun(@(c) c.pass, checks));
printChecks(say, checks);

say('\nparams.py:\n');
say('  tauV = %.4f        # [s] drive velocity loop lag', r.tauV_s);
if ~isempty(tauP)
    say(' at the piston (pressure fit; motor %.4f s)', r.tauV_motor_s);
elseif ~isempty(caps)
    say(' at the motor encoder');
else
    say(' including dead time, from 50 ms samples');
end
say('\n');
end


% ===========================================================================
function c = fitCapture(cap, opts, say)
Ts = cap.Ts;
t = cap.t;
cmd = cap.command;
v = cap.velocity;
pre = t < 0;
if ~any(pre)
    pre = 1;
end
v0 = mean(v(pre));
[~, c.file] = fileparts(cap.file);
if numel(c.file) > 24
    c.file = c.file(1:24);
end
dc = diff(cmd);
j = find(abs(dc) >= opts.minStep, 1);
if isempty(j)
    j = find(abs(dc) > 0, 1);
end
if isempty(j)
    error('%s: the command does not change in the capture', cap.file);
end
c.stepSize = cmd(j + 1) - cmd(j);

% Fit tauV, Td (and the slew scale) to the velocity feedback.
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
after = t >= t(j + 1);
vt = v(after); tt = t(after);
i10 = find(sign(c.stepSize) * (vt - v0) >= 0.10 * abs(c.stepSize), 1);
i63 = find(sign(c.stepSize) * (vt - v0) >= 0.63 * abs(c.stepSize), 1);
c.t10 = pick(tt, i10) - t(j + 1);
c.t63 = pick(tt, i63) - t(j + 1);

% Pressure channel: the lag at the piston.
c.tauP = NaN; c.TdP = NaN; c.k_MPa_mm = NaN; c.residualP = NaN;
P = cap.pressure;
swing = max(P) - min(P);
if swing >= opts.minPressureSwing
    P0 = mean(P(pre));
    dx = sum(v) * Ts;
    if abs(dx) > 1e-6
        k0 = (P(end) - P(1)) / dx;
    else
        k0 = 10;
    end
    thp0 = [log(0.02) 0.004 k0 P0];
    costP = @(th) sum((P - pressModel(th, cmd, Ts, v0, opts)).^2);
    thp = fminsearch(costP, thp0, optimset('TolX', 1e-7, 'TolFun', 1e-10, 'MaxFunEvals', 8000, 'MaxIter', 4000));
    c.tauP = exp(thp(1));
    c.TdP = max(thp(2), 0);
    c.k_MPa_mm = thp(3);
    c.residualP = sqrt(mean((P - pressModel(thp, cmd, Ts, v0, opts)).^2)) / swing;
end
say('  %s: step %+.2f mm/s, tauV %.1f ms, Td %.1f ms, residual %.1f %%%s\n', c.file, c.stepSize, ...
    1000 * c.tauV, 1000 * c.Td, 100 * c.residual, pressText(c));
end


function txt = pressText(c)
if isnan(c.tauP)
    txt = '';
else
    txt = sprintf('; pressure: tauP %.1f ms, TdP %.1f ms, k %.2f MPa/mm, residual %.1f %%', ...
        1000 * c.tauP, 1000 * c.TdP, c.k_MPa_mm, 100 * c.residualP);
end
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


function P = pressModel(th, cmd, Ts, v0, opts)
% Piston velocity through the lag model, integrated into pressure at
% stiffness k, then the slave's median (a delay of (N-1)/2 scans for a
% smooth signal), first-order filter and two-scan pipeline.
tau = exp(th(1));
Td = max(th(2), 0);
k = th(3);
P0 = th(4);
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
% Ensemble fit of every velocity-mode reference change in a 50 ms log.
inVel = d.modeCode == 3 & d.motionEnable == 1 & d.status == 4;
idx = find(inVel(1:end - 1) & inVel(2:end) & abs(diff(d.reference)) >= opts.minStep);
rl = [];
if isempty(idx)
    return;
end
Ts = opts.Ts;
dtEx = median(diff(d.t));
steps = {};
for i = idx'
    t0 = d.t(i + 1);                              % the row that logged the new reference
    tCmd = t0 + dtEx;                             % it goes out in the next exchange
    tEnd = tCmd + max(1.5, abs(d.reference(i + 1) - d.reference(i)) / opts.accel + 0.6);
    rows = find(d.t >= t0 - 0.3 & d.t <= tEnd & inVel);
    if numel(rows) < 10
        continue;
    end
    s.t = d.t(rows) - tCmd;
    s.v = d.velocity(rows);
    s.ref0 = d.reference(i);
    s.ref1 = d.reference(i + 1);
    s.v0 = mean(d.velocity(rows(d.t(rows) < t0)));
    if isnan(s.v0)
        s.v0 = s.ref0;
    end
    steps{end + 1} = s; %#ok<AGROW>
end
if isempty(steps)
    return;
end
cost = @(th) ensembleCost(th, steps, Ts, opts);
th = fminsearch(cost, [log(0.03) 0.0], optimset('TolX', 1e-6, 'TolFun', 1e-9, 'MaxFunEvals', 4000));
rl.nSteps = numel(steps);
rl.tauV = exp(th(1));
rl.Td = max(th(2), 0);
[~, res, amp] = ensembleCost(th, steps, Ts, opts);
rl.residual = sqrt(res / amp);
say('  run log: %d steps, lag %.0f ms with dead time %.0f ms, residual %.1f %% (50 ms samples)\n', ...
    rl.nSteps, 1000 * rl.tauV, 1000 * rl.Td, 100 * rl.residual);
end


function [J, res, amp] = ensembleCost(th, steps, Ts, opts)
J = 0; res = 0; amp = 0;
for k = 1:numel(steps)
    s = steps{k};
    tg = (floor(s.t(1) / Ts) * Ts:Ts:s.t(end) + Ts)';
    cmd = s.ref0 * ones(size(tg));
    cmd(tg >= 0) = s.ref1;
    vm = velModel(th, cmd, Ts, s.v0, opts);
    vi = interp1(tg, vm, s.t, 'linear', 'extrap');
    e = sum((s.v - vi).^2);
    J = J + e;
    res = res + e / numel(s.v);
    amp = amp + (s.ref1 - s.ref0)^2;
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


function s = fmt(v, f)
if isnan(v)
    s = '-';
else
    s = sprintf(f, v);
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


function n = shortName(file)
[~, n] = fileparts(file);
if numel(n) > 22
    n = n(1:22);
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
