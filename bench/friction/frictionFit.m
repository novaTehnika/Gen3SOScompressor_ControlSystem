function r = frictionFit(unloaded, loaded, opts)
%FRICTIONFIT Fit the friction of the servo axis from frictionRun logs and
%   print it in the units of the pressure loop design model.
%
%   r = frictionFit(unloaded)
%   r = frictionFit(unloaded, loaded)
%   r = frictionFit(unloaded, loaded, opts)
%
%   unloaded, loaded  a log file, a pattern ('captures/u1_unloaded_*.csv')
%                     or a cell array of either, from frictionRun. The
%                     unloaded logs are jog sets with the chamber vented;
%                     the loaded logs are jog sets under a gas charge and
%                     pressure-mode deliveries. Either may be empty.
%   opts.quiet        no printing
%   opts.logShuntOhms G_cfgPressureShuntOhms the logs were recorded with;
%                     their pressure is rescaled to the present setting
%                     (benchPressureRescale; default: present)
%
%   Every torque sample is converted to the force the motor would exert at
%   the piston through a lossless drivetrain, F = T * newtonPerPct
%   (frictionBenchConfig). Each jog step gives one point: the mean force,
%   velocity and pressure over its steady part, which starts settle
%   seconds after the command has reached speed and ends trim seconds
%   before the stop. Each pressure-mode run gives one point per binSeconds
%   of samples moving faster than vMin and accelerating slower than aMax.
%   The fits are least squares over the points, one weight each:
%
%     unloaded   F = F0 + Fc sign(v) + b v
%                F = F0 + sign(v) (Fc + (Fs - Fc) exp(-(|v|/vs)^2)) + b v
%                The second (Stribeck) form is kept when it lowers the
%                residual by a fifth with a positive falling term.
%     loaded     F - P A = F0 + sign(v) (Fc + mu P A) + b v
%                P A is the pressure force on the piston. mu is the part
%                of the friction proportional to the load, which is what
%                the drivetrain's efficiency is: 1/(1 + mu) pushing
%                against the load, 1 - mu being pushed by it. When the
%                points cannot separate mu (one direction at one
%                pressure) b is fixed to the unloaded value, and failing
%                that mu is fixed to zero and Fc includes the load term.
%
%   F0 is a constant offset: the weight of the piston on an inclined
%   axis, a preload, or a torque zero error. The design model has no such
%   term, so it is reported and left out of the values printed for it.
%
%   The acceptance checks are printed per step: the force is stationary
%   (its drift over the step within its ripple or 3 %), the step covers
%   at least one screw lead, the velocity reached the command, and each
%   reversal pair's mean lies at the load within a tenth of its step. The
%   breakaway force is the largest force between the command and the
%   first sample moving faster than vMin, less the pressure force at that
%   sample.
%
%   The values printed for params.py come from the Stribeck form when it
%   is kept, since the plain form's b is then biased by the falling
%   region, and from the plain form otherwise.
%
%   r holds the segments, bins, pairs and fits with their standard errors.

if nargin < 2
    loaded = {};
end
if nargin < 3
    opts = struct();
end
if ~isfield(opts, 'quiet')
    opts.quiet = false;
end
if ~isfield(opts, 'logShuntOhms')
    opts.logShuntOhms = [];
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'));
cfg = frictionBenchConfig();
say = @(varargin) printUnless(opts.quiet, varargin{:});

filesU = expandFiles(unloaded);
filesL = expandFiles(loaded);
if isempty(filesU) && isempty(filesL)
    error('no log files found');
end

% --- Segments and bins -------------------------------------------------------
segs = [];
bins = [];
for k = 1:numel(filesU)
    L = readLog(filesU{k});
    L.pressure = benchPressureRescale(L.pressure, opts.logShuntOhms);
    segs = [segs jogSegments(L, cfg, 'unloaded', filesU{k})]; %#ok<AGROW>
end
for k = 1:numel(filesL)
    L = readLog(filesL{k});
    L.pressure = benchPressureRescale(L.pressure, opts.logShuntOhms);
    segs = [segs jogSegments(L, cfg, 'loaded', filesL{k})]; %#ok<AGROW>
    bins = [bins pressureBins(L, cfg, filesL{k})]; %#ok<AGROW>
end
r.segments = segs;
r.bins = bins;
r.cfg = cfg;

say('\nforce = torque [%%] * %.1f N: %.2f N m rated, %g:1, %g mm lead, lossless\n', ...
    cfg.newtonPerPct, cfg.ratedTorqueNm, cfg.gearRatio, cfg.leadMm);
say('piston area %.4g m^2: 1 MPa is %.0f N, %.1f %% of rated\n', cfg.pistonAreaM2, ...
    1e6 * cfg.pistonAreaM2, 1e6 * cfg.pistonAreaM2 / cfg.newtonPerPct);

if ~isempty(segs)
    say('\n%-9s %-4s %7s %7s %7s %6s %6s %8s %7s %6s %6s  %s\n', 'set', 'step', 'v cmd', ...
        'v mean', 'P MPa', 'leads', 'n', 'F N', 'ripple', 'drift', 'stick', 'checks');
    for s = segs
        say('%-9s %-4d %7.3f %7.3f %7.3f %6.1f %6d %8.0f %7.0f %6.0f %6.2f  %s\n', s.set, ...
            s.step, s.vRef, s.v, s.P, s.leads, s.n, s.F, s.Fstd, s.drift, s.stickFrac, s.checks);
    end
    say('v in mm/s from the position slope; ripple is the sample std and drift the slope times the duration, N\n');
end
if ~isempty(bins)
    say('\npressure-mode bins (%g s): %d of %d moving steadily\n', cfg.binSeconds, ...
        sum([bins.used]), numel(bins));
end

% --- Breakaway ---------------------------------------------------------------
bw = [];
if ~isempty(segs)
    bw = segs(arrayfun(@(s) ~isnan(s.Fbreak), segs));
end
if ~isempty(bw)
    say('\nbreakaway (pressure force removed): %-9s %-4s %7s %5s %9s %9s\n', 'set', 'step', 'v cmd', 'n', 'F peak N', '/ F slide');
    for s = bw
        say('           %-9s %-4d %7.3f %5d %9.0f %9.2f\n', s.set, s.step, s.vRef, s.nBreak, ...
            s.Fbreak, s.Fbreak / (s.F - s.PA));
    end
    r.breakaway = struct('F', mean(abs([bw.Fbreak])), 'n', numel(bw), ...
        'few', mean([bw.nBreak] < 3));
    say('           mean |F peak| %.0f N over %d starts; %.0f %% of them caught in fewer than 3 samples\n', ...
        r.breakaway.F, r.breakaway.n, 100 * r.breakaway.few);
else
    r.breakaway = struct('F', NaN, 'n', 0, 'few', NaN);
end

% --- Fits --------------------------------------------------------------------
segsU = []; segsL = []; binsL = [];
if ~isempty(segs)
    segsU = segs(strcmp({segs.set}, 'unloaded'));
    segsL = segs(strcmp({segs.set}, 'loaded'));
end
if ~isempty(bins)
    binsL = bins([bins.used]);
end
r.unloaded = fitUnloaded(segsU, cfg);
r.loaded = fitLoaded(segsL, binsL, r.unloaded, cfg);
r.pairs = reversalPairs(segs, r.unloaded, r.loaded);

if ~isempty(r.pairs)
    say('\nreversal pairs: %-9s %-7s %7s %9s %9s %9s  %s\n', 'set', 'steps', '|v|', 'step N', ...
        'offset N', 'P A N', 'check');
    for p = r.pairs
        say('                %-9s %3d/%-3d %7.3f %9.0f %9.0f %9.0f  %s\n', p.set, p.steps(1), ...
            p.steps(2), p.v, p.halfStep, p.offset, p.PA, p.check);
    end
    say('                step = (F+ - F-)/2, the Coulomb part and the load term; offset = (F+ + F-)/2 - P A, the constant F0\n');
end

printFit(say, 'unloaded', r.unloaded, cfg);
printFit(say, 'loaded', r.loaded, cfg);
printParams(say, r, cfg);
end


% ===========================================================================
function files = expandFiles(arg)
% A file, a pattern or a cell array of either, as a cell array of paths.
files = {};
if isempty(arg)
    return;
end
if ischar(arg)
    arg = {arg};
end
for k = 1:numel(arg)
    d = dir(arg{k});
    d = d(~[d.isdir]);
    if isempty(d) && exist(arg{k}, 'file') == 2
        files{end + 1} = arg{k}; %#ok<AGROW>
        continue;
    end
    for j = 1:numel(d)
        files{end + 1} = fullfile(d(j).folder, d(j).name); %#ok<AGROW>
    end
end
end


function L = readLog(file)
% The frictionRun CSV as a struct of columns.
fid = fopen(file, 'r');
if fid < 0
    error('cannot open %s', file);
end
header = fgetl(fid);
fclose(fid);
names = strsplit(strtrim(header), ',');
data = dlmread(file, ',', 1, 0);
if size(data, 2) ~= numel(names)
    error('%s: %d columns, header names %d', file, size(data, 2), numel(names));
end
for k = 1:numel(names)
    L.(names{k}) = data(:, k);
end
L.file = file;
end


function segs = jogSegments(L, cfg, set, file)
% One segment per jog step: statistics of its steady part and its
% breakaway.
segs = [];
lead = cfg.leadMm;
dt = median(diff(L.t));
for step = unique(L.step(L.mode == 3 & L.step > 0))'
    idx = find(L.step == step);
    cmd = idx(L.ref(idx) ~= 0);
    if isempty(cmd)
        continue;
    end
    on = cmd(1);
    off = cmd(end);
    vRef = L.ref(on);
    tOn = L.t(on);
    tOff = L.t(off);
    ramp = abs(vRef) / cfg.accel;
    win = idx(L.t(idx) >= tOn + ramp + cfg.settle & L.t(idx) <= tOff - cfg.trim);
    s.set = set;
    s.file = file;
    s.step = step;
    s.vRef = vRef;
    s.checks = '';
    if numel(win) < 4
        s.n = numel(win);
        s = emptySeg(s);
        s.checks = 'too short';
        segs = [segs s]; %#ok<AGROW>
        continue;
    end
    tol = max(cfg.vTolAbs, cfg.vTolRel * abs(vRef));
    atSpeed = abs(L.velocity(win) - vRef) <= tol;
    if mean(atSpeed) >= 0.5
        win = win(atSpeed);
    end
    t = L.t(win);
    F = L.torque(win) * cfg.newtonPerPct;
    x = L.position(win);
    v = L.velocity(win);
    dur = t(end) - t(1);
    s.n = numel(win);
    s.t0 = t(1);
    s.t1 = t(end);
    s.vIr = mean(v);
    s.v = (x(end) - x(1)) / dur;
    if abs(s.v) < cfg.vMin
        s.v = s.vIr;
    end
    s.vStd = std(v);
    s.dist = abs(x(end) - x(1));
    s.leads = s.dist / lead;
    s.F = mean(F);
    s.Fstd = std(F);
    pf = polyfit(t - t(1), F, 1);
    s.drift = pf(1) * dur;
    half = floor(numel(F) / 2);
    s.halfDiff = mean(F(1:half)) - mean(F(end - half + 1:end));
    s.stickFrac = mean(abs(v) < cfg.vMin);
    s.P = mean(L.pressure(win));
    s.Pmin = min(L.pressure(win));
    s.Pmax = max(L.pressure(win));
    s.PA = s.P * 1e6 * cfg.pistonAreaM2;
    % Ripple at the lead period, when the log can resolve it.
    fLead = abs(s.v) / lead;
    if fLead < 0.5 / dt && fLead > 1 / dur
        s.leadAmp = 2 / numel(F) * abs(sum((F - s.F) .* exp(-2i * pi * fLead * (t - t(1)))));
    else
        s.leadAmp = NaN;
    end
    % Breakaway: from the command to the first sample moving.
    moving = find(abs(L.velocity(on:off)) > cfg.vMin, 1, 'first');
    if isempty(moving)
        bw = on:off;
    else
        bw = on:on + moving - 1;
    end
    Ff = L.torque(bw) * cfg.newtonPerPct - L.pressure(bw) * 1e6 * cfg.pistonAreaM2;
    [~, i] = max(abs(Ff));
    s.nBreak = numel(bw);
    s.Fbreak = Ff(i);
    % Checks.
    flags = {};
    if abs(s.drift) > max(s.Fstd, 0.03 * abs(s.F))
        flags{end + 1} = 'drift';
    end
    if s.leads < 1
        flags{end + 1} = sprintf('%.1f lead', s.leads);
    end
    if abs(s.v - vRef) > tol
        flags{end + 1} = 'off speed';
    end
    if s.stickFrac > 0.1
        flags{end + 1} = 'stick-slip';
    end
    if isempty(flags)
        s.checks = 'ok';
    else
        s.checks = strjoin(flags, ', ');
    end
    segs = [segs s]; %#ok<AGROW>
end
end


function s = emptySeg(s)
for f = {'t0', 't1', 'vIr', 'v', 'vStd', 'dist', 'leads', 'F', 'Fstd', 'drift', ...
        'halfDiff', 'stickFrac', 'P', 'Pmin', 'Pmax', 'PA', 'leadAmp', 'nBreak', 'Fbreak'}
    s.(f{1}) = NaN;
end
end


function bins = pressureBins(L, cfg, file)
% Points from pressure-mode runs: bins of samples moving steadily.
bins = [];
idx = find(L.mode == 7 & L.status == 5);
if numel(idx) < 4
    return;
end
dt = median(diff(L.t));
v = L.velocity(idx);
nSmooth = max(1, round(0.25 / dt));
vs = filter(ones(1, nSmooth) / nSmooth, 1, v);
a = [0; diff(vs)] / dt;
ok = abs(v) >= cfg.vMin & abs(a) <= cfg.aMax & L.pressure(idx) > 0.1;
b0 = floor((L.t(idx) - L.t(idx(1))) / cfg.binSeconds);
for k = unique(b0)'
    sel = idx(b0 == k & ok);
    bin.file = file;
    bin.n = numel(sel);
    bin.used = bin.n >= 0.5 * cfg.binSeconds / dt;
    if bin.n == 0
        bin.t = L.t(idx(find(b0 == k, 1)));
        bin.v = NaN; bin.F = NaN; bin.Fstd = NaN; bin.P = NaN; bin.PA = NaN;
    else
        bin.t = mean(L.t(sel));
        bin.v = mean(L.velocity(sel));
        bin.F = mean(L.torque(sel)) * cfg.newtonPerPct;
        bin.Fstd = std(L.torque(sel)) * cfg.newtonPerPct;
        bin.P = mean(L.pressure(sel));
        bin.PA = bin.P * 1e6 * cfg.pistonAreaM2;
    end
    bins = [bins bin]; %#ok<AGROW>
end
end


function f = fitUnloaded(segs, cfg)
% F = F0 + Fc s + b v, and the Stribeck form.
f.n = 0;
f.ok = false;
segs = segs(arrayfun(@(s) ~isnan(s.F), segs));
if numel(segs) < 3
    return;
end
F = [segs.F]';
v = [segs.v]';
s = sign(v);
f.n = numel(F);
f.points = [v F];
[th, se, rms, res] = lsq([ones(size(v)) s v], F);
f.ok = true;
f.F0 = th(1); f.Fc = th(2); f.b = th(3);
f.se = struct('F0', se(1), 'Fc', se(2), 'b', se(3));
f.rms = rms;
f.res = res;
f.stribeck.ok = false;
if sum(unique(abs(v)) <= 1) >= 2 && f.n >= 5
    best = [];
    for vs = cfg.stribeckVs
        X = [ones(size(v)) s v s .* exp(-(abs(v) / vs).^2)];
        [th2, se2, rms2, res2] = lsq(X, F);
        if isempty(best) || rms2 < best.rms
            best = struct('vs', vs, 'th', th2, 'se', se2, 'rms', rms2, 'res', res2);
        end
    end
    f.stribeck.vs = best.vs;
    f.stribeck.F0 = best.th(1); f.stribeck.Fc = best.th(2); f.stribeck.b = best.th(3);
    f.stribeck.dFs = best.th(4);
    f.stribeck.Fs = best.th(2) + best.th(4);
    f.stribeck.se = struct('Fc', best.se(2), 'b', best.se(3), 'dFs', best.se(4));
    f.stribeck.rms = best.rms;
    f.stribeck.res = best.res;
    f.stribeck.ok = best.th(4) > 0 && best.rms <= 0.8 * rms;
end
end


function f = fitLoaded(segs, bins, fu, cfg)
% F - P A = F0 + s (Fc + mu P A) + b v, with the fallbacks described in
% the header when the points cannot separate the terms.
f.n = 0;
f.ok = false;
F = [col(segs, 'F') col(bins, 'F')]';
v = [col(segs, 'v') col(bins, 'v')]';
PA = [col(segs, 'PA') col(bins, 'PA')]';
keep = ~isnan(F);
F = F(keep); v = v(keep); PA = PA(keep);
if numel(F) < 3
    return;
end
s = sign(v);
R = F - PA;
f.n = numel(F);
f.points = [v PA F];
f.ok = true;
Xfull = [ones(size(v)) s v s .* PA];
if numel(F) >= 5 && wellPosed(Xfull)
    [th, se, rms, res] = lsq(Xfull, R);
    f.form = 'full';
    f.F0 = th(1); f.Fc = th(2); f.b = th(3); f.mu = th(4);
    f.se = struct('F0', se(1), 'Fc', se(2), 'b', se(3), 'mu', se(4));
elseif fu.ok && wellPosed([ones(size(v)) s s .* PA])
    [th, se, rms, res] = lsq([ones(size(v)) s s .* PA], R - fu.b * v);
    f.form = 'b fixed to the unloaded value';
    f.F0 = th(1); f.Fc = th(2); f.b = fu.b; f.mu = th(3);
    f.se = struct('F0', se(1), 'Fc', se(2), 'b', fu.se.b, 'mu', se(3));
elseif wellPosed([ones(size(v)) s v])
    [th, se, rms, res] = lsq([ones(size(v)) s v], R);
    f.form = sprintf('mu fixed to 0: Fc includes the load term at %.1f MPa', mean(PA) / 1e6 / cfg.pistonAreaM2);
    f.F0 = th(1); f.Fc = th(2); f.b = th(3); f.mu = 0;
    f.se = struct('F0', se(1), 'Fc', se(2), 'b', se(3), 'mu', NaN);
else
    bFix = 0;
    if fu.ok
        bFix = fu.b;
    end
    [th, se, rms, res] = lsq([ones(size(v)) s], R - bFix * v);
    f.form = sprintf('one direction: F0 and Fc not separable, b fixed, mu 0; Fc includes the load term at %.1f MPa', ...
        mean(PA) / 1e6 / cfg.pistonAreaM2);
    f.F0 = th(1); f.Fc = th(2); f.b = bFix; f.mu = 0;
    f.se = struct('F0', se(1), 'Fc', se(2), 'b', NaN, 'mu', NaN);
end
f.rms = rms;
f.res = res;
f.etaForward = 1 / (1 + f.mu);
f.etaBackward = 1 - f.mu;
end


function x = col(a, name)
% One field of a struct array as a row, [] when the array is empty.
if isempty(a)
    x = [];
else
    x = [a.(name)];
end
end


function ok = wellPosed(X)
% Columns scaled to unit size must be independent enough to separate.
sc = max(abs(X), [], 1);
sc(sc == 0) = 1;
Xn = X ./ repmat(sc, size(X, 1), 1);
ok = size(X, 1) > size(X, 2) && rank(Xn) == size(X, 2) && cond(Xn) < 1e4;
end


function [th, se, rms, res] = lsq(X, y)
% Least squares with standard errors from the residual variance.
th = X \ y;
res = y - X * th;
dof = max(size(X, 1) - size(X, 2), 1);
sigma2 = (res' * res) / dof;
C = sigma2 * pinv(X' * X);
se = sqrt(max(diag(C), 0));
rms = sqrt(mean(res.^2));
end


function pairs = reversalPairs(segs, fu, fl)
% Consecutive jog steps at opposite commands of the same size.
pairs = [];
for k = 1:numel(segs) - 1
    a = segs(k);
    b = segs(k + 1);
    if ~strcmp(a.file, b.file) || a.vRef ~= -b.vRef || isnan(a.F) || isnan(b.F)
        continue;
    end
    if a.vRef > 0
        p.Fplus = a.F; p.Fminus = b.F;
    else
        p.Fplus = b.F; p.Fminus = a.F;
    end
    p.set = a.set;
    p.steps = [a.step b.step];
    p.v = abs(a.vRef);
    p.PA = (a.PA + b.PA) / 2;
    p.halfStep = (p.Fplus - p.Fminus) / 2;
    p.offset = (p.Fplus + p.Fminus) / 2 - p.PA;
    if strcmp(a.set, 'unloaded')
        f = fu;
    else
        f = fl;
    end
    if f.ok
        p.F0 = f.F0;
    else
        p.F0 = 0;
    end
    if abs(p.offset - p.F0) <= 0.1 * abs(p.halfStep)
        p.check = 'ok';
    else
        p.check = sprintf('offset %.0f N from F0', p.offset - p.F0);
    end
    pairs = [pairs p]; %#ok<AGROW>
end
end


function printFit(say, name, f, cfg)
if ~f.ok
    say('\n%s fit: not enough points\n', name);
    return;
end
say('\n%s fit, %d points, residual rms %.0f N (%.2f %% of rated):\n', name, f.n, f.rms, ...
    f.rms / cfg.newtonPerPct);
say('  F0 = %8.0f +/- %-6.0f N      constant offset\n', f.F0, f.se.F0);
say('  Fc = %8.0f +/- %-6.0f N      Coulomb\n', f.Fc, f.se.Fc);
say('  b  = %8.0f +/- %-6.0f N s/m  viscous\n', f.b * 1000, f.se.b * 1000);
if isfield(f, 'mu')
    say('  mu = %8.3f +/- %-6.3f        load-proportional friction: efficiency %.3f pushing, %.3f pushed (%s)\n', ...
        f.mu, f.se.mu, f.etaForward, f.etaBackward, f.form);
end
if isfield(f, 'stribeck') && isfield(f.stribeck, 'vs')
    st = f.stribeck;
    if st.ok
        verdict = 'kept';
    else
        verdict = 'not kept';
    end
    say('  Stribeck form: Fs = %.0f N (Fc %.0f + %.0f +/- %.0f), knee %.2f mm/s, b %.0f N s/m, rms %.0f N: %s\n', ...
        st.Fs, st.Fc, st.dFs, st.se.dFs, st.vs, st.b * 1000, st.rms, verdict);
end
say('  residual per point (N):%s\n', sprintf(' %.0f', f.res));
end


function printParams(say, r, cfg)
say('\ndesign/pressure-loop/params.py values (SI):\n');
fu = r.unloaded;
fl = r.loaded;
if fu.ok
    if fu.stribeck.ok
        src = fu.stribeck;
        form = 'Stribeck form';
    else
        src = fu;
        form = 'unloaded fit';
    end
    say('    b    = %-10.4g  # [N s/m] viscous friction, %s (+/- %.2g)\n', ...
        src.b * 1000, form, src.se.b * 1000);
    say('    Fc   = %-10.4g  # [N] Coulomb friction of the seals and screw, %s (+/- %.2g)\n', ...
        src.Fc, form, src.se.Fc);
    if fu.stribeck.ok
        say('    # Stribeck: Fs = %.0f N falling to Fc over %.2f mm/s. The model has no falling\n', ...
            fu.stribeck.Fs, fu.stribeck.vs);
        say('    # term; veps smooths the sign only, keep it below %.1e m/s\n', fu.stribeck.vs / 1000 / 5);
    else
        say('    # no Stribeck region resolved: veps stays numerical (1e-5 m/s)\n');
    end
end
if fl.ok
    say('    # under load: Fc = %.4g N, b = %.4g N s/m, mu = %.3f (%s)\n', fl.Fc, fl.b * 1000, ...
        fl.mu, fl.form);
    eta = fl.etaForward;
    src = sprintf('measured mu %.3f', fl.mu);
    if isnan(fl.se.mu)
        eta = cfg.etaAssumed;
        src = sprintf('mu not separable, efficiency assumed %.2f', cfg.etaAssumed);
    end
else
    eta = cfg.etaAssumed;
    src = sprintf('no loaded set, efficiency assumed %.2f', cfg.etaAssumed);
end
say('    Fcap = %-10.4g  # [N] 100 %% rated torque at the piston, %s\n', ...
    100 * cfg.newtonPerPct * eta, src);
if ~isnan(r.breakaway.F)
    say('    # breakaway %.0f N from rest, mean of %d starts\n', r.breakaway.F, r.breakaway.n);
end
end


function printUnless(quiet, varargin)
if ~quiet
    fprintf(varargin{:});
end
end
