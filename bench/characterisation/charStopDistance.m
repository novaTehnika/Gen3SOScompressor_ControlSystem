function r = charStopDistance(files, opts)
%CHARSTOPDISTANCE Stopping distance of the piston under load from the
%   one-pass run of test 4 (README section 6): every command step to zero
%   in a velocity session is a stop, and the travel from the command to
%   rest, against the speed, the pressure and the direction, is what the
%   slave's stop law has to allow for.
%
%   r = charStopDistance(file)
%   r = charStopDistance({capture1, ..., runLog1, ...}, opts)
%
%   Run logs (charRun or frictionRun, 50 ms: t, step, status, mode, ref,
%   position, velocity, torque, pressure, ...) give every stop: the row
%   whose reference went to zero from a speed, in velocity mode, is the
%   command; the piston is at rest when the velocity has stayed below
%   opts.vRest for opts.restSamples rows. The command leaves at the
%   start of its exchange and the row's position is read at the end of
%   it, by which time the slave has acted, so the position at the
%   command is taken from the row before, advanced at its velocity to
%   the command row's time. Captures (PRG_StepProbe, 2 ms: t, vCmd,
%   velocity, torque, position, pressure) give the same stops at the
%   scan rate where the probe was armed for them, with no such
%   correction; the log resolves a stop to about half an exchange, the
%   capture to a scan.
%
%   The fit, per direction (compression, the pressure force against the
%   motion; return, with it), is the two-term law the slave uses,
%     x_stop = T0 v + v^2 / (2 D),
%   by least squares with both terms held non-negative; T0 is the delay
%   before the drive acts and D the deceleration it then achieves. With
%   enough stops at a level, D is also fitted per level with T0 held, to
%   show the load dependence. The slave's own allowance, v lag + v^2 /
%   (2 G_cfgPressureDecel), is compared with every measured stop: the law
%   is conservative where the measured distance is below it.
%
%   opts.vRest         mm/s, rest threshold (default 0.05)
%   opts.restSamples   rows or scans below vRest that define rest (default
%                      3 for the log, 10 for a capture)
%   opts.minSpeed      mm/s, smaller commands are not stops (default 0.2)
%   opts.lag_s         the slave's delay term for the comparison (default
%                      0.05: filter, median, pipeline, dead time and drive)
%   opts.D_cfg         mm/s^2 for the comparison (default charConfig:
%                      G_cfgPressureDecel)
%   opts.A_mm2         piston area for the load column (charConfig)
%   opts.maxCycle_s    longest log exchange cycle whose stops are fitted
%                      alongside better ones (default 0.04)
%   opts.quiet         no printing
%
%   r.stops      one per stop: source ('log' or 'capture'), file, t0,
%                v_mm_s (signed), P_MPa at the command, dir (+1
%                compression, -1 return), x_mm (travel to rest), dP_MPa
%                (the pressure's own excursion during the stop, signed with
%                the motion), torquePeak, t_stop_s, x_cfg_mm (the slave's
%                allowance), creep_mm (further travel in the direction of
%                the motion over the second after rest, with dPcreep_MPa
%                the pressure change over it; the machine showed none),
%                resolved (false for a log stop whose exchange cycle was
%                too long to place the command, which the fit leaves out
%                when captures or resolved stops exist), and alarm (a
%                fault status within the stop)
%
%   A log resolves a stop only when its exchange cycle is short against
%   the stop: the command's position is extrapolated over one cycle, so
%   a cycle of 90 ms puts 0.09 mm of uncertainty on a stop of 0.03 mm at
%   1 mm/s. Log stops from cycles over opts.maxCycle_s are listed but not
%   fitted when anything better exists.
%   r.fit.comp, r.fit.ret
%                per direction: T0_s, D_mm_s2, n, residual_mm (rms), and
%                perLevel (P_MPa, D_mm_s2, n) where fitted
%   r.T0_s, r.D_mm_s2
%                the values for the slave: the larger T0 and the smaller
%                D of the two directions
%   r.pass, r.checks

if ischar(files)
    files = {files};
end
if nargin < 2
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'common'));
cfg = charConfig();
opts = withDefault(opts, 'vRest', 0.05);
opts = withDefault(opts, 'restSamples', []);
opts = withDefault(opts, 'minSpeed', 0.2);
opts = withDefault(opts, 'lag_s', 0.05);
opts = withDefault(opts, 'D_cfg', cfg.pressureDecel);
opts = withDefault(opts, 'A_mm2', cfg.pistonArea_mm2);
opts = withDefault(opts, 'maxCycle_s', 0.04);
opts = withDefault(opts, 'quiet', false);
say = @(varargin) sayIf(~opts.quiet, varargin{:});

stops = emptyStops();
for f = 1:numel(files)
    d = benchReadCsv(files{f});
    if isfield(d, 'vCmd')
        stops = [stops stopsOfCapture(d, opts)];             %#ok<AGROW>
    else
        stops = [stops stopsOfLog(d, opts)];                 %#ok<AGROW>
    end
end
if isempty(stops)
    error('charStopDistance: no stop (a reference step to zero from at least %.1f mm/s) in %d file(s)', ...
        opts.minSpeed, numel(files));
end
for k = 1:numel(stops)
    v = abs(stops(k).v_mm_s);
    stops(k).x_cfg_mm = v * opts.lag_s + v^2 / (2 * opts.D_cfg);
end
r.stops = stops;

% --- Report the stops ------------------------------------------------------------
say('stops: %d (%d from captures, %d from the log)\n', numel(stops), ...
    sum(strcmp({stops.source}, 'capture')), sum(strcmp({stops.source}, 'log')));
say('%-8s %7s %7s %5s %9s %9s %8s %9s %8s %8s %6s\n', 'source', 'P MPa', 'v mm/s', 'dir', ...
    'x mm', 'x cfg mm', 'dP MPa', 'creep mm', 'T peak', 't stop', 'alarm');
for k = 1:numel(stops)
    s = stops(k);
    say('%-8s %7.2f %7.2f %5s %9.3f %9.3f %8.3f %9.3f %8.1f %8.3f %6s\n', s.source, s.P_MPa, ...
        abs(s.v_mm_s), dirText(s.dir), s.x_mm, s.x_cfg_mm, s.dP_MPa, s.creep_mm, s.torquePeak, ...
        s.t_stop_s, yesNo(s.alarm));
end
creep = [stops.creep_mm];
if any(~isnan(creep))
    say('creep after rest: median %.3f mm, max %.3f mm (%.2f MPa) over the second after the velocity reads zero\n', ...
        median(creep(~isnan(creep))), max(creep), max([stops.dPcreep_MPa]));
end

% --- Fit per direction -----------------------------------------------------------
% Unresolved log stops are left out when resolved stops or captures exist
resolved = [stops.resolved];
if any(resolved)
    fitSet = stops(resolved);
    if ~all(resolved)
        say('%d log stops from exchange cycles over %.0f ms are not fitted\n', ...
            sum(~resolved), 1000 * opts.maxCycle_s);
    end
else
    fitSet = stops;
    say('every stop comes from a log with exchange cycles over %.0f ms: the fit is coarse\n', ...
        1000 * opts.maxCycle_s);
end
r.fit = struct();
names = {'comp', 'ret'};
dirs = [1 -1];
for j = 1:2
    sel = fitSet([fitSet.dir] == dirs(j) & ~[fitSet.alarm]);
    fit = struct('T0_s', NaN, 'D_mm_s2', NaN, 'n', numel(sel), 'residual_mm', NaN, ...
        'perLevel', struct('P_MPa', {}, 'D_mm_s2', {}, 'n', {}));
    if numel(sel) >= 2
        v = abs([sel.v_mm_s])';
        x = [sel.x_mm]';
        th = lsqnonneg([v v.^2], x);                    % [T0, 1/(2 D)]
        fit.T0_s = th(1);
        fit.D_mm_s2 = 1 / (2 * max(th(2), 1e-9));
        fit.residual_mm = sqrt(mean((x - [v v.^2] * th).^2));
        % D per level with T0 held: the load dependence
        levels = unique(round([sel.P_MPa] * 2) / 2);
        for P = levels
            at = abs([sel.P_MPa] - P) <= 0.25;
            if sum(at) >= 2
                vv = abs([sel(at).v_mm_s])';
                xx = [sel(at).x_mm]' - fit.T0_s * vv;
                c = lsqnonneg(vv.^2, xx);
                fit.perLevel(end + 1) = struct('P_MPa', P, 'D_mm_s2', 1 / (2 * max(c, 1e-9)), ...
                    'n', sum(at));                                                      %#ok<AGROW>
            end
        end
    end
    r.fit.(names{j}) = fit;
    if ~isnan(fit.T0_s)
        say('%s: x_stop = %.3f s * v + v^2 / (2 * %.1f mm/s^2), %d stops, rms residual %.3f mm\n', ...
            dirText(dirs(j)), fit.T0_s, fit.D_mm_s2, fit.n, fit.residual_mm);
        for q = 1:numel(fit.perLevel)
            say('    at %.1f MPa: D %.1f mm/s^2 from %d stops\n', fit.perLevel(q).P_MPa, ...
                fit.perLevel(q).D_mm_s2, fit.perLevel(q).n);
        end
    else
        say('%s: fewer than two stops, no fit\n', dirText(dirs(j)));
    end
end
r.T0_s = max([r.fit.comp.T0_s r.fit.ret.T0_s]);
r.D_mm_s2 = min([r.fit.comp.D_mm_s2 r.fit.ret.D_mm_s2]);

% --- Checks ------------------------------------------------------------------------
worst = max([stops.x_mm] - [stops.x_cfg_mm]);
r.checks = {check('law conservative', worst, -Inf, 0, ...
    'every measured stop within the slave''s allowance v lag + v^2 / (2 D_cfg) (worst excess, mm)'), ...
    check('no alarm', sum([stops.alarm]), 0, 0, 'stops that ended in a fault'), ...
    check('both directions', isnan(r.D_mm_s2), 0, 0, 'a fit in each direction (0 = yes)')};
if ~isnan(r.fit.comp.residual_mm) && ~isnan(r.fit.ret.residual_mm)
    xm = mean([stops.x_mm]);
    r.checks{end + 1} = check('fit residual', max(r.fit.comp.residual_mm, r.fit.ret.residual_mm) / xm, ...
        0, 0.2, 'rms residual of the law over the mean stopping distance');
end
r.pass = all(cellfun(@(c) c.pass, r.checks));
printChecks(say, r.checks);
if ~isnan(r.D_mm_s2)
    say('GVL: G_cfgPressureDecel at or below %.0f mm/s^2 with a lag of %.3f s is what the stops support ', ...
        r.D_mm_s2, r.T0_s);
    say('(configured %.0f mm/s^2 and %.3f s)\n', opts.D_cfg, opts.lag_s);
end
end


% ===========================================================================
function stops = stopsOfLog(d, opts)
% Every reference step to zero in velocity mode (mode 3, status 4).
rest = opts.restSamples;
if isempty(rest)
    rest = 3;
end
stops = emptyStops();
inVel = d.mode == 3;
n = numel(d.t);
cycle = median(diff(d.t));                                  % [s]
for i = 2:n
    if ~(inVel(i - 1) && inVel(i)) || d.ref(i) ~= 0 || abs(d.ref(i - 1)) < opts.minSpeed
        continue;
    end
    s = stopFrom(d.t, d.position, d.velocity, d.torque, d.pressure, i, d.ref(i - 1), rest, opts, true);
    if isempty(s)
        continue;
    end
    j = i:min(n, i + 60);
    s.alarm = any(d.status(j) == 10);
    s.resolved = cycle <= opts.maxCycle_s;
    s.source = 'log';
    s.file = d.file;
    stops(end + 1) = s;                                     %#ok<AGROW>
end
end


function stops = stopsOfCapture(d, opts)
% The command step to zero within a capture, if it holds one.
rest = opts.restSamples;
if isempty(rest)
    rest = 10;
end
stops = emptyStops();
i = find(d.vCmd(2:end) == 0 & abs(d.vCmd(1:end - 1)) >= opts.minSpeed, 1) + 1;
if isempty(i)
    return;
end
if ~isfield(d, 'pressure')
    d.pressure = zeros(size(d.t));
end
s = stopFrom(d.t, d.position, d.velocity, d.torque, d.pressure, i, d.vCmd(i - 1), rest, opts, false);
if isempty(s)
    return;
end
s.alarm = false;
s.resolved = true;
s.source = 'capture';
s.file = d.file;
stops(end + 1) = s;
end


function s = stopFrom(t, x, v, trq, P, i, vCmd, rest, opts, fromLog)
% The stop commanded at sample i from the speed vCmd: travel to rest, the
% pressure excursion with the motion, the peak torque and the stop time.
% Empty when the piston does not come to rest in the record. From a log
% the position at the command is the previous row's, advanced at that
% row's velocity to the command row's time (the row's own position was
% read after the slave had acted).
n = numel(t);
if fromLog && i > 1 && abs(v(i - 1)) >= 0.5 * abs(vCmd)
    x0 = x(i - 1) + v(i - 1) * (t(i) - t(i - 1));
    P0 = P(i - 1);
else
    x0 = x(i);
    P0 = P(i);
end
j = i;
below = 0;
while j < n
    j = j + 1;
    if abs(v(j)) < opts.vRest
        below = below + 1;
        if below >= rest
            break;
        end
    else
        below = 0;
    end
end
if below < rest || t(j) - t(i) > 5
    s = [];
    return;
end
jRest = j - rest + 1;
dirSign = sign(vCmd);
s.source = '';
s.file = '';
s.t0 = t(i);
s.v_mm_s = vCmd;
s.P_MPa = P0;
s.dir = dirSign;
s.x_mm = dirSign * (x(jRest) - x0);
s.dP_MPa = dirSign * (P(jRest) - P0);
s.torquePeak = max(abs(trq(i:j)));
s.t_stop_s = t(jRest) - t(i);
s.x_cfg_mm = NaN;
% Creep: travel in the direction of the motion over the second after the
% velocity first read zero, while the command stays at zero
jEnd = jRest;
while jEnd < n && t(jEnd + 1) - t(jRest) <= 1.0
    jEnd = jEnd + 1;
end
if t(jEnd) - t(jRest) >= 0.3
    s.creep_mm = dirSign * (x(jEnd) - x(jRest));
    s.dPcreep_MPa = dirSign * (P(jEnd) - P(jRest));
else
    s.creep_mm = NaN;
    s.dPcreep_MPa = NaN;
end
s.resolved = true;
s.alarm = false;
end


function stops = emptyStops()
stops = struct('source', {}, 'file', {}, 't0', {}, 'v_mm_s', {}, 'P_MPa', {}, ...
    'dir', {}, 'x_mm', {}, 'dP_MPa', {}, 'torquePeak', {}, 't_stop_s', {}, ...
    'x_cfg_mm', {}, 'creep_mm', {}, 'dPcreep_MPa', {}, 'resolved', {}, 'alarm', {});
end


function txt = dirText(dirSign)
if dirSign > 0
    txt = 'comp';
else
    txt = 'ret';
end
end


function txt = yesNo(b)
if b
    txt = 'yes';
else
    txt = 'no';
end
end


function c = check(name, value, lo, hi, text)
c = struct('name', name, 'value', value, 'lo', lo, 'hi', hi, 'text', text, ...
    'pass', value >= lo && value <= hi);
end


function printChecks(say, checks)
for k = 1:numel(checks)
    c = checks{k};
    if c.pass
        v = 'pass';
    else
        v = 'FAIL';
    end
    say('  %-18s %s  %g  (%s)\n', c.name, v, c.value, c.text);
end
end


function sayIf(on, varargin)
if on
    fprintf(varargin{:});
end
end


function opts = withDefault(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end
