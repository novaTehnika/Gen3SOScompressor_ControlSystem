function r = charOrificeFlow(files, opts)
%CHARORIFICEFLOW Orifice bank discharge coefficient Cd from deliveries at a
%   held pressure logged by frictionRun (README section 4).
%
%   r = charOrificeFlow(file)
%   r = charOrificeFlow({file1, file2, ...}, opts)
%
%   In a pressure-mode step the piston stands still once the target is
%   reached; when the outlet valve opens the pressure dips and the loop
%   drives the piston at the velocity that balances the outflow. In each
%   steady window (pressure drift below opts.settleRate, after the tube has
%   filled) the outflow is Q = A dx/dt - (V/K) dP/dt, with the second term
%   the compressibility correction (small when the pressure is steady), and
%   Cd = Q / (A_o sqrt(2 P / rho)), eq. (36) of the design README. With
%   windows at two or more pressures the exponent of Q = c P^n is fitted to
%   confirm the square-root law. Steady windows longer than opts.subWindow
%   are also split so Q can be seen to hold along the stroke.
%
%   opts.K_Pa            bulk modulus for the compressibility correction and
%                        the decompression in the collected volume
%                        (default 1e9; use charBulkModulus's result)
%   opts.deadVolume_mL   measured dead volume (mL); default charConfig
%   opts.fillVolume_mL   liquid volume at zero gauge at the start of each
%                        file (mL), scalar or one per file; default geometry
%   opts.collected_mL    volume collected per file (mL, NaN to skip), for
%                        the gravimetric check; by default over the valve
%                        opening to the end of the pressure step
%   opts.collectWindow   [t1 t2] run clock per file (rows) for collected_mL
%   opts.tOpen           valve opening time per file (s on the run clock);
%                        default detected from the velocity and pressure
%   opts.settleRate      MPa/s drift limit for a steady window (default 0.05)
%   opts.minWindow       s, shortest steady window kept (default 3)
%   opts.subWindow       s, length of the sub-windows (default 10)
%   opts.quiet           no printing
%
%   r.windows   one row per window: file, step, t1, t2, P_MPa, Q_mL_s,
%               v_mm_s, Cd, dPdt
%   r.Cd        discharge coefficient for params.py (sqrt-law fit through
%               every window, or the mean when one pressure only)
%   r.exponent  fitted exponent n of Q = c P^n (NaN with one pressure)
%   r.pass, r.checks

if ischar(files)
    files = {files};
end
if nargin < 2
    opts = struct();
end
cfg = charConfig();
opts = withDefault(opts, 'K_Pa', 1e9);
opts = withDefault(opts, 'deadVolume_mL', cfg.deadVolume_mL);
opts = withDefault(opts, 'collected_mL', nan(1, numel(files)));
opts = withDefault(opts, 'settleRate', 0.05);
opts = withDefault(opts, 'minWindow', 3);
opts = withDefault(opts, 'subWindow', 10);
opts = withDefault(opts, 'quiet', false);
opts = withDefault(opts, 'vOpen', 0.2);
A = cfg.pistonArea_mm2;             % mm^2
Ao = cfg.orificeArea_m2;
rho = cfg.rho;
K = opts.K_Pa / 1e6;                % MPa
say = @(varargin) sayIf(~opts.quiet, varargin{:});
qOf = @(P, Cd) Cd * Ao * sqrt(2 * max(P, 0) * 1e6 / rho) * 1e6;     % mL/s at P MPa

say('\n=== charOrificeFlow: %d file(s), K %.2f GPa for the corrections\n', numel(files), K / 1000);
say('Expected (%d orifices of %g um, A_o %.3g m^2):\n', cfg.nOrifice, cfg.dOrifice_um, Ao);
say('  %7s %12s %12s %12s\n', 'P MPa', 'Q Cd 0.6', 'Q Cd 0.7', 'Q Cd 0.8');
for P = [4 7 10]
    say('  %7.1f %9.2f mL/s %9.2f mL/s %9.2f mL/s   piston %.2f mm/s at Cd 0.7\n', P, ...
        qOf(P, 0.6), qOf(P, 0.7), qOf(P, 0.8), qOf(P, 0.7) * 1000 / A);
end

win = struct('file', {}, 'step', {}, 't1', {}, 't2', {}, 'P_MPa', {}, 'Q_mL_s', {}, ...
    'v_mm_s', {}, 'Cd', {}, 'dPdt', {}, 'sub', {});
grav = struct('file', {}, 'collected_mL', {}, 'predicted_mL', {}, 'ratio', {});
for f = 1:numel(files)
    d = benchReadCsv(files{f});
    name = shortName(files{f});
    inP = d.mode == 7 & d.status == 5;
    steps = unique(d.step(inP));
    for st = steps'
        rows = find(inP & d.step == st);
        if numel(rows) < 40
            continue;
        end
        t = d.t(rows); x = d.position(rows); v = d.velocity(rows); P = d.pressure(rows);
        if isfield(opts, 'tOpen') && numel(opts.tOpen) >= f && ~isnan(opts.tOpen(f))
            tOpen = opts.tOpen(f);
        else
            tOpen = detectOpening(t, v, P, d.inBand(rows), opts.vOpen);
        end
        if isnan(tOpen)
            say('  %s step %d: no valve opening found (velocity never above %.2f mm/s after the target was reached)\n', ...
                name, st, opts.vOpen);
            continue;
        end
        say('  %s step %d: valve opened at %.1f s (%.3f mm, %.2f MPa)\n', name, st, tOpen, ...
            interp1(t, x, tOpen), interp1(t, P, tOpen));
        xOpen = interp1(t, x, tOpen);
        if isfield(opts, 'fillVolume_mL')
            V0 = opts.fillVolume_mL(min(f, numel(opts.fillVolume_mL))) * 1000;
        else
            V0 = A * (cfg.posEOT - xOpen) + opts.deadVolume_mL * 1000;
        end
        % Liquid left and chamber volume along the stroke; the tube's air
        % is the first thing to leave.
        swept = A * (x - xOpen);
        after = t >= tOpen;
        dPdt = slopeOver(t, P, 2);
        steady = after & swept > cfg.tubeVolume_mL * 1000 + 500 & abs(dPdt) < opts.settleRate;
        [i1, i2] = longestRun(steady);
        if isempty(i1) || t(i2) - t(i1) < opts.minWindow
            say('    no steady window of %.0f s (drift below %.2f MPa/s)\n', opts.minWindow, opts.settleRate);
        else
            edges = t(i1):opts.subWindow:t(i2);
            if t(i2) - edges(end) < opts.minWindow
                edges(end) = t(i2);
            else
                edges(end + 1) = t(i2);
            end
            w = window(t, x, P, i1, i2, V0, xOpen, A, K, Ao, rho);
            w.file = name; w.step = st; w.sub = 0;
            win(end + 1) = orderfields(w, win); %#ok<AGROW>
            if numel(edges) > 2
                for e = 1:numel(edges) - 1
                    j1 = find(t >= edges(e), 1);
                    j2 = find(t <= edges(e + 1), 1, 'last');
                    w = window(t, x, P, j1, j2, V0, xOpen, A, K, Ao, rho);
                    w.file = name; w.step = st; w.sub = e;
                    win(end + 1) = orderfields(w, win); %#ok<AGROW>
                end
            end
        end
        % Gravimetric check over the collection window.
        if numel(opts.collected_mL) >= f && ~isnan(opts.collected_mL(f))
            if isfield(opts, 'collectWindow') && size(opts.collectWindow, 1) >= f
                tw = opts.collectWindow(f, :);
            else
                tw = [tOpen t(end)];
            end
            x1 = interp1(t, x, tw(1)); x2 = interp1(t, x, tw(2));
            P1 = interp1(t, P, tw(1)); P2 = interp1(t, P, tw(2));
            Vmid = V0 - A * ((x1 + x2) / 2 - xOpen);
            pred = (A * (x2 - x1) + Vmid * (P1 - P2) / K) / 1000;
            if tw(1) <= tOpen + 0.05
                pred = pred - cfg.tubeVolume_mL;
            end
            g.file = name; g.collected_mL = opts.collected_mL(f); g.predicted_mL = pred;
            g.ratio = g.collected_mL / pred;
            grav(end + 1) = g; %#ok<AGROW>
            say('    collected %.1f mL against %.1f mL swept (%+.1f %%) over %.1f..%.1f s\n', ...
                g.collected_mL, pred, 100 * (g.ratio - 1), tw);
        end
    end
end
r.windows = win;
r.gravimetric = grav;
if isempty(win)
    error('no steady delivery window found');
end

% --- Report ---------------------------------------------------------------------
say('\n%-22s %4s %3s %7s %7s %8s %9s %8s %6s %10s\n', 'file', 'step', 'sub', 't1 s', 't2 s', ...
    'P MPa', 'Q mL/s', 'v mm/s', 'Cd', 'dP/dt');
for k = 1:numel(win)
    w = win(k);
    say('%-22s %4d %3d %7.1f %7.1f %8.3f %9.3f %8.3f %6.3f %10.4f\n', w.file, w.step, w.sub, ...
        w.t1, w.t2, w.P_MPa, w.Q_mL_s, w.v_mm_s, w.Cd, w.dPdt);
end

whole = [win.sub] == 0;
P = [win(whole).P_MPa];
Q = [win(whole).Q_mL_s];
Cd = [win(whole).Cd];
r.Cd_mean = mean(Cd);
r.Cd_scatter = std([win.Cd]) / mean([win.Cd]);
if max(P) / min(P) >= 1.5
    p = polyfit(log(P), log(Q), 1);
    r.exponent = p(1);
    c = (sqrt(P) * Q') / (sqrt(P) * sqrt(P)');                 % Q = c sqrt(P), least squares
    r.Cd = c / (Ao * sqrt(2e6 / rho) * 1e6);
    r.sqrtResidual = std(Q - c * sqrt(P)) / mean(Q);
    say('\nQ = c P^n: n = %.3f (0.5 for an orifice). Square-root fit: Cd = %.3f, residual %.1f %% of Q\n', ...
        r.exponent, r.Cd, 100 * r.sqrtResidual);
else
    r.exponent = NaN;
    r.Cd = r.Cd_mean;
    r.sqrtResidual = NaN;
    say('\nOne pressure only: Cd = %.3f (mean); add deliveries at other pressures to check the square-root law\n', r.Cd);
end

% --- Acceptance -------------------------------------------------------------------
checks = {};
checks{end + 1} = check('Cd', r.Cd, 0.5, 0.9, 'sharp-edged orifice, Re 4000..6000, expected 0.6..0.8');
checks{end + 1} = check('scatter', r.Cd_scatter, 0, 0.1, 'spread of Cd over windows and sub-windows');
if ~isnan(r.exponent)
    checks{end + 1} = check('exponent', r.exponent, 0.4, 0.6, 'square-root law');
end
if ~isempty(grav)
    checks{end + 1} = check('collected', [grav.ratio], 0.95, 1.05, 'collected against swept volume');
end
r.checks = checks;
r.pass = all(cellfun(@(c) c.pass, checks));
printChecks(say, checks);

say('\nparams.py:\n');
say('  Cd = %.3f          # [-] %d delivery window(s), %.1f..%.1f MPa\n', r.Cd, sum(whole), min(P), max(P));
say('  Q at 10 MPa: %.2f mL/s, piston %.2f mm/s\n', qOf(10, r.Cd), qOf(10, r.Cd) * 1000 / A);
end


% ===========================================================================
function tOpen = detectOpening(t, v, P, inBand, vOpen)
% The valve opening: the pressure has been reached (in band, or within
% 10 % of its later plateau) and the piston then starts to move.
tOpen = NaN;
reached = find(inBand ~= 0, 1);
if isempty(reached)
    reached = find(P >= 0.9 * max(P), 1);
end
if isempty(reached)
    return;
end
k = reached;
while k <= numel(t) - 4
    if all(v(k:k + 4) > vOpen)
        tOpen = t(k) - 0.05;
        dip = min(P(k:min(k + 20, end))) - P(max(k - 2, 1));
        if dip > -0.1
            warning('charOrificeFlow:opening', ...
                'piston moved at %.1f s without a pressure dip (%.3f MPa); is this a valve opening?', ...
                t(k), dip);
        end
        return;
    end
    k = k + 1;
end
end


function w = window(t, x, P, i1, i2, V0, xOpen, A, K, Ao, rho)
dt = t(i2) - t(i1);
Vmid = V0 - A * ((x(i1) + x(i2)) / 2 - xOpen);                 % mm^3 of liquid left
w.t1 = t(i1);
w.t2 = t(i2);
w.P_MPa = mean(P(i1:i2));
w.dPdt = (P(i2) - P(i1)) / dt;
w.v_mm_s = (x(i2) - x(i1)) / dt;
w.Q_mL_s = (A * w.v_mm_s - Vmid / K * w.dPdt) / 1000;     % eq. (5): Q = A v - C dP/dt
w.Cd = w.Q_mL_s * 1e-6 / (Ao * sqrt(2 * max(w.P_MPa, 1e-3) * 1e6 / rho));
end


function s = slopeOver(t, y, T)
% Least-squares slope of y over a sliding window of T seconds.
n = numel(t);
s = zeros(n, 1);
for k = 1:n
    j = find(t >= t(k) - T / 2 & t <= t(k) + T / 2);
    if numel(j) >= 3
        p = polyfit(t(j) - t(k), y(j), 1);
        s(k) = p(1);
    end
end
end


function [i1, i2] = longestRun(mask)
i1 = []; i2 = [];
best = 0;
k = 1;
n = numel(mask);
while k <= n
    if mask(k)
        j = k;
        while j < n && mask(j + 1)
            j = j + 1;
        end
        if j - k + 1 > best
            best = j - k + 1;
            i1 = k; i2 = j;
        end
        k = j + 1;
    else
        k = k + 1;
    end
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
