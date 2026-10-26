function r = charBulkModulus(files, opts)
%CHARBULKMODULUS Effective bulk modulus K of the saline-filled chamber from
%   slow constant-velocity ramps logged by frictionRun (README section 3).
%
%   r = charBulkModulus(file)
%   r = charBulkModulus({file1, file2, ...}, opts)
%
%   Each ramp segment (one command of a 'ramps' step, pressure inside
%   opts.pressureRange) is fitted as x(P) = x0 + c1 (P - P0) + c2 g(P),
%   with g(P) = 1/(P0 + Patm) - 1/(P + Patm). A liquid of bulk modulus K
%   gives c1 = V / (K A); an undissolved gas volume Vg at P0 gives
%   c2 = Vg (P0 + Patm) / A, isothermal. So K follows from c1 and the
%   residual gas is bounded by c2. The linear slope dP/dx of the same data
%   is the stiffness the pressure loop sees; its K ("effective", including
%   any gas) is the value for the design model when the gas bound is small.
%
%   The liquid volume V is the chamber volume at the segment's mean
%   position, A (posEOT - x) + deadVolume, unless opts.fillVolume_mL gives
%   the measured fill (scalar, or one value per file). deadVolume_mL
%   defaults to charConfig (masterConfig.deadVolume, a placeholder) with a
%   warning: measure it (README section 2).
%
%   With files at liquid volumes that differ by more than 20 %, the
%   compliance dx/dP is also regressed on V: slope 1/(K A) gives the liquid
%   alone, intercept A/ks the series compliance of the actuator and frame
%   (ks in N/mm), which the motor encoder cannot separate from the liquid
%   in a single fill.
%
%   opts.deadVolume_mL     measured dead volume (mL)
%   opts.fillVolume_mL     measured liquid volume at zero gauge (mL)
%   opts.pressureRange     [min max] MPa gauge fitted (default [2.5 10.5])
%   opts.skip              s dropped at the start of each segment (default 1)
%   opts.minSpeed          mm/s, rows slower are dropped (default 0.03)
%   opts.chargeRatio       oxygen charge (STP volume / saline volume) of a
%                          charged run, to compare the gas bound with the
%                          model's free gas (default none)
%   opts.quiet             no printing
%
%   r.segments  one row per segment: file, step, direction, Pmin, Pmax,
%               xMean, V_mL, dPdx (MPa/mm), K_lin_GPa, K_liq_GPa, Vg_mL,
%               residual_MPa, n
%   r.K_Pa      effective bulk modulus for params.py (mean of the segments)
%   r.pass      every acceptance check passed
%   r.checks    the checks, with their values and limits

if ischar(files)
    files = {files};
end
if nargin < 2
    opts = struct();
end
cfg = charConfig();
opts = withDefault(opts, 'pressureRange', [2.5 10.5]);
opts = withDefault(opts, 'skip', 1);
opts = withDefault(opts, 'minSpeed', 0.03);
opts = withDefault(opts, 'chargeRatio', []);
opts = withDefault(opts, 'quiet', false);
if ~isfield(opts, 'deadVolume_mL')
    opts.deadVolume_mL = cfg.deadVolume_mL;
    if ~isfield(opts, 'fillVolume_mL')
        warning('charBulkModulus:deadVolume', ...
            'deadVolume_mL not given: using %.0f mL from masterConfig, a placeholder. Measure it.', ...
            cfg.deadVolume_mL);
    end
end
A = cfg.pistonArea_mm2;             % mm^2
Patm = cfg.Patm_MPa;
say = @(varargin) sayIf(~opts.quiet, varargin{:});

say('\n=== charBulkModulus: %d file(s), %.1f..%.1f MPa\n', numel(files), opts.pressureRange);
expectedTable(say, A, cfg);

seg = struct('file', {}, 'step', {}, 'dir', {}, 'Pmin', {}, 'Pmax', {}, 'xMean', {}, ...
    'V_mL', {}, 'dPdx', {}, 'K_lin_GPa', {}, 'K_liq_GPa', {}, 'Vg_mL', {}, ...
    'residual_MPa', {}, 'residual_x_mm', {}, 'n', {}, 'rate_MPa_s', {});
for f = 1:numel(files)
    d = charReadCsv(files{f});
    if isfield(opts, 'fillVolume_mL')
        fill = opts.fillVolume_mL(min(f, numel(opts.fillVolume_mL))) * 1000;    % mm^3
    else
        fill = NaN;
    end
    inVel = d.mode == 3 & d.status == 4 & d.ref ~= 0;
    keys = unique([d.step(inVel) sign(d.ref(inVel))], 'rows');
    for k = 1:size(keys, 1)
        rows = find(inVel & d.step == keys(k, 1) & sign(d.ref) == keys(k, 2));
        t0 = d.t(rows(1));
        rows = rows(d.t(rows) >= t0 + opts.skip & abs(d.velocity(rows)) >= opts.minSpeed & ...
            d.pressure(rows) >= opts.pressureRange(1) & d.pressure(rows) <= opts.pressureRange(2));
        if numel(rows) < 20
            continue;
        end
        x = d.position(rows);
        P = d.pressure(rows);
        if max(P) - min(P) < 0.5
            say('  %s step %d: only %.2f MPa of pressure change inside the range, skipped\n', ...
                shortName(files{f}), keys(k, 1), max(P) - min(P));
            continue;
        end
        s = fitSegment(x, P, A, Patm);
        s.file = shortName(files{f});
        s.step = keys(k, 1);
        if keys(k, 2) > 0
            s.dir = 'up';
        else
            s.dir = 'down';
        end
        s.Pmin = min(P);
        s.Pmax = max(P);
        s.n = numel(rows);
        s.rate_MPa_s = abs((P(end) - P(1)) / (d.t(rows(end)) - d.t(rows(1))));
        if isnan(fill)
            V = A * (cfg.posEOT - s.xMean) + opts.deadVolume_mL * 1000;
        else
            V = fill;
        end
        s.V_mL = V / 1000;
        s.K_lin_GPa = s.dPdx * V / A / 1000;            % MPa -> GPa
        s.K_liq_GPa = V / (A * s.c1) / 1000;
        s.Vg_mL = s.c2 * A / (s.P0 + Patm) / 1000;
        seg(end + 1) = orderfields(rmfield(s, {'c1', 'c2', 'P0'}), seg); %#ok<AGROW>
    end
end
r.segments = seg;
if isempty(seg)
    error('no ramp segment with more than 0.5 MPa of change inside %.1f..%.1f MPa', opts.pressureRange);
end

% --- Report -----------------------------------------------------------------
say('\n%-22s %4s %-4s %11s %8s %8s %10s %7s %7s %7s %8s\n', 'file', 'step', 'dir', 'P range MPa', ...
    'x mm', 'V mL', 'dP/dx', 'K lin', 'K liq', 'Vg mL', 'res MPa');
for k = 1:numel(seg)
    s = seg(k);
    say('%-22s %4d %-4s %5.2f-%5.2f %8.3f %8.1f %7.2f/mm %7.2f %7.2f %7.3f %8.3f\n', s.file, s.step, ...
        s.dir, s.Pmin, s.Pmax, s.xMean, s.V_mL, s.dPdx, s.K_lin_GPa, s.K_liq_GPa, s.Vg_mL, s.residual_MPa);
end

Klin = [seg.K_lin_GPa];
Kliq = [seg.K_liq_GPa];
Vg = [seg.Vg_mL];
up = strcmp({seg.dir}, 'up');
r.K_lin_GPa = mean(Klin);
r.K_liq_GPa = mean(Kliq);
r.K_Pa = r.K_lin_GPa * 1e9;
r.dPdx_MPa_mm = mean([seg.dPdx]);
r.Vg_mL = max(Vg);
if any(up) && any(~up)
    r.hysteresis = abs(mean(Klin(up)) - mean(Klin(~up))) / mean(Klin);
else
    r.hysteresis = NaN;
end
gasShare = zeros(size(seg));
for k = 1:numel(seg)
    Cliq = seg(k).V_mL * 1000 / (seg(k).K_liq_GPa * 1000);                 % mm^3/MPa
    Cgas = max(seg(k).Vg_mL, 0) * 1000 / ((seg(k).Pmin + seg(k).Pmax) / 2 + Patm);
    gasShare(k) = Cgas / (Cliq + Cgas);
end
r.gasShare = max(gasShare);

% --- Two or more liquid volumes: split liquid and series compliance ----------
r.split = [];
Vs = [seg.V_mL];
if (max(Vs) - min(Vs)) / mean(Vs) > 0.2
    C = 1 ./ [seg.dPdx];                                 % mm/MPa
    p = polyfit(Vs * 1000, C, 1);                        % C = V/(K A) + A/ks
    split.K_GPa = 1 / (p(1) * A) / 1000;
    if p(2) > 0
        split.ks_N_per_um = A / p(2) / 1000;             % MPa mm^2 / mm = N/mm -> N/um
    else
        split.ks_N_per_um = Inf;
    end
    split.residual_mm_per_MPa = std(C - polyval(p, Vs * 1000));
    r.split = split;
    say('\nTwo fills: liquid K %.2f GPa, series stiffness %.0f N/um (compliance residual %.4f mm/MPa)\n', ...
        split.K_GPa, split.ks_N_per_um, split.residual_mm_per_MPa);
end

% --- Gas term check against the model's free gas ------------------------------
if ~isempty(opts.chargeRatio)
    kH = 1.22e-5; R = 8.314; T = 293;                   % params.py values
    Vl = mean(Vs) * 1e-6;                                % m^3
    n = opts.chargeRatio * Vl * 101325 / (R * 273.15);   % mol
    say('\nCharge %.2f:1 in %.0f mL. Model free gas (isothermal, dissolved to saturation):\n', ...
        opts.chargeRatio, mean(Vs));
    for P = [3 5 7 10]
        Pabs = (P + Patm) * 1e6;
        nfree = max(n - kH * Pabs * Vl, 0);
        say('  %5.1f MPa: %.3f mL free gas\n', P, nfree * R * T / Pabs * 1e6);
    end
    say('  fitted gas bound: %.3f mL (largest segment)\n', r.Vg_mL);
end

% --- Acceptance ---------------------------------------------------------------
checks = {};
checks{end + 1} = check('segments', numel(seg), 2, Inf, 'at least two ramps');
checks{end + 1} = check('K_lin_GPa', [min(Klin) max(Klin)], 0.5, 2.5, 'every segment inside 0.5..2.5 GPa');
checks{end + 1} = check('residual_MPa', max([seg.residual_MPa]), 0, 0.1, 'fit residual (as pressure)');
checks{end + 1} = check('gasShare', r.gasShare, 0, 0.05, 'gas compliance share of the total');
checks{end + 1} = check('scatter', std(Klin) / mean(Klin), 0, 0.1, 'segment-to-segment spread of K');
if ~isnan(r.hysteresis)
    checks{end + 1} = check('hysteresis', r.hysteresis, 0, 0.15, 'up against down');
end
r.checks = checks;
r.pass = all(cellfun(@(c) c.pass, checks));
printChecks(say, checks);

say('\nparams.py:\n');
say('  K = %.3g          # [Pa] effective bulk modulus, %d ramps, %.1f..%.1f MPa\n', ...
    r.K_Pa, numel(seg), min([seg.Pmin]), max([seg.Pmax]));
say('  Vl0 = %.3g       # [m^3] liquid volume of these runs\n', mean(Vs) * 1e-6);
say('  stiffness at %.0f mL: %.1f MPa/mm (A K / V)\n', mean(Vs), r.dPdx_MPa_mm);
end


% ===========================================================================
function s = fitSegment(x, P, A, Patm)
% Regress x on (P - P0) and the isothermal gas term; also the plain slope.
P0 = mean(P);
g = 1 ./ (P0 + Patm) - 1 ./ (P + Patm);
X = [ones(size(P)) (P - P0) g];
c = X \ x;
res = x - X * c;
s.xMean = mean(x);
s.P0 = P0;
s.c1 = c(2);                                 % mm/MPa, liquid
s.c2 = c(3);                                 % mm MPa, gas
pl = polyfit(x, P, 1);                       % P = dPdx x + b
s.dPdx = pl(1);
s.residual_MPa = std(P - polyval(pl, x));
if s.c1 <= 0
    s.c1 = NaN;
end
s.residual_x_mm = std(res);
end


function expectedTable(say, A, cfg)
V = 300e3;                                   % mm^3
say('Expected at %.0f mL (A %.0f mm^2), ramp at 0.1 mm/s, 3 -> 10 MPa:\n', V / 1000, A);
say('  %6s %12s %12s %12s\n', 'K GPa', 'dP/dx MPa/mm', 'dP/dt MPa/s', 'travel mm');
for K = [0.5 1.0 1.5 2.0 2.5]
    k = K * 1000 * A / V;
    say('  %6.1f %12.1f %12.2f %12.2f\n', K, k, k * 0.1, 7 / k);
end
say('  Water alone is 2.2 GPa; seals, hoses and the actuator in series lower the effective value.\n');
say('  A gas bubble of 0.1 mL at 5 MPa adds %.0f %% to the compliance of 300 mL at 1 GPa.\n', ...
    100 * (0.1e3 / (5 + cfg.Patm_MPa)) / (V / 1000));
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
