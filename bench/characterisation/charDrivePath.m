function r = charDrivePath(files, opts)
%CHARDRIVEPATH The drive path between the motor and the piston from the
%   motor's position, velocity and torque against the chamber pressure
%   (README section 10, "Test 5: drive path"): the reflected inertia, the
%   friction reversal under load, the stiffness of the path, the backlash
%   and the motor-side friction.
%
%   r = charDrivePath(files)
%   r = charDrivePath(files, opts)
%
%   files  names, patterns or a cell array of either (benchFiles), any mix of
%     step captures   open chamber, the command changes by a step (the
%                     frictionRun 'steps' set, charRun velstep): the
%                     inertia
%     K logs          50 ms run logs of the pressure ramps of test 1: the
%                     friction reversal under load and the up/down offset
%                     of the motor position, which bound the stiffness
%     reversal captures
%                     open chamber, the command reverses without a stop
%                     (README section 10, "Reversal, open chamber"): the
%                     backlash and the motor-side friction
%     dither captures closed liquid-filled chamber, small triangles about
%                     a pressure (README section 10, "Dither"): backlash
%                     and windup at the pressure
%   A file is classified by its columns and its first command change: a
%   log without a vCmd column is a K log; a capture whose first change
%   reverses the sign of the command is a reversal capture (median
%   pressure below opts.openMax_MPa) or a dither capture (above); any
%   other change is a step, kept for the inertia when the chamber is open.
%
%   Every force is the torque (percent of rated) times the lossless
%   constant r.newtonPerPct (bench/friction/frictionBenchConfig: the
%   piston force at 100 % torque through the 70:1 gearbox and the 5.08 mm
%   lead, with no loss). Results are given in torque units as well, so
%   that an efficiency can be applied later: force = efficiency x
%   newtonPerPct x torque for the motor pushing the piston, divided by it
%   for the motor being pushed.
%
%   Inertia (steps). The torque excess over the steady torque after the
%   step, regressed through the origin on the motor acceleration (the
%   gradient of the velocity feedback, one row per scan) over the
%   opts.inertia_s after the command, on the rows where the acceleration
%   exceeds opts.minAccel. J is in percent of rated torque per mm/s^2; the
%   reflected mass at the piston is J x newtonPerPct x 1e3 kg. The rows
%   are one scan apart whatever the t column says (opts.rowPeriod_s).
%
%   Friction reversal and the stiffness bound (K logs). The segments of
%   constant velocity sign of a ramp log; for each consecutive pair of
%   opposite direction, the torque and the motor position of the up ramp
%   minus those of the down ramp at the same pressure, interpolated on
%   opts.grid_MPa; the median over the pairs. The torque difference is
%   twice the friction (Coulomb plus load-dependent, 2 Fc + 2 mu P A: the
%   fit of dT against P gives both). The friction on the piston's side
%   of the drive path reverses with it, so a path of stiffness k_s
%   deflects by dF / k_s between the two ramps, dF the force that passes
%   the path, and the position offset bounds it: k_s > dF / max(|dx|,
%   opts.floor_um). The floor is the offset the log resolves (the
%   pair-to-pair scatter, and the pressure sensor's lag of about 36 ms at
%   the ramp speed, which adds to the offset in the same direction and so
%   keeps the bound valid). r.stiffness.bound_* takes the whole torque
%   reversal as dF, which holds when all the friction is on the piston's
%   side; the motor's own friction f_m reverses without reaching the path,
%   so with it known (from the reversal captures, or
%   opts.motorFriction_pct) r.stiffness.boundPath_* takes dF as dT - 2 f_m.
%
%   Reversal and dither captures. The first command change of a capture
%   is a reversal of the commanded velocity. The torque is corrected for
%   the inertia (T - J a, J from the steps or opts.inertia_pct), oriented
%   along the old direction s, and normalised between its level before
%   the reversal (L1) and its level after (L2, at the reversal's
%   pressure when the chamber is closed: the late torque is regressed on
%   the pressure), phi = (L1 - s T) / (L1 - L2), 0 at the old level and 1
%   at the new. The motor turns at the extremum of s x; xi is the motor
%   travel since, over the rows within opts.maxTravel_um and
%   opts.transition_s of the turn. phi(xi) is fitted by the transition of
%   a drive path with backlash: from phi(0) a ramp (the windup of the
%   path) to a flat level, which holds for the backlash, a second ramp
%   (the windup on the other side) up to 1, and 1 after, by least squares
%   over the three breakpoints. The null model has no flat segment: one
%   ramp from phi(0) to 1. A flat segment is taken for backlash when it
%   is at least opts.minBacklash_um long, the null model fits at least
%   opts.hingeRatio times worse and it holds three rows. Its level is the
%   torque of the motor with no force through the path: the motor-side
%   friction f_m = -s T there. The end of the last ramp is the travel of
%   the whole transition, W; the part outside the flat segment is the
%   windup, over which the torque changes by (1 - phi(0)) (L1 - L2); their
%   ratio is the stiffness, or a bound at opts.res_um when the windup is
%   below the travel resolution or the torque hardly changes over it. The pressure change over the transition
%   (one sensor lag later) says whether the chamber stood still.
%
%   opts.rowPeriod_s     s between capture rows (default charConfig scan)
%   opts.logShuntOhms    G_cfgPressureShuntOhms the logs were recorded with
%   opts.minStep         mm/s, smallest step used for the inertia (0.25)
%   opts.inertia_s       s of the regression window after the command (0.16)
%   opts.settle_s        s window [from to] after the command for the
%                        steady torque of the inertia fit ([0.1 0.24])
%   opts.minAccel        mm/s^2, rows below this acceleration are left out
%                        of the inertia regression (5)
%   opts.vSign           mm/s, the velocity below which a log row is at
%                        rest (0.05)
%   opts.minRampRows     rows a ramp needs to count as a segment (20)
%   opts.grid_MPa        pressures of the reversal comparison (4:0.5:9)
%   opts.floor_um        the smallest up/down offset the logs resolve (5)
%   opts.motorFriction_pct  motor-side friction in torque percent for the
%                        path-force bound (default: from the reversal
%                        captures when there are any, else not applied)
%   opts.inertia_pct     J for the reversal and dither captures (default:
%                        the step fit of the same call, else
%                        charConfig's driveInertia_pct_per_mm_s2)
%   opts.openMax_MPa     median pressure below which a capture is open (0.3)
%   opts.minReversal_pct smallest torque reversal analysed (1.5)
%   opts.post_s          s window [from to] after the command for the torque
%                        level after a reversal ([0.3 1.8])
%   opts.transition_s    s after the turn that the transition is fitted over (0.3)
%   opts.maxTravel_um    um of motor travel after the turn that it is fitted
%                        over (80)
%   opts.beta            fraction of the reversal that counts as arrived;
%                        the fit runs to where phi last sits below 1 - beta,
%                        or three times the noise if larger (0.15)
%   opts.res_um          motor travel resolution of a transition: the windup
%                        below it is not measured (3)
%   opts.minWindupShare  share of the torque reversal that must change over
%                        the windup travel for a stiffness to be read from
%                        it (0.1); a torque that changes at the turn alone
%                        leaves only a bound
%   opts.minBacklash_um  shortest flat segment taken for backlash (4)
%   opts.hingeRatio      line over hinge squared error needed for backlash (2)
%   opts.pressureBin_MPa width of the pressure groups of the dither (1)
%   opts.quiet           no printing
%
%   r.newtonPerPct       N per percent of rated torque, lossless
%   r.inertia            .steps (one per step: file, dv, J, steady change,
%                        peak acceleration), .sizes (per |dv|: n, J, q25,
%                        q75, mass_kg), .J_pct_per_mm_s2, .mass_kg, .n
%   r.friction           .logs (per K log: pairs, dT_pct and dX_um on the
%                        grid, median dT and |dx|, bound), .P_MPa, .dT_pct,
%                        .dF_kN, .dX_um (median over logs), .a_pct and
%                        .b_pct_per_MPa (dT = a + b P), .Fc_N (half of
%                        newtonPerPct a, which includes the viscous term
%                        at the ramp speed), .mu
%   r.stiffness          .bound_N_per_um, .bound_pct_per_um (the whole
%                        reversal through the path; the weakest log),
%                        .boundPath_N_per_um (with the motor-side
%                        friction out), .floor_um, .resolved (some log's
%                        offset is above the floor)
%   r.reversal           .rec (one per capture), .groups (per commanded
%                        speed: n, plateau count, dT, travel, backlash,
%                        f_m, windup, stiffness), .backlash_um (NaN
%                        without backlash), .backlashBound_um,
%                        .motorFriction_pct, .motorFriction_kN
%   r.dither             .rec, .groups (per reversal pressure and
%                        direction: n, dT, backlash, windup, stiffness)
%   r.skipped            files not used, with the reason
%   r.pass, r.checks

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'), ...
    fullfile(fileparts(mfilename('fullpath')), '..', 'friction'));
files = benchFiles(files);
if nargin < 2
    opts = struct();
end
cfg = charConfig();
fcfg = frictionBenchConfig();
nPP = fcfg.newtonPerPct;                                    % [N per % rated torque]
areaN = fcfg.pistonAreaM2 * 1e6;                            % [N per MPa] (m^2 -> mm^2 x 1e6 Pa/MPa x 1e-6 m^2/mm^2)
opts = withDefault(opts, 'rowPeriod_s', cfg.scanTime);
opts = withDefault(opts, 'logShuntOhms', []);
opts = withDefault(opts, 'minStep', 0.25);
opts = withDefault(opts, 'inertia_s', 0.16);
opts = withDefault(opts, 'settle_s', [0.1 0.24]);
opts = withDefault(opts, 'minAccel', 5);
opts = withDefault(opts, 'vSign', 0.05);
opts = withDefault(opts, 'minRampRows', 20);
opts = withDefault(opts, 'grid_MPa', 4:0.5:9);
opts = withDefault(opts, 'floor_um', 5);
opts = withDefault(opts, 'motorFriction_pct', []);
opts = withDefault(opts, 'inertia_pct', []);
opts = withDefault(opts, 'openMax_MPa', 0.3);
opts = withDefault(opts, 'minReversal_pct', 1.5);
opts = withDefault(opts, 'post_s', [0.3 1.8]);
opts = withDefault(opts, 'transition_s', 0.3);
opts = withDefault(opts, 'maxTravel_um', 80);
opts = withDefault(opts, 'beta', 0.15);
opts = withDefault(opts, 'res_um', 3);
opts = withDefault(opts, 'minWindupShare', 0.1);
opts = withDefault(opts, 'minBacklash_um', 4);
opts = withDefault(opts, 'hingeRatio', 2);
opts = withDefault(opts, 'pressureBin_MPa', 1);
opts = withDefault(opts, 'quiet', false);
say = @(varargin) sayIf(~opts.quiet, varargin{:});
Ts = opts.rowPeriod_s;                                      % [s]
sensorLag = cfg.filterTau + ((cfg.medianSize - 1) / 2 + cfg.pipelineScans) * Ts;   % [s] pressure chain, for a ramp

say('\n=== charDrivePath: %d file(s); %.1f N per %% of rated torque (lossless), scan %.1f ms\n', ...
    numel(files), nPP, 1000 * Ts);

% --- Classify the files ------------------------------------------------------------
steps = {}; klogs = {}; revs = {}; dith = {};
r.skipped = struct('file', {}, 'why', {});
for f = 1:numel(files)
    d = benchReadCsv(files{f});
    if isfield(d, 'pressure')
        d.pressure = benchPressureRescale(d.pressure, opts.logShuntOhms);
    else
        d.pressure = zeros(size(d.t));
    end
    if ~isfield(d, 'vCmd')
        klogs{end + 1} = d;                                 %#ok<AGROW>
        continue;
    end
    i = find(diff(d.vCmd) ~= 0, 1);
    closed = median(d.pressure) > opts.openMax_MPa;
    if isempty(i)
        r.skipped(end + 1) = struct('file', files{f}, 'why', 'the command never changes');
    elseif d.vCmd(i) * d.vCmd(i + 1) < 0
        if closed
            dith{end + 1} = d;                              %#ok<AGROW>
        else
            revs{end + 1} = d;                              %#ok<AGROW>
        end
    elseif ~closed
        steps{end + 1} = d;                                 %#ok<AGROW>
    else
        r.skipped(end + 1) = struct('file', files{f}, 'why', 'a step with the chamber closed');
    end
end
say('%d step captures, %d K logs, %d reversal captures, %d dither captures', numel(steps), ...
    numel(klogs), numel(revs), numel(dith));
if ~isempty(r.skipped)
    say(', %d skipped', numel(r.skipped));
end
say('\n');
if isempty(steps) && isempty(klogs) && isempty(revs) && isempty(dith)
    error('charDrivePath: no usable file (%d skipped)', numel(r.skipped));
end

r.newtonPerPct = nPP;
r.inertia = [];
r.friction = [];
r.stiffness = [];
r.reversal = [];
r.dither = [];
checks = {};

% --- Inertia --------------------------------------------------------------------------
Jused = opts.inertia_pct;
if ~isempty(steps)
    r.inertia = inertiaFit(steps, opts, Ts, nPP);
    if isempty(r.inertia.steps)
        say('inertia: no step of at least %.2f mm/s with enough acceleration\n', opts.minStep);
        r.inertia = [];
    else
        printInertia(say, r.inertia);
        big = r.inertia.sizes([r.inertia.sizes.size] >= 1 & [r.inertia.sizes.n] >= 5);
        if ~isempty(big)
            spread = max(([big.q75] - [big.q25]) ./ [big.J]);
            checks{end + 1} = check('inertia spread', spread, 0, 0.35, ...
                'interquartile range over the median of J at the steps of 1 mm/s and more');
        end
        checks{end + 1} = check('inertia positive', r.inertia.J_pct_per_mm_s2, 1e-6, Inf, ...
            'the torque excess goes with the acceleration');
        if isempty(Jused)
            Jused = r.inertia.J_pct_per_mm_s2;
        end
    end
end
if isempty(Jused)
    Jused = cfg.driveInertia_pct_per_mm_s2;
end
r.inertiaUsed_pct_per_mm_s2 = Jused;

% --- Reversal and dither captures -----------------------------------------------------
if ~isempty(revs)
    r.reversal = reversalFit(revs, Jused, nPP, opts, Ts, sensorLag, false);
    printReversal(say, r.reversal, nPP);
    checks{end + 1} = check('reversals resolved', mean([r.reversal.rec.ok]), 0.7, 1, ...
        'share of the captures whose reversal the noise lets through');
end
if ~isempty(dith)
    r.dither = reversalFit(dith, Jused, nPP, opts, Ts, sensorLag, true);
    printDither(say, r.dither, nPP);
    checks{end + 1} = check('dither resolved', mean([r.dither.rec.ok]), 0.7, 1, ...
        'share of the captures whose reversal the noise lets through');
end

% --- K logs ---------------------------------------------------------------------------
if ~isempty(klogs)
    fm = opts.motorFriction_pct;
    if isempty(fm) && ~isempty(r.reversal) && ~isnan(r.reversal.motorFriction_pct)
        fm = r.reversal.motorFriction_pct;
    end
    [r.friction, r.stiffness] = kLogFit(klogs, opts, nPP, areaN, fm);
    printFriction(say, r.friction, r.stiffness, nPP);
    if ~isempty(r.friction.logs)
        dts = [r.friction.logs.medDT_pct];
        checks{end + 1} = check('reversal repeats', (max(dts) - min(dts)) / mean(dts), 0, 0.3, ...
            'spread of the logs'' median torque reversal over their mean');
        checks{end + 1} = check('reversal rises with load', r.friction.b_pct_per_MPa, 0, Inf, ...
            'the torque reversal grows with the pressure (load-dependent friction)');
        checks{end + 1} = check('offset below floor', max([r.friction.logs.medDX_um]), 0, opts.floor_um, ...
            'the up/down motor offset is within the resolution, so the path is bounded and not measured');
    end
end

r.checks = checks;
r.pass = all(cellfun(@(c) c.pass, checks));
printChecks(say, checks);
if ~isempty(r.inertia)
    say('\nmodels/params.py:\n  mr = %.3g        # [kg] reflected inertia at the piston, lossless torque constant (%.3f %% per mm/s^2)\n', ...
        r.inertia.mass_kg, r.inertia.J_pct_per_mm_s2);
end
end


% ===========================================================================
function in = inertiaFit(caps, opts, Ts, nPP)
% Torque excess over the post-step steady torque against the motor
% acceleration, per step, then per step size.
nW = round(opts.inertia_s / Ts);                            % [s -> rows]
p1 = round(opts.settle_s(1) / Ts);
p2 = round(opts.settle_s(2) / Ts) - 1;
steps = struct('file', {}, 'dv', {}, 'J', {}, 'dTsteady', {}, 'aMax', {});
for k = 1:numel(caps)
    d = caps{k};
    n = numel(d.t);
    i = find(diff(d.vCmd) ~= 0, 1);
    i0 = i + 1;                                             % first row of the new command
    dv = d.vCmd(i0) - d.vCmd(i0 - 1);                       % [mm/s]
    if abs(dv) < opts.minStep
        continue;
    end
    seg = i0:min(i0 + nW - 1, n - 1);
    a = gradient(d.velocity, Ts);                           % [mm/s^2]
    a = a(seg);
    post = min(i0 + p1, n):min(i0 + p2, n);
    pre = max(i0 - round(0.1 / Ts), 1):(i0 - 1);
    tPost = med(d.torque(post));                            % [% rated]
    tPre = med(d.torque(pre));
    m = abs(a) > opts.minAccel;
    if sum(m) < 4
        continue;
    end
    ex = d.torque(seg) - tPost;                             % [%]
    J = sum(ex(m) .* a(m)) / sum(a(m) .^ 2);                % [% per mm/s^2]
    steps(end + 1) = struct('file', d.file, 'dv', dv, 'J', J, 'dTsteady', tPost - tPre, ...
        'aMax', max(abs(a)));                               %#ok<AGROW>
end
in.steps = steps;
in.n = numel(steps);
in.sizes = struct('size', {}, 'n', {}, 'J', {}, 'q25', {}, 'q75', {}, 'mass_kg', {}, ...
    'dTsteady', {}, 'aMax', {});
if isempty(steps)
    in.J_pct_per_mm_s2 = NaN;
    in.mass_kg = NaN;
    return;
end
sz = round(abs([steps.dv]) * 10) / 10;
for s = unique(sz)
    m = sz == s;
    J = [steps(m).J];
    in.sizes(end + 1) = struct('size', s, 'n', sum(m), 'J', med(J), 'q25', quant(J, 0.25), ...
        'q75', quant(J, 0.75), 'mass_kg', med(J) * nPP * 1e3, 'dTsteady', med([steps(m).dTsteady]), ...
        'aMax', med([steps(m).aMax]));                      %#ok<AGROW>
end
in.J_pct_per_mm_s2 = med([steps.J]);
in.mass_kg = in.J_pct_per_mm_s2 * nPP * 1e3;                % [% per mm/s^2 x N per % x 1e3 mm/m -> kg]
end


function [fr, st] = kLogFit(logs, opts, nPP, areaN, fm)
% Friction reversal and up/down offset against pressure from ramp logs.
grid = opts.grid_MPa(:)';
fr.P_MPa = grid;
fr.logs = struct('file', {}, 'nPairs', {}, 'dT_pct', {}, 'dX_um', {}, 'medDT_pct', {}, ...
    'medDX_um', {}, 'bound_pct_per_um', {}, 'bound_N_per_um', {});
for k = 1:numel(logs)
    d = logs{k};
    sgn = zeros(size(d.velocity));
    sgn(d.velocity > opts.vSign) = 1;
    sgn(d.velocity < -opts.vSign) = -1;
    segs = rampSegments(sgn, opts.minRampRows);
    dT = zeros(0, numel(grid));
    dX = zeros(0, numel(grid));
    for j = 1:size(segs, 1) - 1
        if segs(j, 1) == segs(j + 1, 1)
            continue;
        end
        if segs(j, 1) > 0
            up = segs(j, 2:3); dn = segs(j + 1, 2:3);
        else
            up = segs(j + 1, 2:3); dn = segs(j, 2:3);
        end
        dT(end + 1, :) = onPressure(d, up, d.torque, grid) - onPressure(d, dn, d.torque, grid);       %#ok<AGROW> [%]
        dX(end + 1, :) = onPressure(d, up, d.position, grid) - onPressure(d, dn, d.position, grid);   %#ok<AGROW> [mm]
    end
    if isempty(dT)
        continue;
    end
    row.file = d.file;
    row.nPairs = size(dT, 1);
    row.dT_pct = colMedian(dT);                             % [%]
    row.dX_um = 1000 * colMedian(dX);                       % [mm -> um]
    row.medDT_pct = med(row.dT_pct);
    row.medDX_um = med(abs(row.dX_um));
    row.bound_pct_per_um = row.medDT_pct / max(row.medDX_um, opts.floor_um);   % [% per um]
    row.bound_N_per_um = row.bound_pct_per_um * nPP;        % [N per um]
    fr.logs(end + 1) = row;                                 %#ok<AGROW>
end
fr.dT_pct = nan(1, numel(grid));
fr.dX_um = nan(1, numel(grid));
fr.dF_kN = nan(1, numel(grid));
fr.a_pct = NaN; fr.b_pct_per_MPa = NaN; fr.Fc_N = NaN; fr.mu = NaN;
st = struct('bound_N_per_um', NaN, 'bound_pct_per_um', NaN, 'boundPath_N_per_um', NaN, ...
    'floor_um', opts.floor_um, 'resolved', false);
if isempty(fr.logs)
    return;
end
fr.dT_pct = colMedian(vertcat(fr.logs.dT_pct));
fr.dX_um = colMedian(vertcat(fr.logs.dX_um));
fr.dF_kN = fr.dT_pct * nPP / 1e3;                           % [% x N per % -> kN]
ok = ~isnan(fr.dT_pct);
if sum(ok) >= 2
    c = [ones(sum(ok), 1) grid(ok)'] \ fr.dT_pct(ok)';
    fr.a_pct = c(1);                                        % [%] at zero pressure
    fr.b_pct_per_MPa = c(2);                                % [% per MPa]
    fr.Fc_N = 0.5 * c(1) * nPP;                             % [N] dT = 2 (Fc + mu P A) / newtonPerPct
    fr.mu = 0.5 * c(2) * nPP / areaN;
end
[st.bound_pct_per_um, w] = min([fr.logs.bound_pct_per_um]);
st.bound_N_per_um = fr.logs(w).bound_N_per_um;
st.resolved = any([fr.logs.medDX_um] > opts.floor_um);
if ~isempty(fm)
    dTpath = max([fr.logs(w).medDT_pct] - 2 * fm, 0);       % [%] what reaches the path
    st.boundPath_N_per_um = dTpath / max(fr.logs(w).medDX_um, opts.floor_um) * nPP;
end
end


function segs = rampSegments(sgn, minRows)
% [sign first last] of every run of constant nonzero sign longer than
% minRows rows.
segs = zeros(0, 3);
n = numel(sgn);
s = 1;
for i = 2:n + 1
    if i == n + 1 || sgn(i) ~= sgn(s)
        if sgn(s) ~= 0 && i - s > minRows
            segs(end + 1, :) = [sgn(s) s i - 1];            %#ok<AGROW>
        end
        s = i;
    end
end
end


function y = onPressure(d, seg, col, grid)
% col along a ramp, interpolated at the grid pressures (NaN outside the
% ramp's range).
idx = seg(1):seg(2);
[p, o] = sort(d.pressure(idx));
c = col(idx);
y = interpSorted(p, c(o), grid);
end


function y = interpSorted(xp, yp, g)
% Linear interpolation on increasing xp, NaN outside.
y = nan(size(g));
n = numel(xp);
for k = 1:numel(g)
    if g(k) < xp(1) || g(k) > xp(n)
        continue;
    end
    j = find(xp <= g(k), 1, 'last');
    if j == n
        y(k) = yp(n);
    else
        y(k) = yp(j) + (yp(j + 1) - yp(j)) * (g(k) - xp(j)) / (xp(j + 1) - xp(j));
    end
end
end


function m = colMedian(x)
% Median of every column ignoring NaN.
m = nan(1, size(x, 2));
for k = 1:size(x, 2)
    m(k) = med(x(~isnan(x(:, k)), k));
end
end


% ===========================================================================
function out = reversalFit(caps, J, nPP, opts, Ts, sensorLag, loaded)
% The first command change of each capture analysed as a reversal.
rec = struct('file', {}, 'ok', {}, 'why', {}, 's', {}, 'v1', {}, 'v2', {}, 'P_MPa', {}, ...
    'dT_pct', {}, 'W_um', {}, 'plateau', {}, 'b_um', {}, 'windup_um', {}, 'dTw_pct', {}, ...
    'ks_N_per_um', {}, 'ksBound_N_per_um', {}, 'fm_pct', {}, 'dP_MPa', {}, 'phi', {}, 'xi_um', {});
for k = 1:numel(caps)
    rec(end + 1) = reversalOf(caps{k}, J, nPP, opts, Ts, sensorLag, loaded);   %#ok<AGROW>
end
out.rec = rec;
out.loaded = loaded;
out.groups = struct('key', {}, 'P_MPa', {}, 'dir', {}, 'speed', {}, 'n', {}, 'nPlateau', {}, ...
    'dT_pct', {}, 'W_um', {}, 'b_um', {}, 'fm_pct', {}, 'windup_um', {}, 'ks_N_per_um', {}, ...
    'ksBound_N_per_um', {}, 'dP_MPa', {});
good = rec([rec.ok]);
out.backlash_um = NaN;
out.backlashBound_um = NaN;
out.motorFriction_pct = NaN;
out.motorFriction_kN = NaN;
if isempty(good)
    return;
end
if loaded
    key = round([good.P_MPa] / opts.pressureBin_MPa) * opts.pressureBin_MPa + 1e-3 * [good.s];
else
    key = round(0.5 * ([good.v1] + [good.v2]) * 10) / 10;
end
for q = unique(key)
    g = good(key == q);
    o.key = q;
    o.P_MPa = med([g.P_MPa]);
    o.dir = g(1).s;
    o.speed = med(0.5 * ([g.v1] + [g.v2]));
    o.n = numel(g);
    o.nPlateau = sum([g.plateau]);
    o.dT_pct = med([g.dT_pct]);
    o.W_um = med([g.W_um]);
    pl = g([g.plateau]);
    if o.nPlateau > o.n / 2
        o.b_um = med([pl.b_um]);
        o.fm_pct = med([pl.fm_pct]);
    else
        o.b_um = 0;
        o.fm_pct = NaN;
    end
    o.windup_um = med([g.windup_um]);
    if sum(~isnan([g.ks_N_per_um])) >= ceil(o.n / 2)
        o.ks_N_per_um = med([g.ks_N_per_um]);
        o.ksBound_N_per_um = NaN;
    else
        o.ks_N_per_um = NaN;
        o.ksBound_N_per_um = med([g.ksBound_N_per_um]);
    end
    o.dP_MPa = med([g.dP_MPa]);
    out.groups(end + 1) = o;                                %#ok<AGROW>
end
allPl = good([good.plateau]);
if numel(allPl) >= 2 && numel(allPl) > 0.5 * numel(good)
    out.backlash_um = med([allPl.b_um]);
    out.motorFriction_pct = med([allPl.fm_pct]);
    out.motorFriction_kN = out.motorFriction_pct * nPP / 1e3;   % [% x N per % -> kN]
else
    out.backlashBound_um = max(opts.res_um, med([good.W_um]));
end
end


function rec = reversalOf(d, J, nPP, opts, Ts, sensorLag, loaded)
% One capture: levels, turn, transition and its fit.
[~, name] = fileparts(d.file);
rec = struct('file', name, 'ok', false, 'why', '', 's', 0, 'v1', 0, 'v2', 0, 'P_MPa', NaN, ...
    'dT_pct', NaN, 'W_um', NaN, 'plateau', false, 'b_um', NaN, 'windup_um', NaN, 'dTw_pct', NaN, ...
    'ks_N_per_um', NaN, 'ksBound_N_per_um', NaN, 'fm_pct', NaN, 'dP_MPa', NaN, 'phi', [], 'xi_um', []);
n = numel(d.t);
iR = find(diff(d.vCmd) ~= 0, 1) + 1;                        % first row of the reversed command
s = sign(d.vCmd(iR - 1));
rec.s = s;
rec.v1 = abs(d.vCmd(iR - 1));
rec.v2 = abs(d.vCmd(iR));
nxt = find(diff(d.vCmd(iR:end)) ~= 0, 1);
if isempty(nxt)
    iNext = n + 1;
else
    iNext = iR + nxt;
end
% Inertia-corrected torque: T - J a, with a the slope over five rows
v = d.velocity;
a = zeros(n, 1);
k = (3:n - 2)';
a(k) = (-2 * v(k - 2) - v(k - 1) + v(k + 1) + 2 * v(k + 2)) / (10 * Ts);   % [mm/s^2]
Tn = s * (d.torque - J * a);                                % [% rated], along the old direction
P = d.pressure;
% Levels before and after
pre = 3:iR - 1;
w1 = iR + round(opts.post_s(1) / Ts);
w2 = min([n, iNext - round(0.05 / Ts), iR + round(opts.post_s(2) / Ts)]);
if numel(pre) < 6 || w2 - w1 < 25
    rec.why = 'the capture is too short around the reversal';
    return;
end
L1 = med(Tn(pre));
win = max(1, iR - 10):min(n, iR + round(0.2 / Ts));
if s > 0
    PR = max(P(win));
else
    PR = min(P(win));
end
rec.P_MPa = PR;
post = w1:w2;
if loaded && max(P(post)) - min(P(post)) >= 0.3
    c = [ones(numel(post), 1) P(post)] \ Tn(post);
    L2 = c(1) + c(2) * PR;                                  % [%] the late torque at the reversal's pressure
    res = Tn(post) - [ones(numel(post), 1) P(post)] * c;
else
    L2 = med(Tn(post));
    res = Tn(post) - L2;
end
sigma = std(res);                                           % [%]
dT = L1 - L2;                                               % [%]
rec.dT_pct = dT;
if dT < opts.minReversal_pct || dT < 5 * sigma
    rec.why = sprintf('reversal of %.1f %% against noise of %.2f %%', dT, sigma);
    return;
end
% Turn of the motor
tw = iR:min(n, iR + round(0.4 / Ts));
[~, kk] = max(s * d.position(tw));
if kk == numel(tw)
    rec.why = 'the motor does not turn';
    return;
end
iz = tw(kk);
rows = iz:min(n, iz + round(opts.transition_s / Ts));
xi = cummax(max(-s * (d.position(rows) - d.position(iz)) * 1000, 0));   % [mm -> um]
phi = (L1 - Tn(rows)) / dT;
% The fit takes the rows to where phi last sits below 1 - beta (three rows
% of the median-filtered phi beyond it), within the travel limit: the
% ripple of the torque after the transition is not part of it.
beta = max(opts.beta, 3 * sigma / dT);
below = find(med3(phi) < 1 - beta & xi <= opts.maxTravel_um, 1, 'last');
if isempty(below)
    below = 1;
end
last = min([max(below + 3, 8), numel(rows)]);
rows = rows(1:last);
xi = xi(1:last);
phi = phi(1:last);
if numel(rows) < 8
    rec.why = 'too few rows within the travel of the fit';
    return;
end
f = transitionFit(xi, phi, opts);
rec.W_um = f.W_um;
% Pressure change over the transition, one sensor lag later
lag = round(sensorLag / Ts);
ie = find(xi >= f.W_um, 1);
if isempty(ie)
    ie = numel(xi);
end
rec.dP_MPa = s * (P(min(n, rows(ie) + lag)) - P(min(n, iz + lag)));
rec.plateau = f.plateau;
rec.b_um = f.b_um;
rec.windup_um = f.windup_um;
rec.dTw_pct = (1 - f.c0) * dT;
if f.plateau
    rec.fm_pct = -(L1 - f.phiPlateau * dT);                 % [%] torque with no force through the path, along s
end
if rec.windup_um >= opts.res_um && 1 - f.c0 >= opts.minWindupShare
    rec.ks_N_per_um = rec.dTw_pct * nPP / rec.windup_um;    % [% x N per % / um]
end
rec.ksBound_N_per_um = rec.dTw_pct * nPP / max(rec.windup_um, opts.res_um);   % the stiffness, or its bound where the windup is within the resolution
rec.phi = phi(:)';
rec.xi_um = xi(:)';
rec.ok = true;
end


function f = transitionFit(xi, phi, opts)
% phi against the motor travel xi [um]: the path's transition, a ramp from
% phi(0) to a flat level p, the flat segment for the backlash, a ramp to 1
% and 1 after (breakpoints x1 <= x2 <= x3), by least squares; and the
% same without the flat segment (one ramp, x1 = x2 = x3). phi(0) and p are
% linear in the fit, the breakpoints are searched on a coarse grid and
% then refined around the best.
xi = xi(:);
phi = phi(:);
cand = [0.25 0.5 1 1.5 2 3 4 5 6 8 10 12 15 18 22 26 30 35 40 50 60 80];
cand = cand(cand <= max(opts.maxTravel_um, 2));
% Without a flat segment: one ramp to 1 at xe
bestN = Inf;
for xe = cand
    [sse, c] = ramps(xi, phi, xe, xe, xe);
    if sse < bestN
        bestN = sse;
        xN = xe;
        cN = c;
    end
end
% With one
best = Inf;
for i1 = 1:numel(cand)
    for i2 = i1:numel(cand)
        for i3 = i2:numel(cand)
            [sse, c] = ramps(xi, phi, cand(i1), cand(i2), cand(i3));
            if sse < best
                best = sse;
                bx = [cand(i1) cand(i2) cand(i3)];
                bc = c;
            end
        end
    end
end
% Refine around the best: eight steps across a candidate step either side
step = zeros(1, 3);
for k = 1:3
    j = find(cand == bx(k), 1);
    lo = cand(max(j - 1, 1));
    hi = cand(min(j + 1, numel(cand)));
    step(k) = max((hi - lo) / 8, 0.05);
end
x0 = bx;
for d1 = -4:4
    for d2 = -4:4
        for d3 = -4:4
            x = max(x0 + [d1 d2 d3] .* step, 0.05);
            if x(1) > x(2) || x(2) > x(3)
                continue;
            end
            [sse, c] = ramps(xi, phi, x(1), x(2), x(3));
            if sse < best
                best = sse;
                bx = x;
                bc = c;
            end
        end
    end
end
f = struct('plateau', false, 'b_um', 0, 'c0', min(max(cN(1), 0), 1), 'phiPlateau', NaN, ...
    'W_um', xN, 'windup_um', xN, 'x', [xN xN xN], 'sseNull', bestN, 'sseHinge', best);
nFlat = sum(xi > bx(1) & xi < bx(2));
if bx(2) - bx(1) >= opts.minBacklash_um && nFlat >= 3 && bestN >= opts.hingeRatio * best ...
        && bc(2) > -0.1 && bc(2) < 1.1
    f.plateau = true;
    f.b_um = bx(2) - bx(1);
    f.c0 = min(max(bc(1), 0), 1);
    f.phiPlateau = bc(2);
    f.W_um = bx(3);
    f.windup_um = bx(1) + bx(3) - bx(2);
    f.x = bx;
end
end


function [sse, c] = ramps(xi, phi, x1, x2, x3)
% Squared error of the transition with breakpoints x1 <= x2 <= x3 and the
% levels c = [phi(0); p] that fit best: phi(0) to p over [0, x1], p over
% [x1, x2], p to 1 over [x2, x3], 1 after.
w0 = zeros(size(xi));
w1 = zeros(size(xi));
o = zeros(size(xi));
a = xi < x1;
w0(a) = 1 - xi(a) / x1;
w1(a) = xi(a) / x1;
b = xi >= x1 & xi < x2;
w1(b) = 1;
cc = xi >= x2 & xi < x3;
u = (xi(cc) - x2) / (x3 - x2);
w1(cc) = 1 - u;
o(cc) = u;
o(xi >= x3) = 1;
y = phi - o;
G = [w0' * w0, w0' * w1; w0' * w1, w1' * w1];
g = [w0' * y; w1' * y];
if abs(det(G)) < 1e-12
    c = [0; 0];
    if w1' * w1 > 0
        c(2) = (w1' * y) / (w1' * w1);
    end
else
    c = G \ g;
end
r = y - c(1) * w0 - c(2) * w1;
sse = r' * r;
end


% ===========================================================================
function y = med3(x)
% Running median of three, the ends repeated.
x = x(:);
n = numel(x);
if n < 3
    y = x;
    return;
end
p = [x(1); x; x(n)];
y = median([p(1:n) p(2:n + 1) p(3:n + 2)], 2);
end


function printInertia(say, in)
say('\nInertia: torque excess over the steady torque after a step, against the motor acceleration\n');
say('%8s %4s %12s %20s %12s %9s %10s\n', '|dv| mm/s', 'n', 'J %/(mm/s2)', 'IQR', 'mass kg', 'dT st %', 'a max');
for q = 1:numel(in.sizes)
    s = in.sizes(q);
    say('%8.1f %4d %12.4f %9.4f..%-9.4f %12.3g %9.2f %10.0f\n', s.size, s.n, s.J, s.q25, s.q75, ...
        s.mass_kg, s.dTsteady, s.aMax);
end
say('all %d steps: J = %.4f %% per mm/s^2 = %.3g kg reflected at the piston (lossless torque constant)\n', ...
    in.n, in.J_pct_per_mm_s2, in.mass_kg);
end


function printFriction(say, fr, st, nPP)
say('\nFriction reversal under load: torque on the up ramp minus the down ramp at the same pressure\n');
for k = 1:numel(fr.logs)
    l = fr.logs(k);
    [~, nm] = fileparts(l.file);
    say('%s: %d pairs; median dT %.1f %% (%.1f kN), median |dx| %.1f um; k_s > %.0f N/um (%.2f %%/um)\n', ...
        nm, l.nPairs, l.medDT_pct, l.medDT_pct * nPP / 1e3, l.medDX_um, l.bound_N_per_um, l.bound_pct_per_um);
end
if isempty(fr.logs)
    say('no pair of opposite ramps in the logs\n');
    return;
end
say('%7s %9s %9s %9s\n', 'P MPa', 'dT %', 'dF kN', 'dx um');
for k = 1:numel(fr.P_MPa)
    say('%7.1f %9.1f %9.1f %9.1f\n', fr.P_MPa(k), fr.dT_pct(k), fr.dF_kN(k), fr.dX_um(k));
end
say('dT = %.1f + %.2f P %% (P in MPa): Fc %.2f kN, mu %.2f (2 Fc + 2 mu P A, lossless; Fc includes the viscous term at the ramp speed)\n', ...
    fr.a_pct, fr.b_pct_per_MPa, fr.Fc_N / 1e3, fr.mu);
if st.resolved
    say('an up/down offset above the %.0f um floor in at least one log: the offset is not the drive path alone\n', ...
        st.floor_um);
end
say('drive path: k_s > %.0f N/um (%.2f %% torque per um), from the weakest log', st.bound_N_per_um, st.bound_pct_per_um);
if ~isnan(st.boundPath_N_per_um)
    say('; k_s > %.0f N/um with the motor-side friction out of the reversal', st.boundPath_N_per_um);
end
say('\n');
end


function printReversal(say, rv, nPP)
say('\nReversal, open chamber: %d captures, %d analysed\n', numel(rv.rec), sum([rv.rec.ok]));
for k = find(~[rv.rec.ok])
    say('  %s: %s\n', rv.rec(k).file, rv.rec(k).why);
end
say('%8s %4s %5s %8s %8s %8s %9s %9s %10s\n', 'mm/s', 'n', 'plat', 'dT %', 'W um', 'b um', 'f_m %', ...
    'windup um', 'ks N/um');
for q = 1:numel(rv.groups)
    g = rv.groups(q);
    say('%8.1f %4d %5d %8.1f %8.1f %8.1f %9s %9.1f %10s\n', g.speed, g.n, g.nPlateau, g.dT_pct, g.W_um, ...
        g.b_um, fmt(g.fm_pct, '%.2f'), g.windup_um, ksText(g));
end
if ~isnan(rv.backlash_um)
    say('backlash %.1f um; motor-side friction %.2f %% = %.2f kN (lossless)\n', rv.backlash_um, ...
        rv.motorFriction_pct, rv.motorFriction_kN);
elseif ~isnan(rv.backlashBound_um)
    say('no backlash resolved: the torque reverses within %.1f um of motor travel\n', rv.backlashBound_um);
end
end


function printDither(say, dt, nPP)
say('\nDither, closed chamber: %d captures, %d analysed\n', numel(dt.rec), sum([dt.rec.ok]));
for k = find(~[dt.rec.ok])
    say('  %s: %s\n', dt.rec(k).file, dt.rec(k).why);
end
say('%8s %4s %4s %5s %8s %8s %8s %9s %10s %9s\n', 'P MPa', 'dir', 'n', 'plat', 'dT %', 'W um', 'b um', ...
    'windup um', 'ks N/um', 'dP MPa');
for q = 1:numel(dt.groups)
    g = dt.groups(q);
    say('%8.2f %4d %4d %5d %8.1f %8.1f %8.1f %9.1f %10s %9.3f\n', g.P_MPa, g.dir, g.n, g.nPlateau, ...
        g.dT_pct, g.W_um, g.b_um, g.windup_um, ksText(g), g.dP_MPa);
end
end


function txt = ksText(g)
if ~isnan(g.ks_N_per_um)
    txt = sprintf('%.0f', g.ks_N_per_um);
elseif ~isnan(g.ksBound_N_per_um)
    txt = sprintf('> %.0f', g.ksBound_N_per_um);
else
    txt = '-';
end
end


function txt = fmt(x, f)
if isnan(x)
    txt = '-';
else
    txt = sprintf(f, x);
end
end


% ===========================================================================
function m = med(x)
x = x(~isnan(x));
if isempty(x)
    m = NaN;
else
    m = median(x);
end
end


function q = quant(x, p)
% Linear-interpolated quantile.
x = sort(x(~isnan(x)));
n = numel(x);
if n == 0
    q = NaN;
    return;
end
pos = (n - 1) * p + 1;
lo = floor(pos);
hi = ceil(pos);
q = x(lo) + (x(hi) - x(lo)) * (pos - lo);
end


function c = check(name, value, lo, hi, text)
c = struct('name', name, 'value', value, 'lo', lo, 'hi', hi, 'text', text, ...
    'pass', all(value >= lo) && all(value <= hi));
end


function printChecks(say, checks)
for k = 1:numel(checks)
    c = checks{k};
    if c.pass
        v = 'pass';
    else
        v = 'FAIL';
    end
    say('  %-24s %s  %g  (%s)\n', c.name, v, c.value(1), c.text);
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
