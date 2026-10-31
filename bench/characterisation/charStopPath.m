function [path, plan] = charStopPath(opts)
%CHARSTOPPATH Build the one-pass path of test 4 (README section 6): a
%   staircase of pressure levels joined by ramps, with an excursion set at
%   each level that measures the stopping distance under load in both
%   directions, the drive lag under load, and the friction at speed, and
%   that returns the piston to the level before the next speed.
%
%   [path, plan] = charStopPath()
%   [path, plan] = charStopPath(opts)
%
%   At each level the set is, for every speed v that fits the budget:
%     {'move', +v, d, maxSec}  {'dwell', pause}  {'move', -v, d, maxSec}  {'dwell', pause}
%   The move runs d mm at v and the dwell that follows is the stop, which
%   the step probe captures (charRun arms it before every command change
%   in a velocity session when opts.probe is set). The pair returns the
%   piston to where it started, so the level is unchanged.
%
%   The distance d is set by the pressure budget at the level: the leg
%   plus the stopping distance the slave's own law allows (v T0 + v^2 /
%   (2 D)) must not move the pressure by more than the budget. The travel
%   that the budget allows is the compliance integrated from the level to
%   the level plus the budget, at the stiffness the model predicts for the
%   fill and the gas charge (which rises through the excursion when gas
%   dominates). A speed is left out of a level when the steady part of its
%   leg would be shorter than opts.minRun.
%
%   opts.levels_MPa     pressure levels, MPa gauge (default [2 4 6 8 10])
%   opts.speeds         mm/s, each run in both directions (default [1 2 5 7])
%   opts.budget_MPa     pressure excursion allowed at a level, one value or
%                       one per level (default 2.0, and 1.0 at or above 9 MPa;
%                       the run's guard is the backstop)
%   opts.rampSpeed      mm/s between levels (default 2); the ramps are the
%                       stiffness measurement
%   opts.hold_s         dwell at each level before the set (default 2)
%   opts.pause_s        dwell after each leg, the stop (default 1)
%   opts.T0_s, opts.D_mm_s2
%                       stopping-distance allowance v T0 + v^2 / (2 D)
%                       (default 0.05 s and charConfig.pressureDecel)
%   opts.minRun_mm      shortest steady part of a leg (default 0.2 mm or
%                       0.15 s at the speed, whichever is larger)
%   opts.fillVolume_mL  saline at the first level (default 343)
%   opts.chargeRatio    gas charge, STP volume per saline volume, taken as
%                       free gas at every level (default 2.0, air: softer
%                       than the oxygen runs, so the fast legs fit at
%                       pressure; the stopping distance depends on the
%                       load, not on the stiffness)
%   opts.K_Pa           bulk modulus of the saline (default 0.72e9)
%   opts.C_hose_m3_Pa   hose compliance when the hose is open (default 0,
%                       the proximal valve shut)
%   opts.stiffness      function P_MPa -> k in MPa/mm, replacing the model
%                       above (the simulator test gives its gas spring)
%   opts.pressureStop   MPa, the run's guard; levels plus budget must stay
%                       below it (default charConfig.pressureStop)
%   opts.deadVolume_mL  passages at the end of travel, for the piston
%                       position at each level (default charConfig)
%   opts.posStop        mm, the run's position guard; every level must
%                       sit below it (default charConfig.posStop)
%   opts.quiet          no printing
%
%   path   the cell array of steps for charRun
%   plan   per level: P_MPa, k_MPa_mm, x_mm (the piston position the
%          model puts the level at, with the hose shut), budget_MPa,
%          speeds, d_mm (the move distance), stop_mm (the allowance) and
%          rise_MPa (the predicted excursion); plan.nStops; plan.seconds
%          (an estimate of the run)

if nargin < 1
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'common'));
cfg = charConfig();
known = {'levels_MPa', 'speeds', 'budget_MPa', 'rampSpeed', 'hold_s', 'pause_s', 'T0_s', ...
    'D_mm_s2', 'minRun_mm', 'fillVolume_mL', 'chargeRatio', 'K_Pa', 'C_hose_m3_Pa', ...
    'stiffness', 'pressureStop', 'deadVolume_mL', 'posStop', 'quiet'};
given = fieldnames(opts);
unknown = given(~ismember(given, known));
if ~isempty(unknown)
    error('charStopPath:option', 'unknown option %s (the names are case-sensitive: %s)', ...
        strjoin(unknown, ', '), strjoin(known, ', '));
end
opts = withDefault(opts, 'levels_MPa', [2 4 6 8 10]);
opts = withDefault(opts, 'speeds', [1 2 5 7]);
opts = withDefault(opts, 'budget_MPa', []);
opts = withDefault(opts, 'rampSpeed', 2);
opts = withDefault(opts, 'hold_s', 2);
opts = withDefault(opts, 'pause_s', 1);
opts = withDefault(opts, 'T0_s', 0.05);
opts = withDefault(opts, 'D_mm_s2', cfg.pressureDecel);
opts = withDefault(opts, 'minRun_mm', 0.2);
opts = withDefault(opts, 'fillVolume_mL', 343);
opts = withDefault(opts, 'chargeRatio', 2.0);
opts = withDefault(opts, 'K_Pa', 0.72e9);
opts = withDefault(opts, 'C_hose_m3_Pa', 0);
opts = withDefault(opts, 'stiffness', []);
opts = withDefault(opts, 'pressureStop', cfg.pressureStop);
opts = withDefault(opts, 'deadVolume_mL', cfg.deadVolume_mL);
opts = withDefault(opts, 'posStop', cfg.posStop);
opts = withDefault(opts, 'quiet', false);
say = @(varargin) sayIf(~opts.quiet, varargin{:});

levels = opts.levels_MPa(:)';
nL = numel(levels);
budget = opts.budget_MPa;
if isempty(budget)
    budget = 2.0 * ones(1, nL);
    budget(levels >= 9) = 1.0;
elseif isscalar(budget)
    budget = budget * ones(1, nL);
end
assert(numel(budget) == nL, 'one budget per level');
assert(all(levels + budget <= opts.pressureStop), ...
    'a level plus its budget exceeds the %.1f MPa guard', opts.pressureStop);
assert(all(abs(opts.speeds) <= cfg.velMax), 'a speed exceeds G_cfgVelLimitMax');

% Stiffness at a level, MPa/mm: the free gas (isothermal) and the saline
% in series with the hose, eq. (4) of the design README, with the saline
% volume at the level taken as the fill (the sweep between levels is a
% few mL).
if isempty(opts.stiffness)
    A_m2 = cfg.pistonArea_mm2 * 1e-6;
    Vl = opts.fillVolume_mL * 1e-6;                         % m^3
    Vstp = opts.chargeRatio * Vl;                           % m^3 at atmosphere
    kOf = @(P) A_m2 / (Vstp * cfg.Patm_MPa / (P + cfg.Patm_MPa) / ((P + cfg.Patm_MPa) * 1e6) ...
        + Vl / opts.K_Pa + opts.C_hose_m3_Pa) / 1e9;          % Pa/m -> MPa/mm
else
    kOf = opts.stiffness;
end
% Piston position at a level, mm: the chamber holds the saline and the
% charge compressed isothermally; NaN with a stiffness function given
if isempty(opts.stiffness)
    xOf = @(P) cfg.posEOT - (opts.fillVolume_mL + opts.chargeRatio * opts.fillVolume_mL ...
        * cfg.Patm_MPa / (P + cfg.Patm_MPa) - opts.deadVolume_mL) * 1000 / cfg.pistonArea_mm2;
else
    xOf = @(P) NaN;
end

stopOf = @(v) v * opts.T0_s + v^2 / (2 * opts.D_mm_s2);     % mm
% Travel between two pressures: the compliance 1/k integrated over the
% pressure, on a fine grid
travelOf = @(P1, P2) travelBetween(kOf, P1, P2);           % mm
path = {};
plan = struct('P_MPa', {}, 'k_MPa_mm', {}, 'x_mm', {}, 'budget_MPa', {}, 'speeds', {}, ...
    'd_mm', {}, 'stop_mm', {}, 'rise_MPa', {});
seconds = 0;
nStops = 0;
say('one-pass stopping-distance path: %d levels, speeds %s mm/s, charge %g:1 air, fill %g mL\n', ...
    nL, mat2str(opts.speeds), opts.chargeRatio, opts.fillVolume_mL);
say('%6s %10s %8s %10s %6s %8s %8s %8s\n', 'P MPa', 'k MPa/mm', 'x mm', 'budget', 'v', 'd mm', 'stop mm', 'rise');
Pprev = 0;
for L = 1:nL
    P = levels(L);
    k = kOf(P);
    % Ramp up to the level at the ramp speed; the time allowed covers twice
    % the travel at the model's stiffness, with a floor for a soft start.
    travel = travelOf(max(Pprev, 0.5), P);
    path{end + 1} = {'ramp', opts.rampSpeed, P, max(2 * travel / opts.rampSpeed, 5) + 10};
    seconds = seconds + travel / opts.rampSpeed;
    path{end + 1} = {'dwell', opts.hold_s};
    seconds = seconds + opts.hold_s;
    x = xOf(P);
    if x > opts.posStop
        error('charStopPath:position', ['the %.0f MPa level sits at %.0f mm, beyond the %.0f mm ' ...
            'position guard: add saline (the chamber at the top level must hold more than ' ...
            '%.0f mL) or raise opts.posStop if the slave''s soft limit allows'], P, x, opts.posStop, ...
            cfg.pistonArea_mm2 * (cfg.posEOT - opts.posStop) / 1000 + opts.deadVolume_mL);
    end
    pl = struct('P_MPa', P, 'k_MPa_mm', k, 'x_mm', x, 'budget_MPa', budget(L), 'speeds', [], ...
        'd_mm', [], 'stop_mm', [], 'rise_MPa', []);
    allowed = travelOf(P, P + budget(L));                   % mm within the budget
    for v = opts.speeds(:)'
        xs = stopOf(v);
        d = allowed - xs;                                   % the leg itself
        dmin = max(opts.minRun_mm, 0.15 * v);
        if d < dmin
            say('%6.1f %10.2f %8.1f %10.2f %6g %8s\n', P, k, x, budget(L), v, 'skipped');
            continue;
        end
        maxSec = 2 * d / v + 2;
        path{end + 1} = {'move', v, d, maxSec};
        path{end + 1} = {'dwell', opts.pause_s};
        path{end + 1} = {'move', -v, d, maxSec};
        path{end + 1} = {'dwell', opts.pause_s};
        seconds = seconds + 2 * (d / v + opts.pause_s);
        nStops = nStops + 2;
        pl.speeds(end + 1) = v;
        pl.d_mm(end + 1) = d;
        pl.stop_mm(end + 1) = xs;
        pl.rise_MPa(end + 1) = budget(L);
        say('%6.1f %10.2f %8.1f %10.2f %6g %8.3f %8.3f %8.2f\n', P, k, x, budget(L), v, d, xs, budget(L));
    end
    plan(end + 1) = pl;
    Pprev = P;
end
% Back down to the first level at the ramp speed: the stiffness on the way
% out and the return-stroke torque.
travel = travelOf(levels(1), levels(end));
path{end + 1} = {'ramp', -opts.rampSpeed, levels(1), max(2 * travel / opts.rampSpeed, 5) + 10};
path{end + 1} = {'dwell', opts.hold_s};
seconds = seconds + travel / opts.rampSpeed + opts.hold_s;
plan(1).nStops = nStops;
plan(1).seconds = seconds;
say('%d steps, %d stops, about %.0f s of motion\n', numel(path), nStops, seconds);
end


function x = travelBetween(kOf, P1, P2)
% [mm] piston travel that takes the pressure from P1 to P2 [MPa]: the
% compliance 1/k [mm/MPa] integrated over the pressure
Pg = linspace(P1, P2, 201);
c = arrayfun(@(P) 1 / kOf(P), Pg);
x = trapz(Pg, c);
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
