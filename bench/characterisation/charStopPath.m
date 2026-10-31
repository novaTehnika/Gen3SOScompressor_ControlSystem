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
%     {'settle', 0.5, P, maxSec}
%   The move runs d mm at v and the dwell that follows is the stop, which
%   the step probe captures (charRun arms it for every command to zero
%   with opts.probe = 'stops'). The pair returns the piston to where it
%   started; the settle then brings the pressure back to the level at a
%   slow speed, since the bench's latency at the end of each leg and the
%   gas dissolving leave the pair a little off it.
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
%   opts.pause_s        dwell after each leg, the stop (default 2; a probe
%                       capture and its readout need about 2 s, so run
%                       charRun with opts.probe = 'stops' and a pause of 3)
%   opts.T0_s, opts.D_mm_s2, opts.latency_s
%                       stopping-distance allowance v (T0 + latency) +
%                       v^2 / (2 D): T0 the delay before the drive acts on
%                       a command (31 ms measured), D its ramp down (none
%                       measurable), and latency the bench's own: a move
%                       or ramp ends when the master sees it in an
%                       exchange and the stop goes out in the next, two
%                       cycles of 25 ms on a clean link and of 100 ms with
%                       a large register group mapped (defaults
%                       0.05 s, charConfig.pressureDecel and 0.2 s)
%   opts.rampOvershoot_MPa
%                       how far a ramp may overshoot its level once its
%                       stop is over; each ramp's speed is the smaller of
%                       opts.rampSpeed and what this allows at the level's
%                       stiffness (default 0.5)
%   opts.minRun_mm, opts.minSteady_s
%                       shortest steady part of a leg: the larger of
%                       minRun_mm and minSteady_s at the speed (defaults
%                       0.2 mm and 0.15 s; 0 and 0 for stops alone, which
%                       need no steady part)
%   opts.fillVolume_mL  saline at the first level (default 343)
%   opts.chargeRatio    gas charge admitted, STP volume per liquid volume
%                       (default 2.0, air: softer than the oxygen runs, so
%                       the fast legs fit at pressure; the stopping
%                       distance depends on the load, not on the
%                       stiffness)
%   opts.solubility     mL of gas at STP that a mL of the liquid holds per
%                       atmosphere of absolute pressure; the free gas at a
%                       level is the charge less what the liquid holds at
%                       equilibrium there, the stiffest the chamber can be
%                       (default 0.018, air in water at room temperature;
%                       oxygen is 0.03, and 0 keeps all the charge free)
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
%   opts.P_start_MPa    pressure the run starts from, for the time the
%                       first ramp is allowed (default 0, atmosphere)
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
    'latency_s', 'rampOvershoot_MPa', 'minSteady_s', ...
    'D_mm_s2', 'minRun_mm', 'fillVolume_mL', 'chargeRatio', 'solubility', 'K_Pa', ...
    'C_hose_m3_Pa', 'stiffness', 'pressureStop', 'deadVolume_mL', 'posStop', 'P_start_MPa', ...
    'quiet'};
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
opts = withDefault(opts, 'pause_s', 2);
opts = withDefault(opts, 'T0_s', 0.05);
opts = withDefault(opts, 'latency_s', 0.2);
opts = withDefault(opts, 'rampOvershoot_MPa', 0.5);
opts = withDefault(opts, 'D_mm_s2', cfg.pressureDecel);
opts = withDefault(opts, 'minRun_mm', 0.2);
opts = withDefault(opts, 'minSteady_s', 0.15);
opts = withDefault(opts, 'fillVolume_mL', 343);
opts = withDefault(opts, 'chargeRatio', 2.0);
opts = withDefault(opts, 'solubility', 0.018);
opts = withDefault(opts, 'K_Pa', 0.72e9);
opts = withDefault(opts, 'C_hose_m3_Pa', 0);
opts = withDefault(opts, 'stiffness', []);
opts = withDefault(opts, 'pressureStop', cfg.pressureStop);
opts = withDefault(opts, 'deadVolume_mL', cfg.deadVolume_mL);
opts = withDefault(opts, 'posStop', cfg.posStop);
opts = withDefault(opts, 'P_start_MPa', 0);
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
% few mL). The free gas is the charge less what the liquid holds in
% solution at equilibrium at the level (Henry's law, eq. (2)), which is
% the least gas and so the stiffest chamber the level can present.
if isempty(opts.stiffness)
    A_m2 = cfg.pistonArea_mm2 * 1e-6;
    Vl = opts.fillVolume_mL * 1e-6;                         % m^3
    freeStp = @(P) max(opts.chargeRatio * Vl ...
        - opts.solubility * (P + cfg.Patm_MPa) / cfg.Patm_MPa * Vl, 0);   % m^3 at atmosphere
    kOf = @(P) A_m2 / (freeStp(P) * cfg.Patm_MPa / (P + cfg.Patm_MPa) / ((P + cfg.Patm_MPa) * 1e6) ...
        + Vl / opts.K_Pa + opts.C_hose_m3_Pa) / 1e9;          % Pa/m -> MPa/mm
else
    kOf = opts.stiffness;
end
% Piston position at a level, mm: the chamber holds the saline and the
% charge compressed isothermally; NaN with a stiffness function given
if isempty(opts.stiffness)
    xOf = @(P) cfg.posEOT - (opts.fillVolume_mL + freeStp(P) * 1e6 ...
        * cfg.Patm_MPa / (P + cfg.Patm_MPa) - opts.deadVolume_mL) * 1000 / cfg.pistonArea_mm2;
else
    xOf = @(P) NaN;
end

stopOf = @(v) v * (opts.T0_s + opts.latency_s) + v^2 / (2 * opts.D_mm_s2);   % mm
% Ramp speed a level allows: its stop, after the bench's latency, must
% not carry the pressure more than rampOvershoot past the level
rampSpeedAt = @(k) min(opts.rampSpeed, opts.rampOvershoot_MPa / (k * (opts.T0_s + opts.latency_s)));
% Travel between two pressures: the compliance 1/k integrated over the
% pressure, on a fine grid
travelOf = @(P1, P2) travelBetween(kOf, P1, P2);           % mm
path = {};
plan = struct('P_MPa', {}, 'k_MPa_mm', {}, 'x_mm', {}, 'budget_MPa', {}, 'speeds', {}, ...
    'd_mm', {}, 'stop_mm', {}, 'rise_MPa', {});
seconds = 0;
nStops = 0;
say(['one-pass stopping-distance path: %d levels, speeds %s mm/s, charge %g:1, fill %g mL, ' ...
    'solubility %g mL/mL/atm\n'], nL, mat2str(opts.speeds), opts.chargeRatio, opts.fillVolume_mL, ...
    opts.solubility);
say('%6s %10s %8s %10s %6s %8s %8s %8s\n', 'P MPa', 'k MPa/mm', 'x mm', 'budget', 'v', 'd mm', 'stop mm', 'rise');
Pprev = opts.P_start_MPa;
for L = 1:nL
    P = levels(L);
    k = kOf(P);
    % Ramp up to the level at the ramp speed; the time allowed is twice
    % the travel the model predicts from the pressure the ramp starts at
    % (the run's start for the first), plus a margin. Near atmosphere the
    % gas makes the chamber very soft, so the first ramp can be long.
    % The ramp runs in stages of a megapascal, each at the speed the
    % stiffness at its end allows, so that it is fast while the chamber is
    % soft and slow only near the level; stages at one speed are merged.
    P0 = max(Pprev, 0.02);
    edges = [P0, (ceil(P0 + 1e-9):1:P - 0.5), P];
    edges = unique(edges(edges <= P));
    vStage = arrayfun(@(Pe) rampSpeedAt(kOf(Pe)), edges(2:end));
    q = 1;
    while q <= numel(vStage)
        r2 = q;
        while r2 < numel(vStage) && abs(vStage(r2 + 1) - vStage(q)) < 1e-9
            r2 = r2 + 1;
        end
        Pend = edges(r2 + 1);
        vRamp = vStage(q);
        travel = travelOf(edges(q), Pend);
        path{end + 1} = {'ramp', vRamp, Pend, max(2 * travel / vRamp, 5) + 10};
        seconds = seconds + travel / vRamp;
        say('%6s ramp to %.1f MPa: about %.1f mm, %.0f s at %.2f mm/s\n', '', Pend, travel, ...
            travel / vRamp, vRamp);
        q = r2 + 1;
    end
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
        dmin = max(opts.minRun_mm, opts.minSteady_s * v);
        if d < dmin
            say('%6.1f %10.2f %8.1f %10.2f %6g %8s\n', P, k, x, budget(L), v, 'skipped');
            continue;
        end
        maxSec = 2 * d / v + 2;
        path{end + 1} = {'move', v, d, maxSec};
        path{end + 1} = {'dwell', opts.pause_s};
        path{end + 1} = {'move', -v, d, maxSec};
        path{end + 1} = {'dwell', opts.pause_s};
        path{end + 1} = {'settle', 0.5, P, 2 * d / 0.5 + 5};
        seconds = seconds + 2 * (d / v + opts.pause_s) + 2;
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
vRamp = rampSpeedAt(kOf(levels(1)));
path{end + 1} = {'ramp', -vRamp, levels(1), max(2 * travel / vRamp, 5) + 10};
path{end + 1} = {'dwell', opts.hold_s};
seconds = seconds + travel / vRamp + opts.hold_s;
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
