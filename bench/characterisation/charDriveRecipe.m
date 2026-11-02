function [path, info] = charDriveRecipe(kind, opts)
%CHARDRIVERECIPE Build the paths of test 5 (README section 10), the two
%   runs that put a reversal of the drive through the motor's position
%   and torque: all from 'velstep', so charRun needs nothing new.
%
%   [path, info] = charDriveRecipe('reversal')
%   [path, info] = charDriveRecipe('reversal', opts)
%   [path, info] = charDriveRecipe('dither', opts)
%
%   'reversal', open chamber (session 0). For each speed in opts.speeds
%   one velstep: +v from rest, then opts.nReversals direct reversals
%   (-v, +v, ...) with no stop between, each command held opts.hold_s,
%   then a stop. The commands alternate, so the piston ends where it
%   began after the last pair; the first command goes the way
%   opts.firstSign says. Run it with charRun's opts.probe = true (the
%   probe is armed at every command change), the piston in the bore and
%   the chamber open, with opts.travel_mm of room in the direction of
%   the first command. charDrivePath reads every reversal as a capture.
%
%   'dither', closed liquid-filled chamber (session 1). For each pressure
%   in opts.levels_MPa a ramp to the level (at opts.rampSpeed, upward or
%   downward as the level lies above or below the one before, the first
%   from opts.P_start_MPa), a dwell, and a velstep of triangles about the
%   level: half a leg down, opts.cycles cycles of a leg up and a leg
%   down, half a leg up (back to the level), each held opts.leg_s at
%   opts.speed, and a stop. The legs are +-opts.amplitude_mm about the
%   level, so the pressure swings by the amplitude times the chamber's
%   stiffness (opts.stiffness_MPa_mm: 7.2 MPa/mm for 343 mL with the
%   proximal valve shut, README section 3); the builder stops with an
%   error when the top of the swing would come within 0.5 MPa of the
%   run's pressure guard. Run it with opts.probe = true.
%
%   opts.speeds          mm/s of the reversal runs (default [0.3 1 2.5])
%   opts.nReversals      reversals per speed; an odd number (an even
%                        number of commands) returns the piston to where
%                        it began (default 7)
%   opts.hold_s          s per command (default 2: a capture is 500 scans
%                        of 4 ms)
%   opts.firstSign       direction of the first command (default +1)
%   opts.travel_mm       returned: the largest excursion from the start
%   opts.levels_MPa      mean pressures of the dither (default [1 5 9])
%   opts.P_start_MPa     pressure the run starts at (default 0)
%   opts.rampSpeed       mm/s of the ramps to a level (default 0.1)
%   opts.speed           mm/s of the dither legs (default 0.2)
%   opts.amplitude_mm    half the travel of a triangle (default 0.15)
%   opts.cycles          triangles per level (default 10)
%   opts.leg_s           s per leg (default amplitude * 2 / speed: 1.5)
%   opts.stiffness_MPa_mm
%                        chamber stiffness, for the pressure swing
%                        (default 7.2)
%
%   info.duration_s      motion time of the path
%   info.nCaptures       probe captures the run takes (one per command
%                        change, the ramps' starts and their dwells too)
%   info.travel_mm       the largest excursion of the position from its
%                        start along the path
%   info.swing_MPa       (dither) half the pressure swing at the
%                        stiffness given
%   info.maxPressure_MPa (dither) the top of the swing at the highest level

if nargin < 2
    opts = struct();
end
switch lower(kind)
    case 'reversal'
        [path, info] = reversalPath(opts);
    case 'dither'
        [path, info] = ditherPath(opts);
    otherwise
        error('charDriveRecipe: kind is ''reversal'' or ''dither'', not ''%s''', kind);
end
end


function [path, info] = reversalPath(opts)
opts = withDefault(opts, 'speeds', [0.3 1 2.5]);
opts = withDefault(opts, 'nReversals', 7);
opts = withDefault(opts, 'hold_s', 2);
opts = withDefault(opts, 'firstSign', 1);
v = [];
for sp = opts.speeds(:)'
    s = opts.firstSign * (-1) .^ (0:opts.nReversals);        % +v, then alternating
    v = [v sp * s 0];                                        %#ok<AGROW>
end
path = {{'velstep', v, opts.hold_s * ones(size(v))}};
info.duration_s = opts.hold_s * numel(v);
info.nCaptures = numel(v);
travel = 0;
for sp = opts.speeds(:)'
    s = opts.firstSign * (-1) .^ (0:opts.nReversals);
    x = cumsum(sp * s * opts.hold_s);                        % [mm/s x s -> mm]
    travel = max(travel, max(abs(x)));
end
info.travel_mm = travel;
end


function [path, info] = ditherPath(opts)
ccfg = charConfig();
opts = withDefault(opts, 'levels_MPa', [1 5 9]);
opts = withDefault(opts, 'P_start_MPa', 0);
opts = withDefault(opts, 'rampSpeed', 0.1);
opts = withDefault(opts, 'speed', 0.2);
opts = withDefault(opts, 'amplitude_mm', 0.15);
opts = withDefault(opts, 'cycles', 10);
opts = withDefault(opts, 'leg_s', 2 * opts.amplitude_mm / opts.speed);
opts = withDefault(opts, 'stiffness_MPa_mm', 7.2);
swing = opts.amplitude_mm * opts.stiffness_MPa_mm;          % [mm x MPa/mm -> MPa]
top = max(opts.levels_MPa) + swing;
if top > ccfg.pressureStop - 0.5
    error(['charDriveRecipe: the dither about %.1f MPa swings to %.1f MPa at %.1f MPa/mm, within 0.5 MPa of the ' ...
        'run''s %.1f MPa guard: lower the level or the amplitude'], max(opts.levels_MPa), top, ...
        opts.stiffness_MPa_mm, ccfg.pressureStop);
end
half = opts.amplitude_mm / opts.speed;                      % [s] half a leg
v = [-1 repmat([1 -1], 1, opts.cycles) 1 0] * opts.speed;   % down, cycles of up and down, up, stop
t = [half opts.leg_s * ones(1, 2 * opts.cycles) half max(opts.leg_s, 2)];
path = {};
last = opts.P_start_MPa;
for P = opts.levels_MPa(:)'
    if P ~= last
        path{end + 1} = {'ramp', sign(P - last) * opts.rampSpeed, P, 240};     %#ok<AGROW>
    end
    path{end + 1} = {'dwell', 2};                                              %#ok<AGROW>
    path{end + 1} = {'velstep', v, t};                                         %#ok<AGROW>
    last = P;
end
info.duration_s = numel(opts.levels_MPa) * (sum(t) + 2);
info.nCaptures = numel(opts.levels_MPa) * (numel(v) + 2);       % the ramp's start and the dwell too
info.travel_mm = opts.amplitude_mm;
info.swing_MPa = swing;
info.maxPressure_MPa = top;
end


function opts = withDefault(opts, name, value)
if ~isfield(opts, name) || isempty(opts.(name))
    opts.(name) = value;
end
end
