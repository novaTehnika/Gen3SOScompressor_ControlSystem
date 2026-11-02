function r = charScheduleCheck(files, opts)
%CHARSCHEDULECHECK The slave's stiffness bound against the stiffness a log
%   measured (README section 9): the first step of commissioning the
%   scheduled pressure controller (FB_PressureSchedule, off by default
%   behind G_cfgPressureScheduleEnable). The block's bound is an FB output
%   that no register publishes, so the bench computes it from the same
%   settings (charScheduleBound, held equal to the compiled block by
%   tests/slavePressureMode.m) at the pressure and position of every ramp
%   of a log, and compares it with the ramp's measured dP/dx
%   (charBulkModulus).
%
%   r = charScheduleCheck(files)
%   r = charScheduleCheck(files, opts)
%
%   files   run logs of ramps (charRun), as for charBulkModulus
%
%   opts.salineVolume_mL  saline volume the master would send in HR9-10 for
%                         this fill (default masterConfig.salineVolume,
%                         343 mL, the fill of the 26 October K runs); a
%                         lower bound on the true volume is what the
%                         bound needs
%   opts.oxygenRatio      oxygen charge the master would send in HR11-12
%                         for it (default 0: a liquid-only run; a charged
%                         run passes its charge)
%   opts.hoseOpen         the proximal valve is open to the hose
%                         (G_cfgPressureSchedHoseOpen; default false, as
%                         in the K runs with the valve shut)
%   opts.settings         struct of GVL values replacing the file's
%                         (for example BulkModulus), by GVL name
%   opts.bulk             options for charBulkModulus (deadVolume_mL,
%                         fillVolume_mL, pressureRange, minRows, skip,
%                         logShuntOhms, ...); logShuntOhms also rescales the
%                         pressure used here
%   opts.margin           smallest bound / measured ratio that passes
%                         (default 1: the bound must not be below the
%                         stiffness)
%   opts.quiet            no printing
%
%   Per ramp the bound is evaluated at the pressure and the position of its
%   highest-pressure row, with the highest pressure since the start of the
%   log as the block's reference pressure and the pressure as its target;
%   for a gas-charged chamber, which stiffens with pressure, that is the
%   end the bound is tightest at. The schedule sets
%     Kp = LIMIT(KpMin, Crossover / k_bound, KpMax)
%   so the loop crosses over at Kp times the true stiffness; the table
%   gives that crossover (rad/s) against the configured one.
%
%   r.rows       per ramp: file, step, dir, P_MPa (top), x_mm, dPdx
%                (measured, MPa/mm), kBound (MPa/mm), ratio (bound over
%                measured), Kp (mm/s per MPa), wc (rad/s, with the
%                measured stiffness)
%   r.minRatio   smallest ratio; the bound is safe where it is at least 1
%   r.lag_s      the stop law's assumed delay (Lag)
%   r.pass       every ramp's ratio is at least opts.margin
%   r.bulk       the charBulkModulus result for the same files

if nargin < 2
    opts = struct();
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'), fullfile(here, '..', 'common'));
files = benchFiles(files);
m = masterConfig();
g = benchGvl();
opts = withDefault(opts, 'salineVolume_mL', m.salineVolume);
opts = withDefault(opts, 'oxygenRatio', 0);
opts = withDefault(opts, 'hoseOpen', false);
opts = withDefault(opts, 'settings', struct());
opts = withDefault(opts, 'bulk', struct());
opts = withDefault(opts, 'margin', 1);
opts = withDefault(opts, 'quiet', false);
say = @(varargin) sayIf(~opts.quiet, varargin{:});
names = fieldnames(opts.settings);
for k = 1:numel(names)
    g.(names{k}) = opts.settings.(names{k});
end
required = {'G_cfgPressureSchedCrossover', 'G_cfgPressureSchedKpMin', 'G_cfgPressureSchedKpMax', ...
    'G_cfgPressureSchedPistonArea', 'G_cfgPressureSchedEndOfTravel', 'G_cfgPressureSchedPassagesVol', ...
    'G_cfgPressureSchedHoseVolume', 'G_cfgPressureSchedHoseCompl', 'G_cfgPressureSchedBulkModulus', ...
    'G_cfgPressureSchedHenry', 'G_cfgPressureSchedGasTemp', 'G_cfgPressureSchedPatm', ...
    'G_cfgPressureSchedDeadTime', 'G_cfgPressureSchedDriveLag'};
missing = required(~isfield(g, required));
if ~isempty(missing)
    error('charScheduleCheck:gvl', 'not found in %s: %s', g.file, strjoin(missing, ', '));
end
fill = struct('saline_mL', opts.salineVolume_mL, 'oxygenRatio', opts.oxygenRatio, 'hoseOpen', opts.hoseOpen);

bopts = opts.bulk;
bopts.quiet = true;
bk = charBulkModulus(files, bopts);
seg = bk.segments;

rows = struct('file', {}, 'step', {}, 'dir', {}, 'P_MPa', {}, 'x_mm', {}, 'dPdx', {}, ...
    'kBound', {}, 'ratio', {}, 'Kp', {}, 'wc', {});
logs = cell(1, numel(files));
for k = 1:numel(seg)
    s = seg(k);
    if isempty(logs{s.fileIdx})
        d = benchReadCsv(files{s.fileIdx});
        d.pressure = benchPressureRescale(d.pressure, getOr(bopts, 'logShuntOhms', []));
        logs{s.fileIdx} = d;
    end
    d = logs{s.fileIdx};
    Ptop = d.pressure(s.rowTop);                                % [MPa]
    xTop = d.position(s.rowTop);                                % [mm]
    pRef = max(d.pressure(1:s.rowTop));                         % [MPa]
    [kEst, kTarget, lag] = charScheduleBound(Ptop, xTop, pRef, Ptop, fill, g);
    kBound = max(kEst, kTarget);                                % [MPa/mm]
    Kp = min(max(g.G_cfgPressureSchedCrossover / kBound, g.G_cfgPressureSchedKpMin), ...
        g.G_cfgPressureSchedKpMax);                             % [mm/s per MPa]
    row.file = s.file;
    row.step = s.step;
    row.dir = s.dir;
    row.P_MPa = Ptop;
    row.x_mm = xTop;
    row.dPdx = s.dPdx;
    row.kBound = kBound;
    row.ratio = kBound / s.dPdx;
    row.Kp = Kp;
    row.wc = Kp * s.dPdx;                                       % [mm/s per MPa * MPa/mm = 1/s]
    rows(end + 1) = row;                                        %#ok<AGROW>
end

say('\n=== charScheduleCheck: %d ramp(s) of %d file(s); saline %g mL, oxygen %g:1, hose %s\n', ...
    numel(rows), numel(files), fill.saline_mL, fill.oxygenRatio, openText(fill.hoseOpen));
say('Bulk modulus %.2f GPa (an upper bound), crossover %g rad/s, Kp within %g..%g mm/s per MPa.\n', ...
    g.G_cfgPressureSchedBulkModulus / 1e9, g.G_cfgPressureSchedCrossover, ...
    g.G_cfgPressureSchedKpMin, g.G_cfgPressureSchedKpMax);
say('%-22s %4s %-4s %7s %8s %11s %11s %7s %8s %9s\n', 'file', 'step', 'dir', 'P MPa', 'x mm', ...
    'measured', 'bound', 'ratio', 'Kp', 'wc rad/s');
for k = 1:numel(rows)
    q = rows(k);
    say('%-22s %4d %-4s %7.2f %8.2f %6.2f MPa/mm %6.2f MPa/mm %7.2f %8.3f %9.2f\n', q.file, q.step, q.dir, ...
        q.P_MPa, q.x_mm, q.dPdx, q.kBound, q.ratio, q.Kp, q.wc);
end

r.rows = rows;
r.minRatio = min([rows.ratio]);
r.lag_s = lag;
r.pass = r.minRatio >= opts.margin;
r.bulk = bk;
say('\nbound / measured: smallest %.2f, largest %.2f; loop crossover with the measured stiffness %.1f to %.1f rad/s (configured %g)\n', ...
    r.minRatio, max([rows.ratio]), min([rows.wc]), max([rows.wc]), g.G_cfgPressureSchedCrossover);
if r.pass
    say('pass: the bound is not below the measured stiffness on any ramp (margin %.2f)\n', opts.margin);
else
    say(['FAIL: the bound is below the measured stiffness on %d ramp(s): the schedule would set a gain that crosses over\n' ...
        '      above %g rad/s there. Check the saline volume and charge sent, G_cfgPressureSchedBulkModulus and\n' ...
        '      the position of the end of travel against the machine.\n'], ...
        sum([rows.ratio] < opts.margin), g.G_cfgPressureSchedCrossover);
end
say('Stop-law delay the schedule assumes (Lag): %.1f ms; compare charStopDistance''s r.T0_s.\n', 1000 * lag);
end


function t = openText(open)
if open
    t = 'open';
else
    t = 'shut';
end
end


function v = getOr(s, name, default)
if isfield(s, name)
    v = s.(name);
else
    v = default;
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
