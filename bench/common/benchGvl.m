function g = benchGvl()
%BENCHGVL The slave's G_cfg* settings, parsed from
%   slave-src/GVL/GlobalVariables_Reference.st, which must match what is
%   loaded on the controller. One field per declaration, TIME literals in
%   seconds, numbers as doubles; g.file is the file read. The benches take
%   the slew limits, the velocity clamp, the soft limits, the pressure
%   filter and the scan time from here.
%
%   A setting the parse does not find takes the reference value below, and
%   the function warns once naming the file and the names, so a run is not
%   stopped by a formatting difference.

here = fileparts(mfilename('fullpath'));
file = fullfile(here, '..', '..', 'slave-src', 'GVL', 'GlobalVariables_Reference.st');
g = readGvl(file);
ref = struct('G_cfgScanTime', 0.004, 'G_cfgAnalogFilterTimeConst', 0.020, 'G_cfgAnalogMedianSize', 5, ...
    'G_cfgDirectCtrlAccel', 5, 'G_cfgDirectCtrlDecel', 5, 'G_cfgVelLimitMax', 5, ...
    'G_cfgPressureAccel', 50, 'G_cfgPressureDecel', 50, 'G_cfgPressureVelMax', 5, ...
    'G_cfgTorqueLimitNormal', 100, 'G_cfgPressureTorqueLimit', 100, ...
    'G_cfgPosSoftLimitMax', 305, 'G_cfgPosSoftLimitMin', -2.5, ...
    'G_cfgPosCylinderEntry', 200, 'G_cfgPosExitGuardMargin', 20, ...
    'G_cfgGearRatio', 70, 'G_cfgLeadScrewPitch', 5.08);
names = fieldnames(ref);
missing = {};
for k = 1:numel(names)
    if ~isfield(g, names{k}) || isnan(g.(names{k}))
        g.(names{k}) = ref.(names{k});
        missing{end + 1} = names{k}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    warning('benchGvl:parse', ['%d of %d G_cfg* settings not parsed from %s (%d declarations read); ' ...
        'reference values used for: %s. Check that the file is the one on the controller.'], ...
        numel(missing), numel(names), file, numel(fieldnames(g)) - numel(missing), strjoin(missing, ', '));
end
g.file = file;
end


function g = readGvl(file)
% Values of the G_cfg* declarations in the GVL reference, one per line as
% "name : TYPE := value", whatever follows the value (a comment, a
% semicolon). TIME literals are returned in seconds; numbers as doubles.
g = struct();
if exist(file, 'file') ~= 2
    warning('benchGvl:parse', 'GVL reference %s not found', file);
    return;
end
txt = fileread(file);
txt = strrep(txt, char(13), '');
lines = strsplit(txt, char(10));
for k = 1:numel(lines)
    tok = regexp(lines{k}, '(G_cfg\w+)\s*:\s*(\w+)\s*:=\s*(T#\w+|[-+]?[0-9.eE+-]+)', 'tokens', 'once');
    if isempty(tok) || numel(tok) < 3
        continue;
    end
    name = tok{1};
    val = strtrim(tok{3});
    if strcmp(tok{2}, 'TIME')
        g.(name) = timeLiteral(val);
    else
        g.(name) = str2double(val);
    end
end
end


function s = timeLiteral(val)
% T#500MS, T#2S, T#2M, T#1H to seconds.
tok = regexp(val, '^T#([0-9.]+)(MS|S|M|H)$', 'tokens', 'once');
if isempty(tok)
    s = NaN;
    return;
end
n = str2double(tok{1});
switch tok{2}
    case 'MS', s = n / 1000;
    case 'S',  s = n;
    case 'M',  s = n * 60;
    case 'H',  s = n * 3600;
end
end
