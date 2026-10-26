function cfg = charConfig()
%CHARCONFIG Constants for the pressure loop characterisation bench: the
%   chamber and outlet geometry the fits need, the pressure limits for the
%   runs and the slave settings read from the GVL reference.
%
%   Geometry comes from masterConfig (bore, end of travel, dead volume) so
%   the bench and the operator app agree. The dead volume there is a
%   placeholder: measure it (README section 2) and pass it to the fits.
%
%   Slave settings are parsed from slave-src/GVL/GlobalVariables_Reference.st,
%   which must match what is loaded on the controller.

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
addpath(fullfile(root, 'master-src', 'logic'), fullfile(here, '..', 'modbus'));
m = masterConfig();

% --- Chamber -----------------------------------------------------------------
cfg.boreDiameter_mm = m.boreDiameter;                    % 2.602 in
cfg.pistonArea_mm2  = pi / 4 * m.boreDiameter^2;         % 3431 mm^2 (3.431e-3 m^2)
cfg.posEOT          = m.posEOT;                          % mm, piston at the end of travel
cfg.deadVolume_mL   = m.deadVolume;                      % mL left at the end of travel: placeholder, measure it
cfg.rho             = 1000;                              % kg/m^3 saline
cfg.Patm_MPa        = 0.101325;

% --- Outlet: ball valve, tube, orifice plate -----------------------------------
cfg.nOrifice        = 32;
cfg.dOrifice_um     = 40;
cfg.orificeArea_m2  = cfg.nOrifice * pi / 4 * (cfg.dOrifice_um * 1e-6)^2;   % 4.02e-8 m^2
cfg.tubeVolume_mL   = pi / 4 * 1.0^2 * 300 / 1000;      % 1 mm bore, 300 mm: 0.236 mL of air before the valve opens
cfg.CdNominal       = 0.7;

% --- Run limits (frictionRun opts.pMax, opts.xStop) ----------------------------
cfg.pressureCeiling = 12.0;     % MPa gauge, never exceeded
cfg.pressureStop    = 11.0;     % MPa gauge: pMax for the ramps and deliveries of this bench
cfg.posStopMargin   = 3.0;      % mm inside the slave's positive soft limit
cfg.exchangePeriod  = m.exchangePeriod;
cfg.dt              = m.dt;

% --- Slave settings from the GVL reference -------------------------------------
cfg.gvlFile = fullfile(root, 'slave-src', 'GVL', 'GlobalVariables_Reference.st');
g = readGvl(cfg.gvlFile);
cfg.gvl = g;
cfg.scanTime        = g.G_cfgScanTime;                   % s
cfg.filterTau       = g.G_cfgAnalogFilterTimeConst;      % s, pressure low-pass
cfg.medianSize      = g.G_cfgAnalogMedianSize;
cfg.pipelineScans   = 2;        % PRG_Main reads the filter output of two scans earlier
cfg.velAccel        = g.G_cfgDirectCtrlAccel;            % mm/s^2, Y_DirectControl in velocity mode
cfg.velDecel        = g.G_cfgDirectCtrlDecel;
cfg.velMax          = g.G_cfgVelLimitMax;                % mm/s, velocity mode clamp
cfg.pressureAccel   = g.G_cfgPressureAccel;
cfg.pressureDecel   = g.G_cfgPressureDecel;
cfg.pressureVelMax  = g.G_cfgPressureVelMax;
cfg.torqueLimit     = g.G_cfgTorqueLimitNormal;          % % of rated
cfg.pressureTorqueLimit = g.G_cfgPressureTorqueLimit;
cfg.softLimitMax    = g.G_cfgPosSoftLimitMax;
cfg.softLimitMin    = g.G_cfgPosSoftLimitMin;
cfg.posStop         = min(cfg.softLimitMax, m.posMax) - cfg.posStopMargin;
cfg.posMin          = max(cfg.softLimitMin, m.posMin) + 0.5;
end


function g = readGvl(file)
% Values of the G_cfg* declarations in the GVL reference. TIME literals are
% returned in seconds; numbers as doubles.
g = struct();
txt = fileread(file);
lines = strsplit(txt, char(10));
for k = 1:numel(lines)
    tok = regexp(lines{k}, '^\s*(G_cfg\w+)\s*:\s*(\w+)\s*:=\s*([^(]+?)\s*(\(\*.*)?$', 'tokens', 'once');
    if isempty(tok)
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
