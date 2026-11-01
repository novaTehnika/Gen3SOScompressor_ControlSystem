function cfg = charConfig()
%CHARCONFIG Constants for the pressure loop characterisation bench: the
%   chamber and outlet geometry the fits need, the pressure limits for the
%   runs and the slave settings read from the GVL reference (benchGvl).
%
%   Geometry comes from masterConfig (bore, end of travel, dead volume) so
%   the bench and the operator app agree.
%

here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..', '..');
addpath(fullfile(root, 'master-src', 'logic'), fullfile(here, '..', 'common'));
m = masterConfig();

% --- Chamber -----------------------------------------------------------------
cfg.boreDiameter_mm = m.boreDiameter;                    % 2.602 in
cfg.pistonArea_mm2  = pi / 4 * m.boreDiameter^2;         % 3431 mm^2 (3.431e-3 m^2)
cfg.posEOT          = m.posEOT;                          % mm, piston at the end of travel
cfg.deadVolume_mL   = m.deadVolume;                      % mL left at the end of travel (measured, masterConfig)
cfg.rho             = 1000;                              % kg/m^3 saline
cfg.Patm_MPa        = 0.101325;
% mL at atmosphere: air in the passage (about 2 cm) to the pressure
% transducer and in its cavity, which cannot be purged and dissolves only
% very slowly, so it is a fixed isothermal gas pocket in every run. It is
% params.py V_air (0.1e-6 m^3, an estimate from the geometry until the
% fits at two fill levels settle it), the same number; charRunSim checks it.
cfg.airVolume_mL    = 0.1;

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

% --- Slave settings from the GVL reference (bench/common/benchGvl) --------------
g = benchGvl();
cfg.gvlFile = g.file;
cfg.gvl = g;
cfg.scanTime        = g.G_cfgScanTime;                   % s
cfg.filterTau       = g.G_cfgAnalogFilterTimeConst;      % s, pressure low-pass
cfg.medianSize      = g.G_cfgAnalogMedianSize;
cfg.pipelineScans   = 2;        % PRG_Main reads the filter output of two scans earlier
cfg.velAccel        = g.G_cfgDirectCtrlAccel;            % mm/s^2, Y_DirectControl in velocity mode
cfg.velDecel        = g.G_cfgDirectCtrlAccel;            % mm/s^2, the block has one rate for both directions
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

