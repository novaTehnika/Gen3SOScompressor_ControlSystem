function cfg = frictionBenchConfig()
%FRICTIONBENCHCONFIG Constants of the servo axis friction bench: the torque
%   to piston-force conversion, the speed points and their durations, the
%   limits the run enforces and the windows the fit uses.
%
%   Torque is reported by the slave in percent of the motor's rated torque
%   (IR15-16). The force the motor would exert at the piston through a
%   lossless gearbox and screw is
%
%       F [N] = T [%] / 100 * ratedTorqueNm * 2*pi * gearRatio / lead [m]
%
%   which is newtonPerPct per percent. Every force in this bench is that
%   ideal conversion. Drivetrain losses are not assumed; they are what the
%   loaded set measures, as a friction term proportional to the load.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'), fullfile(here, '..', 'common'));
master = masterConfig();

% Drivetrain, as configured on the slave (G_cfgGearRatio, G_cfgLeadScrewPitch).
cfg.ratedTorqueNm = 1.27;   % N m, SGM7J-04A rated torque; 100 % in IR15-16
cfg.gearRatio     = 70;     % motor turns per screw turn
cfg.leadMm        = 5.08;   % mm per screw turn
cfg.newtonPerPct  = cfg.ratedTorqueNm / 100 * 2 * pi * cfg.gearRatio / (cfg.leadMm / 1000);
cfg.pistonAreaM2  = pi / 4 * (master.boreDiameter / 1000)^2;   % m^2
cfg.etaAssumed    = 0.85;   % efficiency the design model's Fcap assumes, replaced by the measured value

% Speed points and the time at steady speed for each, in each direction.
% The slew to speed (G_cfgDirectCtrlAccel) is added to each step. The
% times give at least one screw lead of travel at every speed, so the
% torque ripple at the lead period averages out.
gvl = benchGvl();
cfg.accel        = gvl.G_cfgDirectCtrlAccel;   % mm/s^2, velocity-mode slew on the slave (GVL)
cfg.speeds       = [0.1 0.3 1 3 5];     % mm/s; 5 is G_cfgVelLimitMax and masterConfig.jogVelMax
cfg.tSteady      = [60 40 20 10 10];    % s at speed, unloaded
cfg.gasStrokeMax = 20;                  % mm per step under a gas charge: bounds the pressure excursion
cfg.repeats      = 2;                   % passes over the speeds: descending, then ascending

% Delivery set: pressure mode at the target while the outlet is open.
cfg.deliveryTarget  = 10;     % MPa gauge
cfg.deliverySeconds = 120;    % s in the mode unless xStop or pMax ends it

% Limits the run enforces. In velocity mode the slave limits only the
% torque (G_cfgTorqueLimitNormal, 100 %), which the stalled axis reaches at
% about 27 MPa; the pressure ceiling is the run's to keep.
cfg.pMax.unloaded = 0.3;      % MPa gauge: the chamber is vented for this set
cfg.pMax.gas      = 3.5;      % MPa gauge
cfg.pMax.delivery = 11.5;     % MPa gauge, under the 12 MPa mechanical ceiling
cfg.torqueAbort   = 90;       % % of rated, under the slave's 100 % limit
cfg.exitGuardPos  = 220;      % mm, G_cfgPosCylinderEntry + G_cfgPosExitGuardMargin

% Velocity steps for the drive lag (the 'stepv' step of frictionRun): the
% reference goes 0, v, 2v, v, 0, -v, -2v, -v, 0, each held stepHold
% seconds, with a scan-rate capture armed before every change.
cfg.stepHold = 3;         % s per level; the capture and its readout take about 2 s

% Scan-rate capture by PRG_StepProbe: registers, length and scales are
% shared with the other benches (bench/common/benchConfig.m).
cfg.probe = benchConfig().probe;

% Fit windows.
cfg.settle      = 1.0;    % s after the command reaches speed before samples count
cfg.trim        = 0.2;    % s discarded before the stop
cfg.vTolAbs     = 0.05;   % mm/s, velocity within this or vTolRel of the command counts as steady
cfg.vTolRel     = 0.1;
cfg.vMin        = 0.05;   % mm/s, below this the axis counts as at rest
cfg.aMax        = 0.5;    % mm/s^2, pressure-mode samples accelerating faster are dropped
cfg.binSeconds  = 1.0;    % s, pressure-mode samples are averaged over bins this long
cfg.stribeckVs  = [0.05 0.1 0.15 0.2 0.3 0.5 0.7 1 1.5 2];   % mm/s, knee speeds tried
end
