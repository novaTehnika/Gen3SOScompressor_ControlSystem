function cfg = masterConfig()
%MASTERCONFIG Constants for the master state machine and signal scaling.

cfg.dt             = 0.025;     % model step (s)
cfg.exchangePeriod = 0.05;      % Modbus write + read period (s), a multiple of dt
cfg.commTimeout    = 0.3;       % slave counter unchanged this long = link lost (s): several exchanges, to ride out a slow one

% Cylinder geometry. Displayed positions are distances from the end of
% travel (posEOT); the displayed volume is one of three views: swept
% (from there, without the dead volume), mixing (swept plus
% mixingDeadVolume) and since zero (swept volume change since the
% operator zeroed it).
cfg.posEOT       = 367;             % mm, position at the end of travel (compression end)
cfg.boreDiameter = 2.602 * 25.4;    % mm (2.602 in)
cfg.deadVolume   = 57;              % mL, volume left at the end of travel (measured: 57 g of water fills it, with the present fittings)
cfg.mixingDeadVolume = 57;          % mL, the part of deadVolume that takes part in mixing (at most deadVolume); added to the swept volume in the mixing view

% Operator app display.
cfg.flowDeadband  = 0.01;           % mL/s, a flow smaller than this reads 0.00
cfg.displayPeriod = 0.1;            % s, the display timer period

% Pressure: the app works in atm absolute, the slave in MPa gauge.
cfg.atmPerMPa   = 9.869;
cfg.atmAmbient  = 1;        % atm, added to the slave's gauge reading
cfg.pressureMax = 15;       % MPa gauge, the slave's G_cfgPressureTargetMax
cfg.pressureCeiling     = 12.0;   % MPa gauge, the slave's G_cfgPressureCeiling: above it pressurising motion is refused (keep equal to the GVL)
cfg.pressureCeilingHyst = 0.5;    % MPa, the slave's G_cfgPressureCeilingHyst (keep equal to the GVL)
cfg.psiPerAtm   = 14.696;   % psi per atm, for the pressure display units

% Fill description sent to the slave for its pressure gain schedule. The
% saline volume and the charge must be at or below the true values (see
% docs/slave/development/ConfigurationGuide.md, section 8).
cfg.salineVolume = 343;     % mL, saline volume of the fill including the dead volume
cfg.oxygenRatio  = 1.0;     % oxygen charge as STP volume per saline volume (1.0 for 1:1)

% Master soft limits: G_cfgPosSoftLimitMin <= posMin and
% posMax <= G_cfgPosSoftLimitMax.
cfg.posMin = 0;             % mm
cfg.posMax = 367;           % mm

% Operator speed: one setting, as a percentage of speedMax, for jog,
% position moves, Go Home and homing. It is sent as the jog reference and
% in HR7-8 every exchange; the slave caps each mode by its own ceiling.
cfg.speedMax     = 6;               % mm/s at 100 %; must not exceed G_cfgVelLimitMax
cfg.speedPresets = [2 5 10 25 50 100];   % % of speedMax, the app's preset buttons
cfg.speedDefault = 25;             % % at app start and the shell's default

% Position arrival judged from feedback.
cfg.posTolerance = 0.5;     % mm
cfg.tArrive      = 0.1;     % s inside tolerance before "at target"

% Handshake and sequencing times (s).
cfg.tInit          = 0.1;   % outputs held low after start
cfg.tConfirmStable = 0.02;  % confirm code must match this long
cfg.tHandshake     = 0.5;   % give up waiting for confirmation
cfg.tEnableMin     = 0.05;  % minimum dwell after raising MotionEnable
cfg.tEnableMax     = 1.0;   % proceed without BrakeDisengage after this (slave reset + power-on + brake release)
cfg.tStopMin       = 0.1;   % ignore InMotion this long after dropping MotionEnable
cfg.tStopMax       = 3.0;   % give up waiting for InMotion low
cfg.tModeClear     = 0.15;  % mode 0 held before a new request (slave debounce + stable time)
cfg.tGoHomeDone    = 1.0;   % InMotion low this long, after motion, ends Go Home
cfg.tGoHomeStartMax = 5.0;  % Go Home with no motion by then ends (axis already home)

% Fault handling times (s).
cfg.tFaultSettle  = 0.02;   % after FaultActive before reading the code
cfg.tMirrorStable = 0.05;   % mirrored code stable before reset may be asserted
cfg.tResetMax     = 1.0;    % give up waiting for FaultActive low

% Operator app: operation tracking and the procedure engine.
cfg.opConfirmWait    = 0.5;  % s a request has before a status that does not show it running ends it
cfg.procedure        = 'sopProcedure';  % function in master-src/procedure returning the procedure
cfg.procStatusDelay  = 0.1;  % s after a request before a status counts toward completion (> model step + exchange)
cfg.procPressureTol  = 0.2;  % atm, default band for "settled" pressure
cfg.procSettleTime   = 2.0;  % s inside the band for "settled", default
cfg.procEndVolume    = 5;    % mL swept: at or below this an alsoWhen 'endOfTravel' step ends (above the slave's end margin of 1.0 mm, 3.4 mL)
cfg.procMaxChecks    = 6;    % checklist items per step (the panel's checkbox pool)
cfg.procMaxParams    = 8;    % parameters per procedure (the panel's field pool)
end
