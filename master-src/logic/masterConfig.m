function cfg = masterConfig()
%MASTERCONFIG Constants for the master state machine and signal scaling.

cfg.dt             = 0.025;     % model step (s)
cfg.exchangePeriod = 0.05;      % Modbus write + read period (s), a multiple of dt
cfg.commTimeout    = 0.3;       % slave counter unchanged this long = link lost (s): several exchanges, to ride out a slow one

% Cylinder geometry. Displayed positions are distances from the end of
% travel (posEOT) and volume is swept volume from there plus deadVolume.
cfg.posEOT       = 365;             % mm, position at the end of travel (compression end)
cfg.boreDiameter = 2.602 * 25.4;    % mm (2.602 in)
cfg.deadVolume   = 50;              % mL, volume left at the end of travel (measured: 50 g of water fills it)

% Pressure: the app works in atm absolute, the slave in MPa gauge.
cfg.atmPerMPa   = 9.869;
cfg.atmAmbient  = 1;        % atm, added to the slave's gauge reading
cfg.pressureMax = 15;       % MPa gauge, the slave's G_cfgPressureTargetMax

% Master soft limits, inside the slave's configured travel.
cfg.posMin = 0;             % mm
cfg.posMax = 367;           % mm

% Velocity command limit.
cfg.jogVelMax = 5;          % mm/s
cfg.jogSpeeds = [0.1 0.25 0.5 1 2 5];   % mm/s, app presets; the largest is the default

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
end
