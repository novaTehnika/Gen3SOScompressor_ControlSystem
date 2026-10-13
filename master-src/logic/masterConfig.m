function cfg = masterConfig()
%MASTERCONFIG Constants for the master state machine and signal scaling.

cfg.dt             = 0.025;     % model step (s)
cfg.exchangePeriod = 0.05;      % Modbus write + read period (s), a multiple of dt
cfg.commTimeout    = 0.15;      % slave counter unchanged this long = link lost (s)

% Cylinder geometry. Displayed positions are distances from the end of
% travel (posEOT) and volume is swept volume from there plus deadVolume.
cfg.posEOT       = 365;             % mm, position at the end of travel (compression end)
cfg.boreDiameter = 2.602 * 25.4;    % mm (2.602 in)
cfg.deadVolume   = 10;              % mL, volume left at the end of travel

% Pressure transducer: a 4-20 mA loop read on AI 2 as the voltage across a
% shunt resistor. 4 mA is pressureRangeMin and 20 mA pressureRangeMax. A
% gauge transducer reads 0 at atmosphere, so one atmosphere is added to give
% absolute pressure.
cfg.pressureShuntOhms = 250;    % ohm
cfg.pressureRangeMin  = 0;      % psi at 4 mA
cfg.pressureRangeMax  = 1500;   % psi at 20 mA
cfg.pressureIsGauge   = true;   % false for an absolute transducer
cfg.atmPerPsi         = 1 / 14.6959;

% Input filtering.
cfg.pressureMedianLength = 3;

% Pressure loop: velocity command per unit pressure error.
cfg.pressureKp = 0.02533;   % mm/s per atm (0.25 mm/s per MPa)
cfg.pressureKi = 0;         % mm/s per atm*s

% Master soft limits, inside the slave's configured travel.
cfg.posMin = 5;             % mm
cfg.posMax = 360;           % mm

% Velocity command limits.
cfg.jogVelMax      = 5;     % mm/s
cfg.jogSpeeds      = [0.1 0.25 0.5 1 2 5];  % mm/s, app presets; the largest is the default
cfg.pressureVelMax = 5;     % mm/s

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
cfg.tGoHomeDone    = 1.0;   % InMotion low this long ends Go Home

% Fault handling times (s).
cfg.tFaultSettle  = 0.02;   % after FaultActive before reading the code
cfg.tMirrorStable = 0.05;   % mirrored code stable before reset may be asserted
cfg.tResetMax     = 1.0;    % give up waiting for FaultActive low
end
