function cfg = masterConfig()
%MASTERCONFIG Constants for the master state machine and signal scaling.

cfg.dt             = 0.025;     % model step (s)
cfg.exchangePeriod = 0.05;      % Modbus write + read period (s), a multiple of dt
cfg.commTimeout    = 0.3;       % slave counter unchanged this long = link lost (s): several exchanges, to ride out a slow one

% Pressure transducer: P = (V - offset) * gain.
cfg.pressureOffsetV = 4;        % V
cfg.pressureGain    = 2.3095;   % MPa per V
cfg.atmPerMPa       = 9.869;

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
