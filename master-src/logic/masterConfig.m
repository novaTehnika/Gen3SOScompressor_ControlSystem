function cfg = masterConfig()
%MASTERCONFIG Constants for the master state machine and signal scaling.

% Analog position map shared by reference (AO) and feedback (AI):
% two linear segments joined at (posMapVtr, posMapXtr).
cfg.posMapVmin = -10;       % V
cfg.posMapVtr  = 5;         % V
cfg.posMapVmax = 10;        % V
cfg.posMapXmin = 0;         % mm
cfg.posMapXtr  = 200;       % mm
cfg.posMapXmax = 365;       % mm

% Velocity reference scaling (slave Velocity mode).
cfg.velPerVolt = 10;        % mm/s per V

% Pressure transducer: P = (V - offset) * gain.
cfg.pressureOffsetV = 4;        % V
cfg.pressureGain    = 2.3095;   % MPa per V
cfg.atmPerMPa       = 9.869;

% Master soft limits, inside the slave's configured travel.
cfg.posMin = 5;             % mm
cfg.posMax = 360;           % mm

% Velocity command limits.
cfg.jogVelMax      = 3;     % mm/s
cfg.pressureVelMax = 3;     % mm/s

% Position arrival judged from feedback.
cfg.posTolerance = 0.5;     % mm
cfg.tArrive      = 0.1;     % s inside tolerance before "at target"

% Handshake and sequencing times (s).
cfg.tInit          = 0.1;   % outputs held low after start
cfg.tConfirmStable = 0.02;  % confirm bits must match this long
cfg.tHandshake     = 0.5;   % give up waiting for confirmation
cfg.tEnableMin     = 0.05;  % minimum dwell after raising MotionEnable
cfg.tEnableMax     = 0.3;   % proceed without BrakeDisengage after this
cfg.tStopMin       = 0.1;   % ignore InMotion this long after dropping MotionEnable
cfg.tStopMax       = 3.0;   % give up waiting for InMotion low
cfg.tModeClear     = 0.15;  % mode 000 held before a new request (slave debounce + stable time)
cfg.tGoHomeDone    = 1.0;   % InMotion low this long ends Go Home when already homed

% Fault handling times (s).
cfg.tFaultSettle  = 0.02;   % after FaultActive before reading the code
cfg.tMirrorStable = 0.05;   % mirrored code stable before reset may be asserted
cfg.tResetMax     = 1.0;    % give up waiting for FaultActive low
end
