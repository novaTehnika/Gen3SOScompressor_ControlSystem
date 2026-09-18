function sim = slaveSimConfig()
%SLAVESIMCONFIG Parameters of the simulated axis. The slave's own
%   configuration is not here: it is read from GlobalVariables_Reference.st
%   by tools/st2m.py and lives in sl.G (G_cfg*).
%
%   Physical axis coordinate x (mm): x = 0 is where the homing sequence
%   places the coordinate zero, HomeLimRetractDist positive of the negative
%   overtravel switch.

sim.switchNegPos   = -5.0;   % negative overtravel switch active at or below (mm)
sim.switchPosPos   = 310.0;  % positive overtravel switch active at or above (mm)
sim.hardStopMargin = 3.0;    % travel past either switch before the hard stop (mm)
sim.initialPos     = 40.0;   % x at power-up (mm)
sim.bootFrameError = 12.3;   % controller position minus x before homing (mm)
sim.powerOnDelay   = 0.1;    % MC_Power Enable -> Status (s)
sim.torqueAccel    = 0.2;    % TorqueVLMode acceleration per % torque (mm/s^2)
end
