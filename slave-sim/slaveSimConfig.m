function sim = slaveSimConfig()
%SLAVESIMCONFIG Parameters of the simulated axis. The slave's own
%   configuration is not here: it is read from GlobalVariables_Reference.st
%   by tools/st2m.py and lives in sl.G (G_cfg*).
%
%   Physical axis coordinate x (mm): x = 0 is where the homing sequence
%   places the coordinate zero, HomeLimRetractDist positive of the negative
%   overtravel switch.

sim.switchNegPos   = -5.0;   % negative overtravel switch active at or below (mm)
sim.switchPosPos   = 370.0;  % positive overtravel switch active at or above (mm), past the 367 mm soft limit
sim.hardStopMargin = 3.0;    % travel past either switch before the hard stop (mm)
sim.initialPos     = 40.0;   % x at power-up (mm)
sim.bootFrameError = 0.0;    % controller position minus x at power-up (mm); 0 = retained absolute frame
sim.powerOnDelay   = 0.1;    % MC_Power Enable -> Status (s)
sim.torqueAccel    = 0.2;    % TorqueVLMode acceleration per % torque (mm/s^2)

% Chamber: a gas spring. Past chamberContactPos the piston compresses an
% isothermal gas volume that starts at chamberGasVolume at atmosphere, plus
% whatever liquid hook.outflow has removed. Pressure is reported in MPa
% gauge and read back through the transducer model below.
sim.chamberContactPos = 200.0;   % x where compression starts (mm)
sim.chamberGasVolume  = 200e3;   % gas volume at atmosphere (mm^3)
sim.chamberMinVolume  = 1e3;     % volume floor (mm^3): pressure stops rising there
sim.pistonArea        = 3431;    % mm^2
sim.atmMPa            = 0.101325;

% Transducer and shunt as wired: 4-20 mA over sensorRangeMPa, read as the
% voltage across the shunt on the controller's analog input.
sim.sensorZeroMa   = 4;
sim.sensorSpanMa   = 16;
sim.sensorRangeMPa = 16;
sim.sensorShuntOhm = 434;
end
