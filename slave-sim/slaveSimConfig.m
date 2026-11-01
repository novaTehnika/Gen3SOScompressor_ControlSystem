function sim = slaveSimConfig()
%SLAVESIMCONFIG Parameters of the simulated axis. The slave's own
%   configuration is not here: it is read from GlobalVariables_Reference.st
%   by tools/st_translate.py and lives in sl.G (G_cfg*).
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

% Chamber, outlet and drive: the design model's plant (design/pressure-loop,
% ported to models/c/plant.c), with its parameters and contents at a
% run's start (models/plant_export.py). Its piston coordinate is zero at
% plantStartPos, the axis position of Params.Vl0's fill; before
% chamberContactPos the piston is out of the cylinder, which is open, and
% the pressure reads zero.
sim.chamberContactPos = 200.0;   % x where the piston enters the cylinder (mm)
sim.plantStartPos     = 280.0;   % x of the plant's x = 0, a run's start position (mm)

% Transducer and shunt as wired: 4-20 mA over sensorRangeMPa, read as the
% voltage across the shunt on the controller's analog input. The shunt is
% the effective resistance the input reads: the 434 ohm resistor in
% parallel with the input's own load (bench/pressure/README.md section 6).
sim.sensorZeroMa   = 4;
sim.sensorSpanMa   = 16;
sim.sensorRangeMPa = 16;
sim.sensorShuntOhm = 421.4;
end
