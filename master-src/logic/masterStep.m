function [mbWrite, doSafety, doMixer, position, velocity, pressure, ...
          statusCode, faultCode, state, linkOk] = masterStep( ...
          mbRead, op, cmdSeq, targetPosition, ...
          targetVelocity, jogDirection, targetPressure, estop, mixer, ...
          positionSpeed, salineVolume, oxygenRatio)
%MASTERSTEP Entry point for the Simulink shell's MATLAB Function block:
%   Modbus registers and app requests in; Modbus registers, DAQ lines and
%   displayed values out. Called once per model step (cfg.dt). The Modbus
%   blocks run every cfg.exchangePeriod and hold their values in between.
%
%   mbRead        input registers IR1..IR20 as read by the Modbus Client
%                 Read block (modbusMap, raw 16-bit values as doubles)
%   op, cmdSeq, targetPosition, targetVelocity, jogDirection,
%   targetPressure, estop, mixer, positionSpeed
%                 values of the Constant blocks the app writes;
%                 targetPressure in atm absolute, as the app shows pressure;
%                 positionSpeed in mm/s, the speed of a position move
%   salineVolume, oxygenRatio
%                 the fill description for the slave's pressure gain
%                 schedule: saline volume (mL) and oxygen charge as STP
%                 volume per saline volume; cfg.salineVolume and
%                 cfg.oxygenRatio when omitted
%
%   mbWrite       holding registers HR1..HR12 for the Modbus Client Write
%                 block (heartbeat, mode code, MotionEnable, FaultReset,
%                 reference, position speed, saline volume and oxygen
%                 ratio, each REAL as two registers)
%   doSafety      safety relay / STO enable (P0.0): 0 on E-stop or link loss
%   doMixer       stir bar motor (P0.4)
%   position (mm), velocity (mm/s), pressure (atm absolute: the slave's
%   gauge reading plus cfg.atmAmbient), statusCode, faultCode, state, linkOk  for the app
%
%   Link supervision: until the slave scan counter has been seen to advance
%   the link is not up (STATUS_LINK_WAIT, safety relay off, E-stop path).
%   Once up, a counter that stops advancing for cfg.commTimeout means the
%   link is lost: the E-stop path is forced, the safety relay drops and
%   statusCode shows STATUS_LINK_LOST. That stays latched until the counter
%   advances again and the operator sends Stop.

persistent heartbeat lastCounter tStale linkUp linkLost lastSeq started

STATUS_LINK_LOST = 34;
STATUS_LINK_WAIT = 35;
OP_STOP = 0;

cfg = masterConfig();
map = modbusMap();
r = double(mbRead(:)');
if nargin < 11
    salineVolume = cfg.salineVolume;
end
if nargin < 12
    oxygenRatio = cfg.oxygenRatio;
end

if isempty(started)
    started = true;
    heartbeat = 0;
    lastCounter = r(map.ir.slaveCounter);
    tStale = 0;
    linkUp = false;
    linkLost = false;
    lastSeq = cmdSeq;
end

% Link watchdog on the slave scan counter.
counter = r(map.ir.slaveCounter);
if counter ~= lastCounter
    tStale = 0;
    lastCounter = counter;
    linkUp = true;
elseif linkUp
    tStale = tStale + cfg.dt;
end
if tStale >= cfg.commTimeout
    linkLost = true;
end
newCmd = cmdSeq ~= lastSeq;
lastSeq = cmdSeq;
if linkLost && tStale < cfg.commTimeout && newCmd && op == OP_STOP
    linkLost = false;
end

% Slave status from the input registers.
faultActive = r(map.ir.faultActive) ~= 0;
position = modbusUnpack(r(map.ir.position), r(map.ir.position + 1));
velocity = modbusUnpack(r(map.ir.velocity), r(map.ir.velocity + 1));
pressure = modbusUnpack(r(map.ir.pressure), r(map.ir.pressure + 1)) * cfg.atmPerMPa ...
    + cfg.atmAmbient;

in.reset            = 0;
if faultActive
    in.confCode     = r(map.ir.faultCode);
else
    in.confCode     = r(map.ir.confCode);
end
in.faultActive      = double(faultActive);
in.homingComplete   = double(r(map.ir.homingComplete) ~= 0);
in.inMotion         = double(r(map.ir.inMotion) ~= 0);
in.brakeDisengage   = 0;    % not carried by the link
in.position         = position;
in.estop            = double(estop ~= 0 || linkLost || ~linkUp);
in.op               = op;
in.cmdSeq           = cmdSeq;
in.targetPosition   = targetPosition;
in.jogVelocity      = abs(targetVelocity) * sign(jogDirection);
in.targetPressure   = (targetPressure - cfg.atmAmbient) / cfg.atmPerMPa;
in.positionSpeed    = positionSpeed;

out = masterStateMachine(in, cfg, cfg.dt);

% Holding registers for the next exchange.
heartbeat = mod(heartbeat + 1, 65536);
mbWrite = zeros(1, map.nWrite);
mbWrite(map.hr.heartbeat)    = heartbeat;
mbWrite(map.hr.modeCode)     = out.modeCode;
mbWrite(map.hr.motionEnable) = out.motionEnable;
mbWrite(map.hr.faultReset)   = out.faultReset;
mbWrite(map.hr.reference:map.hr.reference + 1) = modbusPack(out.reference);
mbWrite(map.hr.positionSpeed:map.hr.positionSpeed + 1) = modbusPack(out.positionSpeed);
mbWrite(map.hr.salineVolume:map.hr.salineVolume + 1) = modbusPack(salineVolume);
mbWrite(map.hr.oxygenRatio:map.hr.oxygenRatio + 1) = modbusPack(oxygenRatio);

doSafety = double(in.estop == 0);
doMixer  = double(mixer ~= 0 && in.estop == 0);

statusCode = out.statusCode;
if linkLost
    statusCode = STATUS_LINK_LOST;
elseif ~linkUp
    statusCode = STATUS_LINK_WAIT;
end
faultCode = out.faultCode;
state     = out.state;
linkOk    = double(linkUp && ~linkLost);
end
