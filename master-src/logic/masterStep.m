function [mbWrite, doSafety, doMixer, position, velocity, pressure, ...
          statusCode, faultCode, state, linkOk] = masterStep( ...
          mbRead, pressureVolt, op, cmdSeq, targetPosition, ...
          targetVelocity, jogDirection, targetPressure, estop, mixer)
%MASTERSTEP Entry point for the Simulink shell's MATLAB Function block:
%   Modbus registers, DAQ pressure and app requests in; Modbus registers,
%   DAQ lines and displayed values out. Called once per model step (cfg.dt).
%   The Modbus blocks run every cfg.exchangePeriod and hold their values in
%   between.
%
%   mbRead        input registers IR1..IR18 as read by the Modbus Client
%                 Read block (modbusMap, raw 16-bit values as doubles)
%   pressureVolt  AI 2, pressure transducer (V)
%   op, cmdSeq, targetPosition, targetVelocity, jogDirection,
%   targetPressure, estop, mixer
%                 values of the Constant blocks the app writes
%
%   mbWrite       holding registers HR1..HR6 for the Modbus Client Write
%                 block (heartbeat, mode code, MotionEnable, FaultReset,
%                 reference as two registers)
%   doSafety      safety relay / STO enable (P0.0): 0 on E-stop or link loss
%   doMixer       stir bar motor (P0.4)
%   position (mm), velocity (mm/s), pressure (atm), statusCode, faultCode,
%   state, linkOk  for the app
%
%   Link supervision: until the slave scan counter has been seen to advance
%   the link is not up (STATUS_LINK_WAIT, safety relay off, E-stop path).
%   Once up, a counter that stops advancing for cfg.commTimeout means the
%   link is lost: the E-stop path is forced, the safety relay drops and
%   statusCode shows STATUS_LINK_LOST. That stays latched until the counter
%   advances again and the operator sends Stop.

persistent presBuf k heartbeat lastCounter tStale linkUp linkLost lastSeq ...
           integ pressureActive started

STATUS_LINK_LOST = 34;
STATUS_LINK_WAIT = 35;
OP_STOP = 0;

cfg = masterConfig();
map = modbusMap();
r = double(mbRead(:)');

if isempty(started)
    started = true;
    presBuf = pressureVolt * ones(1, cfg.pressureMedianLength);
    k = 0;
    heartbeat = 0;
    lastCounter = r(map.ir.slaveCounter);
    tStale = 0;
    linkUp = false;
    linkLost = false;
    lastSeq = cmdSeq;
    integ = 0;
    pressureActive = false;
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

% Median filter on the pressure input.
k = k + 1;
presBuf(mod(k, cfg.pressureMedianLength) + 1) = pressureVolt;
pressure = V2P(medianOf(presBuf), cfg);

% Slave status from the input registers.
faultActive = r(map.ir.faultActive) ~= 0;
position = modbusUnpack(r(map.ir.position), r(map.ir.position + 1));
velocity = modbusUnpack(r(map.ir.velocity), r(map.ir.velocity + 1));

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
in.pressureVelocity = 0;

% The pressure loop runs only while the pressure operation is active, as
% reported by the state machine on the previous step.
[in.pressureVelocity, integ] = pressureController(targetPressure, pressure, ...
    integ, pressureActive, cfg, cfg.dt);

out = masterStateMachine(in, cfg, cfg.dt);
pressureActive = out.statusCode == 5;

% Holding registers for the next exchange.
heartbeat = mod(heartbeat + 1, 65536);
mbWrite = zeros(1, map.nWrite);
mbWrite(map.hr.heartbeat)    = heartbeat;
mbWrite(map.hr.modeCode)     = out.modeCode;
mbWrite(map.hr.motionEnable) = out.motionEnable;
mbWrite(map.hr.faultReset)   = out.faultReset;
mbWrite(map.hr.reference:map.hr.reference + 1) = modbusPack(out.reference);

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
