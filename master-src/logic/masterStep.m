function [doSlave, doMixer, refVoltage, position, velocity, pressure, ...
          statusCode, faultCode, homed, state] = masterStep( ...
          diSlave, positionVolt, pressureVolt, op, cmdSeq, targetPosition, ...
          targetVelocity, jogDirection, targetPressure, estop, mixer)
%MASTERSTEP Entry point for the Simulink shell's MATLAB Function block:
%   DAQ signals and app requests in, DAQ signals and displayed values out.
%   Called once per model step (cfg.dt).
%
%   diSlave       raw DAQ digital inputs, in the order
%                 [conf0 conf1 conf2 performanceStatus faultActive
%                  homingComplete inMotion]. The slave outputs sink the DAQ
%                 lines, so a raw 0 is an asserted signal.
%   positionVolt  AI 0, position feedback (V)
%   pressureVolt  AI 2, pressure transducer (V)
%   op, cmdSeq, targetPosition, targetVelocity, jogDirection,
%   targetPressure, estop, mixer
%                 values of the Constant blocks the app writes
%
%   doSlave       DAQ digital outputs, in the order
%                 [safetyEnable modeBit1 motionEnable modeBit0 modeBit2 faultReset]
%                 (lines P0.0 P0.1 P0.2 P0.5 P0.6 P0.7)
%   doMixer       stir bar motor (P0.4)
%   refVoltage    AO reference to the slave (V)
%   position (mm), velocity (mm/s), pressure (atm), statusCode, faultCode,
%   homed, state  for the app

persistent posBuf presBuf k xPrev vFilt integ pressureActive started

cfg = masterConfig();

if isempty(started)
    started = true;
    posBuf = positionVolt * ones(1, cfg.posMedianLength);
    presBuf = pressureVolt * ones(1, cfg.pressureMedianLength);
    k = 0;
    xPrev = V2x(positionVolt, cfg);
    vFilt = 0;
    integ = 0;
    pressureActive = false;
end

% Median filters on the two analog inputs.
k = k + 1;
posBuf(mod(k, cfg.posMedianLength) + 1) = positionVolt;
presBuf(mod(k, cfg.pressureMedianLength) + 1) = pressureVolt;
position = V2x(medianOf(posBuf), cfg);
pressure = V2P(medianOf(presBuf), cfg);

% Velocity for display: differentiated position through a first-order lag.
alpha = cfg.dt / (cfg.velFilterTau + cfg.dt);
vFilt = vFilt + alpha * ((position - xPrev) / cfg.dt - vFilt);
xPrev = position;
velocity = vFilt;

in.reset            = 0;
in.confCode         = bitsToCode(diSlave(1) == 0, diSlave(2) == 0, diSlave(3) == 0);
in.faultActive      = double(diSlave(5) == 0);
in.homingComplete   = double(diSlave(6) == 0);
in.inMotion         = double(diSlave(7) == 0);
in.brakeDisengage   = 0;    % not wired to the master
in.position         = position;
in.estop            = double(estop ~= 0);
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

[bit0, bit1, bit2] = modeBits(out.modeCode);
safetyEnable = double(in.estop == 0);
doSlave = [safetyEnable, bit1, out.motionEnable, bit0, bit2, out.faultReset];
doMixer = double(mixer ~= 0 && in.estop == 0);
refVoltage = min(max(out.refVoltage, -10), 10);

statusCode = out.statusCode;
faultCode  = out.faultCode;
homed      = out.homed;
state      = out.state;
end
