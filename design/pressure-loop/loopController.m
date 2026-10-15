function c = loopController(name, mode, Kp, Ki, Kd, stopLaw, amax)
% loopController - controller description for simPressureLoop.
%
%   mode     'V' closes the loop through the drive velocity loop: output is
%            a velocity command. 'T' closes it through the torque loop:
%            output is a force, with feedforward A*target always applied.
%   Kp Ki Kd 'V': mm/s per MPa, per s, per 1/s. 'T': N per MPa, per s,
%            per 1/s. Converted here to SI.
%   stopLaw  clamp |v| to the rate from which the slew-limited actuator can
%            still stop within the pressure error, using an online
%            stiffness estimate (same form as FB_PositionCommandGate)
%   amax     velocity slew to use in place of plantParams amax (m/s^2)
%
% Fields the caller may set afterwards:
%   c.vff    'V' only: add the expected valve flow / A as velocity
%            feedforward
%   c.tauF   sensor filter time constant override (s)
  if nargin < 6, stopLaw = false; end
  if nargin < 7, amax = NaN; end
  c.name = name; c.mode = mode; c.stopLaw = stopLaw; c.amax = amax;
  c.vff = false; c.tauF = NaN;
  if mode == 'V'
    c.Kp = Kp*1e-9; c.Ki = Ki*1e-9; c.Kd = Kd*1e-9;
  else
    c.Kp = Kp*1e-6; c.Ki = Ki*1e-6; c.Kd = Kd*1e-6;
  end
end
