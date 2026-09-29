function [v, integ] = pressureController(targetAtm, pressureAtm, integ, active, cfg, dt)
%PRESSURECONTROLLER PI pressure loop producing a velocity command (mm/s).
%   Positive velocity drives the piston into the cylinder and raises the
%   pressure. integ is the integral of the error (atm*s), held by the
%   caller. While not active the integrator is cleared and v is zero.

if ~active
    v = 0;
    integ = 0;
    return;
end

err = targetAtm - pressureAtm;
vRaw = cfg.pressureKp * err + cfg.pressureKi * integ;
v = min(max(vRaw, -cfg.pressureVelMax), cfg.pressureVelMax);

% Integrate only while unsaturated, or while the error drives the output
% back out of saturation.
if vRaw == v || (vRaw > v && err < 0) || (vRaw < v && err > 0)
    integ = integ + err * dt;
end
end
