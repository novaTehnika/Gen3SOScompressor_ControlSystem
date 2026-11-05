function y = pressureConvert(value, from, to, cfg)
%PRESSURECONVERT Convert an absolute pressure between the display units.
%   y = pressureConvert(value, from, to)
%   y = pressureConvert(value, from, to, cfg)
%
%   value  scalar or array of absolute pressures in unit from
%   from, to  'atm', 'psi' or 'MPa'; all three are absolute (the gauge
%          reading plus one atmosphere), as the app shows pressure
%   cfg    masterConfig (psiPerAtm, atmPerMPa); default masterConfig()
%
%   An unknown unit raises 'pressureConvert:unit'. The conversion goes
%   through atm, which is what the app, the engine and masterStep hold.

if nargin < 4
    cfg = masterConfig();
end
atm = value;
switch from
    case 'atm'
    case 'psi', atm = value / cfg.psiPerAtm;
    case 'MPa', atm = value * cfg.atmPerMPa;
    otherwise, unitError(from);
end
switch to
    case 'atm', y = atm;
    case 'psi', y = atm * cfg.psiPerAtm;
    case 'MPa', y = atm / cfg.atmPerMPa;
    otherwise, unitError(to);
end
end

function unitError(unit)
error('pressureConvert:unit', 'unknown pressure unit "%s"', num2str(unit));
end
