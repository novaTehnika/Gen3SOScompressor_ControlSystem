function info = pressureUnitInfo(unit, cfg)
%PRESSUREUNITINFO How a pressure unit is shown and converted.
%   info = pressureUnitInfo(unit)
%   info = pressureUnitInfo(unit, cfg)
%
%   unit  'psi', 'atm' or 'MPa' (all absolute); anything else raises
%         'pressureConvert:unit'
%   cfg   masterConfig; default masterConfig()
%
%   info fields:
%     name      the unit
%     label     'psi abs', 'atm abs' or 'MPa abs'
%     decimals  digits after the point in a readout
%     format    printf format with those digits
%     step      spinner step in the unit
%     toUnit    @(atm) value in the unit
%     fromUnit  @(x) atm absolute from a value in the unit
%
%   This is the display options struct that procView takes.

if nargin < 2
    cfg = masterConfig();
end
switch unit
    case 'psi', decimals = 1; step = 5;
    case 'atm', decimals = 2; step = 0.5;
    case 'MPa', decimals = 3; step = 0.05;
    otherwise
        error('pressureConvert:unit', 'unknown pressure unit "%s"', ...
            num2str(unit));
end
info.name = unit;
info.label = [unit ' abs'];
info.decimals = decimals;
info.format = sprintf('%%.%df', decimals);
info.step = step;
info.toUnit = @(atm) pressureConvert(atm, 'atm', unit, cfg);
info.fromUnit = @(x) pressureConvert(x, unit, 'atm', cfg);
end
