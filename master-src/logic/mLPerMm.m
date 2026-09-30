function k = mLPerMm(cfg)
%MLPERMM Volume swept per mm of piston travel (mL/mm).

k = pi / 4 * cfg.boreDiameter^2 / 1000;
end
