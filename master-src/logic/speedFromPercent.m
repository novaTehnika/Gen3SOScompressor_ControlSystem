function v = speedFromPercent(pct, cfg)
%SPEEDFROMPERCENT Operator speed (mm/s) for a percentage of cfg.speedMax, limited to 0..100 %.
v = min(max(pct, 0), 100) / 100 * cfg.speedMax;
end
