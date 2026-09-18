function V = x2V(x, cfg)
%X2V Analog voltage from position (mm); inverse of V2x.

x = min(max(x, cfg.posMapXmin), cfg.posMapXmax);

if x <= cfg.posMapXtr
    V = (x - cfg.posMapXmin) * (cfg.posMapVtr - cfg.posMapVmin) ...
        / (cfg.posMapXtr - cfg.posMapXmin) + cfg.posMapVmin;
else
    V = (x - cfg.posMapXtr) * (cfg.posMapVmax - cfg.posMapVtr) ...
        / (cfg.posMapXmax - cfg.posMapXtr) + cfg.posMapVtr;
end
end
