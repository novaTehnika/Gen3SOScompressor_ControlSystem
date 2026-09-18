function x = V2x(V, cfg)
%V2X Position (mm) from analog voltage using the two-segment position map.

V = min(max(V, cfg.posMapVmin), cfg.posMapVmax);

if V <= cfg.posMapVtr
    x = (V - cfg.posMapVmin) * (cfg.posMapXtr - cfg.posMapXmin) ...
        / (cfg.posMapVtr - cfg.posMapVmin) + cfg.posMapXmin;
else
    x = (V - cfg.posMapVtr) * (cfg.posMapXmax - cfg.posMapXtr) ...
        / (cfg.posMapVmax - cfg.posMapVtr) + cfg.posMapXtr;
end
end
