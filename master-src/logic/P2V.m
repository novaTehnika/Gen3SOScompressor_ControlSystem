function V = P2V(P, cfg)
%P2V Shunt voltage for an absolute pressure (atm); inverse of V2P.

psi = (P - double(cfg.pressureIsGauge)) / cfg.atmPerPsi;
mA = 4 + 16 * (psi - cfg.pressureRangeMin) / ...
     (cfg.pressureRangeMax - cfg.pressureRangeMin);
V = mA / 1000 * cfg.pressureShuntOhms;
end
