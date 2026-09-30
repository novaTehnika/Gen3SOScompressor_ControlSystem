function P = V2P(V, cfg)
%V2P Absolute pressure (atm) from the voltage across the transducer's shunt.

mA = V / cfg.pressureShuntOhms * 1000;
psi = cfg.pressureRangeMin + (mA - 4) / 16 * ...
      (cfg.pressureRangeMax - cfg.pressureRangeMin);
P = psi * cfg.atmPerPsi + double(cfg.pressureIsGauge);
end
