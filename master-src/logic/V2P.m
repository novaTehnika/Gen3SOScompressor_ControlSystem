function P = V2P(V, cfg)
%V2P Pressure (atm) from transducer voltage.

P = (V - cfg.pressureOffsetV) * cfg.pressureGain * cfg.atmPerMPa;
end
