function mL = x2mL(x, cfg)
%X2ML Chamber volume (mL) at position x (mm): the bore swept from x to the
%   end of travel, plus the dead volume.

mL = mLPerMm(cfg) * (cfg.posEOT - x) + cfg.deadVolume;
end
