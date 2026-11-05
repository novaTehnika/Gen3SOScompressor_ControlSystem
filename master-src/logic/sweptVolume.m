function mL = sweptVolume(x, cfg)
%SWEPTVOLUME Saline still deliverable (mL) at position x (mm): the bore
%   swept from x to the end of travel, without the dead volume.

mL = mLPerMm(cfg) * (cfg.posEOT - x);
end
