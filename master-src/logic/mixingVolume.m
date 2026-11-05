function mL = mixingVolume(x, cfg)
%MIXINGVOLUME Volume taking part in mixing (mL) at position x (mm): the
%   swept volume plus the part of the dead volume that mixes
%   (cfg.mixingDeadVolume).

mL = sweptVolume(x, cfg) + cfg.mixingDeadVolume;
end
