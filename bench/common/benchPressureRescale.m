function P = benchPressureRescale(P, logShuntOhms)
%BENCHPRESSURERESCALE Logged pressure (MPa gauge) as the slave's present
%   scaling reads it. FB_PressureInput divides the shunt voltage by
%   G_cfgPressureShuntOhms, so a log recorded while that setting was
%   logShuntOhms holds a loop current scaled by logShuntOhms / the present
%   value. The voltage is recovered from the logged pressure and the loop
%   current recomputed with the present setting; the transducer range and
%   zero come from the GVL reference (benchGvl). An empty or equal
%   logShuntOhms returns P unchanged.

g = benchGvl();
if nargin < 2 || isempty(logShuntOhms) || logShuntOhms == g.G_cfgPressureShuntOhms
    return;
end
mAPerMPa = g.G_cfgPressureSpanMa / (g.G_cfgPressureRangeMax - g.G_cfgPressureRangeMin);
mA = g.G_cfgPressureZeroMa + (P - g.G_cfgPressureRangeMin) * mAPerMPa;
mA = mA * logShuntOhms / g.G_cfgPressureShuntOhms;
P = g.G_cfgPressureRangeMin + (mA - g.G_cfgPressureZeroMa) / mAPerMPa;
end
