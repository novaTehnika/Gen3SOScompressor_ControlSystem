function s = pressureNoiseStats(volts)
%PRESSURENOISESTATS One-second window statistics of the raw pressure input:
%   [mean rms min max] in volts of the samples of one window, the rms
%   about the mean (population form, sqrt(mean((x - mean(x))^2))). The
%   samples are a step probe capture's raw-input column (benchProbe, 0.2 mV
%   per count), one window being round(1 / scan) consecutive records.
%
%   s = pressureNoiseStats(volts)

volts = volts(:);
m = mean(volts);
s = [m, sqrt(mean((volts - m) .^ 2)), min(volts), max(volts)];
end
