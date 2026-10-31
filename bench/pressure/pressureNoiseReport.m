function r = pressureNoiseReport(volts, fs, stats, label)
%PRESSURENOISEREPORT Print the noise statistics of a transducer capture:
%   window statistics from the slave, the capture's rms, the rms in three
%   frequency bands and the strongest spectral lines.
%
%   r = pressureNoiseReport(volts, fs, stats, label)
%       volts   raw samples (V) at fs (Hz)
%       stats   rows of [mean rms min max] in volts from the slave's
%               one-second windows, or []
%   r = pressureNoiseReport('captures/<file>.csv')
%       re-analyses a saved capture (fs from its time column)
%
%   Scale: G_cfgPressureShuntOhms (benchGvl), the volts the input reads per
%   mA of loop current, and 4-20 mA over 0-16 MPa, so 1 MPa per mA.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'common'));
shuntOhm = benchGvl().G_cfgPressureShuntOhms;
mpaPerVolt = 1000 / shuntOhm;           % 1 MPa per mA
mV = 1000;

if ischar(volts)
    data = dlmread(volts, ',', 1, 0);
    label = volts;
    fs = 1 / (data(2, 1) - data(1, 1));
    volts = data(:, 2);
    stats = [];
end
volts = volts(:);
N = numel(volts);
r.label = label;
r.fs = fs;
r.mpaPerVolt = mpaPerVolt;
r.shuntOhm = shuntOhm;

fprintf('\n=== %s\n', label);
if ~isempty(stats)
    fprintf('one-second windows from the slave:\n');
    fprintf('%6s %10s %10s %10s %10s\n', 'window', 'mean V', 'MPa', 'rms mV', 'p-p mV');
    for k = 1:size(stats, 1)
        fprintf('%6d %10.4f %10.3f %10.2f %10.2f\n', k, stats(k, 1), ...
            (stats(k, 1) - 4 * shuntOhm / 1000) * mpaPerVolt, stats(k, 2) * mV, ...
            (stats(k, 4) - stats(k, 3)) * mV);
    end
    r.windowRmsMv = mean(stats(:, 2)) * mV;
end

% Capture statistics.
x = volts - mean(volts);
r.meanV = mean(volts);
r.rmsMv = std(x) * mV;
r.ppMv = (max(volts) - min(volts)) * mV;
fprintf('capture: %d samples at %.0f Hz, mean %.4f V (%.3f MPa), rms %.2f mV (%.4f MPa), p-p %.2f mV\n', ...
    N, fs, r.meanV, (r.meanV - 4 * shuntOhm / 1000) * mpaPerVolt, r.rmsMv, ...
    r.rmsMv / mV * mpaPerVolt, r.ppMv);

% Spectrum: Hann window, one-sided amplitude in mV.
n = (0:N - 1)';
w = 0.5 - 0.5 * cos(2 * pi * n / (N - 1));
X = fft(x .* w) / sum(w) * 2;
half = 1:floor(N / 2) + 1;
f = (half - 1)' * fs / N;
amp = abs(X(half)) * mV;
amp(1) = 0;
r.f = f;
r.ampMv = amp;

% Band rms from the periodogram (Parseval, window power corrected).
P = abs(fft(x .* w)).^2 / sum(w.^2) / N;
P = P(half);
P(2:end - 1) = 2 * P(2:end - 1);
bands = [0 25; 25 100; 100 fs / 2];
fprintf('rms by band:');
r.bandRmsMv = zeros(1, size(bands, 1));
for b = 1:size(bands, 1)
    sel = f > bands(b, 1) & f <= bands(b, 2);
    r.bandRmsMv(b) = sqrt(sum(P(sel))) * mV;
    fprintf('  %g-%g Hz %.2f mV', bands(b, 1), bands(b, 2), r.bandRmsMv(b));
end
fprintf('\n');

% Strongest lines, at least 4 Hz apart from each other.
[~, order] = sort(amp, 'descend');
peaks = [];
for k = order'
    if f(k) < fs / N * 2
        continue;
    end
    if isempty(peaks) || all(abs(f(peaks) - f(k)) >= 4)
        peaks(end + 1) = k; %#ok<AGROW>
    end
    if numel(peaks) == 5
        break;
    end
end
fprintf('strongest lines:');
for k = peaks
    fprintf('  %.0f Hz %.2f mV', f(k), amp(k));
end
fprintf('\n');
r.peakHz = f(peaks)';
r.peakMv = amp(peaks)';
end
