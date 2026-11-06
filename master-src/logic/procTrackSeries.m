function [raw, why, T] = procTrackSeries(st, values, cfg)
%PROCTRACKSERIES Sample a track step's signal at the exchange period.
%   [raw, why, T] = procTrackSeries(st, values, cfg)
%
%   st      a normalized track step (procNew): signal, duration (a number
%           or a parameter key)
%   values  struct, parameter key -> value, that the signal and a duration
%           key read
%   cfg     masterConfig (exchangePeriod, trackTableLength)
%
%   raw     column of N = floor(duration / exchangePeriod) + 1 samples at
%           t = (0 : N-1)' * exchangePeriod, without the step's offset;
%           [] when why is not empty
%   why     '' or what is wrong, phrased to follow "track signal "
%   T       the resolved duration in s (NaN when it cannot be resolved)
%
%   A function signal @(t, values) is called once with the column t and
%   must return N finite values; a result of another size makes it be
%   called once per sample. A [t value] matrix needs at least one row,
%   finite entries and strictly increasing t >= 0; it is linear between
%   its points, holds the first value before t(1) and the last value after
%   its end. The length is bounded by cfg.trackTableLength.

raw = [];
why = '';
T = NaN;
Tx = cfg.exchangePeriod;       % [s]

% ----- Duration -----
d = st.duration;
if ischar(d)
    if isfield(values, d)
        d = values.(d);
    else
        d = NaN;
    end
end
if ~(isnumeric(d) && isscalar(d) && isfinite(d) && d > 0)
    why = 'duration is not a number above 0';
    return
end
T = d;
N = floor(T / Tx + 1e-9) + 1;
if N > cfg.trackTableLength
    why = sprintf('is longer than the %g s a track may last', ...
        (cfg.trackTableLength - 1) * Tx);
    return
end
t = (0:N-1)' * Tx;             % [s]

% ----- Signal -----
sig = st.signal;
v = [];
if isa(sig, 'function_handle')
    try
        v = sig(t, values);
        if ~(isnumeric(v) && numel(v) == N)
            v = zeros(N, 1);
            for i = 1:N
                x = sig(t(i), values);
                if ~(isnumeric(x) && isscalar(x))
                    error('procTrackSeries:size', ...
                        'the signal does not return a number for a time');
                end
                v(i) = x;
            end
        end
    catch err
        why = ['could not be computed: ' err.message];
        return
    end
elseif isnumeric(sig) && ismatrix(sig) && size(sig, 2) == 2 && size(sig, 1) >= 1
    t0 = sig(:, 1);
    v0 = sig(:, 2);
    if ~(all(isfinite(sig(:))) && all(t0 >= 0) && all(diff(t0) > 0))
        why = 'matrix needs finite values and increasing times from 0';
        return
    end
    tq = min(max(t, t0(1)), t0(end));   % hold the ends
    if numel(t0) == 1
        v = v0 * ones(N, 1);
    else
        v = interp1(t0, v0, tq, 'linear');
    end
else
    why = 'must be a function of time or a [t value] matrix';
    return
end
v = double(v(:));
if numel(v) ~= N || ~all(isfinite(v))
    why = 'has values that are not finite numbers';
    return
end
raw = v;
end
