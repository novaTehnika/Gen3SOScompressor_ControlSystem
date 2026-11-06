function why = procTrackCheck(v, mode, relative, cfg, speed)
%PROCTRACKCHECK Check a track series against the machine's limits.
%   why = procTrackCheck(v, mode, relative, cfg, speed)
%
%   v         column series, one value per exchange: atm absolute for mode
%             'pressure', mm from the end of travel for mode 'position'
%   mode      'pressure' or 'position'
%   relative  true: v is added to a reading later, so only its span is
%             checked against the width of the allowed range; false: v is
%             absolute and every value is checked against the range
%   cfg       masterConfig (exchangePeriod, atmAmbient, pressureMax,
%             atmPerMPa, posEOT, posMin, posMax)
%   speed     the fastest the series may change: mm/s for a position,
%             atm/s for a pressure; Inf is not checked
%
%   why       '' when the series passes, else a message without numbers or
%             units, phrased to follow "track signal ". The range is
%             atmAmbient to pressureMax (MPa gauge) + atmAmbient for a
%             pressure, and max(posEOT - posMax, 0) to posEOT - posMin for
%             a position. The change per exchange is bounded by speed
%             (1e-9 relative slack).

why = '';
Tx = cfg.exchangePeriod;       % [s]
if strcmp(mode, 'pressure')
    lo = cfg.atmAmbient;                                      % [atm]
    hi = cfg.pressureMax * cfg.atmPerMPa + cfg.atmAmbient;    % [MPa -> atm]
    name = 'pressure';
else
    lo = max(cfg.posEOT - cfg.posMax, 0);                     % [mm]
    hi = cfg.posEOT - cfg.posMin;                             % [mm]
    name = 'position';
end
if relative
    if max(v) - min(v) > hi - lo
        why = sprintf('wider than the %s range', name);
        return
    end
elseif min(v) < lo || max(v) > hi
    why = sprintf('outside the %s limits', name);
    return
end
if numel(v) > 1 && isfinite(speed)
    if max(abs(diff(v))) / Tx > speed * (1 + 1e-9)
        if strcmp(mode, 'pressure')
            why = 'faster than the pressure slope limit';
        else
            why = 'faster than the axis speed';
        end
    end
end
end
