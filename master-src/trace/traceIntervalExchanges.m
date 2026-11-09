function n = traceIntervalExchanges(interval, cfg)
%TRACEINTERVALEXCHANGES Whole exchanges in a record interval.
%   n = traceIntervalExchanges(interval, cfg) is
%   max(1, round(max(interval, cfg.exchangePeriod) / cfg.exchangePeriod)):
%   an interval below the exchange period is raised to it and any other
%   is rounded to whole exchanges. interval is in s.

n = max(1, round(max(interval, cfg.exchangePeriod) / cfg.exchangePeriod));
end
