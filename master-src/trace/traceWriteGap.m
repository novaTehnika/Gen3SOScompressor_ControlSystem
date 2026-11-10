function g = traceWriteGap(rows, cfg)
%TRACEWRITEGAP Longest time between two master writes in trace records.
%   g = traceWriteGap(rows, cfg) is the longest window of the slave's
%   exchange summary among the k x traceLayout().nCol records rows: its
%   scan count (summaryScans) times cfg.slaveScanPeriod, in s. It is 0
%   when no row carries a window (summaryScans 0).
%
%   A window runs from one change of the master's heartbeat to the next,
%   so its length is what the slave's link watchdog measures
%   (G_cfgCommTimeout). A model that stops stepping because MATLAB's
%   thread is busy lengthens the window that is open at the time. The
%   rows may be raw or aligned (traceAlign); only the longest matters.

L = traceLayout();
if isempty(rows)
    g = 0;
    return
end
g = max([0; double(rows(:, L.col.summaryScans))]) * cfg.slaveScanPeriod;   % [scans -> s]
end
