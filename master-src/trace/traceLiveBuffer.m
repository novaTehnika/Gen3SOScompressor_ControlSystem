function b = traceLiveBuffer(cfg)
%TRACELIVEBUFFER Empty circular buffer of the live plot's history.
%   b = traceLiveBuffer(cfg) holds cfg.livePlotSeconds of the aligned
%   20 Hz stream in the eight columns the plot draws, so that it need not
%   copy the 44-column records:
%     data   cap x 8, cap = round(livePlotSeconds / exchangePeriod):
%            t_s [s], pAbs [atm abs] (the snapshot), pMeanAbs, pMinAbs,
%            pMaxAbs [atm abs] (the window summary), posFromEOT [mm],
%            velocity [mm/s], torque [% rated]
%     head   row the next record goes to
%     count  rows held, up to cap
%     lastSeq  record number of the newest row, 0 before the first
%     posEOT, atmPerMPa, atmAmbient  the conversions, from cfg
%     span   seconds of history shown (cfg.livePlotSeconds)
%   traceLiveAppend fills it and traceLiveUpdate draws it.

cap = round(cfg.livePlotSeconds / cfg.exchangePeriod);   % [s -> rows]
b = struct('data', NaN(cap, 8), 'head', 1, 'count', 0, 'lastSeq', 0, ...
    'posEOT', cfg.posEOT, 'atmPerMPa', cfg.atmPerMPa, ...
    'atmAmbient', cfg.atmAmbient, 'span', cfg.livePlotSeconds);
end
