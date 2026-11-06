function d = traceDerive(d)
%TRACEDERIVE Add the displayed quantities to a trace read by traceReadCsv.
%
%   d = traceDerive(d)
%
%   The geometry and pressure scaling come from d.meta (posEOT_mm,
%   boreDiameter_mm, mixingDeadVolume_mL, atmPerMPa, atmAmbient_atm), the
%   values in force when the trace was recorded; a key the file lacks
%   falls back to masterConfig(). A file without the procStep and
%   nExchanges columns gets 0 and 1. Added columns:
%     posFromEOT_mm        posEOT - position, distance to the end of travel
%     commandedFromEOT_mm  the same for the slave's commanded position
%     targetFromEOT_mm     the same for the app's target position
%     sweptVolume_mL       saline still deliverable at the position
%                          (sweptVolume, without the dead volume)
%     mixingVolume_mL      swept volume plus the dead volume that takes
%                          part in mixing (mixingVolume)
%     flow_mL_s            mL/mm * velocity, positive toward the end of
%                          travel as the app shows it
%     pressureAbs_atm      gauge MPa * atmPerMPa + atmAmbient
%     pressureMeanAbs_atm, pressureMinAbs_atm, pressureMaxAbs_atm
%                          mean, minimum and maximum of the slave's
%                          pressure over the scans of the row's window,
%                          the same conversion as pressureAbs_atm.
%                          Format 3 files carry the summary on the row of
%                          its window (traceAlign): NaN where
%                          summaryScans is 0. In a format 2 file the
%                          slave's summary arrives two records late, so
%                          row k takes the summary of record k+2 (NaN
%                          when record k+2 is missing, when seq(k+2) ~=
%                          seq(k)+2 and when its summaryScans is 0); in
%                          a format 1 file all are NaN
%     posMeanFromEOT_mm    posEOT - positionMean, over the same window;
%                          NaN where summaryScans is 0 and in files
%                          before format 3
%     interval_s           nExchanges * exchange period: the time the
%                          row covers
%     tStartRel_s          tRel_s - (nExchanges-1) * exchange period,
%                          where the row's interval begins
%     decimated            scalar, true when any row combines more than
%                          one exchange
%     gap                  true at the first record after a seq jump
%                          larger than the row's nExchanges
%     tRel_s               t_s from the first record

cfg = masterConfig();
meta = struct();
if isfield(d, 'meta') && isstruct(d.meta)
    meta = d.meta;
end
cfg = fromMeta(cfg, meta, 'posEOT', 'posEOT_mm');
cfg = fromMeta(cfg, meta, 'boreDiameter', 'boreDiameter_mm');
cfg = fromMeta(cfg, meta, 'mixingDeadVolume', 'mixingDeadVolume_mL');
cfg = fromMeta(cfg, meta, 'atmPerMPa', 'atmPerMPa');
cfg = fromMeta(cfg, meta, 'atmAmbient', 'atmAmbient_atm');

d.posFromEOT_mm       = cfg.posEOT - d.position_mm;
d.commandedFromEOT_mm = cfg.posEOT - d.commandedPosition_mm;
d.targetFromEOT_mm    = cfg.posEOT - d.targetPosition_mm;
d.sweptVolume_mL      = sweptVolume(d.position_mm, cfg);
d.mixingVolume_mL     = mixingVolume(d.position_mm, cfg);
d.flow_mL_s           = mLPerMm(cfg) * d.velocity_mm_s;
d.pressureAbs_atm     = d.pressureGauge_MPa * cfg.atmPerMPa + cfg.atmAmbient;
nRec = numel(d.seq);
hasInterval = isfield(d, 'nExchanges');
if ~isfield(d, 'procStep')
    d.procStep = zeros(nRec, 1);
end
if ~isfield(d, 'nExchanges')
    d.nExchanges = ones(nRec, 1);
end
% File format: from the metadata, else guessed from the columns
format = 1;
if isfield(d, 'summaryScans')
    format = 2;
end
if hasInterval
    format = 3;
end
if isfield(meta, 'format') && isnumeric(meta.format) && isscalar(meta.format)
    format = meta.format;
end
Tx = cfg.exchangePeriod;
if isfield(meta, 'exchangePeriod_s') && isnumeric(meta.exchangePeriod_s) ...
        && isscalar(meta.exchangePeriod_s) && meta.exchangePeriod_s > 0
    Tx = meta.exchangePeriod_s;
end
conv = @(x) x * cfg.atmPerMPa + cfg.atmAmbient;     % [MPa gauge -> atm absolute]
d.pressureMeanAbs_atm = NaN(nRec, 1);
d.pressureMinAbs_atm  = NaN(nRec, 1);
d.pressureMaxAbs_atm  = NaN(nRec, 1);
d.posMeanFromEOT_mm   = NaN(nRec, 1);
if isfield(d, 'summaryScans') && format >= 3
    kk = find(d.summaryScans > 0);
    d.pressureMeanAbs_atm(kk) = conv(d.pressureMeanGauge_MPa(kk));
    d.pressureMinAbs_atm(kk)  = conv(d.pressureMinGauge_MPa(kk));
    d.pressureMaxAbs_atm(kk)  = conv(d.pressureMaxGauge_MPa(kk));
    d.posMeanFromEOT_mm(kk)   = cfg.posEOT - d.positionMean_mm(kk);
elseif isfield(d, 'summaryScans') && nRec > 2
    k = (1:nRec - 2)';
    ok = d.seq(k + 2) == d.seq(k) + 2 & d.summaryScans(k + 2) ~= 0; % [records]
    kk = k(ok);
    d.pressureMeanAbs_atm(kk) = conv(d.pressureMeanGauge_MPa(kk + 2));
    d.pressureMinAbs_atm(kk)  = conv(d.pressureMinGauge_MPa(kk + 2));
    d.pressureMaxAbs_atm(kk)  = conv(d.pressureMaxGauge_MPa(kk + 2));
end
d.interval_s = d.nExchanges * Tx;
d.decimated = any(d.nExchanges > 1);
if isempty(d.seq)
    d.gap = false(0, 1);
    d.tRel_s = zeros(0, 1);
    d.tStartRel_s = zeros(0, 1);
else
    d.gap = [false; diff(d.seq) > d.nExchanges(2:end)];
    d.tRel_s = d.t_s - d.t_s(1);
    d.tStartRel_s = d.tRel_s - (d.nExchanges - 1) * Tx;
end
end

function cfg = fromMeta(cfg, meta, field, key)
%FROMMETA Replace cfg.(field) by meta.(key) when the file has it as a number.
if isfield(meta, key) && isnumeric(meta.(key)) && isscalar(meta.(key)) ...
        && isfinite(meta.(key))
    cfg.(field) = meta.(key);
end
end
