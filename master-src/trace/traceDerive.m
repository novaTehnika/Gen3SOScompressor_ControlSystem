function d = traceDerive(d)
%TRACEDERIVE Add the displayed quantities to a trace read by traceReadCsv.
%
%   d = traceDerive(d)
%
%   The geometry and pressure scaling come from d.meta (posEOT_mm,
%   boreDiameter_mm, mixingDeadVolume_mL, atmPerMPa, atmAmbient_atm), the
%   values in force when the trace was recorded; a key the file lacks
%   falls back to masterConfig(). Added columns:
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
%                          pressure over the scans of one master write,
%                          the same conversion as pressureAbs_atm. The
%                          slave publishes a window one to two exchanges
%                          after it closes, so row k takes the summary of
%                          record k+2, the window opened by row k's write.
%                          NaN when record k+2 is missing, when
%                          seq(k+2) ~= seq(k)+2, when its summaryScans is
%                          0 (no window yet) and in a format 1 file
%     gap                  true at the first record after a seq jump
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
d.pressureMeanAbs_atm = NaN(nRec, 1);
d.pressureMinAbs_atm  = NaN(nRec, 1);
d.pressureMaxAbs_atm  = NaN(nRec, 1);
if isfield(d, 'summaryScans') && nRec > 2
    k = (1:nRec - 2)';
    ok = d.seq(k + 2) == d.seq(k) + 2 & d.summaryScans(k + 2) ~= 0; % [records]
    kk = k(ok);
    d.pressureMeanAbs_atm(kk) = d.pressureMeanGauge_MPa(kk + 2) * cfg.atmPerMPa + cfg.atmAmbient;
    d.pressureMinAbs_atm(kk)  = d.pressureMinGauge_MPa(kk + 2) * cfg.atmPerMPa + cfg.atmAmbient;
    d.pressureMaxAbs_atm(kk)  = d.pressureMaxGauge_MPa(kk + 2) * cfg.atmPerMPa + cfg.atmAmbient;
end
if isempty(d.seq)
    d.gap = false(0, 1);
    d.tRel_s = zeros(0, 1);
else
    d.gap = [false; diff(d.seq) > 1];
    d.tRel_s = d.t_s - d.t_s(1);
end
end

function cfg = fromMeta(cfg, meta, field, key)
%FROMMETA Replace cfg.(field) by meta.(key) when the file has it as a number.
if isfield(meta, key) && isnumeric(meta.(key)) && isscalar(meta.(key)) ...
        && isfinite(meta.(key))
    cfg.(field) = meta.(key);
end
end
