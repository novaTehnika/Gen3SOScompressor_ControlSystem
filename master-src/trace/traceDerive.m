function d = traceDerive(d)
%TRACEDERIVE Add the displayed quantities to a trace read by traceReadCsv.
%
%   d = traceDerive(d)
%
%   The geometry and pressure scaling come from d.meta (posEOT_mm,
%   boreDiameter_mm, deadVolume_mL, atmPerMPa, atmAmbient_atm), the
%   values in force when the trace was recorded; a key the file lacks
%   falls back to masterConfig(). Added columns:
%     posFromEOT_mm        posEOT - position, distance to the end of travel
%     commandedFromEOT_mm  the same for the slave's commanded position
%     targetFromEOT_mm     the same for the app's target position
%     volume_mL            chamber volume at the position (x2mL)
%     flow_mL_s            mL/mm * velocity, positive toward the end of
%                          travel as the app shows it
%     pressureAbs_atm      gauge MPa * atmPerMPa + atmAmbient
%     gap                  true at the first record after a seq jump
%     tRel_s               t_s from the first record

cfg = masterConfig();
meta = struct();
if isfield(d, 'meta') && isstruct(d.meta)
    meta = d.meta;
end
cfg = fromMeta(cfg, meta, 'posEOT', 'posEOT_mm');
cfg = fromMeta(cfg, meta, 'boreDiameter', 'boreDiameter_mm');
cfg = fromMeta(cfg, meta, 'deadVolume', 'deadVolume_mL');
cfg = fromMeta(cfg, meta, 'atmPerMPa', 'atmPerMPa');
cfg = fromMeta(cfg, meta, 'atmAmbient', 'atmAmbient_atm');

d.posFromEOT_mm       = cfg.posEOT - d.position_mm;
d.commandedFromEOT_mm = cfg.posEOT - d.commandedPosition_mm;
d.targetFromEOT_mm    = cfg.posEOT - d.targetPosition_mm;
d.volume_mL           = x2mL(d.position_mm, cfg);
d.flow_mL_s           = mLPerMm(cfg) * d.velocity_mm_s;
d.pressureAbs_atm     = d.pressureGauge_MPa * cfg.atmPerMPa + cfg.atmAmbient;
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
