function meta = traceMeta(buf, cfg, startedAt)
%TRACEMETA Metadata block of a trace CSV: the recording's size and the
%   configuration needed to derive the displayed quantities from the raw
%   columns (traceDerive), as a struct whose field order is the line order
%   of the file's "# key: value" header.
%
%   meta = traceMeta(buf, cfg, startedAt)
%
%   buf        trace buffer (traceStart / traceDrain): records and missed
%              give the 'records' and 'missed' fields
%   cfg        masterConfig() in force when the trace was recorded
%   startedAt  char, start time as 'yyyy-mm-ddTHH:MM:SS'
%
%   The units follow the field names; the geometry and pressure scaling
%   are frozen in the file so a later change of masterConfig does not
%   alter an old trace.

meta.format            = 1;
meta.source            = 'SOS_Gen3_Compressor_Interface';
meta.started           = startedAt;
meta.records           = buf.n;
meta.missed            = buf.missed;
meta.dt_s              = cfg.dt;
meta.exchangePeriod_s  = cfg.exchangePeriod;
meta.posEOT_mm         = cfg.posEOT;
meta.boreDiameter_mm   = cfg.boreDiameter;
meta.deadVolume_mL     = cfg.deadVolume;
meta.atmPerMPa         = cfg.atmPerMPa;
meta.atmAmbient_atm    = cfg.atmAmbient;
meta.referenceUnits    = 'mm in position mode, mm/s in jog, MPa gauge in pressure mode';
end
