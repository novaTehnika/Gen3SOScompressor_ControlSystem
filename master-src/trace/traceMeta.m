function meta = traceMeta(buf, cfg, startedAt, recordInterval)
%TRACEMETA Metadata block of a trace CSV: the recording's size and the
%   configuration needed to derive the displayed quantities from the raw
%   columns (traceDerive), as a struct whose field order is the line order
%   of the file's "# key: value" header.
%
%   meta = traceMeta(buf, cfg, startedAt)
%   meta = traceMeta(buf, cfg, startedAt, recordInterval)
%
%   buf        trace buffer (traceStart / traceDrain): n and missed give
%              the 'records' and 'missed' fields
%   cfg        masterConfig() in force when the trace was recorded
%   startedAt  char, start time as 'yyyy-mm-ddTHH:MM:SS'
%   recordInterval  s, the effective record interval when the recording
%              began (default cfg.exchangePeriod, a record per exchange).
%              A procedure can change it during the file; the interval of
%              a row is its nExchanges column times exchangePeriod_s
%
%   The units follow the field names; the geometry and pressure scaling
%   are frozen in the file so a later change of masterConfig does not
%   alter an old trace. The file format is 3: the summary columns are
%   aligned to the row of their window and the file has the procStep and
%   nExchanges columns.

if nargin < 4
    recordInterval = cfg.exchangePeriod;
end
meta.format            = 3;
meta.source            = 'SOS_Gen3_Compressor_Interface';
meta.started           = startedAt;
meta.records           = buf.n;
meta.missed            = buf.missed;
meta.dt_s              = cfg.dt;
meta.exchangePeriod_s  = cfg.exchangePeriod;
meta.recordInterval_s  = recordInterval;
meta.posEOT_mm         = cfg.posEOT;
meta.boreDiameter_mm   = cfg.boreDiameter;
meta.deadVolume_mL     = cfg.deadVolume;
meta.mixingDeadVolume_mL = cfg.mixingDeadVolume;
meta.atmPerMPa         = cfg.atmPerMPa;
meta.atmAmbient_atm    = cfg.atmAmbient;
meta.referenceUnits    = 'mm in position mode, mm/s in jog, MPa gauge in pressure mode';
end
