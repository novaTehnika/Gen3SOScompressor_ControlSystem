function L = traceLayout()
%TRACELAYOUT Layout of the trace ring kept by traceStep: one record per
%   Modbus exchange, in a ring of L.depth rows. Column names carry their
%   unit after the last underscore where they have one; they are the CSV
%   header (traceColumnNames). L.guard repeats the record number so a row
%   read while being written is recognised.
L.depth = 512;              % records (25.6 s at the 50 ms exchange)
L.nCol  = 34;               % data columns, written to the CSV
L.width = 35;               % ring columns: data plus the guard
L.guard = 35;               % repeats seq
L.col.seq                  = 1;   % record number since model start, from 1
L.col.t_s                  = 2;   % model time of the recording step (s)
L.col.heartbeat            = 3;   % HR1 written in this exchange
L.col.heartbeatEcho        = 4;   % IR1
L.col.slaveCounter         = 5;   % IR2
L.col.confCode             = 6;   % IR3
L.col.slaveFaultCode       = 7;   % IR4
L.col.faultActive          = 8;   % IR5, 0/1
L.col.inMotion             = 9;   % IR6, 0/1
L.col.performanceStatus    = 10;  % IR7, 0/1
L.col.homingComplete       = 11;  % IR8, 0/1
L.col.position_mm          = 12;  % IR11-12, slave axis position
L.col.velocity_mm_s        = 13;  % IR13-14
L.col.torque_pct           = 14;  % IR15-16, % rated
L.col.commandedPosition_mm = 15;  % IR17-18
L.col.pressureGauge_MPa    = 16;  % IR19-20, as the slave reports it
L.col.modeCode             = 17;  % HR2 sent
L.col.motionEnable         = 18;  % HR3 sent
L.col.faultReset           = 19;  % HR4 sent
L.col.reference            = 20;  % HR5-6 sent: mm (position), mm/s (jog), MPa gauge (pressure)
L.col.positionSpeed_mm_s   = 21;  % HR7-8 sent
L.col.salineVolume_mL      = 22;  % HR9-10 sent
L.col.oxygenRatio          = 23;  % HR11-12 sent, STP volume per saline volume
L.col.statusCode           = 24;  % masterStep statusCode
L.col.faultCode            = 25;  % masterStep faultCode
L.col.state                = 26;  % masterStateMachine ST_*
L.col.linkOk               = 27;  % 0/1
L.col.doSafety             = 28;  % safety relay output, 0/1
L.col.op                   = 29;  % RequestedMode block
L.col.cmdSeq               = 30;  % CommandSeq block
L.col.targetPosition_mm    = 31;  % TargetPosition block, slave axis mm
L.col.targetPressure_atm   = 32;  % TargetPressure block, atm absolute
L.col.estop                = 33;  % ESTOP block
L.col.mixer                = 34;  % Mixer block
end
