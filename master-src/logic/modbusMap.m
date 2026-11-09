function map = modbusMap()
%MODBUSMAP Register layout of the master <-> slave Modbus TCP exchange.
%   Indices are 1-based positions in the write vector (holding registers
%   HR1..) and the read vector (input registers IR1..), as in
%   docs/master/ModbusRegisterMap.md. REAL values occupy two registers, low
%   word first (modbusPack / modbusUnpack).

map.nWrite = 12;            % HR1-12, one FC16 write
map.nRead  = 34;            % IR1-34, one FC04 read

map.hr.heartbeat    = 1;    % master step counter, wraps at 65536
map.hr.modeCode     = 2;    % mode code, or the full fault code during reset
map.hr.motionEnable = 3;
map.hr.faultReset   = 4;
map.hr.reference    = 5;    % REAL: mm, mm/s or % by mode (5-6)
map.hr.positionSpeed = 7;   % REAL: mm/s, speed of a position move (7-8)
map.hr.salineVolume = 9;    % REAL: mL, saline volume of the fill (9-10)
map.hr.oxygenRatio  = 11;   % REAL: oxygen charge as STP volume per saline volume (11-12)

map.ir.heartbeatEcho     = 1;
map.ir.slaveCounter      = 2;   % slave scan counter, wraps at 65536
map.ir.confCode          = 3;   % confirmed mode code, 0 outside the handshake
map.ir.faultCode         = 4;   % full fault code, 0 when none
map.ir.faultActive       = 5;
map.ir.inMotion          = 6;
map.ir.performanceStatus = 7;
map.ir.homingComplete    = 8;
map.ir.pressureBlock     = 9;   % 1 = pressurising motion blocked above the ceiling
map.ir.atEndOfTravel     = 10;  % 1 = at the end of travel (soft limit minus the end margin)
map.ir.position          = 11;  % REAL mm (11-12)
map.ir.velocity          = 13;  % REAL mm/s (13-14)
map.ir.torque            = 15;  % REAL % (15-16)
map.ir.commandedPosition = 17;  % REAL mm (17-18)
map.ir.pressure          = 19;  % REAL MPa gauge (19-20)
% Summary of the last closed window (one master write long), IR21-34
map.ir.pressureMean      = 21;  % REAL MPa gauge (21-22)
map.ir.pressureMin       = 23;  % REAL MPa gauge (23-24)
map.ir.pressureMax       = 25;  % REAL MPa gauge (25-26)
map.ir.velocityMean      = 27;  % REAL mm/s (27-28)
map.ir.torqueMean        = 29;  % REAL % (29-30)
map.ir.positionMean      = 31;  % REAL mm (31-32)
map.ir.summaryScans      = 33;  % UINT, scans in the window, 0 before the first
map.ir.summarySeq        = 34;  % UINT, window number, wraps at 65536
end
