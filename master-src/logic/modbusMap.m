function map = modbusMap()
%MODBUSMAP Register layout of the master <-> slave Modbus TCP exchange.
%   Indices are 1-based positions in the write vector (holding registers
%   HR1..) and the read vector (input registers IR1..), as in
%   docs/master/ModbusRegisterMap.md. REAL values occupy two registers, low
%   word first (modbusPack / modbusUnpack).

map.nWrite = 10;            % HR1-10, one FC16 write
map.nRead  = 20;            % IR1-20, one FC04 read

map.hr.heartbeat    = 1;    % master step counter, wraps at 65536
map.hr.modeCode     = 2;    % mode code, or the full fault code during reset
map.hr.motionEnable = 3;
map.hr.faultReset   = 4;
map.hr.reference    = 5;    % REAL: mm, mm/s or % by mode (5-6)
map.hr.salineVolume = 7;    % REAL: mL, saline volume of the fill (7-8)
map.hr.oxygenRatio  = 9;    % REAL: oxygen charge as STP volume per saline volume (9-10)

map.ir.heartbeatEcho     = 1;
map.ir.slaveCounter      = 2;   % slave scan counter, wraps at 65536
map.ir.confCode          = 3;   % confirmed mode code, 0 outside the handshake
map.ir.faultCode         = 4;   % full fault code, 0 when none
map.ir.faultActive       = 5;
map.ir.inMotion          = 6;
map.ir.performanceStatus = 7;
map.ir.homingComplete    = 8;
map.ir.position          = 11;  % REAL mm (11-12)
map.ir.velocity          = 13;  % REAL mm/s (13-14)
map.ir.torque            = 15;  % REAL % (15-16)
map.ir.commandedPosition = 17;  % REAL mm (17-18)
map.ir.pressure          = 19;  % REAL MPa gauge (19-20)
end
