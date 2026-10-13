function w = modbusPack(v)
%MODBUSPACK Two 16-bit register values for a REAL, low word first.
%   w = modbusPack(v) returns [low high] as doubles in 0..65535, the IEEE 754
%   single-precision bit pattern of v in the slave's word order.
b = typecast(single(v), 'uint32');
w = double([bitand(b, uint32(65535)), bitshift(b, -16)]);
end
