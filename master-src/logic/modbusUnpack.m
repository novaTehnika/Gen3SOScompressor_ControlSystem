function v = modbusUnpack(lo, hi)
%MODBUSUNPACK REAL value from two 16-bit registers, low word first.
%   v = modbusUnpack(lo, hi) is the inverse of modbusPack.
b = bitor(bitshift(uint32(hi), 16), uint32(lo));
v = double(typecast(b, 'single'));
end
