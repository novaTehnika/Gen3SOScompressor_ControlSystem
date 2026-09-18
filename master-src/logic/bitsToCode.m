function code = bitsToCode(bit0, bit1, bit2)
%BITSTOCODE Combine three I/O bits (bit0 = LSB) into a 0..7 code.

code = (bit2 ~= 0) * 4 + (bit1 ~= 0) * 2 + (bit0 ~= 0);
end
