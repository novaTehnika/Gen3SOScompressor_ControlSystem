function [bit0, bit1, bit2] = modeBits(code)
%MODEBITS Split a 0..7 mode or fault code into its three I/O bits (bit0 = LSB).

bit0 = mod(code, 2);
bit1 = mod(floor(code / 2), 2);
bit2 = mod(floor(code / 4), 2);
end
