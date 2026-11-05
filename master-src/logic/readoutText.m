function text = readoutText(value, decimals, deadband)
%READOUTTEXT Text of a readout with the given number of decimals. A NaN
%   shows as '--'; a value smaller than deadband in magnitude shows as
%   zero; the value is rounded to the display resolution first, so a zero
%   carries no sign.

if isnan(value)
    text = '--';
    return
end
if abs(value) < deadband
    value = 0;
end
value = round(value * 10^decimals) / 10^decimals;
if value == 0
    value = 0;
end
text = sprintf(['%.' num2str(decimals) 'f'], value);
end
