function s = R_TRIG(s, E, dt) %#ok<INUSD>
%R_TRIG IEC 61131-3 rising-edge detector.

s.Q = s.CLK && ~s.M;
s.M = s.CLK;
end
