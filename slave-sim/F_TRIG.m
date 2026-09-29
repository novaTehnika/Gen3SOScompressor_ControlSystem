function s = F_TRIG(s, E, dt) %#ok<INUSD>
%F_TRIG IEC 61131-3 falling-edge detector.

s.Q = ~s.CLK && ~s.M;
s.M = ~s.CLK;
end
