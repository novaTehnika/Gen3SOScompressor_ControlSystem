function s = TON(s, E, dt) %#ok<INUSL>
%TON IEC 61131-3 on-delay timer. Q is set once IN has been TRUE for PT
%   seconds; a call with IN FALSE resets it.

if ~s.IN
    s.ET = 0;
    s.Q = false;
else
    if s.M
        s.ET = s.ET + dt;
    else
        s.ET = 0;
    end
    s.Q = s.ET >= s.PT;
end
s.M = s.IN;
end
