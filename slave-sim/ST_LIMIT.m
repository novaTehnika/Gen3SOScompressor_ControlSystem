function y = ST_LIMIT(mn, in, mx)
%ST_LIMIT IEC 61131-3 LIMIT(MN, IN, MX).

y = min(max(in, mn), mx);
end
