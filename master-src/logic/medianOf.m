function m = medianOf(buf)
%MEDIANOF Median of an odd-length vector.

sorted = sort(buf);
m = sorted((numel(buf) + 1) / 2);
end
