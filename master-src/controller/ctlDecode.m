function [msgs, rest] = ctlDecode(buf)
%CTLDECODE Split received bytes into messages.
%   [msgs, rest] = ctlDecode(buf) takes the bytes received so far (uint8
%   row, the remainder of the previous call first) and returns the
%   complete lines as decoded structs, in order, and the bytes after the
%   last line feed, which the next call completes. A line that is not a
%   JSON object is dropped.

buf = uint8(buf(:)');
msgs = {};
ends = find(buf == 10);
if isempty(ends)
    rest = buf;
    return
end
starts = [1, ends(1:end - 1) + 1];
for k = 1:numel(ends)
    line = char(buf(starts(k):ends(k) - 1));
    if isempty(strtrim(line))
        continue
    end
    try
        m = jsondecode(line);
    catch
        continue
    end
    if isstruct(m) && isscalar(m)
        msgs{end + 1} = m; %#ok<AGROW>
    end
end
rest = buf(ends(end) + 1:end);
end
