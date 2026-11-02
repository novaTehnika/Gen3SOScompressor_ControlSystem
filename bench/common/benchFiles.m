function files = benchFiles(arg)
%BENCHFILES Log and capture file names from a name, a pattern or a cell
%   array of either, as a cell array of paths in name order. The fits take
%   explicit files; this lets their callers write
%       charDriveLag('../captures/s1_cap_steps_*.csv')
%   A name without a wildcard is returned as given (so that a missing
%   file is reported by the reader); a pattern that matches nothing is an
%   error.

if ischar(arg)
    arg = {arg};
end
files = {};
for k = 1:numel(arg)
    p = arg{k};
    if isempty(strfind(p, '*')) && isempty(strfind(p, '?'))
        files{end + 1} = p; %#ok<AGROW>
        continue;
    end
    d = dir(p);
    d = d(~[d.isdir]);
    if isempty(d)
        error('benchFiles:none', 'no file matches %s', p);
    end
    folder = fileparts(p);
    names = sort({d.name});
    for j = 1:numel(names)
        files{end + 1} = fullfile(folder, names{j}); %#ok<AGROW>
    end
end
end
