function d = benchReadCsv(file)
%BENCHREADCSV Read a bench CSV with a header line into a struct of column
%   vectors named by the header: a frictionRun log (t, step, status, mode,
%   ref, position, velocity, torque, pressure, inMotion, inBand,
%   exchangeMs) or a step probe capture (t, vCmd, velocity, torque,
%   position, pressure, volts, status). d.file keeps the name.

fid = fopen(file, 'r');
if fid < 0
    error('cannot open %s', file);
end
header = fgetl(fid);
fclose(fid);
names = strsplit(strtrim(header), ',');
data = dlmread(file, ',', 1, 0);
if size(data, 2) ~= numel(names)
    error('%s: %d columns, header names %d', file, size(data, 2), numel(names));
end
for k = 1:numel(names)
    d.(names{k}) = data(:, k);
end
d.file = file;
end
