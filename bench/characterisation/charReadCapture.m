function c = charReadCapture(file)
%CHARREADCAPTURE Read an axis probe capture written by charRun
%   (<label>_<n>_<step>_<timestamp>.csv): t (0 at the scan the command
%   changed), velocity and command (mm/s), torque (%), pressure (MPa gauge),
%   one row per 2 ms scan. c.Ts is the sample period from the time column.

raw = dlmread(file, ',', 1, 0);
if size(raw, 2) < 5
    error('%s has %d columns, expected the 5 of a probe capture', file, size(raw, 2));
end
c.file = file;
c.t = raw(:, 1);
c.velocity = raw(:, 2);
c.command = raw(:, 3);
c.torque = raw(:, 4);
c.pressure = raw(:, 5);
c.Ts = median(diff(c.t));
end
