function d = charReadRun(file)
%CHARREADRUN Read a charRun run log (<label>_run_<timestamp>.csv) into a
%   struct of column vectors: t, position, velocity, torque, pressure (MPa
%   gauge), reference, modeCode, motionEnable, status, inBand, step,
%   exchangeMs. d.file keeps the name.

raw = dlmread(file, ',', 1, 0);
if size(raw, 2) < 12
    error('%s has %d columns, expected the 12 of a charRun log', file, size(raw, 2));
end
d.file = file;
d.t = raw(:, 1);
d.position = raw(:, 2);
d.velocity = raw(:, 3);
d.torque = raw(:, 4);
d.pressure = raw(:, 5);
d.reference = raw(:, 6);
d.modeCode = raw(:, 7);
d.motionEnable = raw(:, 8);
d.status = raw(:, 9);
d.inBand = raw(:, 10);
d.step = raw(:, 11);
d.exchangeMs = raw(:, 12);
end
