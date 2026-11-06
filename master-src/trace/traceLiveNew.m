function h = traceLiveNew(visible)
%TRACELIVENEW Create the live trace figure.
%   h = traceLiveNew(visible) opens a figure named 'Live trace' ('on' or
%   'off') with four stacked axes sharing the time axis: pressure [atm
%   abs] with the window maximum and minimum, window mean and the
%   snapshot, position from the end of travel [mm], velocity [mm/s] and
%   torque [% rated]. The lines start empty; traceLiveUpdate fills them
%   from a traceLiveBuffer.
%
%   h  struct with fig (the figure), ax (1 x 4 axes) and lines (struct
%      of line handles pMax, pMin, pMean, p, pos, vel, torque)

cLine = [0 0.45 0.74];
cLine2 = [0.47 0.67 0.19];
cWin = [0.65 0.82 0.95];
f = figure('Name', 'Live trace', 'NumberTitle', 'off', 'Visible', visible);
ax = zeros(1, 4);
ax(1) = subplot(4, 1, 1);
hold on;
lines.pMax = plot(NaN, NaN, 'Color', cWin, 'LineWidth', 0.5);
lines.pMin = plot(NaN, NaN, 'Color', cWin, 'LineWidth', 0.5);
lines.pMean = plot(NaN, NaN, 'Color', cLine2);
lines.p = plot(NaN, NaN, 'Color', cLine);
legend([lines.pMax lines.pMean lines.p], {'window max/min', ...
    'window mean', 'pressure'}, 'Location', 'northwest');
ylabel('pressure [atm abs]');
ax(2) = subplot(4, 1, 2);
lines.pos = plot(NaN, NaN, 'Color', cLine);
ylabel('from EOT [mm]');
ax(3) = subplot(4, 1, 3);
lines.vel = plot(NaN, NaN, 'Color', cLine);
ylabel('velocity [mm/s]');
ax(4) = subplot(4, 1, 4);
lines.torque = plot(NaN, NaN, 'Color', cLine);
ylabel('torque [%]');
for k = 1:4
    set(ax(k), 'XGrid', 'on', 'YGrid', 'on');
end
xlabel(ax(4), 'time before now [s]');
h = struct('fig', f, 'ax', ax, 'lines', lines);
end
