function h = traceLiveNew(visible)
%TRACELIVENEW Create the live trace figure.
%   h = traceLiveNew(visible) opens a figure named 'Live trace' ('on' or
%   'off') with four stacked axes sharing the time axis: pressure [atm
%   abs] with the window maximum and minimum, window mean and the
%   snapshot, position from the end of travel [mm], velocity [mm/s] and
%   torque [% rated]. The lines start empty; traceLiveUpdate fills them
%   from a traceLiveBuffer.
%
%   The figure is built to be cheap to lay out, because under Connected
%   IO the model steps on the same MATLAB thread and a resize lays the
%   figure out again for every size the window passes through: the axes
%   have fixed normalized inner positions (no fitting of the labels into
%   the outer position), the time labels are on the bottom axes only,
%   the pressure lines are named in the top axes' title instead of a
%   legend, and the axes' interactions, toolbar and line picking are off
%   where MATLAB has them.
%
%   h  struct with fig (the figure), ax (1 x 4 axes) and lines (struct
%      of line handles pMax, pMin, pMean, p, pos, vel, torque)

cLine = [0 0.45 0.74];
cLine2 = [0.47 0.67 0.19];
cWin = [0.65 0.82 0.95];
f = figure('Name', 'Live trace', 'NumberTitle', 'off', 'Visible', visible);
trySet(f, 'GraphicsSmoothing', 'off');

% Inner positions, normalized: a left margin for the tick labels and
% the y label, the time labels under the bottom axes, the title above
% the top one.
left = 0.11;
width = 0.86;
bottom = 0.08;
top = 0.94;
gap = 0.025;
height = (top - bottom - 3 * gap) / 4;
ax = zeros(1, 4);
for k = 1:4
    y0 = bottom + (4 - k) * (height + gap);
    ax(k) = axes('Parent', f, 'Units', 'normalized', ...
        'Position', [left y0 width height], 'XGrid', 'on', 'YGrid', 'on');
    if k < 4
        set(ax(k), 'XTickLabel', {});
    end
end

lines.pMax = newLine(ax(1), cWin, 0.5);
lines.pMin = newLine(ax(1), cWin, 0.5);
lines.pMean = newLine(ax(1), cLine2, 0.5);
lines.p = newLine(ax(1), cLine, 0.5);
title(ax(1), [colorTag(cLine) 'pressure   ' colorTag(cLine2) ...
    'window mean   ' colorTag(cWin) 'window max/min']);
ylabel(ax(1), 'pressure [atm abs]');
lines.pos = newLine(ax(2), cLine, 0.5);
ylabel(ax(2), 'from EOT [mm]');
lines.vel = newLine(ax(3), cLine, 0.5);
ylabel(ax(3), 'velocity [mm/s]');
lines.torque = newLine(ax(4), cLine, 0.5);
ylabel(ax(4), 'torque [%]');
xlabel(ax(4), 'time before now [s]');

% MATLAB's default interactions and axes toolbar react to the pointer
% on every move; neither is used here.
for k = 1:4
    try
        disableDefaultInteractivity(ax(k));
    catch
    end
    try
        set(get(ax(k), 'Toolbar'), 'Visible', 'off');
    catch
    end
end
h = struct('fig', f, 'ax', ax, 'lines', lines);
end

% %% ===== LOCAL FUNCTIONS =====

% An empty line in ax that the pointer does not pick
function l = newLine(ax, color, width)
l = line('Parent', ax, 'XData', NaN, 'YData', NaN, 'Color', color, ...
    'LineWidth', width);
trySet(l, 'PickableParts', 'none');
end

% The TeX tag that colours the title text after it
function s = colorTag(c)
s = sprintf('\\color[rgb]{%.2f %.2f %.2f}', c);
end

% Set a property that only some MATLAB or Octave versions have
function trySet(obj, name, value)
try
    set(obj, name, value);
catch
end
end
