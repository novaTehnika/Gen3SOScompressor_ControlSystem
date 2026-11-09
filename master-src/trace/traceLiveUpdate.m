function ok = traceLiveUpdate(h, b)
%TRACELIVEUPDATE Redraw the live trace figure from its buffer.
%   ok = traceLiveUpdate(h, b) sets the data of the lines of h
%   (traceLiveNew) from the buffer b (traceLiveBuffer), oldest first, with
%   the time relative to the newest row, and the limits of every axis to
%   [-b.span 0]. It returns false when the figure has been closed. With
%   an empty buffer it sets no data. It does not call drawnow.

ok = ~isempty(h) && isstruct(h) && isfield(h, 'fig') && ishghandle(h.fig);
if ~ok
    return
end
if b.count > 0
    cap = size(b.data, 1);
    idx = mod(b.head - 1 - b.count + (0:b.count - 1)', cap) + 1;
    m = b.data(idx, :);
    x = m(:, 1) - m(end, 1);                    % [s relative to the newest row]
    set(h.lines.pMax, 'XData', x, 'YData', m(:, 5));
    set(h.lines.pMin, 'XData', x, 'YData', m(:, 4));
    set(h.lines.pMean, 'XData', x, 'YData', m(:, 3));
    set(h.lines.p, 'XData', x, 'YData', m(:, 2));
    set(h.lines.pos, 'XData', x, 'YData', m(:, 6));
    set(h.lines.vel, 'XData', x, 'YData', m(:, 7));
    set(h.lines.torque, 'XData', x, 'YData', m(:, 8));
end
for k = 1:numel(h.ax)
    set(h.ax(k), 'XLim', [-b.span 0]);
end
end
