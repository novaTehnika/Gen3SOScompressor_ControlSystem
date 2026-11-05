function [ring, ok] = traceRingCheck(ring)
%TRACERINGCHECK Ring as a traceLayout().depth x width matrix.
%   [ring, ok] = traceRingCheck(ring) reshapes a ring that arrives with
%   the right number of elements but another shape (the Data of a signal
%   read as a column vector) and sets ok true. Anything else, such as the
%   scalar 0 readBlock returns when the model is not running, is returned
%   unchanged with ok false. The reshape follows MATLAB's column-major
%   order, in which a matrix read from a signal is flattened.

L = traceLayout();
ok = isnumeric(ring) && numel(ring) == L.depth * L.width;
if ok
    ring = reshape(double(ring), L.depth, L.width);
end
end
