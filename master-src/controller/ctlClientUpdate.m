function c = ctlClientUpdate(c, msgs, t)
%CTLCLIENTUPDATE Take the controller's messages into the app's view.
%   c = ctlClientUpdate(c, msgs, t) applies each status message of msgs
%   (ctlDecode) in order: the newest running state, values, cmdSeq and
%   reader counts replace the old, the trace records are appended to
%   c.rows and the notes to c.notes (the app takes both and empties
%   them), and statusAt becomes t. A value that arrived as null is NaN.
%   Other messages are ignored.

B = ctlBlocks();
L = traceLayout();
for k = 1:numel(msgs)
    m = msgs{k};
    if ~isstruct(m) || ~isfield(m, 't') || ~strcmp(m.t, 'status')
        continue
    end
    c.running = flag(m, 'running');
    c.live = flag(m, 'live');
    if isfield(m, 'sim') && ischar(m.sim)
        c.sim = m.sim;
    end
    c.cmdSeq = num(m, 'cmdSeq', c.cmdSeq);
    if isfield(m, 'values') && isstruct(m.values)
        for j = 1:numel(B.read)
            c.values.(B.read{j}) = num(m.values, B.read{j}, NaN);
        end
    end
    if isfield(m, 'rec') && isnumeric(m.rec) && ~isempty(m.rec) && ...
            num(m, 'cols', 0) == L.nCol && mod(numel(m.rec), L.nCol) == 0
        c.rows = [c.rows; reshape(double(m.rec(:)), L.nCol, []).'];
    end
    c.lastSeq = num(m, 'lastSeq', c.lastSeq);
    c.missed = num(m, 'missed', c.missed);
    if isfield(m, 'notes')
        n = m.notes;
        if ischar(n)
            n = {n};
        end
        if iscell(n)
            c.notes = [c.notes, reshape(n(cellfun(@ischar, n)), 1, [])];
        end
    end
    c.statusAt = t;
    c.nStatus = c.nStatus + 1;
end
end

% %% ===== LOCAL FUNCTIONS =====

function v = num(s, name, default)
v = default;
if isfield(s, name) && isnumeric(s.(name)) && isscalar(s.(name))
    v = double(s.(name));
elseif isfield(s, name) && isempty(s.(name))
    v = NaN;
end
end

function tf = flag(s, name)
tf = isfield(s, name) && isscalar(s.(name)) && logical(s.(name));
end
