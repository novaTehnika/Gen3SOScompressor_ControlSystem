function [b, act] = controllerStep(b, in, cfg)
%CONTROLLERSTEP One pass of the controller's loop: what to do with the
%   app's messages and the model's state.
%   [b, act] = controllerStep(b, in, cfg) is the controller's decision
%   logic, with no I/O: runController reads the connection and the model
%   into in and carries out act in the order of its fields.
%
%   b    state (controllerNew)
%   in   t          s since the controller started
%        connected  the app's TCP connection is open
%        msgs       messages received since the last pass (ctlDecode)
%        running    the shell model runs
%        statusCode masterStep's status code (any value when not running)
%   cfg  masterConfig: appTimeout, controllerOrphanTime,
%        controllerSendQuiet, controllerKeepalive, tStopMax
%
%   act  sets   n x 2 cell {block, value}, in the order to write them:
%               the app's writes as received, then any stop the
%               controller issues itself
%        track  the app's 'track' message (file, n), or [] for none
%        start  start the model
%        stop   stop the model (after a last drain of the trace ring)
%        exit   end the controller
%        send   send the app a status: its connection is open and its
%               last message at most cfg.controllerSendQuiet old, so a
%               stalled app that reads nothing does not fill the
%               connection and block the controller's writes
%        keepalive  send a keepalive line instead: the app is connected
%               but quiet and nothing was sent for
%               cfg.controllerKeepalive. A connection whose app has died
%               fails only when written to, so the controller keeps
%               writing, a few bytes at a time, and takes a failed write
%               as the connection closed (runController).
%        notes  cell of texts for the status line and the log
%
%   Messages: 'set' (name, value) writes one of ctlBlocks().write, a
%   finite real scalar; 'start' starts a stopped model; 'stop' stops a
%   running one once the axis is at rest (status outside 1 to 8) or
%   cfg.tStopMax + 0.5 s have passed; 'track' hands over the track table;
%   'quit' stops the model the same way and ends the controller; 'ping'
%   and 'hello' only show the app is there. Anything else is noted and
%   ignored.
%
%   The app is present while its connection is open and its last message
%   is at most cfg.appTimeout old (it pings on every poll). When it is
%   not and the model runs, the controller stops motion once (RequestedMode
%   0, TrackMode 0 and CommandSeq advanced), and again after the app
%   returns and leaves again. After cfg.controllerOrphanTime without the
%   app, or on 'quit', it stops motion, stops the model as for 'stop' and
%   ends.

act = struct('sets', {cell(0, 2)}, 'track', [], 'start', false, ...
    'stop', false, 'exit', false, 'send', false, 'keepalive', false, ...
    'notes', {{}});
names = ctlBlocks().write;

for k = 1:numel(in.msgs)
    m = in.msgs{k};
    if ~isstruct(m) || ~isfield(m, 't') || ~ischar(m.t)
        continue
    end
    b.lastMsgAt = in.t;
    switch m.t
        case 'set'
            if ~isfield(m, 'name') || ~ischar(m.name) || ~any(strcmp(m.name, names))
                act.notes{end + 1} = 'Write refused: not a block the app sets';
            elseif ~isfield(m, 'value') || ~isnumeric(m.value) || ...
                    ~isscalar(m.value) || ~isfinite(m.value)
                act.notes{end + 1} = ['Write to ' m.name ' refused: not a finite number'];
            else
                act.sets(end + 1, :) = {m.name, double(m.value)};
                if strcmp(m.name, 'CommandSeq')
                    b.cmdSeq = double(m.value);
                end
            end
        case 'start'
            if ~in.running && ~b.quitting
                act.start = true;
            end
        case 'stop'
            if in.running && ~b.stopping
                b.stopping = true;
                b.stopSince = in.t;
            end
        case 'track'
            if isfield(m, 'file') && ischar(m.file) && isfield(m, 'n') && ...
                    isnumeric(m.n) && isscalar(m.n) && m.n >= 1
                act.track = m;
            else
                act.notes{end + 1} = 'Track table refused: no file or length';
            end
        case 'quit'
            b.quitting = true;
        case {'ping', 'hello'}
        otherwise
            act.notes{end + 1} = ['Unknown message ' m.t ' ignored'];
    end
end

% The app's presence: an open connection and a recent message
alive = in.connected && in.t - b.lastMsgAt <= cfg.appTimeout;
if alive
    b.aliveAt = in.t;
    b.appLost = false;
elseif in.t - b.aliveAt >= cfg.controllerOrphanTime && ~b.quitting
    b.quitting = true;
    act.notes{end + 1} = sprintf('No app for %g s: stopping the model and ending', ...
        cfg.controllerOrphanTime);
end

% Without the app, motion stops once per loss; quitting stops it too.
if in.running && ~b.stopping && ((~alive && ~b.appLost) || b.quitting)
    [b, act] = stopMotion(b, act);
    if ~alive && ~b.appLost
        b.appLost = true;
        act.notes{end + 1} = 'App not responding: motion stopped';
    end
    if b.quitting
        b.stopping = true;
        b.stopSince = in.t;
    end
end

% A stop of the model waits for the axis to stop
if b.stopping
    if ~in.running
        b.stopping = false;
    elseif in.statusCode < 1 || in.statusCode > 8 || ...
            in.t - b.stopSince > cfg.tStopMax + 0.5
        act.stop = true;
        b.stopping = false;
    end
end

act.exit = b.quitting && ~in.running && ~act.start;
act.send = in.connected && in.t - b.lastMsgAt <= cfg.controllerSendQuiet;
act.keepalive = ~act.send && in.connected && ...
    in.t - b.sentAt >= cfg.controllerKeepalive;
if act.send || act.keepalive
    b.sentAt = in.t;
end
end

% %% ===== LOCAL FUNCTIONS =====

% RequestedMode 0 with a new CommandSeq stops any operation; TrackMode 0
% ends a track
function [b, act] = stopMotion(b, act)
b.cmdSeq = b.cmdSeq + 1;
act.sets(end + 1:end + 3, :) = {'RequestedMode', 0; 'TrackMode', 0; ...
    'CommandSeq', b.cmdSeq};
end
