function controllerBridge()
%CONTROLLERBRIDGE The link between the operator app and the controller
%   process, without either: the message encoding (ctlEncode, ctlDecode),
%   the controller's decisions (controllerStep) against a stand-in for the
%   shell model, the status it sends (ctlStatusMessage) and the app's view
%   of it (ctlClientUpdate), the launch command (ctlLaunchCommand).
%
%   The stand-in model holds the Constant blocks the controller writes and
%   answers a new CommandSeq: a motion mode runs (status 4), RequestedMode
%   0 stops it over three passes (status 6, then 0).
%
%   Run from the repository root:
%     addpath master-src/logic master-src/trace master-src/controller tests
%     controllerBridge
%   Depends on: master-src/controller master-src/logic master-src/trace

cfg = masterConfig();
L = traceLayout();
B = ctlBlocks();

% ----- Encoding -----
m1 = struct('t', 'set', 'name', 'TargetPressure', 'value', 3.25);
m2 = struct('t', 'ping');
m3 = struct('t', 'track', 'file', 'C:\temp\a "b".bin', 'n', 72001);
bytes = [ctlEncode(m1), ctlEncode(m2), ctlEncode(m3)];
assert(isa(bytes, 'uint8') && bytes(end) == 10);
[msgs, rest] = ctlDecode(bytes);
assert(numel(msgs) == 3 && isempty(rest));
assert(strcmp(msgs{1}.name, 'TargetPressure') && msgs{1}.value == 3.25);
assert(strcmp(msgs{3}.file, m3.file) && msgs{3}.n == 72001);
% Split anywhere: the same messages, in order
for cut = [1, 7, numel(ctlEncode(m1)), numel(bytes) - 1]
    [a, r] = ctlDecode(bytes(1:cut));
    [b, r] = ctlDecode([r, bytes(cut + 1:end)]);
    got = [a, b];
    assert(numel(got) == 3 && isempty(r) && strcmp(got{2}.t, 'ping'));
end
% Garbage lines and an empty line are dropped; a partial line is kept
[msgs, rest] = ctlDecode([uint8('not json'), 10, 10, uint8('[1,2]'), 10, ...
    ctlEncode(m2), uint8('{"t":"pi')]);
assert(numel(msgs) == 1 && strcmp(msgs{1}.t, 'ping') && strcmp(char(rest), '{"t":"pi'));
% NaN travels as null and comes back NaN in the app's view
v = struct('StatusCode', 4, 'Position', NaN);
[msgs, ~] = ctlDecode(ctlEncode(ctlStatusMessage(true, 'running', v, [], [], 0, {})));
c = ctlClientUpdate(ctlClientNew(), msgs, 1);
assert(c.values.StatusCode == 4 && isnan(c.values.Position) && isnan(c.values.Pressure));
fprintf('ok  messages: round trip, split reads, garbage dropped, NaN as null\n');

% ----- Status and the app's view -----
rows = synthRows(1:3);
reader = struct('lastSeq', 3, 'missed', 2);
vals = struct();
for k = 1:numel(B.read)
    vals.(B.read{k}) = k;
end
s1 = ctlStatusMessage(true, 'running', vals, rows, reader, 7, {'Track table set'});
s2 = ctlStatusMessage(true, 'running', vals, synthRows(4), ...
    struct('lastSeq', 4, 'missed', 2), 8, {});
s3 = ctlStatusMessage(true, 'running', vals, zeros(0, L.nCol), ...
    struct('lastSeq', 4, 'missed', 2), 8, {});
[msgs, ~] = ctlDecode([ctlEncode(s1), ctlEncode(s2), ctlEncode(s3)]);
c = ctlClientUpdate(ctlClientNew(), msgs, 12.5);
assert(c.running && c.live && strcmp(c.sim, 'running') && c.cmdSeq == 8);
assert(isequal(c.rows, synthRows(1:4)), 'records appended in order, one row and none decode');
assert(c.lastSeq == 4 && c.missed == 2 && c.statusAt == 12.5 && c.nStatus == 3);
assert(isequal(c.notes, {'Track table set'}));
for k = 1:numel(B.read)
    assert(c.values.(B.read{k}) == k);
end
% Not live: every value NaN, running false
[msgs, ~] = ctlDecode(ctlEncode(ctlStatusMessage(false, 'stopped', [], [], [], 8, {})));
c = ctlClientUpdate(c, msgs, 13);
assert(~c.running && ~c.live && isnan(c.values.StatusCode) && size(c.rows, 1) == 4);
% Other messages leave the view alone
c2 = ctlClientUpdate(c, {struct('t', 'ping')}, 99);
assert(isequaln(c2, c));
fprintf('ok  status: values, records in order, reader counts, notes, not live\n');

% ----- Controller: writes, start, stop -----
M = modelNew();
b = controllerNew();
t = 0;
appMsgs = {msg('hello'), set('RequestedMode', 0), set('CommandSeq', 0), ...
    set('ESTOP', 0), set('TargetPressure', 4.5), msg('start')};
[b, M, act] = pass(b, M, t, true, appMsgs, cfg);
assert(isequal(act.sets(:, 1)', {'RequestedMode', 'CommandSeq', 'ESTOP', 'TargetPressure'}));
assert(act.start && M.running && ~act.exit && M.blocks.TargetPressure == 4.5);
% Refused writes: an unknown block, a TrackTable, a non-finite value
t = t + 0.05;
[b, M, act] = pass(b, M, t, true, {set('Bogus', 1), set('TrackTable', 1), ...
    struct('t', 'set', 'name', 'Mixer', 'value', NaN), msg('nonsense')}, cfg);
assert(isempty(act.sets) && numel(act.notes) == 4 && ~act.start);
% A jog: the model runs it
t = t + 0.05;
[b, M] = pass(b, M, t, true, {set('RequestedMode', 3), set('CommandSeq', 1)}, cfg);
assert(M.status == 4 && b.cmdSeq == 1);
% 'stop' waits for the axis: the app's stop first, then the model stops
% once the status leaves 1 to 8
t = t + 0.05;
[b, M, act] = pass(b, M, t, true, {set('RequestedMode', 0), set('CommandSeq', 2), ...
    msg('stop')}, cfg);
assert(~act.stop && M.running && b.stopping);
stopped = false;
seen = [];
for i = 1:10
    t = t + 0.05;
    seen(end + 1) = M.status; %#ok<AGROW>   the status the controller decides on
    [b, M, act] = pass(b, M, t, true, {msg('ping')}, cfg);
    if act.stop
        stopped = true;
        break
    end
end
assert(stopped && ~M.running && ~b.stopping);
assert(seen(end) == 0 && all(seen(1:end - 1) == 6), ...
    'the model is stopped only once the axis is at rest');
% A stop while stopped does nothing; a start runs it again
t = t + 0.05;
[b, M, act] = pass(b, M, t, true, {msg('stop')}, cfg);
assert(~act.stop && ~b.stopping && ~act.exit);
t = t + 0.05;
[b, M, act] = pass(b, M, t, true, {msg('start')}, cfg);
assert(act.start && M.running);
fprintf('ok  controller: writes in order, refusals, start, stop once the axis is at rest\n');

% A stop that never sees the axis at rest stops the model after tStopMax + 0.5
M2 = M;
M2.stuck = true;
b2 = b;
[b2, M2] = pass(b2, M2, t, true, {set('RequestedMode', 3), set('CommandSeq', 3)}, cfg);
t2 = t + 0.05;
[b2, M2] = pass(b2, M2, t2, true, {msg('stop')}, cfg);
tStop = t2;
while M2.running
    t2 = t2 + 0.05;
    [b2, M2] = pass(b2, M2, t2, true, {msg('ping')}, cfg);
    assert(t2 - tStop < cfg.tStopMax + 1);
end
assert(t2 - tStop > cfg.tStopMax + 0.5 - 1e-9);
fprintf('ok  controller: a stop gives up waiting after tStopMax + 0.5 s\n');

% ----- Controller: the app lost -----
% A running jog; the connection closes: motion stops at once, once
t = t + 0.05;
[b, M] = pass(b, M, t, true, {set('RequestedMode', 3), set('CommandSeq', 10)}, cfg);
assert(M.status == 4);
t = t + 0.05;
[b, M, act] = pass(b, M, t, false, {}, cfg);
assert(isequal(act.sets, {'RequestedMode', 0; 'TrackMode', 0; 'CommandSeq', 11}));
assert(b.appLost && b.cmdSeq == 11 && any(strcmp(act.notes, 'App not responding: motion stopped')));
for i = 1:5
    t = t + 0.05;
    [b, M, act] = pass(b, M, t, false, {}, cfg);
    assert(isempty(act.sets) && M.running, 'one stop per loss; the model keeps running');
end
assert(M.status == 0);
% The app returns and continues from the controller's CommandSeq
t = t + 0.05;
[b, M] = pass(b, M, t, true, {msg('hello')}, cfg);
assert(~b.appLost);
[msgs, ~] = ctlDecode(ctlEncode(ctlStatusMessage(true, 'running', M.blocks, [], [], b.cmdSeq, {})));
c = ctlClientUpdate(ctlClientNew(), msgs, t);
assert(c.cmdSeq == 11);
t = t + 0.05;
[b, M] = pass(b, M, t, true, {set('RequestedMode', 3), set('CommandSeq', c.cmdSeq + 1)}, cfg);
assert(M.status == 4, 'the next command after a return is seen');
% Silence with the connection open: motion stops after appTimeout
tLast = t;
while true
    t = t + 0.05;
    [b, M, act] = pass(b, M, t, true, {}, cfg);
    if ~isempty(act.sets)
        break
    end
end
assert(t - tLast > cfg.appTimeout && t - tLast < cfg.appTimeout + 0.06);
assert(strcmp(act.sets{1, 1}, 'RequestedMode') && act.sets{1, 2} == 0);
% Still silent: after controllerOrphanTime the model stops and the
% controller ends
exited = false;
while t - tLast < cfg.controllerOrphanTime + 10
    t = t + 0.05;
    [b, M, act] = pass(b, M, t, false, {}, cfg);
    if act.exit
        exited = true;
        break
    end
end
assert(exited && ~M.running && t - tLast >= cfg.controllerOrphanTime);
fprintf('ok  controller: app lost (closed or silent) stops motion once, orphan ends it\n');

% A controller no app ever reaches ends after controllerOrphanTime
b = controllerNew();
M = modelNew();
t = 0;
act.exit = false;
while ~act.exit
    t = t + 0.05;
    [b, M, act] = pass(b, M, t, false, {}, cfg);
end
assert(abs(t - cfg.controllerOrphanTime) < 0.06);

% ----- Controller: quit -----
b = controllerNew();
M = modelNew();
t = 0.05;
[b, M] = pass(b, M, t, true, {msg('start'), set('RequestedMode', 3), set('CommandSeq', 1)}, cfg);
assert(M.running);
t = t + 0.05;
[b, M] = pass(b, M, t, true, {set('RequestedMode', 3), set('CommandSeq', 2)}, cfg);
assert(M.status == 4);
t = t + 0.05;
[b, M, act] = pass(b, M, t, true, {msg('quit')}, cfg);
assert(any(strcmp(act.sets(:, 1), 'CommandSeq')) && ~act.exit, 'quit stops motion first');
n = 0;
while ~act.exit
    t = t + 0.05;
    [b, M, act] = pass(b, M, t, false, {}, cfg);
    n = n + 1;
    assert(n < 20);
end
assert(~M.running && M.lastStatus == 0);
% Quit with the model stopped ends at once, and a start after quit is refused
b = controllerNew();
M = modelNew();
[~, M, act] = pass(b, M, 0.05, true, {msg('quit'), msg('start')}, cfg);
assert(act.exit && ~act.start && ~M.running);
fprintf('ok  controller: quit stops motion, waits for rest, stops the model, ends\n');

% ----- Track message -----
[~, ~, act] = pass(controllerNew(), modelNew(), 0.05, true, ...
    {m3, struct('t', 'track', 'file', 'x')}, cfg);
assert(isequal(act.track, m3) || (strcmp(act.track.file, m3.file) && act.track.n == m3.n));
assert(numel(act.notes) == 1);
fprintf('ok  controller: track message passed on, one without a length refused\n');

% ----- Sending only to an app that is reading -----
b = controllerNew();
M = modelNew();
[b, M, act] = pass(b, M, 1, true, {msg('hello')}, cfg);
assert(act.send);
[b, M, act] = pass(b, M, 1 + cfg.controllerSendQuiet - 0.01, true, {}, cfg);
assert(act.send);
[b, M, act] = pass(b, M, 1 + cfg.controllerSendQuiet + 0.01, true, {}, cfg);
assert(~act.send, 'a quiet app gets nothing');
[b, M, act] = pass(b, M, 2, false, {msg('ping')}, cfg);
assert(~act.send, 'nothing without a connection');
[~, ~, act] = pass(b, M, 2.05, true, {msg('ping')}, cfg);
assert(act.send);
fprintf('ok  controller: statuses only while the app has been heard from\n');

% ----- Launch command -----
cmd = ctlLaunchCommand(cfg, 'C:\SOS\master-src\controller', ...
    'C:\Program Files\MATLAB\R2025b\bin\matlab.exe', true);
assert(strcmp(cmd, ['start "SOS Gen3 controller" /min ' ...
    '"C:\Program Files\MATLAB\R2025b\bin\matlab.exe" -batch ' ...
    '"cd(''C:\SOS\master-src\controller''); runController"']));
cmd = ctlLaunchCommand(cfg, '/opt/o''brien/controller', '/usr/bin/matlab', false);
assert(~isempty(strfind(cmd, 'cd(''/opt/o''''brien/controller'')')) && cmd(end) == '&');
fprintf('ok  launch command: Windows start /min with -batch, quoting\n');
end

% %% ===== LOCAL FUNCTIONS =====

function m = msg(t)
m = struct('t', t);
end

function m = set(name, value)
m = struct('t', 'set', 'name', name, 'value', value);
end

% Records with the given sequence numbers, every other column its
% column number plus the sequence number
function r = synthRows(seq)
L = traceLayout();
r = repmat(1:L.nCol, numel(seq), 1) + seq(:);
r(:, L.col.seq) = seq(:);
end

% The stand-in shell model: the Constant blocks, the status masterStep
% would report and whether it runs
function M = modelNew()
B = ctlBlocks();
M.blocks = struct();
for k = 1:numel(B.write)
    M.blocks.(B.write{k}) = 0;
end
M.running = false;
M.status = 0;
M.lastStatus = 0;
M.seenSeq = 0;
M.stopLeft = 0;
M.stuck = false;
end

% One model pass after the controller's writes: a new CommandSeq starts the
% requested mode (any motion mode runs as status 4) or, with mode 0, a
% stop of three passes (status 6, then 0)
function M = modelStep(M)
M.lastStatus = M.status;
if ~M.running
    M.status = 0;
    return
end
seq = M.blocks.CommandSeq;
if seq ~= M.seenSeq
    M.seenSeq = seq;
    if M.blocks.RequestedMode == 0
        if M.status ~= 0
            M.status = 6;
            M.stopLeft = 3;
        end
    else
        M.status = 4;
    end
elseif M.status == 6 && ~M.stuck
    M.stopLeft = M.stopLeft - 1;
    if M.stopLeft <= 0
        M.status = 0;
    end
end
M.lastStatus = M.status;
end

% One controller pass, carried out on the stand-in as runController does:
% the writes in order, then start or stop, then a model step
function [b, M, act] = pass(b, M, t, connected, msgs, cfg)
in = struct('t', t, 'connected', connected, 'msgs', {msgs}, ...
    'running', M.running, 'statusCode', M.status);
[b, act] = controllerStep(b, in, cfg);
for k = 1:size(act.sets, 1)
    M.blocks.(act.sets{k, 1}) = act.sets{k, 2};
end
if act.start
    M.running = true;
    M.seenSeq = M.blocks.CommandSeq;
end
if act.stop
    M.running = false;
end
M = modelStep(M);
end
