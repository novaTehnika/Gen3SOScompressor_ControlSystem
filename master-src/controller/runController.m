function runController()
%RUNCONTROLLER The controller process: runs the shell model for the app.
%   runController runs in a MATLAB of its own, which the operator app
%   starts (ctlLaunchCommand) and which does nothing else, so nothing the
%   app does (a window resize, a dialog, a plot) holds back the model's
%   steps and with them the master's heartbeat. It returns, and with
%   -batch that MATLAB ends, once the app has quit or been gone for
%   cfg.controllerOrphanTime and the model is stopped.
%
%   It listens on 127.0.0.1:cfg.controllerPort for the app (one connection
%   at a time; a reopened app connects again). A write that fails, or
%   leaves the server reporting no client, means the app's connection is
%   gone (an app ended from the Task Manager is seen only this way): the
%   server is opened again, so the next pass sees no app and a new app can
%   connect. Every cfg.controllerPeriod it
%     1. reads the app's messages (ctlDecode) and the model's outputs
%        (ctlBlocks().read, through the blocks' RuntimeObject),
%     2. decides with controllerStep: block writes, the track table,
%        start, stop, exit,
%     3. writes the Constant blocks in order (set_param), loads a track
%        table from the file the app wrote, starts or stops the model,
%     4. drains the TraceRing (traceDrain) and sends the app a status
%        (ctlStatusMessage) with the new records and notes; while the app
%        is quiet (act.send false) they are held back, at most one ring's
%        worth of records (traceLayout().depth, the oldest dropped and
%        counted as missed), and sent once it answers again,
%   and pauses, which lets the model step. Notes go to the app's status
%   line and, with the connection and model events, to
%   master-src/captures/controller.log.
%
%   It can also be run by hand in a MATLAB session, to watch the log; it
%   then returns instead of ending that MATLAB.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'logic'), fullfile(root, 'shell'), ...
    fullfile(root, 'trace'), here);
cfg = masterConfig();
B = ctlBlocks();
L = traceLayout();
mdl = 'SOS_Gen3_Master_Shell';

logFolder = fullfile(root, 'captures');
if ~isfolder(logFolder)
    mkdir(logFolder);
end
logFid = fopen(fullfile(logFolder, 'controller.log'), 'a');
closeLog = onCleanup(@() closeIfOpen(logFid));
say(logFid, 'Controller starting (MATLAB %s, port %d)', version, cfg.controllerPort);

[srv, err] = openServer(cfg.controllerPort);
if isempty(srv)
    say(logFid, 'Cannot listen on port %d (%s): another controller runs?', ...
        cfg.controllerPort, err);
    return
end

sim = [];
modelNote = '';
try
    load_system(mdl);
    sim = simulation(mdl);
catch ME
    modelNote = ['Shell model not loaded: ' ME.message ...
        ' (run buildShell on this PC)'];
    say(logFid, '%s', modelNote);
end

b = controllerNew();
t0 = tic;
buf = uint8([]);
reader = [];
rto = struct();
wasConnected = false;
wasRunning = false;
heldRows = zeros(0, L.nCol);
heldNotes = {};
dropped = 0;
selfStop = false;
trackTimed = false;
stopModel = onCleanup(@() stopIfRunning(sim));

while true
    t = toc(t0);
    notes = {};

    % 1. The app's messages and the model's outputs
    if isempty(srv)
        srv = openServer(cfg.controllerPort);   % reopened after a broken connection
    end
    connected = ~isempty(srv) && srv.Connected;
    if connected ~= wasConnected
        buf = uint8([]);
        if connected
            say(logFid, 'App connected');
            if ~isempty(modelNote)
                notes{end + 1} = modelNote; %#ok<AGROW>
            end
        else
            say(logFid, 'App disconnected');
        end
        wasConnected = connected;
    end
    if connected
        try
            if srv.NumBytesAvailable > 0
                buf = [buf, read(srv, srv.NumBytesAvailable, 'uint8')]; %#ok<AGROW>
            end
        catch
        end
    end
    [msgs, buf] = ctlDecode(buf);
    running = modelRunning(sim);
    [values, rto] = readValues(mdl, B.read, rto, running);
    code = NaN;
    if isstruct(values)
        code = values.StatusCode;
    end

    % 2. Decide
    in = struct('t', t, 'connected', connected, 'msgs', {msgs}, ...
        'running', running, 'statusCode', code);
    [b, act] = controllerStep(b, in, cfg);
    notes = [notes, act.notes]; %#ok<AGROW>

    % 3. Carry it out: the writes in order, the table, then start or stop
    for k = 1:size(act.sets, 1)
        try
            set_param([mdl '/' act.sets{k, 1}], 'Value', ...
                num2str(act.sets{k, 2}, 17));
        catch ME
            notes{end + 1} = ['Write to ' act.sets{k, 1} ' failed: ' ME.message]; %#ok<AGROW>
        end
    end
    if ~isempty(act.track)
        [note, trackTimed] = loadTrackTable(sim, mdl, act.track, cfg, trackTimed);
        if ~isempty(note)
            notes{end + 1} = note; %#ok<AGROW>
        end
    end
    if act.start
        if isempty(sim)
            notes{end + 1} = 'Model unable to start: the shell model is not loaded'; %#ok<AGROW>
        else
            try
                start(sim);
                reader = traceStart(0, false);
                rto = struct();
                selfStop = false;
                say(logFid, 'Model started (%s)', char(string(sim.Status)));
            catch ME
                notes{end + 1} = ['Model unable to start: ' ME.message]; %#ok<AGROW>
            end
        end
    end

    % 4. The new trace records, a last time before a stop
    rows = zeros(0, L.nCol);
    if running && ~isempty(reader)
        [ring, rto] = readBlock(mdl, 'TraceRing', rto);
        try
            [reader, rows] = traceDrain(reader, ring);
        catch ME
            notes{end + 1} = ['Trace read failed: ' ME.message]; %#ok<AGROW>
        end
    end
    if act.stop
        try
            stop(sim);
            selfStop = true;
            say(logFid, 'Model stopped');
        catch ME
            notes{end + 1} = ['Model stop failed: ' ME.message]; %#ok<AGROW>
        end
    end
    running = modelRunning(sim);
    if wasRunning && ~running && ~selfStop
        notes{end + 1} = sprintf('Model stopped (%s)', char(string(sim.Status))); %#ok<AGROW>
    end
    wasRunning = running;
    if ~running
        values = [];
    end

    % Send, or hold back while the app is quiet (at most one ring's
    % worth of records; the oldest beyond that count as missed)
    heldRows = [heldRows; rows]; %#ok<AGROW>
    heldNotes = [heldNotes, notes]; %#ok<AGROW>
    if size(heldRows, 1) > L.depth
        dropped = dropped + size(heldRows, 1) - L.depth;
        heldRows = heldRows(end - L.depth + 1:end, :);
    end
    heldNotes = heldNotes(max(1, end - 19):end);
    if act.start
        dropped = 0;
    end
    if act.send || act.keepalive
        if act.send
            counts = [];
            if isstruct(reader)
                counts = struct('lastSeq', reader.lastSeq, 'missed', reader.missed + dropped);
            end
            bytes = ctlEncode(ctlStatusMessage(running, statusText(sim), ...
                values, heldRows, counts, b.cmdSeq, heldNotes));
        else
            bytes = ctlEncode(struct('t', 'k'));
        end
        ok = false;
        try
            write(srv, bytes);
            ok = srv.Connected;
        catch
        end
        if ok && act.send
            heldRows = zeros(0, L.nCol);
            heldNotes = {};
        elseif ~ok
            % The app's connection is gone: open the server again, so the
            % next pass sees no app (motion stops) and a new app can connect
            say(logFid, 'App connection broken');
            try
                delete(srv);
            catch
            end
            srv = [];
            wasConnected = false;
            heldRows = zeros(0, L.nCol);
        end
    elseif ~connected
        heldRows = zeros(0, L.nCol);    % no app to give them to
    end
    for k = 1:numel(notes)
        if ~strcmp(notes{k}, modelNote)
            say(logFid, '%s', notes{k});
        end
    end
    if act.exit
        break
    end
    pause(cfg.controllerPeriod);
end
say(logFid, 'Controller ended');
end

% %% ===== LOCAL FUNCTIONS =====

% Listen on 127.0.0.1:port; [] and the reason when that fails
function [srv, err] = openServer(port)
srv = [];
err = '';
try
    srv = tcpserver('127.0.0.1', port);
catch ME
    err = ME.message;
end
end

function tf = modelRunning(sim)
tf = ~isempty(sim) && isvalid(sim) && sim.Status == "running";
end

function s = statusText(sim)
s = 'not loaded';
if ~isempty(sim) && isvalid(sim)
    s = char(string(sim.Status));
end
end

% The output of one of the model's Gain blocks through its cached
% RuntimeObject; 0 when the block has none yet (the model has not
% stepped)
function [value, rto] = readBlock(mdl, name, rto)
key = matlab.lang.makeValidName(name);
if isfield(rto, key)
    try
        value = rto.(key).OutputPort(1).Data;
        return
    catch
    end
end
value = 0;
try
    r = get_param([mdl '/' name], 'RuntimeObject');
    if ~isempty(r)
        rto.(key) = r;
        value = r.OutputPort(1).Data;
    end
catch
end
end

% The read blocks as a struct, or [] while the model does not run or has
% not stepped
function [values, rto] = readValues(mdl, names, rto, running)
values = [];
if ~running
    return
end
v = struct();
for k = 1:numel(names)
    key = matlab.lang.makeValidName(names{k});
    [x, rto] = readBlock(mdl, names{k}, rto);
    if ~isfield(rto, key)
        return
    end
    v.(names{k}) = double(x);
end
values = v;
end

% Load the track table the app wrote (msg.n doubles in msg.file) into the
% model workspace's trackTable: the simulation's variable tuning first,
% else the variable assigned and the model updated. The first load is
% timed in the note.
function [note, timed] = loadTrackTable(sim, mdl, msg, cfg, timed)
note = '';
t0 = tic;
try
    fid = fopen(msg.file, 'r');
    if fid < 0
        error('cannot open %s', msg.file);
    end
    tbl = fread(fid, msg.n, 'double')';
    fclose(fid);
    if numel(tbl) ~= cfg.trackTableLength
        error('%d values, the table holds %d', numel(tbl), cfg.trackTableLength);
    end
    try
        setVariable(sim, 'trackTable', tbl, 'Workspace', mdl);
        path = 'tuned';
    catch
        assignin(get_param(mdl, 'ModelWorkspace'), 'trackTable', tbl);
        set_param(mdl, 'SimulationCommand', 'update');
        path = 'model update';
    end
    if ~timed
        timed = true;
        note = sprintf('Track table set in %.2f s (%s, %d values)', ...
            toc(t0), path, numel(tbl));
    end
catch ME
    note = ['Track table not set: ' ME.message];
end
try
    delete(msg.file);
catch
end
end

function stopIfRunning(sim)
if modelRunning(sim)
    try
        stop(sim);
    catch
    end
end
end

function say(fid, varargin)
line = sprintf('%s  %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF'), ...
    sprintf(varargin{:}));
if fid > 0
    fprintf(fid, '%s', line);
end
if ~batchStartupOptionUsed     % run by hand: shown in that session too
    fprintf('%s', line);
end
end

function closeIfOpen(fid)
if fid > 0
    fclose(fid);
end
end
