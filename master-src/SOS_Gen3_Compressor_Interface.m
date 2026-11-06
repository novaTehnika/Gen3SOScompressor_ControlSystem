classdef SOS_Gen3_Compressor_Interface < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        ProcedureMenu                   matlab.ui.container.Menu
        ProcedureFileMenus              matlab.ui.container.Menu
        ProcedureOpenMenu               matlab.ui.container.Menu
        ProcedureReloadMenu             matlab.ui.container.Menu
        TraceMenu                       matlab.ui.container.Menu
        TraceRecordMenu                 matlab.ui.container.Menu
        TracePlotMenu                   matlab.ui.container.Menu
        TraceLiveMenu                   matlab.ui.container.Menu
        TracePanel                      matlab.ui.container.Panel
        PlotTraceButton                 matlab.ui.control.Button
        TraceStatusLabel                matlab.ui.control.Label
        IntervalDropDown                matlab.ui.control.DropDown
        LivePlotButton                  matlab.ui.control.Button
        RecordButton                    matlab.ui.control.StateButton
        FaultActiveLamp                 matlab.ui.control.Lamp
        FaultActiveLampLabel            matlab.ui.control.Label
        MixerOnLamp                     matlab.ui.control.Lamp
        MixerOnLampLabel                matlab.ui.control.Label
        FlowPanel                       matlab.ui.container.Panel
        FlowValue                       matlab.ui.control.Label
        PositionPanel                   matlab.ui.container.Panel
        PositionValue                   matlab.ui.control.Label
        PressurePanel                   matlab.ui.container.Panel
        PressureValue                   matlab.ui.control.Label
        PressureUnitGroup               matlab.ui.container.ButtonGroup
        PressureUnitPsiButton           matlab.ui.control.ToggleButton
        PressureUnitAtmButton           matlab.ui.control.ToggleButton
        PressureUnitMPaButton           matlab.ui.control.ToggleButton
        VolumePanel                     matlab.ui.container.Panel
        VolumeValue                     matlab.ui.control.Label
        VolumeViewGroup                 matlab.ui.container.ButtonGroup
        VolumeSweptButton               matlab.ui.control.ToggleButton
        VolumeMixingButton              matlab.ui.control.ToggleButton
        VolumeSinceZeroButton           matlab.ui.control.ToggleButton
        VolumeZeroButton                matlab.ui.control.Button
        StatusLabel                     matlab.ui.control.Label
        ControlsPanel                   matlab.ui.container.Panel
        ESTOP                           matlab.ui.control.StateButton
        PressureGoButton                matlab.ui.control.Button
        TargetPressureSpinner           matlab.ui.control.Spinner
        TargetPressureatmSpinnerLabel   matlab.ui.control.Label
        MixerButton                     matlab.ui.control.StateButton
        JogDownButton                   matlab.ui.control.StateButton
        JogUpButton                     matlab.ui.control.StateButton
        PositionGoButton                matlab.ui.control.Button
        TargetPositionSpinner           matlab.ui.control.Spinner
        TargetPositionmmSpinnerLabel    matlab.ui.control.Label
        SpeedPresetButtons              matlab.ui.control.Button
        SpeedMmsLabel                   matlab.ui.control.Label
        SpeedField                      matlab.ui.control.NumericEditField
        SpeedLabel                      matlab.ui.control.Label
        GoHomeButton                    matlab.ui.control.Button
        HomeButton                      matlab.ui.control.Button
        ManualControlButton            matlab.ui.control.StateButton
        StartProcedureButton            matlab.ui.control.Button
        ConnectToCompressorButton       matlab.ui.control.Button
        ProcedurePanel                  matlab.ui.container.Panel
        ProcStepList                    matlab.ui.control.ListBox
        ProcHeadingLabel                matlab.ui.control.Label
        ProcTitleLabel                  matlab.ui.control.Label
        ProcTextArea                    matlab.ui.control.TextArea
        ProcChecks                      matlab.ui.control.CheckBox
        ProcParamLabels                 matlab.ui.control.Label
        ProcParamFields                 matlab.ui.control.NumericEditField
        ProcConfirmButton               matlab.ui.control.Button
        ProcRepeatButton                matlab.ui.control.Button
        ProcPauseButton                 matlab.ui.control.Button
        ProcAbortButton                 matlab.ui.control.Button
        ProcTimerLabel                  matlab.ui.control.Label
        ProcVolumeLabel                 matlab.ui.control.Label
        ProcPressureTargetLabel         matlab.ui.control.Label
        ProcMessageLabel                matlab.ui.control.Label
    end


    % Public properties that correspond to the Simulink model
    properties (Access = public, Transient)
        Simulation simulink.Simulation
    end

    % Cost of the display poll in seconds (n polls, mean, max, last) and
    % the polls that took longer than cfg.displayPeriod
    properties (SetAccess = private)
        PollStats = struct('n', 0, 'mean', 0, 'max', 0, 'last', 0, 'overruns', 0)
    end


    properties (Access = private)
        modelName
        updateTimer
        cfg                 % masterConfig: geometry and limits
        commandSeq = 0      % mirrors the CommandSeq block; a change marks a new command
        lastStatusCode = 0
        wasRunning = false      % the model was running at the last poll
        resetDialogOpen = false
        estopActive = false     % master reports STATUS_ESTOP
        linkReady = false       % master reports the slave link up

        % Operation started from a mode button. It stays active until the
        % model reports it ended, so its button can stop it.
        activeOp = 0            % OP_STOP when none
        activeJogDir = 0
        speedPercent = 100      % operator speed, % of cfg.speedMax
        volumeView = "swept"    % "swept", "mixing" or "sinceZero"
        volumeZeroPosition = NaN    % mm, slave frame, where the incremental volume was zeroed
        lastPosition = NaN      % mm, slave frame, at the last poll
        rto = struct()          % cached RuntimeObject of each read block
        activeTarget = NaN      % position or pressure sent with activeOp
        opConfirmed = false     % the model has reported activeOp running
        opSentAt = uint64(0)    % tic of the last request

        % Pressure display unit: psi, atm or MPa, all absolute, with how it
        % is shown and converted (pressureUnitInfo). The model, the engine
        % and the target of a running operation stay in atm absolute.
        pressureUnit = 'atm'
        pressureInfo = []
        flagPressureBlock = false   % the slave refuses pressurising motion (IR9)
        flagAtEnd = false           % the slave is at the end of travel (IR10)
        procSpeedCap = NaN          % mm/s: the running procedure step's speed cap

        % Procedure (non-manual mode): the engine state, the normalized
        % definition it runs, the inputs of the last poll (for steps taken
        % from a callback), the tic of the last step and the text of the
        % last refused operator action.
        procDef = []
        proc = []
        procName = ''           % function of the loaded procedure ('' when none loaded)
        procIn = struct('connected', false, 'statusCode', 0, 'faultCode', 0, ...
            'estop', false, 'pressure', 0, 'volume', 0, ...
            'pressureBlock', false, 'atEndOfTravel', false, ...
            'positionFromEOT', NaN, 'positionSpeed', Inf)
        procTick = []
        procNote = ''

        % Trace stream: every poll the new records of the model's TraceRing
        % block are drained (traceReader, which keeps no records), aligned
        % (traceAligner) and appended to the live plot's buffer (traceLive).
        % While traceRecording is set and not paused, the same aligned rows
        % are combined into one row per record interval (traceDecim) and
        % appended to trace.
        traceReader = []        % traceStart(ring, false): newest record, missed count
        traceAligner = []       % traceAlignNew state
        traceLive = []          % traceLiveBuffer
        traceDecim = []         % traceDecimateNew state of the recording
        traceRecording = false
        trace = []              % traceAppend buffer of the recording
        traceStartedAt = ''     % wall-clock start, for the file's metadata
        traceFromSeq = 0        % rows with a record number above this are taken
        traceStartSeq = 0       % newest record number when the recording started
        traceMissed0 = 0        % reader.missed when the recording started
        traceInterval = 0       % s, the record interval in force (whole exchanges)
        traceIntervalStart = 0  % s, the interval the recording started with
        traceChoice = 0.05      % s, the operator's interval (the dropdown outside a procedure)
        traceOwner = ''         % '' | 'operator' | 'procedure': who ends the recording
        tracePaused = false     % a procedure step switched recording off
        traceDeclined = false   % the operator stopped recording during this run
        procRecord = []         % s, the procedure's last numeric record request
        liveFig = []            % traceLiveNew struct while the live plot is open
        livePolls = 0           % polls since the live plot was opened

        % Track steps: the number the TrackSeq block holds, whether a
        % series is being played, the procedure step the ProcStep block
        % holds and whether the table update has been timed yet.
        trackSeq = 0
        trackActive = false
        procStepShown = 0
        trackTableTimed = false
    end

    properties (Access = private, Constant)
        % Operations understood by masterStateMachine (RequestedMode block)
        OP_STOP     = 0
        OP_HOME     = 1
        OP_POSITION = 2
        OP_JOG      = 3
        OP_PRESSURE = 4
        OP_RESET    = 5
        OP_GO_HOME  = 6

        STATUS_FAULT     = 10     % FaultCode block holds the code
        STATUS_ESTOP     = 20
        STATUS_LINK_LOST = 34
        STATUS_LINK_WAIT = 35

        COLOR_IDLE     = [0.96 0.96 0.96]
        COLOR_STARTING = [1 0.8 0.3]
        COLOR_ACTIVE   = [0.4 0.8 0.4]
        COLOR_STOP     = [1 0 0]
        COLOR_STOP_REQ = [0.55 0 0]
        COLOR_ESTOP    = [1 0.85 0]

        faultNames = ["Handshake", "Drive", "Position limit", ...
            "Master link lost", "Piston exit guard", "Limit switch", "Encoder", ...
            "Pressure sensor"]
    end

    methods (Access = private)

        function tf = isRunning(app)
            tf = ~isempty(app.Simulation) && isvalid(app.Simulation) && ...
                app.Simulation.Status == "running";
        end

        % Stop any operation, wait for the master to report the axis
        % stopped (up to its own stop timeout), then stop the model. The
        % DAQ outputs fall to their final values (safety relay off) with
        % MotionEnable already low.
        function disconnect(app)
            app.StatusLabel.Text = "Stopping...";
            drawnow;
            try
                sendCommand(app, app.OP_STOP);
                clearOp(app);
                deadline = tic;
                while toc(deadline) < app.cfg.tStopMax + 0.5
                    code = readBlock(app, "StatusCode");
                    if code < 1 || code > 8   % not moving, starting or stopping
                        break
                    end
                    pause(0.05);
                end
                streamTrace(app);       % the records since the last poll, while the ring is readable
                stop(app.Simulation);
                app.wasRunning = false;     % not an unexpected stop
                finishRecording(app, false);
                app.StatusLabel.Text = "Disconnected";
            catch ME
                app.StatusLabel.Text = append("Disconnect failed: ", ME.message);
            end
            updateButtons(app);
        end

        % Folder the traces are saved in, created when missing
        function folder = capturesFolder(~)
            folder = fullfile(fileparts(mfilename("fullpath")), "captures");
            if ~isfolder(folder)
                mkdir(folder);
            end
        end

        % Show the recording's progress; the label turns red once the
        % ring has overwritten records the app did not read in time.
        function showTraceProgress(app)
            missed = app.traceReader.missed - app.traceMissed0;
            elapsed = (app.traceReader.lastSeq - app.traceStartSeq) * ...
                app.cfg.exchangePeriod;
            text = sprintf("%d records, %.0f s, every %g s", app.trace.n, ...
                elapsed, app.traceInterval);
            if app.tracePaused
                text = append(text, ", paused by step");
            end
            color = [0 0 0];
            if missed > 0
                text = append(text, sprintf(", %d missed", missed));
                color = [1 0 0];
            end
            setText(app, app.TraceStatusLabel, text);
            if ~isequal(app.TraceStatusLabel.FontColor, color)
                app.TraceStatusLabel.FontColor = color;
            end
        end

        % Begin a recording that combines the exchanges into rows of the
        % interval (s; whole exchanges, at least one). The owner is who
        % ends it: 'operator' (the Record button) or 'procedure'.
        function startRecording(app, interval, owner)
            streamTrace(app);
            L = traceLayout();
            app.trace = traceAppend([], zeros(0, L.nCol));
            app.traceFromSeq = app.traceReader.lastSeq;
            app.traceStartSeq = app.traceReader.lastSeq;
            app.traceMissed0 = app.traceReader.missed;
            n = traceIntervalExchanges(interval, app.cfg);
            app.traceDecim = traceDecimateNew(n);
            app.traceInterval = n * app.cfg.exchangePeriod;
            app.traceIntervalStart = app.traceInterval;
            app.tracePaused = false;
            app.traceOwner = owner;
            app.traceStartedAt = datestr(now, "yyyy-mm-ddTHH:MM:SS");
            app.traceRecording = true;
            app.TraceStatusLabel.Tooltip = "";
            showTraceProgress(app);
        end

        % Emit the interval being combined, complete or not, into the
        % recording
        function flushRecording(app)
            L = traceLayout();
            [app.traceDecim, out] = traceDecimate(app.traceDecim, ...
                zeros(0, L.nCol), true);
            app.trace = traceAppend(app.trace, out);
        end

        % Change the interval of the recording at once: the open interval
        % is emitted and a new grid starts at the next record.
        function setRecordInterval(app, interval)
            streamTrace(app);
            flushRecording(app);
            n = traceIntervalExchanges(interval, app.cfg);
            app.traceDecim = traceDecimateNew(n);
            app.traceInterval = n * app.cfg.exchangePeriod;
            showTraceProgress(app);
        end

        % Stop taking rows and keep the file (a procedure step switched
        % recording off)
        function pauseRecording(app)
            streamTrace(app);
            flushRecording(app);
            app.tracePaused = true;
            showTraceProgress(app);
        end

        % Take rows again from the next record, at the interval
        function resumeRecording(app, interval)
            streamTrace(app);
            n = traceIntervalExchanges(interval, app.cfg);
            app.traceFromSeq = app.traceReader.lastSeq;
            app.traceDecim = traceDecimateNew(n);
            app.traceInterval = n * app.cfg.exchangePeriod;
            app.tracePaused = false;
            showTraceProgress(app);
        end

        % Put the aligned rows that are newer than the recording's start
        % through the decimator into the recording
        function takeRecords(app, rows)
            if isempty(rows)
                return
            end
            L = traceLayout();
            rows = rows(rows(:, L.col.seq) > app.traceFromSeq, :);
            [app.traceDecim, out] = traceDecimate(app.traceDecim, rows, false);
            app.trace = traceAppend(app.trace, out);
        end

        % Drain the ring on every poll while the model runs, recording or
        % not: the new records are aligned and appended to the live plot's
        % buffer, and to the recording unless it is paused.
        function streamTrace(app)
            if ~isRunning(app) || isempty(app.traceReader)
                return
            end
            try
                [app.traceReader, new] = traceDrain(app.traceReader, ...
                    readBlock(app, "TraceRing"));
                [app.traceAligner, aligned] = traceAlign(app.traceAligner, new, false);
                app.traceLive = traceLiveAppend(app.traceLive, aligned);
                if app.traceRecording && ~app.tracePaused
                    takeRecords(app, aligned);
                end
                if app.traceRecording
                    showTraceProgress(app);
                end
            catch ME
                app.TraceStatusLabel.Text = append("Trace read failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
        end

        % Redraw the live plot every cfg.livePlotPeriod while it is open;
        % a closed figure clears it.
        function updateLivePlot(app)
            if isempty(app.liveFig)
                return
            end
            app.livePolls = app.livePolls + 1;
            every = max(1, round(app.cfg.livePlotPeriod / app.cfg.displayPeriod));
            if mod(app.livePolls, every) ~= 0
                return
            end
            if ~traceLiveUpdate(app.liveFig, app.traceLive)
                app.liveFig = [];
            end
        end

        function closeLivePlot(app)
            if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig)
                delete(app.liveFig.fig);
            end
            app.liveFig = [];
        end

        % End the recording and save it. The file is written to the
        % captures folder first; an interactive stop then offers to move
        % it, and cancelling keeps it where it is.
        function finishRecording(app, interactive)
            if ~app.traceRecording
                return
            end
            L = traceLayout();
            streamTrace(app);
            try
                if ~app.tracePaused
                    % The rows the aligner still holds back, without a window
                    [~, tail] = traceAlign(app.traceAligner, zeros(0, L.nCol), true);
                    takeRecords(app, tail);
                end
                flushRecording(app);
            catch ME
                app.TraceStatusLabel.Text = append("Trace read failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
            app.traceRecording = false;     % before any file I/O or dialog
            app.traceOwner = '';
            app.tracePaused = false;
            app.TraceStatusLabel.Tooltip = "";
            app.TraceStatusLabel.FontColor = [0 0 0];
            missed = app.traceReader.missed - app.traceMissed0;
            if app.trace.n == 0
                app.TraceStatusLabel.Text = "Nothing recorded";
                updateButtons(app);
                return
            end
            try
                file = char(fullfile(capturesFolder(app), append("trace_", ...
                    datestr(now, "yyyymmdd_HHMMSS"), ".csv")));
                traceWriteCsv(file, app.trace.data(1:app.trace.n, :), ...
                    traceMeta(struct('n', app.trace.n, 'missed', missed), ...
                    app.cfg, app.traceStartedAt, app.traceIntervalStart));
                if interactive
                    [name, folder] = uiputfile("*.csv", "Save trace", file);
                    figure(app.UIFigure);
                    if ~isequal(name, 0) && ~strcmp(fullfile(folder, name), file)
                        chosen = fullfile(folder, name);
                        movefile(file, chosen);
                        file = chosen;
                    end
                end
                [~, base, ext] = fileparts(file);
                app.TraceStatusLabel.Text = append("Saved ", base, ext);
                app.TraceStatusLabel.Tooltip = string(file);
                if missed > 0
                    app.TraceStatusLabel.Text = append(app.TraceStatusLabel.Text, ...
                        sprintf(" (%d missed)", missed));
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            catch ME
                app.TraceStatusLabel.Text = append("Trace not saved: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
            updateButtons(app);
        end

        % The interval dropdown's text: the interval, and its rate when
        % faster than 1 Hz
        function text = intervalText(~, interval)
            text = sprintf('%g s', interval);
            if interval < 1
                text = sprintf('%s (%g Hz)', text, 1 / interval);
            end
        end

        % The dropdown shows the operator's interval, or while a procedure
        % records the interval in force (as an added item when it is not
        % one of the presets).
        function showInterval(app)
            shown = app.traceChoice;
            if procIsActive(app) && app.traceRecording && ~app.tracePaused
                shown = app.traceInterval;
            end
            values = app.cfg.traceIntervalPresets(:)';
            if ~any(abs(values - shown) < 1e-9)
                values = [values shown];
            end
            dd = app.IntervalDropDown;
            if ~isequal(dd.ItemsData, values)
                dd.ItemsData = [];
                dd.Items = arrayfun(@(v) intervalText(app, v), values, ...
                    'UniformOutput', false);
                dd.ItemsData = values;
            end
            [~, idx] = min(abs(values - shown));
            if ~isequal(dd.Value, values(idx))
                dd.Value = values(idx);
            end
        end

        % A procedure is running, waiting for the operator, paused or
        % aborting
        function tf = procIsActive(app)
            tf = ~isempty(app.proc) && any(strcmp(app.proc.state, ...
                {'running', 'waitOperator', 'paused', 'aborting'}));
        end

        function p = blockPath(app, name)
            p = append(app.modelName, "/", name);
        end

        % Write a value to one of the model's Constant blocks
        function setBlock(app, name, value)
            set_param(blockPath(app, name), "Value", num2str(double(value)));
        end

        % Read the output of one of the model's named Gain blocks,
        % through the cached RuntimeObject; a handle that no longer reads
        % is fetched again.
        function value = readBlock(app, name)
            key = matlab.lang.makeValidName(name);
            if isfield(app.rto, key)
                try
                    value = app.rto.(key).OutputPort(1).Data;
                    return
                catch
                end
            end
            rto = get_param(blockPath(app, name), "RuntimeObject");
            if isempty(rto)
                value = 0;
            else
                app.rto.(key) = rto;
                value = rto.OutputPort(1).Data;
            end
        end

        % The values shown on every poll, each block read once. The fault
        % code is read only while the status reports a fault.
        function d = readDisplay(app)
            d.statusCode = readBlock(app, "StatusCode");
            d.position = readBlock(app, "Position");
            d.velocity = readBlock(app, "Velocity");
            d.pressure = readBlock(app, "Pressure");
            d.pressureBlock = readBlock(app, "PressureBlock") ~= 0;
            d.atEndOfTravel = readBlock(app, "AtEndOfTravel") ~= 0;
            d.faultCode = 0;
            if d.statusCode == app.STATUS_FAULT
                d.faultCode = readBlock(app, "FaultCode");
            end
        end

        % Set a label's text only when it changed, which saves redraws
        function setText(~, label, text)
            if ~strcmp(char(label.Text), char(text))
                label.Text = text;
            end
        end

        % Request an operation. RequestedMode is written before CommandSeq
        % so the model sees the new operation when the counter changes.
        function sent = sendCommand(app, op)
            sent = isRunning(app);
            if ~sent
                app.StatusLabel.Text = "Not connected";
                return
            end
            setBlock(app, "RequestedMode", op);
            app.commandSeq = app.commandSeq + 1;
            setBlock(app, "CommandSeq", app.commandSeq);
        end

        % Start a motion operation from its mode button
        function sent = requestOp(app, op)
            sent = sendCommand(app, op);
            if sent
                if op ~= app.activeOp
                    app.opConfirmed = false;
                end
                app.activeOp = op;
                app.opSentAt = tic;
            end
        end

        % Stop the active operation from its mode button
        function stopOp(app)
            sendCommand(app, app.OP_STOP);
            clearOp(app);
            updateButtons(app);
        end

        function clearOp(app)
            app.activeOp = app.OP_STOP;
            app.activeJogDir = 0;
            app.activeTarget = NaN;
            app.opConfirmed = false;
            setSpeedCap(app, NaN);
            if app.trackActive
                endTrack(app);
            end
        end

        % Stop playing the track table: TrackMode back to 0, so the model
        % passes the Constant targets through again.
        function endTrack(app)
            app.trackActive = false;
            try
                setBlock(app, "TrackMode", 0);
            catch
            end
        end

        % Replace the model workspace's trackTable. The variable-tuning
        % API of the simulation is tried first; without it the variable
        % is assigned and the model updated. The first update is timed on
        % the status line.
        function setTrackTable(app, tbl)
            t0 = tic;
            try
                setVariable(app.Simulation, "trackTable", tbl, ...
                    "Workspace", char(app.modelName));
                path = "tuned";
            catch
                assignin(get_param(char(app.modelName), "ModelWorkspace"), ...
                    "trackTable", tbl);
                set_param(char(app.modelName), "SimulationCommand", "update");
                path = "model update";
            end
            if ~app.trackTableTimed
                app.trackTableTimed = true;
                app.procNote = sprintf( ...
                    "Track table set in %.2f s (%s, %d values)", toc(t0), path, numel(tbl));
                app.StatusLabel.Text = app.procNote;
            end
        end

        % The procedure step's speed cap (mm/s, NaN for none). The
        % operator's lower speed still wins. While the model runs the
        % speed sent to the slave follows.
        function setSpeedCap(app, cap)
            changed = ~isequaln(app.procSpeedCap, cap);
            app.procSpeedCap = cap;
            if changed
                showSpeed(app);
                if isRunning(app)
                    setBlock(app, "PositionSpeed", effectiveSpeed(app));
                end
            end
        end

% Follow the active operation through the model's status with the
        % rule the procedure engine shares (opTrack): confirmed once its
        % running status appears, ended when that status goes.
        function trackOp(app, statusCode)
            if app.activeOp == app.OP_STOP
                return
            end
            [app.opConfirmed, ended] = opTrack(app.activeOp, app.opConfirmed, ...
                statusCode, toc(app.opSentAt), app.cfg);
            if ended
                clearOp(app);
            end
        end

        % The model is running, the slave link is up and E-STOP is not
        % requested or active: motion may be commanded.
        function tf = motionAllowed(app)
            tf = isRunning(app) && app.linkReady && ~app.ESTOP.Value && ...
                ~app.estopActive;
        end

        % Mode button look: background shows the operation's state (idle,
        % starting, running), text shows what pressing it does.
        function setModeButton(app, button, isActive, idleText, activeText)
            if ~isActive
                button.BackgroundColor = app.COLOR_IDLE;
                button.Text = idleText;
            else
                if app.opConfirmed
                    button.BackgroundColor = app.COLOR_ACTIVE;
                else
                    button.BackgroundColor = app.COLOR_STARTING;
                end
                button.Text = activeText;
            end
        end

        function updateButtons(app)
            connected = isRunning(app);
            if connected
                app.ConnectToCompressorButton.Text = "Disconnect";
            else
                app.ConnectToCompressorButton.Text = "Connect to Compressor";
            end

            % Mode buttons act only in manual control, connected with the
            % slave link up and out of E-STOP. Manual control and the
            % procedure exclude each other: manual control is not available
            % while a procedure is active (paused included), and the
            % procedure starts only out of manual control, idle.
            canMove = app.ManualControlButton.Value && motionAllowed(app);
            procActive = ~isempty(app.proc) && procView(app.proc).active;
            app.ManualControlButton.Enable = ~procActive;
            app.StartProcedureButton.Enable = ~isempty(app.proc) && ...
                ~app.ManualControlButton.Value && motionAllowed(app) && ...
                ~procActive && app.activeOp == app.OP_STOP && ...
                app.lastStatusCode == 0;
            buttons = {app.HomeButton, app.GoHomeButton, app.PositionGoButton, ...
                app.PressureGoButton, app.JogUpButton, app.JogDownButton};
            for k = 1:numel(buttons)
                buttons{k}.Enable = canMove;
            end

            op = app.activeOp;
            setModeButton(app, app.HomeButton, op == app.OP_HOME, ...
                "Home", "Stop");
            setModeButton(app, app.GoHomeButton, op == app.OP_GO_HOME, ...
                "Go Home", "Stop");

            % A changed target makes Go retarget the running operation
            % instead of stopping it.
            text = "Stop";
            if app.TargetPositionSpinner.Value ~= app.activeTarget
                text = "Go";
            end
            setModeButton(app, app.PositionGoButton, op == app.OP_POSITION, ...
                "Go", text);

            text = "Stop";
            if ~sameTarget(app, pressureTargetAtm(app), app.activeTarget)
                text = "Go";
            end
            setModeButton(app, app.PressureGoButton, op == app.OP_PRESSURE, ...
                "Go", text);

            jogUp = op == app.OP_JOG && app.activeJogDir > 0;
            jogDown = op == app.OP_JOG && app.activeJogDir < 0;
            setModeButton(app, app.JogUpButton, jogUp, "Jog Up", "Stop Jog");
            setModeButton(app, app.JogDownButton, jogDown, "Jog Down", "Stop Jog");
            app.JogUpButton.Value = jogUp;
            app.JogDownButton.Value = jogDown;

            % Record is available while connected, and while recording so
            % a recording can always be ended.
            app.RecordButton.Enable = connected || app.traceRecording;
            app.RecordButton.Value = app.traceRecording;
            app.TraceRecordMenu.Enable = app.RecordButton.Enable;
            app.TracePlotMenu.Enable = app.PlotTraceButton.Enable;
            app.TraceLiveMenu.Enable = app.LivePlotButton.Enable;
            app.ProcedureMenu.Enable = ~procActive;
            app.IntervalDropDown.Enable = (connected || app.traceRecording) && ~procActive;
            showInterval(app);
            if app.traceRecording
                app.RecordButton.BackgroundColor = app.COLOR_STOP;
                app.RecordButton.FontColor = [1 1 1];
                app.RecordButton.Text = "Stop recording";
                app.TraceRecordMenu.Text = "Stop recording";
            else
                app.RecordButton.BackgroundColor = app.COLOR_IDLE;
                app.RecordButton.FontColor = [0 0 0];
                app.RecordButton.Text = "Record";
                app.TraceRecordMenu.Text = "Record";
            end

            % STOP shows the master's E-STOP state, and a request the
            % master has not yet acted on (or cannot, while disconnected).
            if app.estopActive
                app.ESTOP.BackgroundColor = app.COLOR_ESTOP;
                app.ESTOP.FontColor = app.COLOR_STOP;
                app.ESTOP.Text = ["E-STOP"; "Release"];
            elseif app.ESTOP.Value
                app.ESTOP.BackgroundColor = app.COLOR_STOP_REQ;
                app.ESTOP.FontColor = [1 1 1];
                app.ESTOP.Text = ["STOP"; "requested"];
            else
                app.ESTOP.BackgroundColor = app.COLOR_STOP;
                app.ESTOP.FontColor = [1 1 1];
                app.ESTOP.Text = "STOP";
            end
        end

        % Timer callback. An error here would stop the timer and freeze the
        % display, so it is reported on the status line instead.
        function pollModel(app)
            if ~isvalid(app)
                return
            end
            t0 = tic;
            try
                updateFromModel(app);
            catch ME
                app.StatusLabel.Text = append("Display update failed: ", ME.message);
            end
            st = app.PollStats;
            st.last = toc(t0);
            st.mean = (st.mean * st.n + st.last) / (st.n + 1);
            st.n = st.n + 1;
            st.max = max(st.max, st.last);
            st.overruns = st.overruns + (st.last > app.cfg.displayPeriod);
            app.PollStats = st;
        end

        function updateFromModel(app)
            if ~isRunning(app)
                if app.wasRunning
                    % A lost Modbus connection stops the model with an error;
                    % the DAQ outputs are at their final values (safety relay off).
                    app.StatusLabel.Text = append("Model stopped (", ...
                        string(app.Simulation.Status), "). Check the slave link, then Connect.");
                    app.wasRunning = false;
                end
                app.estopActive = false;
                app.linkReady = false;
                app.rto = struct();
                showPressureBlock(app, false);
                app.flagAtEnd = false;
                clearOp(app);
                app.procIn.connected = false;
                stepProcedure(app);
                finishRecording(app, false);
                updateButtons(app);
                return
            end
            app.wasRunning = true;

            d = readDisplay(app);
            statusCode = d.statusCode;
            app.estopActive = statusCode == app.STATUS_ESTOP;
            app.linkReady = statusCode ~= app.STATUS_LINK_LOST && ...
                statusCode ~= app.STATUS_LINK_WAIT;
            trackOp(app, statusCode);
            app.lastPosition = d.position;
            refreshVolume(app);
            showPressureBlock(app, d.pressureBlock);
            app.flagAtEnd = d.atEndOfTravel;
            setText(app, app.PressureValue, readoutText( ...
                app.pressureInfo.toUnit(d.pressure), app.pressureInfo.decimals, 0));
            setText(app, app.PositionValue, readoutText(app.cfg.posEOT - d.position, 2, 0));
            streamTrace(app);
            updateLivePlot(app);
            % Positive while the piston moves toward the end of travel
            setText(app, app.FlowValue, readoutText( ...
                mLPerMm(app.cfg) * d.velocity, 2, app.cfg.flowDeadband));

            faulted = statusCode == app.STATUS_FAULT;
            if faulted
                app.FaultActiveLamp.Color = [1 0 0];
            else
                app.FaultActiveLamp.Color = [0.6 0.6 0.6];
            end
            if app.MixerButton.Value && ~app.ESTOP.Value
                app.MixerOnLamp.Color = [0 1 0];
            else
                app.MixerOnLamp.Color = [0.6 0.6 0.6];
            end

            setText(app, app.StatusLabel, statusText(app, statusCode, d.faultCode));

            % The procedure engine takes the same reads as the readouts. The
            % buttons follow its state, so they are refreshed after it.
            app.procIn = struct('connected', true, 'statusCode', statusCode, ...
                'faultCode', d.faultCode, ...
                'estop', app.ESTOP.Value || app.estopActive, ...
                'pressure', d.pressure, ...
                'volume', sweptVolume(d.position, app.cfg), ...
                'pressureBlock', d.pressureBlock, ...
                'atEndOfTravel', d.atEndOfTravel, ...
                'positionFromEOT', app.cfg.posEOT - d.position, ...
                'positionSpeed', effectiveSpeed(app));
            stepProcedure(app);
            updateButtons(app);

            % Ask the operator before resetting a fault. The dialog opens
            % when a fault is first latched and again after a failed reset.
            if faulted && ~app.resetDialogOpen && ...
                    app.lastStatusCode ~= app.STATUS_FAULT
                askFaultReset(app, d.faultCode);
            end
            app.lastStatusCode = statusCode;
        end

        % Non-blocking, so polling continues while the dialog is open
        function askFaultReset(app, faultCode)
            app.resetDialogOpen = true;
            message = sprintf("Slave fault %d: %s.\n\nCorrect the cause, then reset.", ...
                faultCode, faultName(app, faultCode));
            uiconfirm(app.UIFigure, message, "Fault", ...
                "Options", ["Reset fault", "Leave faulted"], ...
                "DefaultOption", 2, "CancelOption", 2, "Icon", "error", ...
                "CloseFcn", @(~, event) faultDialogClosed(app, event));
        end

        function faultDialogClosed(app, event)
            app.resetDialogOpen = false;
            if event.SelectedOption == "Reset fault"
                sendCommand(app, app.OP_RESET);
            end
        end

        % Operator entry ranges from masterConfig, so the app and the
        % master logic share one set of limits.
        function applyConfig(app)
            here = fileparts(mfilename("fullpath"));
            if exist("masterConfig", "file") ~= 2
                addpath(fullfile(here, "logic"));
            end
            app.cfg = masterConfig();
            if exist(app.cfg.procedure, "file") ~= 2
                addpath(fullfile(here, "procedure"));
            end
            if exist("traceDrain", "file") ~= 2
                addpath(fullfile(here, "trace"));
            end

            % Manual targets are distances from the end of travel, within
            % the master's soft limits and not past the end of travel. The
            % initial target is the retracted end, away from compression.
            limits = [max(app.cfg.posEOT - app.cfg.posMax, 0), ...
                app.cfg.posEOT - app.cfg.posMin];
            setSpinnerRange(app, app.TargetPositionSpinner, limits);
            app.TargetPositionSpinner.Value = limits(2);

            % Pressure targets: the spinner shows the display unit (the one
            % chosen last time), from ambient up to the slave's highest
            % accepted target.
            app.pressureUnit = 'atm';
            app.pressureInfo = pressureUnitInfo('atm', app.cfg);
            setPressureUnit(app, savedPressureUnit(app));

            % One operator speed in percent of cfg.speedMax: a button per
            % preset (the count comes from the configuration) and the
            % typed field, starting at the default.
            app.SpeedPresetButtons = matlab.ui.control.Button.empty;
            for k = 1:numel(app.cfg.speedPresets)
                p = app.cfg.speedPresets(k);
                app.SpeedPresetButtons(k) = uibutton(app.ControlsPanel, 'push');
                app.SpeedPresetButtons(k).ButtonPushedFcn = ...
                    createCallbackFcn(app, @SpeedPresetPushed, true);
                app.SpeedPresetButtons(k).Position = [990 + (k - 1) * 35, 9, 33, 22];
                app.SpeedPresetButtons(k).Text = num2str(p);
                app.SpeedPresetButtons(k).UserData = p;
                app.SpeedPresetButtons(k).FontSize = 11;
                app.SpeedPresetButtons(k).Enable = 'off';
                app.SpeedPresetButtons(k).Tooltip = {sprintf('%g %% (%.2f mm/s)', ...
                    p, speedFromPercent(p, app.cfg))};
            end
            setSpeedPercent(app, app.cfg.speedDefault);

            % Trace stream state, so the live plot can open before the
            % first connection; the dropdown starts at the default interval.
            app.traceAligner = traceAlignNew();
            app.traceLive = traceLiveBuffer(app.cfg);
            app.traceChoice = app.cfg.traceIntervalDefault;
            showInterval(app);

            createProcPools(app);
            loadProcedure(app);
            buildProcedureMenu(app);
        end

        % The procedure panel's checkbox and parameter-field pools, sized
        % by the configuration. Each callback finds its index in UserData.
        function createProcPools(app)
            panel = app.ProcedurePanel;
            nChecks = app.cfg.procMaxChecks;
            app.ProcChecks = matlab.ui.control.CheckBox.empty;
            for k = 1:nChecks
                c = uicheckbox(panel);
                c.ValueChangedFcn = createCallbackFcn(app, @ProcCheckChanged, true);
                c.Text = '';
                c.UserData = k;
                c.Visible = 'off';
                c.Position = [214, 160 - 21 * (k - 1), 454, 20];
                app.ProcChecks(k) = c;
            end

            % Two columns of parameters; each label wraps onto two lines.
            nParams = app.cfg.procMaxParams;
            rows = ceil(nParams / 2);
            pitch = min(32, floor(128 / rows));
            app.ProcParamLabels = matlab.ui.control.Label.empty;
            app.ProcParamFields = matlab.ui.control.NumericEditField.empty;
            for k = 1:nParams
                col = ceil(k / rows) - 1;
                row = mod(k - 1, rows);
                x = 214 + col * 228;
                y = 150 - pitch * row;
                l = uilabel(panel);
                l.HorizontalAlignment = 'right';
                l.WordWrap = 'on';
                l.FontSize = 10;
                l.Text = '';
                l.Visible = 'off';
                l.Position = [x, y, 150, 28];
                app.ProcParamLabels(k) = l;
                e = uieditfield(panel, 'numeric');
                e.ValueChangedFcn = createCallbackFcn(app, @ProcParamChanged, true);
                e.UserData = k;
                e.Visible = 'off';
                e.Position = [x + 154, y + 3, 66, 22];
                app.ProcParamFields(k) = e;
            end
        end

        % Read the procedure of the given function (default cfg.procedure)
        % and build the engine state. A procedure that does not validate
        % leaves a loaded one in place, or the panel empty and Start
        % disabled when none is loaded, and puts the reason on the status
        % line.
        function loadProcedure(app, name)
            if nargin < 2
                name = app.cfg.procedure;
            end
            try
                p = procNew(feval(name), app.cfg);
            catch ME
                if isempty(app.proc)
                    app.procDef = [];
                    app.ProcMessageLabel.Text = ['Procedure not loaded: ' ME.message];
                end
                app.StatusLabel.Text = append("Procedure not loaded: ", ME.message);
                return
            end
            app.procName = name;
            app.procDef = p.def;
            app.proc = p;
            renderProcedure(app);
            showProcedureChoice(app);
            app.StatusLabel.Text = append("Procedure loaded: ", name);
        end

        % The Procedure menu: Open... and Reload, then one item per .m file
        % of master-src/procedure, sorted, then the file the operator
        % opened from elsewhere, if it is the loaded one. Only the file
        % items are made again when the list can change; Open... and
        % Reload stay, so no item is deleted from inside its own callback.
        function buildProcedureMenu(app)
            delete(app.ProcedureFileMenus(isvalid(app.ProcedureFileMenus)));
            folder = fullfile(fileparts(mfilename("fullpath")), "procedure");
            files = dir(fullfile(folder, "*.m"));
            names = sort(string({files.name}));
            names = erase(names, ".m");
            if ~isempty(app.procName) && ~any(names == string(app.procName))
                names(end + 1) = string(app.procName);
            end
            app.ProcedureFileMenus = matlab.ui.container.Menu.empty;
            for k = 1:numel(names)
                m = uimenu(app.ProcedureMenu);
                m.Text = char(names(k));
                m.UserData = char(names(k));
                m.Separator = matlab.lang.OnOffSwitchState(k == 1);
                m.MenuSelectedFcn = createCallbackFcn(app, @ProcedureFileSelected, true);
                app.ProcedureFileMenus(k) = m;
            end
            showProcedureChoice(app);
        end

        % Check the item of the loaded procedure
        function showProcedureChoice(app)
            for k = 1:numel(app.ProcedureFileMenus)
                m = app.ProcedureFileMenus(k);
                m.Checked = strcmp(m.UserData, app.procName);
            end
        end

        % Set the operator speed (percent of cfg.speedMax). While the
        % model runs it is written to both blocks that carry it: the jog
        % reference and the speed sent in HR7-8.
        function setSpeedPercent(app, pct)
            pct = min(max(pct, 0), 100);
            app.speedPercent = pct;
            v = speedFromPercent(pct, app.cfg);
            app.SpeedField.Value = pct;
            showSpeed(app);
            for k = 1:numel(app.SpeedPresetButtons)
                if app.SpeedPresetButtons(k).UserData == pct
                    app.SpeedPresetButtons(k).BackgroundColor = app.COLOR_ACTIVE;
                else
                    app.SpeedPresetButtons(k).BackgroundColor = app.COLOR_IDLE;
                end
            end
            if isRunning(app)
                setBlock(app, "PositionSpeed", effectiveSpeed(app));
                setBlock(app, "TargetVelocity", v);
            end
        end

        % The speed label: the speed in force, marked while a procedure
        % step caps it
        function showSpeed(app)
            text = sprintf("%.2f mm/s", effectiveSpeed(app));
            if ~isnan(app.procSpeedCap)
                text = append(text, " (step)");
            end
            setText(app, app.SpeedMmsLabel, text);
        end

        % Volume shown in the selected view
        function refreshVolume(app)
            switch app.volumeView
                case "mixing"
                    v = mixingVolume(app.lastPosition, app.cfg);
                case "sinceZero"
                    v = volumeSinceZero(app);
                otherwise
                    v = sweptVolume(app.lastPosition, app.cfg);
            end
            setText(app, app.VolumeValue, readoutText(v, 1, 0));
        end

        % Swept volume delivered since the operator zeroed it (mL),
        % positive as the piston advances toward the end of travel, like
        % the flow; NaN until zeroed.
        function v = volumeSinceZero(app)
            v = sweptVolume(app.volumeZeroPosition, app.cfg) ...
                - sweptVolume(app.lastPosition, app.cfg);
        end

        function zeroVolume(app)
            app.volumeZeroPosition = app.lastPosition;
            refreshVolume(app);
        end

        % Operator speed in mm/s
        function v = speedMmPerS(app)
            v = speedFromPercent(app.speedPercent, app.cfg);
        end

        % Speed sent for position moves: the operator's, or the running
        % procedure step's cap when that is lower
        function v = effectiveSpeed(app)
            v = speedMmPerS(app);
            if ~isnan(app.procSpeedCap)
                v = min(v, app.procSpeedCap);
            end
        end

        % The pressure unit saved by the last session, 'atm' when none
        function unit = savedPressureUnit(~)
            unit = 'atm';
            try
                saved = getpref("SOS_Gen3_Compressor", "pressureUnit", "atm");
                if any(strcmp(char(saved), {'psi', 'atm', 'MPa'}))
                    unit = char(saved);
                end
            catch
            end
        end

        % Show pressures in the unit: the spinner (its value converted,
        % its limits, step and format), the panel and label titles, the
        % toggle and the procedure panel's pressure fields. The choice is
        % kept for the next session.
        function setPressureUnit(app, unit)
            if ~any(strcmp(unit, {'psi', 'atm', 'MPa'}))
                return
            end
            atm = pressureTargetAtm(app);
            info = pressureUnitInfo(unit, app.cfg);
            maxAtm = min(app.cfg.pressureMax, app.cfg.pressureCeiling - ...
                app.cfg.pressureCeilingHyst) * app.cfg.atmPerMPa + app.cfg.atmAmbient;
            limits = info.toUnit([app.cfg.atmAmbient, maxAtm]);
            spinner = app.TargetPressureSpinner;
            spinner.Limits = [-Inf Inf];
            spinner.Value = min(max(info.toUnit(atm), limits(1)), limits(2));
            spinner.Limits = limits;
            spinner.Step = info.step;
            spinner.ValueDisplayFormat = info.format;
            app.pressureUnit = unit;
            app.pressureInfo = info;
            app.PressurePanel.Title = sprintf('Pressure (%s)', info.label);
            app.TargetPressureatmSpinnerLabel.Text = sprintf('Target (%s)', info.label);
            switch unit
                case 'psi', app.PressureUnitGroup.SelectedObject = app.PressureUnitPsiButton;
                case 'atm', app.PressureUnitGroup.SelectedObject = app.PressureUnitAtmButton;
                otherwise,  app.PressureUnitGroup.SelectedObject = app.PressureUnitMPaButton;
            end
            try
                setpref("SOS_Gen3_Compressor", "pressureUnit", unit);
            catch
            end
            renderProcedure(app);
            updateButtons(app);
        end

        % The spinner's target in atm absolute
        function atm = pressureTargetAtm(app)
            atm = app.pressureInfo.fromUnit(app.TargetPressureSpinner.Value);
        end

        % Two targets in atm are the same one (the spinner holds a
        % converted value)
        function tf = sameTarget(~, a, b)
            tf = abs(a - b) <= 1e-9 * max(1, abs(b));
        end

        % The pressure readout is red while the slave refuses pressurising
        % motion
        function showPressureBlock(app, blocked)
            if blocked ~= app.flagPressureBlock
                app.flagPressureBlock = blocked;
                if blocked
                    app.PressureValue.FontColor = [1 0 0];
                else
                    app.PressureValue.FontColor = [0 0 0];
                end
            end
        end

        % A spinner rejects a value outside its limits and limits that
        % exclude its value, so widen, move the value, then narrow.
        function setSpinnerRange(~, spinner, limits)
            spinner.Limits = [-Inf Inf];
            spinner.Value = min(max(spinner.Value, limits(1)), limits(2));
            spinner.Limits = limits;
        end

        function text = statusText(app, statusCode, faultCode)
            text = statusTextBase(app, statusCode, faultCode);
            if app.speedPercent == 0 && any(statusCode == [2 4 8])
                text = append(text, " (held: speed 0)");
            end
            if app.flagPressureBlock
                text = append(text, " (pressure ceiling: pressurising blocked)");
            end
            if app.flagAtEnd && statusCode == 5
                text = append(text, " (end of travel)");
            end
        end

        function text = statusTextBase(app, statusCode, faultCode)
            switch statusCode
                case 0,  text = "Idle";
                case 1,  text = "Homing";
                case 2,  text = "Moving to position";
                case 3,  text = "At position";
                case 4,  text = "Jogging";
                case 5,  text = "Pressure control";
                case 6,  text = "Stopping";
                case 7,  text = "Starting";
                case 8,  text = "Going home";
                case 9,  text = "Resetting fault";
                case app.STATUS_FAULT
                    text = append("FAULT: ", faultName(app, faultCode));
                case app.STATUS_ESTOP, text = "E-STOP";
                case 30, text = "Slave did not confirm the mode";
                case 32, text = "Fault reset failed";
                case 33, text = "Slave did not report stopped";
                case app.STATUS_LINK_LOST
                    text = "Slave link lost: press Stop once it is back";
                case app.STATUS_LINK_WAIT
                    text = "Waiting for the slave link";
                otherwise
                    text = append("Status ", num2str(statusCode));
            end
        end

        function name = faultName(app, faultCode)
            if faultCode >= 1 && faultCode <= numel(app.faultNames)
                name = app.faultNames(faultCode);
            else
                name = sprintf("code %d", faultCode);
            end
        end

        % Jog in direction, or stop if already jogging that way. The other
        % direction reverses a running jog without leaving the mode.
        function toggleJog(app, direction)
            if app.activeOp == app.OP_JOG && app.activeJogDir == direction
                stopOp(app);
                return
            end
            setBlock(app, "TargetVelocity", speedMmPerS(app));
            setBlock(app, "JogDirection", direction);
            if requestOp(app, app.OP_JOG)
                app.activeJogDir = direction;
            end
            updateButtons(app);
        end

        % One operator action on the procedure engine. A refused action
        % leaves the engine as it was and its reason on the panel until the
        % next accepted action. The engine then steps at once, so an abort
        % sends its stop now and not at the next poll.
        function procAct(app, varargin)
            if isempty(app.proc)
                return
            end
            [app.proc, ok, msg] = procAction(app.proc, varargin{:});
            if ok
                app.procNote = '';
            else
                app.procNote = msg;
            end
            stepProcedure(app);
            updateButtons(app);
        end

        % Advance the procedure engine with the inputs of the last poll and
        % carry out what it asks for. It never blocks: no pause, drawnow or
        % modal dialog.
        function stepProcedure(app)
            if isempty(app.proc)
                return
            end
            if isempty(app.procTick)
                dt = 0;
            else
                dt = toc(app.procTick);
            end
            app.procTick = tic;
            [app.proc, req] = procStep(app.proc, app.procIn, dt);
            % The step number for the trace's procStep column, written on change
            k = 0;
            if procIsActive(app)
                k = app.proc.k;
            end
            if k ~= app.procStepShown && isRunning(app)
                setBlock(app, "ProcStep", k);
                app.procStepShown = k;
            end
            executeProcRequest(app, req);
            renderProcedure(app);
        end

        % Turn the engine's requests into commands, in the engine's order:
        % parameters, mixer, stop, track, operation, volume zero, record
        % interval, end of recording. The operation
        % goes through the same interlock as the manual buttons; when it
        % is not allowed nothing is sent and the engine sees the cause on
        % its next poll.
        function executeProcRequest(app, req)
            if ~isempty(req.params) && isRunning(app)
                names = fieldnames(req.params);
                for k = 1:numel(names)
                    setBlock(app, names{k}, req.params.(names{k}));
                end
            end
            if req.mixer >= 0
                app.MixerButton.Value = req.mixer == 1;
                if isRunning(app)
                    setBlock(app, "Mixer", req.mixer);
                end
            end
            if req.stop
                if isRunning(app)
                    stopOp(app);
                else
                    clearOp(app);
                end
            end
            if ~isempty(req.track)
                executeTrackRequest(app, req.track);
            end
            if req.op >= 0 && motionAllowed(app) && ~app.ManualControlButton.Value
                switch req.op
                    case app.OP_POSITION
                        if strcmp(req.unit, 'mL')
                            % Chamber volume: the dead volume plus the swept
                            % volume, which is the bore area times the
                            % travel left to the end of travel.
                            x = app.cfg.posEOT - ...
                                (req.target - app.cfg.deadVolume) / mLPerMm(app.cfg);
                        else
                            x = app.cfg.posEOT - req.target;
                        end
                        x = min(max(x, app.cfg.posMin), app.cfg.posMax);
                        setBlock(app, "TargetPosition", x);
                        setSpeedCap(app, req.speed);
                        setBlock(app, "PositionSpeed", effectiveSpeed(app));
                        if requestOp(app, app.OP_POSITION)
                            app.activeTarget = app.cfg.posEOT - x;
                        end
                    case app.OP_PRESSURE
                        setSpeedCap(app, NaN);
                        setBlock(app, "TargetPressure", req.target);
                        if requestOp(app, app.OP_PRESSURE)
                            app.activeTarget = req.target;
                        end
                    case app.OP_HOME
                        % Re-homing changes the position frame, so the zero
                        % of the incremental volume no longer holds.
                        setSpeedCap(app, NaN);
                        if requestOp(app, app.OP_HOME)
                            app.volumeZeroPosition = NaN;
                            refreshVolume(app);
                        end
                    case app.OP_GO_HOME
                        setSpeedCap(app, NaN);
                        requestOp(app, app.OP_GO_HOME);
                end
                updateButtons(app);
            end
            if req.zeroVolume
                zeroVolume(app);
            end
            if ~isempty(req.record)
                executeRecordRequest(app, req.record);
            end
            if req.recordEnd
                endProcedureRecording(app);
            end
        end

        % A track step's start or end. An end is carried out whenever the
        % model runs, so no stale series plays in the next pressure or
        % position operation; a start only where an operation is allowed.
        % The table is written first, then the mode, then the sequence
        % number that restarts the playback.
        function executeTrackRequest(app, track)
            if strcmp(track.action, 'end')
                endTrack(app);
                return
            end
            if ~(motionAllowed(app) && ~app.ManualControlButton.Value)
                return
            end
            try
                series = track.series(:)';
                if strcmp(track.mode, 'position')
                    code = app.OP_POSITION;
                    series = min(max(app.cfg.posEOT - series, app.cfg.posMin), ...
                        app.cfg.posMax);
                else
                    code = app.OP_PRESSURE;
                end
                tbl = [series, series(end) * ones(1, app.cfg.trackTableLength - numel(series))];
                setTrackTable(app, tbl);
                setBlock(app, "TrackMode", code);
                app.trackSeq = app.trackSeq + 1;
                setBlock(app, "TrackSeq", app.trackSeq);
                app.trackActive = true;
            catch ME
                app.procNote = append("Track not started: ", ME.message);
                app.StatusLabel.Text = app.procNote;
            end
        end

        % The procedure's record interval (s), or 'off'. A number starts a
        % procedure-owned recording unless the operator stopped one in
        % this run, changes the interval of a recording that runs, or
        % resumes a paused one. 'off' pauses a recording.
        function executeRecordRequest(app, record)
            if ischar(record) || isstring(record)
                if app.traceRecording && ~app.tracePaused
                    pauseRecording(app);
                end
                return
            end
            app.procRecord = record;
            if app.traceRecording
                if app.tracePaused
                    resumeRecording(app, record);
                else
                    setRecordInterval(app, record);
                end
            elseif ~app.traceDeclined && isRunning(app)
                try
                    startRecording(app, record, 'procedure');
                catch ME
                    app.traceRecording = false;
                    app.TraceStatusLabel.Text = append("Recording not started: ", ME.message);
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            end
        end

        % The run ended. A recording the procedure started is saved
        % without a dialog; one the operator started goes on at the
        % operator's interval.
        function endProcedureRecording(app)
            if ~app.traceRecording
                return
            end
            if strcmp(app.traceOwner, 'procedure')
                finishRecording(app, false);
            elseif app.tracePaused
                resumeRecording(app, app.traceChoice);
            else
                setRecordInterval(app, app.traceChoice);
            end
        end

        % Copy what procView says into the panel's components. A property
        % is written only when it changed, which saves redraws.
        function renderProcedure(app)
            if isempty(app.proc)
                return
            end
            v = procView(app.proc, app.pressureInfo);
            def = app.procDef;

            items = v.list(:)';
            if ~isequal(app.ProcStepList.Items, items)
                app.ProcStepList.Items = items;
                app.ProcStepList.ItemsData = 1:numel(items);
            end
            if isempty(app.ProcStepList.Value) || app.ProcStepList.Value ~= v.listIndex
                app.ProcStepList.Value = v.listIndex;
            end
            setText(app, app.ProcHeadingLabel, v.heading);
            title = char(v.title);
            if ~isempty(v.stepProgress)
                title = [title '  (' v.stepProgress ')'];
            end
            setText(app, app.ProcTitleLabel, title);
            if ~isequal(app.ProcTextArea.Value, {v.text})
                app.ProcTextArea.Value = {v.text};
            end

            for k = 1:numel(app.ProcChecks)
                box = app.ProcChecks(k);
                if k <= numel(v.checks.labels)
                    setText(app, box, v.checks.labels{k});
                    box.Value = v.checks.values(k);
                    box.Enable = v.checks.enable;
                    box.Visible = true;
                else
                    box.Visible = false;
                end
            end

            % The parameters show during a param step, editable only while
            % it waits for the operator.
            showParams = false;
            if v.active && app.proc.k >= 1
                showParams = strcmp(def.steps(app.proc.k).type, 'param');
            end
            for k = 1:numel(app.ProcParamFields)
                if k <= numel(v.params.keys)
                    setText(app, app.ProcParamLabels(k), v.params.labels{k});
                    if ~strcmp(app.ProcParamFields(k).ValueDisplayFormat, v.params.formats{k})
                        app.ProcParamFields(k).ValueDisplayFormat = v.params.formats{k};
                    end
                    if app.ProcParamFields(k).Value ~= v.params.values(k)
                        app.ProcParamFields(k).Value = v.params.values(k);
                    end
                    app.ProcParamFields(k).Enable = v.params.editable(k);
                    app.ProcParamLabels(k).Enable = v.params.editable(k);
                    app.ProcParamFields(k).Visible = showParams;
                    app.ProcParamLabels(k).Visible = showParams;
                else
                    app.ProcParamFields(k).Visible = false;
                    app.ProcParamLabels(k).Visible = false;
                end
            end

            app.ProcConfirmButton.Visible = v.confirm.visible;
            app.ProcConfirmButton.Enable = v.confirm.enable;
            setText(app, app.ProcConfirmButton, v.confirm.label);
            app.ProcRepeatButton.Visible = v.repeat.visible;
            setText(app, app.ProcRepeatButton, v.repeat.label);
            app.ProcPauseButton.Enable = v.pause.enable;
            setText(app, app.ProcPauseButton, v.pause.label);
            app.ProcAbortButton.Enable = v.abortEnable;

            setText(app, app.ProcTimerLabel, [def.timerLabel ': ' v.timerText]);
            setText(app, app.ProcVolumeLabel, [def.volumeLabel ': ' v.volumeText]);
            setText(app, app.ProcPressureTargetLabel, ...
                ['Held pressure: ' v.pressureTargetText]);
            text = v.stateText;
            note = v.message;
            if ~isempty(app.procNote)
                note = app.procNote;
            end
            if ~isempty(note)
                text = [text '. ' note];
            end
            setText(app, app.ProcMessageLabel, text);
            app.ProcMessageLabel.Tooltip = {text};
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.modelName = string(app.Simulation.ModelName);

            applyConfig(app);

            app.updateTimer = timer( ...
                ExecutionMode="fixedRate", ...
                Period=app.cfg.displayPeriod, ...
                BusyMode="drop", ...
                TimerFcn=@(~,~) pollModel(app));
            start(app.updateTimer);
        end

        % Button pushed function: ConnectToCompressorButton
        function ConnectButtonPushed(app, event)
            if isRunning(app)
                disconnect(app);
                return
            end

            app.StatusLabel.Text = "Connecting to Simulink...";
            drawnow;

            try
                if isRunning(app)
                    app.StatusLabel.Text = "Model is already running";
                else
                    app.traceReader = [];
                    start(app.Simulation);
                    app.rto = struct();
                    if isRunning(app)
                        app.commandSeq = 0;
                        setBlock(app, "RequestedMode", app.OP_STOP);
                        setBlock(app, "CommandSeq", app.commandSeq);
                        setBlock(app, "ESTOP", app.ESTOP.Value);
                        setBlock(app, "Mixer", app.MixerButton.Value);
                        setBlock(app, "PositionSpeed", speedMmPerS(app));
                        setBlock(app, "TargetVelocity", speedMmPerS(app));
                        setBlock(app, "ProcStep", 0);
                        setBlock(app, "TrackMode", 0);
                        setBlock(app, "TrackSeq", 0);
                        app.procStepShown = 0;
                        app.trackSeq = 0;
                        app.trackActive = false;
                        app.traceReader = traceStart(readBlock(app, "TraceRing"), false);
                        app.traceAligner = traceAlignNew();
                        app.traceLive = traceLiveBuffer(app.cfg);
                        app.StatusLabel.Text = "Connected to Simulink";
                    else
                        app.StatusLabel.Text = append(...
                            "Simulink Status: ", string(app.Simulation.Status));
                    end
                end
            catch ME
                app.StatusLabel.Text = append("Model unable to start: ", ME.message);
            end
            updateButtons(app);
        end

        % Value changed function: ManualControlButton
        function ManualControlButtonPressed(app, event)
            value = app.ManualControlButton.Value;
            app.TargetPositionSpinner.Enable = value;
            app.TargetPositionmmSpinnerLabel.Enable = value;
            app.TargetPressureSpinner.Enable = value;
            app.TargetPressureatmSpinnerLabel.Enable = value;
            app.SpeedField.Enable = value;
            app.SpeedLabel.Enable = value;
            app.SpeedMmsLabel.Enable = value;
            for k = 1:numel(app.SpeedPresetButtons)
                app.SpeedPresetButtons(k).Enable = value;
            end
            app.MixerButton.Enable = value;
            updateButtons(app);
        end

        % Button pushed function: HomeButton
        function HomeButtonPushed(app, event)
            if app.activeOp == app.OP_HOME
                stopOp(app);
            else
                % Re-homing changes the position frame, so the zero of the
                % incremental volume no longer holds.
                if requestOp(app, app.OP_HOME)
                    app.volumeZeroPosition = NaN;
                    refreshVolume(app);
                end
                updateButtons(app);
            end
        end

        % Button pushed function: GoHomeButton
        function GoHomeButtonPushed(app, event)
            if app.activeOp == app.OP_GO_HOME
                stopOp(app);
            else
                requestOp(app, app.OP_GO_HOME);
                updateButtons(app);
            end
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            % The model keeps driving the axis without the app, so it is
            % stopped before the app closes.
            if isRunning(app)
                choice = uiconfirm(app.UIFigure, ...
                    "The compressor is connected. Stop it and disconnect before closing?", ...
                    "Close", "Options", ["Stop and close", "Cancel"], ...
                    "DefaultOption", 1, "CancelOption", 2, "Icon", "warning");
                if choice ~= "Stop and close"
                    return
                end
                disconnect(app);
            end
            finishRecording(app, false);
            closeLivePlot(app);

            if ~isempty(app.updateTimer) && isvalid(app.updateTimer)
                stop(app.updateTimer);
                delete(app.updateTimer);
            end

            delete(app)
        end

        % Button pushed function: StartProcedureButton
        function StartProcedureButtonPushed(app, event)
            if isempty(app.procDef)
                return
            end
            % A fresh run; the parameter values entered before are kept.
            app.proc = procNew(app.procDef, app.cfg, app.proc.values);
            app.procNote = '';
            app.traceDeclined = false;
            app.procRecord = [];
            procAct(app, 'start');
        end

        % Button pushed function: ProcConfirmButton
        function ProcConfirmButtonPushed(app, event)
            procAct(app, 'confirm');
        end

        % Button pushed function: ProcRepeatButton
        function ProcRepeatButtonPushed(app, event)
            procAct(app, 'repeat');
        end

        % Button pushed function: ProcPauseButton
        function ProcPauseButtonPushed(app, event)
            if strcmp(app.proc.state, 'paused')
                procAct(app, 'resume');
            else
                procAct(app, 'pause');
            end
        end

        % Button pushed function: ProcAbortButton
        % No confirmation: stopping is never delayed.
        function ProcAbortButtonPushed(app, event)
            procAct(app, 'abort');
        end

        % Value changed function: ProcChecks
        function ProcCheckChanged(app, event)
            procAct(app, 'check', event.Source.UserData, event.Source.Value);
        end

        % Value changed function: ProcParamFields
        % A pressure field holds the display unit: its range is checked
        % here in that unit, and the engine gets atm absolute.
        function ProcParamChanged(app, event)
            k = event.Source.UserData;
            q = app.procDef.params(k);
            value = event.Source.Value;
            if strcmp(q.kind, 'pressure')
                info = app.pressureInfo;
                v = procView(app.proc, info);
                lo = v.params.mins(k);
                hi = v.params.maxs(k);
                if value < lo || value > hi
                    app.procNote = sprintf(['%s must be between ' info.format ...
                        ' and ' info.format ' %s'], q.label, lo, hi, info.label);
                    renderProcedure(app);       % puts the accepted value back
                    return
                end
                value = info.fromUnit(value);
            end
            procAct(app, 'setParam', q.key, value);
        end

        % Value changed function: ProcStepList
        % The list only shows the procedure: put the selection back.
        function ProcStepListChanged(app, event)
            renderProcedure(app);
        end

        % Button pushed function: PositionGoButton
        function PositionGoButtonPushed(app, event)
            target = app.TargetPositionSpinner.Value;
            if app.activeOp == app.OP_POSITION && target == app.activeTarget
                stopOp(app);
                return
            end
            setBlock(app, "TargetPosition", app.cfg.posEOT - target);
            setBlock(app, "PositionSpeed", speedMmPerS(app));
            if requestOp(app, app.OP_POSITION)
                app.activeTarget = target;
            end
            updateButtons(app);
        end

        % Button pushed function: PressureGoButton
        function PressureGoButtonPushed(app, event)
            target = pressureTargetAtm(app);
            if app.activeOp == app.OP_PRESSURE && sameTarget(app, target, app.activeTarget)
                stopOp(app);
                return
            end
            setBlock(app, "TargetPressure", target);
            if requestOp(app, app.OP_PRESSURE)
                app.activeTarget = target;
            end
            updateButtons(app);
        end

        % Value changed function: TargetPositionSpinner, TargetPressureSpinner
        function TargetSpinnerChanged(app, event)
            updateButtons(app);
        end

        % Button pushed function: SpeedPresetButtons
        % The model reads both speed blocks continuously, so a running
        % operation takes the new speed at once.
        function SpeedPresetPushed(app, event)
            setSpeedPercent(app, event.Source.UserData);
        end

        % Selection changed function: PressureUnitGroup
        function PressureUnitChanged(app, event)
            setPressureUnit(app, event.NewValue.Tag);
        end

        % Selection changed function: VolumeViewGroup
        function VolumeViewChanged(app, event)
            app.volumeView = string(event.NewValue.Tag);
            switch app.volumeView
                case "mixing",    app.VolumePanel.Title = 'Mixing volume (mL)';
                case "sinceZero", app.VolumePanel.Title = 'Volume since zero (mL)';
                otherwise,        app.VolumePanel.Title = 'Swept volume (mL)';
            end
            app.VolumeZeroButton.Enable = app.volumeView == "sinceZero";
            refreshVolume(app);
        end

        % Button pushed function: VolumeZeroButton
        function VolumeZeroButtonPushed(app, event)
            zeroVolume(app);
        end

        % Value changed function: SpeedField
        function SpeedFieldChanged(app, event)
            setSpeedPercent(app, app.SpeedField.Value);
        end

        % Value changed function: JogDownButton
        function JogDownButtonPushed(app, event)
            toggleJog(app, -1);
        end

        % Value changed function: JogUpButton
        function JogUpButtonPushed(app, event)
            toggleJog(app, 1);
        end

        % Value changed function: MixerButton
        function MixerButtonPushed(app, event)
            if isRunning(app)
                setBlock(app, "Mixer", app.MixerButton.Value);
            end
        end

        % Value changed function: ESTOP
        function ESTOPPushed(app, event)
            % Pressing STOP also sends a Stop command, which is what clears
            % a lost slave link once the link is back.
            if isRunning(app)
                setBlock(app, "ESTOP", app.ESTOP.Value);
                if app.ESTOP.Value
                    sendCommand(app, app.OP_STOP);
                end
            end
            if app.ESTOP.Value
                clearOp(app);
            end
            updateButtons(app);
        end

        % Value changed function: RecordButton
        function RecordButtonChanged(app, event)
            if app.RecordButton.Value
                if ~isRunning(app)
                    app.RecordButton.Value = false;
                    app.TraceStatusLabel.Text = "Not connected";
                    app.TraceStatusLabel.FontColor = [0 0 0];
                    return
                end
                % During a run the recording starts at the procedure's last
                % record interval; it is the operator's and outlives the run.
                interval = app.IntervalDropDown.Value;
                if procIsActive(app) && ~isempty(app.procRecord)
                    interval = app.procRecord;
                end
                try
                    startRecording(app, interval, 'operator');
                catch ME
                    app.traceRecording = false;
                    app.TraceStatusLabel.Text = append("Recording not started: ", ME.message);
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            else
                % Stopping during a run keeps the procedure from starting
                % another recording in it.
                if procIsActive(app)
                    app.traceDeclined = true;
                end
                finishRecording(app, true);
            end
            updateButtons(app);
        end

        % Menu selected function: TraceRecordMenu
        % The same path as pressing the Record button.
        function TraceRecordMenuSelected(app, event)
            app.RecordButton.Value = ~app.traceRecording;
            RecordButtonChanged(app, event);
        end

        % Menu selected function: TraceLiveMenu
        function TraceLiveMenuSelected(app, event)
            LivePlotButtonPushed(app, event);
        end

        % Menu selected function: TracePlotMenu
        function TracePlotMenuSelected(app, event)
            PlotTraceButtonPushed(app, event);
        end

        % Menu selected function: ProcedureFileMenus
        function ProcedureFileSelected(app, event)
            loadProcedure(app, event.Source.UserData);
        end

        % Menu selected function: ProcedureOpenMenu
        % Any .m file; its folder joins the path and the function is
        % loaded by name.
        function ProcedureOpenSelected(app, event)
            [name, folder] = uigetfile("*.m", "Open a procedure", ...
                append(fullfile(fileparts(mfilename("fullpath")), "procedure"), filesep));
            figure(app.UIFigure);
            if isequal(name, 0)
                return
            end
            addpath(folder);
            [~, fn] = fileparts(name);
            loadProcedure(app, fn);
            buildProcedureMenu(app);
        end

        % Menu selected function: ProcedureReloadMenu
        % The current choice again, for a file just edited; the SOP when
        % none is loaded.
        function ProcedureReloadSelected(app, event)
            name = app.procName;
            if isempty(name)
                name = app.cfg.procedure;
            end
            loadProcedure(app, name);
            buildProcedureMenu(app);
        end

        % Value changed function: IntervalDropDown
        % The choice is the operator's; it applies to a running recording
        % at once, except while a procedure owns the interval.
        function IntervalDropDownChanged(app, event)
            if procIsActive(app)
                showInterval(app);
                return
            end
            app.traceChoice = app.IntervalDropDown.Value;
            if app.traceRecording
                try
                    setRecordInterval(app, app.traceChoice);
                catch ME
                    app.TraceStatusLabel.Text = append("Interval not changed: ", ME.message);
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            end
            updateButtons(app);
        end

        % Button pushed function: LivePlotButton
        % The plot reads the stream the app drains on every poll, so it
        % shows the last cfg.livePlotSeconds whether or not Record is on.
        function LivePlotButtonPushed(app, event)
            if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig)
                figure(app.liveFig.fig);
                return
            end
            try
                app.liveFig = traceLiveNew('on');
                set(app.liveFig.fig, 'CloseRequestFcn', @(~, ~) closeLivePlot(app));
                app.livePolls = 0;
                traceLiveUpdate(app.liveFig, app.traceLive);
            catch ME
                app.liveFig = [];
                app.TraceStatusLabel.Text = append("Live plot failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
        end

        % Button pushed function: PlotTraceButton
        function PlotTraceButtonPushed(app, event)
            [name, folder] = uigetfile("*.csv", "Plot a trace", ...
                append(capturesFolder(app), filesep));
            figure(app.UIFigure);
            if isequal(name, 0)
                return
            end
            try
                tracePlot(fullfile(folder, name));
                app.TraceStatusLabel.Text = append("Plotted ", name);
                app.TraceStatusLabel.FontColor = [0 0 0];
            catch ME
                app.TraceStatusLabel.Text = append("Plot failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % A titled panel holding one large numeric value
        function [panel, value] = createReadout(app, title, position, fontSize)
            panel = uipanel(app.UIFigure);
            panel.Title = title;
            panel.FontWeight = 'bold';
            panel.Position = position;
            value = uilabel(panel);
            value.HorizontalAlignment = 'center';
            value.FontSize = fontSize;
            value.FontWeight = 'bold';
            value.Position = [5 10 position(3) - 10 position(4) - 45];
            value.Text = '--';
        end

        % The volume panel's view selector and Zero button
        function createVolumeControls(app)
            app.VolumeValue.Position = [5 38 160 62];
            app.VolumeValue.FontSize = 36;

            app.VolumeViewGroup = uibuttongroup(app.VolumePanel);
            app.VolumeViewGroup.BorderType = 'none';
            app.VolumeViewGroup.SelectionChangedFcn = ...
                createCallbackFcn(app, @VolumeViewChanged, true);
            app.VolumeViewGroup.Position = [4 4 122 30];

            app.VolumeSweptButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeSweptButton.Text = 'Swept';
            app.VolumeSweptButton.Tag = 'swept';
            app.VolumeSweptButton.FontSize = 10;
            app.VolumeSweptButton.Tooltip = {'Swept volume: the saline still deliverable, without the dead volume'};
            app.VolumeSweptButton.Position = [2 4 38 22];

            app.VolumeMixingButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeMixingButton.Text = 'Mix';
            app.VolumeMixingButton.Tag = 'mixing';
            app.VolumeMixingButton.FontSize = 10;
            app.VolumeMixingButton.Tooltip = {'Mixing volume: the swept volume plus the dead volume that takes part in mixing'};
            app.VolumeMixingButton.Position = [42 4 38 22];

            app.VolumeSinceZeroButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeSinceZeroButton.Text = 'Incr';
            app.VolumeSinceZeroButton.Tag = 'sinceZero';
            app.VolumeSinceZeroButton.FontSize = 10;
            app.VolumeSinceZeroButton.Tooltip = {'Volume delivered since Zero was pressed; -- until then'};
            app.VolumeSinceZeroButton.Position = [82 4 38 22];

            app.VolumeZeroButton = uibutton(app.VolumePanel, 'push');
            app.VolumeZeroButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @VolumeZeroButtonPushed, true);
            app.VolumeZeroButton.Text = 'Zero';
            app.VolumeZeroButton.FontSize = 10;
            app.VolumeZeroButton.Enable = 'off';
            app.VolumeZeroButton.Tooltip = {'Zero the incremental volume at the present position'};
            app.VolumeZeroButton.Position = [128 8 38 22];
        end

        % The pressure panel's unit selector
        function createPressureControls(app)
            app.PressureValue.Position = [5 38 160 62];
            app.PressureValue.FontSize = 36;

            app.PressureUnitGroup = uibuttongroup(app.PressurePanel);
            app.PressureUnitGroup.BorderType = 'none';
            app.PressureUnitGroup.SelectionChangedFcn = ...
                createCallbackFcn(app, @PressureUnitChanged, true);
            app.PressureUnitGroup.Position = [4 4 122 30];

            app.PressureUnitPsiButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitPsiButton.Text = 'psi';
            app.PressureUnitPsiButton.Tag = 'psi';
            app.PressureUnitPsiButton.FontSize = 10;
            app.PressureUnitPsiButton.Tooltip = {'Show pressures in psi, absolute'};
            app.PressureUnitPsiButton.Position = [2 4 38 22];

            app.PressureUnitAtmButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitAtmButton.Text = 'atm';
            app.PressureUnitAtmButton.Tag = 'atm';
            app.PressureUnitAtmButton.FontSize = 10;
            app.PressureUnitAtmButton.Tooltip = {'Show pressures in atm, absolute'};
            app.PressureUnitAtmButton.Position = [42 4 38 22];

            app.PressureUnitMPaButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitMPaButton.Text = 'MPa';
            app.PressureUnitMPaButton.Tag = 'MPa';
            app.PressureUnitMPaButton.FontSize = 10;
            app.PressureUnitMPaButton.Tooltip = {'Show pressures in MPa, absolute'};
            app.PressureUnitMPaButton.Position = [82 4 38 22];
        end

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1208 432];
            app.UIFigure.Name = 'SOS Gen3 Compressor';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

            % Create ProcedureMenu: the choice of procedure. The file items
            % are made in buildProcedureMenu, once the configuration is read.
            app.ProcedureMenu = uimenu(app.UIFigure);
            app.ProcedureMenu.Text = 'Procedure';

            % Create ProcedureOpenMenu
            app.ProcedureOpenMenu = uimenu(app.ProcedureMenu);
            app.ProcedureOpenMenu.MenuSelectedFcn = createCallbackFcn(app, @ProcedureOpenSelected, true);
            app.ProcedureOpenMenu.Text = 'Open...';

            % Create ProcedureReloadMenu
            app.ProcedureReloadMenu = uimenu(app.ProcedureMenu);
            app.ProcedureReloadMenu.MenuSelectedFcn = createCallbackFcn(app, @ProcedureReloadSelected, true);
            app.ProcedureReloadMenu.Text = 'Reload';

            % Create TraceMenu
            app.TraceMenu = uimenu(app.UIFigure);
            app.TraceMenu.Text = 'Trace';

            % Create TraceRecordMenu
            app.TraceRecordMenu = uimenu(app.TraceMenu);
            app.TraceRecordMenu.MenuSelectedFcn = createCallbackFcn(app, @TraceRecordMenuSelected, true);
            app.TraceRecordMenu.Accelerator = 'R';
            app.TraceRecordMenu.Enable = 'off';
            app.TraceRecordMenu.Text = 'Record';

            % Create TracePlotMenu
            app.TracePlotMenu = uimenu(app.TraceMenu);
            app.TracePlotMenu.MenuSelectedFcn = createCallbackFcn(app, @TracePlotMenuSelected, true);
            app.TracePlotMenu.Accelerator = 'P';
            app.TracePlotMenu.Text = 'Plot trace...';

            % Create TraceLiveMenu
            app.TraceLiveMenu = uimenu(app.TraceMenu);
            app.TraceLiveMenu.MenuSelectedFcn = createCallbackFcn(app, @TraceLiveMenuSelected, true);
            app.TraceLiveMenu.Accelerator = 'L';
            app.TraceLiveMenu.Text = 'Live plot';

            % Create ProcedurePanel: the walk-through of the procedure, in the
            % place of the old tabs. The checkbox and parameter pools are
            % created in applyConfig, sized by the configuration.
            app.ProcedurePanel = uipanel(app.UIFigure);
            app.ProcedurePanel.Title = 'Procedure';
            app.ProcedurePanel.FontWeight = 'bold';
            app.ProcedurePanel.Position = [524 120 676 308];

            app.ProcStepList = uilistbox(app.ProcedurePanel);
            app.ProcStepList.ValueChangedFcn = createCallbackFcn(app, @ProcStepListChanged, true);
            app.ProcStepList.FontSize = 10;
            app.ProcStepList.Position = [8 84 196 190];

            app.ProcTimerLabel = uilabel(app.ProcedurePanel);
            app.ProcTimerLabel.FontSize = 10;
            app.ProcTimerLabel.Position = [8 56 196 22];
            app.ProcTimerLabel.Text = '--';

            app.ProcVolumeLabel = uilabel(app.ProcedurePanel);
            app.ProcVolumeLabel.FontSize = 10;
            app.ProcVolumeLabel.Position = [8 32 196 22];
            app.ProcVolumeLabel.Text = '--';

            app.ProcPressureTargetLabel = uilabel(app.ProcedurePanel);
            app.ProcPressureTargetLabel.FontSize = 10;
            app.ProcPressureTargetLabel.Position = [8 8 196 22];
            app.ProcPressureTargetLabel.Text = 'Held pressure: --';

            app.ProcHeadingLabel = uilabel(app.ProcedurePanel);
            app.ProcHeadingLabel.FontWeight = 'bold';
            app.ProcHeadingLabel.FontSize = 12;
            app.ProcHeadingLabel.Position = [214 252 454 22];
            app.ProcHeadingLabel.Text = '';

            app.ProcTitleLabel = uilabel(app.ProcedurePanel);
            app.ProcTitleLabel.FontSize = 12;
            app.ProcTitleLabel.Position = [214 230 454 22];
            app.ProcTitleLabel.Text = '';

            app.ProcTextArea = uitextarea(app.ProcedurePanel);
            app.ProcTextArea.Editable = 'off';
            app.ProcTextArea.Position = [214 184 454 44];

            app.ProcMessageLabel = uilabel(app.ProcedurePanel);
            app.ProcMessageLabel.FontSize = 10;
            app.ProcMessageLabel.Position = [214 34 454 18];
            app.ProcMessageLabel.Text = '';

            app.ProcConfirmButton = uibutton(app.ProcedurePanel, 'push');
            app.ProcConfirmButton.ButtonPushedFcn = createCallbackFcn(app, @ProcConfirmButtonPushed, true);
            app.ProcConfirmButton.Visible = 'off';
            app.ProcConfirmButton.Position = [214 6 110 26];
            app.ProcConfirmButton.Text = 'Confirm';

            app.ProcRepeatButton = uibutton(app.ProcedurePanel, 'push');
            app.ProcRepeatButton.ButtonPushedFcn = createCallbackFcn(app, @ProcRepeatButtonPushed, true);
            app.ProcRepeatButton.Visible = 'off';
            app.ProcRepeatButton.Position = [330 6 110 26];
            app.ProcRepeatButton.Text = 'Repeat';

            app.ProcPauseButton = uibutton(app.ProcedurePanel, 'push');
            app.ProcPauseButton.ButtonPushedFcn = createCallbackFcn(app, @ProcPauseButtonPushed, true);
            app.ProcPauseButton.Enable = 'off';
            app.ProcPauseButton.Position = [446 6 100 26];
            app.ProcPauseButton.Text = 'Pause';

            app.ProcAbortButton = uibutton(app.ProcedurePanel, 'push');
            app.ProcAbortButton.ButtonPushedFcn = createCallbackFcn(app, @ProcAbortButtonPushed, true);
            app.ProcAbortButton.Enable = 'off';
            app.ProcAbortButton.Position = [552 6 100 26];
            app.ProcAbortButton.Text = 'Abort';

            % Create ControlsPanel
            app.ControlsPanel = uipanel(app.UIFigure);
            app.ControlsPanel.Title = 'Controls';
            app.ControlsPanel.Position = [1 22 1207 94];

            % Create ConnectToCompressorButton
            app.ConnectToCompressorButton = uibutton(app.ControlsPanel, 'push');
            app.ConnectToCompressorButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectToCompressorButton.Position = [12 39 142 23];
            app.ConnectToCompressorButton.Text = 'Connect to Compressor';

            % Create StartProcedureButton
            app.StartProcedureButton = uibutton(app.ControlsPanel, 'push');
            app.StartProcedureButton.ButtonPushedFcn = createCallbackFcn(app, @StartProcedureButtonPushed, true);
            app.StartProcedureButton.Position = [12 11 142 23];
            app.StartProcedureButton.Text = 'Start Procedure';

            % Create ManualControlButton
            app.ManualControlButton = uibutton(app.ControlsPanel, 'state');
            app.ManualControlButton.ValueChangedFcn = createCallbackFcn(app, @ManualControlButtonPressed, true);
            app.ManualControlButton.Text = 'Manual Control';
            app.ManualControlButton.Position = [264 40 130 23];

            % Create HomeButton
            app.HomeButton = uibutton(app.ControlsPanel, 'push');
            app.HomeButton.ButtonPushedFcn = createCallbackFcn(app, @HomeButtonPushed, true);
            app.HomeButton.Enable = 'off';
            app.HomeButton.Tooltip = {'Home to the negative overtravel switch and reset the zero'};
            app.HomeButton.Position = [264 12 62 23];
            app.HomeButton.Text = 'Home';

            % Create GoHomeButton
            app.GoHomeButton = uibutton(app.ControlsPanel, 'push');
            app.GoHomeButton.ButtonPushedFcn = createCallbackFcn(app, @GoHomeButtonPushed, true);
            app.GoHomeButton.Enable = 'off';
            app.GoHomeButton.Tooltip = {'Move to the home position'};
            app.GoHomeButton.Position = [332 12 62 23];
            app.GoHomeButton.Text = 'Go Home';

            % Create TargetPositionmmSpinnerLabel
            app.TargetPositionmmSpinnerLabel = uilabel(app.ControlsPanel);
            app.TargetPositionmmSpinnerLabel.HorizontalAlignment = 'right';
            app.TargetPositionmmSpinnerLabel.Enable = 'off';
            app.TargetPositionmmSpinnerLabel.Position = [396 40 128 22];
            app.TargetPositionmmSpinnerLabel.Text = 'Target from EOT (mm)';

            % Create TargetPositionSpinner
            app.TargetPositionSpinner = uispinner(app.ControlsPanel);
            app.TargetPositionSpinner.ValueChangedFcn = createCallbackFcn(app, @TargetSpinnerChanged, true);
            app.TargetPositionSpinner.Step = 5;
            app.TargetPositionSpinner.RoundFractionalValues = 'on';
            app.TargetPositionSpinner.Enable = 'off';
            app.TargetPositionSpinner.Position = [539 40 100 22];

            % Create PositionGoButton
            app.PositionGoButton = uibutton(app.ControlsPanel, 'push');
            app.PositionGoButton.ButtonPushedFcn = createCallbackFcn(app, @PositionGoButtonPushed, true);
            app.PositionGoButton.Enable = 'off';
            app.PositionGoButton.Position = [657 40 100 23];
            app.PositionGoButton.Text = 'Go';

            % Create SpeedLabel
            app.SpeedLabel = uilabel(app.ControlsPanel);
            app.SpeedLabel.HorizontalAlignment = 'right';
            app.SpeedLabel.Enable = 'off';
            app.SpeedLabel.Position = [990 40 60 22];
            app.SpeedLabel.Text = 'Speed (%)';

            % Create SpeedField
            app.SpeedField = uieditfield(app.ControlsPanel, 'numeric');
            app.SpeedField.ValueChangedFcn = createCallbackFcn(app, @SpeedFieldChanged, true);
            app.SpeedField.Limits = [0 100];
            app.SpeedField.ValueDisplayFormat = '%g';
            app.SpeedField.Enable = 'off';
            app.SpeedField.Tooltip = {'Percent of the maximum speed for jog, moves, Go Home and homing; 0 holds'};
            app.SpeedField.Position = [1055 40 50 22];
            app.SpeedField.Value = 100;

            % Create SpeedMmsLabel
            app.SpeedMmsLabel = uilabel(app.ControlsPanel);
            app.SpeedMmsLabel.Enable = 'off';
            app.SpeedMmsLabel.Position = [1108 40 98 22];
            app.SpeedMmsLabel.Text = '';

            % Create JogUpButton
            app.JogUpButton = uibutton(app.ControlsPanel, 'state');
            app.JogUpButton.ValueChangedFcn = createCallbackFcn(app, @JogUpButtonPushed, true);
            app.JogUpButton.Enable = 'off';
            app.JogUpButton.Text = 'Jog Up';
            app.JogUpButton.Position = [768 38 100 23];

            % Create JogDownButton
            app.JogDownButton = uibutton(app.ControlsPanel, 'state');
            app.JogDownButton.ValueChangedFcn = createCallbackFcn(app, @JogDownButtonPushed, true);
            app.JogDownButton.Enable = 'off';
            app.JogDownButton.Text = 'Jog Down';
            app.JogDownButton.Position = [768 9 100 23];

            % Create MixerButton
            app.MixerButton = uibutton(app.ControlsPanel, 'state');
            app.MixerButton.ValueChangedFcn = createCallbackFcn(app, @MixerButtonPushed, true);
            app.MixerButton.Enable = 'off';
            app.MixerButton.Text = 'Mixer';
            app.MixerButton.Position = [879 38 100 23];

            % Create TargetPressureatmSpinnerLabel
            app.TargetPressureatmSpinnerLabel = uilabel(app.ControlsPanel);
            app.TargetPressureatmSpinnerLabel.HorizontalAlignment = 'right';
            app.TargetPressureatmSpinnerLabel.Enable = 'off';
            app.TargetPressureatmSpinnerLabel.Position = [406 13 118 22];
            app.TargetPressureatmSpinnerLabel.Text = 'Target (atm abs)';

            % Create TargetPressureSpinner
            app.TargetPressureSpinner = uispinner(app.ControlsPanel);
            app.TargetPressureSpinner.ValueChangedFcn = createCallbackFcn(app, @TargetSpinnerChanged, true);
            app.TargetPressureSpinner.Limits = [1 100];   % replaced by applyConfig
            app.TargetPressureSpinner.Enable = 'off';
            app.TargetPressureSpinner.Position = [539 13 100 22];
            app.TargetPressureSpinner.Value = 1;

            % Create PressureGoButton
            app.PressureGoButton = uibutton(app.ControlsPanel, 'push');
            app.PressureGoButton.ButtonPushedFcn = createCallbackFcn(app, @PressureGoButtonPushed, true);
            app.PressureGoButton.Enable = 'off';
            app.PressureGoButton.Position = [657 11 100 23];
            app.PressureGoButton.Text = 'Go';

            % Create ESTOP
            app.ESTOP = uibutton(app.ControlsPanel, 'state');
            app.ESTOP.ValueChangedFcn = createCallbackFcn(app, @ESTOPPushed, true);
            app.ESTOP.Text = 'STOP';
            app.ESTOP.BackgroundColor = [1 0 0];
            app.ESTOP.FontColor = [1 1 1];
            app.ESTOP.Position = [164 13 90 49];

            % Create StatusLabel
            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.Position = [5 1 439 22];
            app.StatusLabel.Text = 'Idle';

            % Create readouts
            [app.VolumePanel, app.VolumeValue] = createReadout(app, ...
                'Swept volume (mL)', [15 275 170 140], 40);
            createVolumeControls(app);
            [app.PressurePanel, app.PressureValue] = createReadout(app, ...
                'Pressure (atm abs)', [200 275 170 140], 40);
            createPressureControls(app);
            [app.PositionPanel, app.PositionValue] = createReadout(app, ...
                'Position from EOT (mm)', [15 125 170 140], 28);
            [app.FlowPanel, app.FlowValue] = createReadout(app, ...
                'Flow (mL/s)', [200 125 170 140], 28);

            % Create MixerOnLampLabel
            app.MixerOnLampLabel = uilabel(app.UIFigure);
            app.MixerOnLampLabel.HorizontalAlignment = 'right';
            app.MixerOnLampLabel.Position = [395 207 54 22];
            app.MixerOnLampLabel.Text = 'Mixer On';

            % Create MixerOnLamp
            app.MixerOnLamp = uilamp(app.UIFigure);
            app.MixerOnLamp.Position = [464 207 20 20];

            % Create FaultActiveLampLabel
            app.FaultActiveLampLabel = uilabel(app.UIFigure);
            app.FaultActiveLampLabel.HorizontalAlignment = 'right';
            app.FaultActiveLampLabel.Position = [382 162 67 22];
            app.FaultActiveLampLabel.Text = 'Fault Active';

            % Create FaultActiveLamp
            app.FaultActiveLamp = uilamp(app.UIFigure);
            app.FaultActiveLamp.Position = [464 162 20 20];
            app.FaultActiveLamp.Color = [0.6 0.6 0.6];

            % Create TracePanel
            app.TracePanel = uipanel(app.UIFigure);
            app.TracePanel.Title = 'Trace';
            app.TracePanel.FontWeight = 'bold';
            app.TracePanel.Position = [382 233 137 182];

            % Create RecordButton
            app.RecordButton = uibutton(app.TracePanel, 'state');
            app.RecordButton.ValueChangedFcn = createCallbackFcn(app, @RecordButtonChanged, true);
            app.RecordButton.Tooltip = {'Record the link and the commands to a CSV file'};
            app.RecordButton.Enable = 'off';
            app.RecordButton.Text = 'Record';
            app.RecordButton.Position = [8 128 121 28];

            % Create IntervalDropDown: items and values are set by applyConfig
            app.IntervalDropDown = uidropdown(app.TracePanel);
            app.IntervalDropDown.ValueChangedFcn = createCallbackFcn(app, @IntervalDropDownChanged, true);
            app.IntervalDropDown.Enable = 'off';
            app.IntervalDropDown.Tooltip = {'Record interval'};
            app.IntervalDropDown.Position = [8 102 121 22];

            % Create TraceStatusLabel
            app.TraceStatusLabel = uilabel(app.TracePanel);
            app.TraceStatusLabel.WordWrap = 'on';
            app.TraceStatusLabel.VerticalAlignment = 'top';
            app.TraceStatusLabel.Position = [8 56 121 44];
            app.TraceStatusLabel.Text = 'Not recording';

            % Create LivePlotButton
            app.LivePlotButton = uibutton(app.TracePanel, 'push');
            app.LivePlotButton.ButtonPushedFcn = createCallbackFcn(app, @LivePlotButtonPushed, true);
            app.LivePlotButton.Tooltip = {'Pressure, position, velocity and torque of the last minutes'};
            app.LivePlotButton.Position = [8 31 121 22];
            app.LivePlotButton.Text = 'Live plot';

            % Create PlotTraceButton
            app.PlotTraceButton = uibutton(app.TracePanel, 'push');
            app.PlotTraceButton.ButtonPushedFcn = createCallbackFcn(app, @PlotTraceButtonPushed, true);
            app.PlotTraceButton.Position = [8 6 121 22];
            app.PlotTraceButton.Text = 'Plot trace...';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = SOS_Gen3_Compressor_Interface

            % Associate the Simulink Model
            app.Simulation = simulation('SOS_Gen3_Master_Shell');

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end