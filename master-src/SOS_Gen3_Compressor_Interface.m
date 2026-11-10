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
        ViewMenu                        matlab.ui.container.Menu
        ViewSideBySideMenu              matlab.ui.container.Menu
        ViewSecondMonitorMenu           matlab.ui.container.Menu
        ViewDefaultSizeMenu             matlab.ui.container.Menu
        MainGrid                        matlab.ui.container.GridLayout
        ReadoutGrid                     matlab.ui.container.GridLayout
        LampGrid                        matlab.ui.container.GridLayout
        ControlsGrid                    matlab.ui.container.GridLayout
        SpeedGrid                       matlab.ui.container.GridLayout
        ProcedureGrid                   matlab.ui.container.GridLayout
        ProcLeftGrid                    matlab.ui.container.GridLayout
        ProcContentGrid                 matlab.ui.container.GridLayout
        ProcChecksGrid                  matlab.ui.container.GridLayout
        ProcParamsGrid                  matlab.ui.container.GridLayout
        ProcButtonsGrid                 matlab.ui.container.GridLayout
        TraceStatusLabel                matlab.ui.control.Label
        IntervalDropDown                matlab.ui.control.DropDown
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
        PollStats = struct('n', 0, 'mean', 0, 'max', 0, 'last', 0, 'overruns', 0, ...
            'writeGapMax', 0, 'stalls', 0)
        % One entry per link stall: a gap between master writes of at
        % least cfg.stallWarn (time, gap_s, activity: what the app last
        % did that holds MATLAB's thread, or '' when it did nothing)
        StallLog = struct('time', {}, 'gap_s', {}, 'activity', {})
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
        targetFromPiston = false  % set the manual target to the piston's position once the link is up

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

        % Height in pixels of the checklist and parameter pools when shown
        % (rows 4 and 5 of ProcContentGrid; 0 when the step has none)
        procChecksHeight = 0
        procParamsHeight = 0

        % Link stalls: the last activity that holds MATLAB's thread (and
        % so the model) and when it was noted, the stall shown on the
        % status line and since when, whether the windows are locked
        % against resizing, and recordings stopped while motion was
        % commanded, written once it ends.
        activityName = ''
        activityAt = []
        stallText = ""
        stallShownAt = []
        resizeLocked = false
        tracePending = {}
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

        % Window geometry in pixels: the drawable size of a new window and
        % of the smallest one before the grid scrolls, the heights of the
        % readout and controls bands, the grid's padding and spacing, the
        % bottom margin of a monitor's work area, and the window
        % decorations (left, bottom, right, top) used before they can be
        % measured
        DEFAULT_SIZE    = [940 900]
        MIN_SIZE        = [830 680]
        READOUT_HEIGHT  = 190
        CONTROLS_HEIGHT = 128
        MAIN_PAD        = 6
        TASKBAR_MARGIN  = 48
        DECO_FALLBACK   = [8 8 8 90]

        % A stall is put down to an activity noted at most this long
        % before it was seen, beyond the stall's own length (s), is
        % shown on the status line this long (s), and is not looked for
        % in the first records after the model starts
        STALL_SLACK = 2
        STALL_SHOW  = 15
        STALL_SKIP_RECORDS = 20

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
                flushPendingTraces(app);
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
                checkStall(app, aligned);
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

        % Open the live plot, or bring it forward when open
        function openLivePlot(app)
            if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig)
                figure(app.liveFig.fig);
                return
            end
            noteActivity(app, 'live plot opened');
            try
                app.liveFig = traceLiveNew('on');
                set(app.liveFig.fig, 'CloseRequestFcn', @(~, ~) closeLivePlot(app));
                try
                    app.liveFig.fig.AutoResizeChildren = 'off';  % so it takes a SizeChangedFcn
                catch
                end
                set(app.liveFig.fig, 'SizeChangedFcn', ...
                    @(~, ~) noteActivity(app, 'live plot resized'));
                app.liveFig.fig.Units = 'pixels';
                if ~isempty(app.cfg.livePlotPosition)
                    app.liveFig.fig.Position = app.cfg.livePlotPosition;
                end
                applyResizeLock(app);
                app.livePolls = 0;
                traceLiveUpdate(app.liveFig, app.traceLive);
                noteActivity(app, 'live plot opened');
            catch ME
                app.liveFig = [];
                app.TraceStatusLabel.Text = append("Live plot failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
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
        % it, and cancelling keeps it where it is. While motion is
        % commanded nothing is written and no dialog opens, since either
        % holds the model's thread: the recording is kept and written to
        % the captures folder once motion ends (flushPendingTraces).
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
            job = struct('data', app.trace.data(1:app.trace.n, :), ...
                'meta', traceMeta(struct('n', app.trace.n, 'missed', missed), ...
                app.cfg, app.traceStartedAt, app.traceIntervalStart), ...
                'missed', missed, 'stamp', datestr(now, "yyyymmdd_HHMMSS"));
            if motionCommanded(app)
                app.tracePending{end + 1} = job;
                app.TraceStatusLabel.Text = sprintf( ...
                    "Stopped, %d records: saved when motion ends", size(job.data, 1));
                updateButtons(app);
                return
            end
            saveTrace(app, job, interactive);
            updateButtons(app);
        end

        % Write the recordings kept while motion was commanded, without a
        % dialog, once motion has ended or the model has stopped
        function flushPendingTraces(app)
            if isempty(app.tracePending) || motionCommanded(app)
                return
            end
            jobs = app.tracePending;
            app.tracePending = {};
            for k = 1:numel(jobs)
                saveTrace(app, jobs{k}, false);
            end
        end

        % Write one finished recording to the captures folder, offer to
        % move it when interactive, and name it on the trace status line
        function saveTrace(app, job, interactive)
            noteActivity(app, 'trace file written');
            try
                file = char(fullfile(capturesFolder(app), append("trace_", ...
                    job.stamp, ".csv")));
                traceWriteCsv(file, job.data, job.meta);
                if interactive
                    noteActivity(app, 'Save trace dialog');
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
                app.TraceStatusLabel.FontColor = [0 0 0];
                if job.missed > 0
                    app.TraceStatusLabel.Text = append(app.TraceStatusLabel.Text, ...
                        sprintf(" (%d missed)", job.missed));
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            catch ME
                app.TraceStatusLabel.Text = append("Trace not saved: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
            noteActivity(app, 'trace file written');
        end

        % Note what the app is doing that holds MATLAB's thread, so that a
        % link stall seen in the trace afterwards can be put down to it
        function noteActivity(app, name)
            app.activityName = name;
            app.activityAt = tic;
        end

        % The longest gap between master writes in the new aligned
        % records. One of at least cfg.stallWarn is counted, logged with
        % the activity noted just before it (StallLog and the command
        % window) and shown on the status line for STALL_SHOW. The
        % records of the model's first second are left out, while the
        % model and the link settle after Connect.
        function checkStall(app, rows)
            L = traceLayout();
            if ~isempty(rows)
                rows = rows(rows(:, L.col.seq) > app.STALL_SKIP_RECORDS, :);
            end
            gap = traceWriteGap(rows, app.cfg);
            st = app.PollStats;
            st.writeGapMax = max(st.writeGapMax, gap);
            if gap >= app.cfg.stallWarn
                st.stalls = st.stalls + 1;
                activity = '';
                if ~isempty(app.activityAt) && ...
                        toc(app.activityAt) <= gap + app.STALL_SLACK
                    activity = app.activityName;
                end
                app.StallLog(end + 1) = struct('time', ...
                    datestr(now, 'yyyy-mm-dd HH:MM:SS'), 'gap_s', gap, ...
                    'activity', activity);
                if isempty(activity)
                    activity = 'no app activity noted';
                end
                fprintf('%s  link stall: %.2f s between master writes (%s)\n', ...
                    app.StallLog(end).time, gap, activity);
                app.stallText = sprintf(" (link stall %.2f s: %s)", gap, activity);
                app.stallShownAt = tic;
            end
            app.PollStats = st;
        end

        % The slave watches the master's heartbeat while motion is
        % commanded: an operation requested, running, starting or
        % stopping, a held pressure included. A model step held back past
        % G_cfgCommTimeout then faults it with FAULT_COMM.
        function tf = motionCommanded(app)
            tf = isRunning(app) && (app.activeOp ~= app.OP_STOP || ...
                (app.lastStatusCode >= 1 && app.lastStatusCode <= 8));
        end

        % Refuse an action that holds MATLAB's thread for long (a file
        % dialog, a plot, loading a procedure) while motion is commanded,
        % and say why on the status line
        function refused = refuseWhileMoving(app, what)
            refused = motionCommanded(app);
            if refused
                app.StatusLabel.Text = append(what, ...
                    " is not available while the axis is under command: it would hold up the slave link");
            end
        end

        % Lock or free the app's and the live plot's window size, as
        % resizeLocked says
        function applyResizeLock(app)
            value = 'on';
            if app.resizeLocked
                value = 'off';
            end
            try
                if ~strcmp(char(app.UIFigure.Resize), value)
                    app.UIFigure.Resize = value;
                end
                if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig) && ...
                        ~strcmp(char(app.liveFig.fig.Resize), value)
                    app.liveFig.fig.Resize = value;
                end
            catch
            end
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
        function [value, live] = readBlock(app, name)
            % live is false when the block has no runtime object yet (the
            % model has not stepped since it started) and value is 0.
            key = matlab.lang.makeValidName(name);
            live = true;
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
                live = false;
            else
                app.rto.(key) = rto;
                value = rto.OutputPort(1).Data;
            end
        end

        % The values shown on every poll, each block read once. The fault
        % code is read only while the status reports a fault.
        function d = readDisplay(app)
            [d.statusCode, liveStatus] = readBlock(app, "StatusCode");
            [d.position, livePosition] = readBlock(app, "Position");
            d.live = liveStatus && livePosition;    % the model has stepped
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

        % Set the manual position target to the piston's position (mm,
        % slave frame), so Go moves nothing until the operator changes it.
        function targetFromPistonPosition(app, position)
            if ~isfinite(position)
                app.StatusLabel.Text = "Position target kept: no position reading yet";
                return
            end
            x = round(app.cfg.posEOT - position);
            lim = app.TargetPositionSpinner.Limits;
            app.TargetPositionSpinner.Value = min(max(x, lim(1)), lim(2));
            app.StatusLabel.Text = sprintf( ...
                "Position target set to the piston, %d mm from EOT", x);
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
            noteActivity(app, 'track table update');
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
            noteActivity(app, 'track table update');
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

            % While motion is commanded nothing may hold MATLAB's thread
            % (and so the model) for long: the windows keep their size and
            % the file dialogs and plots are off. Opening the live plot
            % and the View menu stay, each a single layout.
            moving = motionCommanded(app);
            app.ProcedureMenu.Enable = ~procActive && ~moving;
            app.TracePlotMenu.Enable = ~moving;
            lock = app.cfg.lockResizeInMotion && moving;
            if lock ~= app.resizeLocked
                app.resizeLocked = lock;
                applyResizeLock(app);
            end
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
            if st.last >= app.cfg.stallWarn && ...
                    (isempty(app.activityAt) || toc(app.activityAt) > st.last)
                % A long poll with nothing noted inside it
                noteActivity(app, sprintf('display poll of %.2f s', st.last));
            end
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
                flushPendingTraces(app);
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
            if app.targetFromPiston && d.live && app.linkReady ...
                    && isfinite(d.position) && app.activeOp ~= app.OP_POSITION
                % The first live position reading of the connection becomes
                % the manual target.
                targetFromPistonPosition(app, d.position);
                app.targetFromPiston = false;
            end
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
            flushPendingTraces(app);
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
            % target shown before a connection is the configured in-cylinder
            % position; once the slave link is up, and whenever manual
            % control is engaged, it takes the piston's position, so Go
            % moves nothing until the operator changes it.
            limits = [max(app.cfg.posEOT - app.cfg.posMax, 0), ...
                app.cfg.posEOT - app.cfg.posMin];
            setSpinnerRange(app, app.TargetPositionSpinner, limits);
            app.TargetPositionSpinner.Value = min(max( ...
                round(app.cfg.targetFromEotDefault), limits(1)), limits(2));

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
            nPresets = numel(app.cfg.speedPresets);
            app.SpeedGrid.ColumnWidth = [{'fit', 48}, repmat({30}, 1, nPresets), {'1x'}];
            app.SpeedMmsLabel.Layout.Column = nPresets + 3;
            for k = 1:nPresets
                p = app.cfg.speedPresets(k);
                app.SpeedPresetButtons(k) = uibutton(app.SpeedGrid, 'push');
                app.SpeedPresetButtons(k).Layout.Row = 1;
                app.SpeedPresetButtons(k).Layout.Column = k + 2;
                app.SpeedPresetButtons(k).ButtonPushedFcn = ...
                    createCallbackFcn(app, @SpeedPresetPushed, true);
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
        % by the configuration, each in a grid in a row of ProcContentGrid.
        % Each callback finds its index in UserData.
        function createProcPools(app)
            nChecks = app.cfg.procMaxChecks;
            nRows = max(nChecks, 1);
            app.ProcChecksGrid = uigridlayout(app.ProcContentGrid, [nRows 1]);
            app.ProcChecksGrid.Layout.Row = 4;
            app.ProcChecksGrid.Layout.Column = 1;
            app.ProcChecksGrid.RowHeight = repmat({20}, 1, nRows);
            app.ProcChecksGrid.ColumnWidth = {'1x'};
            app.ProcChecksGrid.Padding = [0 0 0 0];
            app.ProcChecksGrid.RowSpacing = 2;
            app.procChecksHeight = 20 * nRows + 2 * (nRows - 1);
            app.ProcChecks = matlab.ui.control.CheckBox.empty;
            for k = 1:nChecks
                c = uicheckbox(app.ProcChecksGrid);
                c.Layout.Row = k;
                c.Layout.Column = 1;
                c.ValueChangedFcn = createCallbackFcn(app, @ProcCheckChanged, true);
                c.Text = '';
                c.UserData = k;
                c.Visible = 'off';
                app.ProcChecks(k) = c;
            end

            % Two columns of parameters, filled down the first column and
            % then the second; each label wraps onto two lines.
            nParams = app.cfg.procMaxParams;
            rows = max(ceil(nParams / 2), 1);
            app.ProcParamsGrid = uigridlayout(app.ProcContentGrid, [rows 4]);
            app.ProcParamsGrid.Layout.Row = 5;
            app.ProcParamsGrid.Layout.Column = 1;
            app.ProcParamsGrid.RowHeight = repmat({28}, 1, rows);
            app.ProcParamsGrid.ColumnWidth = {'1x', 70, '1x', 70};
            app.ProcParamsGrid.Padding = [0 0 0 0];
            app.ProcParamsGrid.RowSpacing = 4;
            app.ProcParamsGrid.ColumnSpacing = 6;
            app.procParamsHeight = 28 * rows + 4 * (rows - 1);
            app.ProcParamLabels = matlab.ui.control.Label.empty;
            app.ProcParamFields = matlab.ui.control.NumericEditField.empty;
            for k = 1:nParams
                r = mod(k - 1, rows) + 1;
                col = 2 * (ceil(k / rows) - 1) + 1;
                l = uilabel(app.ProcParamsGrid);
                l.Layout.Row = r;
                l.Layout.Column = col;
                l.HorizontalAlignment = 'right';
                l.WordWrap = 'on';
                l.FontSize = 10;
                l.Text = '';
                l.Visible = 'off';
                app.ProcParamLabels(k) = l;
                e = uieditfield(app.ProcParamsGrid, 'numeric');
                e.Layout.Row = r;
                e.Layout.Column = col + 1;
                e.ValueChangedFcn = createCallbackFcn(app, @ProcParamChanged, true);
                e.UserData = k;
                e.Visible = 'off';
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
            noteActivity(app, 'procedure loaded');
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
            if strlength(app.stallText) > 0
                if toc(app.stallShownAt) > app.STALL_SHOW
                    app.stallText = "";
                else
                    text = append(text, app.stallText);
                end
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

            % The pools' rows take their height while shown and none
            % otherwise; the grid is written only when a height changes.
            hChecks = 0;
            if ~isempty(v.checks.labels)
                hChecks = app.procChecksHeight;
            end
            hParams = 0;
            if showParams && ~isempty(v.params.keys)
                hParams = app.procParamsHeight;
            end
            rh = app.ProcContentGrid.RowHeight;
            if ~isequal(rh{4}, hChecks) || ~isequal(rh{5}, hParams)
                rh{4} = hChecks;
                rh{5} = hParams;
                app.ProcContentGrid.RowHeight = rh;
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

        % Monitors in pixels, one row [x y width height] each with the
        % primary first, and their work areas: the monitor without a
        % margin along its bottom edge for the taskbar.
        function [mons, work] = monitorAreas(app)
            root = groot;
            units = root.Units;
            root.Units = 'pixels';
            mons = root.MonitorPositions;
            root.Units = units;
            work = mons + [0 app.TASKBAR_MARGIN 0 -app.TASKBAR_MARGIN];
        end

        % Row of mons holding the centre of fig, else the nearest one
        function k = monitorOf(~, fig, mons)
            p = fig.Position;
            c = [p(1) + p(3) / 2, p(2) + p(4) / 2];
            inside = c(1) >= mons(:, 1) & c(1) < mons(:, 1) + mons(:, 3) & ...
                c(2) >= mons(:, 2) & c(2) < mons(:, 2) + mons(:, 4);
            k = find(inside, 1);
            if isempty(k)
                centres = [mons(:, 1) + mons(:, 3) / 2, mons(:, 2) + mons(:, 4) / 2];
                [~, k] = min(sum((centres - c) .^ 2, 2));
            end
        end

        % Window decorations [left bottom right top] in pixels: the
        % outer position minus the position of a visible figure, else
        % (or when implausible) DECO_FALLBACK.
        function d = decorations(app, fig)
            d = app.DECO_FALLBACK;
            if ~strcmp(char(fig.Visible), 'on')
                return
            end
            try
                drawnow;
                op = fig.OuterPosition;
                ip = fig.Position;
                m = [ip(1) - op(1), ip(2) - op(2), ...
                    op(1) + op(3) - ip(1) - ip(3), op(2) + op(4) - ip(2) - ip(4)];
                if all(m >= 0 & m <= 150)
                    d = m;
                end
            catch
            end
        end

        % Put the outer window rectangle rect = [x y width height] on fig:
        % the position is the rectangle less the decorations.
        function placeOuter(app, fig, rect)
            if ~strcmp(string(fig.WindowState), "normal")
                fig.WindowState = 'normal';
                drawnow;
            end
            d = decorations(app, fig);
            fig.Position = [rect(1) + d(1), rect(2) + d(2), ...
                rect(3) - d(1) - d(3), rect(4) - d(2) - d(4)];
        end

        % Restore the default size, keeping the top-left corner, inside
        % the work area of the monitor the window is on
        function setDefaultSize(app)
            fig = app.UIFigure;
            if ~strcmp(string(fig.WindowState), "normal")
                fig.WindowState = 'normal';
                drawnow;
            end
            [mons, work] = monitorAreas(app);
            a = work(monitorOf(app, fig, mons), :);
            d = decorations(app, fig);
            sz = min(app.DEFAULT_SIZE, [a(3) - d(1) - d(3), a(4) - d(2) - d(4)]);
            p = fig.Position;
            x = min(max(p(1), a(1) + d(1)), a(1) + a(3) - d(3) - sz(1));
            y = min(max(p(2) + p(4) - sz(2), a(2) + d(2)), a(2) + a(4) - d(4) - sz(2));
            fig.Position = [x y sz];
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.modelName = string(app.Simulation.ModelName);

            applyConfig(app);
            setDefaultSize(app);
            app.UIFigure.Visible = 'on';

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
                        app.targetFromPiston = true;
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
            % Engaging manual control takes the piston's position as the
            % position target, so Go moves nothing until it is changed.
            if value && isRunning(app) && app.linkReady
                targetFromPistonPosition(app, app.lastPosition);
            end
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
            flushPendingTraces(app);
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
        % The plot reads the stream the app drains on every poll, so it
        % shows the last cfg.livePlotSeconds whether or not Record is on.
        function TraceLiveMenuSelected(app, event)
            openLivePlot(app);
        end

        % Menu selected function: TracePlotMenu
        function TracePlotMenuSelected(app, event)
            if refuseWhileMoving(app, "Plot trace")
                return
            end
            noteActivity(app, 'Plot trace dialog');
            [name, folder] = uigetfile("*.csv", "Plot a trace", ...
                append(capturesFolder(app), filesep));
            figure(app.UIFigure);
            noteActivity(app, 'Plot trace dialog');
            if isequal(name, 0)
                return
            end
            try
                tracePlot(fullfile(folder, name));
                noteActivity(app, 'trace plotted');
                app.TraceStatusLabel.Text = append("Plotted ", name);
                app.TraceStatusLabel.FontColor = [0 0 0];
            catch ME
                app.TraceStatusLabel.Text = append("Plot failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
        end

        % Menu selected function: ViewSideBySideMenu
        % The app takes the left half of the monitor it is on, at least
        % its minimum width, and the live plot the rest.
        function ViewSideBySideSelected(app, event)
            noteActivity(app, 'View: side by side');
            [mons, work] = monitorAreas(app);
            a = work(monitorOf(app, app.UIFigure, mons), :);
            wApp = min(max(round(a(3) / 2), app.MIN_SIZE(1) + 16), a(3));
            placeOuter(app, app.UIFigure, [a(1) a(2) wApp a(4)]);
            openLivePlot(app);
            if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig)
                if a(3) - wApp >= 200
                    placeOuter(app, app.liveFig.fig, ...
                        [a(1) + wApp, a(2), a(3) - wApp, a(4)]);
                else
                    app.StatusLabel.Text = "No room for the live plot beside the app";
                end
            end
            figure(app.UIFigure);
        end

        % Menu selected function: ViewSecondMonitorMenu
        % The live plot fills the largest monitor the app is not on.
        function ViewSecondMonitorSelected(app, event)
            noteActivity(app, 'View: second monitor');
            [mons, work] = monitorAreas(app);
            if size(mons, 1) < 2
                app.StatusLabel.Text = "Only one monitor";
                return
            end
            others = setdiff(1:size(mons, 1), monitorOf(app, app.UIFigure, mons));
            [~, largest] = max(mons(others, 3) .* mons(others, 4));
            openLivePlot(app);
            if ~isempty(app.liveFig) && ishghandle(app.liveFig.fig)
                placeOuter(app, app.liveFig.fig, work(others(largest), :));
                drawnow;
                app.liveFig.fig.WindowState = 'maximized';
            end
            figure(app.UIFigure);
        end

        % Menu selected function: ViewDefaultSizeMenu
        function ViewDefaultSizeSelected(app, event)
            noteActivity(app, 'View: default size');
            setDefaultSize(app);
        end

        % Size changed function: UIFigure
        % Below the minimum size the grid keeps its minimum extent and
        % scrolls; above it the grid fills the window.
        function UIFigureSizeChanged(app, event)
            noteActivity(app, 'app window resized');
            if isempty(app.MainGrid) || ~isvalid(app.MainGrid)
                return
            end
            sz = app.UIFigure.Position(3:4);
            width = {'1x'};
            if sz(1) < app.MIN_SIZE(1)
                width = {app.MIN_SIZE(1) - 2 * app.MAIN_PAD};
            end
            height = '1x';
            if sz(2) < app.MIN_SIZE(2)
                height = app.MIN_SIZE(2) - app.READOUT_HEIGHT - ...
                    app.CONTROLS_HEIGHT - 4 * app.MAIN_PAD;
            end
            if ~isequal(app.MainGrid.ColumnWidth, width)
                app.MainGrid.ColumnWidth = width;
            end
            rh = app.MainGrid.RowHeight;
            if ~isequal(rh{3}, height)
                rh{3} = height;
                app.MainGrid.RowHeight = rh;
            end
        end

        % Menu selected function: ProcedureFileMenus
        function ProcedureFileSelected(app, event)
            if refuseWhileMoving(app, "Loading a procedure")
                return
            end
            loadProcedure(app, event.Source.UserData);
        end

        % Menu selected function: ProcedureOpenMenu
        % Any .m file; its folder joins the path and the function is
        % loaded by name.
        function ProcedureOpenSelected(app, event)
            if refuseWhileMoving(app, "Opening a procedure")
                return
            end
            noteActivity(app, 'Open procedure dialog');
            [name, folder] = uigetfile("*.m", "Open a procedure", ...
                append(fullfile(fileparts(mfilename("fullpath")), "procedure"), filesep));
            figure(app.UIFigure);
            noteActivity(app, 'Open procedure dialog');
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
            if refuseWhileMoving(app, "Reloading the procedure")
                return
            end
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
    end

    % Component initialization
    methods (Access = private)

        % A titled panel in the readout band's cell (1, column), holding one
        % large numeric value in a grid of its own
        function [panel, value, grid] = createReadout(app, title, column, fontSize)
            panel = uipanel(app.ReadoutGrid);
            panel.Layout.Row = 1;
            panel.Layout.Column = column;
            panel.Title = title;
            panel.FontWeight = 'bold';
            grid = uigridlayout(panel, [1 1]);
            grid.Padding = [4 2 4 2];
            grid.RowSpacing = 4;
            grid.ColumnSpacing = 4;
            value = uilabel(grid);
            value.Layout.Row = 1;
            value.Layout.Column = 1;
            value.HorizontalAlignment = 'center';
            value.FontSize = fontSize;
            value.FontWeight = 'bold';
            value.Text = '--';
        end

        % The volume panel's view selector and Zero button, in the panel's
        % grid under the value
        function createVolumeControls(app, grid)
            grid.RowHeight = {'1x', 30};
            grid.ColumnWidth = {124, '1x'};
            app.VolumeValue.Layout.Column = [1 2];

            app.VolumeViewGroup = uibuttongroup(grid);
            app.VolumeViewGroup.Layout.Row = 2;
            app.VolumeViewGroup.Layout.Column = 1;
            app.VolumeViewGroup.BorderType = 'none';
            app.VolumeViewGroup.SelectionChangedFcn = ...
                createCallbackFcn(app, @VolumeViewChanged, true);

            app.VolumeSweptButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeSweptButton.Text = 'Swept';
            app.VolumeSweptButton.Tag = 'swept';
            app.VolumeSweptButton.FontSize = 10;
            app.VolumeSweptButton.Tooltip = {'Swept volume: the saline still deliverable, without the dead volume'};
            app.VolumeSweptButton.Position = [2 2 38 26];

            app.VolumeMixingButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeMixingButton.Text = 'Mix';
            app.VolumeMixingButton.Tag = 'mixing';
            app.VolumeMixingButton.FontSize = 10;
            app.VolumeMixingButton.Tooltip = {'Mixing volume: the swept volume plus the dead volume that takes part in mixing'};
            app.VolumeMixingButton.Position = [42 2 38 26];

            app.VolumeSinceZeroButton = uitogglebutton(app.VolumeViewGroup);
            app.VolumeSinceZeroButton.Text = 'Incr';
            app.VolumeSinceZeroButton.Tag = 'sinceZero';
            app.VolumeSinceZeroButton.FontSize = 10;
            app.VolumeSinceZeroButton.Tooltip = {'Volume delivered since Zero was pressed; -- until then'};
            app.VolumeSinceZeroButton.Position = [82 2 38 26];

            app.VolumeZeroButton = uibutton(grid, 'push');
            app.VolumeZeroButton.Layout.Row = 2;
            app.VolumeZeroButton.Layout.Column = 2;
            app.VolumeZeroButton.ButtonPushedFcn = ...
                createCallbackFcn(app, @VolumeZeroButtonPushed, true);
            app.VolumeZeroButton.Text = 'Zero';
            app.VolumeZeroButton.FontSize = 10;
            app.VolumeZeroButton.Enable = 'off';
            app.VolumeZeroButton.Tooltip = {'Zero the incremental volume at the present position'};
        end

        % The pressure panel's unit selector, in the panel's grid under
        % the value
        function createPressureControls(app, grid)
            grid.RowHeight = {'1x', 30};
            grid.ColumnWidth = {124, '1x'};
            app.PressureValue.Layout.Column = [1 2];

            app.PressureUnitGroup = uibuttongroup(grid);
            app.PressureUnitGroup.Layout.Row = 2;
            app.PressureUnitGroup.Layout.Column = 1;
            app.PressureUnitGroup.BorderType = 'none';
            app.PressureUnitGroup.SelectionChangedFcn = ...
                createCallbackFcn(app, @PressureUnitChanged, true);

            app.PressureUnitPsiButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitPsiButton.Text = 'psi';
            app.PressureUnitPsiButton.Tag = 'psi';
            app.PressureUnitPsiButton.FontSize = 10;
            app.PressureUnitPsiButton.Tooltip = {'Show pressures in psi, absolute'};
            app.PressureUnitPsiButton.Position = [2 2 38 26];

            app.PressureUnitAtmButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitAtmButton.Text = 'atm';
            app.PressureUnitAtmButton.Tag = 'atm';
            app.PressureUnitAtmButton.FontSize = 10;
            app.PressureUnitAtmButton.Tooltip = {'Show pressures in atm, absolute'};
            app.PressureUnitAtmButton.Position = [42 2 38 26];

            app.PressureUnitMPaButton = uitogglebutton(app.PressureUnitGroup);
            app.PressureUnitMPaButton.Text = 'MPa';
            app.PressureUnitMPaButton.Tag = 'MPa';
            app.PressureUnitMPaButton.FontSize = 10;
            app.PressureUnitMPaButton.Tooltip = {'Show pressures in MPa, absolute'};
            app.PressureUnitMPaButton.Position = [82 2 38 26];
        end

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until startupFcn has sized it. The
            % children are laid out by grids, so the figure's resize
            % callback only adjusts the grid below the minimum size.
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 60 app.DEFAULT_SIZE];
            app.UIFigure.Name = 'SOS Gen3 Compressor';
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @UIFigureSizeChanged, true);
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
            app.TraceLiveMenu.Tooltip = 'Pressure, position, velocity and torque of the last minutes';
            app.TraceLiveMenu.Text = 'Live plot';

            % Create ViewMenu: window placement
            app.ViewMenu = uimenu(app.UIFigure);
            app.ViewMenu.Text = 'View';

            % Create ViewSideBySideMenu
            app.ViewSideBySideMenu = uimenu(app.ViewMenu);
            app.ViewSideBySideMenu.MenuSelectedFcn = createCallbackFcn(app, @ViewSideBySideSelected, true);
            app.ViewSideBySideMenu.Text = 'Side by side';

            % Create ViewSecondMonitorMenu
            app.ViewSecondMonitorMenu = uimenu(app.ViewMenu);
            app.ViewSecondMonitorMenu.MenuSelectedFcn = createCallbackFcn(app, @ViewSecondMonitorSelected, true);
            app.ViewSecondMonitorMenu.Enable = size(get(groot, 'MonitorPositions'), 1) > 1;
            app.ViewSecondMonitorMenu.Text = 'Live plot on second monitor';

            % Create ViewDefaultSizeMenu
            app.ViewDefaultSizeMenu = uimenu(app.ViewMenu);
            app.ViewDefaultSizeMenu.MenuSelectedFcn = createCallbackFcn(app, @ViewDefaultSizeSelected, true);
            app.ViewDefaultSizeMenu.Separator = 'on';
            app.ViewDefaultSizeMenu.Text = 'Default size';

            % Create MainGrid: the readout band, the controls band and the
            % procedure panel, which takes the remaining height
            app.MainGrid = uigridlayout(app.UIFigure, [3 1]);
            app.MainGrid.RowHeight = {app.READOUT_HEIGHT, app.CONTROLS_HEIGHT, '1x'};
            app.MainGrid.ColumnWidth = {'1x'};
            app.MainGrid.Padding = repmat(app.MAIN_PAD, 1, 4);
            app.MainGrid.RowSpacing = app.MAIN_PAD;
            app.MainGrid.Scrollable = 'on';

            % Create ReadoutGrid: four readouts, the status line and lamps
            % under them, STOP at the right
            app.ReadoutGrid = uigridlayout(app.MainGrid, [2 5]);
            app.ReadoutGrid.Layout.Row = 1;
            app.ReadoutGrid.Layout.Column = 1;
            app.ReadoutGrid.RowHeight = {'1x', 24};
            app.ReadoutGrid.ColumnWidth = {'1x', '1x', '1x', '1x', 120};
            app.ReadoutGrid.Padding = [0 0 0 0];
            app.ReadoutGrid.ColumnSpacing = 8;
            app.ReadoutGrid.RowSpacing = 4;

            % Create readouts
            [app.VolumePanel, app.VolumeValue, grid] = createReadout(app, ...
                'Swept volume (mL)', 2, 44);
            createVolumeControls(app, grid);
            [app.PressurePanel, app.PressureValue, grid] = createReadout(app, ...
                'Pressure (atm abs)', 4, 44);
            createPressureControls(app, grid);
            [app.PositionPanel, app.PositionValue] = createReadout(app, ...
                'Position from EOT (mm)', 3, 44);
            [app.FlowPanel, app.FlowValue] = createReadout(app, ...
                'Flow (mL/s)', 1, 44);

            % Create StatusLabel
            app.StatusLabel = uilabel(app.ReadoutGrid);
            app.StatusLabel.Layout.Row = 2;
            app.StatusLabel.Layout.Column = [1 3];
            app.StatusLabel.Text = 'Idle';

            % Create LampGrid
            app.LampGrid = uigridlayout(app.ReadoutGrid, [1 5]);
            app.LampGrid.Layout.Row = 2;
            app.LampGrid.Layout.Column = 4;
            app.LampGrid.RowHeight = {18};
            app.LampGrid.ColumnWidth = {'1x', 'fit', 18, 'fit', 18};
            app.LampGrid.Padding = [0 3 0 3];
            app.LampGrid.ColumnSpacing = 6;

            % Create FaultActiveLampLabel
            app.FaultActiveLampLabel = uilabel(app.LampGrid);
            app.FaultActiveLampLabel.Layout.Row = 1;
            app.FaultActiveLampLabel.Layout.Column = 2;
            app.FaultActiveLampLabel.HorizontalAlignment = 'right';
            app.FaultActiveLampLabel.Text = 'Fault';

            % Create FaultActiveLamp
            app.FaultActiveLamp = uilamp(app.LampGrid);
            app.FaultActiveLamp.Layout.Row = 1;
            app.FaultActiveLamp.Layout.Column = 3;
            app.FaultActiveLamp.Color = [0.6 0.6 0.6];

            % Create MixerOnLampLabel
            app.MixerOnLampLabel = uilabel(app.LampGrid);
            app.MixerOnLampLabel.Layout.Row = 1;
            app.MixerOnLampLabel.Layout.Column = 4;
            app.MixerOnLampLabel.HorizontalAlignment = 'right';
            app.MixerOnLampLabel.Text = 'Mixer';

            % Create MixerOnLamp
            app.MixerOnLamp = uilamp(app.LampGrid);
            app.MixerOnLamp.Layout.Row = 1;
            app.MixerOnLamp.Layout.Column = 5;

            % Create ESTOP
            app.ESTOP = uibutton(app.ReadoutGrid, 'state');
            app.ESTOP.Layout.Row = [1 2];
            app.ESTOP.Layout.Column = 5;
            app.ESTOP.ValueChangedFcn = createCallbackFcn(app, @ESTOPPushed, true);
            app.ESTOP.Text = 'STOP';
            app.ESTOP.FontSize = 22;
            app.ESTOP.FontWeight = 'bold';
            app.ESTOP.BackgroundColor = [1 0 0];
            app.ESTOP.FontColor = [1 1 1];

            % Create ControlsPanel: three rows of controls
            app.ControlsPanel = uipanel(app.MainGrid);
            app.ControlsPanel.Layout.Row = 2;
            app.ControlsPanel.Layout.Column = 1;
            app.ControlsPanel.Title = 'Controls';

            % Create ControlsGrid
            app.ControlsGrid = uigridlayout(app.ControlsPanel, [3 9]);
            app.ControlsGrid.RowHeight = {26, 26, 26};
            app.ControlsGrid.ColumnWidth = {146, 130, 84, 52, 78, 72, 64, '1x', 120};
            app.ControlsGrid.Padding = [8 6 8 6];
            app.ControlsGrid.RowSpacing = 6;
            app.ControlsGrid.ColumnSpacing = 6;

            % Create ConnectToCompressorButton
            app.ConnectToCompressorButton = uibutton(app.ControlsGrid, 'push');
            app.ConnectToCompressorButton.Layout.Row = 1;
            app.ConnectToCompressorButton.Layout.Column = 1;
            app.ConnectToCompressorButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectToCompressorButton.Text = 'Connect to Compressor';

            % Create StartProcedureButton
            app.StartProcedureButton = uibutton(app.ControlsGrid, 'push');
            app.StartProcedureButton.Layout.Row = 2;
            app.StartProcedureButton.Layout.Column = 1;
            app.StartProcedureButton.ButtonPushedFcn = createCallbackFcn(app, @StartProcedureButtonPushed, true);
            app.StartProcedureButton.Text = 'Start Procedure';

            % Create ManualControlButton
            app.ManualControlButton = uibutton(app.ControlsGrid, 'state');
            app.ManualControlButton.Layout.Row = 3;
            app.ManualControlButton.Layout.Column = 1;
            app.ManualControlButton.ValueChangedFcn = createCallbackFcn(app, @ManualControlButtonPressed, true);
            app.ManualControlButton.Text = 'Manual Control';

            % Create TargetPositionmmSpinnerLabel
            app.TargetPositionmmSpinnerLabel = uilabel(app.ControlsGrid);
            app.TargetPositionmmSpinnerLabel.Layout.Row = 1;
            app.TargetPositionmmSpinnerLabel.Layout.Column = 2;
            app.TargetPositionmmSpinnerLabel.HorizontalAlignment = 'right';
            app.TargetPositionmmSpinnerLabel.Enable = 'off';
            app.TargetPositionmmSpinnerLabel.Text = 'Target from EOT (mm)';

            % Create TargetPositionSpinner
            app.TargetPositionSpinner = uispinner(app.ControlsGrid);
            app.TargetPositionSpinner.Layout.Row = 1;
            app.TargetPositionSpinner.Layout.Column = 3;
            app.TargetPositionSpinner.ValueChangedFcn = createCallbackFcn(app, @TargetSpinnerChanged, true);
            app.TargetPositionSpinner.Step = 5;
            app.TargetPositionSpinner.RoundFractionalValues = 'on';
            app.TargetPositionSpinner.Enable = 'off';

            % Create PositionGoButton
            app.PositionGoButton = uibutton(app.ControlsGrid, 'push');
            app.PositionGoButton.Layout.Row = 1;
            app.PositionGoButton.Layout.Column = 4;
            app.PositionGoButton.ButtonPushedFcn = createCallbackFcn(app, @PositionGoButtonPushed, true);
            app.PositionGoButton.Enable = 'off';
            app.PositionGoButton.Text = 'Go';

            % Create TargetPressureatmSpinnerLabel
            app.TargetPressureatmSpinnerLabel = uilabel(app.ControlsGrid);
            app.TargetPressureatmSpinnerLabel.Layout.Row = 2;
            app.TargetPressureatmSpinnerLabel.Layout.Column = 2;
            app.TargetPressureatmSpinnerLabel.HorizontalAlignment = 'right';
            app.TargetPressureatmSpinnerLabel.Enable = 'off';
            app.TargetPressureatmSpinnerLabel.Text = 'Target (atm abs)';

            % Create TargetPressureSpinner
            app.TargetPressureSpinner = uispinner(app.ControlsGrid);
            app.TargetPressureSpinner.Layout.Row = 2;
            app.TargetPressureSpinner.Layout.Column = 3;
            app.TargetPressureSpinner.ValueChangedFcn = createCallbackFcn(app, @TargetSpinnerChanged, true);
            app.TargetPressureSpinner.Limits = [1 100];   % replaced by applyConfig
            app.TargetPressureSpinner.Enable = 'off';
            app.TargetPressureSpinner.Value = 1;

            % Create PressureGoButton
            app.PressureGoButton = uibutton(app.ControlsGrid, 'push');
            app.PressureGoButton.Layout.Row = 2;
            app.PressureGoButton.Layout.Column = 4;
            app.PressureGoButton.ButtonPushedFcn = createCallbackFcn(app, @PressureGoButtonPushed, true);
            app.PressureGoButton.Enable = 'off';
            app.PressureGoButton.Text = 'Go';

            % Create JogUpButton
            app.JogUpButton = uibutton(app.ControlsGrid, 'state');
            app.JogUpButton.Layout.Row = 1;
            app.JogUpButton.Layout.Column = 5;
            app.JogUpButton.ValueChangedFcn = createCallbackFcn(app, @JogUpButtonPushed, true);
            app.JogUpButton.Enable = 'off';
            app.JogUpButton.Text = 'Jog Up';

            % Create JogDownButton
            app.JogDownButton = uibutton(app.ControlsGrid, 'state');
            app.JogDownButton.Layout.Row = 2;
            app.JogDownButton.Layout.Column = 5;
            app.JogDownButton.ValueChangedFcn = createCallbackFcn(app, @JogDownButtonPushed, true);
            app.JogDownButton.Enable = 'off';
            app.JogDownButton.Text = 'Jog Down';

            % Create HomeButton
            app.HomeButton = uibutton(app.ControlsGrid, 'push');
            app.HomeButton.Layout.Row = 1;
            app.HomeButton.Layout.Column = 6;
            app.HomeButton.ButtonPushedFcn = createCallbackFcn(app, @HomeButtonPushed, true);
            app.HomeButton.Enable = 'off';
            app.HomeButton.Tooltip = {'Home to the negative overtravel switch and reset the zero'};
            app.HomeButton.Text = 'Home';

            % Create GoHomeButton
            app.GoHomeButton = uibutton(app.ControlsGrid, 'push');
            app.GoHomeButton.Layout.Row = 2;
            app.GoHomeButton.Layout.Column = 6;
            app.GoHomeButton.ButtonPushedFcn = createCallbackFcn(app, @GoHomeButtonPushed, true);
            app.GoHomeButton.Enable = 'off';
            app.GoHomeButton.Tooltip = {'Move to the home position'};
            app.GoHomeButton.Text = 'Go Home';

            % Create MixerButton
            app.MixerButton = uibutton(app.ControlsGrid, 'state');
            app.MixerButton.Layout.Row = 1;
            app.MixerButton.Layout.Column = 7;
            app.MixerButton.ValueChangedFcn = createCallbackFcn(app, @MixerButtonPushed, true);
            app.MixerButton.Enable = 'off';
            app.MixerButton.Text = 'Mixer';

            % Create SpeedGrid: the speed label and field, then a button
            % per preset (added by applyConfig) and the speed in mm/s
            app.SpeedGrid = uigridlayout(app.ControlsGrid, [1 3]);
            app.SpeedGrid.Layout.Row = 3;
            app.SpeedGrid.Layout.Column = [2 6];
            app.SpeedGrid.RowHeight = {'1x'};
            app.SpeedGrid.ColumnWidth = {'fit', 48, '1x'};
            app.SpeedGrid.Padding = [0 0 0 0];
            app.SpeedGrid.ColumnSpacing = 4;

            % Create SpeedLabel
            app.SpeedLabel = uilabel(app.SpeedGrid);
            app.SpeedLabel.Layout.Row = 1;
            app.SpeedLabel.Layout.Column = 1;
            app.SpeedLabel.HorizontalAlignment = 'right';
            app.SpeedLabel.Enable = 'off';
            app.SpeedLabel.Text = 'Speed (%)';

            % Create SpeedField
            app.SpeedField = uieditfield(app.SpeedGrid, 'numeric');
            app.SpeedField.Layout.Row = 1;
            app.SpeedField.Layout.Column = 2;
            app.SpeedField.ValueChangedFcn = createCallbackFcn(app, @SpeedFieldChanged, true);
            app.SpeedField.Limits = [0 100];
            app.SpeedField.ValueDisplayFormat = '%g';
            app.SpeedField.Enable = 'off';
            app.SpeedField.Tooltip = {'Percent of the maximum speed for jog, moves, Go Home and homing; 0 holds'};
            app.SpeedField.Value = 100;

            % Create SpeedMmsLabel
            app.SpeedMmsLabel = uilabel(app.SpeedGrid);
            app.SpeedMmsLabel.Layout.Row = 1;
            app.SpeedMmsLabel.Layout.Column = 3;
            app.SpeedMmsLabel.Enable = 'off';
            app.SpeedMmsLabel.Text = '';

            % Create RecordButton
            app.RecordButton = uibutton(app.ControlsGrid, 'state');
            app.RecordButton.Layout.Row = 1;
            app.RecordButton.Layout.Column = 9;
            app.RecordButton.ValueChangedFcn = createCallbackFcn(app, @RecordButtonChanged, true);
            app.RecordButton.Tooltip = {'Record the link and the commands to a CSV file'};
            app.RecordButton.Enable = 'off';
            app.RecordButton.Text = 'Record';

            % Create IntervalDropDown: items and values are set by applyConfig
            app.IntervalDropDown = uidropdown(app.ControlsGrid);
            app.IntervalDropDown.Layout.Row = 2;
            app.IntervalDropDown.Layout.Column = 9;
            app.IntervalDropDown.ValueChangedFcn = createCallbackFcn(app, @IntervalDropDownChanged, true);
            app.IntervalDropDown.Enable = 'off';
            app.IntervalDropDown.Tooltip = {'Record interval'};

            % Create TraceStatusLabel
            app.TraceStatusLabel = uilabel(app.ControlsGrid);
            app.TraceStatusLabel.Layout.Row = 3;
            app.TraceStatusLabel.Layout.Column = [8 9];   % right-aligned under Record and the interval
            app.TraceStatusLabel.HorizontalAlignment = 'right';
            app.TraceStatusLabel.WordWrap = 'on';
            app.TraceStatusLabel.FontSize = 11;
            app.TraceStatusLabel.VerticalAlignment = 'center';
            app.TraceStatusLabel.Text = 'Not recording';

            % Create ProcedurePanel: the walk-through of the procedure. The
            % checkbox and parameter pools are created in applyConfig,
            % sized by the configuration.
            app.ProcedurePanel = uipanel(app.MainGrid);
            app.ProcedurePanel.Layout.Row = 3;
            app.ProcedurePanel.Layout.Column = 1;
            app.ProcedurePanel.Title = 'Procedure';
            app.ProcedurePanel.FontWeight = 'bold';

            % Create ProcedureGrid: step list and times at the left, the
            % current step at the right
            app.ProcedureGrid = uigridlayout(app.ProcedurePanel, [1 2]);
            app.ProcedureGrid.RowHeight = {'1x'};
            app.ProcedureGrid.ColumnWidth = {220, '1x'};
            app.ProcedureGrid.Padding = [8 8 8 8];
            app.ProcedureGrid.ColumnSpacing = 10;

            % Create ProcLeftGrid
            app.ProcLeftGrid = uigridlayout(app.ProcedureGrid, [4 1]);
            app.ProcLeftGrid.Layout.Row = 1;
            app.ProcLeftGrid.Layout.Column = 1;
            app.ProcLeftGrid.RowHeight = {'1x', 22, 22, 22};
            app.ProcLeftGrid.ColumnWidth = {'1x'};
            app.ProcLeftGrid.Padding = [0 0 0 0];
            app.ProcLeftGrid.RowSpacing = 4;

            app.ProcStepList = uilistbox(app.ProcLeftGrid);
            app.ProcStepList.Layout.Row = 1;
            app.ProcStepList.Layout.Column = 1;
            app.ProcStepList.ValueChangedFcn = createCallbackFcn(app, @ProcStepListChanged, true);
            app.ProcStepList.FontSize = 12;

            app.ProcTimerLabel = uilabel(app.ProcLeftGrid);
            app.ProcTimerLabel.Layout.Row = 2;
            app.ProcTimerLabel.Layout.Column = 1;
            app.ProcTimerLabel.FontSize = 12;
            app.ProcTimerLabel.Text = '--';

            app.ProcVolumeLabel = uilabel(app.ProcLeftGrid);
            app.ProcVolumeLabel.Layout.Row = 3;
            app.ProcVolumeLabel.Layout.Column = 1;
            app.ProcVolumeLabel.FontSize = 12;
            app.ProcVolumeLabel.Text = '--';

            app.ProcPressureTargetLabel = uilabel(app.ProcLeftGrid);
            app.ProcPressureTargetLabel.Layout.Row = 4;
            app.ProcPressureTargetLabel.Layout.Column = 1;
            app.ProcPressureTargetLabel.FontSize = 12;
            app.ProcPressureTargetLabel.Text = 'Held pressure: --';

            % Create ProcContentGrid: rows 4 and 5 hold the checklist and
            % parameter pools and are 0 high while a step has none
            app.ProcContentGrid = uigridlayout(app.ProcedureGrid, [7 1]);
            app.ProcContentGrid.Layout.Row = 1;
            app.ProcContentGrid.Layout.Column = 2;
            app.ProcContentGrid.RowHeight = {22, 22, '1x', 0, 0, 18, 28};
            app.ProcContentGrid.ColumnWidth = {'1x'};
            app.ProcContentGrid.Padding = [0 0 0 0];
            app.ProcContentGrid.RowSpacing = 4;

            app.ProcHeadingLabel = uilabel(app.ProcContentGrid);
            app.ProcHeadingLabel.Layout.Row = 1;
            app.ProcHeadingLabel.Layout.Column = 1;
            app.ProcHeadingLabel.FontWeight = 'bold';
            app.ProcHeadingLabel.FontSize = 12;
            app.ProcHeadingLabel.Text = '';

            app.ProcTitleLabel = uilabel(app.ProcContentGrid);
            app.ProcTitleLabel.Layout.Row = 2;
            app.ProcTitleLabel.Layout.Column = 1;
            app.ProcTitleLabel.FontSize = 12;
            app.ProcTitleLabel.Text = '';

            app.ProcTextArea = uitextarea(app.ProcContentGrid);
            app.ProcTextArea.Layout.Row = 3;
            app.ProcTextArea.Layout.Column = 1;
            app.ProcTextArea.Editable = 'off';

            app.ProcMessageLabel = uilabel(app.ProcContentGrid);
            app.ProcMessageLabel.Layout.Row = 6;
            app.ProcMessageLabel.Layout.Column = 1;
            app.ProcMessageLabel.FontSize = 10;
            app.ProcMessageLabel.Text = '';

            % Create ProcButtonsGrid
            app.ProcButtonsGrid = uigridlayout(app.ProcContentGrid, [1 5]);
            app.ProcButtonsGrid.Layout.Row = 7;
            app.ProcButtonsGrid.Layout.Column = 1;
            app.ProcButtonsGrid.RowHeight = {'1x'};
            app.ProcButtonsGrid.ColumnWidth = {120, 110, 100, 100, '1x'};
            app.ProcButtonsGrid.Padding = [0 0 0 0];
            app.ProcButtonsGrid.ColumnSpacing = 6;

            app.ProcConfirmButton = uibutton(app.ProcButtonsGrid, 'push');
            app.ProcConfirmButton.Layout.Row = 1;
            app.ProcConfirmButton.Layout.Column = 1;
            app.ProcConfirmButton.ButtonPushedFcn = createCallbackFcn(app, @ProcConfirmButtonPushed, true);
            app.ProcConfirmButton.Visible = 'off';
            app.ProcConfirmButton.Text = 'Confirm';

            app.ProcRepeatButton = uibutton(app.ProcButtonsGrid, 'push');
            app.ProcRepeatButton.Layout.Row = 1;
            app.ProcRepeatButton.Layout.Column = 2;
            app.ProcRepeatButton.ButtonPushedFcn = createCallbackFcn(app, @ProcRepeatButtonPushed, true);
            app.ProcRepeatButton.Visible = 'off';
            app.ProcRepeatButton.Text = 'Repeat';

            app.ProcPauseButton = uibutton(app.ProcButtonsGrid, 'push');
            app.ProcPauseButton.Layout.Row = 1;
            app.ProcPauseButton.Layout.Column = 3;
            app.ProcPauseButton.ButtonPushedFcn = createCallbackFcn(app, @ProcPauseButtonPushed, true);
            app.ProcPauseButton.Enable = 'off';
            app.ProcPauseButton.Text = 'Pause';

            app.ProcAbortButton = uibutton(app.ProcButtonsGrid, 'push');
            app.ProcAbortButton.Layout.Row = 1;
            app.ProcAbortButton.Layout.Column = 4;
            app.ProcAbortButton.ButtonPushedFcn = createCallbackFcn(app, @ProcAbortButtonPushed, true);
            app.ProcAbortButton.Enable = 'off';
            app.ProcAbortButton.Text = 'Abort';
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