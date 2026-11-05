classdef SOS_Gen3_Compressor_Interface < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TracePanel                      matlab.ui.container.Panel
        PlotTraceButton                 matlab.ui.control.Button
        TraceStatusLabel                matlab.ui.control.Label
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

        % Procedure (non-manual mode): the engine state, the normalized
        % definition it runs, the inputs of the last poll (for steps taken
        % from a callback), the tic of the last step and the text of the
        % last refused operator action.
        procDef = []
        proc = []
        procIn = struct('connected', false, 'statusCode', 0, 'faultCode', 0, ...
            'estop', false, 'pressure', 0, 'volume', 0)
        procTick = []
        procNote = ''

        % Trace recording: the buffer traceDrain fills from the model's
        % TraceRing block while traceRecording is set.
        traceRecording = false
        trace = []              % traceStart / traceDrain buffer
        traceStartedAt = ''     % wall-clock start, for the file's metadata
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
                drainTrace(app);        % the records since the last poll, while the ring is readable
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
            elapsed = (app.trace.n + app.trace.missed) * app.cfg.exchangePeriod;
            if app.trace.missed > 0
                app.TraceStatusLabel.Text = sprintf("%d samples, %.0f s, %d missed", ...
                    app.trace.n, elapsed, app.trace.missed);
                app.TraceStatusLabel.FontColor = [1 0 0];
            else
                app.TraceStatusLabel.Text = sprintf("%d samples, %.0f s", ...
                    app.trace.n, elapsed);
                app.TraceStatusLabel.FontColor = [0 0 0];
            end
        end

        function startRecording(app)
            app.trace = traceStart(readBlock(app, "TraceRing"));
            app.traceStartedAt = datestr(now, "yyyy-mm-ddTHH:MM:SS");
            app.traceRecording = true;
            app.TraceStatusLabel.Tooltip = "";
            showTraceProgress(app);
        end

        % Take the records written since the last read from the ring
        function drainTrace(app)
            if ~app.traceRecording || ~isRunning(app)
                return
            end
            try
                app.trace = traceDrain(app.trace, readBlock(app, "TraceRing"));
                showTraceProgress(app);
            catch ME
                app.TraceStatusLabel.Text = append("Trace read failed: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
        end

        % End the recording and save it. The file is written to the
        % captures folder first; an interactive stop then offers to move
        % it, and cancelling keeps it where it is.
        function finishRecording(app, interactive)
            if ~app.traceRecording
                return
            end
            drainTrace(app);
            app.traceRecording = false;     % before any file I/O or dialog
            app.TraceStatusLabel.Tooltip = "";
            app.TraceStatusLabel.FontColor = [0 0 0];
            if app.trace.n == 0
                app.TraceStatusLabel.Text = "Nothing recorded";
                updateButtons(app);
                return
            end
            try
                file = char(fullfile(capturesFolder(app), append("trace_", ...
                    datestr(now, "yyyymmdd_HHMMSS"), ".csv")));
                traceWriteCsv(file, app.trace.data(1:app.trace.n, :), ...
                    traceMeta(app.trace, app.cfg, app.traceStartedAt));
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
                if app.trace.missed > 0
                    app.TraceStatusLabel.Text = append(app.TraceStatusLabel.Text, ...
                        sprintf(" (%d missed)", app.trace.missed));
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            catch ME
                app.TraceStatusLabel.Text = append("Trace not saved: ", ME.message);
                app.TraceStatusLabel.FontColor = [1 0 0];
            end
            updateButtons(app);
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
            if app.TargetPressureSpinner.Value ~= app.activeTarget
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
            if app.traceRecording
                app.RecordButton.BackgroundColor = app.COLOR_STOP;
                app.RecordButton.FontColor = [1 1 1];
                app.RecordButton.Text = "Stop recording";
            else
                app.RecordButton.BackgroundColor = app.COLOR_IDLE;
                app.RecordButton.FontColor = [0 0 0];
                app.RecordButton.Text = "Record";
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
            setText(app, app.PressureValue, readoutText(d.pressure, 2, 0));
            setText(app, app.PositionValue, readoutText(app.cfg.posEOT - d.position, 2, 0));
            drainTrace(app);
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
                'volume', sweptVolume(d.position, app.cfg));
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

            % Pressure targets are atm absolute, up to the slave's highest
            % accepted target.
            setSpinnerRange(app, app.TargetPressureSpinner, [app.cfg.atmAmbient, ...
                floor(app.cfg.pressureMax * app.cfg.atmPerMPa + app.cfg.atmAmbient)]);

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

            createProcPools(app);
            loadProcedure(app);
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

        % Read the procedure and build the engine state. A procedure that
        % does not validate leaves the panel empty and Start disabled.
        function loadProcedure(app)
            try
                p = procNew(feval(app.cfg.procedure), app.cfg);
            catch ME
                app.proc = [];
                app.procDef = [];
                app.ProcMessageLabel.Text = ['Procedure not loaded: ' ME.message];
                app.StatusLabel.Text = append("Procedure not loaded: ", ME.message);
                return
            end
            app.procDef = p.def;
            app.proc = p;
            for k = 1:numel(app.ProcParamFields)
                if k <= numel(p.def.params)
                    try
                        app.ProcParamFields(k).ValueDisplayFormat = p.def.params(k).format;
                    catch
                    end
                end
            end
            renderProcedure(app);
        end

        % Set the operator speed (percent of cfg.speedMax). While the
        % model runs it is written to both blocks that carry it: the jog
        % reference and the speed sent in HR7-8.
        function setSpeedPercent(app, pct)
            pct = min(max(pct, 0), 100);
            app.speedPercent = pct;
            v = speedFromPercent(pct, app.cfg);
            app.SpeedField.Value = pct;
            app.SpeedMmsLabel.Text = sprintf("%.2f mm/s", v);
            for k = 1:numel(app.SpeedPresetButtons)
                if app.SpeedPresetButtons(k).UserData == pct
                    app.SpeedPresetButtons(k).BackgroundColor = app.COLOR_ACTIVE;
                else
                    app.SpeedPresetButtons(k).BackgroundColor = app.COLOR_IDLE;
                end
            end
            if isRunning(app)
                setBlock(app, "PositionSpeed", v);
                setBlock(app, "TargetVelocity", v);
            end
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

        % A spinner rejects a value outside its limits and limits that
        % exclude its value, so widen, move the value, then narrow.
        function setSpinnerRange(~, spinner, limits)
            spinner.Limits = [-Inf Inf];
            spinner.Value = min(max(spinner.Value, limits(1)), limits(2));
            spinner.Limits = limits;
        end

        function text = statusText(app, statusCode, faultCode)
            text = statusTextBase(app, statusCode, faultCode);
            if app.speedPercent == 0 && any(statusCode == [1 2 4 8])
                text = append(text, " (held: speed 0)");
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
            executeProcRequest(app, req);
            renderProcedure(app);
        end

        % Turn the engine's requests into commands, in the engine's order:
        % parameters, mixer, stop, operation, volume zero. The operation
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
                        setBlock(app, "PositionSpeed", speedMmPerS(app));
                        if requestOp(app, app.OP_POSITION)
                            app.activeTarget = app.cfg.posEOT - x;
                        end
                    case app.OP_PRESSURE
                        setBlock(app, "TargetPressure", req.target);
                        if requestOp(app, app.OP_PRESSURE)
                            app.activeTarget = req.target;
                        end
                    case app.OP_HOME
                        % Re-homing changes the position frame, so the zero
                        % of the incremental volume no longer holds.
                        if requestOp(app, app.OP_HOME)
                            app.volumeZeroPosition = NaN;
                            refreshVolume(app);
                        end
                    case app.OP_GO_HOME
                        requestOp(app, app.OP_GO_HOME);
                end
                updateButtons(app);
            end
            if req.zeroVolume
                zeroVolume(app);
            end
        end

        % Copy what procView says into the panel's components. A property
        % is written only when it changed, which saves redraws.
        function renderProcedure(app)
            if isempty(app.proc)
                return
            end
            v = procView(app.proc);
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
            setText(app, app.ProcTitleLabel, v.title);
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
            if isnan(v.pressureTarget)
                setText(app, app.ProcPressureTargetLabel, 'Held pressure: --');
            else
                setText(app, app.ProcPressureTargetLabel, ...
                    sprintf('Held pressure: %.2f atm', v.pressureTarget));
            end
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
        function ProcParamChanged(app, event)
            key = app.procDef.params(event.Source.UserData).key;
            procAct(app, 'setParam', key, event.Source.Value);
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
            target = app.TargetPressureSpinner.Value;
            if app.activeOp == app.OP_PRESSURE && target == app.activeTarget
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
                try
                    startRecording(app);
                catch ME
                    app.traceRecording = false;
                    app.TraceStatusLabel.Text = append("Recording not started: ", ME.message);
                    app.TraceStatusLabel.FontColor = [1 0 0];
                end
            else
                finishRecording(app, true);
            end
            updateButtons(app);
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

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1208 432];
            app.UIFigure.Name = 'SOS Gen3 Compressor';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

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
            app.SpeedMmsLabel.Position = [1110 40 90 22];
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
            app.TargetPressureatmSpinnerLabel.Text = 'Target Pressure (atm)';

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
                'Pressure (atm)', [200 275 170 140], 40);
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
            app.TracePanel.Position = [382 275 137 140];

            % Create RecordButton
            app.RecordButton = uibutton(app.TracePanel, 'state');
            app.RecordButton.ValueChangedFcn = createCallbackFcn(app, @RecordButtonChanged, true);
            app.RecordButton.Tooltip = {'Record the link and the commands to a CSV file'};
            app.RecordButton.Enable = 'off';
            app.RecordButton.Text = 'Record';
            app.RecordButton.Position = [8 82 121 30];

            % Create TraceStatusLabel
            app.TraceStatusLabel = uilabel(app.TracePanel);
            app.TraceStatusLabel.WordWrap = 'on';
            app.TraceStatusLabel.VerticalAlignment = 'top';
            app.TraceStatusLabel.Position = [8 36 121 44];
            app.TraceStatusLabel.Text = 'Not recording';

            % Create PlotTraceButton
            app.PlotTraceButton = uibutton(app.TracePanel, 'push');
            app.PlotTraceButton.ButtonPushedFcn = createCallbackFcn(app, @PlotTraceButtonPushed, true);
            app.PlotTraceButton.Position = [8 8 121 23];
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