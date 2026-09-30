classdef SOS_Gen3_Compressor_Interface < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        FileMenu                        matlab.ui.container.Menu
        SimulinkMenu                    matlab.ui.container.Menu
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
        StatusLabel                     matlab.ui.control.Label
        ControlsPanel                   matlab.ui.container.Panel
        ESTOP                           matlab.ui.control.StateButton
        PressureGoButton                matlab.ui.control.Button
        TargetPressureSpinner           matlab.ui.control.Spinner
        TargetPressureatmSpinnerLabel   matlab.ui.control.Label
        MixerButton                     matlab.ui.control.StateButton
        JogDownButton                   matlab.ui.control.StateButton
        JogUpButton                     matlab.ui.control.StateButton
        JogSpeedDropDown                matlab.ui.control.DropDown
        PositionGoButton                matlab.ui.control.Button
        TargetPositionSpinner           matlab.ui.control.Spinner
        TargetPositionmmSpinnerLabel    matlab.ui.control.Label
        GoHomeButton                    matlab.ui.control.Button
        HomeButton                      matlab.ui.control.Button
        ManualControlButton            matlab.ui.control.StateButton
        StartProcedureButton            matlab.ui.control.Button
        ConnectToCompressorButton       matlab.ui.control.Button
        TabGroup                        matlab.ui.container.TabGroup
        ProcessParametersTab            matlab.ui.container.Tab
        ParameterField2                 matlab.ui.control.NumericEditField
        ParameterField3Label_2          matlab.ui.control.Label
        OxygenSalineRatioPanel          matlab.ui.container.Panel
        EditField                       matlab.ui.control.NumericEditField
        EditFieldLabel                  matlab.ui.control.Label
        OxygenSalineMassRatioEditField  matlab.ui.control.NumericEditField
        EditField2Label                 matlab.ui.control.Label
        HenrysLimitEditField            matlab.ui.control.NumericEditField
        HenrysLimitEditFieldLabel       matlab.ui.control.Label
        ParameterField3                 matlab.ui.control.NumericEditField
        ParameterField3Label            matlab.ui.control.Label
        ParameterField1                 matlab.ui.control.NumericEditField
        ParameterField1Label            matlab.ui.control.Label
        SalinePurgeFillTab              matlab.ui.container.Tab
        SalineButton2                   matlab.ui.control.Button
        SalineCheckBox7                 matlab.ui.control.CheckBox
        SalineCheckBox6                 matlab.ui.control.CheckBox
        SalineButton1                   matlab.ui.control.Button
        SalineButton4                   matlab.ui.control.Button
        SalineButton3                   matlab.ui.control.Button
        SalineCheckBox2                 matlab.ui.control.CheckBox
        SalineCheckBox1                 matlab.ui.control.CheckBox
        SalineCheckBox5                 matlab.ui.control.CheckBox
        SalineCheckBox4                 matlab.ui.control.CheckBox
        SalineCheckBox3                 matlab.ui.control.CheckBox
        OxygenFillTab                   matlab.ui.container.Tab
        ContinueButton                  matlab.ui.control.Button
        CheckBox_2                      matlab.ui.control.CheckBox
        ClosetheoxygeninletCheckBox     matlab.ui.control.CheckBox
        OpentheoxCheckBox               matlab.ui.control.CheckBox
        FillOxygenButton                matlab.ui.control.Button
        CheckBox                        matlab.ui.control.CheckBox
        OpentheoxygeninletandCheckBox   matlab.ui.control.CheckBox
        CloseallvalvesCheckBox          matlab.ui.control.CheckBox
        PressurizeDischargeTab          matlab.ui.container.Tab
        UseMixerCheckBox                matlab.ui.control.CheckBox
        VolumeDischargedmLPanel         matlab.ui.container.Panel
        mLLabel                         matlab.ui.control.Label
        OpenthevalveattheneedleLabel    matlab.ui.control.Label
        Label_2                         matlab.ui.control.Label
        OpenthemainchambervalveLabel    matlab.ui.control.Label
        TodischargeLabel                matlab.ui.control.Label
        TimePressurizedPanel            matlab.ui.container.Panel
        Label                           matlab.ui.control.Label
        PressurizeButton                matlab.ui.control.Button
        CheckBox_3                      matlab.ui.control.CheckBox
    end


    % Public properties that correspond to the Simulink model
    properties (Access = public, Transient)
        Simulation simulink.Simulation
    end


    properties (Access = private)
        modelName
        updateTimer
        cfg                 % masterConfig: geometry and limits
        commandSeq = 0      % mirrors the CommandSeq block; a change marks a new command
        lastStatusCode = 0
        resetDialogOpen = false
        estopActive = false     % master reports STATUS_ESTOP

        % Operation started from a mode button. It stays active until the
        % model reports it ended, so its button can stop it.
        activeOp = 0            % OP_STOP when none
        activeJogDir = 0
        activeTarget = NaN      % position or pressure sent with activeOp
        opConfirmed = false     % the model has reported activeOp running
        opSentAt = uint64(0)    % tic of the last request
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

        STATUS_STARTING = 7
        STATUS_STOPPING = 6
        STATUS_ESTOP    = 20

        % Time the model gets to act on a request before a status that
        % does not show it running ends it
        CONFIRM_WAIT = 0.5

        COLOR_IDLE     = [0.96 0.96 0.96]
        COLOR_STARTING = [1 0.8 0.3]
        COLOR_ACTIVE   = [0.4 0.8 0.4]
        COLOR_STOP     = [1 0 0]
        COLOR_STOP_REQ = [0.55 0 0]
        COLOR_ESTOP    = [1 0.85 0]

        faultNames = ["Handshake", "Drive", "Position limit", ...
            "Reserved", "Piston exit guard", "Limit switch", "Encoder"]
    end

    methods (Access = private)

        function tf = isRunning(app)
            tf = app.Simulation.Status == "running";
        end

        function p = blockPath(app, name)
            p = append(app.modelName, "/", name);
        end

        % Write a value to one of the model's Constant blocks
        function setBlock(app, name, value)
            set_param(blockPath(app, name), "Value", num2str(double(value)));
        end

        % Read the output of one of the model's named Gain blocks
        function value = readBlock(app, name)
            rto = get_param(blockPath(app, name), "RuntimeObject");
            if isempty(rto)
                value = 0;
            else
                value = rto.OutputPort(1).Data;
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

        % Status codes that show op running
        function codes = runningStatus(app, op)
            switch op
                case app.OP_HOME,     codes = 1;
                case app.OP_GO_HOME,  codes = 8;
                case app.OP_POSITION, codes = [2 3];
                case app.OP_JOG,      codes = 4;
                case app.OP_PRESSURE, codes = 5;
                otherwise,            codes = [];
            end
        end

        % Follow the active operation through the model's status. It is
        % confirmed once its running status appears and ends when that
        % status goes. Before confirmation, starting and stopping (a
        % switch from another operation) are waited out and any other
        % status ends it once the model has had time to act.
        function trackOp(app, statusCode)
            if app.activeOp == app.OP_STOP
                return
            end
            if any(statusCode == runningStatus(app, app.activeOp))
                app.opConfirmed = true;
            elseif app.opConfirmed
                clearOp(app);
            elseif statusCode ~= app.STATUS_STARTING && ...
                    statusCode ~= app.STATUS_STOPPING && ...
                    toc(app.opSentAt) >= app.CONFIRM_WAIT
                clearOp(app);
            end
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

        function pollModel(app)
            if ~isvalid(app)
                return
            end
            if ~isRunning(app)
                app.estopActive = false;
                clearOp(app);
                updateButtons(app);
                return
            end

            statusCode = readBlock(app, "StatusCode");
            app.estopActive = statusCode == app.STATUS_ESTOP;
            trackOp(app, statusCode);
            updateButtons(app);
            position = readBlock(app, "Position");
            app.VolumeValue.Text = sprintf("%.1f", x2mL(position, app.cfg));
            app.PressureValue.Text = sprintf("%.2f", readBlock(app, "Pressure"));
            app.PositionValue.Text = sprintf("%.2f", app.cfg.posEOT - position);
            % Positive while the piston moves toward the end of travel
            app.FlowValue.Text = sprintf("%.2f", ...
                mLPerMm(app.cfg) * readBlock(app, "Velocity"));

            faulted = statusCode > 10 && statusCode < 20;
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

            app.StatusLabel.Text = statusText(app, statusCode);

            % Ask the operator before resetting a fault. The dialog opens
            % when a fault is first latched and again after a failed reset.
            if faulted && ~app.resetDialogOpen && ...
                    ~(app.lastStatusCode > 10 && app.lastStatusCode < 20)
                askFaultReset(app, statusCode - 10);
            end
            app.lastStatusCode = statusCode;
        end

        % Non-blocking, so polling continues while the dialog is open
        function askFaultReset(app, faultCode)
            app.resetDialogOpen = true;
            message = sprintf("Slave fault %d: %s.\n\nCorrect the cause, then reset.", ...
                faultCode, app.faultNames(faultCode));
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

        function text = statusText(app, statusCode)
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
                case app.STATUS_ESTOP, text = "E-STOP";
                case 30, text = "Slave did not confirm the mode";
                case 32, text = "Fault reset failed";
                case 33, text = "Slave did not report stopped";
                otherwise
                    if statusCode >= 10 && statusCode < 20
                        code = statusCode - 10;
                        if code >= 1
                            text = append("FAULT: ", app.faultNames(code));
                        else
                            text = "FAULT";
                        end
                    else
                        text = append("Status ", num2str(statusCode));
                    end
            end
        end

        function salineTabEnableChecks(app)
            allChecked = app.SalineCheckBox1.Value && app.SalineCheckBox2.Value && ...
                app.SalineCheckBox3.Value && app.SalineCheckBox4.Value && ...
                app.SalineCheckBox5.Value;
            app.SalineButton1.Enable = allChecked;
        end

        % Jog in direction, or stop if already jogging that way. The other
        % direction reverses a running jog without leaving the mode.
        function toggleJog(app, direction)
            if app.activeOp == app.OP_JOG && app.activeJogDir == direction
                stopOp(app);
                return
            end
            setBlock(app, "TargetVelocity", app.JogSpeedDropDown.Value);
            setBlock(app, "JogDirection", direction);
            if requestOp(app, app.OP_JOG)
                app.activeJogDir = direction;
            end
            updateButtons(app);
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.modelName = string(app.Simulation.ModelName);

            % Manual targets are distances from the end of travel, within
            % the master's soft limits and not past the end of travel. The
            % initial target is the retracted end, away from compression.
            app.cfg = masterConfig();
            app.TargetPositionSpinner.Limits = ...
                [max(app.cfg.posEOT - app.cfg.posMax, 0), app.cfg.posEOT - app.cfg.posMin];
            app.TargetPositionSpinner.Value = app.TargetPositionSpinner.Limits(2);

            % Preset jog speeds, starting at the fastest. The master clamps
            % any preset above jogVelMax.
            speeds = unique(app.cfg.jogSpeeds);
            app.JogSpeedDropDown.Items = compose("%g mm/s", speeds);
            app.JogSpeedDropDown.ItemsData = speeds;
            app.JogSpeedDropDown.Value = speeds(end);

            app.updateTimer = timer( ...
                ExecutionMode="fixedSpacing", ...
                Period=0.2, ...
                BusyMode="drop", ...
                TimerFcn=@(~,~) pollModel(app));
            start(app.updateTimer);
        end

        % Button pushed function: ConnectToCompressorButton
        function ConnectButtonPushed(app, event)
            app.StatusLabel.Text = "Connecting to Simulink...";
            drawnow;

            try
                if isRunning(app)
                    app.StatusLabel.Text = "Model is already running";
                else
                    start(app.Simulation);
                    if isRunning(app)
                        app.commandSeq = 0;
                        setBlock(app, "RequestedMode", app.OP_STOP);
                        setBlock(app, "CommandSeq", app.commandSeq);
                        setBlock(app, "ESTOP", app.ESTOP.Value);
                        setBlock(app, "Mixer", app.MixerButton.Value);
                        app.StatusLabel.Text = "Connected to Simulink";
                    else
                        app.StatusLabel.Text = append(...
                            "Simulink Status: ", string(app.Simulation.Status));
                    end
                end
            catch ME
                app.StatusLabel.Text = append("Model unable to start: ", ME.message);
            end
        end

        % Value changed function: ManualControlButton
        function ManualControlButtonPressed(app, event)
            value = app.ManualControlButton.Value;
            app.HomeButton.Enable = value;
            app.GoHomeButton.Enable = value;
            app.TargetPositionSpinner.Enable = value;
            app.TargetPositionmmSpinnerLabel.Enable = value;
            app.PositionGoButton.Enable = value;
            app.TargetPressureSpinner.Enable = value;
            app.TargetPressureatmSpinnerLabel.Enable = value;
            app.PressureGoButton.Enable = value;
            app.JogDownButton.Enable = value;
            app.JogUpButton.Enable = value;
            app.JogSpeedDropDown.Enable = value;
            app.MixerButton.Enable = value;
        end

        % Button pushed function: HomeButton
        function HomeButtonPushed(app, event)
            if app.activeOp == app.OP_HOME
                stopOp(app);
            else
                requestOp(app, app.OP_HOME);
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
            if ~isempty(app.updateTimer) && isvalid(app.updateTimer)
                stop(app.updateTimer);
                delete(app.updateTimer);
            end

            delete(app)
        end

        % Button pushed function: StartProcedureButton
        function StartProcedureButtonPushed(app, event)
            % Disable the parameter fields
            app.ParameterField1.Enable = false;
            app.ParameterField1Label.Enable = false;
            app.ParameterField2.Enable = false;
            app.ParameterField3Label_2.Enable = false;
            app.ParameterField3.Enable = false;
            app.ParameterField3Label.Enable = false;

            % Enable the checkboxes in the saline fill tab
            app.SalineCheckBox1.Enable = true;
            app.SalineCheckBox2.Enable = true;
            app.SalineCheckBox3.Enable = true;
            app.SalineCheckBox4.Enable = true;
            app.SalineCheckBox5.Enable = true;
        end

        % Button pushed function: PositionGoButton
        function PositionGoButtonPushed(app, event)
            target = app.TargetPositionSpinner.Value;
            if app.activeOp == app.OP_POSITION && target == app.activeTarget
                stopOp(app);
                return
            end
            setBlock(app, "TargetPosition", app.cfg.posEOT - target);
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

        % Value changed function: JogSpeedDropDown
        % The model reads TargetVelocity continuously, so a running jog
        % takes the new speed at once.
        function JogSpeedChanged(app, event)
            if app.activeOp == app.OP_JOG && isRunning(app)
                setBlock(app, "TargetVelocity", app.JogSpeedDropDown.Value);
            end
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
            if isRunning(app)
                setBlock(app, "ESTOP", app.ESTOP.Value);
                if app.ESTOP.Value
                    setBlock(app, "RequestedMode", app.OP_STOP);
                end
            end
            if app.ESTOP.Value
                clearOp(app);
            end
            updateButtons(app);
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

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1008 432];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

            % Create FileMenu
            app.FileMenu = uimenu(app.UIFigure);
            app.FileMenu.Text = 'File';

            % Create SimulinkMenu
            app.SimulinkMenu = uimenu(app.UIFigure);
            app.SimulinkMenu.Text = 'Simulink';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [524 115 484 318];

            % Create ProcessParametersTab
            app.ProcessParametersTab = uitab(app.TabGroup);
            app.ProcessParametersTab.Title = 'Process Parameters';

            % Create ParameterField1Label
            app.ParameterField1Label = uilabel(app.ProcessParametersTab);
            app.ParameterField1Label.HorizontalAlignment = 'right';
            app.ParameterField1Label.Position = [10 261 148 22];
            app.ParameterField1Label.Text = 'Total Solution Volume (mL)';

            % Create ParameterField1
            app.ParameterField1 = uieditfield(app.ProcessParametersTab, 'numeric');
            app.ParameterField1.Position = [201 261 100 22];

            % Create ParameterField3Label
            app.ParameterField3Label = uilabel(app.ProcessParametersTab);
            app.ParameterField3Label.HorizontalAlignment = 'right';
            app.ParameterField3Label.Position = [10 228 170 22];
            app.ParameterField3Label.Text = 'Oxygen Loading Pressure (psi)';

            % Create ParameterField3
            app.ParameterField3 = uieditfield(app.ProcessParametersTab, 'numeric');
            app.ParameterField3.Position = [201 228 100 22];

            % Create OxygenSalineRatioPanel
            app.OxygenSalineRatioPanel = uipanel(app.ProcessParametersTab);
            app.OxygenSalineRatioPanel.Title = 'Oxygen-Saline Ratio';
            app.OxygenSalineRatioPanel.BackgroundColor = [0.8784 0.9922 1];
            app.OxygenSalineRatioPanel.Position = [10 59 403 122];

            % Create HenrysLimitEditFieldLabel
            app.HenrysLimitEditFieldLabel = uilabel(app.OxygenSalineRatioPanel);
            app.HenrysLimitEditFieldLabel.HorizontalAlignment = 'right';
            app.HenrysLimitEditFieldLabel.Position = [5 74 216 22];
            app.HenrysLimitEditFieldLabel.Text = '% Henry''s Limit (at discharge pressure)';

            % Create HenrysLimitEditField
            app.HenrysLimitEditField = uieditfield(app.OxygenSalineRatioPanel, 'numeric');
            app.HenrysLimitEditField.Position = [290 73 100 22];

            % Create EditField2Label
            app.EditField2Label = uilabel(app.OxygenSalineRatioPanel);
            app.EditField2Label.HorizontalAlignment = 'right';
            app.EditField2Label.Position = [5 42 147 22];
            app.EditField2Label.Text = 'Oxygen-Saline Mass Ratio';

            % Create OxygenSalineMassRatioEditField
            app.OxygenSalineMassRatioEditField = uieditfield(app.OxygenSalineRatioPanel, 'numeric');
            app.OxygenSalineMassRatioEditField.Position = [290 42 100 22];

            % Create EditFieldLabel
            app.EditFieldLabel = uilabel(app.OxygenSalineRatioPanel);
            app.EditFieldLabel.HorizontalAlignment = 'right';
            app.EditFieldLabel.Position = [5 11 275 22];
            app.EditFieldLabel.Text = 'Oxygen-Saline Volume Ratio (at loading pressure)';

            % Create EditField
            app.EditField = uieditfield(app.OxygenSalineRatioPanel, 'numeric');
            app.EditField.Position = [290 11 100 22];

            % Create ParameterField3Label_2
            app.ParameterField3Label_2 = uilabel(app.ProcessParametersTab);
            app.ParameterField3Label_2.HorizontalAlignment = 'right';
            app.ParameterField3Label_2.Position = [10 195 142 22];
            app.ParameterField3Label_2.Text = 'Discharge Pressure (atm)';

            % Create ParameterField2
            app.ParameterField2 = uieditfield(app.ProcessParametersTab, 'numeric');
            app.ParameterField2.Position = [201 195 100 22];

            % Create SalinePurgeFillTab
            app.SalinePurgeFillTab = uitab(app.TabGroup);
            app.SalinePurgeFillTab.Title = 'Saline Purge/Fill';

            % Create SalineCheckBox3
            app.SalineCheckBox3 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox3.Enable = 'off';
            app.SalineCheckBox3.Text = 'Close all valves';
            app.SalineCheckBox3.Position = [10 228 105 22];

            % Create SalineCheckBox4
            app.SalineCheckBox4 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox4.Enable = 'off';
            app.SalineCheckBox4.Text = 'Open the saline valve';
            app.SalineCheckBox4.Position = [10 207 137 22];

            % Create SalineCheckBox5
            app.SalineCheckBox5 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox5.Enable = 'off';
            app.SalineCheckBox5.Text = 'Open the main chamber valve';
            app.SalineCheckBox5.Position = [10 186 182 22];

            % Create SalineCheckBox1
            app.SalineCheckBox1 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox1.Enable = 'off';
            app.SalineCheckBox1.Text = 'Insert the saline line into the saline bag';
            app.SalineCheckBox1.Position = [10 270 231 22];

            % Create SalineCheckBox2
            app.SalineCheckBox2 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox2.Enable = 'off';
            app.SalineCheckBox2.Text = 'Insert the discharge line into an empty saline bag';
            app.SalineCheckBox2.Position = [10 249 286 22];

            % Create SalineButton3
            app.SalineButton3 = uibutton(app.SalinePurgeFillTab, 'push');
            app.SalineButton3.Enable = 'off';
            app.SalineButton3.Position = [10 50 100 23];
            app.SalineButton3.Text = 'Repeat Purge';

            % Create SalineButton4
            app.SalineButton4 = uibutton(app.SalinePurgeFillTab, 'push');
            app.SalineButton4.Enable = 'off';
            app.SalineButton4.Position = [10 20 100 23];
            app.SalineButton4.Text = 'Fill Saline';

            % Create SalineButton1
            app.SalineButton1 = uibutton(app.SalinePurgeFillTab, 'push');
            app.SalineButton1.Enable = 'off';
            app.SalineButton1.Position = [10 158 100 23];
            app.SalineButton1.Text = 'Continue';

            % Create SalineCheckBox6
            app.SalineCheckBox6 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox6.Enable = 'off';
            app.SalineCheckBox6.Text = 'Close the saline valve';
            app.SalineCheckBox6.Position = [10 132 139 22];

            % Create SalineCheckBox7
            app.SalineCheckBox7 = uicheckbox(app.SalinePurgeFillTab);
            app.SalineCheckBox7.Enable = 'off';
            app.SalineCheckBox7.Text = 'Open the discharge valve';
            app.SalineCheckBox7.Position = [10 111 158 22];

            % Create SalineButton2
            app.SalineButton2 = uibutton(app.SalinePurgeFillTab, 'push');
            app.SalineButton2.Enable = 'off';
            app.SalineButton2.Position = [10 81 100 23];
            app.SalineButton2.Text = 'Continue';

            % Create OxygenFillTab
            app.OxygenFillTab = uitab(app.TabGroup);
            app.OxygenFillTab.Title = 'Oxygen Fill';

            % Create CloseallvalvesCheckBox
            app.CloseallvalvesCheckBox = uicheckbox(app.OxygenFillTab);
            app.CloseallvalvesCheckBox.Enable = 'off';
            app.CloseallvalvesCheckBox.Text = 'Close all valves';
            app.CloseallvalvesCheckBox.Position = [10 270 105 22];

            % Create OpentheoxygeninletandCheckBox
            app.OpentheoxygeninletandCheckBox = uicheckbox(app.OxygenFillTab);
            app.OpentheoxygeninletandCheckBox.Enable = 'off';
            app.OpentheoxygeninletandCheckBox.Text = 'Open the oxygen inlet and main chamber valve';
            app.OpentheoxygeninletandCheckBox.Position = [10 249 276 22];

            % Create CheckBox
            app.CheckBox = uicheckbox(app.OxygenFillTab);
            app.CheckBox.Enable = 'off';
            app.CheckBox.Text = 'Open the oxygen regulator until the pressure reaches 50 psi';
            app.CheckBox.Position = [10 228 346 22];

            % Create FillOxygenButton
            app.FillOxygenButton = uibutton(app.OxygenFillTab, 'push');
            app.FillOxygenButton.Enable = 'off';
            app.FillOxygenButton.Position = [10 195 100 23];
            app.FillOxygenButton.Text = 'Fill Oxygen';

            % Create OpentheoxCheckBox
            app.OpentheoxCheckBox = uicheckbox(app.OxygenFillTab);
            app.OpentheoxCheckBox.Enable = 'off';
            app.OpentheoxCheckBox.Text = 'Open the oxygen regulator until the pressure reaches 100 psi';
            app.OpentheoxCheckBox.Position = [10 158 353 22];

            % Create ClosetheoxygeninletCheckBox
            app.ClosetheoxygeninletCheckBox = uicheckbox(app.OxygenFillTab);
            app.ClosetheoxygeninletCheckBox.Enable = 'off';
            app.ClosetheoxygeninletCheckBox.Text = 'Close the oxygen inlet and main chamber valve';
            app.ClosetheoxygeninletCheckBox.Position = [10 137 277 22];

            % Create CheckBox_2
            app.CheckBox_2 = uicheckbox(app.OxygenFillTab);
            app.CheckBox_2.Enable = 'off';
            app.CheckBox_2.Text = 'Close the oxygen regulator';
            app.CheckBox_2.Position = [10 116 165 22];

            % Create ContinueButton
            app.ContinueButton = uibutton(app.OxygenFillTab, 'push');
            app.ContinueButton.Enable = 'off';
            app.ContinueButton.Position = [10 89 100 23];
            app.ContinueButton.Text = 'Continue';

            % Create PressurizeDischargeTab
            app.PressurizeDischargeTab = uitab(app.TabGroup);
            app.PressurizeDischargeTab.Title = 'Pressurize/Discharge';

            % Create CheckBox_3
            app.CheckBox_3 = uicheckbox(app.PressurizeDischargeTab);
            app.CheckBox_3.Enable = 'off';
            app.CheckBox_3.Text = 'Ensure all valves are closed';
            app.CheckBox_3.Position = [10 270 171 22];

            % Create PressurizeButton
            app.PressurizeButton = uibutton(app.PressurizeDischargeTab, 'push');
            app.PressurizeButton.Enable = 'off';
            app.PressurizeButton.Position = [10 240 100 23];
            app.PressurizeButton.Text = 'Pressurize';

            % Create TimePressurizedPanel
            app.TimePressurizedPanel = uipanel(app.PressurizeDischargeTab);
            app.TimePressurizedPanel.Enable = 'off';
            app.TimePressurizedPanel.Title = 'Time Pressurized';
            app.TimePressurizedPanel.Position = [218 251 105 41];

            % Create Label
            app.Label = uilabel(app.TimePressurizedPanel);
            app.Label.Enable = 'off';
            app.Label.Position = [4 1 28 22];
            app.Label.Text = '0:00';

            % Create TodischargeLabel
            app.TodischargeLabel = uilabel(app.PressurizeDischargeTab);
            app.TodischargeLabel.Enable = 'off';
            app.TodischargeLabel.Position = [10 207 76 22];
            app.TodischargeLabel.Text = 'To discharge:';

            % Create OpenthemainchambervalveLabel
            app.OpenthemainchambervalveLabel = uilabel(app.PressurizeDischargeTab);
            app.OpenthemainchambervalveLabel.Enable = 'off';
            app.OpenthemainchambervalveLabel.Position = [10 186 359 22];
            app.OpenthemainchambervalveLabel.Text = '1. Open the main chamber valve and wait for pressure to stabilize';

            % Create Label_2
            app.Label_2 = uilabel(app.PressurizeDischargeTab);
            app.Label_2.Enable = 'off';
            app.Label_2.Position = [10 165 335 22];
            app.Label_2.Text = '2. Open the discharge valve and wait for pressure to stabilize';

            % Create OpenthevalveattheneedleLabel
            app.OpenthevalveattheneedleLabel = uilabel(app.PressurizeDischargeTab);
            app.OpenthevalveattheneedleLabel.Enable = 'off';
            app.OpenthevalveattheneedleLabel.Position = [10 144 172 22];
            app.OpenthevalveattheneedleLabel.Text = '3. Open the valve at the needle';

            % Create VolumeDischargedmLPanel
            app.VolumeDischargedmLPanel = uipanel(app.PressurizeDischargeTab);
            app.VolumeDischargedmLPanel.Enable = 'off';
            app.VolumeDischargedmLPanel.Title = 'Volume Discharged (mL)';
            app.VolumeDischargedmLPanel.Position = [10 100 142 41];

            % Create mLLabel
            app.mLLabel = uilabel(app.VolumeDischargedmLPanel);
            app.mLLabel.Enable = 'off';
            app.mLLabel.Position = [4 1 42 22];
            app.mLLabel.Text = '0.0 mL';

            % Create UseMixerCheckBox
            app.UseMixerCheckBox = uicheckbox(app.PressurizeDischargeTab);
            app.UseMixerCheckBox.Enable = 'off';
            app.UseMixerCheckBox.Text = 'Use Mixer';
            app.UseMixerCheckBox.Position = [218 225 76 22];

            % Create ControlsPanel
            app.ControlsPanel = uipanel(app.UIFigure);
            app.ControlsPanel.Title = 'Controls';
            app.ControlsPanel.Position = [1 22 1007 94];

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

            % Create JogSpeedDropDown
            app.JogSpeedDropDown = uidropdown(app.ControlsPanel);
            app.JogSpeedDropDown.ValueChangedFcn = createCallbackFcn(app, @JogSpeedChanged, true);
            app.JogSpeedDropDown.Tooltip = {'Jog speed'};
            app.JogSpeedDropDown.Enable = 'off';
            app.JogSpeedDropDown.Position = [879 9 100 22];

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
            app.TargetPressureSpinner.Limits = [1 100];
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
                'Volume (mL)', [15 275 170 140], 40);
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
            app.FaultActiveLamp.Color = [1 0 0];

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