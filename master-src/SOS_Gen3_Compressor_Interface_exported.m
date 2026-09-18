classdef SOS_Gen3_Compressor_Interface_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        FileMenu                        matlab.ui.container.Menu
        SimulinkMenu                    matlab.ui.container.Menu
        FaultActiveLamp                 matlab.ui.control.Lamp
        FaultActiveLampLabel            matlab.ui.control.Label
        MixerOnLamp                     matlab.ui.control.Lamp
        MixerOnLampLabel                matlab.ui.control.Label
        FlowRatemLsGauge                matlab.ui.control.Gauge
        FlowRatemLsGaugeLabel           matlab.ui.control.Label
        PositionmmGauge                 matlab.ui.control.Gauge
        PositionmmGaugeLabel            matlab.ui.control.Label
        VelocitymmsGauge                matlab.ui.control.Gauge
        VelocitymmsGaugeLabel           matlab.ui.control.Label
        PressureatmGauge                matlab.ui.control.Gauge
        PressureatmGaugeLabel           matlab.ui.control.Label
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
        HomeButton                      matlab.ui.control.Button
        ManualControlButton             matlab.ui.control.StateButton
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
        statusCode
        updateTimer
        appRequestedModePath
        stateDiagramPath
        targetPositionPath
        targetVelocityPath
        jogDirectionPath
        targetPressurePath
        mixerPath
        ESTOPPath
        homingComplete
    end
    
    methods (Access = private)
        
        function pollModel(app)
            %disp("Polling model")
            rto = get_param(app.stateDiagramPath,"RuntimeObject");
            if size(rto,1) == 0
                app.statusCode = 0;
                %disp(app.statusCode)
            else  
                app.statusCode = rto.OutputPort(6).Data;
                %disp(app.statusCode)
            end

            updateStatusbar(app);
        end

        function updateStatusbar(app)
            if app.statusCode == 0
                app.StatusLabel.Text = "Idle";
            elseif app.statusCode == 1
                app.StatusLabel.Text = "Homing";
            end
        end
        
        function salineTabEnableChecks(app)
            cbStatus1 = app.SalineCheckBox1.Value;
            cbStatus2 = app.SalineCheckBox1.Value;
            cbStatus3 = app.SalineCheckBox1.Value;
            cbStatus4 = app.SalineCheckBox1.Value;
            cbStatus5 = app.SalineCheckBox1.Value;

            if cbStatus1 && cbStatus2 && cbStatus3 && cbStatus4 &&...
                    cbStatus5
                app.SalineButton1.Enable = true;
            else
                app.SalineButton1.Enable = false;
            end
        end
    end


    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            clc

            app.modelName = string(app.Simulation.ModelName);
            app.statusCode = 0;

            app.updateTimer = timer( ...
                ExecutionMode="fixedSpacing", ...
                Period=1, ...
                BusyMode="drop", ...
                TimerFcn=@(~,~) pollModel(app));
            start(app.updateTimer);
        end

        % Button pushed function: ConnectToCompressorButton
        function ConnectButtonPushed(app, event)
            app.StatusLabel.Text = "Connecting to Simulink...";
            drawnow;
            
            try
                if app.Simulation.Status == "running"
                    app.StatusLabel.Text = "Model is already running";
                else
                    start(app.Simulation);
                    if app.Simulation.Status == "running"
                        app.appRequestedModePath = append(app.modelName,...
                            "/RequestedMode");
                        app.stateDiagramPath = append(app.modelName,...
                            "/StateDiagram");
                        app.targetPositionPath = append(app.modelName,...
                            "/TargetPosition");
                        app.targetVelocityPath = append(app.modelName,...
                            "/TargetVelocity");
                        app.jogDirectionPath = append(app.modelName,...
                            "/JogDirection");
                        app.targetPressurePath = append(app.modelName,...
                            "/TargetPressure");
                        app.mixerPath = append(app.modelName, "/Mixer");
                        app.ESTOPPath = append(app.modelName, "/ESTOP");
                        set_param(app.appRequestedModePath, "Value", "0");
                        app.StatusLabel.Text = "Connected to Simulink";
                    else
                        app.StatusLabel.Text = append(...
                            "Simulink Status: ",app.Simulation.Status);
                    end
                end
            catch ME
                app.StatusLabel.Text = "Model unable to start";
            end
        end

        % Value changed function: ManualControlButton
        function ManualControlButtonPressed(app, event)
            value = app.ManualControlButton.Value;
            app.HomeButton.Enable = value;
            app.TargetPositionSpinner.Enable = value;
            app.TargetPositionmmSpinnerLabel.Enable = value;
            app.PositionGoButton.Enable = value;
            app.TargetPressureSpinner.Enable = value;
            app.TargetPressureatmSpinnerLabel.Enable = value;
            app.PressureGoButton.Enable = value;
            app.JogDownButton.Enable = value;
            app.JogUpButton.Enable = value;
            app.MixerButton.Enable = value;
        end

        % Button pushed function: HomeButton
        function HomeButtonPushed(app, event)
            set_param(app.appRequestedModePath, "Value", "1");
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
            app.ParameterField1Label.enable = false;
            app.ParameterField2.Enable = false;
            app.ParameterField2Label.enable = false;
            app.ParameterField2.Enable = false;
            app.ParameterField2Label.enable = false;

            % Enable the checkboxes in the saline fill tab
            app.SalineCheckBox1.enable = true;
            app.SalineCheckBox2.enable = true;
            app.SalineCheckBox3.enable = true;
            app.SalineCheckBox4.enable = true;
            app.SalineCheckBox5.enable = true;
        end

        % Button pushed function: PositionGoButton
        function PositionGoButtonPushed(app, event)
            set_param(app.targetPositionPath, "Value",...
                num2str(app.TargetPositionSpinner.Value));
            set_param(app.appRequestedModePath, "Value", "2");
        end

        % Button pushed function: PressureGoButton
        function PressureGoButtonPushed(app, event)
            set_param(app.targetPressurePath, "Value",...
                num2str(app.TargetPressureSpinner.Value));
            set_param(app.appRequestedModePath, "Value", "4");
        end

        % Value changed function: JogDownButton
        function JogDownButtonPushed(app, event)
            value = app.JogDownButton.Value;
            if value == true
                set_param(app.targetVelocityPath, "Value",...
                    num2str(app.appRequestedModePath));
                set_param(app.jogDirectionPath, "Value", "-1");
                set_param(app.appRequestedModePath, "Value", "3");
            else
                set_param(app.appRequestedModePath, "Value", "0");
            end
        end

        % Value changed function: JogUpButton
        function JogUpButtonPushed(app, event)
            value = app.JogUpButton.Value;
            if value == true
                set_param(app.targetVelocityPath, "Value",...
                    num2str(app.appRequestedModePath));
                set_param(app.jogDirectionPath, "Value", "1");
                set_param(app.appRequestedModePath, "Value", "3");
            else
                set_param(app.appRequestedModePath, "Value", "0");
            end
        end

        % Value changed function: MixerButton
        function MixerButtonPushed(app, event)
            value = app.MixerButton.Value;
            set_param(app.mixerPath, "Value", num2str(double(value)));
        end

        % Value changed function: ESTOP
        function ESTOPPushed(app, event)
            value = app.ESTOP.Value;
            set_param(app.ESTOPPath, "Value", num2str(double(value)));
        end
    end

    % Component initialization
    methods (Access = private)

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
            app.ManualControlButton.Position = [295 40 100 23];

            % Create HomeButton
            app.HomeButton = uibutton(app.ControlsPanel, 'push');
            app.HomeButton.ButtonPushedFcn = createCallbackFcn(app, @HomeButtonPushed, true);
            app.HomeButton.Enable = 'off';
            app.HomeButton.Position = [295 12 100 23];
            app.HomeButton.Text = 'Home';

            % Create TargetPositionmmSpinnerLabel
            app.TargetPositionmmSpinnerLabel = uilabel(app.ControlsPanel);
            app.TargetPositionmmSpinnerLabel.HorizontalAlignment = 'right';
            app.TargetPositionmmSpinnerLabel.Enable = 'off';
            app.TargetPositionmmSpinnerLabel.Position = [408 40 116 22];
            app.TargetPositionmmSpinnerLabel.Text = 'Target Position (mm)';

            % Create TargetPositionSpinner
            app.TargetPositionSpinner = uispinner(app.ControlsPanel);
            app.TargetPositionSpinner.Step = 5;
            app.TargetPositionSpinner.Limits = [180 365];
            app.TargetPositionSpinner.RoundFractionalValues = 'on';
            app.TargetPositionSpinner.Enable = 'off';
            app.TargetPositionSpinner.Position = [539 40 100 22];
            app.TargetPositionSpinner.Value = 180;

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
            app.ESTOP.Position = [174 13 100 49];

            % Create StatusLabel
            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.Position = [5 1 439 22];
            app.StatusLabel.Text = 'Idle';

            % Create PressureatmGaugeLabel
            app.PressureatmGaugeLabel = uilabel(app.UIFigure);
            app.PressureatmGaugeLabel.HorizontalAlignment = 'center';
            app.PressureatmGaugeLabel.Position = [54 273 84 22];
            app.PressureatmGaugeLabel.Text = 'Pressure (atm)';

            % Create PressureatmGauge
            app.PressureatmGauge = uigauge(app.UIFigure, 'circular');
            app.PressureatmGauge.Limits = [0 120];
            app.PressureatmGauge.Position = [35 310 120 120];

            % Create VelocitymmsGaugeLabel
            app.VelocitymmsGaugeLabel = uilabel(app.UIFigure);
            app.VelocitymmsGaugeLabel.HorizontalAlignment = 'center';
            app.VelocitymmsGaugeLabel.Position = [53 116 87 22];
            app.VelocitymmsGaugeLabel.Text = 'Velocity (mm/s)';

            % Create VelocitymmsGauge
            app.VelocitymmsGauge = uigauge(app.UIFigure, 'circular');
            app.VelocitymmsGauge.Limits = [0 5];
            app.VelocitymmsGauge.Position = [36 153 120 120];

            % Create PositionmmGaugeLabel
            app.PositionmmGaugeLabel = uilabel(app.UIFigure);
            app.PositionmmGaugeLabel.HorizontalAlignment = 'center';
            app.PositionmmGaugeLabel.Position = [228 276 79 22];
            app.PositionmmGaugeLabel.Text = 'Position (mm)';

            % Create PositionmmGauge
            app.PositionmmGauge = uigauge(app.UIFigure, 'circular');
            app.PositionmmGauge.Limits = [0 365];
            app.PositionmmGauge.MajorTicks = [0 80 160 240 320 365];
            app.PositionmmGauge.Position = [207 313 120 120];

            % Create FlowRatemLsGaugeLabel
            app.FlowRatemLsGaugeLabel = uilabel(app.UIFigure);
            app.FlowRatemLsGaugeLabel.HorizontalAlignment = 'center';
            app.FlowRatemLsGaugeLabel.Position = [221 116 96 22];
            app.FlowRatemLsGaugeLabel.Text = 'Flow Rate (mL/s)';

            % Create FlowRatemLsGauge
            app.FlowRatemLsGauge = uigauge(app.UIFigure, 'circular');
            app.FlowRatemLsGauge.Limits = [0 30];
            app.FlowRatemLsGauge.Position = [208 153 120 120];

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
        function app = SOS_Gen3_Compressor_Interface_exported

            % Associate the Simulink Model
            app.Simulation = simulation('SOS_Gen3_Compressor_Master');

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