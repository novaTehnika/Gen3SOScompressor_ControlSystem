function buildShell()
%BUILDSHELL Generate the Simulink shell model SOS_Gen3_Master_Shell.slx.
%
%   The shell holds no logic: the Modbus Client blocks, the DAQ blocks for
%   the safety relay and the stir bar motor, one MATLAB
%   Function block that calls masterStep (master-src/logic), the Constant
%   blocks the app writes and named Gain blocks the app reads. A second
%   MATLAB Function block, TraceStep, calls traceStep (master-src/logic) with
%   the link vectors, masterStep's status outputs and the app's Constant
%   blocks; it outputs the trace ring, which the TraceRing Gain block (a
%   test point) exposes to the app. A third, TrackStep, calls trackStep
%   (master-src/logic): it replaces the TargetPosition or TargetPressure
%   the app wrote with the value of the track table for the exchange while a
%   procedure's track step plays, and passes them through otherwise. Run in MATLAB
%   with Simulink, Simulink Desktop Real-Time and Industrial Communication
%   Toolbox (R2024b or later), from any folder:
%
%       run('master-src/shell/buildShell.m')     or     buildShell
%
%   Timing: the model steps every cfg.dt (25 ms); the Modbus blocks run
%   every cfg.exchangePeriod (50 ms) through Rate Transition blocks. The
%   DAQ blocks are created by addDaq.m, which holds the board setup.
%
%   The Modbus blocks' register tables and the Modbus client (Configuration
%   Parameters > Modbus) are set here in the forms the toolbox stores them,
%   found with modbusShellDiscover on the lab PC (R2025b).

% --- Modbus settings --------------------------------------------------------
MODBUS_LIBRARY = 'modbuslib';
MODBUS_SETUP.clientName = 'ModbusClient';
MODBUS_SETUP.ip         = '192.168.1.1';     % the controller, as MotionWorks connects to it
MODBUS_SETUP.port       = 502;
MODBUS_SETUP.timeout    = 0.5;               % s, per transaction; a timeout closes and reopens the connection, so leave room above the slowest live reply
MODBUS_SETUP.retries    = 1;
MODBUS_SETUP.byteOrder  = 'big-endian';
MODBUS_SETUP.wordOrder  = 'little-endian';   % as measured: low word first
% The Write block writes as many registers as its input is wide, so its row
% has no count; the Read block's row carries the count.
MODBUS_SETUP.writeRegisterTable = '{ ''1'', ''Reg_1'', ''1'', ''Holding Register'', ''uint16'' }';
MODBUS_SETUP.readRegisterTable  = sprintf( ...
    '{ ''1'', ''Reg_1'', ''1'', ''Input Register'', ''uint16'', ''%d'' }', modbusMap().nRead);

mdl = 'SOS_Gen3_Master_Shell';
here = fileparts(mfilename('fullpath'));
cfg = getConfig(here);
map = modbusMap();
Ts = num2str(cfg.dt);
Tx = num2str(cfg.exchangePeriod);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
target = fullfile(here, [mdl '.slx']);
if exist(target, 'file')
    delete(target);         % a previous build on the path would shadow the new model
end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', Ts, 'StopTime', 'inf', ...
    'PreLoadFcn', ['addpath(fullfile(fileparts(which(''' mdl ''')), ''..'', ''logic''));']);
% The track table is a model-workspace variable that the TrackTable
% Constant reads; tunable, so the app can replace it while the model runs.
set_param(mdl, 'DefaultParameterBehavior', 'Tunable');
assignin(get_param(mdl, 'ModelWorkspace'), 'trackTable', zeros(1, cfg.trackTableLength));

% --- DAQ blocks ---------------------------------------------------------
addDaq(mdl, 'DO Safety',   'Digital Output', '1', Ts);     % P0.0 safety relay
addDaq(mdl, 'DO Mixer',    'Digital Output', '5', Ts);     % P0.4 stir bar
set_param([mdl '/DO Safety'], 'InitialValue', '0', 'FinalValue', '0');
set_param([mdl '/DO Mixer'],  'InitialValue', '0', 'FinalValue', '0');

% --- Modbus Client blocks --------------------------------------------------
if isempty(MODBUS_LIBRARY)
    lib = modbusLibrary();
else
    load_system(MODBUS_LIBRARY);
    lib = MODBUS_LIBRARY;
end
add_block([lib '/Modbus Client Write'], [mdl '/Modbus Write']);
add_block([lib '/Modbus Client Read'],  [mdl '/Modbus Read']);
for name = {'Modbus Write', 'Modbus Read'}
    set_param([mdl '/' name{1}], 'SampleTime', Tx, 'ServerId', '1');
end
manual = {};
manual = setRegisterTable(mdl, 'Modbus Write', MODBUS_SETUP.writeRegisterTable, ...
    sprintf('HR1, holding register, uint16 (no count: the input is 1x%d)', map.nWrite), manual);
manual = setRegisterTable(mdl, 'Modbus Read', MODBUS_SETUP.readRegisterTable, ...
    sprintf('IR1, input register, uint16, count %d', map.nRead), manual);

% Modbus client: the model parameter behind Configuration Parameters > Modbus.
endpoint = sprintf('%s:%d', MODBUS_SETUP.ip, MODBUS_SETUP.port);
info.ClientInfoArray = struct( ...
    'Name', MODBUS_SETUP.clientName, 'TransportProtocol', 'tcpip', ...
    'TransportSettings', struct( ...
        'DeviceAddress', MODBUS_SETUP.ip, 'Port', MODBUS_SETUP.port, ...
        'ByteOrder', MODBUS_SETUP.byteOrder, 'WordOrder', MODBUS_SETUP.wordOrder, ...
        'Timeout', MODBUS_SETUP.timeout, 'NumRetries', MODBUS_SETUP.retries));
info.ColumnHeaders  = true;
info.ColumnLabels   = {'Select', 'Client Name', 'Transport', 'Connection Endpoint'};
info.RowHeaders     = false;
info.SelectRow      = false;
info.Size           = [1 4];
info.Data           = {false, MODBUS_SETUP.clientName, 'TCP/IP', endpoint};
info.Types          = {'checkbox', 'edit', 'edit', 'edit'};
info.ColumnEditable = [true false false false];
info.ColumnIDs      = {'Select', 'ConfigurationName', 'Transport', 'Connection'};
try
    set_param(mdl, 'ModbusTableInfo', info);
    set_param([mdl '/Modbus Write'], 'ClientList', MODBUS_SETUP.clientName);
    set_param([mdl '/Modbus Read'],  'ClientList', MODBUS_SETUP.clientName);
catch err
    manual{end + 1} = sprintf(['Modbus pane: Add client "%s", TCP/IP, %s, timeout %g s, ' ...
        'byte order %s, word order %s; then Refresh and select it in both blocks ' ...
        '(scripted attempt failed: %s)'], MODBUS_SETUP.clientName, endpoint, ...
        MODBUS_SETUP.timeout, MODBUS_SETUP.byteOrder, MODBUS_SETUP.wordOrder, err.message);
end

% Rate transitions between the 25 ms step and the 50 ms exchange.
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT Write'], ...
    'OutPortSampleTime', Tx);
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT Read'], ...
    'OutPortSampleTime', Ts);

% --- Constants written by the app (masterStep inputs 2..12, in order) ------
appInputs = {'RequestedMode', '0'; 'CommandSeq', '0'; 'TargetPosition', '300'; ...
             'TargetVelocity', num2str(speedFromPercent(cfg.speedDefault, cfg)); ...
             'JogDirection', '0'; 'TargetPressure', '5'; ...
             'ESTOP', '0'; 'Mixer', '0'; ...
             'PositionSpeed', num2str(speedFromPercent(cfg.speedDefault, cfg)); ...
             'SalineVolume', num2str(cfg.salineVolume); 'OxygenRatio', num2str(cfg.oxygenRatio)};
for k = 1:size(appInputs, 1)
    add_block('simulink/Sources/Constant', [mdl '/' appInputs{k, 1}], ...
        'Value', appInputs{k, 2}, 'SampleTime', Ts);
end

% --- Constants written by the app that are not masterStep inputs -------------
% TrackTable: the series a track step plays (one value per exchange; the
% model-workspace variable, replaced by the app). TrackMode: 0 none, 2
% position, 4 pressure. TrackSeq: counts the track starts. ProcStep: the
% running procedure's step number, 0 outside a run.
add_block('simulink/Sources/Constant', [mdl '/TrackTable'], ...
    'Value', 'trackTable', 'SampleTime', Ts);
otherInputs = {'TrackMode', '0'; 'TrackSeq', '0'; 'ProcStep', '0'};
for k = 1:size(otherInputs, 1)
    add_block('simulink/Sources/Constant', [mdl '/' otherInputs{k, 1}], ...
        'Value', otherInputs{k, 2}, 'SampleTime', Ts);
end
% The status code one step late: the track plays while it shows the mode's
% operation running.
add_block('simulink/Discrete/Unit Delay', [mdl '/StatusDelay'], ...
    'InitialCondition', '0', 'SampleTime', Ts);

% --- MATLAB Function block --------------------------------------------------
blk = [mdl '/MasterStep'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = sprintf([ ...
    'function [mbWrite, doSafety, doMixer, position, velocity, pressure, ...\n' ...
    '          statusCode, faultCode, state, linkOk, pressureBlock, ...\n' ...
    '          atEndOfTravel] = fcn( ...\n' ...
    '          mbRead, op, cmdSeq, targetPosition, ...\n' ...
    '          targetVelocity, jogDirection, targetPressure, estop, mixer, ...\n' ...
    '          positionSpeed, salineVolume, oxygenRatio)\n' ...
    '%% All logic lives in master-src/logic/masterStep.m.\n' ...
    '[mbWrite, doSafety, doMixer, position, velocity, pressure, ...\n' ...
    '    statusCode, faultCode, state, linkOk, pressureBlock, ...\n' ...
    '    atEndOfTravel] = masterStep( ...\n' ...
    '    double(mbRead(:)''), op, cmdSeq, targetPosition, ...\n' ...
    '    targetVelocity, jogDirection, targetPressure, estop, mixer, ...\n' ...
    '    positionSpeed, salineVolume, oxygenRatio);\n']);

% --- Track playback ----------------------------------------------------------
% Inputs 1..6: the app's TargetPosition and TargetPressure, the TrackTable,
% TrackMode, TrackSeq and the delayed status code. Outputs 1..2 replace the
% targets masterStep receives; output 3 is the table index in play.
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/TrackStep']);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/TrackStep']);
chart.Script = sprintf([ ...
    'function [targetPositionOut, targetPressureOut, trackIndex] = fcn( ...\n' ...
    '          targetPosition, targetPressure, trackTable, trackMode, trackSeq, ...\n' ...
    '          statusCode)\n' ...
    '%% All logic lives in master-src/logic/trackStep.m.\n' ...
    '[targetPositionOut, targetPressureOut, trackIndex] = trackStep( ...\n' ...
    '    targetPosition, targetPressure, trackTable, trackMode, trackSeq, ...\n' ...
    '    statusCode);\n']);
add_block('simulink/Math Operations/Gain', [mdl '/TrackIndex'], 'Gain', '1');
add_block('simulink/Sinks/Display', [mdl '/TrackIndex Display']);

% --- Trace recorder ----------------------------------------------------------
% Inputs 1..14: the read vector, the write vector as sent, masterStep's
% doSafety, statusCode, faultCode, state and linkOk, the app's RequestedMode
% and CommandSeq, the targets masterStep received (TrackStep's outputs),
% the app's ESTOP and Mixer and the procedure step.
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/TraceStep']);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [mdl '/TraceStep']);
chart.Script = sprintf([ ...
    'function traceRing = fcn(mbRead, mbWrite, doSafety, statusCode, faultCode, ...\n' ...
    '          state, linkOk, op, cmdSeq, targetPosition, targetPressure, estop, mixer, ...\n' ...
    '          procStep)\n' ...
    '%% All logic lives in master-src/logic/traceStep.m.\n' ...
    'traceRing = traceStep(double(mbRead(:)''), mbWrite, doSafety, statusCode, ...\n' ...
    '    faultCode, state, linkOk, op, cmdSeq, targetPosition, targetPressure, ...\n' ...
    '    estop, mixer, procStep);\n']);
add_block('simulink/Math Operations/Gain', [mdl '/TraceRing'], 'Gain', '1');
add_block('simulink/Sinks/Terminator', [mdl '/TraceRing Term']);

% --- Wiring -----------------------------------------------------------------
% The Modbus blocks have ports only once their register rows exist.
if hasPort(mdl, 'Modbus Read', 'Outport')
    add_line(mdl, 'Modbus Read/1', 'RT Read/1');
else
    manual{end + 1} = 'connect Modbus Read/1 -> RT Read/1 after adding its register row';
end
add_line(mdl, 'RT Read/1', 'MasterStep/1');
for k = 1:size(appInputs, 1)
    if any(strcmp(appInputs{k, 1}, {'TargetPosition', 'TargetPressure'}))
        continue;                            % TrackStep passes these on
    end
    add_line(mdl, [appInputs{k, 1} '/1'], sprintf('MasterStep/%d', k + 1));
end
add_line(mdl, 'MasterStep/1', 'RT Write/1');
if hasPort(mdl, 'Modbus Write', 'Inport')
    add_line(mdl, 'RT Write/1', 'Modbus Write/1');
else
    manual{end + 1} = 'connect RT Write/1 -> Modbus Write/1 after adding its register row';
end
add_line(mdl, 'MasterStep/2', 'DO Safety/1');
add_line(mdl, 'MasterStep/3', 'DO Mixer/1');

% TrackStep sits between the app's targets and masterStep (inputs 4 and 7).
add_line(mdl, 'TargetPosition/1', 'TrackStep/1');
add_line(mdl, 'TargetPressure/1', 'TrackStep/2');
add_line(mdl, 'TrackTable/1',     'TrackStep/3');
add_line(mdl, 'TrackMode/1',      'TrackStep/4');
add_line(mdl, 'TrackSeq/1',       'TrackStep/5');
add_line(mdl, 'MasterStep/7',     'StatusDelay/1');
add_line(mdl, 'StatusDelay/1',    'TrackStep/6');
add_line(mdl, 'TrackStep/1',      'MasterStep/4');
add_line(mdl, 'TrackStep/2',      'MasterStep/7');
add_line(mdl, 'TrackStep/3',      'TrackIndex/1');
add_line(mdl, 'TrackIndex/1',     'TrackIndex Display/1');

% TraceStep branches off the sources above.
add_line(mdl, 'RT Read/1',    'TraceStep/1');
add_line(mdl, 'MasterStep/1', 'TraceStep/2');
add_line(mdl, 'MasterStep/2', 'TraceStep/3');
for k = 7:10                                 % statusCode, faultCode, state, linkOk
    add_line(mdl, sprintf('MasterStep/%d', k), sprintf('TraceStep/%d', k - 3));
end
add_line(mdl, 'RequestedMode/1', 'TraceStep/8');
add_line(mdl, 'CommandSeq/1',    'TraceStep/9');
add_line(mdl, 'TrackStep/1',     'TraceStep/10');     % the targets masterStep received
add_line(mdl, 'TrackStep/2',     'TraceStep/11');
add_line(mdl, 'ESTOP/1',         'TraceStep/12');
add_line(mdl, 'Mixer/1',         'TraceStep/13');
add_line(mdl, 'ProcStep/1',      'TraceStep/14');
add_line(mdl, 'TraceStep/1', 'TraceRing/1');
add_line(mdl, 'TraceRing/1', 'TraceRing Term/1');
% A test point keeps the signal's buffer, which the app reads through
% get_param(..., 'RuntimeObject') while the model runs.
ph = get_param([mdl '/TraceRing'], 'PortHandles');
set_param(ph.Outport(1), 'TestPoint', 'on');

% Values read by the app through get_param(..., 'RuntimeObject').
appOutputs = {'Position', 'Velocity', 'Pressure', 'StatusCode', 'FaultCode', ...
              'State', 'LinkOk', 'PressureBlock', 'AtEndOfTravel'};
for k = 1:numel(appOutputs)
    name = appOutputs{k};
    add_block('simulink/Math Operations/Gain', [mdl '/' name], 'Gain', '1');
    add_block('simulink/Sinks/Display', [mdl '/' name ' Display']);
    add_line(mdl, sprintf('MasterStep/%d', k + 3), [name '/1']);
    add_line(mdl, [name '/1'], [name ' Display/1']);
end

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Built %s\n', fullfile(here, [mdl '.slx']));
if ~isempty(manual)
    fprintf('\nSet by hand (see master-src/README.md):\n');
    fprintf('  - %s\n', manual{:});
end
end


function tf = hasPort(mdl, name, kind)
ph = get_param([mdl '/' name], 'PortHandles');
tf = ~isempty(ph.(kind));
end


function manual = setRegisterTable(mdl, name, value, description, manual)
% Apply a discovered register table string, or ask for it to be set by hand.
if ~isempty(value)
    try
        set_param([mdl '/' name], 'RegisterTable', value);
        return;
    catch err
        warning('%s: RegisterTable not accepted (%s)', name, err.message);
    end
end
manual{end + 1} = sprintf('%s: one register row, %s', name, description);
end


function lib = modbusLibrary()
% Path of the library holding the Modbus Client blocks (Industrial
% Communication Toolbox). The library file name is not documented, so the
% registered block libraries are scanned for a block of that name.
persistent found
if ~isempty(found)
    lib = found;
    return;
end
files = which('slblocks', '-all');
for k = 1:numel(files)
    folder = fileparts(files{k});
    try
        back = cd(folder);
        restore = onCleanup(@() cd(back));
        s = slblocks();
        clear restore
    catch
        continue;
    end
    if ~isfield(s, 'Browser')
        continue;
    end
    for b = 1:numel(s.Browser)
        name = s.Browser(b).Library;
        try
            load_system(name);
            hit = find_system(name, 'LookUnderMasks', 'none', 'FollowLinks', 'off', ...
                'Name', 'Modbus Client Read');
        catch
            continue;
        end
        if ~isempty(hit)
            found = get_param(hit{1}, 'Parent');
            lib = found;
            return;
        end
    end
end
error(['Modbus Client blocks not found. Drag one into a new model from the Library ' ...
       'Browser (Industrial Communication Toolbox > Modbus), select it and run ' ...
       'get_param(gcb, ''ReferenceBlock''); then set MODBUS_LIBRARY in buildShell.m ' ...
       'to the part before ''/Modbus Client Read''.']);
end


function cfg = getConfig(here)
addpath(here, fullfile(here, '..', 'logic'));
cfg = masterConfig();
end
