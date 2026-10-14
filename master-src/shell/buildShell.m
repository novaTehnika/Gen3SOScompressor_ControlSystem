function buildShell()
%BUILDSHELL Generate the Simulink shell model SOS_Gen3_Master_Shell.slx.
%
%   The shell holds no logic: the Modbus Client blocks, the DAQ blocks for
%   the safety relay, the stir bar motor and the pressure input, one MATLAB
%   Function block that calls masterStep (master-src/logic), the Constant
%   blocks the app writes and named Gain blocks the app reads. Run in MATLAB
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

% --- DAQ blocks ---------------------------------------------------------
addDaq(mdl, 'DO Safety',   'Digital Output', '1', Ts);     % P0.0 safety relay
addDaq(mdl, 'DO Mixer',    'Digital Output', '5', Ts);     % P0.4 stir bar
addDaq(mdl, 'AI Pressure', 'Analog Input',   '3', Ts);     % AI 2 transducer
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
    'HR1, holding register, uint16 (no count: the input is 1x6)', manual);
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

% --- Constants written by the app (masterStep inputs 3..10, in order) -----
appInputs = {'RequestedMode', '0'; 'CommandSeq', '0'; 'TargetPosition', '300'; ...
             'TargetVelocity', '0'; 'JogDirection', '0'; 'TargetPressure', '5'; ...
             'ESTOP', '0'; 'Mixer', '0'};
for k = 1:size(appInputs, 1)
    add_block('simulink/Sources/Constant', [mdl '/' appInputs{k, 1}], ...
        'Value', appInputs{k, 2}, 'SampleTime', Ts);
end

% --- MATLAB Function block --------------------------------------------------
blk = [mdl '/MasterStep'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = sprintf([ ...
    'function [mbWrite, doSafety, doMixer, position, velocity, pressure, ...\n' ...
    '          statusCode, faultCode, state, linkOk] = fcn( ...\n' ...
    '          mbRead, pressureVolt, op, cmdSeq, targetPosition, ...\n' ...
    '          targetVelocity, jogDirection, targetPressure, estop, mixer)\n' ...
    '%% All logic lives in master-src/logic/masterStep.m.\n' ...
    '[mbWrite, doSafety, doMixer, position, velocity, pressure, ...\n' ...
    '    statusCode, faultCode, state, linkOk] = masterStep( ...\n' ...
    '    double(mbRead(:)''), pressureVolt, op, cmdSeq, targetPosition, ...\n' ...
    '    targetVelocity, jogDirection, targetPressure, estop, mixer);\n']);

% --- Wiring -----------------------------------------------------------------
% The Modbus blocks have ports only once their register rows exist.
if hasPort(mdl, 'Modbus Read', 'Outport')
    add_line(mdl, 'Modbus Read/1', 'RT Read/1');
else
    manual{end + 1} = 'connect Modbus Read/1 -> RT Read/1 after adding its register row';
end
add_line(mdl, 'RT Read/1',     'MasterStep/1');
add_line(mdl, 'AI Pressure/1', 'MasterStep/2');
for k = 1:size(appInputs, 1)
    add_line(mdl, [appInputs{k, 1} '/1'], sprintf('MasterStep/%d', k + 2));
end
add_line(mdl, 'MasterStep/1', 'RT Write/1');
if hasPort(mdl, 'Modbus Write', 'Inport')
    add_line(mdl, 'RT Write/1', 'Modbus Write/1');
else
    manual{end + 1} = 'connect RT Write/1 -> Modbus Write/1 after adding its register row';
end
add_line(mdl, 'MasterStep/2', 'DO Safety/1');
add_line(mdl, 'MasterStep/3', 'DO Mixer/1');

% Values read by the app through get_param(..., 'RuntimeObject').
appOutputs = {'Position', 'Velocity', 'Pressure', 'StatusCode', 'FaultCode', ...
              'State', 'LinkOk'};
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
