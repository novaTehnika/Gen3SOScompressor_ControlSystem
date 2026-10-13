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
%   Two things are configured through dialogs the toolbox does not expose to
%   scripts, and are filled from MODBUS_SETUP below once they are known:
%   the register table of each Modbus block and the Modbus client (IP, port,
%   timeout, word order) in Configuration Parameters > Modbus. On the first
%   build, set them by hand as in master-src/README.md, then run
%   modbusShellDiscover and paste its output into MODBUS_SETUP so later
%   builds are complete.

% --- Values discovered on the lab PC (modbusShellDiscover) -----------------
% Leave empty until known; the builder then prints what to fill by hand.
MODBUS_SETUP.writeRegisterTable = '';   % get_param('.../Modbus Write', 'RegisterTable')
MODBUS_SETUP.readRegisterTable  = '';   % get_param('.../Modbus Read',  'RegisterTable')
MODBUS_SETUP.clientName         = 'ModbusClient';
MODBUS_LIBRARY = '';                    % library path of the Modbus blocks, if the scan fails

mdl = 'SOS_Gen3_Master_Shell';
here = fileparts(mfilename('fullpath'));
cfg = getConfig(here);
map = modbusMap();
Ts = num2str(cfg.dt);
Tx = num2str(cfg.exchangePeriod);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
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
    sprintf('HR1, holding registers, uint16, count %d', map.nWrite), manual);
manual = setRegisterTable(mdl, 'Modbus Read', MODBUS_SETUP.readRegisterTable, ...
    sprintf('IR1, input registers, uint16, count %d', map.nRead), manual);
try
    set_param([mdl '/Modbus Write'], 'ClientList', MODBUS_SETUP.clientName);
    set_param([mdl '/Modbus Read'],  'ClientList', MODBUS_SETUP.clientName);
catch
    manual{end + 1} = sprintf(['Modbus pane: Add client "%s", TCP/IP, controller IP, port 502, ' ...
        'timeout 0.1 s, byte order big-endian, word order little-endian; ' ...
        'then select it in both blocks'], MODBUS_SETUP.clientName);
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
add_line(mdl, 'Modbus Read/1', 'RT Read/1');
add_line(mdl, 'RT Read/1',     'MasterStep/1');
add_line(mdl, 'AI Pressure/1', 'MasterStep/2');
for k = 1:size(appInputs, 1)
    add_line(mdl, [appInputs{k, 1} '/1'], sprintf('MasterStep/%d', k + 2));
end
add_line(mdl, 'MasterStep/1', 'RT Write/1');
add_line(mdl, 'RT Write/1',   'Modbus Write/1');
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
    fprintf('\nSet by hand, then run modbusShellDiscover and fill MODBUS_SETUP in buildShell.m:\n');
    fprintf('  - %s\n', manual{:});
end
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
