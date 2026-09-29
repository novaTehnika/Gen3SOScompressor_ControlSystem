function buildShell()
%BUILDSHELL Generate the Simulink shell model SOS_Gen3_Master_Shell.slx.
%
%   The shell holds no logic: DAQ blocks, one MATLAB Function block that
%   calls masterStep (master-src/logic), the Constant blocks the app writes
%   and named Gain blocks the app reads. Run in MATLAB with Simulink and
%   Simulink Desktop Real-Time, from any folder:
%
%       run('master-src/shell/buildShell.m')     or     buildShell
%
%   The DAQ blocks are copied from SimulinkMaster.slx so they keep its board
%   setup (National Instruments PCI-6251); only channels and sample time
%   are set here.
%
%   Simulink Desktop Real-Time digital channel numbers on the PCI-6251:
%   P0.n = n + 1, P1.n = n + 33, P2.n = n + 41.

mdl = 'SOS_Gen3_Master_Shell';
src = 'SimulinkMaster';
here = fileparts(mfilename('fullpath'));
cfg = getConfig(here);
Ts = num2str(cfg.dt);

load_system(fullfile(here, '..', [src '.slx']));
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', Ts, 'StopTime', 'inf', ...
    'PreLoadFcn', ['addpath(fullfile(fileparts(which(''' mdl ''')), ''..'', ''logic''));']);

% --- DAQ blocks ---------------------------------------------------------
% [conf0 conf1 conf2 performanceStatus faultActive homingComplete inMotion]
%  P2.6  P2.4  P2.1  P2.0              P1.7        P1.6           P2.7
copyDaq(src, 'Digital Input3', mdl, 'DI Slave',     '[47 45 42 41 40 39 48]', Ts);
% [safetyEnable modeBit1 motionEnable modeBit0 modeBit2 faultReset]
%  P0.0         P0.1     P0.2         P0.5     P0.6     P0.7
copyDaq(src, 'Digital Output3', mdl, 'DO Slave',    '[1 2 3 6 7 8]', Ts);
copyDaq(src, 'Digital Output',  mdl, 'DO Mixer',    '5', Ts);            % P0.4
copyDaq(src, 'Analog Input',    mdl, 'AI Position', '1', Ts);            % AI 0
copyDaq(src, 'Analog Input2',   mdl, 'AI Pressure', '3', Ts);            % AI 2
copyDaq(src, 'Analog Output',   mdl, 'AO Reference', '1', Ts);           % AO 0
set_param([mdl '/DO Slave'], 'InitialValue', '0', 'FinalValue', '0');
set_param([mdl '/DO Mixer'], 'InitialValue', '0', 'FinalValue', '0');
set_param([mdl '/AO Reference'], 'InitialValue', '0', 'FinalValue', '0');

% --- Constants written by the app (masterStep inputs 4..11, in order) -----
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
    'function [doSlave, doMixer, refVoltage, position, velocity, pressure, ...\n' ...
    '          statusCode, faultCode, state] = fcn( ...\n' ...
    '          diSlave, positionVolt, pressureVolt, op, cmdSeq, targetPosition, ...\n' ...
    '          targetVelocity, jogDirection, targetPressure, estop, mixer)\n' ...
    '%% All logic lives in master-src/logic/masterStep.m.\n' ...
    '[doSlave, doMixer, refVoltage, position, velocity, pressure, ...\n' ...
    '    statusCode, faultCode, state] = masterStep( ...\n' ...
    '    double(diSlave(:)''), positionVolt, pressureVolt, op, cmdSeq, targetPosition, ...\n' ...
    '    targetVelocity, jogDirection, targetPressure, estop, mixer);\n']);

% --- Wiring -----------------------------------------------------------------
add_line(mdl, 'DI Slave/1',    'MasterStep/1');
add_line(mdl, 'AI Position/1', 'MasterStep/2');
add_line(mdl, 'AI Pressure/1', 'MasterStep/3');
for k = 1:size(appInputs, 1)
    add_line(mdl, [appInputs{k, 1} '/1'], sprintf('MasterStep/%d', k + 3));
end
add_line(mdl, 'MasterStep/1', 'DO Slave/1');
add_line(mdl, 'MasterStep/2', 'DO Mixer/1');
add_line(mdl, 'MasterStep/3', 'AO Reference/1');

% Values read by the app through get_param(..., 'RuntimeObject').
appOutputs = {'Position', 'Velocity', 'Pressure', 'StatusCode', 'FaultCode', ...
              'State'};
for k = 1:numel(appOutputs)
    name = appOutputs{k};
    add_block('simulink/Math Operations/Gain', [mdl '/' name], 'Gain', '1');
    add_block('simulink/Sinks/Display', [mdl '/' name ' Display']);
    add_line(mdl, sprintf('MasterStep/%d', k + 3), [name '/1']);
    add_line(mdl, [name '/1'], [name ' Display/1']);
end

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(here, [mdl '.slx']));
close_system(src, 0);
fprintf('Built %s\n', fullfile(here, [mdl '.slx']));
end


function copyDaq(src, srcName, mdl, name, channels, Ts)
add_block([src '/' srcName], [mdl '/' name]);
set_param([mdl '/' name], 'Channels', channels, 'SampleTime', Ts);
end


function cfg = getConfig(here)
addpath(fullfile(here, '..', 'logic'));
cfg = masterConfig();
end
