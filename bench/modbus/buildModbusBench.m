function buildModbusBench()
%BUILDMODBUSBENCH Generate ModbusBench.slx: the shell's DAQ input blocks for
%   Simulink Desktop Real-Time pacing and load, and one MATLAB Function block
%   that calls modbusBenchStep each step. Results are logged to benchLog as
%   [seq echo slaveCounter txnMs stepMs ok], one row per step.
%
%   The DAQ blocks are inputs only; no DAQ output is driven.

mdl = 'ModbusBench';
src = 'SimulinkMaster';
here = fileparts(mfilename('fullpath'));
addpath(here);
cfg = modbusBenchConfig();
Ts = num2str(cfg.dt);

load_system(fullfile(here, '..', '..', 'master-src', [src '.slx']));
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', Ts, 'StopTime', num2str(cfg.duration), ...
    'PreLoadFcn', 'addpath(fileparts(which(''ModbusBench'')));', ...
    'StopFcn', 'clear modbusBenchStep');

% --- DAQ inputs, as in the shell (channel numbers from buildShell.m) -------
copyDaq(src, 'Digital Input3', mdl, 'DI Slave',    '[47 45 42 41 40 39 48]', Ts);
copyDaq(src, 'Analog Input',   mdl, 'AI Position', '1', Ts);
copyDaq(src, 'Analog Input2',  mdl, 'AI Pressure', '3', Ts);
daq = {'DI Slave', 'AI Position', 'AI Pressure'};
for k = 1:numel(daq)
    add_block('simulink/Sinks/Terminator', [mdl '/' daq{k} ' Term']);
    add_line(mdl, [daq{k} '/1'], [daq{k} ' Term/1']);
end

% --- Step counter -----------------------------------------------------------
add_block('simulink/Sources/Digital Clock', [mdl '/Clock'], 'SampleTime', Ts);

% --- MATLAB Function block ----------------------------------------------------
blk = [mdl '/BenchStep'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = sprintf([ ...
    'function [seq, echo, slaveCounter, txnMs, stepMs, ok] = fcn(t)\n' ...
    '%% All logic lives in bench/modbus/modbusBenchStep.m.\n' ...
    'coder.extrinsic(''modbusBenchStep'');\n' ...
    'seq = mod(round(t / %s), 65536);\n' ...
    'echo = 0; slaveCounter = 0; txnMs = 0; stepMs = 0; ok = 0;\n' ...
    '[echo, slaveCounter, txnMs, stepMs, ok] = modbusBenchStep(seq);\n'], Ts);
add_line(mdl, 'Clock/1', 'BenchStep/1');

% --- Logging --------------------------------------------------------------
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], 'Inputs', '6');
add_block('simulink/Sinks/To Workspace', [mdl '/Log'], ...
    'VariableName', 'benchLog', 'SaveFormat', 'Array', 'SampleTime', Ts);
for k = 1:6
    add_line(mdl, sprintf('BenchStep/%d', k), sprintf('Mux/%d', k));
end
add_line(mdl, 'Mux/1', 'Log/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, fullfile(here, [mdl '.slx']));
close_system(src, 0);
fprintf('Built %s\n', fullfile(here, [mdl '.slx']));
end


function copyDaq(src, srcName, mdl, name, channels, Ts)
add_block([src '/' srcName], [mdl '/' name]);
set_param([mdl '/' name], 'Channels', channels, 'SampleTime', Ts);
end
