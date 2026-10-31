function buildSafetyHold()
%BUILDSAFETYHOLD Generate SafetyHold.slx: a Simulink Desktop Real-Time model
%   that holds the safety relay (P0.0) on and the stir bar (P0.4) off, for
%   runs that take the shell's place as Modbus master (pressureNoiseRun,
%   frictionRun, charRun). The relay drops when the model stops (final
%   value 0).
%   Run with Simulink and Simulink Desktop Real-Time from any folder:
%
%       buildSafetyHold
%
%   Then open the model, select Connected IO on the Desktop Real-Time tab
%   and save it, as for the shell. The run scripts start and stop it.

mdl = 'SafetyHold';
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'shell'), ...
        fullfile(here, '..', '..', 'master-src', 'logic'));
cfg = masterConfig();
Ts = num2str(cfg.dt);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
target = fullfile(here, [mdl '.slx']);
if exist(target, 'file')
    delete(target);
end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', Ts, 'StopTime', 'inf');

addDaq(mdl, 'DO Safety', 'Digital Output', '1', Ts);     % P0.0 safety relay
addDaq(mdl, 'DO Mixer',  'Digital Output', '5', Ts);     % P0.4 stir bar
set_param([mdl '/DO Safety'], 'InitialValue', '0', 'FinalValue', '0');
set_param([mdl '/DO Mixer'],  'InitialValue', '0', 'FinalValue', '0');
add_block('simulink/Sources/Constant', [mdl '/Relay On'],  'Value', '1', 'SampleTime', Ts);
add_block('simulink/Sources/Constant', [mdl '/Mixer Off'], 'Value', '0', 'SampleTime', Ts);
add_line(mdl, 'Relay On/1',  'DO Safety/1');
add_line(mdl, 'Mixer Off/1', 'DO Mixer/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl, target);
fprintf('Built %s\n', target);
end
