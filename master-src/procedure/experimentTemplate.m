function proc = experimentTemplate()
%EXPERIMENTTEMPLATE A procedure file to copy from. Every kind of step the
%   engine runs appears once, with a comment on its fields.
%
%   proc = experimentTemplate()
%
%   Copy this file to master-src/procedure/<name>.m, rename the function to
%   match, keep the steps you need and delete the rest, then set
%   cfg.procedure in masterConfig.m to the name. Check the file without
%   the machine, from the repository root:
%
%       tests/run_all.sh smokeProcedureEngine
%
%   which runs it through procNew and reports the first thing it cannot
%   run. The operator app runs it from Start Procedure. The trace recorder
%   writes one CSV per run to master-src/captures, and tracePlot draws it.
%
%   A procedure is data: a name, parameters the operator can edit before
%   the run, and steps run in order. The "Procedure" section of
%   master-src/README.md lists every field; this file shows the common
%   ones.

cfg = masterConfig();

% ===== Recording =============================================================
%
% The master exchanges data with the compressor every 0.05 s (20 times a
% second). Each trace record summarises the scans inside the exchanges it
% covers: the mean pressure with its minimum and maximum, and the mean
% velocity, torque and position.
%
%   - 0.05 s is the fastest a trace can record. It is the floor for
%     continuous recording; a smaller value is raised to it.
%   - Choose a longer interval as a multiple of 0.05 s (0.1, 0.5, 1, 10,
%     60 ...), so that every record covers whole exchanges. A slow record
%     still carries the lowest and highest exchange values inside its
%     interval.
%   - Size the interval to the step: an hour at 0.05 s is 72 000 rows, at
%     10 s it is 360.
%
% record sets the interval for the whole procedure; a step's own record
% overrides it, and 'off' pauses recording for that step. Anything faster
% than an exchange is the job of the compressor's step probe at 0.004 s; a
% step will be able to request it in a later version.

record = 1;                        % s between records, unless a step says otherwise

% ===== Parameters ============================================================
%
% The operator sees these as fields before the run and must keep them
% within min and max. Steps refer to a parameter by its key: as a target
% or a signal argument, or in text as {key}. format sets how the value is
% shown; sink names a shell block that receives the value ('SalineVolume'
% or 'OxygenRatio', the fill the trace records).

params = {
    struct('key', 'targetPressure', 'label', 'Target pressure', ...
        'unit', 'atm', 'value', 5, 'min', cfg.atmAmbient, ...
        'max', cfg.pressureMax * cfg.atmPerMPa + cfg.atmAmbient)
    struct('key', 'targetPosition', 'label', 'Target position from EOT', ...
        'unit', 'mm', 'value', 200, 'min', 0, 'max', cfg.posEOT)
    struct('key', 'amplitude', 'label', 'Signal amplitude', ...
        'unit', 'atm', 'value', 0.5, 'min', 0, 'max', 5)
    struct('key', 'period', 'label', 'Signal period', ...
        'unit', 's', 'value', 20, 'min', 1, 'max', 3600)
    struct('key', 'holdTime', 'label', 'Hold time', ...
        'unit', 's', 'value', 60, 'min', 0, 'max', 36000)
    struct('key', 'useMixer', 'label', 'Use mixer', 'value', 1, ...
        'min', 0, 'max', 1, 'format', '%d')
    };

% ===== Steps =================================================================
%
% Common fields: section (a heading shown in the list; a step without one
% continues the previous section), title (required), text (shown to the
% operator; {key} is replaced by the parameter's value), id (so a later
% step can repeat from it), record (this step's interval), timerStart and
% timerStop (the procedure timer), zeroVolume (take the reference of the
% volume readout), mixer ('on', 'off' or a parameter key).
%
% Types: confirm, param, machine and wait. A machine step's op is home,
% goHome, position, pressure, track or stop; until says what completes
% it; keep leaves a pressure or position hold running through the steps
% that follow.

steps = {
    % ----- param: the operator enters the parameters listed -----
    struct('section', 'Setup', 'type', 'param', ...
        'title', 'Enter the parameters', ...
        'params', {{'targetPressure', 'targetPosition', 'amplitude', ...
        'period', 'holdTime', 'useMixer'}})

    % ----- confirm: a checklist; the button is enabled once every item
    % is ticked -----
    struct('type', 'confirm', 'title', 'Prepare the chamber', ...
        'checks', {{'Chamber filled and purged', 'All valves closed'}}, ...
        'confirmLabel', 'Continue')

    % ----- machine, home: home to the limit switch; complete when the
    % operation has ended -----
    struct('section', 'Moves', 'type', 'machine', 'title', 'Home the axis', ...
        'op', 'home')

    % ----- machine, position: a move to a target in mm from the end of
    % travel (unit 'mm') or to a chamber volume (unit 'mL'); complete on
    % arrival -----
    struct('type', 'machine', 'title', 'Move to the start position', ...
        'text', 'Moving to {targetPosition} mm from the end of travel.', ...
        'op', 'position', 'target', 'targetPosition', 'unit', 'mm')

    % ----- machine, pressure: pressure control at a target in atm
    % absolute. until 'running' completes once control runs; 'settled'
    % also waits for the pressure to stay within tolerance for duration.
    % keep leaves the hold running in the background -----
    struct('section', 'Pressure', 'type', 'machine', 'title', 'Pressurize', ...
        'text', 'Holding {targetPressure} atm.', 'op', 'pressure', ...
        'target', 'targetPressure', 'until', 'running', 'keep', true, ...
        'timerStart', true, 'mixer', 'useMixer')

    % ----- wait, settled: until the background pressure is within
    % tolerance of its target for the settle time (both from
    % masterConfig unless given as tolerance and duration) -----
    struct('type', 'wait', 'title', 'Pressure settling', 'until', 'settled')

    % ----- machine, track: follow a time series with the pressure (mode
    % 'pressure', atm) or the position (mode 'position', mm from the end
    % of travel) for duration seconds. signal is a function of time t in
    % seconds and the parameters p, or a two-column matrix [t value] that
    % holds its last value once it ends. The signal is sampled at every
    % exchange over the duration before the run, checked against the
    % pressure or position limits and the speed the axis can follow, and
    % played back one value per exchange. With relative true the series
    % is added to the value at step entry. record at the floor keeps every
    % exchange -----
    struct('id', 'signal', 'type', 'machine', 'title', 'Track a signal', ...
        'text', 'Sinusoid of {amplitude} atm about the target, period {period} s.', ...
        'op', 'track', 'mode', 'pressure', ...
        'signal', @(t, p) p.amplitude * sin(2 * pi * t / p.period), ...
        'relative', true, 'duration', 'holdTime', ...
        'record', 0.05)

    % ----- wait, timed: for duration seconds (a number or a parameter
    % key); a slow record interval keeps a long wait small -----
    struct('type', 'wait', 'title', 'Hold', 'duration', 'holdTime', ...
        'record', 10)

    % ----- confirm with repeat: a second button jumps back to the step
    % with that id -----
    struct('type', 'confirm', 'title', 'Repeat the signal?', ...
        'repeatFrom', 'signal', 'repeatLabel', 'Repeat', ...
        'confirmLabel', 'Finish')

    % ----- machine, goHome: move to the home position -----
    struct('section', 'Finish', 'type', 'machine', ...
        'title', 'End pressure control', 'op', 'stop', 'timerStop', true, ...
        'mixer', 'off')
    struct('type', 'machine', 'title', 'Return home', 'op', 'goHome')
    };

proc = struct('name', 'Experiment template', 'record', record, ...
    'params', {params}, 'steps', {steps}, ...
    'timerLabel', 'Time at pressure');
end
