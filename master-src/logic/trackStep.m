function [targetPositionOut, targetPressureOut, trackIndex] = trackStep( ...
    targetPosition, targetPressure, table, trackMode, trackSeq, statusCode)
%TRACKSTEP Entry point for the Simulink shell's TrackStep MATLAB Function
%   block: plays a table of targets, one value per Modbus exchange, in
%   place of the app's target Constant block while a track runs. Called
%   once per model step (cfg.dt) between the Constant blocks and masterStep.
%
%   targetPosition  the TargetPosition Constant, mm from the end of travel
%                   as the slave axis reads it
%   targetPressure  the TargetPressure Constant, atm absolute
%   table           the TrackTable Constant: the series, in the units of
%                   the mode's target, padded with its last value
%   trackMode       0 none, 2 position, 4 pressure (RequestedMode of the
%                   operation that follows the series)
%   trackSeq        counter the app increments for each track it starts
%   statusCode      the status masterStep reported one step earlier
%
%   targetPositionOut, targetPressureOut
%                   the inputs, except that the target of the mode in
%                   trackMode is table(trackIndex)
%   trackIndex      the table index in use, 0 when trackMode is 0
%
%   The index is floor(count / stepsPerExchange) + 1, where count is the
%   number of steps since trackSeq last changed in which the status showed
%   the mode's operation running (5 for pressure, 2 or 3 for position), so
%   the series starts when the slave is in mode and not during the
%   handshake. The index stops at the end of the table, which holds its
%   last value. The first call takes trackSeq as it is, so a model started
%   with a nonzero counter does not play. The state is initialised on the
%   first call; clear trackStep resets it. The table is indexed in place.

persistent lastSeq count

cfg = masterConfig();

if isempty(lastSeq)
    lastSeq = trackSeq;
    count = 0;
end
if trackSeq ~= lastSeq
    lastSeq = trackSeq;
    count = 0;
end

stepsPerExchange = round(cfg.exchangePeriod / cfg.dt);
nTable = numel(table);
idx = min(floor(count / stepsPerExchange) + 1, nTable);

targetPositionOut = targetPosition;
targetPressureOut = targetPressure;
trackIndex = 0;
running = false;
if trackMode == 4
    targetPressureOut = table(idx);
    trackIndex = idx;
    running = statusCode == 5;
elseif trackMode == 2
    targetPositionOut = table(idx);
    trackIndex = idx;
    running = statusCode == 2 || statusCode == 3;
end
if running && idx < nTable
    count = count + 1;
end
end
