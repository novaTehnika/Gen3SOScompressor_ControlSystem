function B = ctlBlocks()
%CTLBLOCKS The shell's blocks the controller writes and reads for the app.
%   B = ctlBlocks() returns
%     write  the Constant blocks the app may set (buildShell): the
%            masterStep inputs and the track, procedure and fill blocks.
%            The TrackTable is not among them; it goes through a 'track'
%            message (runController).
%     read   the Gain blocks whose output the controller sends in every
%            status message (masterStep's outputs 4 to 12)
%   The TraceRing is read separately: the controller drains it and sends
%   the new records.

B.write = {'RequestedMode', 'CommandSeq', 'TargetPosition', ...
    'TargetVelocity', 'JogDirection', 'TargetPressure', 'ESTOP', 'Mixer', ...
    'PositionSpeed', 'TrackMode', 'TrackSeq', 'ProcStep', ...
    'SalineVolume', 'OxygenRatio'};
B.read = {'Position', 'Velocity', 'Pressure', 'StatusCode', 'FaultCode', ...
    'State', 'LinkOk', 'PressureBlock', 'AtEndOfTravel'};
end
