function addDaq(mdl, name, type, channels, Ts)
%ADDDAQ Add a Simulink Desktop Real-Time DAQ block configured for the lab
%   PC's National Instruments board.
%
%   addDaq(mdl, name, type, channels, Ts)
%     type      'Analog Input', 'Analog Output', 'Digital Input' or
%               'Digital Output' (sldrtlib block names)
%     channels  channel vector as a string, e.g. '[1 2 3]'
%     Ts        sample time as a string
%
%   The board setup is the same for every block and lives here, so a board
%   change is one edit. Digital channel numbers: P0.n = n + 1,
%   P1.n = n + 33, P2.n = n + 41.

board.DrvName        = 'National_Instruments/PCI-6221';
board.DrvAddress     = '4294967295';      % auto-detect
board.DrvOptions     = '[1 255 0 0 0]';   % board setup dialog values
board.VoltRange      = '1';               % analog blocks only
board.MaxMissedTicks = '1e6';             % lag never stops the model

blk = [mdl '/' name];
add_block(['sldrtlib/' type], blk);
set_param(blk, 'DrvName', board.DrvName, 'DrvAddress', board.DrvAddress, ...
    'DrvOptions', board.DrvOptions, 'MaxMissedTicks', board.MaxMissedTicks, ...
    'Channels', channels, 'SampleTime', Ts);
if startsWith(type, 'Analog')
    set_param(blk, 'VoltRange', board.VoltRange);
end
end
