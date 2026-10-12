function modbusShellDiscover()
%MODBUSSHELLDISCOVER Print the Modbus settings of the shell model after they
%   have been set by hand, so they can be pasted into MODBUS_SETUP in
%   buildShell.m. Run it with the configured model saved:
%
%       modbusShellDiscover
%
%   Prints the library path of the Modbus blocks, each block's register
%   table string and client name, and every model parameter whose name
%   mentions Modbus (where the client configuration is expected to live).

mdl = 'SOS_Gen3_Master_Shell';
load_system(mdl);
for name = {'Modbus Write', 'Modbus Read'}
    blk = [mdl '/' name{1}];
    fprintf('\n%s\n', blk);
    fprintf('  ReferenceBlock = %s\n', get_param(blk, 'ReferenceBlock'));
    fprintf('  ClientList     = %s\n', char(string(get_param(blk, 'ClientList'))));
    fprintf('  RegisterTable  = %s\n', char(string(get_param(blk, 'RegisterTable'))));
end

fprintf('\nModel parameters mentioning Modbus:\n');
params = fieldnames(get_param(mdl, 'ObjectParameters'));
hits = params(contains(lower(params), 'modbus'));
if isempty(hits)
    fprintf('  none\n');
end
for k = 1:numel(hits)
    v = get_param(mdl, hits{k});
    if ischar(v) || isstring(v)
        fprintf('  %s = %s\n', hits{k}, char(string(v)));
    else
        fprintf('  %s = <%s>\n', hits{k}, class(v));
        disp(v);
    end
end

cs = getActiveConfigSet(mdl);
fprintf('\nConfiguration set components:\n');
comps = cs.Components;
for k = 1:numel(comps)
    fprintf('  %s\n', comps(k).Name);
end
end
