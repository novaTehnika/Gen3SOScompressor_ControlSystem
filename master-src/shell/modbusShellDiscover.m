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
    fprintf('  %s =\n', hits{k});
    dump(v, '    ');
end

% The Modbus configuration component holds the transport settings.
cs = getActiveConfigSet(mdl);
fprintf('\nModbus configuration component:\n');
try
    c = getComponent(cs, 'Modbus');
    disp(c);
    names = properties(c);
    for k = 1:numel(names)
        try
            v = c.(names{k});
            fprintf('  %s =\n', names{k});
            dump(v, '    ');
        catch
            fprintf('  %s = <unreadable>\n', names{k});
        end
    end
    try
        fprintf('  get(c):\n');
        dump(get(c), '    ');
    catch
    end
catch err
    fprintf('  not readable: %s\n', err.message);
end
end


function dump(v, indent)
% Print any value fully, recursing into structs and cells.
if isstruct(v)
    for k = 1:numel(v)
        f = fieldnames(v(k));
        for j = 1:numel(f)
            fprintf('%s%s:\n', indent, f{j});
            dump(v(k).(f{j}), [indent '  ']);
        end
    end
elseif iscell(v)
    for k = 1:numel(v)
        fprintf('%s[%d]\n', indent, k);
        dump(v{k}, [indent '  ']);
    end
elseif ischar(v) || isstring(v)
    fprintf('%s''%s''\n', indent, char(string(v)));
elseif isnumeric(v) || islogical(v)
    fprintf('%s%s\n', indent, mat2str(v));
else
    fprintf('%s<%s>\n', indent, class(v));
    try
        disp(v);
    catch
    end
end
end
