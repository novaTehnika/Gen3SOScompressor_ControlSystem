classdef SlaveSim < handle
%SLAVESIM The compiled simulated slave (slave-sim/c, built by
%   buildSlaveSim), with the interface of the struct slaveInit returns:
%   sl.ax.x, sl.G.G_sysCurrentState, sl.hook.driveFault, sl.sim, sl.E ...
%   read and assign as they do on the struct (a struct assigned to a group
%   sets each of its fields), and slaveStep(sl, u) runs a scan. A field read returns a copy, so a group read (sl.G, sl.sim) is a
%   snapshot. Unlike the struct, sl is a handle: copies of it are the same
%   slave.
%
%   slaveInit returns one.

    properties (SetAccess = private)
        id          % the instance in slaveSimMex
    end

    properties (Access = private)
        enums
    end

    methods
        function obj = SlaveSim()
            obj.id = slaveSimMex('new');
            obj.enums = slaveSimMex('enums');
            sim = slaveSimConfig();
            names = fieldnames(sim);
            for k = 1:numel(names)
                slaveSimMex('set', obj.id, ['sim.' names{k}], sim.(names{k}));
            end
            slaveSimMex('reset', obj.id);
        end

        function delete(obj)
            slaveSimMex('free', obj.id);
        end

        function y = step(obj, u)
            % One scan with the holding registers set from u (engineering
            % units).
            y = slaveSimMex('step', obj.id, u);
        end

        function y = scan(obj)
            % One scan on the holding registers as they stand.
            y = slaveSimMex('scan', obj.id);
        end

        function writeHoldingRegisters(obj, start, words)
            % The master's write (FC16): 16-bit words from register start.
            slaveSimMex('writeRegisters', obj.id, 'holding', start, words);
        end

        function words = readInputRegisters(obj, start, n)
            % The master's read (FC04): n 16-bit words from register start.
            words = slaveSimMex('readRegisters', obj.id, 'input', start, n);
        end

        function varargout = subsref(obj, S)
            if ~strcmp(S(1).type, '.')
                error('SlaveSim:index', 'a simulated slave is accessed by field name');
            end
            if strcmp(S(1).subs, 'E')
                v = obj.enums;
                n = 1;
            elseif any(strcmp(S(1).subs, {'id', 'step', 'scan', 'delete', ...
                    'writeHoldingRegisters', 'readInputRegisters'}))
                [varargout{1:nargout}] = builtin('subsref', obj, S);
                return
            else
                path = S(1).subs;
                n = 1;
                while n < numel(S) && strcmp(S(n + 1).type, '.')
                    n = n + 1;
                    path = [path '.' S(n).subs]; %#ok<AGROW>
                end
                v = slaveSimMex('get', obj.id, path);
            end
            if n < numel(S)
                v = subsref(v, S(n + 1:end));
            end
            varargout{1} = v;
        end

        function obj = subsasgn(obj, S, v)
            path = S(1).subs;
            for k = 2:numel(S)
                if ~strcmp(S(k).type, '.')
                    error('SlaveSim:index', 'only whole fields of a simulated slave can be set');
                end
                path = [path '.' S(k).subs]; %#ok<AGROW>
            end
            setPath(obj.id, path, v);
        end
    end
end


function setPath(id, path, v)
% A struct assigned to a group (sl.hook = h, setfield(sl, 'ax', ...)) sets
% each of its fields.
if isstruct(v)
    f = fieldnames(v);
    for k = 1:numel(f)
        setPath(id, [path '.' f{k}], v.(f{k}));
    end
else
    slaveSimMex('set', id, path, v);
end
end
