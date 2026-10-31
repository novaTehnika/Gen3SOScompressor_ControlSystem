function b = slaveSimBackend()
%SLAVESIMBACKEND Which simulated slave slaveInit builds: 'c', the compiled
%   one (SlaveSim, built by buildSlaveSim), or 'm', the generated Octave
%   functions. The environment variable SLAVE_SIM chooses; without it, 'c'
%   when slaveSimMex is built and 'm' otherwise.

b = getenv('SLAVE_SIM');
if isempty(b)
    if exist('slaveSimMex', 'file') == 3
        b = 'c';
    else
        b = 'm';
    end
elseif ~any(strcmp(b, {'c', 'm'}))
    error('slaveSimBackend:value', 'SLAVE_SIM must be c or m, not %s', b);
end
end
