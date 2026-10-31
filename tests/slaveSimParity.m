function slaveSimParity()
%SLAVESIMPARITY Run the compiled simulated slave (SlaveSim, the C target of
%   tools/st_translate.py with slave-sim/c) and the generated Octave slave
%   side by side on the same holding registers, through every operating
%   mode, the fault hooks, link loss and the fault reset, and require them
%   to agree: the input registers after every scan, and every variable of
%   G, IO, the axis and the PRG_Main instance (nested blocks included)
%   every few scans. The REAL input registers are compared after rounding
%   the Octave value to single precision, as the controller stores them.
%   Run from the repository root after  octave-cli --eval "addpath slave-sim; buildSlaveSim"
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: slave-src slave-sim tools

if exist('slaveSimMex', 'file') ~= 3
    fprintf('skipped: the compiled slave is not built (slave-sim/buildSlaveSim.m)\n');
    return
end
h.m = slaveInit('m');
h.c = slaveInit('c');
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0);
% Every variable of G, IO, the axis and PRG_Main, as the compiled slave
% names them, and the same paths into the Octave slave's struct.
names = slaveSimMex('names');
h.pick = find(~cellfun(@isempty, regexp(names, '^(G|IO|ax|s)\.', 'once')));
h.names = names(h.pick);
h.subs = cellfun(@(n) cellfun(@(p) struct('type', '.', 'subs', p), strsplit(n, '.')), ...
    h.names, 'UniformOutput', false);
h.k = 0;
h.freeze = false;
h.scans = 0;
h.checks = 0;

h = run(h, 0.3);
assert(isequal(sort(fieldnames(h.m.ax)), sort(fieldnames(h.c.ax))), 'axis fields differ');

h = enter(h, 2, 60);                        % position
h = run(h, 4);
h = leave(h);
fprintf('ok  position\n');

h = enter(h, 3, 3);                         % velocity, both directions
h = run(h, 1.5);
h.u.reference = -2;
h = run(h, 1);
h = leave(h);
fprintf('ok  velocity\n');

h = enter(h, 4, 10);                        % torque
h = run(h, 1);
h = leave(h);
fprintf('ok  torque\n');

h = set(h, 'ax.x', 8);                      % homing from near the switch
h = set(h, 'ax.offset', 3.5);
h = enter(h, 6, 0);
h = run(h, 12, @(h) h.m.G.G_flagHomingComplete);
assert(h.m.G.G_flagHomingComplete, 'homing did not complete');
h = leave(h);
fprintf('ok  homing\n');

h = enter(h, 2, 10);                        % position, then Go Home
h = run(h, 3);
h = leave(h);
h = enter(h, 5, 0);
h = run(h, 5);
h = leave(h);
fprintf('ok  go home\n');

h = set(h, 'ax.x', chamberPosition(h.m.sim, 0.5));   % pressure, with outflow
h = enter(h, 7, 5);
h = run(h, 3);
h = set(h, 'hook.outflow', 300);
h = run(h, 1.5);
h = set(h, 'hook.outflow', 0);
h = run(h, 1);
h = leave(h);
fprintf('ok  pressure\n');

h = set(h, 'ax.x', 50);                     % drive fault during a move, reset
h = enter(h, 2, 60);
h = run(h, 1);
h = set(h, 'hook.driveFault', true);
h = run(h, 0.3);
h = set(h, 'hook.driveFault', false);
h = reset(h);
fprintf('ok  drive fault and reset\n');

h = enter(h, 3, 2);                         % heartbeat frozen during motion
h = run(h, 1);
h.freeze = true;
h = run(h, 0.6);
h.freeze = false;
h = reset(h);
fprintf('ok  link loss and reset\n');

h = set(h, 'ax.x', chamberPosition(h.m.sim, 1));     % broken sensor loop
h = enter(h, 7, 5);
h = run(h, 0.5);
h = set(h, 'hook.pressureVolt', 0);
h = run(h, 0.5);
h = set(h, 'hook.pressureVolt', NaN);
h = reset(h);
fprintf('ok  sensor fault and reset\n');

h = set(h, 'ax.x', 40);                     % stall and torque feedback
h = set(h, 'hook.torque', 35);
h = enter(h, 3, 2);
h = run(h, 0.5);
h = set(h, 'hook.blocked', true);
h = run(h, 1);
h = set(h, 'hook.blocked', false);
h = leave(h);
fprintf('ok  stall\n');

h = set(h, 'ax.x', h.m.G.G_cfgPosSoftLimitMax - 1);  % soft limit
h = enter(h, 3, 3);
h = run(h, 1.5);
if h.m.G.G_mbFaultActive
    h = reset(h);
else
    h = leave(h);
end
fprintf('ok  soft limit\n');

fprintf('all passed (%d scans, every variable compared %d times)\n', h.scans, h.checks);
end


function h = enter(h, mode, reference)
% The master's handshake: the mode with MotionEnable low, the slave's
% confirmation, then MotionEnable.
h.u.modeCode = mode;
h.u.motionEnable = 0;
h.u.reference = reference;
h = run(h, 2, @(h) h.m.G.G_mbConfCode == mode);
assert(h.m.G.G_mbConfCode == mode, 'mode %d not confirmed', mode);
h = run(h, 0.05);
h.u.motionEnable = 1;
h = run(h, 0.5);
end


function h = leave(h)
h.u.motionEnable = 0;
h = run(h, 0.5);
h.u.modeCode = 0;
h.u.reference = 0;
h = run(h, 1);
end


function h = reset(h)
% The register reset: MotionEnable low, the fault code mirrored in the
% mode register, then a FaultReset pulse.
h.u.motionEnable = 0;
h.u.modeCode = h.m.G.G_mbFaultCode;
h = run(h, 0.2);
h.u.faultReset = 1;
h = run(h, 2, @(h) ~h.m.G.G_mbFaultActive);
h.u.faultReset = 0;
h.u.modeCode = 0;
h.u.reference = 0;
h = run(h, 1);
end


function h = set(h, path, value)
S = cellfun(@(p) struct('type', '.', 'subs', p), strsplit(path, '.'));
h.m = subsasgn(h.m, S, value);
h.c = subsasgn(h.c, S, value);
end


function h = run(h, T, done)
dt = h.m.G.G_cfgScanTime;
for n = 1:round(T / dt)
    h.k = h.k + 1;
    if mod(h.k, round(0.025 / dt)) == 0 && ~h.freeze
        h.u.heartbeat = mod(h.u.heartbeat + 1, 65536);
    end
    [h.m, ym] = slaveStep(h.m, h.u);
    [h.c, yc] = slaveStep(h.c, h.u);
    h.scans = h.scans + 1;
    compareOutputs(ym, yc, h.k * dt);
    if mod(h.k, 25) == 0
        compareAll(h, h.k * dt);
        h.checks = h.checks + 1;
    end
    if nargin > 2 && done(h)
        return
    end
end
end


function compareOutputs(ym, yc, t)
real = {'actualPosition', 'actualVelocity', 'actualTorque', 'commandedPosition', ...
        'actualPressure'};
f = fieldnames(ym);
for k = 1:numel(f)
    m = double(ym.(f{k}));
    if any(strcmp(f{k}, real))
        m = double(single(m));
    end
    if ~isequaln(m, double(yc.(f{k})))
        error('slaveSimParity:y', 'y.%s at %.3f s: m %.9g, c %.9g', f{k}, t, m, yc.(f{k}));
    end
end
end


function compareAll(h, t)
% Equal to 1e-9 relative; the REAL registers G_mb* after rounding the
% Octave value to single precision.
values = slaveSimMex('values', h.c.id);
values = values(h.pick);
for k = 1:numel(values)
    c = double(values{k});
    m = double(subsref(h.m, h.subs{k}));
    if isequaln(m, c) || all(abs(m - c) <= 1e-9 * max(1, abs(m)))
        continue
    end
    if strncmp(h.names{k}, 'G.G_mb', 6) && isequaln(double(single(m)), c)
        continue
    end
    error('slaveSimParity:state', '%s at %.3f s: m %s, c %s', h.names{k}, t, ...
        mat2str(m, 10), mat2str(c, 10));
end
end
