function smokeMasterStateMachine()
%SMOKEMASTERSTATEMACHINE Run masterStateMachine against a protocol-level
%   stand-in for the slave and assert the expected sequencing.
%
%   The stand-in reproduces the slave's handshake, debounce, hold and fault
%   reset behaviour with a kinematic axis. It is not the slave simulator.
%
%   Run from the repository root:  addpath master-src/logic tests; smokeMasterStateMachine

h.cfg = masterConfig();
h.dt = 0.003;
h.sl = slaveInit();
h.y = slaveOutputs(h.sl);
h.in = struct('reset', 1, 'confCode', 0, 'faultActive', 0, 'inMotion', 0, ...
    'homingComplete', 0, 'brakeDisengage', 0, 'position', 0, 'estop', 0, ...
    'op', 0, 'cmdSeq', 0, 'targetPosition', 300, 'jogVelocity', 0, ...
    'pressureVelocity', 0);
h.out = [];
h.maxRef = 0;

h = runFor(h, 0.5);
assert(h.out.state == 1 && h.out.motionEnable == 0 && h.out.modeCode == 0);

% Position refused before homing.
h = command(h, 2);
h = runFor(h, 0.5);
assert(h.out.statusCode == 31 && h.out.motionEnable == 0);
fprintf('ok  position refused before homing\n');

% Homing.
h = command(h, 1);
h = runFor(h, 0.3);
assert(h.out.statusCode == 1 || h.out.statusCode == 7);
h = runFor(h, 6);
assert(h.out.homed == 1 && h.out.state == 1 && h.out.modeCode == 0);
assert(h.sl.faultCode == 0);
fprintf('ok  homing\n');

% Position move.
h.in.targetPosition = 30;
h = command(h, 2);
h = runFor(h, 15);
assert(h.out.statusCode == 3);
assert(abs(h.sl.pos - 30) <= h.cfg.posTolerance);
fprintf('ok  position move\n');

% Position -> jog without passing through idle.
h.in.jogVelocity = 2;
h = command(h, 3);
p0 = h.sl.pos;
h = runFor(h, 2);
assert(h.out.statusCode == 4 && h.sl.pos > p0 + 1);
assert(h.sl.brakeCycles == 2);     % homing + position entry only
h.in.op = 0;                        % button release: stop needs no cmdSeq
h = runFor(h, 1);
assert(h.out.state == 1 && h.out.motionEnable == 0 && h.out.refVoltage == 0);
fprintf('ok  position -> jog -> stop\n');

% Jog -> pressure share slave mode 3, so the change goes through idle.
h.in.jogVelocity = 1;
h = command(h, 3);
h = runFor(h, 1);
h.in.pressureVelocity = -1;
h = command(h, 4);
h = runFor(h, 2);
assert(h.out.statusCode == 5 && h.sl.state == 3 && h.sl.mode == 3);
h.in.op = 0;
h = runFor(h, 1);
fprintf('ok  jog -> pressure\n');

% Fault during a position move; no automatic reset; operator reset clears.
h.in.targetPosition = 100;
h = command(h, 2);
h = runFor(h, 2);
h.sl = slaveFault(h.sl, 3);
h = runFor(h, 0.5);
assert(h.out.statusCode == 13 && h.out.motionEnable == 0);
assert(h.out.modeCode == 3 && h.out.faultReset == 0);
h = runFor(h, 2);
assert(h.out.statusCode == 13);
h = command(h, 5);
h = runFor(h, 1);
assert(h.sl.faultCode == 0 && h.out.state == 1 && h.out.faultReset == 0);
h = runFor(h, 1);
assert(h.out.motionEnable == 0);   % the interrupted move does not resume
fprintf('ok  fault latch and operator reset\n');

% Homing-required fault voids the master's homed flag.
h.sl = slaveFault(h.sl, 4);
h = runFor(h, 0.5);
assert(h.out.homed == 0 && h.out.statusCode == 14);
h = command(h, 5);
h = runFor(h, 1);
h = command(h, 1);
h = runFor(h, 6);
assert(h.out.homed == 1);
fprintf('ok  homing-required fault\n');

% E-stop during jog; release does not restart motion.
h.in.jogVelocity = 2;
h = command(h, 3);
h = runFor(h, 1);
h.in.estop = 1;
h = runFor(h, 0.01);
assert(h.out.motionEnable == 0 && h.out.refVoltage == 0);
h = runFor(h, 1);
assert(h.out.statusCode == 20);
h.in.estop = 0;
h = runFor(h, 1);
assert(h.out.state == 1 && h.out.motionEnable == 0);
fprintf('ok  e-stop\n');

% Soft limit blocks jog beyond posMax.
h.in.targetPosition = h.cfg.posMax;
h = command(h, 2);
h = runFor(h, 150);
h.in.jogVelocity = 3;
h = command(h, 3);
h = runFor(h, 3);
assert(h.sl.pos <= h.cfg.posMax + 0.1 && h.sl.faultCode == 0);
h.in.op = 0;
h = runFor(h, 1);
fprintf('ok  soft limit\n');

assert(h.maxRef <= 10);
fprintf('all passed\n');
end


function h = command(h, op)
h.in.op = op;
h.in.cmdSeq = h.in.cmdSeq + 1;
end


function h = runFor(h, T)
n = round(T / h.dt);
for k = 1:n
    h.in.confCode       = h.y.confCode;
    h.in.faultActive    = h.y.faultActive;
    h.in.inMotion       = h.y.inMotion;
    h.in.homingComplete = h.y.homingComplete;
    h.in.brakeDisengage = h.y.brakeDisengage;
    h.in.position       = V2x(h.y.positionVolt, h.cfg);
    h.out = masterStateMachine(h.in, h.cfg, h.dt);
    h.in.reset = 0;
    h.maxRef = max(h.maxRef, abs(h.out.refVoltage));
    h.sl = slaveStep(h.sl, h.out, h.cfg, h.dt);
    h.y = slaveOutputs(h.sl);
end
end


% ---------------------------------------------------------------------------
% Slave stand-in
%   state: 0 idle, 1 enabling, 2 brake engage, 3 operating, 4 home complete,
%          5 hold, 6 fault
%   hs:    0 idle, 1 wait motion enable, 2 complete, 3 timeout
% ---------------------------------------------------------------------------

function sl = slaveInit()
sl.state = 0;  sl.t = 0;
sl.hs = 0;     sl.tHs = 0;   sl.hsMode = 0;  sl.hsConf = 0;
sl.mode = 0;
sl.pos = 250;  sl.moving = 0;
sl.homingRequired = 1;
sl.faultCode = 0;
sl.brake = 0;  sl.brakeCycles = 0;
sl.rawMode = 0;    sl.tRawMode = 0;  sl.reqMode = 0;  sl.tReqMode = 0;
sl.rawME = 0;      sl.tRawME = 0;    sl.me = 0;
sl.prevReset = 0;
end


function y = slaveOutputs(sl)
if sl.state == 6
    y.confCode = sl.faultCode;
else
    y.confCode = sl.hsConf;
end
y.faultActive    = double(sl.state == 6);
y.inMotion       = double(sl.moving ~= 0);
y.homingComplete = double(sl.state == 4);
y.brakeDisengage = sl.brake;
cfg = masterConfig();
y.positionVolt   = x2V(sl.pos, cfg);
end


function sl = slaveFault(sl, code)
sl.state = 6;  sl.t = 0;  sl.faultCode = code;  sl.moving = 0;
if code == 4
    sl.homingRequired = 1;
end
end


function sl = slaveStep(sl, u, cfg, dt)
% Input debounce: mode bits 50 ms, motion enable 20 ms.
if u.modeCode ~= sl.rawMode
    sl.rawMode = u.modeCode;  sl.tRawMode = 0;
else
    sl.tRawMode = sl.tRawMode + dt;
end
if sl.tRawMode >= 0.05 && sl.reqMode ~= sl.rawMode
    sl.reqMode = sl.rawMode;  sl.tReqMode = 0;
else
    sl.tReqMode = sl.tReqMode + dt;
end
modeStable = sl.tReqMode >= 0.02;

if u.motionEnable ~= sl.rawME
    sl.rawME = u.motionEnable;  sl.tRawME = 0;
else
    sl.tRawME = sl.tRawME + dt;
end
mePrev = sl.me;
if sl.tRawME >= 0.02
    sl.me = sl.rawME;
end
meFell = mePrev ~= 0 && sl.me == 0;

% Handshake manager, enabled in idle and hold only.
hsEnable = sl.state == 0 || sl.state == 5;
if ~hsEnable
    sl.hs = 0;  sl.hsConf = 0;
elseif sl.hs == 0
    if modeStable && sl.reqMode ~= 0 && sl.me == 0
        sl.hsConf = sl.reqMode;  sl.hsMode = sl.reqMode;
        sl.hs = 1;  sl.tHs = 0;
    elseif modeStable && sl.reqMode == 0
        sl.hsConf = 0;
    end
elseif sl.hs == 1
    sl.tHs = sl.tHs + dt;
    if sl.tHs >= 0.5
        sl.hs = 3;
    elseif sl.reqMode ~= sl.hsMode
        sl.hs = 0;
    elseif sl.me ~= 0
        sl.hs = 2;
    end
elseif sl.hs == 2
    if sl.me == 0
        sl.hs = 0;
    end
else
    sl.hsConf = 0;
    if sl.me == 0
        sl.hs = 0;
    end
end
hsComplete = sl.hs == 2;

resetEdge = u.faultReset ~= 0 && sl.prevReset == 0;
sl.prevReset = u.faultReset;

sl.t = sl.t + dt;
next = sl.state;

if sl.state == 0                                    % idle
    sl.brake = 0;  sl.moving = 0;
    if hsComplete
        sl.mode = sl.hsMode;
        next = 1;
    end

elseif sl.state == 1                                % drive enable + brake release
    if sl.t >= 0.2
        sl.brake = 1;  sl.brakeCycles = sl.brakeCycles + 1;
        next = enterMode(sl);
    end

elseif sl.state == 2                                % brake engage
    sl.brake = 0;
    if sl.t >= 0.2
        next = 0;
    end

elseif sl.state == 3                                % operating
    v = 0;
    if sl.mode == 2
        v = toward(sl.pos, V2x(u.refVoltage, cfg), 3, dt);
    elseif sl.mode == 3
        v = min(max(u.refVoltage * 10, -7), 7);
    elseif sl.mode == 5
        if sl.homingRequired ~= 0
            v = -60;                                % stand-in for the homing sequence
            if sl.pos <= 0
                sl.homingRequired = 0;
                next = 4;
            end
        else
            v = toward(sl.pos, 10, 3, dt);
        end
    end
    sl.pos = max(sl.pos + v * dt, 0);
    sl.moving = double(v ~= 0);
    if sl.pos > 365
        sl = slaveFault(sl, 3);
        next = 6;
    elseif meFell
        next = 5;
    end

elseif sl.state == 4                                % home complete
    sl.moving = 0;
    if meFell
        next = 5;
    end

elseif sl.state == 5                                % hold position
    halted = sl.t >= 0.15;
    sl.moving = double(~halted);
    if halted && modeStable && sl.reqMode == 0
        next = 2;
    elseif halted && hsComplete
        sl.mode = sl.hsMode;
        next = enterMode(sl);
    end

else                                                % fault
    sl.moving = 0;
    if resetEdge && sl.me == 0 && sl.reqMode == sl.faultCode
        sl.faultCode = 0;
        next = 2;
    end
end

if next == 6 && sl.state ~= 6 && sl.faultCode == 0
    sl.faultCode = 4;                               % operational mode while homing required
end
if next ~= sl.state
    sl.state = next;  sl.t = 0;
end
end


function next = enterMode(sl)
if sl.mode >= 1 && sl.mode <= 4 && sl.homingRequired ~= 0
    next = 6;
else
    next = 3;
end
end


function v = toward(pos, target, vMax, dt)
err = target - pos;
if abs(err) <= vMax * dt
    v = err / dt;
else
    v = sign(err) * vMax;
end
end
