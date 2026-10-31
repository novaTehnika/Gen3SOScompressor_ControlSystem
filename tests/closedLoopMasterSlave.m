function closedLoopMasterSlave(handshakeTimeout)
%CLOSEDLOOPMASTERSLAVE Run masterStateMachine against the simulated slave
%   (slave-sim, generated from slave-src) and assert the expected behaviour.
%
%   closedLoopMasterSlave()      slave handshake timeout 0.5 s (docs/master)
%   closedLoopMasterSlave(t)     slave handshake timeout t seconds
%
%   The master steps every 25 ms, the Modbus exchange runs every 50 ms and
%   the slave every scan (G_cfgScanTime, 4 ms), on a 1 ms tick. Registers are exchanged in
%   engineering units; packing is covered by closedLoopModbusStep.
%   Run from the repository root after  python3 tools/st_translate.py --target m slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools

if nargin < 1
    handshakeTimeout = 0.5;
end

h = harnessInit(handshakeTimeout);
E = h.sl.E;

h = runFor(h, 0.5);
assert(h.out.state == 1 && h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
vLimit = h.sl.G.G_cfgVelLimitNormal;

% Position move from power-up: lands inside tolerance, no overshoot,
% velocity limited. No homing is needed first. The move is long enough to
% reach the velocity limit.
target = h.sl.ax.x + 15;
h.in.targetPosition = target;
h.vMax = 0;  h.xMax = -inf;
h = command(h, 2);
h = runUntil(h, 15, @(h) h.out.statusCode == 3);
assert(h.out.statusCode == 3, 'position move did not reach target');
assert(abs(h.sl.ax.x - target) <= h.mcfg.posTolerance);
assert(h.xMax <= target + 0.05 && h.vMax <= vLimit + 0.01);
assert(h.vMax >= vLimit - 0.05, 'move did not reach the velocity limit');
fprintf('ok  position move (peak %.2f mm/s, overshoot %.3f mm)\n', h.vMax, h.xMax - target);

% Position -> jog: the slave changes mode from hold, without a brake cycle.
brake0 = h.brakeEngagements;
h.in.jogVelocity = 2;
h = command(h, 3);
x0 = h.sl.ax.x;
h = runUntil(h, 4, @(h) h.out.statusCode == 4 && abs(h.sl.ax.v - h.in.jogVelocity) < 0.05);
h = runFor(h, 0.5);
assert(h.out.statusCode == 4 && h.sl.ax.x > x0 + 1);
assert(abs(h.sl.ax.v - h.in.jogVelocity) < 0.05, 'jog speed %.3f mm/s, requested %.3f', ...
    h.sl.ax.v, h.in.jogVelocity);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_VELOCITY_CTRL);
assert(h.brakeEngagements == brake0);
h = stopToIdle(h);
assert(h.out.state == 1 && h.out.motionEnable == 0 && h.out.reference == 0);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE && h.sl.ax.v == 0);
fprintf('ok  position -> jog -> stop\n');

% Jog -> pressure: the slave closes the loop on its transducer. The axis
% is placed in the chamber below the target so the loop has a short
% compression to make.
h.in.jogVelocity = 1;
h = command(h, 3);
h = runUntil(h, 4, @(h) h.out.statusCode == 4);
h.sl.ax.x = chamberPosition(h.sl.sim, 0.5);
h.in.targetPressure = 2;
h = command(h, 4);
h = runUntil(h, 10, @(h) h.out.statusCode == 5);
assert(h.out.statusCode == 5 && h.sl.G.G_sysCurrentState == E.E_SystemState.ST_PRESSURE_CTRL);
h = runUntil(h, 30, @(h) h.y.performanceStatus == 1);
assert(h.y.performanceStatus == 1 && abs(h.sl.ax.P - 2) <= 0.1, 'pressure %.3f MPa', h.sl.ax.P);
h = stopToIdle(h);
assert(h.faultSeen == 0 && h.out.state == 1 && h.sl.ax.v == 0);
fprintf('ok  jog -> pressure (%.2f MPa)\n', h.sl.ax.P);

% Homing re-establishes the zero after the frame has been lost. The axis
% is placed out of the chamber, close to the negative overtravel switch.
h.sl.ax.x = 10;
h.sl.ax.offset = 12.3;
t0 = h.t;
h = command(h, 1);
h = runUntil(h, 60, @(h) h.sl.G.G_flagHomingComplete && h.out.state == 1);
assert(h.sl.G.G_flagHomingComplete && h.out.state == 1, 'homing did not complete');
assert(abs(h.sl.ax.x) < 0.05 && abs(h.sl.ax.offset) < 0.05);
assert(h.faultSeen == 0);
h = runUntil(h, 2, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE, 'slave did not leave homing');
fprintf('ok  homing (%.1f s)\n', h.t - t0);

% Go Home from position control: arrives, releases, and the next mode is
% accepted.
h.in.targetPosition = 5;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.out.statusCode == 3);
h = command(h, 6);
h = runUntil(h, 30, @(h) h.out.state == 1 && h.out.statusCode == 0);
assert(abs(h.sl.ax.x - h.sl.G.G_cfgGoHomePosition) < 0.1, 'Go Home did not arrive');
h = runUntil(h, 2, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE, 'slave did not leave Go Home');
h.in.targetPosition = 3;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.out.statusCode == 3);
assert(h.out.statusCode == 3, 'position move after Go Home did not reach target');
h = stopToIdle(h);
fprintf('ok  go home, then position move\n');

% Stop while the slave is still starting the mode (drive reset, enable,
% brake release): the slave must not run the mode with MotionEnable low,
% and must take the next request.
h = command(h, 1);
h = runUntil(h, 3, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_BRAKE_RELEASE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_BRAKE_RELEASE, 'slave did not start the mode');
x0 = h.sl.ax.x;
h = stopToIdle(h);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE, 'slave did not return to idle');
h = runFor(h, 0.5);
assert(abs(h.sl.ax.x - x0) < 0.05, 'slave moved after Stop during start-up');
h.in.targetPosition = x0 + 3;
h = command(h, 2);
h = runUntil(h, 20, @(h) h.out.statusCode == 3);
assert(h.out.statusCode == 3, 'next mode after Stop during start-up was not accepted');
fprintf('ok  stop during mode start-up\n');

% Drive fault during a position move: latched, mirrored, no automatic reset.
h.in.targetPosition = h.sl.ax.x + 10;
h = command(h, 2);
h = runUntil(h, 3, @(h) h.sl.ax.v > 1);
assert(h.sl.ax.v > 1, 'position move did not start');
h.sl.hook.driveFault = true;
h = runFor(h, 0.5);
assert(h.out.statusCode == 10 && h.out.faultCode == 2 && h.out.motionEnable == 0);
assert(h.out.modeCode == 2 && h.out.faultReset == 0);
h.sl.hook.driveFault = false;
h = runFor(h, 2);
assert(h.out.statusCode == 10 && h.y.faultActive == 1);
h = command(h, 5);
h = runFor(h, 1.5);
assert(h.y.faultActive == 0 && h.out.state == 1 && h.out.faultReset == 0);
h = runFor(h, 1);
assert(h.out.motionEnable == 0 && h.sl.ax.v == 0);
fprintf('ok  drive fault, mirror and operator reset\n');

% The next operation starts from wherever the reset left the slave.
target = h.sl.ax.x + 3;
h.in.targetPosition = target;
h = command(h, 2);
h = runUntil(h, 30, @(h) h.out.statusCode == 3);
assert(h.out.statusCode == 3 && abs(h.sl.ax.x - target) <= h.mcfg.posTolerance);
h = stopToIdle(h);
fprintf('ok  position move after fault reset\n');

% E-stop during jog; release does not restart motion.
h.faultSeen = 0;
h.in.jogVelocity = 2;
h = command(h, 3);
h = runUntil(h, 4, @(h) h.out.statusCode == 4 && h.sl.ax.v > 1.5);
assert(h.out.statusCode == 4 && h.sl.ax.v > 1.5, 'jog did not start');
h.in.estop = 1;
h = runFor(h, 0.05);
assert(h.out.motionEnable == 0 && h.out.reference == 0);
h = runUntil(h, 3, @(h) h.out.statusCode == 20 && h.sl.ax.v == 0);
assert(h.out.statusCode == 20 && h.sl.ax.v == 0);
h.in.estop = 0;
h = runFor(h, 1);
assert(h.out.state == 1 && h.out.motionEnable == 0 && h.faultSeen == 0);
fprintf('ok  e-stop\n');

% Master soft limit stops a jog before the slave's own limit faults.
h.sl.ax.x = h.mcfg.posMax - 3;      % place the axis near the limit
h.faultSeen = 0;
h = runFor(h, 0.5);
h.in.jogVelocity = 3;
h = command(h, 3);
h = runFor(h, 3);
assert(h.faultSeen == 0 && h.sl.ax.x < h.sl.G.G_cfgPosSoftLimitMax);
assert(h.sl.ax.x >= h.mcfg.posMax - 0.5);
fprintf('ok  soft limit\n');

fprintf('all passed (slave handshake timeout %g s)\n', handshakeTimeout);
end


function h = harnessInit(handshakeTimeout)
h.sl = slaveInit();
h.sl.G.G_cfgHandshakeTimeout = handshakeTimeout;

% The master's soft limit sits inside the slave's.
h.mcfg = masterConfig();
h.mcfg.posMax = h.sl.G.G_cfgPosSoftLimitMax - 5;

h.in = struct('reset', 1, 'confCode', 0, 'faultActive', 0, 'inMotion', 0, ...
    'homingComplete', 0, 'brakeDisengage', 0, 'position', 0, 'estop', 0, ...
    'op', 0, 'cmdSeq', 0, 'targetPosition', 100, 'jogVelocity', 0, ...
    'targetPressure', 0);
h.u = struct('heartbeat', 0, 'modeCode', 0, 'motionEnable', 0, ...
    'faultReset', 0, 'reference', 0);
[h.sl, h.y] = slaveStep(h.sl, h.u);
h.ySnap = h.y;
h.out = [];
h.heartbeat = 0;
h.tick = 0;
h.t = 0;
h.vMax = 0;
h.xMax = -inf;
h.faultSeen = 0;
h.brakeEngagements = 0;
h.prevBrake = 0;
end


function h = command(h, op)
h.in.op = op;
h.in.cmdSeq = h.in.cmdSeq + 1;
end


function h = runFor(h, T)
h = runUntil(h, T, []);
end


function h = stopToIdle(h)
% Stop (mode 0, no CommandSeq needed) and run until the master is idle
% with MotionEnable low and the slave back in idle at rest.
h.in.op = 0;
h = runUntil(h, 5, @(h) h.out.state == 1 && h.out.motionEnable == 0 && ...
    h.sl.G.G_sysCurrentState == h.sl.E.E_SystemState.ST_IDLE && h.sl.ax.v == 0);
end


function h = runUntil(h, T, done)
for k = 1:round(T * 1000)
    h.tick = h.tick + 1;
    h.t = h.tick / 1000;

    if mod(h.tick, round(h.mcfg.dt * 1000)) == 0
        y = h.ySnap;
        if y.faultActive ~= 0
            h.in.confCode   = y.faultCode;
        else
            h.in.confCode   = y.confCode;
        end
        h.in.faultActive    = y.faultActive;
        h.in.inMotion       = y.inMotion;
        h.in.homingComplete = y.homingComplete;
        h.in.brakeDisengage = 0;
        h.in.position       = y.actualPosition;
        h.out = masterStateMachine(h.in, h.mcfg, h.mcfg.dt);
        h.in.reset = 0;
        h.heartbeat = mod(h.heartbeat + 1, 65536);
    end

    % Modbus exchange: the master's registers reach the slave and the
    % slave's registers are sampled for the master's next steps.
    if mod(h.tick, round(h.mcfg.exchangePeriod * 1000)) == 0 && ~isempty(h.out)
        h.u.heartbeat    = h.heartbeat;
        h.u.modeCode     = h.out.modeCode;
        h.u.motionEnable = h.out.motionEnable;
        h.u.faultReset   = h.out.faultReset;
        h.u.reference    = h.out.reference;
        h.ySnap = h.y;
    end

    if mod(h.tick, round(1000 * h.sl.G.G_cfgScanTime)) == 0
        [h.sl, h.y] = slaveStep(h.sl, h.u);
        h.vMax = max(h.vMax, abs(h.sl.ax.v));
        h.xMax = max(h.xMax, h.sl.ax.x);
        h.faultSeen = max(h.faultSeen, h.y.faultActive);
        if h.prevBrake == 1 && h.y.brakeDisengage == 0
            h.brakeEngagements = h.brakeEngagements + 1;
        end
        h.prevBrake = h.y.brakeDisengage;
    end

    if ~isempty(done) && ~isempty(h.out) && done(h)
        return;
    end
end
end
