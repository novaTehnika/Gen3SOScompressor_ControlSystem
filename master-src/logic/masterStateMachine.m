function out = masterStateMachine(in, cfg, dt)
%MASTERSTATEMACHINE Master-side handshake, motion and fault sequencing.
%   out = masterStateMachine(in, cfg, dt) advances the state machine by one
%   step of dt seconds. cfg comes from masterConfig.
%
%   in fields
%     reset             1 reinitialises all internal state
%     confCode          slave DO0-2 as 0..7: mode confirmation, or the fault
%                       code while faultActive is high
%     faultActive       slave status bits (0/1)
%     inMotion
%     homingComplete
%     brakeDisengage
%     position          position feedback (mm)
%     estop             app E-stop (1 = stop)
%     op                operation requested by the app (OP_* below)
%     cmdSeq            app command counter; a change marks op as a new
%                       command. op = OP_STOP acts without a change.
%     targetPosition    mm, read continuously during OP_POSITION
%     jogVelocity       signed mm/s, read continuously during OP_JOG
%     pressureVelocity  signed mm/s from the pressure controller, read
%                       continuously during OP_PRESSURE
%
%   out fields
%     modeCode          0..7 for DO0-2 (mode command, or mirrored fault code)
%     motionEnable      0/1
%     faultReset        0/1
%     refVoltage        analog reference (V)
%     statusCode        STATUS_* below
%     faultCode         last fault code read from the slave
%     state             ST_* below
%
%   Mode bits only change while motionEnable is low. A mode is commanded
%   with motionEnable low, the slave confirms it, and only then is
%   motionEnable raised.

persistent s

ST_INIT        = 0;
ST_IDLE        = 1;
ST_REQUEST     = 2;
ST_ENABLE      = 3;
ST_ACTIVE      = 4;
ST_STOP        = 5;
ST_FAULT_LATCH = 6;
ST_FAULT_RESET = 7;
ST_ESTOP       = 8;

OP_STOP     = 0;
OP_HOME     = 1;
OP_POSITION = 2;
OP_JOG      = 3;
OP_PRESSURE = 4;
OP_RESET    = 5;
OP_GO_HOME  = 6;

STATUS_IDLE            = 0;
STATUS_HOMING          = 1;
STATUS_POS_MOVING      = 2;
STATUS_POS_AT_TARGET   = 3;
STATUS_JOG             = 4;
STATUS_PRESSURE        = 5;
STATUS_STOPPING        = 6;
STATUS_STARTING        = 7;
STATUS_GOING_HOME      = 8;
STATUS_RESETTING       = 9;
STATUS_FAULT_BASE      = 10;    % + fault code
STATUS_ESTOP           = 20;
STATUS_NO_CONFIRM      = 30;
STATUS_RESET_FAILED    = 32;
STATUS_STOP_TIMEOUT    = 33;

if isempty(s) || in.reset ~= 0
    % t is the time in the current state, tDwell a per-state condition
    % timer and note a status that stays on display while idle.
    s = struct('state', ST_INIT, 't', 0, 'tDwell', 0, ...
        'activeOp', OP_STOP, 'pendingOp', OP_STOP, 'lastSeq', in.cmdSeq, ...
        'note', STATUS_IDLE, 'modeCode', 0, 'motionEnable', 0, ...
        'faultReset', 0, 'refVoltage', 0, 'statusCode', STATUS_IDLE, ...
        'faultCode', 0);
end

newCmd = in.cmdSeq ~= s.lastSeq;
s.lastSeq = in.cmdSeq;

isMotionOp = in.op == OP_HOME || in.op == OP_GO_HOME || ...
             in.op == OP_POSITION || in.op == OP_JOG || in.op == OP_PRESSURE;

% Overrides from any state.
if in.estop ~= 0
    if s.state ~= ST_ESTOP
        s.state = ST_ESTOP;
        s.t = 0;
    end
elseif in.faultActive ~= 0 && s.state ~= ST_FAULT_LATCH && ...
        s.state ~= ST_FAULT_RESET && s.state ~= ST_ESTOP
    s.state = ST_FAULT_LATCH;
    s.t = 0;
    s.tDwell = 0;
    s.faultCode = 0;
end

next = s.state;

switch s.state
    case ST_INIT
        s.modeCode = 0;
        s.motionEnable = 0;
        s.faultReset = 0;
        s.refVoltage = 0;
        s.statusCode = STATUS_IDLE;
        if s.t >= cfg.tInit
            next = ST_IDLE;
        end

    case ST_IDLE
        s.modeCode = 0;
        s.motionEnable = 0;
        s.faultReset = 0;
        s.refVoltage = 0;
        s.activeOp = OP_STOP;
        if newCmd
            s.note = STATUS_IDLE;
            s.pendingOp = OP_STOP;
            if isMotionOp
                s.pendingOp = in.op;
            end
        end
        if in.op == OP_STOP
            s.pendingOp = OP_STOP;
        end
        % Mode 000 is held long enough for the slave to register it, so
        % the confirmation of the next request is a new one.
        if s.pendingOp ~= OP_STOP && s.t >= cfg.tModeClear
            s.activeOp = s.pendingOp;
            s.pendingOp = OP_STOP;
            next = ST_REQUEST;
        end
        s.statusCode = s.note;

    case ST_REQUEST
        s.modeCode = slaveModeFor(s.activeOp);
        s.motionEnable = 0;
        s.refVoltage = entryReference(s.activeOp, in.position, cfg);
        s.statusCode = STATUS_STARTING;
        if s.t == 0
            s.tDwell = 0;
        end
        if in.confCode == s.modeCode
            s.tDwell = s.tDwell + dt;
        else
            s.tDwell = 0;
        end
        if in.op == OP_STOP
            next = ST_IDLE;
        elseif s.tDwell >= cfg.tConfirmStable
            next = ST_ENABLE;
        elseif s.t >= cfg.tHandshake
            s.note = STATUS_NO_CONFIRM;
            next = ST_IDLE;
        end

    case ST_ENABLE
        s.motionEnable = 1;
        s.refVoltage = entryReference(s.activeOp, in.position, cfg);
        s.statusCode = STATUS_STARTING;
        if in.op == OP_STOP
            next = ST_STOP;
        elseif s.t >= cfg.tEnableMin && ...
                (in.brakeDisengage ~= 0 || s.t >= cfg.tEnableMax)
            next = ST_ACTIVE;
        end

    case ST_ACTIVE
        s.motionEnable = 1;
        if s.t == 0
            s.tDwell = 0;
        end
        done = false;

        if s.activeOp == OP_HOME
            s.refVoltage = 0;
            s.statusCode = STATUS_HOMING;
            done = in.homingComplete ~= 0;

        elseif s.activeOp == OP_GO_HOME
            s.refVoltage = 0;
            s.statusCode = STATUS_GOING_HOME;
            % The slave has no arrival output for Go Home; the axis coming
            % to rest ends it.
            if in.inMotion == 0
                s.tDwell = s.tDwell + dt;
            else
                s.tDwell = 0;
            end
            done = s.tDwell >= cfg.tGoHomeDone;

        elseif s.activeOp == OP_POSITION
            target = min(max(in.targetPosition, cfg.posMin), cfg.posMax);
            s.refVoltage = x2V(target, cfg);
            if abs(in.position - target) <= cfg.posTolerance
                s.tDwell = min(s.tDwell + dt, cfg.tArrive);
            else
                s.tDwell = 0;
            end
            if s.tDwell >= cfg.tArrive
                s.statusCode = STATUS_POS_AT_TARGET;
            else
                s.statusCode = STATUS_POS_MOVING;
            end

        elseif s.activeOp == OP_JOG
            v = limitVelocity(in.jogVelocity, cfg.jogVelMax, in.position, cfg);
            s.refVoltage = v / cfg.velPerVolt;
            s.statusCode = STATUS_JOG;

        else    % OP_PRESSURE
            v = limitVelocity(in.pressureVelocity, cfg.pressureVelMax, ...
                              in.position, cfg);
            s.refVoltage = v / cfg.velPerVolt;
            s.statusCode = STATUS_PRESSURE;
        end

        if in.op == OP_STOP || done
            s.pendingOp = OP_STOP;
            next = ST_STOP;
        elseif newCmd && isMotionOp && in.op ~= s.activeOp
            s.pendingOp = in.op;
            next = ST_STOP;
        end

    case ST_STOP
        s.motionEnable = 0;
        if s.activeOp ~= OP_POSITION
            s.refVoltage = 0;
        end
        s.statusCode = STATUS_STOPPING;
        if in.op == OP_STOP
            s.pendingOp = OP_STOP;
        elseif newCmd && isMotionOp
            s.pendingOp = in.op;
        end
        % With motionEnable low and a mode still commanded, the slave
        % re-confirms that mode and restarts its handshake window, so a
        % confirmation seen later for the same mode may be about to expire.
        % The old mode is therefore held only when the pending operation
        % uses a different mode, whose confirmation is necessarily new.
        % Otherwise the bits return to 000 and any pending operation is
        % started from ST_IDLE.
        holdMode = s.pendingOp ~= OP_STOP && ...
                   slaveModeFor(s.pendingOp) ~= slaveModeFor(s.activeOp);
        if s.t >= cfg.tStopMin && ~holdMode
            s.modeCode = 0;
        end
        halted = s.t >= cfg.tStopMin && in.inMotion == 0;
        if halted || s.t >= cfg.tStopMax
            if ~halted
                s.note = STATUS_STOP_TIMEOUT;
                s.pendingOp = OP_STOP;
            end
            if s.pendingOp ~= OP_STOP && holdMode && s.modeCode ~= 0
                s.activeOp = s.pendingOp;
                s.pendingOp = OP_STOP;
                next = ST_REQUEST;
            else
                next = ST_IDLE;
            end
        end

    case ST_FAULT_LATCH
        s.motionEnable = 0;
        s.faultReset = 0;
        s.refVoltage = 0;
        s.activeOp = OP_STOP;
        s.pendingOp = OP_STOP;
        if s.t >= cfg.tFaultSettle
            % Mirror the slave's fault code onto the mode bits.
            if in.confCode ~= s.modeCode || s.faultCode ~= in.confCode
                s.tDwell = 0;
            else
                s.tDwell = s.tDwell + dt;
            end
            s.faultCode = in.confCode;
            s.modeCode = in.confCode;
        end
        s.statusCode = STATUS_FAULT_BASE + s.faultCode;
        if in.faultActive == 0
            s.note = STATUS_IDLE;
            next = ST_IDLE;
        elseif newCmd && in.op == OP_RESET && s.tDwell >= cfg.tMirrorStable
            next = ST_FAULT_RESET;
        end

    case ST_FAULT_RESET
        s.motionEnable = 0;
        s.faultReset = 1;
        s.refVoltage = 0;
        s.statusCode = STATUS_RESETTING;
        if in.faultActive == 0
            s.faultReset = 0;
            s.note = STATUS_IDLE;
            next = ST_IDLE;
        elseif s.t >= cfg.tResetMax
            s.faultReset = 0;
            s.note = STATUS_RESET_FAILED;
            next = ST_FAULT_LATCH;
        end

    otherwise   % ST_ESTOP
        s.motionEnable = 0;
        s.faultReset = 0;
        s.refVoltage = 0;
        s.activeOp = OP_STOP;
        s.pendingOp = OP_STOP;
        s.statusCode = STATUS_ESTOP;
        if s.t >= cfg.tStopMin
            s.modeCode = 0;
        end
        if in.estop == 0
            s.note = STATUS_IDLE;
            if in.faultActive ~= 0
                next = ST_FAULT_LATCH;
            else
                next = ST_IDLE;
            end
        end
end

if next ~= s.state
    s.state = next;
    s.t = 0;
    if next == ST_FAULT_LATCH
        s.tDwell = 0;
    end
else
    s.t = s.t + dt;
end

out.modeCode     = s.modeCode;
out.motionEnable = s.motionEnable;
out.faultReset   = s.faultReset;
out.refVoltage   = s.refVoltage;
out.statusCode   = s.statusCode;
out.faultCode    = s.faultCode;
out.state        = s.state;
end


function mode = slaveModeFor(op)
% Slave mode code commanded for each app operation.
if op == 1          % OP_HOME: Home to Negative Overtravel (re-establishes zero)
    mode = 6;
elseif op == 6      % OP_GO_HOME: move to the home position
    mode = 5;
elseif op == 2      % OP_POSITION
    mode = 2;
else                % OP_JOG, OP_PRESSURE: Velocity
    mode = 3;
end
end


function V = entryReference(op, position, cfg)
% Reference held while a mode is being entered. Position mode is entered
% with the reference equal to the actual position; velocity modes with zero.
if op == 2
    V = x2V(position, cfg);
else
    V = 0;
end
end


function v = limitVelocity(v, vMax, position, cfg)
% Clamp a velocity command and block motion further past a soft limit.
v = min(max(v, -vMax), vMax);
if (v > 0 && position >= cfg.posMax) || (v < 0 && position <= cfg.posMin)
    v = 0;
end
end
