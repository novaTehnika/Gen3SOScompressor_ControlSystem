function traceRing = traceStep(mbRead, mbWrite, doSafety, statusCode, faultCode, ...
    state, linkOk, op, cmdSeq, targetPosition, targetPressure, estop, mixer)
%TRACESTEP Entry point for the Simulink shell's TraceStep MATLAB Function
%   block: keeps a ring of one record per Modbus exchange and returns the
%   whole ring. Called once per model step (cfg.dt) next to masterStep,
%   which it does not touch; the app reads the ring while recording
%   (traceStart / traceDrain) and writes it to a CSV.
%
%   mbRead        input registers IR1..IR20 as read by the Modbus Client
%                 Read block (modbusMap, raw 16-bit values as doubles)
%   mbWrite       holding registers HR1..HR12 as masterStep returned them
%                 and the Modbus Client Write block sends them
%   doSafety, statusCode, faultCode, state, linkOk
%                 masterStep outputs of the same step
%   op, cmdSeq, targetPosition, targetPressure, estop, mixer
%                 values of the Constant blocks the app writes;
%                 targetPressure in atm absolute
%
%   traceRing     traceLayout().depth x traceLayout().width matrix, one
%                 record per row. Record n (n = 1, 2, ... since the first
%                 call) is in row mod(n-1, depth) + 1. Column 1 holds n and
%                 the guard column repeats it, so a row read while it is
%                 being written is recognised by guard ~= n. Rows not yet
%                 written are zero. The columns are named in traceLayout.
%
%   A record is taken in every step whose count is a multiple of
%   cfg.exchangePeriod/cfg.dt, counted from the first call. The read
%   vector changes only on those steps, so each exchange is recorded once,
%   and recording goes on while the link is lost. Its time t_s is the model
%   time of the step (step count times cfg.dt). The state is initialised on
%   the first call; clear traceStep resets it.

persistent ring stepCount seq

L = traceLayout();
cfg = masterConfig();
map = modbusMap();

if isempty(ring)
    ring = zeros(L.depth, L.width);
    stepCount = 0;
    seq = 0;
end

stepsPerExchange = round(cfg.exchangePeriod / cfg.dt);
if mod(stepCount, stepsPerExchange) == 0
    seq = seq + 1;
    r = double(mbRead(:)');
    w = double(mbWrite(:)');

    rec = zeros(1, L.width);
    rec(L.col.seq)                  = seq;
    rec(L.col.t_s)                  = stepCount * cfg.dt;     % [steps -> s]
    rec(L.col.heartbeat)            = w(map.hr.heartbeat);
    rec(L.col.heartbeatEcho)        = r(map.ir.heartbeatEcho);
    rec(L.col.slaveCounter)         = r(map.ir.slaveCounter);
    rec(L.col.confCode)             = r(map.ir.confCode);
    rec(L.col.slaveFaultCode)       = r(map.ir.faultCode);
    rec(L.col.faultActive)          = double(r(map.ir.faultActive) ~= 0);
    rec(L.col.inMotion)             = double(r(map.ir.inMotion) ~= 0);
    rec(L.col.performanceStatus)    = double(r(map.ir.performanceStatus) ~= 0);
    rec(L.col.homingComplete)       = double(r(map.ir.homingComplete) ~= 0);
    rec(L.col.position_mm)          = modbusUnpack(r(map.ir.position), r(map.ir.position + 1));
    rec(L.col.velocity_mm_s)        = modbusUnpack(r(map.ir.velocity), r(map.ir.velocity + 1));
    rec(L.col.torque_pct)           = modbusUnpack(r(map.ir.torque), r(map.ir.torque + 1));
    rec(L.col.commandedPosition_mm) = modbusUnpack(r(map.ir.commandedPosition), r(map.ir.commandedPosition + 1));
    rec(L.col.pressureGauge_MPa)    = modbusUnpack(r(map.ir.pressure), r(map.ir.pressure + 1));
    rec(L.col.modeCode)             = w(map.hr.modeCode);
    rec(L.col.motionEnable)         = w(map.hr.motionEnable);
    rec(L.col.faultReset)           = w(map.hr.faultReset);
    rec(L.col.reference)            = modbusUnpack(w(map.hr.reference), w(map.hr.reference + 1));
    rec(L.col.positionSpeed_mm_s)   = modbusUnpack(w(map.hr.positionSpeed), w(map.hr.positionSpeed + 1));
    rec(L.col.salineVolume_mL)      = modbusUnpack(w(map.hr.salineVolume), w(map.hr.salineVolume + 1));
    rec(L.col.oxygenRatio)          = modbusUnpack(w(map.hr.oxygenRatio), w(map.hr.oxygenRatio + 1));
    rec(L.col.statusCode)           = statusCode;
    rec(L.col.faultCode)            = faultCode;
    rec(L.col.state)                = state;
    rec(L.col.linkOk)               = linkOk;
    rec(L.col.doSafety)             = doSafety;
    rec(L.col.op)                   = op;
    rec(L.col.cmdSeq)               = cmdSeq;
    rec(L.col.targetPosition_mm)    = targetPosition;
    rec(L.col.targetPressure_atm)   = targetPressure;
    rec(L.col.estop)                = estop;
    rec(L.col.mixer)                = mixer;
    rec(L.guard)                    = seq;

    ring(mod(seq - 1, L.depth) + 1, :) = rec;
end

stepCount = stepCount + 1;
traceRing = ring;
end
