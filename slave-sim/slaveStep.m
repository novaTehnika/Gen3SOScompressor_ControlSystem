function [sl, y] = slaveStep(sl, u)
%SLAVESTEP One controller scan (G_cfgScanTime) of the simulated slave.
%   u  the Modbus holding registers as the master wrote them, in engineering
%      units: heartbeat, modeCode, motionEnable, faultReset (0/1) and
%      reference (mm, mm/s or % by mode)
%   y  the Modbus input registers in engineering units: heartbeatEcho,
%      slaveCounter, confCode, faultCode, faultActive, inMotion,
%      performanceStatus, homingComplete, actualPosition, actualVelocity,
%      actualTorque, commandedPosition, actualPressure, plus brakeDisengage
%      from DO7
%
%   Scan order: inputs -> PRG_Input -> PRG_Main -> PRG_Output -> axisModel.
%   The axis model stands in for the LD POU that owns the motion function
%   blocks, so PRG_Main sees motion status one scan late. Packing the
%   registers into 16-bit words is the master-side test's job.

if isobject(sl)
    y = step(sl, u);        % the compiled slave (SlaveSim)
    return
end

dt = sl.G.G_cfgScanTime;

% Modbus holding registers (group FC#06,16) and controller input pins.
% Overtravel switches are normally closed: TRUE = not triggered.
sl.G.G_mbHeartbeat    = u.heartbeat;
sl.G.G_mbModeCode     = u.modeCode;
sl.G.G_mbMotionEnable = u.motionEnable;
sl.G.G_mbFaultReset   = u.faultReset;
sl.G.G_mbReference    = u.reference;
sl.IO.MO1_DI_06 = sl.ax.x > sl.sim.switchNegPos;
sl.IO.MO1_DI_07 = sl.ax.x < sl.sim.switchPosPos;
if isnan(sl.hook.pressureVolt)
    % Transducer: chamber pressure as loop current across the shunt.
    mA = sl.sim.sensorZeroMa + sl.sim.sensorSpanMa * sl.ax.P / sl.sim.sensorRangeMPa;
    sl.IO.MO1_AI_01 = mA * sl.sim.sensorShuntOhm / 1000;
else
    sl.IO.MO1_AI_01 = sl.hook.pressureVolt;
end

sl.G = PRG_Input(sl.G, sl.IO);
[sl.s, sl.G] = PRG_Main(sl.s, sl.G, sl.E, dt);
[sl.G, sl.IO] = PRG_Output(sl.G, sl.IO);
[sl.ax, sl.G] = axisModel(sl.ax, sl.G, sl.E, sl.sim, sl.hook, dt);
sl.t = sl.t + dt;

% Modbus input registers (group FC#04) and the brake output pin.
y.heartbeatEcho     = sl.G.G_mbHeartbeatEcho;
y.slaveCounter      = sl.G.G_mbSlaveCounter;
y.confCode          = sl.G.G_mbConfCode;
y.faultCode         = sl.G.G_mbFaultCode;
y.faultActive       = double(sl.G.G_mbFaultActive);
y.inMotion          = double(sl.G.G_mbInMotion);
y.performanceStatus = double(sl.G.G_mbPerformanceStatus);
y.homingComplete    = double(sl.G.G_mbHomingComplete);
y.actualPosition    = sl.G.G_mbActualPosition;
y.actualVelocity    = sl.G.G_mbActualVelocity;
y.actualTorque      = sl.G.G_mbActualTorque;
y.commandedPosition = sl.G.G_mbCommandedPosition;
y.actualPressure    = sl.G.G_mbActualPressure;
y.brakeDisengage    = double(sl.IO.MO1_DO_07);
end
