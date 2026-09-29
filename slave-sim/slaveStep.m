function [sl, y] = slaveStep(sl, u)
%SLAVESTEP One controller scan (G_cfgScanTime) of the simulated slave.
%   u  signals from the master: modeBit0, modeBit1, modeBit2, motionEnable,
%      faultReset (0/1) and refVoltage (V)
%   y  signals to the master: confBit0..2, performanceStatus, faultActive,
%      homingComplete, inMotion, brakeDisengage (0/1) and positionVolt (V)
%
%   Scan order: inputs -> PRG_Input -> PRG_Main -> PRG_Output -> axisModel.
%   The axis model stands in for the LD POU that owns the motion function
%   blocks, so PRG_Main sees motion status one scan late.

dt = sl.G.G_cfgScanTime;

% Controller input pins (PRG_Input.st). Overtravel switches are normally
% closed: TRUE = not triggered.
sl.IO.MO1_DI_00 = u.modeBit0 ~= 0;
sl.IO.MO1_DI_01 = u.modeBit1 ~= 0;
sl.IO.MO1_DI_02 = u.modeBit2 ~= 0;
sl.IO.MO1_DI_03 = u.motionEnable ~= 0;
sl.IO.MO1_DI_04 = u.faultReset ~= 0;
sl.IO.MO1_DI_06 = sl.ax.x > sl.sim.switchNegPos;
sl.IO.MO1_DI_07 = sl.ax.x < sl.sim.switchPosPos;
sl.IO.MO1_AI_01 = u.refVoltage;

sl.G = PRG_Input(sl.G, sl.IO);
[sl.s, sl.G] = PRG_Main(sl.s, sl.G, sl.E, dt);
sl.IO = PRG_Output(sl.G, sl.IO);
[sl.ax, sl.G] = axisModel(sl.ax, sl.G, sl.E, sl.sim, sl.hook, dt);
sl.t = sl.t + dt;

% Controller output pins (PRG_Output.st).
y.confBit0          = double(sl.IO.MO1_DO_00);
y.confBit1          = double(sl.IO.MO1_DO_01);
y.confBit2          = double(sl.IO.MO1_DO_02);
y.performanceStatus = double(sl.IO.MO1_DO_03);
y.faultActive       = double(sl.IO.MO1_DO_04);
y.homingComplete    = double(sl.IO.MO1_DO_05);
y.inMotion          = double(sl.IO.MO1_DO_06);
y.brakeDisengage    = double(sl.IO.MO1_DO_07);
y.positionVolt      = sl.IO.MO1_AO_01;
end
