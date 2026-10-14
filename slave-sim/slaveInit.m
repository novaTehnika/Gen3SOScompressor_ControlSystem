function sl = slaveInit()
%SLAVEINIT Simulated slave at power-up.
%   sl.s   PRG_Main variables and function block instances
%   sl.G   global variables (G_* in the ST)
%   sl.IO  controller I/O image (MO1_* in PRG_Input / PRG_Output)
%   sl.ax  simulated axis (axisModel)
%   Test hooks, read every scan by slaveStep:
%   sl.hook.driveFault  sets G_sysDriveFault
%   sl.hook.blocked     axis cannot move (mechanical stall)
%   sl.hook.torque      reported actual torque (%)
%   sl.hook.outflow     liquid leaving the chamber (mm^3/s)
%   sl.hook.pressureVolt  analog input override (V); NaN = transducer model

sl.E = ST_enums();
[sl.s, sl.G, sl.IO] = ST_init(sl.E);
sl.sim = slaveSimConfig();

ax.x = sl.sim.initialPos;
ax.v = 0;
ax.offset = sl.sim.bootFrameError;
ax.tPower = 0;
ax.prevResetExec = false;
ax.prevSetPosExec = false;
ax.prevMoveExec = false;
ax.prevStopExec = false;
ax.moveActive = false;
ax.moveTarget = 0;
ax.moveVel = 0;
ax.moveAcc = 0;
ax.moveDec = 0;
ax.moveDone = false;
ax.moveAborted = false;
ax.jogBusy = false;
ax.P = 0;           % chamber pressure (MPa gauge)
ax.vOut = 0;        % liquid removed through hook.outflow (mm^3)
sl.ax = ax;

sl.hook.driveFault = false;
sl.hook.blocked = false;
sl.hook.torque = 0;
sl.hook.outflow = 0;
sl.hook.pressureVolt = NaN;
sl.t = 0;
end
