function sl = slaveInit()
%SLAVEINIT Simulated slave at power-up: the compiled slave (SlaveSim, built
%   by buildSlaveSim from slave-src and slave-sim/c). Its fields read and
%   assign like a struct's:
%   sl.G   global variables (G_* in the ST)
%   sl.IO  controller I/O image (MO1_* in PRG_Input / PRG_Output)
%   sl.s   PRG_Main variables and function block instances
%   sl.ax  simulated axis
%   sl.plant  the design model's plant (state, and parameters in .par)
%   sl.sim axis geometry and transducer (slaveSimConfig)
%   sl.E   enumerations
%   Test hooks, read every scan by slaveStep:
%   sl.hook.driveFault  sets G_sysDriveFault
%   sl.hook.blocked     axis cannot move (mechanical stall)
%   sl.hook.torque      reported actual torque (%)
%   sl.hook.outflow     liquid leaving the chamber (mm^3/s)
%   sl.hook.valve       outlet valve area fraction (the orifice bank)
%   sl.hook.pressureVolt  analog input override (V); NaN = transducer model

sl = SlaveSim();
end
