function [sl, y] = slaveStep(sl, u)
%SLAVESTEP One controller scan (G_cfgScanTime) of the simulated slave.
%   u  the Modbus holding registers as the master wrote them, in engineering
%      units: heartbeat, modeCode, motionEnable, faultReset (0/1) and
%      reference (mm, mm/s, % or MPa by mode) and positionSpeed (mm/s);
%      every field is required
%   y  the Modbus input registers in engineering units: heartbeatEcho,
%      slaveCounter, confCode, faultCode, faultActive, inMotion,
%      performanceStatus, homingComplete, pressureBlock, atEndOfTravel,
%      actualPosition, actualVelocity,
%      actualTorque, commandedPosition, actualPressure, plus brakeDisengage
%      from DO7
%
%   Scan order: inputs -> PRG_Input -> PRG_Main -> PRG_Output -> axis model
%   (slave-sim/c/slave_sim.c). The axis model stands in for the two LD
%   POUs: for PRG_LD, which runs last, so PRG_Main sees the motion blocks'
%   status one scan late, and for PRG_LD_Read, which runs first: the
%   feedback the model leaves is the plant's state at the start of the next
%   scan. sl is a handle; it is returned for the callers' [sl, y] = form.
%
%   This is the engineering-units view. For the registers as 16-bit words,
%   use writeHoldingRegisters(sl, start, words), y = scan(sl) and
%   readInputRegisters(sl, start, n): the simulated slave packs them by its
%   register map, the globals' AT addresses.

y = step(sl, u);
end
