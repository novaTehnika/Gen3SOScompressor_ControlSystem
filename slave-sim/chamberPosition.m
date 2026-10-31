function x = chamberPosition(sim, P)
%CHAMBERPOSITION Axis position x (mm) at which the simulated chamber holds
%   gauge pressure P (MPa) with no liquid removed: the inverse of the gas
%   spring in axisModel. Tests use it to start a pressure scenario close
%   to its target.

vGas = sim.chamberGasVolume * sim.atmMPa / (P + sim.atmMPa);
x = sim.chamberContactPos + (sim.chamberGasVolume - vGas) / sim.pistonArea;
end
