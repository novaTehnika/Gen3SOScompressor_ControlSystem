function P = chamberPressure(sim, x, vOut)
%CHAMBERPRESSURE Gauge pressure (MPa) of the simulated chamber with the
%   piston at x (mm) and vOut (mm^3) of liquid removed. The chamber is an
%   isothermal gas spring (slaveSimConfig): zero until the piston passes
%   sim.chamberContactPos, then the gas starting at sim.chamberGasVolume at
%   atmosphere is compressed, down to sim.chamberMinVolume.
%   chamberPosition is the inverse.

if x > sim.chamberContactPos
    vGas = sim.chamberGasVolume - sim.pistonArea * (x - sim.chamberContactPos) + vOut;
    vGas = max(vGas, sim.chamberMinVolume);
    P = sim.atmMPa * (sim.chamberGasVolume / vGas - 1);
else
    P = 0;
end
end
