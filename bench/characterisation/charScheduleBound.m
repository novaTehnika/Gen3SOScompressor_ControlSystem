function [kEst, kEstTarget, lag] = charScheduleBound(P, x, pRef, target, fill, g)
%CHARSCHEDULEBOUND The chamber stiffness bound of the slave's gain schedule
%   (slave-src/FB/FB_PressureSchedule.st, eq. (39) of
%   design/pressure-loop/README.md) evaluated off the machine, from the
%   slave's settings and a pressure and a position, so that a log's measured
%   stiffness can be compared with what the schedule would have used.
%   charScheduleCheck does the comparison; tests/slavePressureMode.m holds
%   this function equal to the compiled FB_PressureSchedule.
%
%   [kEst, kEstTarget, lag] = charScheduleBound(P, x, pRef, target, fill, g)
%
%   P, x, pRef, target   present pressure (MPa gauge), piston position
%                        (mm), highest pressure since the schedule was
%                        reset (MPa gauge) and target pressure (MPa gauge);
%                        arrays of one size, or scalars
%   fill                 struct: saline_mL (HR9-10, the saline volume of
%                        the fill), oxygenRatio (HR11-12) and hoseOpen
%                        (G_cfgPressureSchedHoseOpen)
%   g                    the slave's settings by their GVL names
%                        (benchGvl returns them): the
%                        G_cfgPressureSched* values, G_cfgScanTime,
%                        G_cfgAnalogFilterTimeConst and G_cfgAnalogMedianSize
%
%   kEst        bound at the present pressure, MPa/mm (the block's KEst)
%   kEstTarget  bound at the larger of the present and the target
%               pressure, MPa/mm (KEstTarget)
%   lag         s, the delay the stop law assumes (Lag): the measurement
%               chain plus the drive's dead time and lag
%
%   The bound is the stiffness of a chamber holding the saline volume as
%   a lower bound, the least free gas the charge leaves once the saline
%   has dissolved all it can at the highest pressure, the largest plausible
%   bulk modulus and the smallest plausible hose compliance:
%     C = Vgas / Pabs + min(Vsaline, Vchamber) / K + Chose,   k = A / C.

Patm = g.G_cfgPressureSchedPatm;                                % [Pa]
A = g.G_cfgPressureSchedPistonArea / 1e6;                       % [mm^2 -> m^2]
if fill.hoseOpen
    Vhose = g.G_cfgPressureSchedHoseVolume;                     % [mL]
    Chose = g.G_cfgPressureSchedHoseCompliance;                 % [m^3/Pa]
else
    Vhose = 0;
    Chose = 0;
end
RT = 8.314 * g.G_cfgPressureSchedGasTemp;                       % [J/mol]
Vl = fill.saline_mL / 1e6;                                      % [mL -> m^3]
% Chamber volume at the position: the swept volume to the end of travel
% plus the volume left there
Vch = max(A * (g.G_cfgPressureSchedEndOfTravel - x) / 1e3, 0) ...
    + (g.G_cfgPressureSchedPassagesVolume + Vhose) / 1e6;       % [mm -> m, mL -> m^3]
Vl = min(Vl, Vch);                                              % saline held by the chamber
% Oxygen charge, and what the saline holds at the highest absolute
% pressure seen (Henry); the rest is free gas
nCharge = fill.oxygenRatio * (fill.saline_mL / 1e6) * Patm / (8.314 * 273.15);   % [mol]
PabsRef = max(max(pRef, P), 0) * 1e6 + Patm;                    % [MPa -> Pa]
nGas = max(nCharge - g.G_cfgPressureSchedHenry * PabsRef * (fill.saline_mL / 1e6), 0);   % [mol]
Pabs = max(P, 0) * 1e6 + Patm;                                  % [Pa]
C = nGas * RT ./ Pabs.^2 + Vl / g.G_cfgPressureSchedBulkModulus + Chose;   % [m^3/Pa]
kEst = A ./ C / 1e9;                                            % [Pa/m -> MPa/mm]
PabsT = max(max(P, target), 0) * 1e6 + Patm;                    % [Pa]
CT = nGas * RT ./ PabsT.^2 + Vl / g.G_cfgPressureSchedBulkModulus + Chose;
kEstTarget = A ./ CT / 1e9;
if g.G_cfgAnalogMedianSize >= 5
    medScans = 2;
else
    medScans = 1;
end
lag = g.G_cfgAnalogFilterTimeConst + (medScans + 3) * g.G_cfgScanTime ...
    + g.G_cfgPressureSchedDriveDeadTime + g.G_cfgPressureSchedDriveLag;
end
