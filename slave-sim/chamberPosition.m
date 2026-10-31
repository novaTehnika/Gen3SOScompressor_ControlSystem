function x = chamberPosition(sl, P)
%CHAMBERPOSITION Axis position x (mm) at which the simulated chamber, with
%   its present contents, holds gauge pressure P (MPa) at rest. Tests use
%   it to start a pressure scenario close to its target:
%     sl.ax.x = chamberPosition(sl, 0.5);

x = slaveSimMex('chamberPosition', sl.id, P);
end
