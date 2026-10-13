function p = plantParams()
% plantParams - plant, actuator, sensor and scenario parameters for the
% pressure loop study. SI units unless the comment says otherwise.
%
% Plant: a piston of area A compresses a chamber holding liquid of
% effective bulk modulus K and oxygen gas (isothermal) that dissolves into
% the liquid per Henry's law with time constant tauDiss. Chamber stiffness
% dP/dx therefore runs from ~0.002 MPa/mm when gas-rich to ~10-25 MPa/mm
% once the gas has dissolved.
%
% Velocity actuator (Y_DirectControl VelocityTLMode): the command is slewed
% at amax, then lagged by tauV for the drive velocity loop, and stalls when
% the chamber force exceeds Fcap. Torque actuator (TorqueVLMode): reflected
% mass mr driven by force F against chamber pressure with viscous and
% Coulomb friction, velocity clamped at vmax.
%
% Sensor: 4-20 mA transducer sampled every Ts with white noise, median of
% medN samples and a first-order filter tauF, as in the slave.

  % chamber
  p.A       = 3.431e-3;   % m^2 piston area, 2.602 in bore
  p.R       = 8.314;      % J/(mol K)
  p.T       = 293;        % K
  p.Patm    = 101325;     % Pa
  p.K       = 1.0e9;      % Pa effective liquid bulk modulus (water 2.2e9; seals and hose lower it)
  p.Vl0     = 300e-6;     % m^3 liquid
  p.Vg0     = 200e-6;     % m^3 gas at atmosphere for the gas-rich start
  p.kH      = 1.28e-5;    % mol/(m^3 Pa) Henry constant, O2 in water
  p.tauDiss = 30;         % s dissolution time constant

  % discharge valve (used by the 'valve' scenario)
  p.Qf      = 1e-6;       % m^3/s final outflow once the valve is open
  p.Topen   = 0.3;        % s valve opening time, 0 for a step
  p.valveT  = 8;          % s simulated time for the valve scenario

  % timing
  p.Ts      = 0.002;      % s slave scan
  p.nsub    = 5;          % plant substeps per scan

  % actuator
  p.vmax    = 5e-3;       % m/s   velocity clamp (G_cfgPressureVelMax)
  p.amax    = 5e-3;       % m/s^2 velocity slew (G_cfgDirectCtrlAccel)
  p.tauV    = 0.02;       % s     drive velocity loop lag
  p.Fcap    = 1.27 * 2*pi*70/5.08e-3 * 0.85;  % N at 100 % rated torque: 1.27 N m, 70:1 gearbox, 5.08 mm lead, 85 % efficiency
  p.mr      = 5e4;        % kg reflected motor and gearbox inertia at the piston
  p.b       = 5e3;        % N s/m viscous friction
  p.Fc      = 500;        % N Coulomb friction (seals, screw)
  p.veps    = 1e-5;       % m/s stiction regularisation

  % sensor and filters
  p.noise   = 0.005e6;    % Pa rms per sample
  p.medN    = 5;          % median window (G_cfgAnalogMedianSize)
  p.tauF    = 0.05;       % s first-order filter (G_cfgAnalogFilterTimeConst)
  p.Tf      = 0.02;       % s derivative filter

  % metrics
  p.tol     = 0.1e6;      % Pa settling band
end
