function s = loopScenario(name, p)
% loopScenario - operating case for simPressureLoop.
%
%   compress   gas-rich chamber at atmosphere, target 10 MPa; stiffness
%              rises by four orders of magnitude as the gas dissolves
%   stiffstep  liquid-only chamber at 10 MPa, target steps to 12 MPa at 1 s
%   discharge  liquid-only at 10 MPa, 1 mL/s outflow from 1 s to 21 s
%   valve      liquid-only at 10 MPa, outflow ramps to p.Qf over p.Topen
%              starting at 1 s (step when p.Topen is 0)
%
% s.target(t) and s.Qout(t) are the reference and the outflow; s.Qff(t) is
% the flow the controller is told about when feedforward is enabled.
  s.name = name;
  switch name
    case 'compress'
      s.T = 60; s.tEvent = 0;
      s.Pg0 = 0; s.ng0 = p.Patm*p.Vg0/(p.R*p.T);
      s.ndis0 = p.kH*p.Patm*p.Vl0;
      s.target = @(t) 10e6;
      s.Qout = @(t) 0;
    case 'stiffstep'
      s.T = 8; s.tEvent = 1;
      s.Pg0 = 10e6; s.ng0 = 0; s.ndis0 = p.kH*(10e6+p.Patm)*p.Vl0;
      s.target = @(t) 10e6 + 2e6*(t >= 1);
      s.Qout = @(t) 0;
    case 'discharge'
      s.T = 30; s.tEvent = 1;
      s.Pg0 = 10e6; s.ng0 = 0; s.ndis0 = p.kH*(10e6+p.Patm)*p.Vl0;
      s.target = @(t) 10e6;
      s.Qout = @(t) 1e-6*(t >= 1 & t < 21);
    case 'valve'
      s.T = p.valveT; s.tEvent = 1;
      s.Pg0 = 10e6; s.ng0 = 0; s.ndis0 = p.kH*(10e6+p.Patm)*p.Vl0;
      s.target = @(t) 10e6;
      if p.Topen > 0
        s.Qout = @(t) p.Qf*min(max((t-1)/p.Topen, 0), 1);
      else
        s.Qout = @(t) p.Qf*(t >= 1);
      end
    otherwise
      error('loopScenario: unknown scenario %s', name);
  end
  s.Qff = s.Qout;
end
