% studyCompare - velocity-command versus torque-command pressure loops over
% the compress, stiffstep and discharge scenarios. Prints a metrics table
% per scenario and writes <scenario>.csv for plot.gp.
%
% Set doSweeps = true to also run the gain sweeps that led to the final
% controller set.
p = plantParams();
A_N = p.A*1e6;                 % N per MPa

if ~exist('doSweeps', 'var'), doSweeps = false; end
if doSweeps
  printf('\n=== sweep V-P Kp on stiffstep (mm/s per MPa)\n');
  printf('%8s %9s %9s %9s %9s\n', 'Kp', 'settle s', 'oversh', 'ripple', 'cost');
  s = loopScenario('stiffstep', p);
  for Kp = [0.1 0.25 0.5 1 2 4]
    r = simPressureLoop(p, s, loopController('', 'V', Kp, 0, 0));
    printf('%8.2f %9.2f %9.3f %9.4f %9.3f\n', Kp, r.tSettle, r.overshoot, r.ripple, loopCost(r));
  end

  printf('\n=== sweep V-PI/PID on discharge (Kp, Ki, Kd)\n');
  printf('%6s %6s %6s %9s %9s %9s %9s\n', 'Kp', 'Ki', 'Kd', 'peak|e|', 'oversh', 'ripple', 'cost');
  s = loopScenario('discharge', p);
  bestV = []; bestVc = Inf;
  for Kp = [0.25 0.5 1]
    for Ki = [0 0.1 0.5 2]
      for Kd = [0 0.02 0.05]
        r = simPressureLoop(p, s, loopController('', 'V', Kp, Ki, Kd));
        printf('%6.2f %6.2f %6.3f %9.3f %9.3f %9.4f %9.3f\n', Kp, Ki, Kd, r.peakErr, r.overshoot, r.ripple, loopCost(r));
        if loopCost(r) < bestVc, bestVc = loopCost(r); bestV = [Kp Ki Kd]; end
      end
    end
  end
  printf('best V on discharge: Kp %.2f Ki %.2f Kd %.3f\n', bestV);

  printf('\n=== sweep T-PID on stiffstep (gains as multiples of A = %.0f N/MPa; Kd in N per MPa/s)\n', A_N);
  printf('%6s %6s %6s %9s %9s %9s %9s\n', 'Kp/A', 'Ki/A', 'Kd', 'settle s', 'oversh', 'ripple', 'cost');
  s = loopScenario('stiffstep', p);
  bestT = []; bestTc = Inf;
  for Kp = [0 0.05 0.2 0.5]
    for Ki = [0 0.2 1]
      for Kd = [0 30 100 300]
        r = simPressureLoop(p, s, loopController('', 'T', Kp*A_N, Ki*A_N, Kd));
        printf('%6.2f %6.2f %6.0f %9.2f %9.3f %9.4f %9.3f\n', Kp, Ki, Kd, r.tSettle, r.overshoot, r.ripple, loopCost(r));
        if loopCost(r) < bestTc, bestTc = loopCost(r); bestT = [Kp Ki Kd]; end
      end
    end
  end
  printf('best T on stiffstep: Kp %.2fA Ki %.2fA Kd %.0f\n', bestT);
end

C = { loopController('V-PI.25-.2-a5',       'V', 0.25, 0.2, 0,    false, 5e-3), ...
      loopController('V-PI.25-.2-a50',      'V', 0.25, 0.2, 0,    false, 50e-3), ...
      loopController('V-PI.5-.2-a50-stop',  'V', 0.5,  0.2, 0,    true,  50e-3), ...
      loopController('V-PI.5-.2-a200-stop', 'V', 0.5,  0.2, 0,    true,  200e-3), ...
      loopController('V-PID+Kd.05-a50-stop','V', 0.5,  0.2, 0.05, true,  50e-3), ...
      loopController('T-FF-Kd100',          'T', 0,    0,   100) };
names = cellfun(@(c) c.name, C, 'UniformOutput', false);
S = {'compress', 'stiffstep', 'discharge'};
for si = 1:numel(S)
  s = loopScenario(S{si}, p);
  printf('\n=== %s  (T=%g s, event at %g s)\n', s.name, s.T, s.tEvent);
  printf('%-20s %9s %9s %9s %9s %9s %12s\n', 'controller', 'settle s', 'oversh', 'peak|e|', 'rms e', 'ripple', 'cmd act');
  R = cell(1, numel(C));
  for ci = 1:numel(C)
    r = simPressureLoop(p, s, C{ci}); R{ci} = r;
    printf('%-20s %9.2f %9.3f %9.3f %9.4f %9.4f %12.4f\n', C{ci}.name, r.tSettle, r.overshoot, r.peakErr, r.rmsErr, r.ripple, r.uAct);
  end
  printf('stiffness dP/dx: start %.3g, end %.3g MPa/mm\n', r.stiff(1)/1e9, r.stiff(end)/1e9);
  writeLoopCsv(sprintf('%s.csv', s.name), R, names, p);
end
