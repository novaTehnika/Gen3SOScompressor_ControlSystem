% studyValve - pressure dip when the discharge valve opens, versus valve
% opening time, for a set of velocity-command controllers and the torque
% feedforward reference. Prints the dip and recovery tables, writes
% valve_dip.csv (dip vs opening time) and valve03.csv (traces for a 0.3 s
% opening) for plotValve.gp.
p = plantParams();
C = { loopController('V-PI.25-.2 a5',       'V', 0.25, 0.2, 0,    false, 5e-3), ...
      loopController('V-PI.25-.2 a50',      'V', 0.25, 0.2, 0,    false, 50e-3), ...
      loopController('V-PI.5-.2 a50',       'V', 0.5,  0.2, 0,    false, 50e-3), ...
      loopController('V-PI1-.5 a50 tf20',   'V', 1.0,  0.5, 0,    false, 50e-3), ...
      loopController('V-PID.5-.2-.05 a50',  'V', 0.5,  0.2, 0.05, false, 50e-3), ...
      loopController('V-PI.5-.2 a50 +FF',   'V', 0.5,  0.2, 0,    false, 50e-3), ...
      loopController('T-FF-Kd100',          'T', 0,    0,   100) };
C{4}.tauF = 0.02;      % Kp 1 needs the shorter sensor filter
C{6}.vff = true;       % velocity feedforward of the expected flow
names = cellfun(@(c) c.name, C, 'UniformOutput', false);
Topens = [0 0.1 0.3 1 3 10];
traceTopen = 0.3;
dip = zeros(numel(Topens), numel(C)); rec = dip;
R03 = cell(1, numel(C));
printf('\n=== valve opening: Qf = %.1f mL/s, dip (MPa) vs opening time (s)\n', p.Qf*1e6);
printf('%-22s', 'controller'); printf('%9.1f', Topens); printf('\n');
for ci = 1:numel(C)
  for ti = 1:numel(Topens)
    p.Topen = Topens(ti); p.valveT = max(8, 1 + 1.5*p.Topen + 3);
    s = loopScenario('valve', p); r = simPressureLoop(p, s, C{ci});
    ie = r.t >= s.tEvent; dip(ti,ci) = (r.tgt(1) - min(r.P(ie)))/1e6;
    e = abs(r.tgt - r.P); k = find(e > p.tol & ie, 1, 'last');
    if isempty(k), rec(ti,ci) = 0; else rec(ti,ci) = r.t(k) - s.tEvent; end
    if Topens(ti) == traceTopen, R03{ci} = r; end
  end
  printf('%-22s', C{ci}.name); printf('%9.3f', dip(:,ci)); printf('\n');
end
printf('\nrecovery to within %.1f MPa (s after the valve starts opening)\n', p.tol/1e6);
for ci = 1:numel(C), printf('%-22s', C{ci}.name); printf('%9.2f', rec(:,ci)); printf('\n'); end
fid = fopen('valve_dip.csv', 'w');
fprintf(fid, 'Topen'); fprintf(fid, ',%s', names{:}); fprintf(fid, '\n'); fclose(fid);
dlmwrite('valve_dip.csv', [Topens' dip], '-append', 'precision', '%.4g');
writeLoopCsv('valve03.csv', R03, names, p);
