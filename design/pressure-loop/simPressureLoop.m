function r = simPressureLoop(p, s, c)
% simPressureLoop - run one scenario with one controller.
%
% The controller runs every scan on that scan's filtered pressure and its
% command takes effect on the next scan, matching the FB call order in
% PRG_Main. The plant integrates nsub substeps per scan.
%
% r.t r.P r.u r.v r.tgt r.stiff   traces (SI; r.stiff is dP/dx in Pa/m)
% r.overshoot r.peakErr r.rmsErr r.ripple   MPa, measured after s.tEvent
% r.tSettle   s after the event until |e| stays within p.tol; NaN if never
% r.uAct      command activity over the last 2 s: std of the per-scan
%             command change, mm/s ('V') or MPa of force/A ('T')
  if ~isnan(c.amax), p.amax = c.amax; end
  if ~isnan(c.tauF), p.tauF = c.tauF; end
  N = round(s.T/p.Ts); dts = p.Ts/p.nsub;
  kEst = 25e9; dPbuf = zeros(1,10); dxbuf = zeros(1,10); xPrev = 0; PfEst = s.Pg0;
  alpha = p.Ts/(p.tauF + p.Ts);
  x = 0; v = 0; vref = 0; Pg = s.Pg0;
  ndis = s.ndis0; ntot = s.ng0 + s.ndis0;
  buf = Pg * ones(1, p.medN); Pf = Pg; Pfprev = Pg; d = 0; I = 0;
  if c.mode == 'V', uApplied = 0; else uApplied = p.A*Pg; end
  randn('seed', 1);
  r.t = (0:N-1)'*p.Ts; r.P = zeros(N,1); r.u = zeros(N,1); r.v = zeros(N,1);
  r.tgt = zeros(N,1); r.stiff = zeros(N,1);
  for k = 1:N
    t = r.t(k);
    % --- sensor
    Pm = Pg + p.noise*randn();
    buf = [buf(2:end) Pm]; med = median(buf);
    Pf = Pf + alpha*(med - Pf);
    % --- controller
    tgt = s.target(t); e = tgt - Pf;
    d = d + (p.Ts/(p.Tf + p.Ts))*((Pf - Pfprev)/p.Ts - d); Pfprev = Pf;
    ff = 0;
    if c.mode == 'T', ff = p.A*tgt; end
    if c.vff && c.mode == 'V', ff = s.Qff(t)/p.A; end
    raw = ff + c.Kp*e + c.Ki*I - c.Kd*d;
    if c.mode == 'V', lim = p.vmax; else lim = p.Fcap; end
    if c.stopLaw
      % stiffness estimate over the last 10 scans of travel (Pa/m)
      dPbuf = [dPbuf(2:end) (Pf - PfEst)]; dxbuf = [dxbuf(2:end) (x - xPrev)];
      PfEst = Pf; xPrev = x;
      if abs(sum(dxbuf)) > 5e-6
        kEst = min(max(abs(sum(dPbuf))/abs(sum(dxbuf)), 0.01e9), 100e9);
      end
      dist = abs(e)/kEst;
      rateStop = -p.amax*p.Ts + sqrt((p.amax*p.Ts)^2 + 2*p.amax*dist);
      lim = min(lim, rateStop);
    end
    u = min(max(raw, -lim), lim);
    % conditional anti-windup: integrate unless saturated with the error
    % pushing further into saturation
    if raw == u || (raw > u && e < 0) || (raw < u && e > 0)
      I = I + e*p.Ts;
    end
    % --- plant, one scan at the previous scan's command
    for j = 1:p.nsub
      Pabs = Pg + p.Patm;
      if c.mode == 'V'
        dv = uApplied - vref; vref = vref + max(min(dv, p.amax*dts), -p.amax*dts);
        v = v + (vref - v)*dts/p.tauV;
        if Pg*p.A + p.Fc > p.Fcap && v > 0, v = 0; vref = 0; end   % stalled at the torque limit
      else
        F = uApplied;
        fr = p.b*v + p.Fc*max(min(v/p.veps, 1), -1);
        a = (F - Pg*p.A - fr)/p.mr;
        v = v + a*dts; v = min(max(v, -p.vmax), p.vmax);
      end
      x = x + v*dts;
      neq = min(p.kH*Pabs*p.Vl0, ntot);
      ndot = (neq - ndis)/p.tauDiss; ndis = min(ndis + ndot*dts, ntot);
      ng = ntot - ndis;
      C = ng*p.R*p.T/Pabs^2 + p.Vl0/p.K;      % m^3/Pa chamber compliance
      Pg = Pg + (p.A*v - s.Qout(t) - (p.R*p.T/Pabs)*ndot)/C*dts;
      Pg = max(Pg, -p.Patm + 1);
    end
    uApplied = u;
    r.P(k) = Pg; r.u(k) = u; r.v(k) = v; r.tgt(k) = tgt; r.stiff(k) = p.A/C;
  end
  % --- metrics
  ie = r.t >= s.tEvent; e = r.tgt - r.P;
  r.overshoot = max(r.P(ie) - r.tgt(ie))/1e6;
  ok = abs(e) < p.tol;
  kset = find(~ok & ie, 1, 'last');
  if isempty(kset), r.tSettle = 0;
  elseif kset == N, r.tSettle = NaN;
  else r.tSettle = r.t(kset+1) - s.tEvent; end
  r.peakErr = max(abs(e(ie)))/1e6;
  last = r.t > s.T - 2;
  r.rmsErr = sqrt(mean(e(last).^2))/1e6;
  r.ripple = std(r.P(last))/1e6;
  if c.mode == 'V', r.uAct = std(diff(r.u(last)))*1e3; else r.uAct = std(diff(r.u(last)))/p.A/1e6; end
  r.mode = c.mode;
end
