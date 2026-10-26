function charRunSim()
%CHARRUNSIM Run bench/characterisation/charRun against the simulated slave
%   through fakeAxisLink, then the three fits on what it logged: ramps and
%   velocity steps in a velocity session with axis probe captures, a
%   delivery through the orifice bank in pressure mode, and the pressure
%   and position guards.
%   Run from the repository root after  python3 tools/st2m.py slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'characterisation'), fullfile(here, '..', 'bench', 'modbus'));
saveDir = tempname();
ccfg = charConfig();

% --- Velocity session: ramps on the pressure, dwell, steps, then a hold ---------
% The simulated chamber is a gas spring: 1.7 MPa at 255 mm, 3 MPa at 256.4 mm.
opts = linkOpts(saveDir, 255);
opts.probe = true;
path = {{'ramp', 0.1, 3.0, 40}, {'dwell', 1.2}, {'ramp', -0.1, 2.0, 40}, ...
        {'velstep', [0.5 0], [1.5 1.5]}, {'hold', 1}};
r = charRun('sim', path, opts);
assert(isempty(r.abort), r.abort);
log = r.runLog;
s1 = log(log(:, 11) == 1, :);
assert(s1(end, 5) >= 3.0 && s1(end, 5) < 3.3, 'ramp did not end at 3 MPa (%.2f)', s1(end, 5));
assert(all(s1(10:end, 7) == 3 & s1(10:end, 8) == 1), 'ramp not in velocity mode');
s3 = log(log(:, 11) == 3, :);
assert(s3(end, 5) <= 2.0 && s3(end, 5) > 1.7, 'down ramp did not end at 2 MPa (%.2f)', s3(end, 5));
session = log(log(:, 11) >= 2 & log(:, 11) <= 4, :);
assert(~any(session(:, 9) == 7) && all(session(session(:, 11) <= 3, 7) == 3), 'velocity session left the mode');
s5 = log(log(:, 11) == 5, :);
assert(any(s5(:, 7) == 2) && any(s5(:, 9) == 3), 'hold did not run in position mode');
assert(numel(r.captures) == 5, 'expected one capture per command change, got %d', numel(r.captures));
cap = r.captures{1};
assert(cap.step == 1 && numel(cap.t) == ccfg.probe.nSamples);
i0 = find(cap.t >= 0, 1);
assert(all(cap.command(1:i0 - 1) == 0) && abs(cap.command(i0) - 0.1) < 1e-3, 'capture not aligned on the command change');
assert(cap.velocity(i0 + 40) > 0.09, 'velocity did not follow the command in the capture');
cap = r.captures{4};
assert(abs(cap.command(i0) - 0.5) < 1e-3 && abs(cap.command(i0 - 1) + 0.1) < 1e-3, 'velstep capture command');
assert(exist(r.runFile, 'file') == 2 && numel(r.files) == 5);
assert(r.exchangeFailures == 0 && r.probeFailures == 0);
S = fakeAxisLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  velocity session: two ramps, dwell, steps, hold; five captures\n');

% The simulated axis follows its slewed command exactly: the lag fit must
% find a lag of at most a few scans and a small residual.
lag = charDriveLag(r.files, struct('quiet', true));
assert(all([lag.captures.tauV] < 0.006), 'lag %s s on a lag-free axis', mat2str([lag.captures.tauV], 3));
assert(all([lag.captures.residual] < 0.03), 'lag fit residual %s', mat2str([lag.captures.residual], 3));
assert(~isnan(lag.captures(4).tauP), 'no pressure fit on the 0.5 mm/s step');
fprintf('ok  drive lag fit on the captures: %.1f ms, residual %.1f %%\n', ...
    1000 * lag.tauV_motor_s, 100 * max([lag.captures.residual]));

% The gas spring is not a liquid: the bulk modulus fit must reject it and
% attribute the compliance to gas of about the chamber volume.
bk = charBulkModulus(r.runFile, struct('quiet', true, 'deadVolume_mL', 10, 'pressureRange', [2.0 3.1]));
assert(~bk.pass && bk.K_lin_GPa < 0.5, 'gas spring accepted as a liquid (K %.3f GPa)', bk.K_lin_GPa);
S = fakeAxisLink('state');
vGas = S.sl.sim.chamberGasVolume * S.sl.sim.atmMPa / (2.5 + S.sl.sim.atmMPa) / 1000;
assert(abs(bk.Vg_mL - vGas) / vGas < 0.3, 'gas volume %.2f mL, chamber holds about %.2f mL', bk.Vg_mL, vGas);
fprintf('ok  bulk modulus fit rejects the gas spring (K %.4f GPa, gas %.1f mL of %.1f)\n', ...
    bk.K_lin_GPa, bk.Vg_mL, vGas);

% --- Delivery: pressure mode, valve opens, the loop balances the outflow --------
opts = linkOpts(saveDir, 250);
fakeAxisLink('valveAt', 12, 0.68);
r = charRun('simdeliver', {{'pressure', 2.0, 30}}, opts);
assert(isempty(r.abort), r.abort);
% The volume that left while the step ran (the outflow goes on after the
% stop). The simulated chamber is a gas spring, so what left exceeds the
% swept volume by the gas expansion as the pressure settles, a term the
% script's liquid correction cannot see; the check allows for it.
S = fakeAxisLink('state');
inStep = r.runLog(:, 11) == 1 & r.runLog(:, 9) == 5;
tEnd = max(r.runLog(inStep, 1));
collected = interp1(S.collectLog(:, 1), S.collectLog(:, 2), tEnd) / 1000;
vGasAt = @(P) S.sl.sim.chamberGasVolume * S.sl.sim.atmMPa ./ (P + S.sl.sim.atmMPa) / 1000;   % mL
fl = charOrificeFlow(r.runFile, struct('quiet', true, 'tOpen', 12, 'collected_mL', collected));
assert(abs(fl.Cd - 0.68) / 0.68 < 0.06, 'Cd %.3f, valve set to 0.68', fl.Cd);
% The loop pulls the pressure back up during the window, so the piston also
% compresses the gas; with the gas spring's equivalent bulk modulus the
% correction removes that and Cd comes back within 2 %.
late = inStep & r.runLog(:, 1) > 15;
Pm = mean(r.runLog(late, 5)); xm = mean(r.runLog(late, 2));
Kgas = (ccfg.pistonArea_mm2 * (ccfg.posEOT - xm) + ccfg.deadVolume_mL * 1000) * (Pm + S.sl.sim.atmMPa) ...
    / (vGasAt(Pm) * 1000);                                                 % MPa
flk = charOrificeFlow(r.runFile, struct('quiet', true, 'tOpen', 12, 'K_Pa', Kgas * 1e6));
assert(abs(flk.Cd - 0.68) / 0.68 < 0.02, 'Cd %.3f with the gas correction, valve set to 0.68', flk.Cd);
pAt = @(tq) interp1(r.runLog(inStep, 1), r.runLog(inStep, 5), tq);
expansion = vGasAt(pAt(tEnd)) - vGasAt(pAt(12));
expected = collected / (collected - expansion);
assert(abs(fl.gravimetric.ratio - expected) < 0.02, 'collected %.3f of swept, %.3f expected with %.1f mL of gas expansion', ...
    fl.gravimetric.ratio, expected, expansion);
assert(all(cellfun(@(c) c.pass, fl.checks(1:2))), 'Cd or its spread failed');
fl2 = charOrificeFlow(r.runFile, struct('quiet', true));
assert(abs(fl2.windows(1).t1 - fl.windows(1).t1) < 2, 'valve opening detected at %.1f s, set at 12 s', fl2.windows(1).t1);
fprintf('ok  delivery: Cd %.3f (%.3f with the gas correction), collected %.1f %% of swept (%.1f expected), opening found at %.1f s\n', ...
    fl.Cd, flk.Cd, 100 * fl.gravimetric.ratio, 100 * expected, fl2.windows(1).t1);

% --- Guards ----------------------------------------------------------------------
opts = linkOpts(saveDir, 257);                 % about 10 MPa in the gas spring
opts.pressureStop = 10.5;
r = charRun('simguard', {{'ramp', 0.1, 11.5, 20}}, opts);
assert(~isempty(strfind(r.abort, 'pressure guard')), 'pressure guard did not trip: %s', r.abort);
assert(max(r.runLog(:, 5)) < 11.0, 'pressure reached %.2f MPa after the guard', max(r.runLog(:, 5)));
S = fakeAxisLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0);
opts = linkOpts(saveDir, 150);                 % outside the chamber
opts.posStop = 160;
r = charRun('simposguard', {{'jog', 2, 10}}, opts);
assert(~isempty(strfind(r.abort, 'position guard')), 'position guard did not trip: %s', r.abort);
S = fakeAxisLink('state');
assert(S.sl.ax.x < opts.posStop + 1 && S.sl.ax.v == 0);
fprintf('ok  pressure and position guards stop the run\n');

% A drive fault ends the run with the stop sent.
opts = linkOpts(saveDir, 240);
fakeAxisLink('faultAt', 4);
r = charRun('simfault', {{'jog', 1, 8}}, opts);
assert(~isempty(strfind(r.abort, 'status 10')), 'fault did not abort: %s', r.abort);
S = fakeAxisLink('state');
assert(S.u.motionEnable == 0 && S.u.modeCode ~= 3, 'motion still commanded after the fault');
fprintf('ok  slave fault aborts the run\n');

fprintf('all passed\n');
end


function opts = linkOpts(saveDir, x0)
clear fakeAxisLink
fakeAxisLink('init', x0);
link.read     = @(start, n) fakeAxisLink('read', start, n);
link.write    = @(start, v) fakeAxisLink('write', start, v);
link.writeOne = @(addr, v) fakeAxisLink('writeOne', addr, v);
link.close    = @() [];
opts.link = link;
opts.now = @() fakeAxisLink('now');
opts.wait = @(t) fakeAxisLink('wait', t);
opts.safetyModel = '';
opts.saveDir = saveDir;
end
