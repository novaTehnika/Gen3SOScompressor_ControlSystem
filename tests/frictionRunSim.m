function frictionRunSim()
%FRICTIONRUNSIM Run bench/friction/frictionRun against the simulated slave
%   through fakeProbeLink, with a friction torque model standing in for the
%   drive's torque feedback, and check that frictionFit recovers the model
%   from the logs: Coulomb, viscous and Stribeck terms and the breakaway
%   from the unloaded set, the load-proportional term from a gas-charge set
%   and a delivery, the drive lag, breakaway and stick from the scan-rate
%   captures of the step probe, and that the run stops the axis when the
%   pressure passes its limit.
%   Run from the repository root after  python3 tools/st_translate.py --target m slave-src slave-sim/generated
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools bench/friction bench/common tests/fakeProbeLink.m

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'friction'), fullfile(here, '..', 'bench', 'common'));
saveDir = tempname();
fcfg = frictionBenchConfig();

% The model: forces in N, b in N per mm/s, speeds in mm/s.
truth = struct('F0', 300, 'Fc', 2000, 'dFs', 800, 'vs', 0.3, 'b', 150, 'mu', 0.12, ...
    'noisePct', 0.3, 'tStick', 0.3, 'newtonPerPct', fcfg.newtonPerPct);
randn('seed', 1);

% --- Unloaded: the axis outside the chamber, speeds in both directions ------
opts = linkOpts(saveDir);
fakeProbeLink('torqueModel', truth);
fakeProbeLink('slave', @(sl) placeAxis(sl, 100));
opts.speeds = [0.1 0.3 1 3 5];
opts.tSteady = [5 4 3 2.5 2.5];
opts.repeats = 1;
r = frictionRun('sim', 'unloaded', opts);
assert(isempty(r.abort), r.abort);
assert(exist(r.file, 'file') == 2 && size(r.runLog, 2) == 12);
assert(r.exchangeFailures == 0);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  unloaded set: %d steps logged\n', numel(r.steps));

f = frictionFit(r.file, {}, struct('quiet', true));
assert(numel(f.segments) == 10 && all(~isnan([f.segments.F])), 'a jog step gave no steady segment');
assert(all(abs([f.segments.v] - [f.segments.vRef]) < 0.05), 'segments not at the commanded speed');
u = f.unloaded;
assert(u.ok && u.stribeck.ok, 'Stribeck form not kept');
st = u.stribeck;
assert(abs(st.Fc - truth.Fc) < 150, 'Fc %.0f, model %.0f', st.Fc, truth.Fc);
assert(abs(st.b - truth.b) < 30, 'b %.1f, model %.1f', st.b, truth.b);
assert(abs(st.dFs - truth.dFs) < 250, 'Stribeck %.0f, model %.0f', st.dFs, truth.dFs);
assert(st.vs >= 0.2 && st.vs <= 0.5, 'Stribeck knee %.2f mm/s', st.vs);
assert(abs(st.F0 - truth.F0) < 150, 'F0 %.0f, model %.0f', st.F0, truth.F0);
peak = truth.Fc + truth.dFs;
assert(f.breakaway.n == 10 && f.breakaway.F > 0.7 * peak && f.breakaway.F < 1.3 * (peak + truth.F0), ...
    'breakaway %.0f N, model %.0f', f.breakaway.F, peak);
assert(numel(f.pairs) == 5 && all(strcmp({f.pairs.check}, 'ok')), 'reversal pairs not symmetric');
fprintf('ok  unloaded fit: Fc %.0f N, b %.0f N s/m, Fs %.0f N at %.2f mm/s, breakaway %.0f N\n', ...
    st.Fc, st.b * 1000, st.Fs, st.vs, f.breakaway.F);

% --- Step probe: velocity steps and the breakaway at the scan rate ----------
opts = linkOpts(saveDir);
truthLag = truth;
truthLag.tauV = 0.025;
fakeProbeLink('torqueModel', truthLag);
fakeProbeLink('slave', @(sl) placeAxis(sl, 100));
opts.probe = true;
rs = frictionRun('sim', {{'stepv', 1, 2.2}, {'jog', 0.3, 4}}, opts);
assert(isempty(rs.abort), rs.abort);
assert(numel(rs.captures) == 10 && rs.probeFailures == 0, '%d captures, %d failures', ...
    numel(rs.captures), rs.probeFailures);
assert(all(cellfun(@(c) exist(c, 'file') == 2, rs.capFiles)));
sf = stepResponseFit(rs.capFiles, struct('quiet', true));
tr = sf.transitions;
mv = tr(~[tr.fromRest]);
rest = tr([tr.fromRest]);
assert(numel(mv) == 6 && numel(rest) == 3 && numel(sf.steady) == 1, ...
    '%d moving, %d from rest, %d steady', numel(mv), numel(rest), numel(sf.steady));
assert(all(abs([mv.tauV] - truthLag.tauV) < 0.006), 'tauV %s ms', mat2str([mv.tauV] * 1000, 3));
assert(abs(sf.tauV - truthLag.tauV) < 0.004, 'median tauV %.1f ms', sf.tauV * 1000);
peak = truth.Fc + truth.dFs;
assert(all(abs(abs([rest.Fbreak]) - peak) < 0.15 * peak + truth.F0), 'breakaway %s N', ...
    mat2str([rest.Fbreak], 4));
assert(all(abs([rest.tStick] - truth.tStick) < 0.06), 'stick %s s', mat2str([rest.tStick], 3));
assert(abs(sf.steady(1).v - 0.3) < 0.05 && sf.steady(1).Fstd < 600);
stepResponseFit(rs.capFiles);
fprintf('ok  step probe: tauV %.1f ms (model %.0f), breakaway %.0f N, stick %.2f s\n', ...
    sf.tauV * 1000, truthLag.tauV * 1000, sf.Fbreak, sf.tStick);

% --- Gas charge: a soft chamber at about 1 MPa, short strokes ---------------
opts = linkOpts(saveDir);
fakeProbeLink('torqueModel', truth);
fakeProbeLink('slave', @(sl) softChamber(sl, 240, 1.0));
opts.speeds = [0.3 1 5];
opts.tSteady = [4 3 2.5];
opts.repeats = 1;
opts.strokeMax = 10;
rg = frictionRun('sim', 'gas', opts);
assert(isempty(rg.abort), rg.abort);
P = rg.runLog(rg.runLog(:, 2) >= 1, 9);     % from the first step: before it the registers read zero
assert(min(P) > 0.5 && max(P) < 3.5, 'gas pressure left 0.5-3.5 MPa (%.2f-%.2f)', min(P), max(P));
fprintf('ok  gas set: pressure %.2f to %.2f MPa\n', min(P), max(P));

% --- Delivery: pressure mode at 10 MPa with liquid leaving the chamber ------
opts = linkOpts(saveDir);
fakeProbeLink('torqueModel', truth);
fakeProbeLink('slave', @(sl) setfield(sl, 'hook', setfield(sl.hook, 'outflow', 4000)));
opts.target = 10;
opts.seconds = 25;
rd = frictionRun('sim', 'delivery', opts);
assert(isempty(rd.abort), rd.abort);
d = rd.runLog(rd.runLog(:, 4) == 7 & rd.runLog(:, 3) == 5, :);
assert(max(d(:, 9)) > 8 && max(d(:, 9)) <= fcfg.pMax.delivery, 'delivery pressure %.2f MPa', max(d(:, 9)));
late = d(d(:, 1) > d(end, 1) - 8, :);       % after the compression, the orifice flow sets the speed
assert(mean(late(:, 7)) > 0.8 && mean(late(:, 7)) < 1.6, 'delivery speed %.2f mm/s', mean(late(:, 7)));
fprintf('ok  delivery: %.2f MPa at %.2f mm/s\n', mean(late(:, 9)), mean(late(:, 7)));

f = frictionFit(r.file, {rg.file, rd.file}, struct('quiet', true));
l = f.loaded;
assert(l.ok && strcmp(l.form, 'full'), 'loaded fit form: %s', l.form);
assert(abs(l.mu - truth.mu) < 0.03, 'mu %.3f, model %.3f', l.mu, truth.mu);
assert(abs(l.Fc - truth.Fc) < 300, 'loaded Fc %.0f, model %.0f', l.Fc, truth.Fc);
% The loaded fit is the plain form, so the Stribeck hump at 0.3 mm/s pulls
% its slope low; its own standard error is about 20. At the 4 ms scan the
% fit returns 80 to 91 (truth 150), so the band is 75.
assert(abs(l.b - truth.b) < 75, 'loaded b %.1f, model %.1f', l.b, truth.b);
assert(sum([f.bins.used]) >= 8, 'only %d delivery bins used', sum([f.bins.used]));
fprintf('ok  loaded fit: Fc %.0f N, b %.0f N s/m, mu %.3f (efficiency %.3f)\n', ...
    l.Fc, l.b * 1000, l.mu, l.etaForward);

% The printed report runs through.
frictionFit(r.file, {rg.file, rd.file});

% --- The pressure limit stops the run with the axis stopped ------------------
opts = linkOpts(saveDir);
fakeProbeLink('torqueModel', truth);
opts.target = 10;
opts.seconds = 25;
opts.pMax = 5;
ra = frictionRun('simabort', 'delivery', opts);
assert(~isempty(strfind(ra.abort, 'pressure')), 'pressure limit did not abort: %s', ra.abort);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.u.modeCode == 0 && S.sl.ax.v == 0, 'motion still commanded after the abort');
fprintf('ok  pressure above pMax aborts the run (%s)\n', ra.abort);

fprintf('all passed\n');
end


function opts = linkOpts(saveDir)
clear fakeProbeLink
fakeProbeLink('init');
randn('seed', 3);           % each run draws the same torque noise, whatever ran before it
link.read     = @(start, n) fakeProbeLink('read', start, n);
link.write    = @(start, v) fakeProbeLink('write', start, v);
link.writeOne = @(addr, v) fakeProbeLink('writeOne', addr, v);
link.close    = @() [];
opts.link = link;
opts.now = @() fakeProbeLink('now');
opts.wait = @(t) fakeProbeLink('wait', t);
opts.safetyModel = '';
opts.saveDir = saveDir;
end


function sl = placeAxis(sl, x)
sl.ax.x = x;
end


function sl = softChamber(sl, x, P)
% A large gas volume whose contact position puts the piston at x at P MPa
% gauge, so a few millimetres of travel change the pressure little.
sl.ax.x = x;
sl.sim.chamberGasVolume = 3e6;                               % mm^3 at atmosphere
V = sl.sim.chamberGasVolume * sl.sim.atmMPa / (sl.sim.atmMPa + P);
sl.sim.chamberContactPos = x - (sl.sim.chamberGasVolume - V) / sl.sim.pistonArea;
end
