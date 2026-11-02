function pressureNoiseRunSim()
%PRESSURENOISERUNSIM Run the pressure bench's noise scripts against the
%   simulated slave through fakeProbeLink, which puts the slave's own
%   FB_Probe behind the step probe registers: pressureNoiseProbe at rest
%   (windows and capture from a known noise on the analog input, each
%   found again in the ring's own log of records) and pressureNoiseRun
%   (the path sequenced through masterStep, every step yielding windows and
%   a capture taken while the step's condition holds), and a slave fault
%   ending the run with the axis stopped.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools models bench/pressure bench/common tests/fakeProbeLink.m

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'pressure'), fullfile(here, '..', 'bench', 'common'));
saveDir = tempname();
sigma = 0.002;                  % V rms of the noise on the input
nRec = 250;                     % records in a window

% --- At rest: windows and a capture of the input's noise ------------------------
opts = linkOpts(saveDir, sigma);
r = pressureNoiseProbe('simrest', 3, opts);
assert(size(r.stats, 1) == 3 && numel(r.volts) == nRec && r.probeFailures == 0);
assert(exist(r.file, 'file') == 2);
S = fakeProbeLink('state');
ring = S.ring(:, 6) / 5000;     % the log of raw inputs [V], one row per record
for k = 1:3
    assert(inRing(ring, r.stats(k, :), nRec), 'window %d is not a second of the ring', k);
end
assert(~isempty(findSegment(ring, r.volts)), 'capture is not a run of records of the ring');
assert(all(r.stats(:, 2) > 0.7 * sigma & r.stats(:, 2) < 1.3 * sigma), ...
    'window rms %s V, noise %.4f V', mat2str(r.stats(:, 2)', 3), sigma);
assert(abs(std(r.volts) - sigma) < 0.3 * sigma, 'capture rms %.4f V, noise %.4f V', std(r.volts), sigma);
assert(all(r.stats(:, 4) - r.stats(:, 3) > 3 * sigma) && all(abs(r.stats(:, 1) - mean(r.volts)) < 0.005), ...
    'window extremes or mean');
csv = dlmread(r.file, ',', 1, 0);
assert(size(csv, 1) == nRec && abs(csv(2, 1) - 0.004) < 1e-6 && max(abs(csv(:, 2) - r.volts)) < 1e-5, ...
    'capture file');
rep = pressureNoiseReport(r.file);
assert(abs(rep.rmsMv - 1000 * std(r.volts)) < 0.01, 'report of the saved capture');
fprintf('ok  at rest: three windows and a capture, each a run of the ring''s records\n');

% --- Noise off: the input's own scatter is the quantisation alone -----------------
opts = linkOpts(saveDir, 0);
r = pressureNoiseProbe('simquiet', 1, opts);
assert(r.stats(1, 2) < 0.0003, 'quiet input rms %.5f V', r.stats(1, 2));
fprintf('ok  without noise the window rms is below one count\n');

% --- Moving: a path of steps -------------------------------------------------------
opts = linkOpts(saveDir, sigma);
path = {{'idle', 1}, {'position', 245, 1}, {'jog', 2, 2}, {'pressure', 2, 3}};
r = pressureNoiseRun('sim', path, opts);
assert(isempty(r.abort), r.abort);
assert(numel(r.captures) == 4, 'expected one capture per step');
S = fakeProbeLink('state');
ring = S.ring(:, 6) / 5000;
for c = 1:4
    cap = r.captures{c};
    assert(cap.step == c && numel(cap.volts) == nRec);
    assert(any(r.windows(:, 1) == c), 'no window during step %d', c);
    assert(exist(cap.file, 'file') == 2);
    assert(~isempty(findSegment(ring, cap.volts)), 'capture %d is not a run of the ring''s records', c);
end
for k = 1:size(r.windows, 1)
    assert(inRing(ring, r.windows(k, 3:6), nRec), 'window %d is not a second of the ring', k);
end
% The window rms holds any change of the pressure itself, so only the idle
% step's windows bound it from above.
idle = r.windows(r.windows(:, 1) == 1, 4);
assert(all(r.windows(:, 4) > 0.7 * sigma) && all(idle < 1.3 * sigma), ...
    'window rms %s V', mat2str(r.windows(:, 4)', 3));
assert(exist(r.runFile, 'file') == 2);
assert(r.exchangeFailures == 0 && r.probeFailures == 0);
fprintf('ok  four steps, a capture and windows in each, all found in the ring\n');

% A capture's records are the second after its arm: the condition of its
% step holds throughout.
log = r.runLog;
during = @(cap) log(log(:, 1) >= cap.tArm + 0.2 & log(:, 1) <= cap.tArm + 1, :);
d = during(r.captures{2});
assert(~isempty(d) && all(abs(d(:, 2) - 245) <= 0.5 & d(:, 5) == 3), 'position capture not at target');
d = during(r.captures{3});
assert(~isempty(d) && all(d(:, 3) > 1.5 & d(:, 5) == 4), 'jog capture not taken at speed');
d = during(r.captures{4});
assert(~isempty(d) && all(d(:, 5) == 5), 'pressure capture not taken in pressure control');
assert(d(end, 4) > d(1, 4), 'pressure not rising under pressure control');
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  captures taken at target, at speed and under pressure control\n');

% A drive fault during the jog ends the run; the stop goes out and the
% capture of that step is not taken.
opts = linkOpts(saveDir, sigma);
fakeProbeLink('faultAt', 4);
r = pressureNoiseRun('simfault', {{'jog', 2, 8}, {'jog', -2, 8}}, opts);
assert(~isempty(strfind(r.abort, 'status 10')), 'fault did not abort: %s', r.abort);
assert(numel(r.captures) <= 1 && r.runLog(end, 5) == 10);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.u.modeCode ~= 3, 'motion still commanded after the fault');
fprintf('ok  slave fault aborts the run\n');

fprintf('all passed\n');
end


function opts = linkOpts(saveDir, noise)
clear fakeProbeLink
fakeProbeLink('init');
fakeProbeLink('inputNoise', noise);
link.read     = @(start, n) fakeProbeLink('read', start, n);
link.write    = @(start, v) fakeProbeLink('write', start, v);
link.writeOne = @(addr, v) fakeProbeLink('writeOne', addr, v);
link.close    = @() [];
opts.link = link;
opts.now = @() fakeProbeLink('now');
opts.wait = @(t) fakeProbeLink('wait', t);
opts.safetyModel = '';
opts.settle = 1;
opts.saveDir = saveDir;
opts.report = false;
end


function j = findSegment(ring, seg)
% Index of the record where the run of records seg starts in the ring's log
% (the raw input in volts), [] when there is none.
seg = seg(:);
j = [];
for k = find(abs(ring(1:end - numel(seg) + 1) - seg(1)) < 1e-9)'
    if max(abs(ring(k:k + numel(seg) - 1) - seg)) < 1e-9
        j = k;
        return;
    end
end
end


function found = inRing(ring, stats, n)
% True when stats ([mean rms min max], volts) are those of n consecutive
% records of the ring's log.
found = false;
for k = 1:numel(ring) - n + 1
    if max(abs(pressureNoiseStats(ring(k:k + n - 1)) - stats)) < 1e-9
        found = true;
        return;
    end
end
end
