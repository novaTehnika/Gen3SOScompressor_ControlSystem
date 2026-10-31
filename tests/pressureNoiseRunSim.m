function pressureNoiseRunSim()
%PRESSURENOISERUNSIM Run bench/pressure/pressureNoiseRun against the
%   simulated slave through fakeProbeLink: the path is sequenced through
%   masterStep, every step yields its windows and a capture taken while the
%   step's condition holds, and a slave fault ends the run with the axis
%   stopped.
%   Run from the repository root after  addpath slave-sim; buildSlaveSim
%     addpath master-src/logic slave-sim slave-sim/generated tests
%   Depends on: master-src/logic slave-src slave-sim tools models bench/pressure bench/common tests/fakeProbeLink.m

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'bench', 'pressure'), fullfile(here, '..', 'bench', 'common'));
saveDir = tempname();

opts = linkOpts(saveDir);
path = {{'idle', 1}, {'position', 245, 1}, {'jog', 2, 2}, {'pressure', 2, 3}};
r = pressureNoiseRun('sim', path, opts);
assert(isempty(r.abort), r.abort);
assert(numel(r.captures) == 4, 'expected one capture per step');
for c = 1:4
    cap = r.captures{c};
    assert(cap.step == c && numel(cap.volts) == 250);
    assert(any(r.windows(:, 1) == c & r.windows(:, 2) == 1), 'no window during step %d', c);
    assert(exist(cap.file, 'file') == 2);
end
assert(exist(r.runFile, 'file') == 2);
assert(r.exchangeFailures == 0 && r.probeFailures == 0);
fprintf('ok  four steps, a capture and windows in each\n');

log = r.runLog;
cap = r.captures{2};
at = log(find(log(:, 1) >= cap.tRead, 1), :);
assert(abs(at(2) - 245) <= 0.5 && at(5) == 3, 'position capture not at target');
cap = r.captures{3};
at = log(find(log(:, 1) >= cap.tRead, 1), :);
assert(at(3) > 1.5 && at(5) == 4, 'jog capture not taken at speed (%.2f mm/s)', at(3));
cap = r.captures{4};
during = log(log(:, 1) >= cap.tArm & log(:, 1) <= cap.tRead, :);
assert(all(during(:, 5) == 5), 'pressure capture not taken in pressure control');
assert(at(4) < during(end, 4), 'pressure not rising under pressure control');
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.sl.ax.v == 0, 'axis not stopped at the end');
fprintf('ok  captures taken at target, at speed and under pressure control\n');

% A drive fault during the jog ends the run; the stop goes out and the
% capture of that step is not taken.
opts = linkOpts(saveDir);
fakeProbeLink('faultAt', 4);
r = pressureNoiseRun('simfault', {{'jog', 2, 8}, {'jog', -2, 8}}, opts);
assert(~isempty(strfind(r.abort, 'status 10')), 'fault did not abort: %s', r.abort);
assert(numel(r.captures) <= 1 && r.runLog(end, 5) == 10);
S = fakeProbeLink('state');
assert(S.u.motionEnable == 0 && S.u.modeCode ~= 3, 'motion still commanded after the fault');
fprintf('ok  slave fault aborts the run\n');

fprintf('all passed\n');
end


function opts = linkOpts(saveDir)
clear fakeProbeLink
fakeProbeLink('init');
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
