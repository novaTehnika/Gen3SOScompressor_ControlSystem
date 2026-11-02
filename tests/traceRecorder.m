function traceRecorder()
%TRACERECORDER Run traceStep, traceStart and traceDrain against a
%   synthetic slave stand-in and assert the record layout, sampling,
%   ring addressing, the drain at irregular poll intervals, overrun
%   accounting, recording through link loss and the drain's robustness
%   to unusable and torn rings.
%
%   The stand-in answers every second step with a new input-register
%   vector (scan counter + 1, the heartbeat echoed, smooth position,
%   velocity, torque and pressure) and holds it in between, as the
%   Modbus Read block does. Each 25 ms step calls masterStep, then
%   traceStep with its outputs, as the shell does.
%
%   Run from the repository root:
%     addpath master-src/logic master-src/trace tests; traceRecorder
%   Depends on: master-src/logic master-src/trace

clear masterStep masterStateMachine traceStep

L = traceLayout();
cfg = masterConfig();

% Layout.
names = traceColumnNames();
fields = fieldnames(L.col);
idx = zeros(numel(fields), 1);
for k = 1:numel(fields)
    idx(k) = L.col.(fields{k});
end
assert(isequal(sort(idx(:)'), 1:L.nCol));
assert(L.width == L.nCol + 1 && L.guard == L.width);
assert(numel(names) == L.nCol && numel(unique(names)) == L.nCol);
for k = 1:L.nCol
    assert(isvarname(names{k}));
    assert(L.col.(names{k}) == k);
end
fprintf('ok  layout\n');

% Sampling and phase, with the ring wrapped.
h = newRig();
h = runPolled(h, [], 2000, false);
ring = h.ring;
assert(isequal(size(ring), [L.depth L.width]));
seq = ring(:, L.col.seq);
assert(isequal(sort(seq)', 489:1000));
assert(isequal(ring(:, L.guard), seq));
for i = 1:L.depth
    assert(i == mod(seq(i) - 1, L.depth) + 1);
    step = (seq(i) - 1) * 2;
    t = ring(i, L.col.t_s);
    assert(abs(t / 0.05 - round(t / 0.05)) < 1e-9);
    assert(abs(t - step * cfg.dt) < 1e-9);
    assert(ring(i, L.col.heartbeat) == h.logW(step + 1, 1));
    assert(ring(i, L.col.statusCode) == h.logO(step + 1, 2));
    pos = modbusUnpack(h.logR(step + 1, 11), h.logR(step + 1, 12));
    assert(ring(i, L.col.position_mm) == pos);
    assert(pos == double(single(100 + 0.5 * step * cfg.dt)));
    assert(isequal(ring(i, 1:L.nCol), expectedRecord(h, step, seq(i))));
end
fprintf('ok  sampling, phase and ring addressing\n');

% Irregular drain.
clear masterStep masterStateMachine traceStep
h = newRig();
h = runPolled(h, [], 100, false);
buf = traceStart(h.ring);
assert(buf.lastSeq == 50 && buf.n == 0 && buf.missed == 0);
assert(isequal(size(buf.gaps), [0 2]) && size(buf.data, 2) == L.nCol);
h = runPolled(h, buf, 1900, true);
buf = h.buf;
assert(buf.n > 800 && buf.missed == 0 && isempty(buf.gaps));
d = buf.data(1:buf.n, :);
assert(isequal(d(:, L.col.seq)', 51:50 + buf.n));
assert(all(abs(diff(d(:, L.col.t_s)) - 0.05) < 1e-9));
assert(numel(unique(d(:, L.col.seq))) == buf.n);
assert(buf.lastSeq == d(end, L.col.seq));
assert(isequal(d, expectedRecords(h, d(:, L.col.seq))));
assert(numel(unique(h.pollGaps)) > 10);
fprintf('ok  irregular drain, %d records in %d polls\n', buf.n, numel(h.pollGaps));

% Overrun: one poll gap longer than the ring holds.
clear masterStep masterStateMachine traceStep
h = newRig();
h = runPolled(h, [], 100, false);
h = runPolled(h, traceStart(h.ring), 1000, true);
q = h.buf.lastSeq;
h.nextPoll = h.lastDrainK + 2 * (L.depth + 50);
h = runPolled(h, h.buf, h.nextPoll + 300, true);
buf = h.buf;
assert(buf.missed == 50);
assert(isequal(buf.gaps, [q + 51, 50]));
d = buf.data(1:buf.n, :);
s = d(:, L.col.seq);
jump = find(diff(s) ~= 1);
assert(isscalar(jump) && s(jump + 1) - s(jump) == 51 && s(jump) == q);
assert(s(jump + 1) == q + 51);
assert(all(abs(diff(d(1:jump, L.col.t_s)) - 0.05) < 1e-9));
assert(all(abs(diff(d(jump + 1:end, L.col.t_s)) - 0.05) < 1e-9));
assert(abs(d(jump + 1, L.col.t_s) - d(jump, L.col.t_s) - 51 * 0.05) < 1e-9);
assert(isequal(d, expectedRecords(h, s)));
fprintf('ok  overrun: 50 records missed, one gap at seq %d\n', q + 51);

% Link loss: the slave counter stops for 1 s.
clear masterStep masterStateMachine traceStep
h = newRig();
h.freezeFrom = 400;
h.freezeTo = 440;
h = runPolled(h, traceStart(h.ring), 800, true);
buf = h.buf;
assert(buf.missed == 0);
d = buf.data(1:buf.n, :);
assert(all(abs(diff(d(:, L.col.t_s)) - 0.05) < 1e-9));
step = round(d(:, L.col.t_s) / cfg.dt);
assert(all(d(step >= 8 & step <= 408, L.col.linkOk) == 1));
assert(all(d(step >= 8 & step <= 408, L.col.doSafety) == 1));
first = step(find(d(:, L.col.statusCode) == 34, 1));
assert(first >= 410 && first <= 412);
during = step >= 412 & step <= 438;
assert(any(during));
assert(all(d(during, L.col.statusCode) == 34));
assert(all(d(during, L.col.linkOk) == 0));
assert(all(d(during, L.col.doSafety) == 0));
frozen = step >= 400 & step < 440;
assert(all(d(frozen, L.col.slaveCounter) == d(find(frozen, 1) - 1, L.col.slaveCounter)));
assert(nnz(frozen) == 20);
fprintf('ok  recording continues through link loss\n');

% Robustness of start and drain.
ringA = makeRing(60);
bufA = traceStart(ringA);
assert(bufA.lastSeq == 60 && bufA.n == 0);
assert(isequal(traceDrain(bufA, 0), bufA));
assert(isequal(traceDrain(bufA, zeros(10, L.width)), bufA));
assert(isequal(traceDrain(bufA, ones(L.depth, L.width - 1)), bufA));
assert(isequal(traceDrain(bufA, ringA), bufA));
assert(traceStart(0).lastSeq == 0);
assert(traceStart(zeros(3, 3)).lastSeq == 0);
assert(traceStart(zeros(L.depth, L.width)).lastSeq == 0);

ringB = makeRing(70);
b1 = traceDrain(bufA, ringB);
b2 = traceDrain(bufA, ringB(:));               % column-vector Data
assert(isequal(b1, b2) && b1.n == 10 && b1.lastSeq == 70);
assert(isequal(b1.data(1:10, :), expectedMakeRing(61:70)));

% A row whose guard differs from its record number in the middle holds
% back the records after it; nothing counts as missed.
row = mod(65 - 1, L.depth) + 1;
torn = ringB;
torn(row, L.guard) = 0;
b1 = traceDrain(bufA, torn);
assert(b1.n == 4 && b1.lastSeq == 64 && b1.missed == 0 && isempty(b1.gaps));
b1 = traceDrain(b1, ringB);
assert(b1.n == 10 && b1.lastSeq == 70 && b1.missed == 0 && isempty(b1.gaps));
assert(isequal(b1.data(1:10, :), expectedMakeRing(61:70)));

% The newest row torn: it is taken at the next drain.
ringC = makeRing(80);
torn = ringC;
row = mod(80 - 1, L.depth) + 1;
torn(row, L.guard) = 79;
b1 = traceDrain(b1, torn);
assert(b1.n == 19 && b1.lastSeq == 79 && b1.missed == 0);
b1 = traceDrain(b1, ringC);
assert(b1.n == 20 && b1.lastSeq == 80 && b1.missed == 0 && isempty(b1.gaps));
assert(isequal(b1.data(1:20, :), expectedMakeRing(61:80)));

% A torn row is not the start's newest record.
torn = makeRing(30);
torn(mod(30 - 1, L.depth) + 1, L.guard) = 29;
assert(traceStart(torn).lastSeq == 29);

% The first record is taken from an all-zero start.
b0 = traceStart(zeros(L.depth, L.width));
b0 = traceDrain(b0, makeRing(5));
assert(b0.n == 5 && b0.missed == 0 && b0.lastSeq == 5 && isempty(b0.gaps));
assert(isequal(b0.data(1:5, :), expectedMakeRing(1:5)));
fprintf('ok  robustness: unusable ring, column vector, torn rows, empty start\n');

% Growth of the buffer.
b0 = traceStart(zeros(L.depth, L.width));
for last = L.depth:L.depth:6 * L.depth
    b0 = traceDrain(b0, makeRing(last));
end
assert(b0.n == 6 * L.depth && b0.missed == 0 && isempty(b0.gaps));
assert(size(b0.data, 1) == 4096);
assert(isequal(b0.data(1:b0.n, :), expectedMakeRing(1:b0.n)));
fprintf('ok  buffer grows by doubling\n');
end


% ---------------- stand-in and helpers ----------------

function h = newRig()
%NEWRIG State of the stand-in slave and the logs of one run.
nMax = 4000;
h.k = 0;                            % next step
h.counter = 100;
h.lastHb = 0;
h.mbRead = zeros(1, 20);
h.ring = zeros(512, 35);
h.freezeFrom = Inf;
h.freezeTo = Inf;
h.logR = zeros(nMax, 20);           % mbRead of each step
h.logW = zeros(nMax, 12);           % mbWrite
h.logO = zeros(nMax, 11);           % masterStep outputs and app values
h.logT = zeros(nMax, 5);            % stand-in's true feedback
h.buf = [];
h.nextPoll = 0;
h.pollGaps = [];
h.lastDrainK = 0;
end

function h = stepRig(h)
%STEPRIG One 25 ms step: stand-in, masterStep, traceStep.
cfg = masterConfig();
k = h.k;
t = k * cfg.dt;
op = 0;
cmdSeq = 0;
if k >= 300
    op = 2;
    cmdSeq = 1;
end
if k >= 1500
    op = 0;
    cmdSeq = 2;
end
targetPosition = 50;
targetPressure = 5;
estop = 0;
mixer = double(k >= 200);

truth = [100 + 0.5 * t, 0.5, 10 * sin(t), 100 + 0.5 * t, 2 + sin(t)];
if mod(k, 2) == 0
    frozen = k >= h.freezeFrom && k < h.freezeTo;
    if ~frozen
        h.counter = mod(h.counter + 1, 65536);
    end
    h.mbRead = [h.lastHb, h.counter, 0, 0, 0, double(mod(floor(t), 2) == 1), ...
        1, 1, 0, 0, modbusPack(truth(1)), modbusPack(truth(2)), ...
        modbusPack(truth(3)), modbusPack(truth(4)), modbusPack(truth(5))];
end

[mbWrite, doSafety, ~, ~, ~, ~, statusCode, faultCode, state, linkOk] = ...
    masterStep(h.mbRead, op, cmdSeq, targetPosition, 0.5, 1, targetPressure, ...
    estop, mixer, cfg.positionSpeed, cfg.salineVolume, cfg.oxygenRatio);
h.lastHb = mbWrite(1);
h.ring = traceStep(h.mbRead, mbWrite, doSafety, statusCode, faultCode, ...
    state, linkOk, op, cmdSeq, targetPosition, targetPressure, estop, mixer);

h.logR(k + 1, :) = h.mbRead;
h.logW(k + 1, :) = mbWrite;
h.logO(k + 1, :) = [doSafety, statusCode, faultCode, state, linkOk, op, ...
    cmdSeq, targetPosition, targetPressure, estop, mixer];
h.logT(k + 1, :) = truth;
h.k = k + 1;
end

function h = runPolled(h, buf, untilK, drain)
%RUNPOLLED Step until step count untilK; when drain is true, drain the
%   ring into h.buf at irregular intervals of 1..40 steps (0.025-1 s).
if drain
    h.buf = buf;
end
while h.k < untilK
    h = stepRig(h);
    if drain && h.k >= h.nextPoll
        h.buf = traceDrain(h.buf, h.ring);
        j = numel(h.pollGaps) + 1;
        g = 1 + mod(round(1e4 * abs(sin(j))), 40);
        h.pollGaps(j) = g;
        h.lastDrainK = h.k;
        h.nextPoll = h.k + g;
    end
end
end

function e = expectedRecord(h, step, seq)
%EXPECTEDRECORD Record of a step built from the logged inputs, by column.
L = traceLayout();
r = h.logR(step + 1, :);
w = h.logW(step + 1, :);
o = h.logO(step + 1, :);
T = h.logT(step + 1, :);
cfg = masterConfig();
e = zeros(1, L.nCol);
e(L.col.seq) = seq;
e(L.col.t_s) = step * cfg.dt;
e(L.col.heartbeat) = w(1);
e(L.col.heartbeatEcho) = r(1);
e(L.col.slaveCounter) = r(2);
e(L.col.confCode) = r(3);
e(L.col.slaveFaultCode) = r(4);
e(L.col.faultActive) = double(r(5) ~= 0);
e(L.col.inMotion) = double(r(6) ~= 0);
e(L.col.performanceStatus) = double(r(7) ~= 0);
e(L.col.homingComplete) = double(r(8) ~= 0);
e(L.col.position_mm) = double(single(T(1)));
e(L.col.velocity_mm_s) = double(single(T(2)));
e(L.col.torque_pct) = double(single(T(3)));
e(L.col.commandedPosition_mm) = double(single(T(4)));
e(L.col.pressureGauge_MPa) = double(single(T(5)));
e(L.col.modeCode) = w(2);
e(L.col.motionEnable) = w(3);
e(L.col.faultReset) = w(4);
e(L.col.reference) = modbusUnpack(w(5), w(6));
e(L.col.positionSpeed_mm_s) = modbusUnpack(w(7), w(8));
e(L.col.salineVolume_mL) = modbusUnpack(w(9), w(10));
e(L.col.oxygenRatio) = modbusUnpack(w(11), w(12));
e(L.col.statusCode) = o(2);
e(L.col.faultCode) = o(3);
e(L.col.state) = o(4);
e(L.col.linkOk) = o(5);
e(L.col.doSafety) = o(1);
e(L.col.op) = o(6);
e(L.col.cmdSeq) = o(7);
e(L.col.targetPosition_mm) = o(8);
e(L.col.targetPressure_atm) = o(9);
e(L.col.estop) = o(10);
e(L.col.mixer) = o(11);
end

function e = expectedRecords(h, seq)
%EXPECTEDRECORDS Records for the record numbers in seq (a column).
L = traceLayout();
e = zeros(numel(seq), L.nCol);
for i = 1:numel(seq)
    e(i, :) = expectedRecord(h, (seq(i) - 1) * 2, seq(i));
end
end

function ring = makeRing(last)
%MAKERING Ring holding the newest records up to number last, each row
%   filled by expectedMakeRing.
L = traceLayout();
ring = zeros(L.depth, L.width);
for s = max(1, last - L.depth + 1):last
    ring(mod(s - 1, L.depth) + 1, :) = [expectedMakeRing(s), s];
end
end

function e = expectedMakeRing(seq)
%EXPECTEDMAKERING Data columns of the synthetic record numbered seq (a
%   vector): the number in column 1, distinct offsets in the others.
L = traceLayout();
seq = seq(:);
e = repmat(seq, 1, L.nCol) + repmat((0:L.nCol - 1) / 100, numel(seq), 1);
e(:, L.col.seq) = seq;
end
