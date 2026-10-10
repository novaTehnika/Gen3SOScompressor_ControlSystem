function stats = modbusBenchReport(benchLog)
%MODBUSBENCHREPORT Summarise a ModbusBench run.
%   stats = modbusBenchReport(benchLog)
%
%   benchLog  rows of [seq echo slaveCounter txnMs stepMs ok], as logged by
%             the ModbusBench model (out.benchLog after out = sim('ModbusBench'))

cfg = modbusBenchConfig();
dtMs = cfg.dt * 1000;

log = benchLog(cfg.skip + 1:end, :);
seq = log(:, 1);
echo = log(:, 2);
counter = log(:, 3);
txnMs = log(:, 4);
stepMs = log(:, 5);
ok = log(:, 6) == 1;

n = numel(seq);
fprintf('\nModbusBench: %d steps of %.1f ms (%.1f s), first %d discarded\n', ...
    n, dtMs, n * cfg.dt, cfg.skip);

% Pacing: under Connected IO the host step interval centres on dt.
fprintf('\nModel pacing (host time between steps)\n');
printStats('  step interval     ', stepMs);
if median(stepMs) < 0.5 * dtMs
    fprintf('  WARNING: steps run faster than real time; the model is not paced by\n');
    fprintf('  Simulink Desktop Real-Time. Run it in Connected IO mode.\n');
end
missed = sum(max(0, round(stepMs / dtMs) - 1));
fprintf('  missed ticks (estimated): %d\n', missed);

% Transactions
fprintf('\nModbus write + read\n');
fprintf('  failed: %d of %d (%.3f %%)\n', sum(~ok), n, 100 * mean(~ok));
printStats('  transaction       ', txnMs(ok));
fprintf('  longer than one step (%.1f ms): %.2f %%\n', dtMs, 100 * mean(txnMs(ok) > dtMs));

% Freshness: echo lag in steps and slave scans between reads.
lag = mod(seq(ok) - echo(ok), 65536);
fprintf('\nHeartbeat echo lag (steps)\n');
for k = 0:2
    fprintf('  %d: %.2f %%\n', k, 100 * mean(lag == k));
end
fprintf('  >2: %.2f %%  (max %d)\n', 100 * mean(lag > 2), max(lag));

okIdx = find(ok);
scans = mod(diff(counter(okIdx)), 65536);
fprintf('\nSlave scans between successful reads\n');
fprintf('  median %g, max %g, repeated data (0 scans): %d\n', ...
    median(scans), max(scans), sum(scans == 0));

stats.steps = n;
stats.missedTicks = missed;
stats.failures = sum(~ok);
stats.txnMs = txnMs(ok);
stats.stepMs = stepMs;
stats.lag = lag;
stats.scans = scans;
end


function printStats(label, x)
if isempty(x)
    fprintf('%s no data\n', label);
    return;
end
x = sort(x);
pct = @(p) x(max(1, ceil(p / 100 * numel(x))));
fprintf('%s median %6.2f  p95 %6.2f  p99 %6.2f  max %6.2f ms\n', ...
    label, pct(50), pct(95), pct(99), x(end));
end
