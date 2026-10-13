function stats = modbusBenchReport(benchLog, columns, dt)
%MODBUSBENCHREPORT Summarise a ModbusBench run.
%   stats = modbusBenchReport(benchLog)
%   stats = modbusBenchReport(benchLog, columns)
%   stats = modbusBenchReport(benchLog, columns, dt)
%
%   benchLog  the To Workspace array (out.benchLog), one row per model step.
%             A 1-by-N-by-steps array, as logged when a Mux input is a row
%             vector, is accepted too.
%   columns   cell array naming the Mux inputs in order. Known names:
%               seq      master step counter written to HR1
%               echo     IR1, heartbeat echo
%               counter  IR2, slave scan counter
%               txnMs    time spent in the write and read (ms)
%               stepMs   host time since the previous step (ms)
%               ok       1 when the exchange succeeded
%               missed   Missed Ticks port of a DAQ block (lag in ticks)
%             Other names are ignored. Default: the layout of the generated
%             model, {'seq','echo','counter','txnMs','stepMs','ok'}.
%   dt        model step (s) if it differs from cfg.dt.
%
%   Sections are printed for the columns present. Without txnMs the log is
%   taken to come from Modbus blocks sampled slower than the model step, and
%   the heartbeat and scan sections use one row per exchange (rows where the
%   read values change).

if nargin < 2 || isempty(columns)
    columns = {'seq', 'echo', 'counter', 'txnMs', 'stepMs', 'ok'};
end
cfg = modbusBenchConfig();
if nargin >= 3 && ~isempty(dt)
    cfg.dt = dt;
end
dtMs = cfg.dt * 1000;

% Steps-by-columns, whatever To Workspace produced.
if ndims(benchLog) == 3
    nSteps = size(benchLog, 3);
    benchLog = reshape(permute(benchLog, [3 1 2]), nSteps, []);
end
if size(benchLog, 2) ~= numel(columns)
    error('benchLog has %d columns but %d names were given', ...
        size(benchLog, 2), numel(columns));
end
col = @(name) benchLog(cfg.skip + 1:end, strcmp(columns, name));
has = @(name) any(strcmp(columns, name));
if ~has('seq')
    error('a seq column is required');
end

seq = col('seq');
n = numel(seq);
fprintf('\nModbusBench: %d steps of %.1f ms (%.1f s), first %d discarded\n', ...
    n, dtMs, n * cfg.dt, cfg.skip);
stats.steps = n;

% Pacing: under Connected IO the host step interval centres on dt.
if has('stepMs')
    stepMs = col('stepMs');
    fprintf('\nModel pacing (host time between steps)\n');
    printStats('  step interval     ', stepMs);
    if median(stepMs) < 0.5 * dtMs
        fprintf('  WARNING: steps run faster than real time; the model is not paced by\n');
        fprintf('  Simulink Desktop Real-Time. Run it in Connected IO mode.\n');
    end
    stats.missedTicksEstimate = sum(max(0, round(stepMs / dtMs) - 1));
    fprintf('  missed ticks (estimated from step interval): %d\n', stats.missedTicksEstimate);
    stats.stepMs = stepMs;
end

% Missed Ticks port: how far the model lags the kernel at each step.
if has('missed')
    missed = col('missed');
    fprintf('\nMissed ticks (DAQ block port, lag behind the kernel)\n');
    fprintf('  max %d; steps lagging: %.2f %%; lagging > 5: %.2f %%\n', ...
        max(missed), 100 * mean(missed > 0), 100 * mean(missed > 5));
    fprintf('  (the model stops when the lag exceeds the block''s Maximum missed ticks, default 10)\n');
    stats.missedMax = max(missed);
    stats.missed = missed;
end

% One row per exchange when the Modbus blocks run slower than the step: a
% row where the read values changed.
if has('txnMs')
    ex = true(n, 1);
else
    ex = false(n, 1);
    for name = {'echo', 'counter'}
        if has(name{1})
            ex = ex | [true; diff(col(name{1})) ~= 0];
        end
    end
    period = n / sum(ex);
    fprintf('\n%d exchanges, one per %.1f steps (%.1f ms)\n', ...
        sum(ex), period, period * dtMs);
end
stats.exchanges = sum(ex);

% Transactions
if has('txnMs') && has('ok')
    txnMs = col('txnMs');
    ok = col('ok') == 1;
    fprintf('\nModbus write + read\n');
    fprintf('  failed: %d of %d (%.3f %%)\n', sum(~ok), n, 100 * mean(~ok));
    printStats('  transaction       ', txnMs(ok));
    fprintf('  longer than one step (%.1f ms): %.2f %%\n', dtMs, 100 * mean(txnMs(ok) > dtMs));
    stats.failures = sum(~ok);
    stats.txnMs = txnMs(ok);
else
    ok = true(n, 1);
end
sel = ex & ok;

% Freshness: echo lag in exchanges.
if has('echo')
    echo = col('echo');
    lag = mod(seq(sel) - echo(sel), 65536);
    fprintf('\nHeartbeat echo lag (steps between the seq written and the seq read back)\n');
    for k = 0:2
        fprintf('  %d: %.2f %%\n', k, 100 * mean(lag == k));
    end
    fprintf('  >2: %.2f %%  (max %d)\n', 100 * mean(lag > 2), max(lag));
    stats.lag = lag;
end

% Slave scans between successful reads.
if has('counter')
    counter = col('counter');
    scans = mod(diff(counter(sel)), 65536);
    fprintf('\nSlave scans between successful reads\n');
    fprintf('  median %g, max %g, repeated data (0 scans): %d\n', ...
        median(scans), max(scans), sum(scans == 0));
    stats.scans = scans;
end
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
