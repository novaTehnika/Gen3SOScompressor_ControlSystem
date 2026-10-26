function r = stepResponseFit(files, opts)
%STEPRESPONSEFIT Read the scan-rate captures of frictionRun and report the
%   drive's velocity-loop lag, the breakaway and the torque ripple.
%
%   r = stepResponseFit(files)
%   r = stepResponseFit(files, opts)
%
%   files        a capture file, a pattern ('captures/s1_cap_*.csv') or a
%                cell array of either
%   opts.accel, opts.decel
%                slew of the velocity command on the slave, mm/s^2
%                (default G_cfgDirectCtrlAccel from frictionBenchConfig)
%   opts.quiet   no printing
%
%   A capture holds about a second of the commanded velocity, the actual
%   velocity, the torque and the position every scan. Each is classed by
%   what the command did in it:
%
%     moving     the command changed while the axis was moving. The
%                drive's response is modelled as the command slewed at
%                accel or decel (the profile Y_DirectControl generates),
%                passed through a first-order lag tauV and a delay td,
%                eq. (11) and (12) of design/pressure-loop/README.md. tauV
%                and td are found by a grid search on the rms velocity
%                error over the capture. The tracking lag is also measured
%                without the model, as the time between the command
%                profile and the velocity crossing the middle of the step.
%     from rest  the command stepped from a standing axis. The breakaway
%                is the largest friction force (torque force less the
%                pressure force, which these captures do not carry, so
%                the torque force itself), averaged over five scans,
%                before the first scan moving faster than vMin, and the
%                stick time is how long that took. No lag is fitted: the
%                stick hides it.
%     steady     no command change. The ripple is the standard deviation
%                of the force and its amplitude at the screw lead period
%                and at the motor turn, from a Hann-windowed spectrum.
%
%   r.transitions, r.steady hold the per-capture results and r.tauV,
%   r.td the medians over the moving captures, printed as the params.py
%   value.

if nargin < 2
    opts = struct();
end
if ~isfield(opts, 'quiet')
    opts.quiet = false;
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..', 'master-src', 'logic'));
cfg = frictionBenchConfig();
if ~isfield(opts, 'accel')
    opts.accel = cfg.accel;
end
if ~isfield(opts, 'decel')
    opts.decel = cfg.accel;
end
say = @(varargin) printUnless(opts.quiet, varargin{:});

files = expandFiles(files);
if isempty(files)
    error('no capture files found');
end
k = cfg.newtonPerPct;
lead = cfg.leadMm;

trans = [];
steady = [];
for f = 1:numel(files)
    d = dlmread(files{f}, ',', 1, 0);
    if size(d, 2) ~= 5
        error('%s: %d columns, expected t,vCmd,velocity,torque,position', files{f}, size(d, 2));
    end
    t = d(:, 1); vCmd = d(:, 2); v = d(:, 3); F = d(:, 4) * k; x = d(:, 5);
    dt = median(diff(t));
    N = numel(t);
    [~, name] = fileparts(files{f});
    ch = find(abs(diff(vCmd)) > 1e-3) + 1;
    if isempty(ch)
        s.file = name;
        s.v = mean(v);
        s.vPos = (x(end) - x(1)) / (t(end) - t(1));
        s.F = mean(F);
        s.Fstd = std(F);
        s.fLead = abs(s.v) / lead;
        s.fMotor = s.fLead * cfg.gearRatio;
        s.ampLead = lineAmp(F, t, s.fLead, dt);
        s.ampMotor = lineAmp(F, t, s.fMotor, dt);
        steady = [steady s]; %#ok<AGROW>
        continue;
    end
    i0 = ch(1);
    if numel(ch) > 1
        i1 = ch(2) - 1;
    else
        i1 = N;
    end
    c.file = name;
    c.v0 = vCmd(i0 - 1);
    c.v1 = vCmd(i0);
    c.fromRest = mean(abs(v(max(1, i0 - 10):i0 - 1))) < cfg.vMin;
    c.tauV = NaN; c.td = NaN; c.rms = NaN; c.tTrack = NaN; c.overshoot = NaN;
    c.Fbreak = NaN; c.tStick = NaN; c.nBreak = NaN;
    if c.fromRest
        ib = find(abs(v(i0:i1)) > cfg.vMin, 1, 'first');
        if isempty(ib)
            ib = i1 - i0 + 1;
        end
        win = i0:i0 + ib - 1;
        Fs = conv(F(win), ones(5, 1) / 5, 'same');    % 10 ms average against single-sample noise
        [~, j] = max(abs(Fs));
        c.Fbreak = Fs(j);
        c.nBreak = numel(win);
        c.tStick = (ib - 1) * dt;
    else
        vRef = profile(vCmd, i0, c.v0, opts.accel, opts.decel, dt);
        win = max(1, i0 - 10):i1;
        [c.tauV, c.td, c.rms] = fitLag(vRef, v, c.v0, win, dt);
        c.tTrack = trackLag(vRef, v, t, c.v0, c.v1, i0, i1);
        after = find(abs(vRef - c.v1) < 1e-6 & (1:N)' >= i0, 1, 'first');
        if ~isempty(after)
            c.overshoot = max(sign(c.v1 - c.v0) * (v(after:i1) - c.v1)) / abs(c.v1 - c.v0) * 100;
        end
    end
    trans = [trans c]; %#ok<AGROW>
end
r.transitions = trans;
r.steady = steady;

% --- Report -------------------------------------------------------------------
say('\nvelocity step responses (command slew %g / %g mm/s^2):\n', opts.accel, opts.decel);
say('%-32s %6s %6s %8s %7s %7s %8s %8s %9s %7s\n', 'capture', 'v0', 'v1', 'tauV ms', 'td ms', ...
    'rms', 'track ms', 'over %', 'break N', 'stick s');
for c = trans
    if c.fromRest
        say('%-32s %6.2f %6.2f %8s %7s %7s %8s %8s %9.0f %7.3f\n', c.file, c.v0, c.v1, '-', '-', ...
            '-', '-', '-', c.Fbreak, c.tStick);
    else
        say('%-32s %6.2f %6.2f %8.1f %7.1f %7.3f %8.1f %8.1f %9s %7s\n', c.file, c.v0, c.v1, ...
            c.tauV * 1000, c.td * 1000, c.rms, c.tTrack * 1000, c.overshoot, '-', '-');
    end
end
say('rms in mm/s over the capture; track is the model-free lag at mid-step\n');
if ~isempty(steady)
    say('\nsteady captures:\n%-32s %7s %7s %9s %9s %14s %14s\n', 'capture', 'v', 'v pos', ...
        'F N', 'ripple N', 'lead Hz / N', 'motor Hz / N');
    for s = steady
        say('%-32s %7.3f %7.3f %9.0f %9.0f %6.2f / %6.0f %6.1f / %6.0f\n', s.file, s.v, s.vPos, ...
            s.F, s.Fstd, s.fLead, s.ampLead, s.fMotor, s.ampMotor);
    end
end

moving = trans(arrayfun(@(c) ~c.fromRest, trans));
rest = trans(arrayfun(@(c) c.fromRest, trans));
r.tauV = NaN; r.td = NaN; r.Fbreak = NaN; r.tStick = NaN;
if ~isempty(moving)
    r.tauV = median([moving.tauV]);
    r.td = median([moving.td]);
end
if ~isempty(rest)
    r.Fbreak = mean(abs([rest.Fbreak]));
    r.tStick = mean([rest.tStick]);
end
say('\ndesign/pressure-loop/params.py values (SI):\n');
if ~isnan(r.tauV)
    say('    tauV = %-10.4g  # [s] drive velocity loop lag, median of %d steps (delay %.1f ms besides)\n', ...
        r.tauV, numel(moving), r.td * 1000);
else
    say('    # tauV: no step taken while moving; run a stepv step\n');
end
if ~isnan(r.Fbreak)
    say('    # breakaway %.0f N from rest at the scan rate, stick %.3f s, mean of %d starts\n', ...
        r.Fbreak, r.tStick, numel(rest));
end
end


% ===========================================================================
function vRef = profile(vCmd, i0, v0, accel, decel, dt)
% The slewed command: toward vCmd at accel while its magnitude grows and
% decel while it shrinks or reverses, from v0 at the step.
N = numel(vCmd);
vRef = v0 * ones(N, 1);
for j = i0:N
    target = vCmd(j);
    if abs(target) > abs(vRef(j - 1))
        rate = accel;
    else
        rate = decel;
    end
    vRef(j) = vRef(j - 1) + min(max(target - vRef(j - 1), -rate * dt), rate * dt);
end
end


function [tauV, td, best] = fitLag(vRef, v, v0, win, dt)
% Grid search of the first-order lag and the delay on the rms error.
taus = [0.001:0.001:0.01 0.012:0.002:0.05 0.055:0.005:0.2];
delays = 0:30;
best = Inf;
tauV = NaN;
td = NaN;
u = vRef - v0;
for tau = taus
    alpha = dt / (tau + dt);
    y = filter(alpha, [1, -(1 - alpha)], u);
    for nd = delays
        yd = [zeros(nd, 1); y(1:end - nd)] + v0;
        e = sqrt(mean((yd(win) - v(win)).^2));
        if e < best
            best = e;
            tauV = tau;
            td = nd * dt;
        end
    end
end
end


function tTrack = trackLag(vRef, v, t, v0, v1, i0, i1)
% Time between the command profile and the velocity crossing mid-step.
mid = (v0 + v1) / 2;
s = sign(v1 - v0);
a = find(s * (vRef(i0:i1) - mid) >= 0, 1, 'first');
b = find(s * (v(i0:i1) - mid) >= 0, 1, 'first');
if isempty(a) || isempty(b)
    tTrack = NaN;
else
    tTrack = t(i0 + b - 1) - t(i0 + a - 1);
end
end


function amp = lineAmp(F, t, f, dt)
% Hann-windowed amplitude of the force at frequency f, NaN beyond the
% capture's resolution or Nyquist.
N = numel(F);
if f <= 1 / (t(end) - t(1)) || f >= 0.5 / dt
    amp = NaN;
    return;
end
n = (0:N - 1)';
w = 0.5 - 0.5 * cos(2 * pi * n / (N - 1));
amp = 2 / sum(w) * abs(sum(w .* (F - mean(F)) .* exp(-2i * pi * f * (t - t(1)))));
end


function files = expandFiles(arg)
files = {};
if isempty(arg)
    return;
end
if ischar(arg)
    arg = {arg};
end
for k = 1:numel(arg)
    d = dir(arg{k});
    d = d(~[d.isdir]);
    if isempty(d) && exist(arg{k}, 'file') == 2
        files{end + 1} = arg{k}; %#ok<AGROW>
        continue;
    end
    for j = 1:numel(d)
        files{end + 1} = fullfile(d(j).folder, d(j).name); %#ok<AGROW>
    end
end
end


function printUnless(quiet, varargin)
if ~quiet
    fprintf(varargin{:});
end
end
