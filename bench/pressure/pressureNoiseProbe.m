function r = pressureNoiseProbe(label, nWindows)
%PRESSURENOISEPROBE Read the transducer noise statistics and a raw 2 ms
%   capture from PRG_PressureProbe, report them, and save the capture to
%   captures/<label>_<yyyymmdd_HHMMSS>.csv. Runs from the MATLAB command
%   line on the lab PC (Industrial Communication Toolbox, no Simulink).
%
%   r = pressureNoiseProbe('idle')        5 one-second windows, one capture
%   r = pressureNoiseProbe('jog', 10)     10 windows
%
%   Registers (1-based) as declared in slave/PRG_PressureProbe.st.

if nargin < 1
    label = 'run';
end
if nargin < 2
    nWindows = 5;
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'modbus'));
cfg = modbusBenchConfig();

hrArm      = 101;       % G_mbProbeArm
irStats    = 101;       % IR101-108 mean, rms, min, max (REAL, low word first)
irWindow   = 109;       % G_mbProbeWindow
irState    = 110;       % G_mbProbeState
irArmEcho  = 113;       % G_mbProbeArmEcho
irSamples  = 121;       % IR121-370
nSamples   = 250;
countsPerVolt = 5000;   % 0.2 mV per count
fs = 1 / cfg.slaveScan;

m = modbus('tcpip', cfg.address, cfg.port);
m.Timeout = 1;

% --- One-second windows, each read once its counter has advanced ---------
stats = nan(nWindows, 4);
last = read(m, 'inputregs', irWindow, 1, cfg.serverId, 'uint16');
for k = 1:nWindows
    t0 = tic;
    while true
        w = read(m, 'inputregs', irWindow, 1, cfg.serverId, 'uint16');
        if w ~= last
            last = w;
            break;
        end
        if toc(t0) > 3
            error('window counter IR%d not advancing: is PRG_PressureProbe running?', irWindow);
        end
        pause(0.1);
    end
    raw = read(m, 'inputregs', irStats, 8, cfg.serverId, 'uint16');
    stats(k, :) = [realOf(raw(1:2)) realOf(raw(3:4)) realOf(raw(5:6)) realOf(raw(7:8))];
end

% --- Capture: arm with a new value, wait for ready, read the block --------
arm = mod(round(now * 86400), 65535) + 1;
write(m, 'holdingregs', hrArm, arm, cfg.serverId, 'uint16');
t0 = tic;
while true
    s = read(m, 'inputregs', irState, 4, cfg.serverId, 'uint16');
    if s(1) == 2 && s(4) == arm
        break;
    end
    if toc(t0) > 3
        error('capture not ready (state %d, echo %d, armed %d)', s(1), s(4), arm);
    end
    pause(0.05);
end
counts = zeros(1, nSamples);
done = 0;
while done < nSamples
    n = min(125, nSamples - done);
    counts(done + 1:done + n) = read(m, 'inputregs', irSamples + done, n, cfg.serverId, 'uint16');
    done = done + n;
end
volts = counts / countsPerVolt;

% --- Report and save --------------------------------------------------------
r = pressureNoiseReport(volts, fs, stats, label);

capDir = fullfile(here, 'captures');
if ~exist(capDir, 'dir')
    mkdir(capDir);
end
fname = fullfile(capDir, sprintf('%s_%s.csv', label, datestr(now, 'yyyymmdd_HHMMSS')));
t = (0:nSamples - 1)' / fs;
fid = fopen(fname, 'w');
fprintf(fid, 't,volts,MPa\n');
fprintf(fid, '%.4f,%.5f,%.5f\n', [t volts(:) r.mpaPerVolt * (volts(:) - 4 * 0.434)]');
fclose(fid);
fprintf('saved %s\n', fname);
r.file = fname;
end


function x = realOf(words)
% REAL from two registers, low word first (the controller's word order).
x = double(typecast(uint16(words(:)'), 'single'));
end
