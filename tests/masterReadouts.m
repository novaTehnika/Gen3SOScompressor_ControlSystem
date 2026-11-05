function masterReadouts()
%MASTERREADOUTS The operator app's readout logic: the volume views, the
%   readout text and the speed conversion.
%
%   Run from the repository root:  addpath master-src/logic tests; masterReadouts
%   Depends on: master-src/logic

cfg = masterConfig();

% Volume views: swept volume is the bore area times the travel left.
assert(abs(sweptVolume(cfg.posEOT, cfg)) < 1e-12);
assert(abs(sweptVolume(cfg.posEOT - 100, cfg) ...
           - 100 * pi / 4 * (2.602 * 25.4)^2 / 1000) < 1e-6);
x = 120;
assert(abs(mixingVolume(x, cfg) - sweptVolume(x, cfg) - cfg.mixingDeadVolume) < 1e-9);
assert(cfg.mixingDeadVolume <= cfg.deadVolume);
x0 = 200;
assert(abs(sweptVolume(x0, cfg) - sweptVolume(x0 + 10, cfg) - 10 * mLPerMm(cfg)) < 1e-9);
fprintf('ok  volume views (%.3f mL/mm)\n', mLPerMm(cfg));

% Readout text: round first, no sign on a zero, a deadband, a gap as '--'.
assert(strcmp(readoutText(-0.004, 2, 0), '0.00'));
assert(strcmp(readoutText(-0, 2, 0), '0.00'));
assert(strcmp(readoutText(-0.006, 2, 0), '-0.01'));
assert(strcmp(readoutText(-0.009, 2, 0.01), '0.00'));
assert(strcmp(readoutText(0.012, 2, 0.01), '0.01'));
assert(strcmp(readoutText(NaN, 1, 0), '--'));
assert(strcmp(readoutText(12.34, 1, 0), '12.3'));
fprintf('ok  readout text\n');

% Operator speed: percent of the ceiling, limited to 0..100 %.
assert(speedFromPercent(50, cfg) == cfg.speedMax / 2);
assert(speedFromPercent(-5, cfg) == 0);
assert(speedFromPercent(150, cfg) == cfg.speedMax);
assert(all(cfg.speedPresets > 0 & cfg.speedPresets <= 100));
fprintf('ok  speed conversion\n');

fprintf('all passed\n');
