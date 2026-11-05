function masterReadouts()
%MASTERREADOUTS The operator app's readout logic: the volume views, the
%   readout text, the speed conversion and the pressure units.
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

% Pressure units: all absolute, converted through atm.
assert(pressureConvert(1, 'atm', 'psi') == cfg.psiPerAtm);
assert(pressureConvert(cfg.atmPerMPa, 'atm', 'MPa') == 1);
assert(pressureConvert(7, 'atm', 'atm') == 7);
units = {'atm', 'psi', 'MPa'};
x = [0.5 1 7.8 114.5];
for i = 1:3
    for j = 1:3
        y = pressureConvert(x, units{i}, units{j}, cfg);
        back = pressureConvert(y, units{j}, units{i}, cfg);
        assert(all(abs(back - x) <= 1e-12 * abs(x)));
        assert(isequal(size(y), size(x)));
    end
end
assert(abs(pressureConvert(1, 'MPa', 'psi') - cfg.atmPerMPa * cfg.psiPerAtm) < 1e-9);
for bad = {{1, 'bar', 'atm'}, {1, 'atm', 'kPa'}}
    id = '';
    try
        pressureConvert(bad{1}{:});
    catch err
        id = err.identifier;
    end
    assert(strcmp(id, 'pressureConvert:unit'));
end
id = '';
try
    pressureUnitInfo('bar');
catch err
    id = err.identifier;
end
assert(strcmp(id, 'pressureConvert:unit'));
labels = {'atm abs', 'psi abs', 'MPa abs'};
decimals = [2 1 3];
formats = {'%.2f', '%.1f', '%.3f'};
steps = [0.5 5 0.05];
for i = 1:3
    info = pressureUnitInfo(units{i}, cfg);
    assert(strcmp(info.name, units{i}) && strcmp(info.label, labels{i}));
    assert(info.decimals == decimals(i) && strcmp(info.format, formats{i}));
    assert(info.step == steps(i));
    assert(abs(info.fromUnit(info.toUnit(3.7)) - 3.7) < 1e-12);
end
info = pressureUnitInfo('psi', cfg);
assert(info.toUnit(1) == cfg.psiPerAtm && abs(info.fromUnit(cfg.psiPerAtm) - 1) < 1e-15);
assert(strcmp(readoutText(info.toUnit(1), info.decimals, 0), '14.7'));
assert(strcmp(readoutText(pressureUnitInfo('MPa', cfg).toUnit(1), 3, 0), '0.101'));
fprintf('ok  pressure units\n');

% masterStep's flags for the app: IR9 and IR10 as 0 or 1.
map = modbusMap();
clear masterStep
r = zeros(1, map.nRead);
r(map.ir.pressureBlock) = 1;
r(map.ir.atEndOfTravel) = 0;
[~, ~, ~, ~, ~, ~, ~, ~, ~, ~, pb, eot] = masterStep(r, 0, 0, 250, 0, 0, 20, 0, 0, ...
    speedFromPercent(cfg.speedDefault, cfg));
assert(pb == 1 && eot == 0);
r(map.ir.pressureBlock) = 0;
r(map.ir.atEndOfTravel) = 7;
[~, ~, ~, ~, ~, ~, ~, ~, ~, ~, pb, eot] = masterStep(r, 0, 0, 250, 0, 0, 20, 0, 0, ...
    speedFromPercent(cfg.speedDefault, cfg));
assert(pb == 0 && eot == 1);
clear masterStep
fprintf('ok  masterStep flags\n');

fprintf('all passed\n');
