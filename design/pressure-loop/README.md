# Pressure loop design study

Python model of the slave pressure loop, used to choose between closing the
loop through the drive velocity loop or the torque loop, to pick the
controller structure and gains, and to size the discharge valve opening
rate. The slave's own pressure blocks run inside the model: the sensor chain
is the generated `FB_AnalogInputFilter` and `FB_PressureInput`, and the
slave controller is the generated `FB_PressureController`, wired as
`PRG_Main` wires them, with its gains and limits read from the GVL. Nothing
here runs on the controllers; the slave implementation is in `slave-src/`
and the closed-loop tests in `tests/`.

## Running

```sh
pip install -r design/requirements.txt           # numpy, scipy, matplotlib
python3 design/pressure-loop/test_model.py       # self-checks, a few seconds
python3 design/pressure-loop/study_compare.py    # three scenarios, seven controllers, a few seconds
python3 design/pressure-loop/study_valve.py      # dip versus valve opening time
python3 design/pressure-loop/study_linear.py     # margins against chamber stiffness, about 35 s
python3 design/pressure-loop/study_robust.py     # Monte Carlo over the uncertain parameters, about 5 min on 4 CPUs
python3 design/pressure-loop/test_linear.py      # linear model against the nonlinear sim
python3 design/pressure-loop/test_robust.py      # sampler, criteria and parallel runner
```

The scripts run from the repository root or from this directory. Options:

| Option | Effect |
|---|---|
| `--sweeps` (compare) | also run the gain sweeps and the slave-gain sweeps |
| `--set key=value` | override a `Params` field (`K=2.2e9`) or a slave setting by its GVL name (`G_cfgPressureKp=1`); repeatable |
| `--out DIR` | where CSV traces and PNG plots go (default `out/`, gitignored) |
| `--legacy` | the earlier plant and sensor: fixed liquid volume, one slew limit, median plus first-order filter in pressure units |
| `--workers N`, `--seed N` | processes (default one per CPU) and sensor noise seed |

`design/generated/` holds the slave blocks translated by `tools/st2py.py`;
`slave.py` regenerates it when it is missing or older than `slave-src/FB/`.

## Files

| File | Role |
|---|---|
| `slave.py` | puts the generated slave blocks on `sys.path` (regenerating them if stale); `slave_config()` returns the `G_cfg*` defaults of the GVL |
| `params.py` | `Params`: chamber, actuator, sensor and timing parameters; `with_(**kw)`, `legacy()`, `from_overrides(["key=value"])`; values the slave configures are read from `slave_config()` and overridden through `slave_cfg` |
| `plant.py` | chamber, piston position, velocity and torque actuators; `Plant.scan` advances one slave scan |
| `sensor.py` | `SlaveSensor` (4-20 mA loop, shunt, noise, slave filter and scaling blocks) and `LegacySensor` |
| `controllers.py` | `SlaveController` (the slave's block, optional flow feedforward) and `LegacyController` (PID, stop-law estimator, torque mode); common `reset` / `step` interface |
| `scenarios.py` | `compress`, `stiffstep`, `discharge`, `valve` as dataclasses with overridable fields |
| `sim.py` | `run(params, scenario, controller)` returns a `Result` with traces and metrics |
| `metrics.py` | overshoot, peak error, settling time, rms error, ripple, command activity, stroke used; `loop_cost` |
| `runner.py` | `run_cases` runs `Case(params, scenario, controller)` specs in parallel (spawn) |
| `study_compare.py`, `study_valve.py` | velocity versus torque command and the slave controller across the three scenarios; pressure dip versus valve opening time |
| `studyutil.py` | command line, CSV and plot helpers for the studies |
| `linear.py`, `study_linear.py` | discrete-time linear model of the velocity-command loop; margins, stability boundary, Bode and disturbance responses |
| `robust.py`, `study_robust.py` | uncertain-parameter table, requirement limits, Latin hypercube sampler; tornado, corners and Monte Carlo per controller configuration |
| `test_model.py`, `test_linear.py`, `test_robust.py` | self-checks, also run by `tests/run_all.sh` |

## Model

**Chamber.** A piston of area A compresses a volume holding liquid of
effective bulk modulus K and oxygen gas drawn in at atmosphere. The gas is
isothermal and dissolves into the liquid per Henry's law with a first-order
time constant. Compliance is the gas term plus the liquid term, so the
stiffness dP/dx runs from about 0.002 MPa/mm when gas-rich to 10 to 25 MPa/mm
once the gas has dissolved. The piston position is integrated and the
chamber volume is the initial volume minus A times position; the contents
fill it, so the stressed liquid plus the free gas equals that volume. The
stroke used is reported with every run so it can be checked against the
axis travel.

A discharge outflow Q removes liquid: the liquid volume falls by Q dt, the
compliance uses the current liquid volume, and the gas dissolved in it leaves
in proportion, so the chamber stiffens as it empties.

**Velocity actuator** (Y_DirectControl in VelocityTLMode). The command is
slewed with separate accel and decel limits (accel while its magnitude grows,
decel while it shrinks or reverses; the controller supplies both, as
`G_cfgPressureAccel` and `G_cfgPressureDecel` do on the slave), lagged by the
drive velocity loop, and stalls when the chamber force reaches the torque
limit.

**Torque actuator** (TorqueVLMode). Reflected mass at the piston driven by the
commanded force against chamber pressure, with viscous and Coulomb friction
and the drive's velocity clamp. The controller always adds the feedforward
force A times target.

**Sensor and timing.** The chamber gauge pressure becomes a 4-20 mA loop
current over the configured transducer range, volts across the shunt, plus
white noise specified as pressure rms. The generated median/low-pass block
filters the volts (alpha computed as `PRG_Main` computes it) and the
generated scaling block returns MPa. Because `PRG_Main` reads the filter and
scaling outputs at the top of the scan and calls the blocks at the end, the
controller sees the filter output of two scans earlier. The controller's
command takes effect one scan later, matching the FB call order. Each scan
the plant integrates `nsub` substeps.

**Slave controller.** `FB_PressureController` with the target clamped to
`G_cfgPressureTargetMax`, the integrator reset on the entry scan, the output
clamped to `G_cfgPressureVelMax`, and accel and decel from the GVL. It is a
PI with a stop law that uses a configured stiffness and a minimum velocity,
and has no derivative or feedforward. `SlaveController(flow_ff=True)` adds
the expected flow divided by A to the velocity after the block, which the
slave does not do.

**Legacy controller.** PID with a filtered derivative, output clamp, and
conditional anti-windup (the integrator holds while saturated with the error
pushing further into saturation). Options: a stiffness-aware stop law that
clamps the velocity command to the rate from which the slewed actuator can
still stop within the pressure error, using dP/dx estimated over the last
ten scans; and a velocity feedforward of the expected valve flow divided by
A. Torque mode applies A times target as feedforward. It exists to explore
structures the slave does not have.

## Linear analysis

`linear.py` models the slave's velocity-command loop at the scan time Ts,
for stability margins across the chamber stiffness k = dP/dx:

    L(z) = C(z) z^-1 Gd(z) z^-4 F(z)

- `C = Kp + Ki Ts/(z-1)`: the PI of `FB_PressureController`.
- `z^-1`: the command computed in a scan acts in the next one.
- `Gd`: zero-order-hold discretisation of k/(s(tauV s + 1)), the drive
  velocity loop and the chamber integrator.
- `z^-4`: the median of 5 (two scans for slowly varying signals) and the
  `PRG_Main` read order (two scans).
- `F`: the first-order filter, alpha = Ts/(tau + Ts).

Outflow Q acts on the chamber integrator directly, past the drive lag:
P/Q = -(k Ts/A)/(z-1) S. The clamp, stop law, slew limits, median
nonlinearity and gas dissolution are not modelled. With Ki scaled with Kp
the loop depends on Kp·k only, so a stiffness-scheduled gain is Kp = wc/k
for a chosen crossover wc. `test_linear.py` compares the model with the
nonlinear simulation: step response, stability boundary and oscillation
frequency, and outflow dip.

## Robustness study

`study_robust.py` checks a slave configuration against its requirements over
the plausible ranges of the uncertain parameters and shows which parameters
drive failures. `robust.py` holds the parameter table (`UNCERTAIN`), the
limits (`THRESH`) and the sampler.

| Parameter | Range | Sampling |
|---|---|---|
| K | 0.5–2.5 GPa | log |
| Vl0 | 250–350 mL | linear |
| Vg0 | 100–300 mL | linear |
| tauDiss | 10–100 s | log |
| tauV | 5–50 ms | log |
| sensor noise | 1–20 kPa rms | log |
| discharge flow Qf | 0.5–2 mL/s | linear |

`mr`, `b` and `Fc` act only in torque mode and are not varied. Each sample
runs compress, stiffstep, discharge and valve (0.3 s opening). Requirements:

- compress and stiffstep end in band (`G_cfgPressureInBand`);
- overshoot of at most 0.5 MPa (compress) and 0.6 MPa (stiffstep), from
  `tests/slavePressureMode.m`;
- late ripple of at most InBand/5;
- discharge and valve dip of at most 1 MPa, and they end in band;
- stroke of at most 105 mm (`G_cfgPosSoftLimitMax` minus the slave-sim
  chamber contact position).

The dip and stroke limits are placeholders until there is a stated
requirement.

The study runs a one-at-a-time sensitivity (tornado), all corners of the
four most influential parameters, then a Latin hypercube (`--n`, default
200) per configuration. The default configurations are the GVL, the GVL
with flow feedforward, the 20 ms filter with `StopVelMin` 1 mm/s, and that
with flow feedforward. Every configuration sees the same samples and noise
seeds. `--cfg key=value` and `--flow-ff` run a single custom configuration
instead.

## Assumptions to replace with bench measurements

| Parameter | Value used | How to measure |
|---|---|---|
| Effective liquid bulk modulus K | 1 GPa | run 0.1 mm/s in the liquid-only chamber and read dP/dt |
| Discharge flow Q | 1 mL/s | volume collected per unit time at operating pressure |
| Reflected mass, friction | 50 t, 5 kN s/m, 500 N | torque at constant velocity, in both directions |
| Sensor noise | 5 kPa rms per sample | log the raw analog input at rest |
| Drive velocity loop lag | 20 ms | step response in velocity mode |

## Results with these parameters

Gains are mm/s per MPa (Kp) and per second (Ki). "Slave (GVL)" is the
slave's `FB_PressureController` with the GVL configuration: Kp 0.5, Ki 0.2,
VelMax 5 mm/s, accel and decel 50 mm/s², stop-law stiffness 100 MPa/mm,
StopVelMin 0.5 mm/s, 50 ms filter.

**Velocity versus torque command.** Velocity command wins. The drive's own
velocity loop absorbs the friction and the mass-spring resonance, and the
pressure loop sees an integrator. In torque command the pressure loop sees
the reflected mass on the stiff chamber, a lightly damped resonance near
5 Hz behind the sensor filter. Feedback gain above about 5 percent of A
(N per MPa) is unstable. Through the slave's sensor chain, which adds two
scans of delay, the derivative-damped torque controller (Kd 100) also
oscillates (1.25 MPa ripple on stiffstep). Pure force feedforward holds
only within the friction dead band of about 0.25 MPa, and only if the force
calibration is exact. Torque stays as a force cap inside velocity mode.

**Slave controller, nominal parameters:**

| Scenario | settle s | overshoot MPa | rms error MPa | ripple MPa | stroke mm |
|---|---|---|---|---|---|
| compress to 10 MPa | 22.9 | 0.27 | 0.001 | 0.0002 | 59 |
| stiffstep 10 → 12 MPa | 1.31 | 0.22 | 0.014 | 0.003 | 0.2 |
| discharge 1 mL/s for 20 s | 24.4 | 0.60 | 0.022 | 0.005 | 5.9 |

On the gas-rich approach the configured stop law limits overshoot to
0.27 MPa. The legacy estimating stop law overshoots by 3.1 MPa at the same
gains, because the filtered reading lags the stiffening chamber. The cost is
a slower approach: the clamp stays below VelMax from the start, holding it
near 3 mm/s. The discharge overshoot (0.6 MPa) comes after the flow stops,
from integrator wind-up during the outflow.

**Margins against stiffness** (linear model, GVL gains):

| Chamber | k (MPa/mm) | Gain margin | Phase margin | Crossover (rad/s) | Max stable Kp |
|---|---|---|---|---|---|
| gas-rich | 0.002 | 92 dB | 2.8° | 0.02 | 2.1e4 |
| liquid, K = 1 GPa | 11.4 | 17.1 dB | 61° | 5.5 | 3.65 |
| liquid, K = 2.2 GPa | 25.2 | 10.2 dB | 41° | 10.8 | 1.65 |
| stop-law stiffness | 100 | −1.8 dB | −7° | 26 | 0.405 |

As a crossover wc = Kp·k (k in MPa/mm, Ki = 0.4 Kp):
- the stability limit is about 41 rad/s;
- 45° phase margin needs about 11 rad/s;
- 60° phase margin needs about 6 rad/s.

With the GVL Kp, the phase margin at liquid-only stiffness falls below
45° above K ≈ 1.9 GPa, and the loop goes unstable above K ≈ 7 GPa. It is
also unstable at the 100 MPa/mm the stop law assumes. So the stop law's stiffness
is a conservative braking assumption, not a stiffness the loop could
regulate at.

At liquid-only stiffness, the 50 ms filter and the 20 ms drive lag cost
about the same. The max stable Kp goes from 3.65 to 8.6 without the filter,
to 8.2 without the drive lag, and to 11.6 without both.

**Derivative action** trims the step-opening valve dip from 0.57 to
0.49 MPa, at three to five times the command activity from sensor noise.
Not used.

**Accel and decel limit.** At the direct-control limit of 5 mm/s², the
approach into the stiff regime overshoots by 15 MPa. Pressure mode needs
its own accel and decel of 50 mm/s² together with the stop law.

**Discharge valve opening.** Dip in MPa for a 1 mL/s final flow at 10 MPa
in the liquid-only chamber, against valve opening time:

| Controller | step | 0.3 s | 1 s | 3 s | 10 s |
|---|---|---|---|---|---|
| PI, Kp 0.25, Ki 0.2 | 0.91 | 0.90 | 0.81 | 0.47 | 0.15 |
| Slave (GVL), Kp 0.5, Ki 0.2 | 0.57 | 0.56 | 0.49 | 0.35 | 0.14 |
| PI, Kp 1, Ki 0.5, filter 20 ms | 0.31 | 0.28 | 0.23 | 0.15 | 0.06 |
| Slave (GVL) plus flow feedforward | 0.08 | 0.04 | 0.01 | 0.004 | 0.002 |
| Torque feedforward (reference) | 0.25 | 0.25 | 0.25 | 0.25 | 0.25 |

The dip is quasi-static, about Q/(A Kp), until the opening time exceeds the
loop's integral time of about one second. Opening the valve slowly only
helps beyond that. Feedforward of the expected flow cuts the dip below
0.1 MPa even for a step. The master knows that flow when it opens the valve.

**Robustness** (200 Latin hypercube samples; pass rate in percent, about
±4 points at 90 %):

| Requirement | GVL | GVL + flow FF | filter 20 ms, StopVelMin 1 | same + flow FF |
|---|---|---|---|---|
| compress overshoot ≤ 0.5 MPa | 98 | 98 | 100 | 100 |
| stiffstep overshoot ≤ 0.6 MPa | 80 | 80 | 98 | 98 |
| discharge dip ≤ 1 MPa | 76 | 100 | 88 | 100 |
| valve dip ≤ 1 MPa | 82 | 100 | 94 | 100 |
| valve ends in band | 81 | 100 | 100 | 100 |
| all requirements | 61 | 80 | 86 | 98 |

The other requirements pass in every sample: compress in band and ripple,
stiffstep in band, discharge in band, and stroke (at most 88 mm). A second
seed gives 64 / 82 / 89 / 100 for "all".

The tornado ranks parameters by influence:
Qf ≫ K > Vg0 > tauV > Vl0 > tauDiss > noise.

- **Discharge flow Qf.** It drives the dip and band failures. The stop law
  never commands less than StopVelMin, but above A·StopVelMin (1.7 mL/s at
  0.5 mm/s) it can hold a sustained outflow only with a steady error of
  about StopStiffness·v²/(2·Decel), 0.3 to 0.45 MPa.
- **Bulk modulus K, with tauV.** These drive the stiffstep overshoot through
  loss of margin. There are no failures in the lowest third of K and 58 % in
  the highest third. High K with a slow drive is the worst corner for every
  configuration.
- **Gas volume Vg0.** It sets the stroke only.
- **Sensor noise** up to 20 kPa has no effect on any requirement.

**Single changes that made things worse** (from an 80-sample exploration):
- Kp 0.25, accel and decel 100, or stop-law stiffness 40 each push the
  compress overshoot from 0.27 MPa to between 1 and 2.6 MPa. The GVL gains
  sit in a narrow optimum for the gas-rich approach.
- Kp 1.0 with the 50 ms filter loses margin at high K.

**Open questions:**
- The discharge flow range, and whether the master will send it for
  feedforward.
- The dip limit.
- The stroke budget.
- Whether to raise `G_cfgPressureStopVelMin` above the largest Qf/A and
  shorten `G_cfgAnalogFilterTimeConst` to 20 ms.

Measure Qf, then K, then tauV first. They decide all three of these.
