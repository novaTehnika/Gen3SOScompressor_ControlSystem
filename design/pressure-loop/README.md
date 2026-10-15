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
| `test_model.py` | self-checks, also run by `tests/run_all.sh` |

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

## Assumptions to replace with bench measurements

| Parameter | Value used | How to measure |
|---|---|---|
| Effective liquid bulk modulus K | 1 GPa | run 0.1 mm/s in the liquid-only chamber and read dP/dt |
| Discharge flow Q | 1 mL/s | volume collected per unit time at operating pressure |
| Reflected mass, friction | 50 t, 5 kN s/m, 500 N | torque at constant velocity, in both directions |
| Sensor noise | 5 kPa rms per sample | log the raw analog input at rest |
| Drive velocity loop lag | 20 ms | step response in velocity mode |

## Results with these parameters

These results were produced with the earlier Octave model (fixed liquid volume, single slew limit, separate reimplementation of the controller).

**Velocity versus torque command.** Velocity command wins. The drive's own
velocity loop absorbs the friction and the mass-spring resonance, and the
pressure loop sees an integrator. In torque command the pressure loop sees
the reflected mass on the stiff chamber, a resonance near 6 Hz with almost no
damping, through the 50 ms filter. Feedback gain above about 5 percent of A
(N per MPa) is unstable, leaving pure force feedforward, which holds within
the friction dead band of about 0.25 MPa only if the force calibration is
exact, and stick-slips during discharge. Torque stays as a force cap inside
velocity mode.

**Derivative action** trims the valve-opening dip from 0.55 to 0.49 MPa at
three to five times the command activity from sensor noise. Not used.

**Accel and decel limit.** At the direct-control limit of 5 mm/s² the piston
approaching the stiff regime at 5 mm/s overshoots the target by 15 to
25 MPa. Pressure mode needs its own accel and decel of at least 50 mm/s² plus
the stop-law clamp, which holds the overshoot to about 3 MPa at full approach
velocity.

**Discharge valve opening.** Dip in MPa for a 1 mL/s final flow at 10 MPa in
the liquid-only chamber, versus valve opening time:

| Controller | step | 0.3 s | 1 s | 3 s | 10 s |
|---|---|---|---|---|---|
| PI, Kp 0.25, Ki 0.2 | 0.90 | 0.90 | 0.80 | 0.46 | 0.15 |
| PI, Kp 0.5, Ki 0.2 | 0.57 | 0.55 | 0.49 | 0.34 | 0.14 |
| PI, Kp 1, Ki 0.5, filter 20 ms | 0.31 | 0.28 | 0.23 | 0.15 | 0.06 |
| PI, Kp 0.5, Ki 0.2, plus flow feedforward | 0.07 | 0.04 | 0.01 | 0.005 | 0.002 |
| Torque feedforward (reference) | 0.25 | 0.25 | 0.25 | 0.25 | 0.25 |

The dip is quasi-static, about Q/(A Kp), until the opening time exceeds the
loop's integral time of about one second, so opening the valve slowly only
helps beyond that. Feedforward of the expected flow, which the master knows
when it opens the valve, cuts the dip below 0.1 MPa even for a step and
tolerates a 30 percent flow error at a 0.15 MPa dip.

Gains are mm/s per MPa (Kp) and per second (Ki). Recommended configuration:
PI on velocity, Kp about 0.5, Ki about 0.2, no derivative, pressure-mode
accel and decel of 50 mm/s² or more, stop-law clamp, flow feedforward.
