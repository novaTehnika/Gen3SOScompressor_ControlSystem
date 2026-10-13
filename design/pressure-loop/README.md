# Pressure loop design study

Octave model of the slave pressure loop, used to choose between closing the
loop through the drive velocity loop or the torque loop, to pick the
controller structure and gains, and to size the discharge valve opening
rate. Nothing here runs on the controllers; the slave implementation is in
`slave-src/` and the closed-loop tests in `tests/`.

## Running

```sh
cd design/pressure-loop
octave-cli --quiet studyCompare.m     # three scenarios, six controllers (~2 min)
octave-cli --quiet studyValve.m       # dip vs valve opening time (~1 min)
gnuplot plot.gp plotValve.gp          # compress/stiffstep/discharge/valve_dip/valve03.png
```

Both scripts print metrics tables and write CSV traces in this directory.
`doSweeps = true; studyCompare` also runs the gain sweeps. CSV, PNG and log
files are gitignored.

## Files

| File | Role |
|---|---|
| `plantParams.m` | chamber, actuator, sensor and timing parameters |
| `loopScenario.m` | operating cases: `compress`, `stiffstep`, `discharge`, `valve` |
| `loopController.m` | controller description: mode `V` or `T`, PID gains, stop law, slew, feedforward |
| `simPressureLoop.m` | one run: sensor, controller, plant, metrics |
| `loopCost.m` | scalar ranking used by the sweeps |
| `writeLoopCsv.m` | traces to CSV for gnuplot |
| `studyCompare.m`, `plot.gp` | velocity versus torque command across the three scenarios |
| `studyValve.m`, `plotValve.gp` | pressure dip versus valve opening time |

## Model

**Chamber.** A piston of area A compresses a volume holding liquid of
effective bulk modulus K and oxygen gas drawn in at atmosphere. The gas is
isothermal and dissolves into the liquid per Henry's law with a first-order
time constant. Compliance is the gas term plus the liquid term, so the
stiffness dP/dx runs from about 0.002 MPa/mm when gas-rich to 10 to 25 MPa/mm
once the gas has dissolved. A discharge outflow Q removes liquid.

**Velocity actuator** (Y_DirectControl in VelocityTLMode). The command is
slewed at the accel limit, lagged by the drive velocity loop, and stalls when
the chamber force reaches the torque limit.

**Torque actuator** (TorqueVLMode). Reflected mass at the piston driven by the
commanded force against chamber pressure, with viscous and Coulomb friction
and the drive's velocity clamp. The controller always adds the feedforward
force A times target.

**Sensor and timing.** Pressure is sampled every slave scan with white noise,
passed through a median of 5 and a first-order filter, and the controller's
command takes effect one scan later, matching the FB call order in PRG_Main.

**Controller.** PID with a filtered derivative, output clamp, and conditional
anti-windup (the integrator holds while saturated with the error pushing
further into saturation). Options: a stiffness-aware stop law that clamps the
velocity command to the rate from which the slewed actuator can still stop
within the pressure error, using dP/dx estimated over the last ten scans; and
a velocity feedforward of the expected valve flow divided by A.

## Assumptions to replace with bench measurements

| Parameter | Value used | How to measure |
|---|---|---|
| Effective liquid bulk modulus K | 1 GPa | run 0.1 mm/s in the liquid-only chamber and read dP/dt |
| Discharge flow Q | 1 mL/s | volume collected per unit time at operating pressure |
| Reflected mass, friction | 50 t, 5 kN s/m, 500 N | torque at constant velocity, in both directions |
| Sensor noise | 5 kPa rms per sample | log the raw analog input at rest |
| Drive velocity loop lag | 20 ms | step response in velocity mode |

## Results with these parameters

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
