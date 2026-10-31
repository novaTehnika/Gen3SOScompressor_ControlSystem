# Pressure Transducer Noise Bench

Measures the noise on the pressure transducer input at the slave's full
4 ms sample rate, to size the sensor filter (`G_cfgAnalogFilterTimeConst`)
and to decide whether a capacitor across the loop shunt is needed. The
protocol registers cannot show this: IR19-20 carry the pressure after the
median and low-pass filters, read every 50 ms.

| File | Contents |
|------|----------|
| `slave/PRG_PressureProbe.st` | Slave program: one-second statistics of the raw input and a 250-scan capture on request, in the probe registers |
| `pressureNoiseProbe.m` | Axis at rest: reads the statistics and a capture through the MATLAB `modbus` object, reports them and saves the capture under `captures/` |
| `pressureNoiseRun.m` | Axis moving: acts as the master for the run, drives a path of steps through `masterStep` and reads the probe registers between exchanges; one capture and the windows per step |
| `pressureNoiseReport.m` | The report alone: statistics, capture rms and spectrum. Takes the arrays or a saved CSV |

`SafetyHold.slx` (`bench/common/buildSafetyHold`), the Modbus link and the
`captures/` directory are shared with the other benches in `bench/common/`.

## 1. Slave setup

The transducer is wired as in service: loop shunt (434 ohm; with the
input's load, 421.4 ohm effective, `G_cfgPressureShuntOhms`) and, when fitted, the capacitor at the controller terminals, shunt
low side to the controller's analog ground, `MO1_AI_01` across the shunt.

1. Hardware Configuration, online: the Modbus server setup from
   `bench/modbus/README.md` section 1, with the **FC#04 input register
   group enlarged to at least 370 registers** (byte range `%QB73792` to
   `%QB74531`) so that the capture block IR121-370 lies inside it. Save
   and cycle power.
2. Declare the globals listed in the header of `slave/PRG_PressureProbe.st`
   in the matching groups. `G_mbProbeSamples` is a located array; if the
   editor refuses an array at a direct address, declare it with a named
   type `ARRAY[0..249] OF UINT` from the Data Types worksheet.
3. Add `PRG_PressureProbe` to the project and assign it to the 4 ms cyclic
   task. It uses only its own globals and runs beside `PRG_Main`. Do not
   load it together with `PRG_ModbusBench`, which uses the same registers.
4. Compile, download, start.

## 2. Running

On the lab PC, with `cfg.address` in `bench/common/benchConfig.m` set
to the controller IP. The shell model must be stopped for either script: a
second Modbus client beside the shell delays the shell's exchange, and the
slave raises `FAULT_COMM` while motion is commanded.

### Axis at rest

```matlab
cd bench/pressure
r = pressureNoiseProbe('idle');          % 5 one-second windows and one capture
```

The script prints each window's mean, rms and peak-to-peak in volts and in
MPa, then arms a capture, waits for it, prints its rms, the rms in three
bands (below 25 Hz, 25 to 100 Hz, 100 to 250 Hz) and the five largest
spectral lines, and saves the capture as
`bench/common/captures/<label>_<yyyymmdd_HHMMSS>.csv` (time, volts, MPa). A saved
capture is re-analysed with `pressureNoiseReport('../common/captures/<file>.csv')`.

### Axis moving

`pressureNoiseRun` is the master for the run. It opens one connection,
exchanges the protocol registers every 50 ms with `masterStep` sequencing
the handshake, the motion and the stop as the shell does, and reads one
probe register block in each gap. The safety relay (P0.0), which the shell
normally drives, is held by `SafetyHold.slx`: build it once with
`bench/common/buildSafetyHold`, open it, select **Connected IO** on the Desktop
Real-Time tab and save. The run starts the model before the first command
and stops it at the end, which releases the relay.

```matlab
cd bench/pressure
addpath ../common
buildSafetyHold                          % once
r = pressureNoiseRun('noCap');           % jog +2 mm/s for 8 s, then -2 mm/s for 8 s
r = pressureNoiseRun('noCap', {{'idle', 5}, {'jog', 2, 8}, {'jog', -2, 8}, {'pressure', 2, 10}});
```

| Step | Meaning |
|------|---------|
| `{'idle', s}` | No mode commanded for `s` seconds |
| `{'jog', v, s}` | Velocity mode at `v` mm/s (signed) for `s` seconds |
| `{'position', x, s}` | Position mode to `x` mm, then held for `s` seconds after arrival |
| `{'pressure', P, s}` | Pressure mode at `P` MPa gauge for `s` seconds |

Steps run in order with a stop between them. Each collects the one-second
windows while it runs and takes one capture `opts.settle` seconds (default
1) after its running status appears, or after arrival for a position step;
the step ends after its seconds once the capture is in. The windows and the
captures are reported per step at the end, the captures saved as
`bench/common/captures/<label>_<n>_<step>_<timestamp>.csv` and the run as
`bench/common/captures/<label>_run_<timestamp>.csv` (time, position, velocity, MPa,
master status, step, exchange time). A window's rms includes any change of
the pressure itself during that second; the capture's spectrum separates a
drift (below 25 Hz) from noise.

The run ends early, with the axis stopped, when the master reports a fault,
link loss or an unconfirmed mode (`r.abort`). Ctrl-C sends one exchange
with MotionEnable low, and if the script dies the slave's own watchdog stops
the axis within 300 ms. `tests/pressureNoiseRunSim` runs the script against
the simulated slave.

The scale is 1 MPa per mA and 421.4 ohm per mA: 1 mV is 0.0024 MPa, and the
transducer's own 0.2 % of span is 0.03 MPa, about 14 mV.

## 3. Conditions

Run each with and without the capacitor across the shunt.

| Step | Condition |
|------|-----------|
| `idle` | Drive off or holding, no mode commanded |
| `jog` | 2 mm/s, away from the limits and outside the chamber, so the pressure itself is constant |
| `pressure` | Pressure control at a target the chamber can hold: the service condition, with the drive working against the load |

Six runs. For each step, note the capacitor value, the window rms, the
capture rms and the strongest spectral line.

## 4. Reading the results

- **Scatter below about 2 mV rms** (0.005 MPa, the design study's
  assumption): the input is clean; the sensor filter can be shortened to
  20 ms, which the study shows halves the step overshoot.
- **A line at 50 or 60 Hz, or at the drive's switching frequency** (check
  the SERVOPACK's PWM setting): wiring or grounding; the capacitor helps
  only above its corner (170 Hz at 2.2 uF, 78 Hz at 4.7 uF).
- **Broadband energy in the 100 to 250 Hz band** that falls when the
  capacitor is fitted: aliasing, which the software filters could not have
  removed. Keep the capacitor.
- **Noise that appears only while jogging**: coupling from the servo
  cables; route the transducer cable away from them before changing the
  filter.

## 5. Results

Shunt 434 ohm, transducer wired as in service, chamber without liquid.
Captures at rest first (`pressureNoiseProbe`, static 1.07 MPa gauge), then
with the axis moving (`pressureNoiseRun`). Capacitor across the shunt at
the controller terminals; the electrolytic with its positive lead on the
shunt high side.

| Label | Capacitor | Step | Window rms (mV) | Capture rms (mV) | Bands 25-100 / 100-250 Hz (mV) | Strongest line (Hz, mV) | Notes |
|-------|-----------|------|-----------------|------------------|--------------------------------|-------------------------|-------|
| noCap | none | at rest | | 1.96 | 1.15 / 1.30 | 8, 0.75 | p-p 10.6 mV |
| wCermCap | 10 nF ceramic | at rest | | 1.93 | 1.31 / 1.31 | 58, 0.77 | no effect |
| 2.2uF | 2.2 uF electrolytic | at rest | | 0.90 | 0.48 / 0.61 | 102, 0.30 | |
| 4.7uF | 4.7 uF electrolytic | at rest | | 0.80 | 0.47 / 0.55 | 20, 0.33 | p-p 4.8 mV |
| 6.9uF | 2.2 + 4.7 uF | at rest | | 0.62 | 0.35 / 0.46 | 132, 0.21 | |
| noCap | none | jog +2 mm/s | | 1.60 | 0.77 / 1.27 | 232, 0.57 | |
| noCap | none | jog -2 mm/s | | 1.56 | 0.77 / 1.20 | 134, 0.58 | |
| 4u7 | 4.7 uF + 10 nF | idle | | 0.76 | 0.46 / 0.59 | 26, 0.25 | |
| 4u7 | 4.7 uF + 10 nF | jog +2 mm/s | | 3.42 | 0.43 / 0.53 | 4, 0.52 | rms is the pressure rising 23 mV/s while the piston compressed the gas |
| 4u7 | 4.7 uF + 10 nF | pressure 0.5 MPa | | 0.80 | 0.41 / 0.60 | 18, 0.30 | holding at 0.41-0.43 MPa |

The ADC step is 0.2 mV. No spectral line stands out in any capture: the
noise is broadband, and neither jogging nor pressure control adds to it.
Through the slave's median-5 and 20 ms filters it is 0.001 MPa rms bare
and 0.0005 MPa with 4.7 uF, against the 0.1 MPa in-band threshold.
Fitted: 4.7 uF electrolytic with 10 nF ceramic in parallel, corner 78 Hz,
as the anti-alias for the 4 ms sampling. Sensor filter: 20 ms.

Link during the runs (raw frames): exchange median 12.9 ms, p99 22.5 ms;
longest gap between cycles 108 ms.

## 6. Analog input calibration check

The slave reads the transducer as the voltage across the shunt
(`FB_PressureInput`: `LoopMa = Volts / ShuntOhms`), so its reading carries
three errors in series: the transducer's current, the voltage the loop
develops at the input terminals (the shunt in parallel with the input's own
impedance, specified above 10 kohm), and the input's conversion of that
voltage.

**At the transducer's zero** (2026-10-30, transducer vented, wired as in
service):

| Quantity | Value | Expected | Pressure equivalent |
|---|---|---|---|
| Loop current, meter in series | 3.98 mA | 4.00 mA | -0.02 MPa |
| Shunt resistance | 434 ohm | | |
| Voltage at the input terminals, meter | 1.695 V | 3.98 mA x 434 ohm = 1.727 V | -0.074 MPa |
| Voltage in MotionWorks | 1.685 to 1.688 V | 1.695 V (meter) | -0.020 MPa |
| Pressure the slave reports | -0.11 MPa | 0 | |

The three terms add to the -0.11 MPa the logs show. The largest is the
terminal voltage: 1.695 V for 3.98 mA is 425.9 ohm, which a 434 ohm shunt
in parallel with about 23 kohm would give. A load of that kind is a gain
error, -1.9 % of the loop current at every pressure, not an offset: the
slave would read 0.27 MPa low at 10 MPa. The meter's own current accuracy
is of the same order, so a second point at pressure is needed to tell the
two apart.

**The input against a voltage source.** A bench supply across the input
pins, read by a multimeter (Klein MM300), an oscilloscope (after its
self-calibration) and the slave (MotionWorks debug mode):

| Multimeter (V) | Oscilloscope (V) | Slave (V) |
|---|---|---|
| 0.365 | 0.364 to 0.367 | 0.362 |
| 1.028 | 1.04 | 1.021 |
| 1.978 | 1.99 | 1.966 |
| 1.676 | 1.68 | 1.665 |
| 2.38 | 2.40 | 2.385 |
| 3.08 | 3.11 | 3.078 |
| 3.93 | 3.94 | 3.926 |
| 4.38 | 4.42 | 4.377 |
| 5.14 | 5.19 | 5.133 |
| 5.71 | 5.74 | 5.696 |
| 6.18 | 6.23 | 6.168 |
| 6.68 | 6.72 | 6.669 |
| 7.40 | 7.44 | 7.379 |
| 8.74 | 8.80 | 8.708 |
| 9.44 | 9.45 | 9.409 |

Least squares, slave against multimeter: slave = 0.9971 x meter + 2.0 mV,
residual 6.3 mV rms, largest 9.8 mV. Against the oscilloscope: 0.9930 x
scope - 6.1 mV, residual 13 mV rms; the oscilloscope reads 0.4 % above the
multimeter throughout. Near the zero (1.676 V) the slave reads 11 mV low
(0.025 MPa). The input's +/-0.1 % of a 20 V range is +/-20 mV, and a
hand-held meter's DC volts are typically good to 0.5 %, so neither
instrument resolves an error in the input beyond its specification: the
conversion contributes about 0.02 MPa and is not the main term.

**Calibrating the chain.** Because the errors are a gain and an offset in
the current-to-volts chain, the slave's correction is two numbers,
`G_cfgPressureShuntOhms` (the effective resistance, volts read per mA) and
`G_cfgPressureZeroMa`. Measure them end to end: hold the loop at two or
more currents across the span (the transducer at two pressures against a
reference gauge, or a bench supply through a series resistor in place of
the transducer, with the meter in series) and fit the slave's volts
against the current. The slope is the effective resistance; the
intercept, divided by it, corrects the zero. Read the capacitor across
the shunt out of the circuit too: an electrolytic's leakage is a current
the shunt does not see.

**The chain against a current source** (2026-10-31). A bench supply as a
current source in place of the transducer, the multimeter (Klein MM300)
in series in the loop, the oscilloscope across the input pins, and the
slave's volts in MotionWorks debug mode, shunt and capacitor in place:

| Current (mA) | Oscilloscope (V) | Slave (V) |
|---|---|---|
| 0.136 | 0.061 to 0.065 | 0.063 |
| 1.24 | 0.512 | 0.523 |
| 2.66 | 1.12 | 1.128 |
| 4.08 | 1.74 | 1.727 |
| 5.50 | 2.33 | 2.326 |
| 6.59 | 2.80 | 2.787 |
| 7.87 | 3.36 | 3.328 |
| 9.12 | 3.86 | 3.856 |
| 10.74 | 4.56 | 4.538 |
| 12.09 | 5.14 | 5.103 |
| 13.79 | 5.88 | 5.822 |
| 14.52 | 6.17 | 6.125 |
| 15.61 | 6.64 | 6.582 |
| 16.69 | 7.11 | 7.036 |
| 17.78 | 7.60 | 7.494 |
| 19.05 | 8.15 | 8.029 |
| 19.61 | 8.33 | 8.262 |
| 20.7 | 8.88 | 8.823 |
| 21.0 | 9.04 | 8.980 |
| 22.5 | 9.66 | 9.601 |
| 23.1 | 9.90 | 9.840 |
| 15.43 | 6.57 | 6.503 |
| 9.09 | 3.84 | 3.836 |
| 3.86 | 1.66 | 1.633 |
| 2.05 | 0.877 | 0.870 |
| 1.91 | 0.802 | 0.809 |
| 1.51 | 0.640 | 0.643 |
| 0.97 | 0.400 | 0.4153 |
| 0.50 | 0.214 | 0.2120 |
| 0.17 | 0.080 | 0.0786 |
| 0.1574 | 0.071 | 0.06425 |
| 0.0395 | 0.160 | 0.0144 |
| 0.0016 | 0.002 | -0.0008 |

Least squares over the 29 points below 20 mA: slave volts = 421.4 ohm x
current + 5.0 mV, residual 4.3 mV rms (0.010 MPa), largest 7.7 mV. Up and
down sweeps agree. The four points above 20 mA sit 5 ohm higher (1.2 %)
with 0.1 mA resolution: the meter changed range there, so they are left
out, and the step between ranges is a measure of the meter's own current
accuracy. The effective resistance falls from 423 ohm at 4 mA to 421.3
ohm at 19.6 mA (curvature of about 0.4 % over the span, 2.6 mV rms with a
quadratic term), small beside that. The oscilloscope reads about 1 %
above the slave and is not used.

So the input sees 421.4 ohm per mA, not 434: 2.9 % low, consistent with
the shunt in parallel with about 15 kohm (the input's impedance,
specified above 10 kohm, and the capacitor's leakage together). With
`G_cfgPressureShuntOhms` at 434 the slave reads

  P_read = 0.971 x P_true - 0.104 MPa,

that is, -0.10 MPa at zero, -0.40 MPa at 10 MPa and -0.57 MPa at 16 MPa,
and the zero agrees with the vented transducer above (1.6865 V is
4.002 mA on this line, against 3.98 mA on the meter). Setting
`G_cfgPressureShuntOhms` to 421.4 with `G_cfgPressureZeroMa` at 4.0
leaves +0.012 MPa from the intercept at every pressure, inside the
meter's accuracy. The calibration is only as good as the meter's mA
range; a loop calibrator would tighten it.
