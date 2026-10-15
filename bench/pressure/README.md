# Pressure Transducer Noise Bench

Measures the noise on the pressure transducer input at the slave's full
2 ms sample rate, to size the sensor filter (`G_cfgAnalogFilterTimeConst`)
and to decide whether a capacitor across the loop shunt is needed. The
protocol registers cannot show this: IR19-20 carry the pressure after the
median and low-pass filters, read every 50 ms.

| File | Contents |
|------|----------|
| `slave/PRG_PressureProbe.st` | Slave program: one-second statistics of the raw input and a 250-scan capture on request, in the probe registers |
| `pressureNoiseProbe.m` | Axis at rest: reads the statistics and a capture through the MATLAB `modbus` object, reports them and saves the capture under `captures/` |
| `pressureNoiseRun.m` | Axis moving: acts as the master for the run, drives a path of steps through `masterStep` and reads the probe registers between exchanges; one capture and the windows per step |
| `buildSafetyHold.m` | Generates `SafetyHold.slx` (gitignored), the Desktop Real-Time model that holds the safety relay while `pressureNoiseRun` takes the shell's place |
| `pressureNoiseReport.m` | The report alone: statistics, capture rms and spectrum. Takes the arrays or a saved CSV |
| `captures/` | Saved captures (gitignored) |

## 1. Slave setup

The transducer is wired as in service: loop shunt (`G_cfgPressureShuntOhms`,
434 ohm) and, when fitted, the capacitor at the controller terminals, shunt
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
3. Add `PRG_PressureProbe` to the project and assign it to the 2 ms cyclic
   task. It uses only its own globals and runs beside `PRG_Main`. Do not
   load it together with `PRG_ModbusBench`, which uses the same registers.
4. Compile, download, start.

## 2. Running

On the lab PC, with `cfg.address` in `bench/modbus/modbusBenchConfig.m` set
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
`captures/<label>_<yyyymmdd_HHMMSS>.csv` (time, volts, MPa). A saved
capture is re-analysed with `pressureNoiseReport('captures/<file>.csv')`.

### Axis moving

`pressureNoiseRun` is the master for the run. It opens one connection,
exchanges the protocol registers every 50 ms with `masterStep` sequencing
the handshake, the motion and the stop as the shell does, and reads one
probe register block in each gap. The safety relay (P0.0), which the shell
normally drives, is held by `SafetyHold.slx`: build it once with
`buildSafetyHold`, open it, select **Connected IO** on the Desktop
Real-Time tab and save. The run starts the model before the first command
and stops it at the end, which releases the relay.

```matlab
cd bench/pressure
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
`captures/<label>_<n>_<step>_<timestamp>.csv` and the run as
`captures/<label>_run_<timestamp>.csv` (time, position, velocity, MPa,
master status, step, exchange time). A window's rms includes any change of
the pressure itself during that second; the capture's spectrum separates a
drift (below 25 Hz) from noise.

The run ends early, with the axis stopped, when the master reports a fault,
link loss or an unconfirmed mode (`r.abort`). Ctrl-C sends one exchange
with MotionEnable low, and if the script dies the slave's own watchdog stops
the axis within 300 ms. `tests/pressureNoiseRunSim` runs the script against
the simulated slave.

The scale is 1 MPa per mA and 434 ohm per mA: 1 mV is 0.0023 MPa, and the
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

| Label | Capacitor | Step | Window rms (mV) | Capture rms (mV) | Strongest line (Hz, mV) | Notes |
|-------|-----------|------|-----------------|------------------|-------------------------|-------|
| | none | idle | | | | |
| | none | jog | | | | |
| | none | pressure | | | | |
| | | idle | | | | |
| | | jog | | | | |
| | | pressure | | | | |
