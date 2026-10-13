# Pressure Transducer Noise Bench

Measures the noise on the pressure transducer input at the slave's full
2 ms sample rate, to size the sensor filter (`G_cfgAnalogFilterTimeConst`)
and to decide whether a capacitor across the loop shunt is needed. The
protocol registers cannot show this: IR19-20 carry the pressure after the
median and low-pass filters, read every 50 ms.

| File | Contents |
|------|----------|
| `slave/PRG_PressureProbe.st` | Slave program: one-second statistics of the raw input and a 250-scan capture on request, in the probe registers |
| `pressureNoiseProbe.m` | Reads the statistics and a capture through the MATLAB `modbus` object, reports them and saves the capture under `captures/` |
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
to the controller IP:

```matlab
cd bench/pressure
r = pressureNoiseProbe('idle');          % 5 one-second windows and one capture
r = pressureNoiseProbe('jog', 10);       % 10 windows
```

The script prints each window's mean, rms and peak-to-peak in volts and in
MPa, then arms a capture, waits for it, prints its rms, the rms in three
bands (below 25 Hz, 25 to 100 Hz, 100 to 250 Hz) and the five largest
spectral lines, and saves the capture as
`captures/<label>_<yyyymmdd_HHMMSS>.csv` (time, volts, MPa). A saved
capture is re-analysed with `pressureNoiseReport('captures/<file>.csv')`.

The scale is 1 MPa per mA and 434 ohm per mA: 1 mV is 0.0023 MPa, and the
transducer's own 0.2 % of span is 0.03 MPa, about 14 mV.

## 3. Conditions

All at atmosphere, so the pressure itself is constant. Run each with and
without the capacitor across the shunt.

| Label | Condition |
|-------|-----------|
| `off` | Drive off (slave idle) |
| `hold` | Drive enabled, brake hold (mode 1) |
| `jog` | Jogging at 2 mm/s (mode 3), away from the limits |

Twelve runs. For each, note the capacitor value, the window rms, the
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

| Label | Capacitor | Window rms (mV) | Capture rms (mV) | Strongest line (Hz, mV) | Notes |
|-------|-----------|-----------------|------------------|-------------------------|-------|
| off | none | | | | |
| hold | none | | | | |
| jog | none | | | | |
| off | | | | | |
| hold | | | | | |
| jog | | | | | |
