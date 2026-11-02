# Measurement plan: every model parameter in three sessions

The models (`models/params.py`, the robustness ranges in
`design/pressure-loop/robust.py`) carry fourteen measured quantities and
two that were never measured. This plan remeasures all of them on the
calibrated pressure input, ordered so that each fill of the chamber, the
costly step, yields everything it can before it is drained. The recipes
are the benches' own (`bench/characterisation/README.md`,
`bench/friction/README.md`, `bench/pressure/README.md`); this page only
sequences them and says what each run is for.

| Parameter (`params.py`) | Session | Run |
|---|---|---|
| Transducer chain: `G_cfgPressureShuntOhms`, `ZeroMa`, `P_offset` | done | the current-source calibration of `bench/pressure/README.md` section 6 stands |
| Sensor noise `noise` | 0 | `pressureNoiseProbe('idle')`, and any capture later |
| Drive dead time `t_dead` and lag `tauV` at the motor | 0 | `frictionRun('s1', 'steps')` |
| Whether the drive slews (`slew_limited`) | 0 | the same steps with `G_cfgDirectCtrlAccel` at 1 mm/s² |
| Friction `Fc`, `b`, breakaway | 0 | `frictionRun` unloaded, in the bore and withdrawn |
| Reflected inertia `mr` | 0 | the same steps, `charDrivePath` |
| Backlash and motor-side friction of the drive path | 0 | direct reversals in the open chamber, `charDriveRecipe('reversal')` |
| Fill and dead volume `Vl0`, `V_passages` | 1 | the fill weighed against the geometry |
| Bulk modulus `K`, liquid against series compliance | 1 | `K300` x3, then `K200` x2 on the drained-down fill |
| Lag at the piston | 1 | `lag_wet` with the probe; the `K300` captures too |
| Stop distance under load (`G_cfgPressureDecel` and its delay) | 1 | `charStopPath` at 8 and 10 MPa |
| Load friction `mu` | 1 | the stops' legs and the deliveries |
| Drive path stiffness (bound) and friction reversal under load | 1 | the `K300` ramps, `charDrivePath`; the dither at 1, 5 and 9 MPa |
| Backlash and windup under load | 1 | the dither, `charDriveRecipe('dither')` |
| Hose compliance `C_hose`, wall against air | 1 | `K300_hose` before and after a hose purge |
| Orifice `Cd` | 1 | deliveries at 4, 7 and 10 MPa, discharge weighed |
| Hose procedure inputs: purge volume, `tube_refills`, `Tclose` | 1 and 2 | observed during the deliveries |
| Dissolution time constant `tauDiss` | 2 | a position hold at 10 MPa with a 1:1 charge |
| Stiffness with free gas (the gas model, the schedule's bound) | 2 | compression ramps with the charge |
| Hose opening at pressure (eq. 43, the degassing threshold) | 2 | the proximal valve opened at 10 MPa under the probe |
| Supply pressure `P_supply` | 2 | the gauge when the charge is admitted |

## Before the first session

- The controller runs the project built from `slave-src`: the step
  probe, the Modbus groups to HR114 and IR524, the 4 ms task,
  `G_cfgPressureShuntOhms` 421.4. Every capture below needs the probe.
  Still to confirm in the project: the task watchdog, the two LD wiring
  gaps and the GVL diff (TODO.md, "MotionWorks project").
- `tests\run_all.bat` passes under MATLAB on the lab PC, so the fits run
  there.
- The pressure chain is calibrated (`bench/pressure/README.md` section
  6: the effective shunt 421.4 ohm, the zero +0.002 MPa vented) and is
  not repeated. Every new log is on that scale, so `opts.logShuntOhms`
  is only for the October logs.
- Equipment: a scale reading 0.1 g; tared beakers; a stopwatch; the
  oxygen supply with its gauge. Wireshark if the link items are run.
- Every run logs to `bench/common/captures`. Keep the labels of the
  recipes, with the valve state in the label where it matters
  (`K300`, `K300_hose`, `lag_wet`, `lag_wet_open`), and weigh every fill,
  purge and discharge: the volumes are inputs to the fits.

## Session 0: dry, about an hour and a half

Piston in the bore, chamber open, no liquid.

1. **Check the zero**: with the transducer vented and the controller
   on the calibrated settings, the slave reads 0.00 +/- 0.02 MPa. A
   larger reading means the chain has moved since the calibration.
2. **Noise at rest**: `pressureNoiseProbe('idle')`. The scripts now read
   the step probe, and this is their first run on the hardware; the
   window rms should repeat the 0.8 mV with the capacitor. While the
   chamber is open, note the bore and length of the passage to the
   transducer and the transducer's cavity: their volume is the floor of
   the gas term the K fits will show, and `params.py` carries it as
   `V_air`.
3. **Drive response at the motor** (`bench/friction/README.md` 4.5):

   ```matlab
   r1 = frictionRun('s1', 'steps');
   r2 = frictionRun('s2', 'steps');
   rl = charDriveLag([r1.capFiles r2.capFiles], struct('accel', 1e4, 'decel', 1e4));
   ```

   Then set `G_cfgDirectCtrlAccel` to 1 mm/s² on the controller, run
   `frictionRun('s3', 'steps')`, and restore 10. If the s3 steps take
   longer than 60 ms the `Acceleration` input does limit the drive and
   `slew_limited` is a real option; if not, the question in TODO.md is
   closed.
4. **Inertia and backlash of the drive path**
   (`bench/characterisation/README.md` section 10), after the drive
   steps, the chamber still open and the piston in the bore, 5 mm clear
   of the position guard in the direction of the first command: direct
   reversals from +v to -v with no stop, at 0.3, 1 and 2.5 mm/s, each
   command held 2 s, the probe armed at every command change:

   ```matlab
   opts.probe = true;
   [path, info] = charDriveRecipe('reversal');       % 27 captures, 54 s
   rrev = charRun('rev', path, opts);
   rd = charDrivePath([r1.capFiles r2.capFiles rrev.capFiles]);
   ```

   `rd.inertia.mass_kg` is `mr` (lossless); `rd.reversal.backlash_um` and
   `rd.reversal.motorFriction_kN` are the backlash and the motor-side
   friction, or a bound on the backlash where the torque reverses with
   no flat segment. The 0.3 mm/s reversals are the clean ones.
5. **Friction unloaded** (`bench/friction/README.md` 4.1), both passes,
   with the probe so the breakaway and the stick time are captured:

   ```matlab
   ru = frictionRun('u1', 'unloaded', struct('probe', true));
   rb = frictionRun('bare', 'unloaded', struct('startPosition', 60));   % withdrawn below the cylinder entry
   ```

   `frictionFit` on each; the difference is the seal friction. The
   Coulomb term is the one open question in the friction set: 6.1 kN
   from the friction bench, 4.4 kN from the loaded legs of the stops
   runs. The loaded legs of session 1 settle it.
6. **While the controller is idle, if time allows** (the link model, not
   the physical models): the register count sweep with Wireshark
   (`bench/modbus/README.md`, "Register count") and a pull of the cable
   with the shell running, to settle when the model stops (TODO.md,
   "Lab PC").

## Session 1: one liquid-only fill, about three hours

Fill A: about 343 mL at the 283 mm start, proximal valve shut, the hose
prefilled with saline at atmosphere. Everything up to step 8 runs on
this one fill; the deliveries then drain it to the second fill level, so
no second fill is needed.

1. **Fill, weigh, purge** (`bench/characterisation/README.md` section 2).
   Weigh the saline added; it should agree with `A (x_EOT - x) + 57 mL`
   to a few mL, and the difference is air. Purge at 10 MPa until a
   `{'hold', 60}` step holds the pressure within 0.05 MPa; log the last
   hold, which is the leak and liquid-only baseline for the dissolution
   measurement of session 2. Some air stays whatever the purge: the
   passage to the pressure transducer, about 2 cm, and the transducer's
   cavity hold a pocket that cannot be purged and dissolves only slowly
   through the passage. It is small, about 0.1 mL by the geometry
   (`V_air` in `params.py`), and the fit carries it as the floor of its
   gas term. It is not what the October runs showed: their gas term,
   0.65 mL at the ramps' mean pressure, would be about 40 mL of air at
   atmosphere, which the machine does not hold, so that curvature is
   probably not gas, and the fit cannot tell gas from a compliance that
   falls with pressure. Three things here can: the weighed fill (air
   shows as a shortfall against the geometry), the hold at 10 MPa (gas
   dissolves and the pressure decays at constant position; a compliance
   does not), and the second fill level of step 8 (a series compliance
   is the same at both fills). Take the liquid K from the fit's liquid
   term (`K_liq_GPa`) and record the gas term of every run.
2. **Bulk modulus**, three runs, probe on (section 3):

   ```matlab
   opts.probe = true;
   r1 = charRun('K300', path, opts);  r2 = charRun('K300', path, opts);  r3 = charRun('K300', path, opts);
   rk1 = charBulkModulus({r1.file, r2.file, r3.file}, struct('fillVolume_mL', V1));
   sc  = charScheduleCheck({r1.file, r2.file, r3.file}, struct('salineVolume_mL', V1));
   ```

   The schedule check is the first commissioning step of the gain
   schedule: the bound must not sit below the measured stiffness. The
   same logs give the friction reversal under load and the bound on the
   drive path's stiffness, `charDrivePath({r1.file, r2.file, r3.file})`:
   the up/down offset of the motor position at the same pressure,
   through a 25 kN reversal.
3. **Lag at the piston** (section 5, closed chamber at 3 to 5 MPa):

   ```matlab
   r = charRun('lag_wet', path, opts);
   rl = charDriveLag([r.capFiles {r.file}], struct('accel', 1e4, 'decel', 1e4));
   ```

   `rl.tauP_s` is the parameter the robustness study spans 10 to 50 ms
   for; the K300 captures carry the same information at every ramp
   start and end.
4. **Dither** about 1, 5 and 9 MPa, before the stops, on the same fill
   with the proximal valve shut. Triangles of +-0.15 mm at
   0.2 mm/s, legs of 1.5 s, ten cycles at each level, the probe armed at
   every command change:

   ```matlab
   opts.probe = true;
   [path, info] = charDriveRecipe('dither');         % 75 captures, 105 s; errors if the swing nears the guard
   rdi = charRun('dither', path, opts);
   rd = charDrivePath([r1.capFiles r2.capFiles rrev.capFiles rdi.capFiles]);   % session 0's steps and reversals too
   ```

   At each reversal the torque changes by the friction reversal while
   the pressure stands still; `rd.dither.groups` has, per reversal
   pressure and direction, the backlash (at 1 MPa the pressure force,
   3.4 kN, is below the friction, so the force through the path passes
   zero and the flat segment shows; at 5 and 9 MPa it does not), the
   windup and the stiffness or its bound.
5. **Stops under load** at 8 and 10 MPa (section 6), the probe armed for
   every stop:

   ```matlab
   opts.probe = 'stops';
   [path, plan] = charStopPath(struct('levels_MPa', [8 10], 'speeds', [0.25 0.5], ...
       'budget_MPa', [2 0.7], 'rampOvershoot_MPa', 0.2, 'minRun_mm', 0.03, ...
       'minSteady_s', 0, 'pause_s', 3, 'P_start_MPa', 7));
   r = charRun('stops_hp', path, opts);
   rs = charStopDistance([r.capFiles {r.file}]);
   rr = frictionFit({}, r.file, struct('settle', 0.15, 'trim', 0.05, 'accel', 1e4));
   ```

   `rs.T0_s` and `rs.D_mm_s2` set the slave's allowance; `rr` gives
   `mu` and the loaded Coulomb term against session 0's unloaded one.
   The liquid-only chamber is the stiffest case, which is the one the
   allowance must cover.
6. **Hose compliance** (section 3, "Hose stiffness"). Bring the chamber
   to atmosphere, open the proximal valve, run the K path once; purge
   the hose with a short delivery at 2 MPa (the distal valve open until
   no bubbles leave the orifices; weigh the discharge, which is the purge
   volume the hose model asks for, against the hose's 29 mL); close the
   distal valve with the piston stopped, run the K path again:

   ```matlab
   opts.probe = true;
   ro1 = charRun('K300_hose', path, opts);
   ro2 = charRun('K300_hose_purged', path, opts);
   ```

   The difference of the two from `rk1` is the hose's compliance, before
   and after the purge: the purged value is the wall, the difference is
   the air, which is what the robustness study spans 0.12 to 0.36 mL/MPa
   for. Fit as section 3 shows (`Ch1`, `wall1`), with the fill reduced
   by the purge volume. The gas term of these runs less the closed
   chamber's pocket is the hose's air, a second reading of the same
   split.
7. **Orifice Cd**, three deliveries at held pressure with the discharge
   weighed (section 4), hose open as in service:

   ```matlab
   rq4  = charRun('Q4',  {{'pressure', 4, 15}, {'pressure', 4, 100}},  struct('posStop', x4));
   rq7  = charRun('Q7',  {{'pressure', 4, 15}, {'pressure', 7, 100}},  struct('posStop', x7));
   rq10 = charRun('Q10', {{'pressure', 4, 15}, {'pressure', 10, 100}}, struct('posStop', x10));
   rq = charOrificeFlow({rq4.file, rq7.file, rq10.file}, ...
       struct('K_Pa', rk1.K_Pa, 'fillVolume_mL', V, 'collected_mL', [m4 m7 m10] / 1.005));
   ```

   Set each `posStop` for 30 mL (8.7 mm) from the position at the
   target. Open the distal valve when the run prints that the pressure
   is in band, and close it only after the run has stopped the piston:
   with the GVL gains a closing against a moving piston peaks at
   12.3 MPa. For the hose model, time each closing with the stopwatch
   (`Tclose`), note the pause before the next opening, and look in the
   log for the 0.24 mL step at the next opening that says the tube
   drained (`tube_refills`). The deliveries also give loaded friction
   points at three pressures, in one direction.
8. **Second fill level, from what is left.** After the deliveries the
   chamber holds `V1` minus the purge and the three discharges, about
   190 mL of purged liquid. Shut the proximal valve and run the K path
   twice:

   ```matlab
   r4 = charRun('K200', path, opts);  r5 = charRun('K200', path, opts);
   rk = charBulkModulus({r1.file, r2.file, r3.file, r4.file, r5.file}, ...
       struct('fillVolume_mL', [V1 V1 V1 V2 V2]));
   rk.split
   ```

   `rk.split` separates the liquid's K from the series compliance of
   the seals and the actuator, which only two fill levels can do. `V2`
   is `V1` less the weighed volumes, so nothing is refilled.

## Session 2: one oxygen charge, about two and a half hours

Fresh fill as in session 1 step 1, then the 1:1 charge admitted at the
supply pressure as in the process, stir bar as in the process. Note the
supply gauge: `P_supply` spans 0.52 to 1.38 MPa with the cylinder's
state.

1. **Friction under the gas spring** (`bench/friction/README.md` 4.2),
   in the 0.5 to 3 MPa window before the charge dissolves:
   `frictionRun('g1', 'gas')`. Optional: skip it if session 1's loaded
   legs and session 0's unloaded set agree on the Coulomb term.
2. **Stiffness with free gas**: the K path with the probe, fitted with
   the charge:

   ```matlab
   rc = charRun('K300_o2', path, opts);
   rkc = charBulkModulus(rc.file, struct('fillVolume_mL', V1, 'chargeRatio', 1, ...
       'pressureRange', [0.5 11], 'minRows', 8));
   scg = charScheduleCheck(rc.file, struct('salineVolume_mL', V1, 'oxygenRatio', 1, ...
       'bulk', struct('pressureRange', [0.5 11], 'minRows', 8)));
   ```

   On the 30 and 31 October logs the free gas above 6 MPa was four to
   five times what the saturation model gives; this run says whether
   that is the model or the hold time.
3. **Dissolution time constant.** Compress to 10 MPa in pressure mode,
   then hold the position:

   ```matlab
   rd = charRun('diss10', {{'pressure', 10, 30}, {'hold', 900}}, opts);
   ```

   The pressure falls as the gas dissolves at constant volume; fit the
   trace to an exponential, after subtracting the liquid-only hold of
   session 1 (the leak). Repeat once with the stir bar off if the
   process ever runs without it; `tauDiss` sets the hold time of the
   cycle and is spanned 10 to 100 s in the robustness study, so a single
   value to 20 % narrows it tenfold.
4. **Hose opening at pressure** (eq. 43 of `models/README.md`). At the
   end of the hold, with the chamber at 10 MPa and the charge dissolved,
   arm a capture on the pressure channel and open the proximal valve by
   hand. The one-scan drop (the model says 3.6 MPa, to 6.4 MPa) and the
   loop's recovery are the hose model's check; a 1:1 charge keeps a
   2.8 MPa margin to degassing through it, so this is safe at 10 MPa and
   is also the first commissioning delivery at 1:1. Then run the
   delivery sequence of the process: distal valve open for the purge,
   weighing the discharge until solution appears (the degassed slug the
   hose model carries as 2 to 3 mL behind the 29 mL of virgin saline),
   closed with the piston stopped, the pause, reopened to `posStop`.
   Weigh the delivered solution. Do not run a 2:1 charge until the
   schedule is commissioned: it degasses at the opening.

## Carrying the results over

- `models/params.py`: `K`, `C_hose`, `Cd`, `t_dead`, `tauV` (the piston
  value if resolved), `tauDiss`, `Fc`, `b`, `mu`, `mr`, `noise`,
  `P_offset` (zero with the input calibrated), `Vl0` and `V_passages`
  from the weighed fill. A backlash or a windup that the reversal and
  dither runs resolve is a dead zone and a spring the model does not
  have; the stiffness bound (4.7 kN/um from the October ramps) is what
  keeps it out. `models/README.md` "Measured values" and the
  characterisation README's results table carry the run names.
- `design/pressure-loop/robust.py`: narrow the ranges to the new spread,
  then rerun `study_robust`, `study_compare` and `study_delivery` with
  the settled hose procedure.
- The GVL: `G_cfgPressureSchedBulkModulus`, `HoseCompliance` and
  `PassagesVolume` from the fits (upper bounds, as the schedule needs);
  the stop allowance (`G_cfgPressureDecel` and its delay) from
  `rs.T0_s` and `rs.D_mm_s2`; `masterConfig.deadVolume` if the passages
  changed with the fittings.
- `slave-sim/slaveSimConfig.m`: `chamberContactPos` from the cylinder
  entry, once the positions are verified (TODO.md, "Checks on the
  machine").
