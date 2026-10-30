# Plan: port the gain-scheduled pressure controller to the slave

Branch `sched-pressure-controller` (off `ccr-b45ed608-s25iyc`). This plan
covers steps 1 (the ST block, GVL, PRG_Main wiring) and 2 (translation,
parity, design-model equivalence test, Octave scenario, docs). Every design
decision is made here; where the plan says "exactly", copy it.

Read `AGENTS.md` first. Its conventions apply to everything below: header
comment declarations are functional input to st2m/st2py, comments describe
mechanism only (no dates, history or symptoms), `design/` code stays within
75 columns with units in `[ ]` and `eq. (N)` citations, docs change in the
same commit as behaviour.

Commit after each numbered section (§1 … §9) with a plain message, and push
`git push -u origin sched-pressure-controller` at the end (and whenever
convenient). Do not open a pull request. Delete this plan file in the last
commit or leave it; ask the parent session if unsure (default: leave it).

---

## 0. Facts the plan rests on (verified on this branch)

- Reference: `design/pressure-loop/controllers.py`, class
  `ScheduledSlaveController`. Per scan (`step`), in this order:
  1. `k = estimate_stiffness(P_meas, x)` [Pa/m], eq. (39); updates `P_ref`.
  2. `k_mm = k / 1e9`; `Kp = min(max(wc / k_mm, kp_min), kp_max)`;
     `Ki = ki_ratio * Kp`.
  3. `if fb.Ki > 0.0 and Ki != fb.Ki: fb.rInteg *= fb.Ki / Ki`, then
     `fb.Kp, fb.Ki = Kp, Ki`.
  4. `err = clamp(target*1e-6, 0, TargetMax) - P_meas*1e-6` [MPa];
     `fb.VelMax = min(vel_max, max(StopVelMin, v_stop(err, k_mm)))`, eq. (41),
     with `lag_run = FilterTimeConst + (medDelay + 3) * Ts + t_dead + tauV`,
     `medDelay = 2` if `G_cfgAnalogMedianSize >= 5` else `1`.
  5. `if not arrived and abs(err) < e_int: arrived = True`; `hold = not arrived`.
  6. `SlaveController.step`: `fb.Reset = first_scan`, Target/Actual set,
     block called, output clamped to `±G_cfgPressureVelMax`.
  7. `if hold: fb.rInteg = 0.0` (**after** the call).
  In `reset`: `fb.Stiffness = 1e-6` (block's own stop law off), `P_ref = 0`,
  `arrived = False`, `n_cfg = par.n_charge(ratio) = ratio*par.Vl0*Patm/(R*273.15)`
  (note: uses `par.Vl0`, not `V_l_cfg`), `RT = par.R*par.T = 8.314*293`,
  `x_limit = (Vch0 - par.V_passages)/par.A` in model coordinates (m, 0 at the
  run's start), where `Vch0 = Vl0*(1 - P0/K) + ng0*RT/(P0 + Patm)` from
  `scenario.initial(par)`.
- `FB_PressureController.st`: `rInteg` is a `VAR` (internal). IEC 61131-3 does
  not let PRG_Main write another POU's `VAR`, so the rescale **cannot** be done
  by PRG_Main writing `fbPressureCtrl.rInteg`. This plan adds an input to the
  block instead (§2).
- PRG_Main calls `fbPressureCtrl` once, near the end, after `fbPosGate`
  (`slave-src/PRG/PRG_Main.st` ~line 1130). The pressure state body
  (~line 382) sets `rPressureTarget` and `bPressureCtrlActive`.
- Translators: st2m and st2py share one tokenizer (`tools/st2m.py`
  `TOKEN_RE`; st2py imports `Translator` from st2m). **Bug:** a typed literal
  with a negative exponent, e.g. `LREAL#1.0E-6`, tokenizes as `1.0E - 6`
  (wrong code, silently). Untyped `1.0E-6` and positive exponents
  (`LREAL#1.0E9`) are fine. GVL defaults like `1.2E-13` parse fine (separate
  regex). §1 fixes the tokenizer; the fix was tried on a scratch copy and the
  regenerated `slave-sim/generated` was byte-identical to before.
- `st2py` runtime: `/` becomes `st_div` (IEEE result for a zero divisor) and
  `SQRT` becomes `st_sqrt` (NaN for negative), so random parity inputs cannot
  raise in Python; still guard every domain with `ParamError` (§3).
- `tests/st2pyParity.py` automatically drives **every** FB whose inputs are
  all scalar (BOOL/INT/LREAL/TIME), so `FB_PressureSchedule` is picked up
  without editing the test (800 calls; add it to `CORE` for 3000, §5).
- Position frame: mm, positive into the cylinder. `G_cfgPosSoftLimitMax` is
  305 mm in the reference GVL; the cylinder's end of travel (`posEOT`,
  `master-src/logic/masterConfig.m`) is 365 mm, where the chamber is down to
  the 50 mL passages. `A` = 3431 mm^2 (3.431 mL/mm).
- The Octave simulator's chamber (`slave-sim/axisModel.m`) is an isothermal
  gas spring: `P_abs = atm * V0 / vGas`, `V0 = sim.chamberGasVolume` =
  200e3 mm^3, contact at `sim.chamberContactPos` = 200 mm,
  `sim.pistonArea` = 3431 mm^2, `sim.atmMPa` = 0.101325. Its stiffness is
  `dP/dx = P_abs^2 * A / (atm * V0)` [MPa/mm], independent of `ax.vOut`.
- Baseline: `tests/run_all.sh` needs `pip install -r design/requirements.txt`
  (scipy) and `apt-get install -y octave`; the baseline run on this branch
  before any change is recorded in §8.

---

## 1. st2m tokenizer: typed literals with signed exponents

File `tools/st2m.py`, `TOKEN_RE`. Replace exactly

```
  | (?P<typed>[A-Za-z_]\w*\#[-+]?[A-Za-z0-9_.]+)
```

with

```
  | (?P<typed>[A-Za-z_]\w*\#[-+]?(?:\d+\.\d*[eE][-+]?\d+|[A-Za-z0-9_.]+))
```

No change in `st2py.py` is needed (it uses this tokenizer). Check:

```sh
python3 tools/st2m.py slave-src /tmp/a && git stash && python3 tools/st2m.py slave-src /tmp/b && git stash pop && diff -r /tmp/a /tmp/b   # identical
```

Mention the fix in the AGENTS.md st2m bullet? No: the supported subset
("typed/TIME literals") is unchanged in kind. Commit:
"st2m: tokenize typed real literals with a signed exponent".

---

## 2. `FB_PressureController`: `IntegScale` input

File `slave-src/FB/FB_PressureController.st`.

Header, `VAR_INPUT:` list, add after `CycleTime` (keep the column layout):

```
       IntegScale      : LREAL := 1.0     (* Factor applied to the stored integral at the start of the scan
```

Description block: add a paragraph after the "Anti-windup" paragraph:

```
   Gain changes: the integral term is Ki * rInteg, so a caller that changes
   Ki between scans passes IntegScale = old Ki / new Ki on that scan and the
   term stays continuous; otherwise IntegScale is 1. The scaling is applied
   before Reset clears the integrator.
```

Body: insert as the first statement after the `ParamError := ...;`
assignment, before `IF (NOT Enable) OR Reset OR ParamError THEN`:

```
rInteg := rInteg * IntegScale;
```

Nothing else changes. `SlaveController` and `ScheduledSlaveController` keep
working (st2py's `new_FB_PressureController()` sets `IntegScale = 1.0` from
the header default; the reference rescales `rInteg` itself and leaves it at 1).
MotionWorks: the variable must be declared in the GUI with initial value 1.0.

---

## 3. New block `slave-src/FB/FB_PressureSchedule.st`

Write the file exactly as below (adjust only whitespace if needed). It uses
only: assignments, IF/ELSE, RETURN, ABS, SQRT, MIN, MAX, LIMIT, typed
literals. No FB calls, no globals, no arrays, no TIME (st2py rejects blocks
that reference `G_*` globals, so every setting is an input).

```
(* ============================================================================
   FB_PressureSchedule - Gain Schedule for the Pressure Loop

   Project: Gen3 SOS Compressor Servo Controller
   Controller: Yaskawa MP2600iec

   Description:
   Sets FB_PressureController's gains and stop-law limit every scan from an
   upper bound on the chamber stiffness (design/pressure-loop/README.md,
   equations (39) to (41)). PRG_Main calls it just before fbPressureCtrl and
   passes its outputs to that block when G_cfgPressureScheduleEnable was set
   on entry to pressure mode.

   Stiffness bound, eq. (39): the chamber volume is the swept volume from the
   piston to PosLimit plus DeadVolume. The free gas is the least there can
   be: the configured oxygen charge less what the configured saline volume
   can hold at the highest absolute pressure seen since Reset. The saline
   volume is the configured one, or the chamber volume when that is smaller.
   With BulkModulus the largest plausible and HoseCompliance the smallest,
     C = Vgas / Pabs + min(Vsaline, Vchamber) / BulkModulus + HoseCompliance
     KEst = PistonArea / C
   is never below the true stiffness while SalineVolume and DeadVolume are
   at or below the true values.

   Gains, eq. (40): Kp = LIMIT(KpMin, Crossover / KEst, KpMax) and
   Ki = KiRatio * Kp, so the loop's crossover is at most Crossover.

   Stop law, eq. (41): the piston travels for Lag at its speed and then
   decelerates at DecelMax, and must stop within the error at KEst:
     v = -DecelMax * Lag + SQRT((DecelMax * Lag)^2 + 2 * DecelMax * |e| / KEst)
   VelLimit = MIN(VelMax, MAX(StopVelMin, v)) is given to the controller as
   its VelMax, and Stiffness (a negligible value) disables the controller's
   own stop law. Lag is the measurement chain (filter time constant, median
   and pipeline scans, the scan in which the command takes effect) plus the
   drive's dead time and velocity-loop lag.

   Integrator: the controller multiplies its stored integral by Ki, so on a
   scan where Ki changes IntegScale = previous Ki / new Ki keeps the integral
   term continuous (FB_PressureController.IntegScale). IntegHold is TRUE until
   the error first comes within ArriveBand, and on the scan it does; PRG_Main
   ORs it into the controller's Reset, so the integrator is zero at the start
   of every scan up to and including the arrival scan.

   Reset starts a new run (highest pressure, arrival and previous Ki
   cleared) and computes this scan normally. Enable FALSE or ParamError
   clears the run state, zeroes the outputs and sets IntegScale to 1.

   Variables (define in MWiec GUI):
     VAR_INPUT:
       Enable          : BOOL             (* Compute the schedule; FALSE clears the run state
       Reset           : BOOL             (* Start a new run this scan
       Target          : LREAL            (* Target pressure, clamped by the caller (MPa gauge)
       Actual          : LREAL            (* Measured pressure (MPa gauge)
       Position        : LREAL            (* Piston position (mm), positive into the cylinder
       PosLimit        : LREAL            (* Position at which the chamber is down to DeadVolume (mm)
       VelMax          : LREAL            (* Velocity clamp in pressure mode (mm/s)
       DecelMax        : LREAL            (* Deceleration assumed by the stop law (mm/s^2)
       StopVelMin      : LREAL            (* The stop law never clamps below this (mm/s)
       CycleTime       : LREAL            (* Task period (s)
       FilterTime      : LREAL            (* Pressure input low-pass time constant (s)
       MedianSize      : INT              (* Pressure input median window (3 or 5)
       DriveDeadTime   : LREAL            (* Drive dead time from command to response (s)
       DriveLag        : LREAL            (* Drive velocity-loop time constant (s)
       Crossover       : LREAL            (* Target loop crossover (rad/s)
       KiRatio         : LREAL            (* Ki / Kp (1/s)
       KpMin           : LREAL            (* Lower bound on Kp (mm/s per MPa)
       KpMax           : LREAL            (* Upper bound on Kp (mm/s per MPa)
       PistonArea      : LREAL            (* Piston area (mm^2)
       SalineVolume    : LREAL            (* Saline volume, a lower bound (mL)
       DeadVolume      : LREAL            (* Chamber volume at PosLimit, a lower bound (mL)
       HoseCompliance  : LREAL            (* Hose compliance, a lower bound (m^3/Pa)
       BulkModulus     : LREAL            (* Saline bulk modulus, an upper bound (Pa)
       OxygenRatio     : LREAL            (* Oxygen charge as STP volume per saline volume (-)
       Henry           : LREAL            (* Henry constant of oxygen in saline (mol/(m^3 Pa))
       GasTemp         : LREAL            (* Gas temperature (K)
       Patm            : LREAL            (* Atmospheric pressure (Pa)
       ArriveBand      : LREAL            (* |error| that releases the integrator (MPa)
     VAR_OUTPUT:
       Kp              : LREAL            (* Scheduled proportional gain (mm/s per MPa)
       Ki              : LREAL            (* Scheduled integral gain (mm/s per (MPa * s))
       VelLimit        : LREAL            (* Velocity clamp for the controller's VelMax (mm/s)
       Stiffness       : LREAL            (* Stiffness for the controller's own stop law (MPa/mm)
       IntegScale      : LREAL            (* Factor for the controller's stored integral this scan
       IntegHold       : BOOL             (* Hold the controller's integrator at zero this scan
       KEst            : LREAL            (* Stiffness bound (MPa/mm), for diagnostics
       Lag             : LREAL            (* Lag assumed by the stop law (s), for diagnostics
       ParamError      : BOOL             (* A setting is out of range; outputs are zero
     VAR:
       bArrived        : BOOL := FALSE    (* The error has come within ArriveBand since Reset
       rPRef           : LREAL := 0.0     (* Highest absolute pressure since Reset (Pa)
       rKiPrev         : LREAL := 0.0     (* Ki output on the previous scan; 0 after Reset
       rMedScans       : LREAL            (* Median filter delay (scans)
       rPAbs           : LREAL            (* Absolute pressure (Pa)
       rArea           : LREAL            (* Piston area (m^2)
       rVch            : LREAL            (* Chamber volume (m^3)
       rVlCfg          : LREAL            (* Configured saline volume (m^3)
       rVl             : LREAL            (* Saline volume used by the bound (m^3)
       rNCharge        : LREAL            (* Oxygen charge (mol)
       rNSat           : LREAL            (* Oxygen the saline holds at rPRef (mol)
       rNGas           : LREAL            (* Free gas (mol)
       rRT             : LREAL            (* Gas constant times temperature (J/mol)
       rVGas           : LREAL            (* Free gas volume (m^3)
       rC              : LREAL            (* Chamber compliance (m^3/Pa)
       rErr            : LREAL            (* Target - Actual (MPa)
       rDist           : LREAL            (* Travel that closes the error at KEst (mm)
       rStep           : LREAL            (* Velocity lost over Lag at DecelMax (mm/s)
       rVelStop        : LREAL            (* Stop-law velocity (mm/s)
   ============================================================================ *)

ParamError := (VelMax <= LREAL#0.0) OR (DecelMax <= LREAL#0.0) OR
              (StopVelMin < LREAL#0.0) OR (CycleTime <= LREAL#0.0) OR
              (FilterTime < LREAL#0.0) OR (DriveDeadTime < LREAL#0.0) OR
              (DriveLag < LREAL#0.0) OR (Crossover <= LREAL#0.0) OR
              (KiRatio < LREAL#0.0) OR (KpMin <= LREAL#0.0) OR
              (KpMax < KpMin) OR (PistonArea <= LREAL#0.0) OR
              (SalineVolume <= LREAL#0.0) OR (DeadVolume <= LREAL#0.0) OR
              (HoseCompliance < LREAL#0.0) OR (BulkModulus <= LREAL#0.0) OR
              (OxygenRatio < LREAL#0.0) OR (Henry < LREAL#0.0) OR
              (GasTemp <= LREAL#0.0) OR (Patm <= LREAL#0.0) OR
              (ArriveBand < LREAL#0.0);

IF (NOT Enable) OR Reset OR ParamError THEN
    bArrived := FALSE;
    rPRef := LREAL#0.0;
    rKiPrev := LREAL#0.0;
END_IF;

IF (NOT Enable) OR ParamError THEN
    Kp := LREAL#0.0;
    Ki := LREAL#0.0;
    VelLimit := LREAL#0.0;
    Stiffness := LREAL#0.0;
    IntegScale := LREAL#1.0;
    IntegHold := FALSE;
    KEst := LREAL#0.0;
    Lag := LREAL#0.0;
    RETURN;
END_IF;

(* --------------------------------------------------------------------------
   Lag of the measurement and the drive
   -------------------------------------------------------------------------- *)
IF MedianSize >= 5 THEN
    rMedScans := LREAL#2.0;
ELSE
    rMedScans := LREAL#1.0;
END_IF;
Lag := FilterTime + (rMedScans + LREAL#3.0) * CycleTime + DriveDeadTime + DriveLag;

(* --------------------------------------------------------------------------
   Stiffness bound, eq. (39), in SI units
   -------------------------------------------------------------------------- *)
rPAbs := MAX(Actual, LREAL#0.0) * LREAL#1.0E6 + Patm;
rPRef := MAX(rPRef, rPAbs);
rArea := PistonArea / LREAL#1.0E6;
rVch := MAX(rArea * ((PosLimit - Position) / LREAL#1.0E3), LREAL#0.0)
        + DeadVolume / LREAL#1.0E6;
rVlCfg := SalineVolume / LREAL#1.0E6;
rVl := MIN(rVlCfg, rVch);
rNCharge := OxygenRatio * rVlCfg * Patm / (LREAL#8.314 * LREAL#273.15);
rNSat := Henry * rPRef * rVlCfg;
rNGas := MAX(rNCharge - rNSat, LREAL#0.0);
rRT := LREAL#8.314 * GasTemp;
rVGas := rNGas * rRT / rPAbs;
rC := rVGas / rPAbs + rVl / BulkModulus + HoseCompliance;
KEst := rArea / rC / LREAL#1.0E9;

(* --------------------------------------------------------------------------
   Gains, eq. (40), and the integral rescale on a change of Ki
   -------------------------------------------------------------------------- *)
Kp := LIMIT(KpMin, Crossover / KEst, KpMax);
Ki := KiRatio * Kp;
IF (rKiPrev > LREAL#0.0) AND (Ki > LREAL#0.0) AND (Ki <> rKiPrev) THEN
    IntegScale := rKiPrev / Ki;
ELSE
    IntegScale := LREAL#1.0;
END_IF;
rKiPrev := Ki;

(* --------------------------------------------------------------------------
   Stop law with the lag, eq. (41)
   -------------------------------------------------------------------------- *)
rErr := Target - Actual;
rDist := ABS(rErr) / KEst;
rStep := DecelMax * Lag;
rVelStop := -rStep + SQRT(rStep * rStep + LREAL#2.0 * DecelMax * rDist);
VelLimit := MIN(VelMax, MAX(StopVelMin, rVelStop));
Stiffness := LREAL#1.0E-6;

(* --------------------------------------------------------------------------
   Integrator hold until the first arrival
   -------------------------------------------------------------------------- *)
IntegHold := NOT bArrived;
IF (NOT bArrived) AND (ABS(rErr) < ArriveBand) THEN
    bArrived := TRUE;
END_IF;
```

Why `IntegHold := NOT bArrived` is taken **before** the arrival update: the
reference zeroes `rInteg` *after* each held call, so `rInteg` is 0 at the
start of every held scan **and of the arrival scan**. Driving the
controller's `Reset` (which clears at the *start* of the scan) on held scans
and on the arrival scan reproduces that exactly; using the post-update value
would leave one scan's `Error * CycleTime` in the integrator on the arrival
scan and the §6 test would fail at 1e-9.

Why `KEst := rArea / rC / 1.0E9`: the reference computes `k = A / C` [Pa/m]
then `k / 1e9` [MPa/mm]; left-to-right `/` gives the same operations.

Check after writing (and after §4):

```sh
python3 tools/st2m.py slave-src slave-sim/generated
python3 tools/st2py.py slave-src design/generated
grep -n "1.0E-6" slave-sim/generated/FB_PressureSchedule.m design/generated/FB_PressureSchedule.py   # one hit each, no "E - 6"
```

Commit: "slave: FB_PressureSchedule and FB_PressureController.IntegScale".

---

## 4. GVL and PRG_Main

### 4a. `slave-src/GVL/GlobalVariables_Reference.st`

Insert after the line
`     G_cfgPressureInBand         : LREAL := 0.1        (* |actual - target| for the in-band status output (MPa) *)`
(still inside the same `VAR_GLOBAL`, before the TIMING PARAMETERS banner):

```

     (* Gain schedule (FB_PressureSchedule). When enabled, latched on entry to
        pressure mode, Kp, Ki and the stop law follow an upper bound on the
        chamber stiffness instead of G_cfgPressureKp/Ki/StopStiffness above,
        which remain the fallback. *)
     G_cfgPressureScheduleEnable     : BOOL := FALSE       (* Schedule the pressure-loop gains on the stiffness bound *)
     G_cfgPressureSchedCrossover     : LREAL := 11.0       (* Target loop crossover wc (rad/s) *)
     G_cfgPressureSchedKiRatio       : LREAL := 0.4        (* Ki / Kp (1/s) *)
     G_cfgPressureSchedKpMin         : LREAL := 0.001      (* Lower bound on the scheduled Kp (mm/s per MPa) *)
     G_cfgPressureSchedKpMax         : LREAL := 5.0        (* Upper bound on the scheduled Kp (mm/s per MPa) *)
     G_cfgPressureSchedPistonArea    : LREAL := 3431.0     (* Piston area (mm^2), 2.602 in bore *)
     G_cfgPressureSchedSalineVolume  : LREAL := 343.0      (* Saline volume of the fill including the dead volume (mL);
                                                              a lower bound on the true fill *)
     G_cfgPressureSchedDeadVolume    : LREAL := 74.0       (* Chamber volume with the piston at G_cfgPosSoftLimitMax and
                                                              the hose open (mL); a lower bound. 74 mL is the lower end of
                                                              the 79 mL measured at the end of travel *)
     G_cfgPressureSchedHoseCompliance: LREAL := 1.2E-13    (* Hose compliance (m^3/Pa), a lower bound; 0 if the hose
                                                              is shut during the compression *)
     G_cfgPressureSchedBulkModulus   : LREAL := 1.0E9      (* Saline bulk modulus (Pa), an upper bound *)
     G_cfgPressureSchedOxygenRatio   : LREAL := 1.0        (* Oxygen charge as STP volume per saline volume (-) *)
     G_cfgPressureSchedHenry         : LREAL := 1.22E-5    (* Henry constant of oxygen in 0.9 % saline (mol/(m^3 Pa)) *)
     G_cfgPressureSchedGasTemp       : LREAL := 293.0      (* Gas temperature (K) *)
     G_cfgPressureSchedPatm          : LREAL := 101325.0   (* Atmospheric pressure (Pa) *)
     G_cfgPressureSchedArriveBand    : LREAL := 0.5        (* |error| that first releases the integrator (MPa) *)
     G_cfgPressureSchedDriveDeadTime : LREAL := 0.012      (* Drive dead time, command to response (s), measured *)
     G_cfgPressureSchedDriveLag      : LREAL := 0.0087     (* Drive velocity-loop time constant (s), measured *)
```

Check the GVL parser reads them:
`python3 -c "import sys; sys.path[:0]=['design/pressure-loop']; import slave; c=slave.slave_config(); print({k:v for k,v in c.items() if 'Sched' in k})"`
must list all 17 with the right values (`G_cfgPressureSchedHoseCompliance`
= 1.2e-13, `...ScheduleEnable` = False).

Limit position decision: `PosLimit := G_cfgPosSoftLimitMax`. The piston halts
at the soft limit, and the soft limit is never past the end of travel, so the
estimated chamber volume `A (SoftLimitMax - x) + DeadVolume` is at most the
true one when DeadVolume is the volume at the end of travel: the bound stays
an upper bound (safe, lower gain). With the reference file's 305 mm against
the 365 mm end of travel the bound is stiffer than the chamber late in a
delivery (lower gain only); setting `DeadVolume` to the volume at the soft
limit (74 + 3.431 * (365 - SoftLimitMax) mL) removes that. Document this in
the ConfigurationGuide (§9). No new position setting.

### 4b. `slave-src/PRG/PRG_Main.st`

1. Header `VAR:` list, after `fbPressureCtrl      : FB_PressureController`
   add:

   ```
       fbPressureSched     : FB_PressureSchedule
   ```

2. Header, after
   `       rAnalogFilterAlpha  : LREAL := 0.0`
   add
   ```
       rAnalogFilterTime   : LREAL := 0.0     (* G_cfgAnalogFilterTimeConst in seconds, for fbPressureSched
   ```

3. Header, after
   `       bPressureCtrlActive : BOOL             (* Set by the pressure state body this scan; gates fbPressureCtrl.Enable`
   add
   ```
       bPressureSchedActive : BOOL            (* G_cfgPressureScheduleEnable latched on entry to pressure mode
       bPressureSchedUsed  : BOOL             (* fbPressureSched drives fbPressureCtrl's gains this scan
       rPressureKp         : LREAL            (* Gains and limits passed to fbPressureCtrl this scan:
       rPressureKi         : LREAL            (*   scheduled, or the G_cfgPressure* values
       rPressureVelMax     : LREAL
       rPressureStiffness  : LREAL
       rPressureIntegScale : LREAL := 1.0
   ```
   (st2m skips header lines without a `(*` comment just as well; keep each on
   one line.)

4. First-scan block. Replace

   ```
       IF G_cfgAnalogFilterTimeConst > T#0MS THEN
           rAnalogFilterAlpha := G_cfgScanTime / ((DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / LREAL#1000.0) + G_cfgScanTime);
   ```
   with
   ```
       rAnalogFilterTime := DINT_TO_LREAL(TIME_TO_DINT(G_cfgAnalogFilterTimeConst)) / LREAL#1000.0;
       IF G_cfgAnalogFilterTimeConst > T#0MS THEN
           rAnalogFilterAlpha := G_cfgScanTime / (rAnalogFilterTime + G_cfgScanTime);
   ```
   (same arithmetic for alpha).

5. Pressure state body. Replace
   ```
           IF bEnterState THEN
               G_sysConfirmedMode := E_OperatingMode#MODE_PRESSURE;
               stCurrentLimits.VelMax := G_cfgPressureVelMax;
               stCurrentLimits.TorqueMax := G_cfgPressureTorqueLimit;
           END_IF;
   ```
   with
   ```
           IF bEnterState THEN
               G_sysConfirmedMode := E_OperatingMode#MODE_PRESSURE;
               stCurrentLimits.VelMax := G_cfgPressureVelMax;
               stCurrentLimits.TorqueMax := G_cfgPressureTorqueLimit;
               (* The gain schedule is chosen once per entry, so a change of
                  the switch never changes the gains mid-run. *)
               bPressureSchedActive := G_cfgPressureScheduleEnable;
           END_IF;
   ```

6. FB-call section. Replace the whole existing block

   ```
   (* Pressure loop - enabled only on scans where the pressure state body ran,
      so the transition scan commands zero velocity. Reset on entry clears the
      integrator. *)
   fbPressureCtrl(
       Enable := bPressureCtrlActive,
       Reset := bEnterState AND bPressureCtrlActive,
       Target := rPressureTarget,
       Actual := G_sysActualPressure,
       Kp := G_cfgPressureKp,
       Ki := G_cfgPressureKi,
       VelMax := stCurrentLimits.VelMax,
       DecelMax := G_cfgPressureDecel,
       Stiffness := G_cfgPressureStopStiffness,
       StopVelMin := G_cfgPressureStopVelMin,
       CycleTime := G_cfgScanTime
   );
   ```

   with

   ```
   (* Pressure-loop gain schedule - Kp, Ki and the stop-law limit from the
      stiffness bound, when selected on entry. A ParamError falls back to the
      configured gains. *)
   fbPressureSched(
       Enable := bPressureCtrlActive AND bPressureSchedActive,
       Reset := bEnterState AND bPressureCtrlActive,
       Target := rPressureTarget,
       Actual := G_sysActualPressure,
       Position := G_sysActualPosition,
       PosLimit := G_cfgPosSoftLimitMax,
       VelMax := stCurrentLimits.VelMax,
       DecelMax := G_cfgPressureDecel,
       StopVelMin := G_cfgPressureStopVelMin,
       CycleTime := G_cfgScanTime,
       FilterTime := rAnalogFilterTime,
       MedianSize := G_cfgAnalogMedianSize,
       DriveDeadTime := G_cfgPressureSchedDriveDeadTime,
       DriveLag := G_cfgPressureSchedDriveLag,
       Crossover := G_cfgPressureSchedCrossover,
       KiRatio := G_cfgPressureSchedKiRatio,
       KpMin := G_cfgPressureSchedKpMin,
       KpMax := G_cfgPressureSchedKpMax,
       PistonArea := G_cfgPressureSchedPistonArea,
       SalineVolume := G_cfgPressureSchedSalineVolume,
       DeadVolume := G_cfgPressureSchedDeadVolume,
       HoseCompliance := G_cfgPressureSchedHoseCompliance,
       BulkModulus := G_cfgPressureSchedBulkModulus,
       OxygenRatio := G_cfgPressureSchedOxygenRatio,
       Henry := G_cfgPressureSchedHenry,
       GasTemp := G_cfgPressureSchedGasTemp,
       Patm := G_cfgPressureSchedPatm,
       ArriveBand := G_cfgPressureSchedArriveBand
   );

   bPressureSchedUsed := bPressureCtrlActive AND bPressureSchedActive AND
                         NOT fbPressureSched.ParamError;
   IF bPressureSchedUsed THEN
       rPressureKp := fbPressureSched.Kp;
       rPressureKi := fbPressureSched.Ki;
       rPressureVelMax := fbPressureSched.VelLimit;
       rPressureStiffness := fbPressureSched.Stiffness;
       rPressureIntegScale := fbPressureSched.IntegScale;
   ELSE
       rPressureKp := G_cfgPressureKp;
       rPressureKi := G_cfgPressureKi;
       rPressureVelMax := stCurrentLimits.VelMax;
       rPressureStiffness := G_cfgPressureStopStiffness;
       rPressureIntegScale := LREAL#1.0;
   END_IF;

   (* Pressure loop - enabled only on scans where the pressure state body ran,
      so the transition scan commands zero velocity. Reset on entry clears the
      integrator; under the schedule, so does the hold until the first
      arrival. *)
   fbPressureCtrl(
       Enable := bPressureCtrlActive,
       Reset := (bEnterState AND bPressureCtrlActive) OR
                (bPressureSchedUsed AND fbPressureSched.IntegHold),
       Target := rPressureTarget,
       Actual := G_sysActualPressure,
       Kp := rPressureKp,
       Ki := rPressureKi,
       VelMax := rPressureVelMax,
       DecelMax := G_cfgPressureDecel,
       Stiffness := rPressureStiffness,
       StopVelMin := G_cfgPressureStopVelMin,
       CycleTime := G_cfgScanTime,
       IntegScale := rPressureIntegScale
   );
   ```

   Every input of both blocks is passed on the single call (IEC: an omitted
   input keeps its previous value, which st2m reproduces). If st2m rejects a
   multi-line input expression, put `Reset` into a local BOOL
   (`bPressureCtrlReset`, header-declared) computed just above the call.

Regenerate and run the existing MATLAB-language tests: with the switch FALSE
by default, `closedLoopMasterSlave`, `closedLoopModbusStep` and
`slavePressureMode` must pass unchanged (the fallback path is arithmetically
the old call, `IntegScale` = 1).

Commit: "slave: wire FB_PressureSchedule in pressure mode behind G_cfgPressureScheduleEnable".

---

## 5. Parity

`tests/st2pyParity.py`: add `'FB_PressureSchedule'` to `CORE` (3000 calls).
`MedianSize` is INT (drawn from 0..7, -1), the BOOLs toggle, `Target`,
`Actual`, `Position` are in `SIGNALS` (±12), the rest are parameters with
nominal 0.1–10 and edge jumps to 0, -1, 1e-9, 1e6, which exercise
`ParamError` and the zero-divisor paths. Run `python3 tests/st2pyParity.py`;
every block must print `ok`. Commit with §4 or separately.

---

## 6. Design model: `ScheduledSlaveControllerST` and its test

### 6a. `design/pressure-loop/controllers.py`

- Header docstring: add a paragraph after the ScheduledSlaveController one:
  "ScheduledSlaveControllerST runs the slave's FB_PressureSchedule and
  FB_PressureController, both translated from Structured Text, wired as
  PRG_Main wires them, with the schedule's settings read from the GVL. It is
  the port of ScheduledSlaveController and matches it scan for scan." Add
  `FB_PressureSchedule.py` under FILE DEPENDENCY.
- Import: `from FB_PressureSchedule import (FB_PressureSchedule, new_FB_PressureSchedule)`.
- Refactor the limit-position lines out of `ScheduledSlaveController.reset`
  into a module-level function placed just before that class (behaviour
  unchanged):

  ```python
  def limit_position(par, scenario):
      """[m] Position, in the run's coordinates, at which the chamber is
      down to the cylinder head's passages (the limit switch)."""
      RT = par.R * par.T                                      # [J/mol]
      P0, ng0, nd0 = scenario.initial(par)
      Vch0 = (par.Vl0 * (1 - P0 / par.K)
              + ng0 * RT / (P0 + par.Patm))                   # [m^3]
      return (Vch0 - par.V_passages) / par.A                  # [m]
  ```
  and use `self.x_limit = limit_position(par, scenario)` in the reference
  (same operations in the same order as now, so bit-identical). Run
  `test_model.py` before and after the refactor: same printed numbers.

- New class, after `ScheduledSlaveController`:

  ```python
  # %% ===== SCHEDULED SLAVE CONTROLLER, STRUCTURED TEXT ==================
  class ScheduledSlaveControllerST(SlaveController):
      """FB_PressureSchedule and FB_PressureController wired as PRG_Main
      wires them, eqs. (39) to (41).

      The schedule's settings are the G_cfgPressureSched* values of the
      GVL, with overrides by full or short name as for SlaveController
      (for example SchedCrossover=6.0). The schedule is used when
      G_cfgPressureScheduleEnable is TRUE, which the constructor sets
      unless ScheduleEnable=False is given; otherwise the block runs on
      the configured gains, as PRG_Main does.

      The slave measures the piston position in its own frame, where the
      chamber is down to the dead volume at G_cfgPosSoftLimitMax. The
      run's position x is mapped so that the distance to that limit is
      the distance to the run's limit-switch position.
      """

      def __init__(self, name='scheduled-st', flow_ff=False, **overrides):
          overrides.setdefault('ScheduleEnable', True)
          super().__init__(name, flow_ff, **overrides)

      def reset(self, par, scenario):
          u0 = super().reset(par, scenario)
          c = self.c
          self.active = c['G_cfgPressureScheduleEnable']
          self.x_limit = limit_position(par, scenario)        # [m]
          self.pos_limit = c['G_cfgPosSoftLimitMax']           # [mm]
          s = new_FB_PressureSchedule()
          # Settings, as PRG_Main passes them
          s.PosLimit = self.pos_limit                          # [mm]
          s.VelMax = self.vel_max                              # [mm/s]
          s.DecelMax = c['G_cfgPressureDecel']                 # [mm/s^2]
          s.StopVelMin = c['G_cfgPressureStopVelMin']          # [mm/s]
          s.CycleTime = c['G_cfgScanTime']                     # [s]
          s.FilterTime = c['G_cfgAnalogFilterTimeConst']       # [s]
          s.MedianSize = c['G_cfgAnalogMedianSize']            # [-]
          s.DriveDeadTime = c['G_cfgPressureSchedDriveDeadTime']  # [s]
          s.DriveLag = c['G_cfgPressureSchedDriveLag']         # [s]
          ... one line per remaining input:
          Crossover, KiRatio, KpMin, KpMax, PistonArea, SalineVolume,
          DeadVolume, HoseCompliance, BulkModulus, OxygenRatio, Henry,
          GasTemp, Patm, ArriveBand  <- c['G_cfgPressureSched<Name>']
          self.sched = s
          self.k_est = math.nan                                # [Pa/m]
          return u0

      def step(self, P_meas, target, t, scenario, x=0.0):
          fb, s, c = self.fb, self.sched, self.c
          entry = self.first_scan
          self.first_scan = False
          # PRG_Main limits the target to [0, TargetMax]. [Pa -> MPa]
          tgt = min(max(target * 1e-6, 0.0), self.target_max)
          act = P_meas * 1e-6                                  # [Pa -> MPa]
          # Slave position: the same distance to the limit [m -> mm]
          pos = self.pos_limit - (self.x_limit - x) * 1e3
          s.Enable, s.Reset = self.active, entry
          s.Target, s.Actual, s.Position = tgt, act, pos
          FB_PressureSchedule(s, self.dt)
          used = self.active and not s.ParamError
          self.k_est = s.KEst * 1e9            # [MPa/mm -> Pa/m]
          fb.Enable = True
          fb.Reset = entry or (used and s.IntegHold)
          if used:
              fb.Kp, fb.Ki = s.Kp, s.Ki
              fb.VelMax, fb.Stiffness = s.VelLimit, s.Stiffness
              fb.IntegScale = s.IntegScale
          else:
              fb.Kp = c['G_cfgPressureKp']
              fb.Ki = c['G_cfgPressureKi']
              fb.VelMax = self.vel_max
              fb.Stiffness = c['G_cfgPressureStopStiffness']
              fb.IntegScale = 1.0
          fb.Target, fb.Actual = tgt, act
          FB_PressureController(fb, self.dt)
          # PRG_Main clamps the block output to VelMax. [mm/s]
          v = min(max(fb.Velocity, -self.vel_max), self.vel_max)
          return v * 1e-3                                  # [mm/s -> m/s]
  ```

  Notes: `SlaveController.reset` already builds `self.fb` with the GVL
  settings; `step` does **not** call `SlaveController.step` (it must order
  the calls as PRG_Main does). Ignore `flow_ff` (pass-through to the base
  only). Keep the 75-column limit and unit comments.

### 6b. `design/pressure-loop/test_model.py`

Add `FB_PressureSchedule.py` to the FILE DEPENDENCY list and a function after
`check_scheduled`, called from `__main__` right after `check_scheduled()`:

```python
def check_scheduled_st():
    par = Params()
    cfg = slave_config()
    # The reference takes the plant's values where the slave has settings;
    # they must agree for the two to be compared.
    assert abs(par.A - cfg['G_cfgPressureSchedPistonArea'] * 1e-6) < 1e-15
    assert abs(par.Vl0 - cfg['G_cfgPressureSchedSalineVolume'] * 1e-6) < 1e-15
    assert par.t_dead == cfg['G_cfgPressureSchedDriveDeadTime']
    assert par.tauV == cfg['G_cfgPressureSchedDriveLag']
    assert (par.kH, par.T, par.Patm) == (cfg['G_cfgPressureSchedHenry'],
        cfg['G_cfgPressureSchedGasTemp'], cfg['G_cfgPressureSchedPatm'])

    def reference():
        return ctl.ScheduledSlaveController(
            'ref', wc=cfg['G_cfgPressureSchedCrossover'],
            K_cfg=cfg['G_cfgPressureSchedBulkModulus'],
            V_l_cfg=cfg['G_cfgPressureSchedSalineVolume'] * 1e-6,
            V_dead_cfg=cfg['G_cfgPressureSchedDeadVolume'] * 1e-6,
            C_hose_cfg=cfg['G_cfgPressureSchedHoseCompliance'],
            ratio_cfg=cfg['G_cfgPressureSchedOxygenRatio'],
            ki_ratio=cfg['G_cfgPressureSchedKiRatio'],
            kp_min=cfg['G_cfgPressureSchedKpMin'],
            kp_max=cfg['G_cfgPressureSchedKpMax'],
            e_int=cfg['G_cfgPressureSchedArriveBand'])

    class Lockstep:
        """Runs the reference and the port on the same inputs and
        returns the reference's command."""
        mode = 'V'

        def __init__(self, ref, port):
            self.ref, self.port = ref, port
            self.cfg = port.cfg

        def reset(self, p, sc):
            u = self.ref.reset(p, sc)
            assert self.port.reset(p, sc) == u
            self.accel, self.decel = self.ref.accel, self.ref.decel
            self.worst, self.kp = 0.0, []
            return u

        def step(self, P, tg, t, sc, x=0.0):
            a = self.ref.step(P, tg, t, sc, x)
            b = self.port.step(P, tg, t, sc, x)
            self.worst = max(self.worst, abs(a - b))
            self.kp.append(self.port.fb.Kp)
            return a

    q = par.with_(hose_before=True)     # the hose is open throughout
    for name in ('compress', 'deliver'):
        L = Lockstep(reference(), ctl.ScheduledSlaveControllerST())
        r = sim.run(q, scenarios.make(name, q), L)
        assert L.worst < 1e-9, (name, L.worst)            # [m/s]
        assert L.port.sched.Enable and not L.port.sched.ParamError
        assert not L.port.sched.IntegHold                  # arrived
        assert max(L.kp) > 1.5 * min(L.kp)                 # scheduled
        ok(f'FB_PressureSchedule (ST) matches ScheduledSlaveController '
           f'scan for scan through {name} (largest command difference '
           f'{L.worst:.1e} m/s, Kp {min(L.kp):.3f} to {max(L.kp):.3f})')

    # With the switch off the port is the GVL loop.
    sc = scenarios.stiffstep(par)
    a, b = ctl.SlaveController(), ctl.ScheduledSlaveControllerST(
        ScheduleEnable=False)
    a.reset(par, sc), b.reset(par, sc)
    rng = random.Random(2)
    for k in range(2000):
        P = 10e6 + 2e6 * math.sin(k / 100.0) + rng.uniform(-1e5, 1e5)
        assert a.step(P, 10.5e6, k * par.Ts, sc) == \
            b.step(P, 10.5e6, k * par.Ts, sc)
    ok('ScheduledSlaveControllerST with the switch off is the GVL loop')
```

Adjust only for style (75 columns). `Lockstep.cfg = port.cfg` makes the
sensor chain read the same G_cfg overrides as the port (`sim.make_sensor`
uses `controller.cfg`); the reference's `cfg` is `{}`, and `ScheduleEnable`
does not touch the sensor, so both see the same chain. `sim.run` creates
the plant **after** `reset`, so `accel`/`decel` set in `reset` are read.

If `L.worst` fails, debug by logging both controllers' `fb.Kp`, `fb.Ki`,
`fb.VelMax`, `fb.rInteg` per scan and finding the first scan that differs:
- first scan, or the arrival scan → the hold timing (§3 note);
- the first Ki change after arrival → `IntegScale`;
- a slow drift → a units/ordering slip in eq. (39) or the position map.

Runtime: deliver is 200 s of simulated time; this check doubles the controller
cost of one run and is comparable to `check_closed_loop`.

Commit: "design: ScheduledSlaveControllerST runs the ported block; lockstep check".

---

## 7. Octave scenario in `tests/slavePressureMode.m`

Update the function's help text: add "and, with the gain schedule
(G_cfgPressureScheduleEnable), compress to 10 MPa with the gains following
the stiffness bound".

Insert before `fprintf('all passed\n');` (the previous scenario leaves the
slave in ST_BRAKE_HOLD, from which `enterMode` works):

```matlab
% Gain schedule: with the estimate's settings describing the simulated gas
% spring (no saline, no dissolution, no hose, its gas as the charge), the
% stiffness bound is the chamber's stiffness. The gains follow wc / k, the
% integrator is held until the first arrival, the pressure settles in band
% without overshoot, and the switch is latched on entry.
sim = h.sl.sim;
h.sl.G.G_cfgPressureScheduleEnable = true;
h.sl.G.G_cfgPressureSchedSalineVolume = 1.0;          % mL, negligible liquid
h.sl.G.G_cfgPressureSchedHenry = 0;
h.sl.G.G_cfgPressureSchedHoseCompliance = 0;
h.sl.G.G_cfgPressureSchedBulkModulus = 1e12;
h.sl.G.G_cfgPressureSchedPistonArea = sim.pistonArea;
h.sl.G.G_cfgPressureSchedPatm = sim.atmMPa * 1e6;
% Charge whose free gas at Patm and GasTemp is the spring's volume:
% n R T = ratio * Vl * Patm * T / 273.15 = Patm * V0.
h.sl.G.G_cfgPressureSchedOxygenRatio = (sim.chamberGasVolume / 1e3) * 273.15 ...
    / h.sl.G.G_cfgPressureSchedGasTemp / h.sl.G.G_cfgPressureSchedSalineVolume;
h.sl.ax.x = sim.chamberContactPos - 5;
h = enterMode(h, MODE_PRESSURE, 10);
assert(h.sl.s.bPressureSchedActive && h.sl.s.fbPressureSched.IntegHold);
h.pMax = 0;
h = runUntil(h, 30, @(h) h.y.performanceStatus == 1);
assert(h.y.performanceStatus == 1, 'scheduled loop did not reach the band');
t1 = h.t;
h = runFor(h, 3);
sc = h.sl.s.fbPressureSched;
kTrue = (h.sl.ax.P + sim.atmMPa)^2 * sim.pistonArea / (sim.atmMPa * sim.chamberGasVolume);
assert(abs(sc.KEst / kTrue - 1) < 0.02, 'k_est %.3f vs %.3f MPa/mm', sc.KEst, kTrue);
wc = h.sl.G.G_cfgPressureSchedCrossover;
assert(abs(sc.Kp - min(max(wc / sc.KEst, h.sl.G.G_cfgPressureSchedKpMin), ...
    h.sl.G.G_cfgPressureSchedKpMax)) < 1e-12);
assert(h.sl.s.fbPressureCtrl.Kp == sc.Kp && h.sl.s.fbPressureCtrl.Stiffness == sc.Stiffness);
assert(~sc.IntegHold && ~sc.ParamError);
assert(abs(h.sl.ax.P - 10) <= G.G_cfgPressureInBand && h.pMax <= 10.5, ...
    'overshoot %.3f MPa', h.pMax - 10);
% The switch is latched on entry: turning it off mid-run changes nothing.
kp = sc.Kp;
h.sl.G.G_cfgPressureScheduleEnable = false;
h = runFor(h, 0.5);
assert(h.sl.s.bPressureSchedActive && abs(h.sl.s.fbPressureCtrl.Kp - kp) < 0.05 * kp);
h.u.motionEnable = 0;
h = runFor(h, 1);
h.u.modeCode = 0;
h.u.reference = 0;
h = runUntil(h, 5, @(h) h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE);
assert(h.sl.G.G_sysCurrentState == E.E_SystemState.ST_IDLE && h.y.faultActive == 0);
fprintf('ok  scheduled gains: 10 MPa in %.1f s, k_est %.2f MPa/mm (true %.2f), Kp %.3f, max %.3f MPa\n', ...
    t1, sc.KEst, kTrue, sc.Kp, h.pMax);
```

Notes:
- `h.sl.s` is PRG_Main's instance state (`slaveStep` calls
  `PRG_Main(sl.s, sl.G, …)`), so `h.sl.s.fbPressureSched` and
  `h.sl.s.bPressureSchedActive` exist after regeneration.
- `G` (top of the test) is a copy taken at start; use it only for unchanged
  settings like `G_cfgPressureInBand`.
- The bound is exact for this chamber except for the 1 mL liquid term (0.5 %
  at 10 MPa) and the filtered reading's lag, hence 2 %.
- Do not assert on `h.faultSeen`: it accumulates over the whole test and the
  earlier scenarios fault on purpose; `h.y.faultActive == 0` is the check.
- If `pMax <= 10.5` fails, do not loosen it silently: report the measured
  peak to the parent session with the run's Kp/VelLimit trace.

Commit: "tests: scheduled pressure mode on the simulated axis".

---

## 8. Commands

Setup (cloud container): `apt-get update && apt-get install -y octave` and
`pip install -r design/requirements.txt`.

```sh
python3 tools/st2m.py slave-src slave-sim/generated
python3 tools/st2py.py slave-src design/generated
python3 tests/st2pyParity.py
python3 design/pressure-loop/test_model.py          # includes check_scheduled_st
octave-cli --quiet --eval "addpath('master-src/logic','slave-sim','slave-sim/generated','tests'); slavePressureMode"
tests/run_all.sh                                     # everything; must end clean
```

Baseline on this branch before any change: see the line below, filled in by
the plan author.

(Baseline pending at first commit; see the next commit.)

Report the outputs (the `ok` lines of `check_scheduled_st`, parity's line for
`FB_PressureSchedule`, the new `slavePressureMode` line) to the parent.

---

## 9. Documentation (same change as the behaviour)

1. `docs/slave/development/ConfigurationGuide.md`: add a section after
   section 6, `## 7. Pressure Loop Gain Schedule`, with a short paragraph
   (what the schedule does, that it is latched on entry, that ParamError
   falls back to the GVL gains, that `G_cfgPressureKp/Ki/StopStiffness`
   remain the fallback and are unchanged) and a table with one row per
   `G_cfgPressureSched*`/`G_cfgPressureScheduleEnable` setting: name,
   default, unit, description. Include in the rows:
   - `DeadVolume`: the chamber volume with the piston at
     `G_cfgPosSoftLimitMax`; 74 mL is a lower bound on the 79 mL at the end
     of travel (passages 50 + hose 29); with the soft limit short of the end
     of travel (305 vs 365 mm in the reference file) the bound is stiffer
     than the chamber, which only lowers the gain; set
     74 + 3.431·(365 − SoftLimitMax) mL to remove that. Must never exceed the
     true volume.
   - `SalineVolume`, `BulkModulus`, `HoseCompliance`: which direction is safe
     (saline and hose compliance at or below the truth, bulk modulus at or
     above); hose compliance 0 when the proximal valve is opened only after
     mixing (the hose is not part of the chamber during the compression).
   - `OxygenRatio`: must equal the recipe of the fill (1.0 for 1:1); a lower
     configured charge makes the bound stiffer (safe, slower), a higher one
     makes it softer (unsafe).
   - `DriveDeadTime`/`DriveLag`: measured on the bench
     (`bench/characterisation`), with Y_DirectControl's FilterTimeConstant as
     wired; re-measure if that pin changes.
   - Crossover/KiRatio/Kp bounds/ArriveBand: as in the design README results.
   Touch no other section of the guide.
2. `docs/slave/development/SlaveDeveloperGuide.md`: in §2's FB description or
   a new subsection at the end of §3 ("Changing a block's gains between
   scans"), state that a POU never writes another block's internal `VAR`
   (IEC forbids it), so `FB_PressureController` takes `IntegScale` and
   `FB_PressureSchedule` computes it; and that every input of a block is
   passed on its one call in PRG_Main. One paragraph.
3. `design/pressure-loop/README.md`:
   - Files table, `controllers.py` row: add `ScheduledSlaveControllerST` (the
     ported ST blocks, wired as PRG_Main wires them).
   - New section `## Porting the schedule` at the end of the file (after
     the open questions list), covering: the block and its settings
     (`G_cfgPressureSched*`, off by default); the mapping from the
     reference's arguments to the settings (wc, K_cfg, V_l_cfg, V_dead_cfg,
     C_hose_cfg, ratio_cfg, ki_ratio, kp_min/max, e_int, lag from the filter,
     median, scan and drive settings); x_limit ↔ `G_cfgPosSoftLimitMax` with
     the dead volume defined at the soft limit; the integrator rescale via
     `IntegScale` and the hold via the controller's Reset on held scans and
     the arrival scan; the check (`check_scheduled_st`, 1e-9 on the command
     through compress and deliver); and what remains for commissioning (the
     dead volume against the machine's soft limit, the hose compliance
     against the procedure, the charge per recipe).
   - Open-questions bullet "Whether the saline volume and the charge can be
     given to the slave (as configuration or over the protocol)": reword to
     "The saline volume and the charge are GVL settings
     (`G_cfgPressureSchedSalineVolume`, `G_cfgPressureSchedOxygenRatio`);
     whether they should come over the protocol per run."
4. `AGENTS.md`, Design studies bullet: after "`controllers.py` holds the
   slave's controller and `ScheduledSlaveController`, …" add
   "and `ScheduledSlaveControllerST`, which runs the ported
   `FB_PressureSchedule`". Tests list: no change (test_model is already
   listed).

Commit: "docs: pressure-loop gain schedule".

---

## 10. LD: what the user must wire or check by hand

The port adds no `G_cmd*`/`G_sta*` field, so no LD pin is new. Report these
to the user (they matter for the stop law's lag):

1. `Y_DirectControl_1.FilterTimeConstant` is hardwired to `LREAL#0.05`
   (`docs/slave/ld/README.md`). A 50 ms filter on the velocity command
   would be a lag the stop law's `Lag` does not include (it uses the measured
   12 ms + 8.7 ms). The bench saw no 50 ms lag, so the measured values
   already reflect the wiring as it is; wire `G_cmdDirectControl.FilterTimeConstant`
   as the LD record recommends and re-measure the drive lag if the value
   changes, then update `G_cfgPressureSchedDriveLag`.
2. `Y_DirectControl_1.Error` is wired to `G_staDirectControl.Active`
   (should be `.Error`); unrelated to the port but it masks a block error
   during pressure mode.
3. `Y_DirectControl` has no deceleration input, and the drive does not slew
   the velocity command in VelocityTLMode; the stop law's `DecelMax`
   (`G_cfgPressureDecel`) is therefore a design value, conservative while the
   drive stops faster than it.
4. In MotionWorks, declare `FB_PressureSchedule`'s variables and
   `FB_PressureController.IntegScale` (initial value 1.0) in the GUI from the
   header comments, and the new GVL entries in the global variable list.

---

## 11. Risks and the check for each

| Risk | Check |
|---|---|
| Typed literal `LREAL#1.0E-6` mistranslated | §1 fix; `grep "E - "` in generated `FB_PressureSchedule.*` finds nothing; regenerated sim identical before/after the fix |
| Writing `fbPressureCtrl.rInteg` from PRG_Main (not IEC) | Not done: `IntegScale` input; `grep -n "rInteg" slave-src/PRG` is empty |
| Hold timing off by one scan (Reset clears at scan start; the reference zeroes after the call) | `IntegHold` taken before the arrival update; `check_scheduled_st` 1e-9 lockstep |
| Rescale wrong (direction, first scan) | `rKiPrev` = 0 after Reset → `IntegScale` 1; lockstep; existing `check_scheduled` continuity check still passes |
| Reference reads the plant (`par.Vl0`, `par.t_dead`, `par.tauV`, `par.A`, `kH`, `T`, `Patm`) where the port reads the GVL | Asserts at the top of `check_scheduled_st` |
| Omitted FB input keeps its old value | Both PRG_Main calls list every input; the fallback branch sets all five `rPressure*` |
| Switch toggled mid-run changes gains without rescale | Latched on entry (`bPressureSchedActive`); Octave scenario toggles it mid-run |
| Division by zero / SQRT of a negative under random parity inputs | `ParamError` guards every domain; `st_div`/`st_sqrt` in Python; parity passes |
| `IntegScale` drawn at random grows `rInteg` to inf/NaN in parity | Both translations follow IEEE; parity's `close()` accepts equal inf/NaN. If it fails, report the call and variable rather than changing the test |
| Position frame: soft limit ≠ end of travel | Bound remains an upper bound (estimate's volume ≤ true); documented with the correction formula |
| Hose compliance configured but hose shut during compression (present procedure) | Documented: set 0 then; not a test item |
| Discrete flips (saturation, arrival) from 1e-16 rounding between reference and port | Arrival uses bit-identical `err`; watch `L.worst` — a flip shows as a jump well above 1e-9 at one scan; debug per §6b |
| Fallback path changes the default slave behaviour | Switch FALSE by default; the old `slavePressureMode`, `closedLoopMasterSlave`, `closedLoopModbusStep` and `check_wiring` pass unchanged |
| Octave sim overshoot above 10.5 MPa with the schedule | Asserted; report rather than loosen |
| MotionWorks: header defaults (`IntegScale := 1.0`, `bArrived := FALSE`) must be entered in the GUI | Listed in §10 for the user |
