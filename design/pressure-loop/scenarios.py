"""
scenarios.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module defines the operating cases that the studies simulate. A
Scenario gives the chamber contents at the start, the pressure target
over time and the outlet valve's state over time. Pressures are in Pa,
flows in m^3/s and times in s.

The process cases follow the machine's cycle. The oxygen charge, given
by Params.ratio, is admitted at the supply pressure Params.P_supply, the
piston compresses the contents to the target Params.P_target (10 MPa by
default), the oxygen dissolves during a hold, and the ball valve is
opened so the saline is driven through the orifices until the piston
reaches the limit switch.

  compress   Start at the supply pressure with the charge as free gas,
             the saline saturated at that pressure and the hose shut;
             drive to the target.
  deliver    Start at the target with the charge dissolved as far as
             the saturation limit allows, the rest free, and the hose
             shut. The proximal valve opens at 1 s (over
             Params.Topen_prox), the distal valve 5 s after it is fully
             open to purge the hose's virgin saline and any solution that
             degassed entering the hose, closes when that has left, and
             opens again after a 10 s pause; the
             run continues to the limit switch. With Params.hose_before
             the hose is open from the start instead, and the distal
             valve closes over Params.Tclose.
  cycle      compress, hold, then the same valve sequence, in one run.
  stiffstep  Liquid-only chamber at 10 MPa, target steps to 12 MPa at
             1 s. This is a margin probe, not a process case.

Outflow. With the valve open the outflow is the orifice flow at the
chamber pressure, Params.Q_orifice(P), scaled by the valve's area
fraction valve(t), which rises from 0 to 1 over Topen (a step when Topen
is 0). A scenario may instead prescribe a fixed flow Qf, which the
linear-model tests use. Qff(t) is the outflow a controller with flow
feedforward is told about; the machine cannot supply it, so it is kept
only as a bound.

FILE DEPENDENCY:
None. The functions take a Params object (params.py).
"""

from dataclasses import dataclass, replace


# %% ===== SCENARIO DEFINITION ===========================================
@dataclass(frozen=True)
class Scenario:
    """One operating case. Use with_() to change fields."""

    name: str
    # [s] Simulated duration; a delivery ends early when the chamber is
    # empty
    T: float
    # [s] Time of the event (target step or valve opening); the metrics
    # are measured from here
    tEvent: float
    # [Pa] Gauge pressure at the start
    P0: float = 0.0
    # [-] Oxygen charge, STP volume per saline volume; 0 for liquid only
    ratio: float = 0.0
    # True if the charge starts dissolved up to the saturation limit at
    # P0 (after a hold); False if it starts as free gas with the saline
    # saturated at P0 (just admitted)
    dissolved: bool = False
    # [Pa] Target before t_step
    target0: float = 10e6
    # [Pa] Target from t_step on
    target1: float = 10e6
    # [s] Time of the target step
    t_step: float = 1.0
    # True if the hose (proximal valve) is open from the start; False
    # if it is shut until t_prox
    hose_open0: bool = False
    # [s] Time the proximal valve starts to open; inf if never. Opening
    # it joins the hose's liquid and compliance to the chamber: in one
    # scan, eq. (43), or through the throttled flow of eq. (45) over
    # Topen_prox.
    t_prox: float = float('inf')
    # [s] Opening ramp of the proximal valve; 0 opens it in one scan
    Topen_prox: float = 0.0
    # [s] Time the distal valve starts to open; inf if it never opens
    t_open: float = float('inf')
    # [s] Opening ramp duration; 0 opens the valve in one scan
    Topen: float = 0.0
    # [s] Closing ramp duration at the end of the purge; 0 closes the
    # valve in one scan
    Tclose: float = 0.0
    # [s] Time the valve closes; inf for never (a purge closes it
    # itself)
    t_close: float = float('inf')
    # [m^3] Purge: the distal valve closes once this much has left the
    # outlet, and opens again after t_pause; 0 for no purge stage.
    # None purges the hose's virgin saline plus whatever solution
    # degassed on its way into the hose (the purge the procedure needs).
    V_purge: float = 0.0
    # [s] Pause between the end of the purge and the delivery
    t_pause: float = 10.0
    # True if the tube holds air again at the second opening (it
    # drained while moved to the delivery vessel); False if it keeps
    # its liquid
    tube_refills: bool = False
    # [m^3/s] Prescribed outflow with the valve open; 0 selects the
    # orifice flow at the chamber pressure
    Qf: float = 0.0
    # True to lose the tube's air volume from the chamber when the valve
    # opens
    tube: bool = True
    # [-] Ratio of the feedforward flow Qff to the actual outflow
    ff_scale: float = 1.0

    def with_(self, **kw):
        """Return a copy with the given fields replaced."""
        return replace(self, **kw)

    def target(self, t):
        """[Pa] Pressure target at time t [s]"""
        return self.target1 if t >= self.t_step else self.target0

    def valve(self, t, t_from=None):
        """[-] Distal valve area fraction at time t [s]: 0 closed, 1 open,
        rising linearly over Topen from t_from (default t_open). The
        purge's closing and reopening are sequenced by the simulation."""
        t_from = self.t_open if t_from is None else t_from
        if t < t_from or t >= self.t_close:
            return 0.0
        if self.Topen > 0:
            return min((t - t_from) / self.Topen, 1.0)
        return 1.0

    def hose_open(self, t):
        """True if the proximal valve is open at time t [s]."""
        return self.hose_open0 or t >= self.t_prox

    def prox(self, t):
        """[-] Proximal valve area fraction at time t [s]: 0 shut, rising
        linearly over Topen_prox from t_prox to 1."""
        if self.hose_open0:
            return 1.0
        if t < self.t_prox:
            return 0.0
        if self.Topen_prox > 0:
            return min((t - self.t_prox) / self.Topen_prox, 1.0)
        return 1.0

    def Qout(self, t, par, P):
        """[m^3/s] Outflow at time t [s] and chamber pressure P [Pa] with
        the distal valve as first opened: the valve fraction times the
        prescribed flow or the orifice flow. The feedforward bound uses
        it; the simulation sequences the valve itself."""
        a = self.valve(t)
        if a == 0.0 or not self.hose_open(t):
            return 0.0
        return a * (self.Qf if self.Qf > 0 else par.Q_orifice(P))

    def Qff(self, t, par, P):
        """[m^3/s] Outflow a feedforward controller expects at time t"""
        return self.ff_scale * self.Qout(t, par, P)

    def initial(self, par):
        """Return the chamber contents at the start for Params par.

        Returns (P0 [Pa], free gas [mol], dissolved gas [mol]). The charge
        is par.n_charge() when ratio is nonzero. The saline can hold
        n_sat = kH (P0 + Patm) Vl0 by Henry's law. If the charge starts
        dissolved, n_dis = min(charge, n_sat) and the rest is free;
        otherwise the charge is free and the saline carries n_sat, the
        equilibrium it reached during the fill.
        """
        n_sat = par.kH * (self.P0 + par.Patm) * par.Vl0
        if self.ratio <= 0:
            return self.P0, 0.0, n_sat
        n_charge = par.n_charge(self.ratio)
        if self.dissolved:
            n_dis = min(n_charge, n_sat)
            return self.P0, n_charge - n_dis, n_dis
        return self.P0, n_charge, n_sat


# %% ===== STANDARD SCENARIOS ============================================
# Each function returns the scenario for Params par, with any field
# replaced by a keyword argument.

def _hose(par):
    """Return the hose fields for Params par: open throughout if
    hose_before is set, else opened at the scenario's t_prox."""
    if par.hose_before:
        return dict(hose_open0=True, t_prox=float('inf'))
    return {}


def compress(par, **kw):
    P = par.P_target
    return Scenario('compress', T=60.0, tEvent=0.0, P0=par.P_supply,
                    ratio=par.ratio, target0=P, target1=P
                    ).with_(**_hose(par)).with_(**kw)


def deliver(par, **kw):
    P = par.P_target
    return Scenario('deliver', T=200.0, tEvent=1.0, P0=P,
                    ratio=par.ratio, dissolved=True, target0=P,
                    target1=P, t_prox=1.0, Topen_prox=par.Topen_prox,
                    t_open=6.0 + par.Topen_prox, Topen=par.Topen,
                    Tclose=par.Tclose, V_purge=None
                    ).with_(**_hose(par)).with_(**kw)


def cycle(par, **kw):
    P = par.P_target
    return Scenario('cycle', T=260.0, tEvent=60.0, P0=par.P_supply,
                    ratio=par.ratio, target0=P, target1=P,
                    t_prox=60.0, Topen_prox=par.Topen_prox,
                    t_open=65.0 + par.Topen_prox, Topen=par.Topen,
                    Tclose=par.Tclose, V_purge=None
                    ).with_(**_hose(par)).with_(**kw)


def stiffstep(par, **kw):
    return Scenario('stiffstep', T=8.0, tEvent=1.0, P0=10e6, ratio=0.0,
                    target0=10e6, target1=12e6, t_step=1.0).with_(**kw)


FACTORIES = {'compress': compress, 'deliver': deliver, 'cycle': cycle,
             'stiffstep': stiffstep}


def make(name, par, **kw):
    """Return the standard scenario called name for Params par, with
    fields replaced by the keyword arguments."""
    if name not in FACTORIES:
        raise KeyError(f'unknown scenario {name}')
    return FACTORIES[name](par, **kw)
