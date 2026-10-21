"""
scenarios.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module defines the operating cases that the studies simulate. A
Scenario gives the initial chamber contents, the pressure target over
time and the liquid outflow over time. Pressures are in Pa, flows in
m^3/s and times in s.

The four standard cases are:

  compress   The chamber starts gas-rich at atmospheric pressure and the
             target is 10 MPa. The chamber stiffness rises by about four
             orders of magnitude as the gas is compressed and dissolves.
  stiffstep  The chamber holds only liquid at 10 MPa, and the target
             steps to 12 MPa at 1 s.
  discharge  The chamber holds only liquid at 10 MPa, and an outflow of
             Params.Qf (1 mL/s by default) runs from 1 s to 21 s.
  valve      The chamber holds only liquid at 10 MPa, and from 1 s the
             outflow rises linearly to Params.Qf over the valve opening
             time Params.Topen (a step when Topen is 0).

Qff(t) is the outflow the controller is told about when it uses flow
feedforward. It is the actual outflow scaled by ff_scale, so that an
error in the expected flow can be studied.

FILE DEPENDENCY:
None.
"""

from dataclasses import dataclass, replace


# %% ===== SCENARIO DEFINITION ===========================================
@dataclass(frozen=True)
class Scenario:
    """One operating case. Use with_() to change fields."""

    name: str
    # [s] Simulated duration
    T: float
    # [s] Time of the event (target step or valve opening); the metrics
    # are measured from here
    tEvent: float
    # [Pa] Gauge pressure at the start
    P0: float = 0.0
    # True if the chamber starts with gas at atmospheric pressure; False
    # if it holds only liquid
    gas: bool = False
    # [Pa] Target before t_step
    target0: float = 10e6
    # [Pa] Target from t_step on
    target1: float = 10e6
    # [s] Time of the target step
    t_step: float = 1.0
    # [m^3/s] Outflow once the valve is fully open
    Qf: float = 0.0
    # [s] Time the outflow starts
    t_open: float = 1.0
    # [s] Opening ramp duration; 0 opens the valve in one step
    Topen: float = 0.0
    # [s] Time the outflow stops
    t_close: float = float('inf')
    # [-] Ratio of the feedforward flow Qff to the actual outflow
    ff_scale: float = 1.0

    def with_(self, **kw):
        """Return a copy with the given fields replaced."""
        return replace(self, **kw)

    def target(self, t):
        """[Pa] Pressure target at time t [s]"""
        return self.target1 if t >= self.t_step else self.target0

    def Qout(self, t):
        """[m^3/s] Liquid outflow at time t [s]. The outflow rises
        linearly from zero to Qf over Topen, starting at t_open, and
        stops at t_close."""
        if t < self.t_open or t >= self.t_close:
            return 0.0
        if self.Topen > 0:
            return self.Qf * min((t - self.t_open) / self.Topen, 1.0)
        return self.Qf

    def Qff(self, t):
        """[m^3/s] Outflow the controller expects at time t [s]"""
        return self.ff_scale * self.Qout(t)

    def initial(self, par):
        """Return the initial chamber contents for Params par.

        Returns (P0 [Pa], free gas n_g0 [mol], dissolved gas n_d0 [mol]).
        The free gas is Vg0 at atmospheric pressure by the ideal gas law,
        n_g0 = Patm Vg0 / (R T), or zero for a liquid-only start. The
        liquid starts saturated at P0 by Henry's law,
        n_d0 = kH (P0 + Patm) Vl0.
        """
        if self.gas:
            n_g0 = par.Patm * par.Vg0 / (par.R * par.T)
        else:
            n_g0 = 0.0
        n_d0 = par.kH * (self.P0 + par.Patm) * par.Vl0
        return self.P0, n_g0, n_d0


# %% ===== STANDARD SCENARIOS ============================================
# Each function returns the scenario for Params par, with any field
# replaced by a keyword argument.

def compress(par, **kw):
    return Scenario('compress', T=60.0, tEvent=0.0, P0=0.0, gas=True,
                    target0=10e6, target1=10e6).with_(**kw)


def stiffstep(par, **kw):
    return Scenario('stiffstep', T=8.0, tEvent=1.0, P0=10e6,
                    target0=10e6, target1=12e6, t_step=1.0).with_(**kw)


def discharge(par, **kw):
    return Scenario('discharge', T=30.0, tEvent=1.0, P0=10e6, Qf=par.Qf,
                    t_open=1.0, t_close=21.0).with_(**kw)


def valve(par, **kw):
    return Scenario('valve', T=par.valveT, tEvent=1.0, P0=10e6,
                    Qf=par.Qf, t_open=1.0, Topen=par.Topen).with_(**kw)


FACTORIES = {'compress': compress, 'stiffstep': stiffstep,
             'discharge': discharge, 'valve': valve}


def make(name, par, **kw):
    """Return the standard scenario called name for Params par, with
    fields replaced by the keyword arguments."""
    if name not in FACTORIES:
        raise KeyError(f'unknown scenario {name}')
    return FACTORIES[name](par, **kw)
