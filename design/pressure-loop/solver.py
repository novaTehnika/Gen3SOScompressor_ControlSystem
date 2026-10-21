"""
solver.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/21/2026

PURPOSE/DESCRIPTION:
This module holds the fixed-step numerical integration methods used to
advance the plant model. Each function takes one step of length dt from
state y using a derivative function f, called as f(y, *args), and returns
the new state. The state is a list of floats so that the per-step
arithmetic stays fast in plain Python.

The plant holds its input constant over each step, and any discrete
logic (the drive's command slew limit, the stall at the torque limit,
and the physical bounds on the state) runs between steps. The methods
here therefore integrate a smooth system over each step.

Two methods are provided:

  euler_step   First-order explicit (forward) Euler, README eq. (34).
               It evaluates the derivative once per step and is the same
               update as ode1.m.
  heun_step    Second-order explicit Heun (improved Euler), eq. (35). It
               averages the derivative at the start of the step with the
               derivative at the Euler prediction, so it evaluates the
               derivative twice per step.

Forward Euler is the default. The two methods agree closely at the
default step (test_model.py), which bounds the integration error.

FILE DEPENDENCY:
None.
"""


def euler_step(f, y, dt, *args):
    """Advance state y by one forward-Euler step of length dt [s],
    eq. (34): y(t + dt) = y(t) + dt f(y(t))."""
    dydt = f(y, *args)
    return [yi + dt * di for yi, di in zip(y, dydt)]


def heun_step(f, y, dt, *args):
    """Advance state y by one Heun step of length dt [s], eq. (35).

    The predictor is the Euler step, y_p = y + dt f(y). The corrector
    averages the two slopes, y(t + dt) = y + (dt/2) (f(y) + f(y_p)).
    """
    k1 = f(y, *args)
    y_pred = [yi + dt * di for yi, di in zip(y, k1)]
    k2 = f(y_pred, *args)
    half_dt = 0.5 * dt
    return [yi + half_dt * (d1 + d2) for yi, d1, d2 in zip(y, k1, k2)]


# Methods selectable by name through Params.solver
METHODS = {'euler': euler_step, 'heun': heun_step}
