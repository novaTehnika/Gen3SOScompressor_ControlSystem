"""Operating cases. All pressures in Pa, flows in m^3/s, times in s.

    compress   gas-rich chamber at atmosphere, target 10 MPa; stiffness rises
               by four orders of magnitude as the gas dissolves
    stiffstep  liquid-only chamber at 10 MPa, target steps to 12 MPa at 1 s
    discharge  liquid-only at 10 MPa, 1 mL/s outflow from 1 s to 21 s
    valve      liquid-only at 10 MPa, outflow ramps to Qf over Topen starting
               at 1 s (a step when Topen is 0)

target(t) and Qout(t) are the reference and the outflow; Qff(t) is the flow
the controller is told about when it uses flow feedforward (Qout scaled by
ff_scale, so a feedforward error can be studied).
"""

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class Scenario:
    name: str
    T: float                    # s simulated time
    tEvent: float               # s metrics are measured from here
    P0: float = 0.0             # Pa gauge at the start
    gas: bool = False           # start with gas at atmosphere in the chamber (else liquid only)
    target0: float = 10e6       # Pa target before t_step
    target1: float = 10e6       # Pa target from t_step on
    t_step: float = 1.0         # s
    Qf: float = 0.0             # m^3/s outflow once open
    t_open: float = 1.0         # s outflow starts
    Topen: float = 0.0          # s opening ramp, 0 for a step
    t_close: float = float('inf')  # s outflow ends
    ff_scale: float = 1.0       # Qff = ff_scale * Qout

    def with_(self, **kw):
        return replace(self, **kw)

    def target(self, t):
        return self.target1 if t >= self.t_step else self.target0

    def Qout(self, t):
        if t < self.t_open or t >= self.t_close:
            return 0.0
        if self.Topen > 0:
            return self.Qf * min((t - self.t_open) / self.Topen, 1.0)
        return self.Qf

    def Qff(self, t):
        return self.ff_scale * self.Qout(t)

    def initial(self, p):
        """(P0, moles of free gas, moles dissolved) for plant parameters p."""
        ng0 = p.Patm * p.Vg0 / (p.R * p.T) if self.gas else 0.0
        return self.P0, ng0, p.kH * (self.P0 + p.Patm) * p.Vl0


def compress(p, **kw):
    return Scenario('compress', 60.0, 0.0, P0=0.0, gas=True, target0=10e6, target1=10e6).with_(**kw)


def stiffstep(p, **kw):
    return Scenario('stiffstep', 8.0, 1.0, P0=10e6, target0=10e6, target1=12e6, t_step=1.0).with_(**kw)


def discharge(p, **kw):
    return Scenario('discharge', 30.0, 1.0, P0=10e6, Qf=p.Qf, t_open=1.0, t_close=21.0).with_(**kw)


def valve(p, **kw):
    return Scenario('valve', p.valveT, 1.0, P0=10e6, Qf=p.Qf, t_open=1.0, Topen=p.Topen).with_(**kw)


FACTORIES = {'compress': compress, 'stiffstep': stiffstep, 'discharge': discharge, 'valve': valve}


def make(name, p, **kw):
    """Scenario `name` for plant parameters p, with fields overridden by kw."""
    if name not in FACTORIES:
        raise KeyError(f'unknown scenario {name}')
    return FACTORIES[name](p, **kw)
