"""Chamber and actuators, stepped one slave scan at a time.

Chamber. Gas (n_g moles, isothermal) and liquid (volume Vl, bulk modulus K)
share the volume the piston sweeps. With gauge pressure P, Pabs = P + Patm and
dissolution rate ndot = (kH Pabs Vl - n_d)/tauDiss,

    dP/dt = (A v - Q - (R T / Pabs) ndot) / C,   C = n_g R T / Pabs^2 + Vl / K

where Q is the liquid outflow. The piston position x integrates v and the
chamber volume is Vch = Vch0 - A x. The contents fill the chamber, so Vch
equals the stressed liquid Vl (1 - P/K) plus the free gas; dissolved gas takes
no volume. With `liquid_depletes` the outflow Q removes the liquid it carries
(Vl falls by Q dt/(1 - P/K)) and the dissolved gas leaves in proportion, which
keeps that identity exact; without it Vl stays at Vl0 and the dissolved gas
stays, and the identity is off by the volume discharged.

Velocity actuator (Y_DirectControl in VelocityTLMode). The command is slewed
at `accel` while the slewed value grows in magnitude and at `decel` while it
shrinks or reverses (one limit without `slew_asym`), lagged by tauV, and
stalls at zero when the chamber force plus friction exceeds Fcap.

Torque actuator. m dv/dt = F - P A - b v - Fc sat(v/veps), v clamped to
+/- vmax.

`Plant.scan` advances nsub substeps of Ts/nsub at a constant command and
outflow, with the state in local variables for speed.
"""

from types import SimpleNamespace


class Plant:
    def __init__(self, p, scenario, mode, accel=None, decel=None):
        self.p, self.mode = p, mode
        self.accel = p.amax if accel is None else accel
        self.decel = (p.dmax if decel is None else decel) if p.slew_asym else self.accel
        self.dts = p.Ts / p.nsub
        P0, ng0, nd0 = scenario.initial(p)
        self.s = s = SimpleNamespace(
            x=0.0, v=0.0, vref=0.0, P=P0, ndis=nd0, ntot=ng0 + nd0, Vl=p.Vl0,
            nout=0.0, C=0.0)
        Pabs = P0 + p.Patm
        s.C = ng0 * p.R * p.T / Pabs ** 2 + p.Vl0 / p.K
        # contents volume at the start: stressed liquid plus free gas
        self.Vch0 = p.Vl0 * (1 - P0 / p.K) + ng0 * p.R * p.T / Pabs

    @property
    def stiffness(self):
        """dP/dx in Pa/m"""
        return self.p.A / self.s.C

    @property
    def Vch(self):
        """chamber volume from the piston position, m^3"""
        return self.Vch0 - self.p.A * self.s.x

    def volume_closure(self):
        """Vch minus (stressed liquid + free gas), m^3."""
        p, s = self.p, self.s
        ng = s.ntot - s.ndis
        return self.Vch - (s.Vl * (1 - s.P / p.K) + ng * p.R * p.T / (s.P + p.Patm))

    def scan(self, u, Q):
        """Advance one scan with command u (m/s for 'V', N for 'T') and outflow Q (m^3/s)."""
        p, s, dts = self.p, self.s, self.dts
        A, RT, Patm, K, kH, tauDiss = p.A, p.R * p.T, p.Patm, p.K, p.kH, p.tauDiss
        depletes = p.liquid_depletes
        x, v, vref, P = s.x, s.v, s.vref, s.P
        ndis, ntot, Vl = s.ndis, s.ntot, s.Vl
        nout = s.nout
        C = s.C
        velocity = self.mode == 'V'
        if velocity:
            acc, dec = self.accel * dts, self.decel * dts
            Fcap_P, kv = (p.Fcap - p.Fc) / A, dts / p.tauV
        else:
            vmax, mr, b, Fc, veps = p.vmax, p.mr, p.b, p.Fc, p.veps
        Qdt = Q * dts
        for _ in range(p.nsub):
            Pabs = P + Patm
            if velocity:
                dv = u - vref
                lim = acc if dv * vref >= 0 else dec
                vref += lim if dv > lim else (-lim if dv < -lim else dv)
                v += (vref - v) * kv
                if P > Fcap_P and v > 0:          # stalled at the torque limit
                    v = 0.0
                    vref = 0.0
            else:
                sat = v / veps
                sat = 1.0 if sat > 1.0 else (-1.0 if sat < -1.0 else sat)
                v += (u - P * A - b * v - Fc * sat) / mr * dts
                v = vmax if v > vmax else (-vmax if v < -vmax else v)
            x += v * dts
            neq = kH * Pabs * Vl
            if neq > ntot:
                neq = ntot
            ndot = (neq - ndis) / tauDiss
            ndis += ndot * dts
            if ndis > ntot:
                ndis = ntot
            C = (ntot - ndis) * RT / (Pabs * Pabs) + Vl / K
            P += (A * v * dts - Qdt - RT / Pabs * ndot * dts) / C
            if P < 1 - Patm:
                P = 1 - Patm
            if depletes and Qdt:
                dVl = Qdt / (1 - P / K)
                carry = ndis * dVl / Vl
                ndis -= carry
                ntot -= carry
                nout += carry
                Vl -= dVl
        s.x, s.v, s.vref, s.P = x, v, vref, P
        s.ndis, s.ntot, s.Vl = ndis, ntot, Vl
        s.nout = nout
        s.C = C
        return P
