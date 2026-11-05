"""
plant.py module

AUTHORS:
Jeremy Simmons (email: simmo536@umn.edu, jeremy@jeremysimmons.me)
GitHub: novaTehnika

CREATION DATE:
10/15/2026

PURPOSE/DESCRIPTION:
This module models the compressor chamber, its outlet and the servo
actuator that drives the piston. The Plant class holds the state,
computes the state derivatives, and advances the state by one slave scan
with the actuator command held constant.

The chamber holds saline and oxygen gas. The piston of area A sweeps the
chamber volume. The gas is isothermal and dissolves into the saline
according to Henry's law with a first-order time constant. The pressure
follows from continuity: the piston displacement, the outflow and the
gas that dissolves change the volume available to the contents, and the
chamber capacitance (the gas term plus the liquid term) converts that
volume change to a pressure change. When the option liquid_depletes is
set, the outflow removes saline and carries away the gas dissolved in
it. A pocket of air in the passage to the pressure transducer cannot be
purged and does not dissolve; it is a second isothermal gas of fixed
amount beside the oxygen, eq. (46). Equations (1) to (10) and (46) of
the README give the chamber model.

The outlet is a hose, a tube and the orifice bank. The hose holds virgin
saline at atmosphere behind the proximal valve; opening that valve joins
the hose to the chamber. Opened in one scan, the pressures equalise in
one step, eq. (43): the chamber's stored compression fills the hose's
liquid and wall compliance, eq. (42). Opened over a time, the valve
throttles the solution into the hose, eq. (45), and the hose pressure
rises with the volume admitted, eq. (44), until the two are equal and
the hose joins the chamber. The solution admitted is tracked as parcels
in the order they entered, each with the hose pressure it met: a parcel
that met a pressure below the saturation pressure of the solution
degassed, and the purge must carry it out with the virgin saline (plug
flow puts it right behind the virgin saline). With the distal valve open,
the outflow is the
orifice flow at the chamber pressure times the valve's area fraction,
eq. (36), or a prescribed flow when the scenario gives one; the virgin
saline leaves first, then the solution. The tube between the distal
valve and the orifices holds air before it first opens, so in that scan
the chamber also loses the tube's volume, eq. (37). The delivery ends
when the piston reaches the limit switch, where the cylinder head is
down to its passages.

The actuator has two forms. In velocity mode, which the slave uses, the
command reaches the drive after a dead time, the drive's velocity loop
follows it with a first-order lag, and the piston stalls when the
chamber force plus the friction reaches the drive's torque limit
(equations (11) to (13)). The drive was found not to slew the command at
the configured accel; the slew limit of eq. (11) is kept as an option.
In torque mode, the commanded force accelerates the reflected mass of
the motor and gearbox against the chamber force and friction (equation
(14)), with a load-dependent friction mu P A.

The method derivatives() is the continuous-time model. The method scan()
advances one slave scan in nsub fixed steps. Before each step it applies
the slew limit to the velocity command; after each step it applies the
stall and the physical bounds on the state. The integration method is
chosen by Params.solver from solver.py.

FILE DEPENDENCY:
./
    solver.py
"""

import math
from collections import deque

import solver


# %% ===== STATE VECTOR INDICES ==========================================
# The continuous state is a list y. These indices name its entries.
iy_x = 0       # [m] piston position, positive into the cylinder
iy_v = 1       # [m/s] piston velocity
iy_P = 2       # [Pa] chamber gauge pressure
iy_ndis = 3    # [mol] gas dissolved in the saline
iy_ntot = 4    # [mol] gas in the chamber, free and dissolved
iy_Vl = 5      # [m^3] saline volume at zero gauge pressure
iy_nout = 6    # [mol] dissolved gas carried out by the outflow
iy_Vout = 7    # [m^3] saline delivered through the outlet
iy_Vvir = 8    # [m^3] virgin saline (from the hose) still in the system
iy_Vin = 9     # [m^3] solution admitted into the hose while it fills
N_STATES = 10


# %% ===== PLANT MODEL ===================================================
class Plant:
    """Chamber, outlet and actuator for one simulation run.

    Arguments:
      par       Params with the plant parameters.
      scenario  Scenario that supplies the initial contents and the
                prescribed outflow, if any.
      mode      'V' for velocity command [m/s] or 'T' for force
                command [N].
      accel     [m/s^2] slew limit while the velocity command grows in
                magnitude. None selects par.amax.
      decel     [m/s^2] slew limit while the velocity command shrinks or
                reverses. None selects par.dmax. Without par.slew_asym
                the accel limit applies in both cases.
    """

    def __init__(self, par, scenario, mode, accel=None, decel=None):
        # Run options
        self.mode = mode
        self.velocity_mode = mode == 'V'
        self.liquid_depletes = par.liquid_depletes
        self.nsub = par.nsub
        self.step = solver.METHODS[par.solver]

        # Chamber parameters. They are copied from par once, because
        # some Params values are computed on access and the model reads
        # them at every step. See params.py for their descriptions.
        self.A = par.A                  # [m^2]
        self.RT = par.R * par.T         # [J/mol] R T
        self.Patm = par.Patm            # [Pa]
        # [J] Pocket of air at the transducer: n_air R T = Patm V_air,
        # eq. (46). The amount is fixed; it never dissolves or leaves.
        self.nRT_air = par.Patm * par.V_air
        self.K = par.K                  # [Pa]
        self.kH = par.kH                # [mol/(m^3 Pa)]
        self.tauDiss = par.tauDiss      # [s]
        self.Ts = par.Ts                # [s] scan time
        self.dt = par.Ts / par.nsub     # [s] integration step

        # Outlet parameters, eq. (36)
        self.Qf = scenario.Qf           # [m^3/s] prescribed flow, or 0
        # [m^3/s per sqrt(Pa)] orifice coefficient Cd A sqrt(2 / rho)
        self.k_orifice = par.Cd * par.A_orifice * math.sqrt(2 / par.rho)
        self.V_passages = par.V_passages    # [m^3]
        self.V_hose = par.V_hose            # [m^3]
        self.C_hose = par.C_hose            # [m^3/Pa]
        self.hose_open = scenario.hose_open0
        # Proximal valve flow constant, eq. (45): Q = a k sqrt(dP)
        self.k_prox = par.Cd_prox * par.A_prox * math.sqrt(2 / par.rho)
        # Parcels of solution admitted into the hose, in order of entry:
        # [volume [m^3], hose pressure met [Pa], degassed]
        self.parcels = []
        self.V_in_total = 0.0               # [m^3] solution admitted
        self.V_degas = 0.0                  # [m^3] of it degassed

        # Velocity actuator parameters
        self.slew_limited = par.slew_limited
        self.accel = par.amax if accel is None else accel   # [m/s^2]
        if par.slew_asym:
            self.decel = par.dmax if decel is None else decel
        else:
            self.decel = self.accel                         # [m/s^2]
        self.tauV = par.tauV            # [s]
        # Command dead time as a queue of whole scans
        n_dead = int(round(par.t_dead / par.Ts))
        self.cmd_queue = deque([0.0] * n_dead)
        # Torque limit, eq. (13): the drive force covers the pressure,
        # the load-dependent friction and the Coulomb and viscous terms
        self.Fcap = par.Fcap            # [N]
        self.mu = par.mu                # [-]

        # Friction and torque actuator parameters
        self.mr = par.mr                # [kg]
        self.b = par.b                  # [N s/m]
        self.Fc = par.Fc                # [N]
        self.veps = par.veps            # [m/s]
        self.vmax = par.vmax            # [m/s]

        # Initial state. The piston starts at rest at x = 0.
        P0, ng0, nd0 = scenario.initial(par)    # [Pa], [mol], [mol]
        y = [0.0] * N_STATES
        y[iy_P] = P0
        y[iy_ndis] = nd0
        y[iy_ntot] = ng0 + nd0
        y[iy_Vl] = par.Vl0
        self.y = y
        self.v_ref = 0.0                        # [m/s] slewed command
        self.Q = 0.0                            # [m^3/s] last outflow

        # [m^3] Cylinder volume at x = 0 (passages and swept volume, not
        # the hose). The contents fill it, so it equals the stressed
        # liquid plus the free gas and the air pocket (eq. (9)).
        self.Vch0 = (par.Vl0 * (1 - P0 / par.K)
                     + ng0 * self.RT / (P0 + par.Patm)
                     + self.nRT_air / (P0 + par.Patm))
        if self.hose_open:
            # The hose is already part of the system: its virgin saline
            # is at the chamber pressure and its wall is dilated, so the
            # cylinder holds the contents less what the hose holds.
            # chamber_volume subtracts V_hose, so Vch0 carries it too.
            y[iy_Vvir] = par.V_hose
            self.Vch0 += par.V_hose * (1 - P0 / par.K) - par.C_hose * P0

    # ----- Outflow ------------------------------------------------------
    def outflow(self, P, V_l, valve, Q_tube):
        """[m^3/s] Outflow at pressure P [Pa] with saline volume V_l
        [m^3], valve area fraction valve and tube flow Q_tube [m^3/s],
        eq. (36). It is zero once the saline is gone."""
        if V_l <= 0.0:
            return 0.0
        if valve > 0.0:
            if self.Qf > 0.0:
                Q = valve * self.Qf
            else:
                Q = valve * self.k_orifice * math.sqrt(max(P, 0.0))
        else:
            Q = 0.0
        return Q + Q_tube

    @property
    def empty(self):
        """True once the cylinder volume is down to its passages, where
        the piston reaches the limit switch and the machine halts."""
        return self.chamber_volume <= self.V_passages

    # ----- Hose ---------------------------------------------------------
    def liquid_capacitance(self, V_l, V_vir):
        """[m^3/Pa] Liquid part of the capacitance: the saline and, with
        the hose open, the hose's compliance, eq. (42)."""
        C = (V_l + V_vir) / self.K
        if self.hose_open:
            C += self.C_hose
        return C

    def open_hose(self):
        """Open the proximal valve in one scan: join the hose's virgin
        saline, at atmosphere, and its compliance to the chamber, and
        equalise the pressure in one step, eq. (43).

        The contents must fill the cylinder and the hose, so the new
        pressure P1 solves
          (V_l + V_vir + V_hose)(1 - P1/K) + n_g R T / (P1 + Patm)
            + Patm V_air / (P1 + Patm) = V_cyl + V_hose + C_hose P1
        where V_cyl is the cylinder volume now; it is found by bisection.
        The solution that moved into the hose to stress its saline and
        fill its dilation is logged as one parcel that met the hose at
        atmosphere, then at P1.
        """
        if self.hose_open:
            return
        y = self.y
        y[iy_Vvir] += self.V_hose
        self.hose_open = True
        self.Vch0 += self.V_hose
        P1 = self.equalise()
        V_in = ((self.V_hose + self.C_hose * P1
                 - self.V_hose * (1 - P1 / self.K)) / (1 - P1 / self.K))
        self.log_parcel(V_in, P1, P1 < self.P_sat(y))

    def equalise(self):
        """Set the pressure at which the contents fill the system (the
        cylinder plus, with the hose open, the hose and its dilation),
        the volume balance of eq. (9), by bisection; return it [Pa]."""
        y = self.y
        lo, hi = 1.0 - self.Patm, max(y[iy_P], 1.0)
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            y[iy_P] = mid
            if self.volume_closure() < 0:    # contents exceed the system
                lo = mid
            else:
                hi = mid
        y[iy_P] = 0.5 * (lo + hi)
        return y[iy_P]

    # ----- Throttled opening of the proximal valve ----------------------
    def P_sat(self, y):
        """[Pa] saturation pressure (gauge) of the solution in state y,
        eq. (38)"""
        return y[iy_ndis] / (self.kH * y[iy_Vl]) - self.Patm

    def hose_pressure(self, V_in):
        """[Pa] Hose gauge pressure with V_in of solution admitted behind
        the proximal valve, eq. (44): the hose's saline and the admitted
        solution, stressed, fill the bore and its dilation."""
        return V_in / (self.C_hose + (self.V_hose + V_in) / self.K)

    def prox_flow(self, y, prox):
        """[m^3/s] Flow through the proximal valve at area fraction prox
        into the shut hose, eq. (45), at the chamber pressure. It is
        capped at the flow that would equalise the two pressures within
        one scan, so that a wide-open valve cannot overshoot."""
        if prox <= 0.0 or self.hose_open:
            return 0.0
        P, V_in = y[iy_P], y[iy_Vin]
        dP = P - self.hose_pressure(V_in)                   # [Pa]
        if dP <= 0.0:
            return 0.0
        Q = prox * self.k_prox * math.sqrt(dP)              # [m^3/s]
        # Equalisation volume: dP over the two capacitances in series
        P_abs = P + self.Patm
        C_ch = ((y[iy_ntot] - y[iy_ndis]) * self.RT / P_abs**2
                + self.nRT_air / P_abs**2
                + (y[iy_Vl] - V_in) / self.K)               # [m^3/Pa]
        C_h = self.C_hose + (self.V_hose + V_in) / self.K   # [m^3/Pa]
        dV_eq = dP / (1 / C_ch + 1 / C_h)                   # [m^3]
        return min(Q, dV_eq / self.Ts)

    def log_parcel(self, dV, P_h, degassed):
        """Record dV [m^3] of solution admitted into the hose at hose
        pressure P_h [Pa]; consecutive parcels of one kind merge."""
        if dV <= 0.0:
            return
        self.V_in_total += dV
        if degassed:
            self.V_degas += dV
        if self.parcels and self.parcels[-1][2] == degassed:
            last = self.parcels[-1]
            last[1] = max(last[1], P_h) if not degassed else last[1]
            last[0] += dV
        else:
            self.parcels.append([dV, P_h, degassed])

    def join_hose(self):
        """Join the filled hose to the chamber once its pressure has met
        the chamber's: the admitted solution and the virgin saline
        become part of the system (the volume balance of eq. (9) then
        covers the cylinder, the hose and its dilation)."""
        y = self.y
        y[iy_Vvir] += self.V_hose
        y[iy_Vin] = 0.0
        self.hose_open = True
        self.Vch0 += self.V_hose
        # The last of the difference between the two pressures is
        # closed exactly, so that the volume balance holds
        self.equalise()

    def hose_lumps(self):
        """Return the liquid in the hose and tube from the distal valve
        back to the chamber as a list of (kind, volume [m^3], degassed):
        the virgin saline, then the admitted parcels, earliest first,
        which the purge expels in this order."""
        lumps = [('virgin saline', self.V_hose, False)]
        for V, P_h, degassed in self.parcels:
            lumps.append(('solution', V, degassed))
        return lumps

    # ----- Continuous-time model ----------------------------------------
    def derivatives(self, y, u_act, valve, Q_tube, prox=0.0):
        """Return dy/dt for state y.

        u_act is the actuator input held over the step: the slewed
        velocity reference [m/s] in velocity mode or the force [N] in
        torque mode. valve is the outlet valve's area fraction, Q_tube
        [m^3/s] the extra flow that fills the tube in the opening scan
        and prox the proximal valve's area fraction while the hose is
        shut.
        """
        x, v, P, n_dis, n_tot, V_l, n_out, V_out, V_vir, V_in = y

        # Absolute pressure, eq. (1)
        P_abs = P + self.Patm                               # [Pa]

        # Outflow through the outlet, eq. (36); nothing can leave while
        # the hose is shut, but the hose takes the throttled flow of
        # eq. (45) while it fills
        if self.hose_open:
            Q = self.outflow(P, V_l + V_vir, valve, Q_tube)  # [m^3/s]
            Q_prox = 0.0
        else:
            Q = 0.0
            Q_prox = self.prox_flow(y, prox)                # [m^3/s]

        # Gas dissolution toward the Henry's-law equilibrium,
        # eqs. (2) and (3). The equilibrium cannot exceed the gas present.
        n_eq = min(self.kH * P_abs * V_l, n_tot)            # [mol]
        r_dis = (n_eq - n_dis) / self.tauDiss               # [mol/s]

        # Chamber capacitance [m^3/Pa], the volume change per unit
        # pressure change of the free gas, the air pocket and the
        # liquid and, with the hose open, its compliance, eqs. (4), (42)
        # and (46)
        C = ((n_tot - n_dis) * self.RT / P_abs**2
             + self.nRT_air / P_abs**2
             + self.liquid_capacitance(V_l - V_in, V_vir))

        # Piston acceleration
        if self.velocity_mode:
            # Drive velocity loop as a first-order lag, eq. (12)
            dv = (u_act - v) / self.tauV                    # [m/s^2]
        else:
            # Force balance on the reflected mass, eq. (14). The Coulomb
            # and load-dependent friction are smoothed over |v| < veps.
            friction_sign = max(-1.0, min(1.0, v / self.veps))
            dv = (u_act - P * self.A - self.b * v
                  - (self.Fc + self.mu * max(P, 0.0) * self.A)
                  * friction_sign) / self.mr                # [m/s^2]

        # Chamber pressure from continuity, eq. (5). The piston adds
        # volume flow A v; the outflow Q and the gas leaving the free
        # phase remove volume. The air pocket exchanges no gas.
        dP = (self.A * v - Q - Q_prox
              - self.RT / P_abs * r_dis) / C                # [Pa/s]

        # Saline and dissolved gas removed by the outflow, eqs. (7)
        # and (8). Q is a flow of liquid at pressure P, so the
        # unstressed volume it removes is larger by 1/(1 - P/K). The
        # virgin saline from the hose leaves first and carries no gas.
        dV_vir = 0.0
        if self.liquid_depletes:
            dV_liq = -Q / (1 - P / self.K)                  # [m^3/s]
            if V_vir > 0.0:
                dV_vir = dV_liq
                dV_l = 0.0
                r_out = 0.0
            else:
                dV_l = dV_liq
                r_out = -n_dis / V_l * dV_l                 # [mol/s]
        else:
            dV_l = 0.0
            r_out = 0.0

        dydt = [0.0] * N_STATES
        dydt[iy_x] = v                                      # eq. (6)
        dydt[iy_v] = dv
        dydt[iy_P] = dP
        dydt[iy_ndis] = r_dis - r_out
        dydt[iy_ntot] = -r_out
        dydt[iy_Vl] = dV_l
        dydt[iy_nout] = r_out
        dydt[iy_Vout] = Q
        dydt[iy_Vvir] = dV_vir
        # Solution admitted into the hose, unstressed, eq. (45)
        dydt[iy_Vin] = Q_prox / (1 - P / self.K)
        return dydt

    # ----- Discrete actuator logic --------------------------------------
    def slew_command(self, u):
        """Move the velocity reference toward command u [m/s] by at most
        the slew limit times one step, eq. (11), and return it."""
        error = u - self.v_ref                              # [m/s]
        # The accel limit applies while the reference grows in magnitude,
        # the decel limit while it shrinks or reverses.
        if error * self.v_ref >= 0:
            max_change = self.accel * self.dt               # [m/s]
        else:
            max_change = self.decel * self.dt               # [m/s]
        self.v_ref += max(-max_change, min(max_change, error))
        return self.v_ref

    def apply_constraints(self, y):
        """Apply the stall, the velocity clamp and the physical bounds to
        state y after a step."""
        if self.velocity_mode:
            # The drive cannot push past its torque limit, so the piston
            # stops and the drive drops its reference, eq. (13): the
            # force needed is P A (1 + mu) + Fc + b v.
            v = y[iy_v]
            P_stall = ((self.Fcap - self.Fc - self.b * max(v, 0.0))
                       / (self.A * (1.0 + self.mu)))        # [Pa]
            if y[iy_P] > P_stall and v > 0:
                y[iy_v] = 0.0
                self.v_ref = 0.0
        else:
            # The drive clamps the velocity in torque mode.
            y[iy_v] = max(-self.vmax, min(self.vmax, y[iy_v]))

        # The dissolved gas cannot exceed the gas present, the absolute
        # pressure cannot fall below 1 Pa, and the saline volumes cannot
        # fall below zero.
        y[iy_ndis] = min(y[iy_ndis], y[iy_ntot])
        y[iy_P] = max(y[iy_P], 1.0 - self.Patm)
        y[iy_Vl] = max(y[iy_Vl], 0.0)
        y[iy_Vvir] = max(y[iy_Vvir], 0.0)
        return y

    # ----- Time step ----------------------------------------------------
    def scan(self, u, valve=0.0, Q_tube=0.0, prox=0.0):
        """Advance one slave scan with command u held constant.

        u is the velocity command [m/s] in velocity mode or the force
        command [N] in torque mode. valve is the outlet valve's area
        fraction, Q_tube [m^3/s] the flow that fills the tube in the
        opening scan and prox the proximal valve's area fraction, all
        held over the scan. Returns the chamber gauge pressure [Pa].
        """
        if self.velocity_mode and self.cmd_queue:
            # The command reaches the drive after the dead time
            self.cmd_queue.append(u)
            u = self.cmd_queue.popleft()
        V_in0 = self.y[iy_Vin]
        for _ in range(self.nsub):
            if self.velocity_mode:
                if self.slew_limited:
                    u_act = self.slew_command(u)
                else:
                    u_act = self.v_ref = u
            else:
                u_act = u
            self.y = self.step(self.derivatives, self.y, self.dt,
                               u_act, valve, Q_tube, prox)
            self.apply_constraints(self.y)
        if not self.hose_open and prox > 0.0:
            # Record the solution admitted this scan against the hose
            # pressure it met, and join the hose once the pressures
            # have met with the valve fully open
            V_in = self.y[iy_Vin]
            P_h = self.hose_pressure(V_in)
            self.log_parcel(V_in - V_in0, P_h, P_h < self.P_sat(self.y))
            if prox >= 1.0 and self.y[iy_P] - P_h < 1e4:
                self.join_hose()
        if self.hose_open:
            self.Q = self.outflow(self.y[iy_P],
                                  self.y[iy_Vl] + self.y[iy_Vvir],
                                  valve, 0.0)
        else:
            self.Q = 0.0
        return self.y[iy_P]

    # ----- Outputs ------------------------------------------------------
    @property
    def P(self):
        """[Pa] chamber gauge pressure"""
        return self.y[iy_P]

    @property
    def x(self):
        """[m] piston position"""
        return self.y[iy_x]

    @property
    def v(self):
        """[m/s] piston velocity"""
        return self.y[iy_v]

    @property
    def n_dis(self):
        """[mol] gas dissolved in the saline"""
        return self.y[iy_ndis]

    @property
    def V_l(self):
        """[m^3] saline volume at zero gauge pressure"""
        return self.y[iy_Vl]

    @property
    def V_out(self):
        """[m^3] saline delivered through the outlet"""
        return self.y[iy_Vout]

    @property
    def V_virgin(self):
        """[m^3] virgin saline from the hose still in the system"""
        return self.y[iy_Vvir]

    def capacitance(self):
        """[m^3/Pa] chamber capacitance at the present state, eqs. (4),
        (42) and (46)"""
        x, v, P, n_dis, n_tot, V_l, n_out, V_out, V_vir, V_in = self.y
        P_abs = P + self.Patm
        return ((n_tot - n_dis) * self.RT / P_abs**2
                + self.nRT_air / P_abs**2
                + self.liquid_capacitance(V_l - V_in, V_vir))

    @property
    def stiffness(self):
        """[Pa/m] chamber stiffness dP/dx = A / C, eq. (10)"""
        return self.A / self.capacitance()

    @property
    def chamber_volume(self):
        """[m^3] cylinder volume (passages and swept volume) from the
        piston position; the hose, when open, is extra"""
        V = self.Vch0 - self.A * self.y[iy_x]
        if self.hose_open:
            V -= self.V_hose
        return V

    def volume_closure(self):
        """[m^3] System volume minus the volume of its contents, eq. (9).

        The system is the cylinder plus, with the hose open, the hose
        and its dilation; the contents are the stressed liquid, the free
        gas and the air pocket (eq. (46)); dissolved gas takes no
        volume. The result is zero up to the integration error when
        liquid_depletes is set.
        """
        x, v, P, n_dis, n_tot, V_l, n_out, V_out, V_vir, V_in = self.y
        system = self.chamber_volume
        if self.hose_open:
            system += self.V_hose + self.C_hose * P
        contents = ((V_l - V_in + V_vir) * (1 - P / self.K)
                    + (n_tot - n_dis) * self.RT / (P + self.Patm)
                    + self.nRT_air / (P + self.Patm))
        return system - contents
