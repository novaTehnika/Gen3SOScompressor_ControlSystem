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
it. Equations (1) to (10) of the README give the chamber model.

The outlet is the orifice bank. With the valve open, the outflow is the
orifice flow at the chamber pressure times the valve's area fraction,
eq. (36), or a prescribed flow when the scenario gives one. The tube
between the valve and the orifices holds air before the valve opens, so
in the opening scan the chamber also loses the tube's volume, eq. (37).
The delivery ends when the piston reaches the limit switch, where the
chamber volume is down to the dead volume.

The actuator has two forms. In velocity mode, which the slave uses, the
drive's profile generator limits the rate of change of the commanded
velocity, the drive's velocity loop follows that reference with a
first-order lag, and the piston stalls when the chamber force reaches
the drive's torque limit (equations (11) to (13)). In torque mode, the
commanded force accelerates the reflected mass of the motor and gearbox
against the chamber force and friction (equation (14)).

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
N_STATES = 8


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
        self.K = par.K                  # [Pa]
        self.kH = par.kH                # [mol/(m^3 Pa)]
        self.tauDiss = par.tauDiss      # [s]
        self.dt = par.Ts / par.nsub     # [s] integration step

        # Outlet parameters, eq. (36)
        self.Qf = scenario.Qf           # [m^3/s] prescribed flow, or 0
        # [m^3/s per sqrt(Pa)] orifice coefficient Cd A sqrt(2 / rho)
        self.k_orifice = par.Cd * par.A_orifice * math.sqrt(2 / par.rho)
        self.V_dead = par.V_dead        # [m^3]

        # Velocity actuator parameters
        self.accel = par.amax if accel is None else accel   # [m/s^2]
        if par.slew_asym:
            self.decel = par.dmax if decel is None else decel
        else:
            self.decel = self.accel                         # [m/s^2]
        self.tauV = par.tauV            # [s]
        # [Pa] Chamber pressure at which the drive force, less the
        # Coulomb friction, reaches the torque limit
        self.P_stall = (par.Fcap - par.Fc) / par.A

        # Torque actuator parameters
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

        # [m^3] Chamber volume at x = 0. The contents fill the chamber,
        # so it equals the stressed liquid plus the free gas (eq. (9)).
        self.Vch0 = (par.Vl0 * (1 - P0 / par.K)
                     + ng0 * self.RT / (P0 + par.Patm))

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
        """True once the chamber volume is down to the dead volume, where
        the piston reaches the limit switch and the machine halts."""
        return self.chamber_volume <= self.V_dead

    # ----- Continuous-time model ----------------------------------------
    def derivatives(self, y, u_act, valve, Q_tube):
        """Return dy/dt for state y.

        u_act is the actuator input held over the step: the slewed
        velocity reference [m/s] in velocity mode or the force [N] in
        torque mode. valve is the outlet valve's area fraction and Q_tube
        [m^3/s] the extra flow that fills the tube in the opening scan.
        """
        x, v, P, n_dis, n_tot, V_l, n_out, V_out = y

        # Absolute pressure, eq. (1)
        P_abs = P + self.Patm                               # [Pa]

        # Outflow through the outlet, eq. (36)
        Q = self.outflow(P, V_l, valve, Q_tube)             # [m^3/s]

        # Gas dissolution toward the Henry's-law equilibrium,
        # eqs. (2) and (3). The equilibrium cannot exceed the gas present.
        n_eq = min(self.kH * P_abs * V_l, n_tot)            # [mol]
        r_dis = (n_eq - n_dis) / self.tauDiss               # [mol/s]

        # Chamber capacitance [m^3/Pa], the volume change per unit
        # pressure change of the free gas plus the liquid, eq. (4)
        C = (n_tot - n_dis) * self.RT / P_abs**2 + V_l / self.K

        # Piston acceleration
        if self.velocity_mode:
            # Drive velocity loop as a first-order lag, eq. (12)
            dv = (u_act - v) / self.tauV                    # [m/s^2]
        else:
            # Force balance on the reflected mass, eq. (14). The Coulomb
            # friction is smoothed over |v| < veps.
            friction_sign = max(-1.0, min(1.0, v / self.veps))
            dv = (u_act - P * self.A - self.b * v
                  - self.Fc * friction_sign) / self.mr      # [m/s^2]

        # Chamber pressure from continuity, eq. (5). The piston adds
        # volume flow A v; the outflow Q and the gas leaving the free
        # phase remove volume.
        dP = (self.A * v - Q - self.RT / P_abs * r_dis) / C  # [Pa/s]

        # Saline and dissolved gas removed by the outflow, eqs. (7)
        # and (8). Q is a flow of liquid at pressure P, so the
        # unstressed volume it removes is larger by 1/(1 - P/K).
        if self.liquid_depletes:
            dV_l = -Q / (1 - P / self.K)                    # [m^3/s]
            r_out = -n_dis / V_l * dV_l                     # [mol/s]
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
            # stops and the drive drops its reference, eq. (13).
            if y[iy_P] > self.P_stall and y[iy_v] > 0:
                y[iy_v] = 0.0
                self.v_ref = 0.0
        else:
            # The drive clamps the velocity in torque mode.
            y[iy_v] = max(-self.vmax, min(self.vmax, y[iy_v]))

        # The dissolved gas cannot exceed the gas present, the absolute
        # pressure cannot fall below 1 Pa, and the saline volume cannot
        # fall below zero.
        y[iy_ndis] = min(y[iy_ndis], y[iy_ntot])
        y[iy_P] = max(y[iy_P], 1.0 - self.Patm)
        y[iy_Vl] = max(y[iy_Vl], 0.0)
        return y

    # ----- Time step ----------------------------------------------------
    def scan(self, u, valve=0.0, Q_tube=0.0):
        """Advance one slave scan with command u held constant.

        u is the velocity command [m/s] in velocity mode or the force
        command [N] in torque mode. valve is the outlet valve's area
        fraction and Q_tube [m^3/s] the flow that fills the tube in the
        opening scan, both held over the scan. Returns the chamber gauge
        pressure [Pa].
        """
        for _ in range(self.nsub):
            if self.velocity_mode:
                u_act = self.slew_command(u)
            else:
                u_act = u
            self.y = self.step(self.derivatives, self.y, self.dt,
                               u_act, valve, Q_tube)
            self.apply_constraints(self.y)
        self.Q = self.outflow(self.y[iy_P], self.y[iy_Vl], valve, 0.0)
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

    def capacitance(self):
        """[m^3/Pa] chamber capacitance at the present state, eq. (4)"""
        x, v, P, n_dis, n_tot, V_l, n_out, V_out = self.y
        P_abs = P + self.Patm
        return (n_tot - n_dis) * self.RT / P_abs**2 + V_l / self.K

    @property
    def stiffness(self):
        """[Pa/m] chamber stiffness dP/dx = A / C, eq. (10)"""
        return self.A / self.capacitance()

    @property
    def chamber_volume(self):
        """[m^3] chamber volume from the piston position"""
        return self.Vch0 - self.A * self.y[iy_x]

    def volume_closure(self):
        """[m^3] Chamber volume minus the volume of its contents, eq. (9).

        The contents are the stressed liquid and the free gas; dissolved
        gas takes no volume. The result is zero up to the integration
        error when liquid_depletes is set.
        """
        x, v, P, n_dis, n_tot, V_l, n_out, V_out = self.y
        contents = (V_l * (1 - P / self.K)
                    + (n_tot - n_dis) * self.RT / (P + self.Patm))
        return self.chamber_volume - contents
