/* C port of models/plant.py; see plant.h. Each function
 * follows the Plant method of the same name statement for statement. */
#include <math.h>
#include <string.h>
#include "plant.h"

static double dmin(double a, double b) { return a < b ? a : b; }
static double dmax(double a, double b) { return a > b ? a : b; }

/* ----- Outflow --------------------------------------------------------- */

/* [m^3/s] Outflow, eq. (36); zero once the saline is gone. */
static double outflow(const plant_t *pl, double P, double V_l, double valve, double Q_tube)
{
    if (V_l <= 0.0)
        return 0.0;
    double Q;
    if (valve > 0.0) {
        if (pl->p.Qf > 0.0)
            Q = valve * pl->p.Qf;
        else
            Q = valve * pl->p.k_orifice * sqrt(dmax(P, 0.0));
    } else {
        Q = 0.0;
    }
    return Q + Q_tube;
}

/* ----- Hose ------------------------------------------------------------ */

/* [m^3/Pa] Liquid part of the capacitance, eq. (42). */
static double liquid_capacitance(const plant_t *pl, double V_l, double V_vir)
{
    double C = (V_l + V_vir) / pl->p.K;
    if (pl->hose_open)
        C += pl->p.C_hose;
    return C;
}

double plant_P_sat(const plant_t *pl, const double y[PLANT_N_STATES])
{
    return y[IY_NDIS] / (pl->p.kH * y[IY_VL]) - pl->p.Patm;
}

static void log_parcel(plant_t *pl, double dV, double P_h, bool degassed)
{
    if (dV <= 0.0)
        return;
    pl->V_in_total += dV;
    if (degassed)
        pl->V_degas += dV;
    if (pl->n_parcels > 0 && pl->parcels[pl->n_parcels - 1].degassed == degassed) {
        plant_parcel_t *last = &pl->parcels[pl->n_parcels - 1];
        last->P_h = !degassed ? dmax(last->P_h, P_h) : last->P_h;
        last->V += dV;
    } else if (pl->n_parcels < PLANT_MAX_PARCELS) {
        pl->parcels[pl->n_parcels++] = (plant_parcel_t){dV, P_h, degassed};
    }
}

double plant_equalise(plant_t *pl)
{
    double *y = pl->y;
    double lo = 1.0 - pl->p.Patm, hi = dmax(y[IY_P], 1.0);
    for (int k = 0; k < 80; k++) {
        double mid = 0.5 * (lo + hi);
        y[IY_P] = mid;
        if (plant_volume_closure(pl) < 0)       /* contents exceed the system */
            lo = mid;
        else
            hi = mid;
    }
    y[IY_P] = 0.5 * (lo + hi);
    return y[IY_P];
}

void plant_open_hose(plant_t *pl)
{
    if (pl->hose_open)
        return;
    double *y = pl->y;
    y[IY_VVIR] += pl->p.V_hose;
    pl->hose_open = true;
    pl->Vch0 += pl->p.V_hose;
    double P1 = plant_equalise(pl);
    double K = pl->p.K;
    double V_in = ((pl->p.V_hose + pl->p.C_hose * P1 - pl->p.V_hose * (1 - P1 / K))
                   / (1 - P1 / K));
    log_parcel(pl, V_in, P1, P1 < plant_P_sat(pl, y));
}

/* [Pa] Hose gauge pressure with V_in admitted, eq. (44). */
static double hose_pressure(const plant_t *pl, double V_in)
{
    return V_in / (pl->p.C_hose + (pl->p.V_hose + V_in) / pl->p.K);
}

/* [m^3/s] Flow through the proximal valve into the shut hose, eq. (45). */
static double prox_flow(const plant_t *pl, const double *y, double prox)
{
    if (prox <= 0.0 || pl->hose_open)
        return 0.0;
    double P = y[IY_P], V_in = y[IY_VIN];
    double dP = P - hose_pressure(pl, V_in);
    if (dP <= 0.0)
        return 0.0;
    double Q = prox * pl->p.k_prox * sqrt(dP);
    double P_abs = P + pl->p.Patm;
    double C_ch = ((y[IY_NTOT] - y[IY_NDIS]) * pl->p.RT / (P_abs * P_abs)
                   + (y[IY_VL] - V_in) / pl->p.K);
    double C_h = pl->p.C_hose + (pl->p.V_hose + V_in) / pl->p.K;
    double dV_eq = dP / (1 / C_ch + 1 / C_h);
    return dmin(Q, dV_eq / pl->p.Ts);
}

static void join_hose(plant_t *pl)
{
    double *y = pl->y;
    y[IY_VVIR] += pl->p.V_hose;
    y[IY_VIN] = 0.0;
    pl->hose_open = true;
    pl->Vch0 += pl->p.V_hose;
    plant_equalise(pl);
}

/* ----- Continuous-time model --------------------------------------------- */

static void derivatives(const plant_t *pl, const double *y, double u_act, double valve,
                        double Q_tube, double prox, double *dydt)
{
    const plant_par_t *p = &pl->p;
    double v = y[IY_V], P = y[IY_P], n_dis = y[IY_NDIS], n_tot = y[IY_NTOT];
    double V_l = y[IY_VL], V_vir = y[IY_VVIR], V_in = y[IY_VIN];

    double P_abs = P + p->Patm;                                 /* eq. (1) */

    double Q, Q_prox;                                           /* eqs. (36), (45) */
    if (pl->hose_open) {
        Q = outflow(pl, P, V_l + V_vir, valve, Q_tube);
        Q_prox = 0.0;
    } else {
        Q = 0.0;
        Q_prox = prox_flow(pl, y, prox);
    }

    double n_eq = dmin(p->kH * P_abs * V_l, n_tot);             /* eqs. (2), (3) */
    double r_dis = (n_eq - n_dis) / p->tauDiss;

    double C = ((n_tot - n_dis) * p->RT / (P_abs * P_abs)       /* eqs. (4), (42) */
                + liquid_capacitance(pl, V_l - V_in, V_vir));

    double dv;
    if (pl->velocity_mode) {
        dv = (u_act - v) / p->tauV;                             /* eq. (12) */
    } else {                                                    /* eq. (14) */
        double friction_sign = dmax(-1.0, dmin(1.0, v / p->veps));
        dv = (u_act - P * p->A - p->b * v
              - (p->Fc + p->mu * dmax(P, 0.0) * p->A) * friction_sign) / p->mr;
    }

    double dP = (p->A * v - Q - Q_prox - p->RT / P_abs * r_dis) / C;   /* eq. (5) */

    double dV_vir = 0.0, dV_l, r_out;                           /* eqs. (7), (8) */
    if (p->liquid_depletes != 0) {
        double dV_liq = -Q / (1 - P / p->K);
        if (V_vir > 0.0) {
            dV_vir = dV_liq;
            dV_l = 0.0;
            r_out = 0.0;
        } else {
            dV_l = dV_liq;
            r_out = -n_dis / V_l * dV_l;
        }
    } else {
        dV_l = 0.0;
        r_out = 0.0;
    }

    dydt[IY_X] = v;                                             /* eq. (6) */
    dydt[IY_V] = dv;
    dydt[IY_P] = dP;
    dydt[IY_NDIS] = r_dis - r_out;
    dydt[IY_NTOT] = -r_out;
    dydt[IY_VL] = dV_l;
    dydt[IY_NOUT] = r_out;
    dydt[IY_VOUT] = Q;
    dydt[IY_VVIR] = dV_vir;
    dydt[IY_VIN] = Q_prox / (1 - P / p->K);                     /* eq. (45) */
}

/* solver.py: Euler, eq. (34), or Heun, eq. (35). */
static void step(const plant_t *pl, double *y, double u_act, double valve, double Q_tube,
                 double prox)
{
    double k1[PLANT_N_STATES], k2[PLANT_N_STATES], yp[PLANT_N_STATES];
    double dt = pl->dt;
    derivatives(pl, y, u_act, valve, Q_tube, prox, k1);
    if (pl->p.heun == 0) {
        for (int i = 0; i < PLANT_N_STATES; i++)
            y[i] = y[i] + dt * k1[i];
        return;
    }
    for (int i = 0; i < PLANT_N_STATES; i++)
        yp[i] = y[i] + dt * k1[i];
    derivatives(pl, yp, u_act, valve, Q_tube, prox, k2);
    double half_dt = 0.5 * dt;
    for (int i = 0; i < PLANT_N_STATES; i++)
        y[i] = y[i] + half_dt * (k1[i] + k2[i]);
}

/* ----- Discrete actuator logic ------------------------------------------- */

static double slew_command(plant_t *pl, double u)
{
    double error = u - pl->v_ref;
    double max_change = (error * pl->v_ref >= 0 ? pl->p.accel : pl->p.decel) * pl->dt;
    pl->v_ref += dmax(-max_change, dmin(max_change, error));
    return pl->v_ref;
}

static void apply_constraints(plant_t *pl, double *y)
{
    const plant_par_t *p = &pl->p;
    if (pl->velocity_mode) {
        double v = y[IY_V];                                     /* eq. (13) */
        double P_stall = (p->Fcap - p->Fc - p->b * dmax(v, 0.0)) / (p->A * (1.0 + p->mu));
        if (y[IY_P] > P_stall && v > 0) {
            y[IY_V] = 0.0;
            pl->v_ref = 0.0;
        }
    } else {
        y[IY_V] = dmax(-p->vmax, dmin(p->vmax, y[IY_V]));
    }
    y[IY_NDIS] = dmin(y[IY_NDIS], y[IY_NTOT]);
    y[IY_P] = dmax(y[IY_P], 1.0 - p->Patm);
    y[IY_VL] = dmax(y[IY_VL], 0.0);
    y[IY_VVIR] = dmax(y[IY_VVIR], 0.0);
}

/* ----- Time step ----------------------------------------------------------- */

double plant_scan(plant_t *pl, double u, double valve, double Q_tube, double prox)
{
    if (pl->velocity_mode && pl->n_dead > 0) {
        double out = pl->queue[0];
        memmove(pl->queue, pl->queue + 1, sizeof(double) * (pl->n_dead - 1));
        pl->queue[pl->n_dead - 1] = u;
        u = out;
    }
    double V_in0 = pl->y[IY_VIN];
    for (int k = 0; k < (int)pl->p.nsub; k++) {
        double u_act;
        if (pl->velocity_mode) {
            if (pl->p.slew_limited != 0)
                u_act = slew_command(pl, u);
            else
                u_act = pl->v_ref = u;
        } else {
            u_act = u;
        }
        step(pl, pl->y, u_act, valve, Q_tube, prox);
        apply_constraints(pl, pl->y);
    }
    if (!pl->hose_open && prox > 0.0) {
        double V_in = pl->y[IY_VIN];
        double P_h = hose_pressure(pl, V_in);
        log_parcel(pl, V_in - V_in0, P_h, P_h < plant_P_sat(pl, pl->y));
        if (prox >= 1.0 && pl->y[IY_P] - P_h < 1e4)
            join_hose(pl);
    }
    if (pl->hose_open)
        pl->Q = outflow(pl, pl->y[IY_P], pl->y[IY_VL] + pl->y[IY_VVIR], valve, 0.0);
    else
        pl->Q = 0.0;
    return pl->y[IY_P];
}

/* Not in plant.py: a scan with the piston velocity prescribed, for the
 * simulated slave's motion function blocks that set the motion themselves
 * (position control, MoveAbsolute, Jog, Stop). v [m/s] is held over the
 * scan with no dead time or lag; the chamber integrates along it and the
 * stall of eq. (13) still stops the piston, for the rest of the scan. The
 * dead-time queue is filled with v, so that a velocity-mode scan that
 * follows continues from it. Built from the parts tests/plantParity.py
 * covers: the velocity-mode derivatives with the command equal to the
 * velocity (dv = 0), the step and the constraints. */
double plant_scan_prescribed(plant_t *pl, double v, double valve, double Q_tube)
{
    for (int k = 0; k < pl->n_dead; k++)
        pl->queue[k] = v;
    bool velocity_mode = pl->velocity_mode;
    pl->velocity_mode = true;
    pl->y[IY_V] = v;
    for (int k = 0; k < (int)pl->p.nsub; k++) {
        pl->v_ref = pl->y[IY_V];
        step(pl, pl->y, pl->y[IY_V], valve, Q_tube, 0.0);
        apply_constraints(pl, pl->y);
    }
    pl->velocity_mode = velocity_mode;
    if (pl->hose_open)
        pl->Q = outflow(pl, pl->y[IY_P], pl->y[IY_VL] + pl->y[IY_VVIR], valve, 0.0);
    else
        pl->Q = 0.0;
    return pl->y[IY_P];
}

/* ----- Outputs --------------------------------------------------------------- */

static double capacitance(const plant_t *pl)
{
    const double *y = pl->y;
    double P_abs = y[IY_P] + pl->p.Patm;
    return ((y[IY_NTOT] - y[IY_NDIS]) * pl->p.RT / (P_abs * P_abs)
            + liquid_capacitance(pl, y[IY_VL] - y[IY_VIN], y[IY_VVIR]));
}

double plant_stiffness(const plant_t *pl)
{
    return pl->p.A / capacitance(pl);                           /* eq. (10) */
}

double plant_chamber_volume(const plant_t *pl)
{
    double V = pl->Vch0 - pl->p.A * pl->y[IY_X];
    if (pl->hose_open)
        V -= pl->p.V_hose;
    return V;
}

double plant_volume_closure(const plant_t *pl)                  /* eq. (9) */
{
    const double *y = pl->y;
    double P = y[IY_P];
    double system = plant_chamber_volume(pl);
    if (pl->hose_open)
        system += pl->p.V_hose + pl->p.C_hose * P;
    double contents = ((y[IY_VL] - y[IY_VIN] + y[IY_VVIR]) * (1 - P / pl->p.K)
                       + (y[IY_NTOT] - y[IY_NDIS]) * pl->p.RT / (P + pl->p.Patm));
    return system - contents;
}

/* ----- Construction ---------------------------------------------------------- */

void plant_init(plant_t *pl, const plant_par_t *par, bool velocity_mode,
                const double y0[PLANT_N_STATES], double Vch0, bool hose_open)
{
    memset(pl, 0, sizeof *pl);
    pl->p = *par;
    pl->dt = par->Ts / par->nsub;
    pl->velocity_mode = velocity_mode;
    memcpy(pl->y, y0, sizeof pl->y);
    pl->Vch0 = Vch0;
    pl->hose_open = hose_open;
    pl->n_dead = (int)par->n_dead;
    if (pl->n_dead > PLANT_MAX_DEAD)
        pl->n_dead = PLANT_MAX_DEAD;
}

#define PLANT_NAME(name) #name,
const char *const plant_param_names[] = {PLANT_PARAMS(PLANT_NAME)};
const int plant_param_count = sizeof plant_param_names / sizeof plant_param_names[0];

void plant_init_flat(plant_t *pl, const double *par, int velocity_mode, const double *y0,
                     double Vch0, int hose_open)
{
    plant_par_t p;
    memcpy(&p, par, sizeof p);
    plant_init(pl, &p, velocity_mode != 0, y0, Vch0, hose_open != 0);
}

int plant_size(void)
{
    return (int)sizeof(plant_t);
}

/* State for the parity test: y, then v_ref, Q, Vch0, hose_open,
 * V_in_total, V_degas and the number of parcels. */
void plant_state(const plant_t *pl, double *out)
{
    memcpy(out, pl->y, sizeof pl->y);
    double *o = out + PLANT_N_STATES;
    o[0] = pl->v_ref;
    o[1] = pl->Q;
    o[2] = pl->Vch0;
    o[3] = pl->hose_open;
    o[4] = pl->V_in_total;
    o[5] = pl->V_degas;
    o[6] = pl->n_parcels;
}
