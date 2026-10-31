/* C port of the design model's plant, design/pressure-loop/plant.py: the
 * chamber (saline and gas, Henry's-law dissolution), the outlet (hose,
 * proximal valve, orifice bank) and the actuator (velocity mode with dead
 * time, first-order lag and stall; torque mode against friction). The
 * equation numbers are those of design/pressure-loop/README.md.
 *
 * plant.py is the reference: a change to it needs the same change here,
 * and tests/plantParity.py runs both on the same inputs and requires their
 * states to agree. Only the dynamics are ported. The constructor's derived
 * values (RT, k_orifice, the initial contents, Vch0, ...) are computed by
 * plant.py and handed over through plant_init, in SI units throughout. */
#ifndef PLANT_H
#define PLANT_H

#include <stdbool.h>

/* State vector indices, as in plant.py */
enum { IY_X, IY_V, IY_P, IY_NDIS, IY_NTOT, IY_VL, IY_NOUT, IY_VOUT, IY_VVIR, IY_VIN,
       PLANT_N_STATES };

/* The parameters plant_init takes, by the names of the Plant attributes
 * they come from (n_dead is len(cmd_queue), heun is solver == 'heun'). */
#define PLANT_PARAMS(X) \
    X(A) X(RT) X(Patm) X(K) X(kH) X(tauDiss) X(Ts) X(nsub) X(heun) \
    X(Qf) X(k_orifice) X(V_passages) X(V_hose) X(C_hose) X(k_prox) \
    X(slew_limited) X(accel) X(decel) X(tauV) X(n_dead) X(Fcap) X(mu) \
    X(mr) X(b) X(Fc) X(veps) X(vmax) X(liquid_depletes)

#define PLANT_DECLARE(name) double name;
typedef struct { PLANT_PARAMS(PLANT_DECLARE) } plant_par_t;

#define PLANT_MAX_DEAD 64
#define PLANT_MAX_PARCELS 256

typedef struct {
    double V;           /* [m^3] */
    double P_h;         /* [Pa] hose pressure met */
    bool degassed;
} plant_parcel_t;

typedef struct {
    plant_par_t p;
    double dt;                      /* [s] integration step, Ts / nsub */
    bool velocity_mode;             /* else torque mode */
    double y[PLANT_N_STATES];
    double v_ref;                   /* [m/s] slewed command */
    double Q;                       /* [m^3/s] last outflow */
    double Vch0;                    /* [m^3] cylinder volume at x = 0 */
    bool hose_open;
    double queue[PLANT_MAX_DEAD];   /* command dead time, oldest first */
    int n_dead;
    plant_parcel_t parcels[PLANT_MAX_PARCELS];
    int n_parcels;
    double V_in_total, V_degas;     /* [m^3] */
} plant_t;

/* par: the PLANT_PARAMS values; y0, Vch0, hose_open, v_ref: Plant's state
 * after construction. The dead-time queue starts at zero, as in plant.py. */
void plant_init(plant_t *pl, const plant_par_t *par, bool velocity_mode,
                const double y0[PLANT_N_STATES], double Vch0, bool hose_open);

/* Plant.scan: one slave scan with u ([m/s] or [N]) held; returns P [Pa]. */
double plant_scan(plant_t *pl, double u, double valve, double Q_tube, double prox);

void plant_open_hose(plant_t *pl);
double plant_equalise(plant_t *pl);
double plant_volume_closure(const plant_t *pl);
double plant_chamber_volume(const plant_t *pl);
double plant_stiffness(const plant_t *pl);
double plant_P_sat(const plant_t *pl, const double y[PLANT_N_STATES]);

/* Flat parameter access for the Python parity test (ctypes). */
extern const char *const plant_param_names[];
extern const int plant_param_count;
void plant_init_flat(plant_t *pl, const double *par, int velocity_mode,
                     const double *y0, double Vch0, int hose_open);
int plant_size(void);
void plant_state(const plant_t *pl, double *out);   /* y and 7 more, see plant.c */

#endif
