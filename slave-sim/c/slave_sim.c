/* Simulated slave: see slave_sim.h. */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "slave_sim.h"
#include "plant_defaults.h"

/* Plant position (m) of the axis at x (mm). */
static double plant_x(const sim_cfg_t *sim, double x)
{
    return (x - sim->plantStartPos) * 1e-3;                     /* [mm -> m] */
}

/* Set the plant's pressure to the one at which its contents fill the
 * system, the volume balance of eq. (9), by bisection over vacuum to
 * 200 MPa. (plant_equalise searches only below the present pressure, as
 * the design model needs when a valve opens.) Returns P [Pa]. */
static double settle(plant_t *pl)
{
    double lo = 1.0 - pl->p.Patm, hi = 200e6;
    for (int k = 0; k < 100; k++) {
        pl->y[IY_P] = 0.5 * (lo + hi);
        if (plant_volume_closure(pl) < 0)       /* contents exceed the system */
            lo = pl->y[IY_P];
        else
            hi = pl->y[IY_P];
    }
    pl->y[IY_P] = 0.5 * (lo + hi);
    return pl->y[IY_P];
}

/* Place the plant's piston at the axis position x (mm), at rest, and
 * settle the chamber pressure there. */
static void place_plant(slave_sim_t *sl, double x)
{
    sl->plant.y[IY_X] = plant_x(&sl->sim, x);
    sl->plant.y[IY_V] = 0;
    settle(&sl->plant);
    sl->ax.xSync = x;
}

double slave_sim_chamber_position(const slave_sim_t *sl, double P)
{
    static plant_t pl;
    double lo = sl->sim.chamberContactPos;
    double hi = sl->sim.switchPosPos + sl->sim.hardStopMargin;
    for (int k = 0; k < 60; k++) {
        double mid = 0.5 * (lo + hi);
        pl = sl->plant;
        pl.y[IY_X] = plant_x(&sl->sim, mid);
        if (settle(&pl) * 1e-6 < P)                             /* [Pa -> MPa] */
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}

void slave_sim_reset(slave_sim_t *sl)
{
    init_PRG_Input(&sl->sIn);
    init_PRG_Main(&sl->s);
    init_PRG_Output(&sl->sOut);
    init_G(&sl->G);
    init_IO(&sl->IO);
    memset(&sl->ax, 0, sizeof sl->ax);
    sl->ax.x = sl->sim.initialPos;
    sl->ax.offset = sl->sim.bootFrameError;
    memset(&sl->hook, 0, sizeof sl->hook);
    sl->hook.pressureVolt = NAN;
    plant_init(&sl->plant, &plant_default_par, true, plant_default_y0, plant_default_Vch0,
               plant_default_hose_open);
    place_plant(sl, sl->ax.x);
    sl->ax.P = sl->ax.x < sl->sim.chamberContactPos ? 0 : sl->plant.y[IY_P] * 1e-6;
    sl->t = 0;
}

/* Move v toward target by at most step. */
static double slew(double v, double target, double step)
{
    double d = target - v;
    if (d > step)
        d = step;
    if (d < -step)
        d = -step;
    return v + d;
}

static double sign(double x) { return (x > 0) - (x < 0); }

/* The LD POU's motion function blocks, the drive and the mechanics. Reads G_cmd*, writes G_sta* and G_sysActual*. */
static void axis_model(slave_sim_t *sl, double dt)
{
    sim_ax_t *ax = &sl->ax;
    const sim_cfg_t *sim = &sl->sim;
    const sim_hook_t *hook = &sl->hook;
    G_t *G = &sl->G;

    /* MC_Power */
    if (G->G_cmdPower.Enable)
        ax->tPower = fmin(ax->tPower + dt, sim->powerOnDelay);
    else
        ax->tPower = 0;
    bool powered = ax->tPower >= sim->powerOnDelay;
    G->G_staPower.Status = powered;
    G->G_staPower.Error = false;

    /* MC_Reset */
    G->G_staReset.Done = G->G_cmdReset.Execute && ax->prevResetExec;
    G->G_staReset.Busy = false;
    G->G_staReset.Error = false;
    ax->prevResetExec = G->G_cmdReset.Execute;

    /* MC_SetPosition */
    if (G->G_cmdSetPosition.Execute && !ax->prevSetPosExec) {
        if (G->G_cmdSetPosition.Mode)
            ax->offset = ax->offset + G->G_cmdSetPosition.Position;
        else
            ax->offset = G->G_cmdSetPosition.Position - ax->x;
    }
    G->G_staSetPosition.Done = G->G_cmdSetPosition.Execute;
    G->G_staSetPosition.Busy = false;
    G->G_staSetPosition.Error = false;
    ax->prevSetPosExec = G->G_cmdSetPosition.Execute;

    /* MC_MoveAbsolute: start on rising Execute */
    bool moveExec = G->G_cmdMoveAbsolute.Execute;
    if (moveExec && !ax->prevMoveExec && powered && !G->G_cmdStop.Execute) {
        ax->moveActive = true;
        ax->moveDone = false;
        ax->moveAborted = false;
        ax->moveTarget = G->G_cmdMoveAbsolute.Position - ax->offset;
        ax->moveVel = fabs(G->G_cmdMoveAbsolute.Velocity);
        ax->moveAcc = G->G_cmdMoveAbsolute.Acceleration;
        ax->moveDec = G->G_cmdMoveAbsolute.Deceleration;
    }
    bool moveDoneOut = ax->moveDone;
    if (ax->moveDone && !moveExec)
        ax->moveDone = false;
    if (!moveExec)
        ax->moveAborted = false;
    ax->prevMoveExec = moveExec;

    /* Motion arbitration */
    bool stopExec = G->G_cmdStop.Execute;
    bool dcEnable = G->G_cmdDirectControl.Enable;
    bool jogCmd = G->G_cmdJog.Forward || G->G_cmdJog.Reverse;
    bool jogDone = false;
    bool dcActive = false;
    double xDirect = ax->x;
    bool directPosition = false;
    /* What drives the piston this scan: the motion profile above (v, or a
     * position for directPosition), or the plant's drive model for the
     * velocity and torque modes of Y_DirectControl. */
    enum { DRIVE_PROFILE, DRIVE_VELOCITY, DRIVE_TORQUE } drive = DRIVE_PROFILE;
    double uDrive = 0;

    if (!powered) {
        ax->v = 0;
        ax->moveActive = false;
        ax->jogBusy = false;
    } else if (stopExec) {
        if (ax->moveActive) {
            ax->moveActive = false;
            ax->moveAborted = true;
        }
        ax->jogBusy = false;
        ax->v = slew(ax->v, 0, G->G_cmdStop.Deceleration * dt);
    } else if (dcEnable) {
        dcActive = true;
        if (ax->moveActive) {
            ax->moveActive = false;
            ax->moveAborted = true;
        }
        ax->jogBusy = false;
        int mode = G->G_cmdDirectControl.ControlMode;
        if (mode == Y_ControlMode__PositionMode) {
            directPosition = true;
            xDirect = G->G_cmdDirectControl.Position - ax->offset;
        } else if (mode == Y_ControlMode__VelocityTLMode) {
            /* The drive follows the command after its dead time and lag,
             * eqs. (11) and (12), and stalls at its torque, eq. (13). */
            drive = DRIVE_VELOCITY;
            uDrive = G->G_cmdDirectControl.Velocity * 1e-3;     /* [mm/s -> m/s] */
        } else {
            /* Torque against the chamber and friction, eq. (14), at the
             * command's velocity limit. */
            drive = DRIVE_TORQUE;
            uDrive = G->G_cmdDirectControl.Torque * 0.01 * sl->plant.p.Fcap;  /* [% -> N] */
            sl->plant.p.vmax = fabs(G->G_cmdDirectControl.Velocity) * 1e-3;   /* [mm/s -> m/s] */
        }
    } else if (ax->moveActive) {
        double dist = ax->moveTarget - ax->x;
        double aDt = ax->moveDec * dt;
        double rateStop = -aDt + sqrt(aDt * aDt + 2 * ax->moveDec * fabs(dist));
        double vT = sign(dist) * fmin(ax->moveVel, rateStop);
        double rate = (fabs(vT) > fabs(ax->v) && vT * ax->v >= 0) ? ax->moveAcc : ax->moveDec;
        ax->v = slew(ax->v, vT, rate * dt);
        if (fabs(dist) <= fabs(ax->v) * dt + 1e-9 && fabs(ax->v) <= aDt + 1e-9) {
            directPosition = true;
            xDirect = ax->moveTarget;
            ax->moveActive = false;
            ax->moveDone = true;
            moveDoneOut = moveExec;
        }
    } else if (jogCmd || ax->jogBusy) {
        if (G->G_cmdJog.Forward && !G->G_cmdJog.Reverse) {
            ax->v = slew(ax->v, fabs(G->G_cmdJog.Velocity), G->G_cmdJog.Acceleration * dt);
            ax->jogBusy = true;
        } else if (G->G_cmdJog.Reverse && !G->G_cmdJog.Forward) {
            ax->v = slew(ax->v, -fabs(G->G_cmdJog.Velocity), G->G_cmdJog.Acceleration * dt);
            ax->jogBusy = true;
        } else {
            ax->v = slew(ax->v, 0, G->G_cmdJog.Deceleration * dt);
            if (ax->v == 0) {
                ax->jogBusy = false;
                jogDone = true;
            }
        }
    } else {
        ax->v = 0;
    }

    /* Integrate through the plant: the chamber, and the drive in the
     * velocity and torque modes. A position set from outside (a test
     * placing the axis) moves the plant's piston there first. */
    plant_t *pl = &sl->plant;
    if (ax->x != ax->xSync)
        place_plant(sl, ax->x);
    double xPrev = ax->x;
    double valve = hook->valve;
    double Q_tube = hook->outflow * 1e-9;                       /* [mm^3/s -> m^3/s] */
    if (hook->blocked || drive == DRIVE_PROFILE) {
        double vProfile = 0;                                    /* [mm/s] */
        if (!hook->blocked)
            vProfile = directPosition ? (xDirect - ax->x) / dt : ax->v;
        plant_scan_prescribed(pl, vProfile * 1e-3, valve, Q_tube);
        if (pl->y[IY_V] != 0 || vProfile == 0) {
            /* Not stalled: the axis is where the profile put it, exactly
             * (the status logic compares velocities), and the plant with it. */
            ax->x = directPosition && !hook->blocked ? xDirect : xPrev + vProfile * dt;
            ax->v = vProfile;
            pl->y[IY_X] = plant_x(sim, ax->x);
        } else {
            ax->x = pl->y[IY_X] * 1e3 + sim->plantStartPos;     /* [m -> mm] */
            ax->v = 0;
        }
    } else {
        pl->velocity_mode = drive == DRIVE_VELOCITY;
        plant_scan(pl, uDrive, valve, Q_tube, 0.0);
        ax->x = pl->y[IY_X] * 1e3 + sim->plantStartPos;         /* [m -> mm] */
        ax->v = pl->y[IY_V] * 1e3;                              /* [m/s -> mm/s] */
    }
    double xMin = sim->switchNegPos - sim->hardStopMargin;
    double xMax = sim->switchPosPos + sim->hardStopMargin;
    if (ax->x < xMin || ax->x > xMax) {
        place_plant(sl, fmin(fmax(ax->x, xMin), xMax));
        ax->x = ax->xSync;
        ax->v = 0;
    }
    if (directPosition && !hook->blocked)
        ax->v = (ax->x - xPrev) / dt;
    ax->xSync = ax->x;

    /* Chamber pressure as read: the cylinder is open before the piston
     * enters it. */
    ax->vOut = pl->y[IY_VOUT] * 1e9;                            /* [m^3 -> mm^3] */
    ax->P = ax->x < sim->chamberContactPos ? 0 : pl->y[IY_P] * 1e-6;   /* [Pa -> MPa] */

    /* Status */
    G->G_staStop.Busy = stopExec && ax->v != 0;
    G->G_staStop.Done = stopExec && ax->v == 0 && ax->prevStopExec;
    G->G_staStop.Error = false;
    ax->prevStopExec = stopExec;

    G->G_staDirectControl.Active = dcActive;
    G->G_staDirectControl.Error = false;

    G->G_staMoveAbsolute.Busy = ax->moveActive;
    G->G_staMoveAbsolute.Active = ax->moveActive;
    G->G_staMoveAbsolute.Done = moveDoneOut;
    G->G_staMoveAbsolute.CommandAborted = ax->moveAborted;
    G->G_staMoveAbsolute.Error = false;

    G->G_staJog.Busy = ax->jogBusy;
    G->G_staJog.InVelocity = ax->jogBusy && jogCmd && fabs(ax->v) == fabs(G->G_cmdJog.Velocity);
    G->G_staJog.Done = jogDone;
    G->G_staJog.Error = false;

    G->G_sysActualPosition = ax->x + ax->offset;
    G->G_sysActualVelocity = ax->v;
    G->G_sysActualTorque = hook->torque;
    G->G_sysDriveFault = hook->driveFault;
}

/* One scan: inputs -> PRG_Input -> PRG_Main -> PRG_Output -> axis. */
void slave_sim_step(slave_sim_t *sl, const sim_u_t *u, sim_y_t *y)
{
    double dt = sl->G.G_cfgScanTime;
    const sim_cfg_t *sim = &sl->sim;

    sl->G.G_mbHeartbeat = u->heartbeat;
    sl->G.G_mbModeCode = u->modeCode;
    sl->G.G_mbMotionEnable = u->motionEnable != 0;
    sl->G.G_mbFaultReset = u->faultReset != 0;
    sl->G.G_mbReference = u->reference;
    sl->G.G_mbPositionSpeed = u->positionSpeed;
    /* Overtravel switches are normally closed: TRUE = not triggered. */
    sl->IO.MO1_DI_06 = sl->ax.x > sim->switchNegPos;
    sl->IO.MO1_DI_07 = sl->ax.x < sim->switchPosPos;
    if (isnan(sl->hook.pressureVolt)) {
        /* Transducer: chamber pressure as loop current across the shunt. */
        double mA = sim->sensorZeroMa + sim->sensorSpanMa * sl->ax.P / sim->sensorRangeMPa;
        sl->IO.MO1_AI_01 = mA * sim->sensorShuntOhm / 1000;
    } else {
        sl->IO.MO1_AI_01 = sl->hook.pressureVolt;
    }

    PRG_Input(&sl->sIn, &sl->G, &sl->IO, dt);
    PRG_Main(&sl->s, &sl->G, &sl->IO, dt);
    PRG_Output(&sl->sOut, &sl->G, &sl->IO, dt);
    axis_model(sl, dt);
    sl->t = sl->t + dt;

    y->heartbeatEcho = sl->G.G_mbHeartbeatEcho;
    y->slaveCounter = sl->G.G_mbSlaveCounter;
    y->confCode = sl->G.G_mbConfCode;
    y->faultCode = sl->G.G_mbFaultCode;
    y->faultActive = sl->G.G_mbFaultActive;
    y->inMotion = sl->G.G_mbInMotion;
    y->performanceStatus = sl->G.G_mbPerformanceStatus;
    y->homingComplete = sl->G.G_mbHomingComplete;
    y->actualPosition = sl->G.G_mbActualPosition;
    y->actualVelocity = sl->G.G_mbActualVelocity;
    y->actualTorque = sl->G.G_mbActualTorque;
    y->commandedPosition = sl->G.G_mbCommandedPosition;
    y->actualPressure = sl->G.G_mbActualPressure;
    y->brakeDisengage = sl->IO.MO1_DO_07;
}

/* Name rows: offsets are from the start of slave_sim_t for these groups. */
#define AX_ROW(ctype, code, name) {"ax." #name, SIM_ROOT_AX, code, offsetof(slave_sim_t, ax.name), 1},
#define CFG_ROW(ctype, code, name) {"sim." #name, SIM_ROOT_SIM, code, offsetof(slave_sim_t, sim.name), 1},
#define HOOK_ROW(ctype, code, name) {"hook." #name, SIM_ROOT_HOOK, code, offsetof(slave_sim_t, hook.name), 1},

const st_name_t sim_names[] = {
    AX_FIELDS(AX_ROW)
    SIM_FIELDS(CFG_ROW)
    HOOK_FIELDS(HOOK_ROW)
    {"t", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, t), 1},
    /* The plant's state in SI units (plant.h) */
    {"plant.x", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_X]), 1},
    {"plant.v", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_V]), 1},
    {"plant.P", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_P]), 1},
    {"plant.n_dis", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_NDIS]), 1},
    {"plant.n_tot", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_NTOT]), 1},
    {"plant.V_l", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_VL]), 1},
    {"plant.V_out", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_VOUT]), 1},
    {"plant.V_vir", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.y[IY_VVIR]), 1},
    {"plant.Q", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.Q), 1},
    {"plant.v_ref", SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.v_ref), 1},
    {"plant.n_dead", SIM_ROOT_TOP, ST_I32, offsetof(slave_sim_t, plant.n_dead), 1},
    /* Its parameters (PLANT_PARAMS of plant.h, SI units) */
#define PAR_ROW(name) {"plant.par." #name, SIM_ROOT_TOP, ST_F64, offsetof(slave_sim_t, plant.p.name), 1},
    PLANT_PARAMS(PAR_ROW)
};
const int sim_names_count = sizeof sim_names / sizeof sim_names[0];
