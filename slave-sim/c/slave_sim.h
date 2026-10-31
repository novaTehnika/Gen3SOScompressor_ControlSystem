/* Simulated slave: the C translation of slave-src (st_slave.h, generated
 * by tools/st_translate.py --target c) plus the axis model, which stands in
 * for the LD POU's motion function blocks, the drive and the mechanics,
 * run one controller scan at a time. slaveSimMex.c exposes it to Octave and
 * MATLAB, and SlaveSim.m, slaveInit.m and slaveStep.m wrap that.
 *
 * The axis, configuration and hook fields are listed once, in the X-macros
 * below, which declare the structs and the name table the gateway uses. */
#ifndef SLAVE_SIM_H
#define SLAVE_SIM_H

#include <stdbool.h>
#include "st_slave.h"
#include "st_names.h"
#include "plant.h"

/* Roots of the hand-written groups, after ST_ROOT_G/IO/S of st_names.h. */
enum { SIM_ROOT_AX = 3, SIM_ROOT_SIM, SIM_ROOT_HOOK, SIM_ROOT_TOP };

/* Simulated axis. */
#define AX_FIELDS(X) \
    X(double, ST_F64, x)            /* physical position (mm) */ \
    X(double, ST_F64, v)            /* velocity (mm/s) */ \
    X(double, ST_F64, offset)       /* controller frame minus x (mm) */ \
    X(double, ST_F64, tPower)       \
    X(bool, ST_BOOL, prevResetExec) \
    X(bool, ST_BOOL, prevSetPosExec) \
    X(bool, ST_BOOL, prevMoveExec)  \
    X(bool, ST_BOOL, prevStopExec)  \
    X(bool, ST_BOOL, moveActive)    \
    X(double, ST_F64, moveTarget)   \
    X(double, ST_F64, moveVel)      \
    X(double, ST_F64, moveAcc)      \
    X(double, ST_F64, moveDec)      \
    X(bool, ST_BOOL, moveDone)      \
    X(bool, ST_BOOL, moveAborted)   \
    X(bool, ST_BOOL, jogBusy)       \
    X(double, ST_F64, P)            /* chamber pressure read (MPa gauge), 0 out of the cylinder */ \
    X(double, ST_F64, vOut)         /* saline delivered through the outlet (mm^3) */ \
    X(double, ST_F64, xSync)        /* x the plant was last placed at (mm) */

/* Physical parameters (slaveSimConfig.m sets every one at creation). */
#define SIM_FIELDS(X) \
    X(double, ST_F64, switchNegPos)      \
    X(double, ST_F64, switchPosPos)      \
    X(double, ST_F64, hardStopMargin)    \
    X(double, ST_F64, initialPos)        \
    X(double, ST_F64, bootFrameError)    \
    X(double, ST_F64, powerOnDelay)      \
    X(double, ST_F64, chamberContactPos) \
    X(double, ST_F64, plantStartPos)     \
    X(double, ST_F64, sensorZeroMa)      \
    X(double, ST_F64, sensorSpanMa)      \
    X(double, ST_F64, sensorRangeMPa)    \
    X(double, ST_F64, sensorShuntOhm)

/* Test hooks, read every scan (slaveInit.m lists them). */
#define HOOK_FIELDS(X) \
    X(bool, ST_BOOL, driveFault)     \
    X(bool, ST_BOOL, blocked)        \
    X(double, ST_F64, torque)        \
    X(double, ST_F64, outflow)       /* extra outflow (mm^3/s) */ \
    X(double, ST_F64, valve)         /* outlet valve area fraction (orifice bank) */ \
    X(double, ST_F64, pressureVolt)  /* NaN: the transducer model */

#define SIM_DECLARE(ctype, code, name) ctype name;
typedef struct { AX_FIELDS(SIM_DECLARE) } sim_ax_t;
typedef struct { SIM_FIELDS(SIM_DECLARE) } sim_cfg_t;
typedef struct { HOOK_FIELDS(SIM_DECLARE) } sim_hook_t;

/* The holding registers in engineering units (slaveStep.m's u).
 * positionSpeed (mm/s) is NaN when u has none, which stands for the speed
 * ceiling G_cfgVelLimitMax; salineVolume (mL) and oxygenRatio are 0 when
 * u has none. */
typedef struct {
    double heartbeat, modeCode, motionEnable, faultReset, reference;
    double positionSpeed, salineVolume, oxygenRatio;
} sim_u_t;

/* The input registers in engineering units, and the brake output pin. */
#define Y_FIELDS(X) \
    X(heartbeatEcho) X(slaveCounter) X(confCode) X(faultCode) X(faultActive) \
    X(inMotion) X(performanceStatus) X(homingComplete) X(actualPosition) \
    X(actualVelocity) X(actualTorque) X(commandedPosition) X(actualPressure) \
    X(brakeDisengage)
#define Y_DECLARE(name) double name;
typedef struct { Y_FIELDS(Y_DECLARE) } sim_y_t;

typedef struct {
    PRG_Input_t sIn;
    PRG_Main_t s;
    PRG_Output_t sOut;
    G_t G;
    IO_t IO;
    sim_ax_t ax;
    sim_cfg_t sim;
    sim_hook_t hook;
    plant_t plant;      /* chamber, outlet and drive (plant.h) */
    double t;
} slave_sim_t;

/* Power-up state; sim must be set before (initial position, frame error). */
void slave_sim_reset(slave_sim_t *sl);
/* One controller scan of G_cfgScanTime. */
void slave_sim_step(slave_sim_t *sl, const sim_u_t *u, sim_y_t *y);

/* Axis position (mm) at which the chamber, with its present contents,
 * holds gauge pressure P (MPa) at rest. */
double slave_sim_chamber_position(const slave_sim_t *sl, double P);

/* Name rows of the hand-written groups, in the format of st_names[]. */
extern const st_name_t sim_names[];
extern const int sim_names_count;

#endif
