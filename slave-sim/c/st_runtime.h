/* Runtime of the C target of tools/st_translate.py: the IEC standard
 * function blocks and the helpers the generated code calls. The helpers
 * follow the py target's runtime (st_min, st_max, st_mod, st_round, ...),
 * so the two targets agree (tests/stTargetParity.py). */
#ifndef ST_RUNTIME_H
#define ST_RUNTIME_H

#include <math.h>
#include <stdbool.h>
#include "st_slave.h"

/* TON: Q once IN has been TRUE for PT seconds; IN FALSE resets it. */
static inline void TON(TON_t *s, double dt)
{
    if (!s->IN) {
        s->ET = 0;
        s->Q = false;
    } else {
        s->ET = s->M ? s->ET + dt : 0;
        s->Q = s->ET >= s->PT;
    }
    s->M = s->IN;
}

static inline void R_TRIG(R_TRIG_t *s, double dt)
{
    (void)dt;
    s->Q = s->CLK && !s->M;
    s->M = s->CLK;
}

static inline void F_TRIG(F_TRIG_t *s, double dt)
{
    (void)dt;
    s->Q = !s->CLK && !s->M;
    s->M = !s->CLK;
}

/* MIN and MAX ignore a NaN operand, as Octave's min and max do. */
static inline double st_min(double a, double b) { return (a != a || b < a) ? b : a; }
static inline double st_max(double a, double b) { return (a != a || b > a) ? b : a; }
static inline double st_limit(double mn, double x, double mx) { return st_min(st_max(x, mn), mx); }
static inline double st_abs(double x) { return fabs(x); }
static inline double st_sqrt(double x) { return sqrt(x); }

/* REAL_TO_INT and friends: nearest, halves away from zero (Octave round). */
static inline double st_round(double x) { return round(x); }

/* Integer division truncates toward zero (Octave fix of the quotient). */
static inline long st_idiv(long a, long b) { return b == 0 ? 0 : a / b; }

/* MOD with the divisor's sign, and mod(a, 0) = a, as Octave's mod. */
static inline double st_mod(double a, double b)
{
    if (b == 0)
        return a;
    double r = fmod(a, b);
    return (r != 0 && ((r < 0) != (b < 0))) ? r + b : r;
}

#endif
