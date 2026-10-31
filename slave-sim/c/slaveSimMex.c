/* MEX gateway of the compiled simulated slave (slave_sim.h). It uses only
 * the MEX API, so it builds with MATLAB's mex and Octave's mkoctfile --mex
 * (slave-sim/buildSlaveSim.m). SlaveSim.m wraps it; tests do not call it
 * directly.
 *
 *   id = slaveSimMex('new')                 a slave at power-up, sim fields zero
 *   slaveSimMex('reset', id)                power-up state from the sim fields
 *   slaveSimMex('free', id)
 *   y  = slaveSimMex('step', id, u)         one scan; u, y as slaveStep.m
 *   y  = slaveSimMex('step', id, u, n)      n scans with the same u
 *   y  = slaveSimMex('scan', id[, n])        scans on the holding registers as they stand
 *   w  = slaveSimMex('readRegisters', id, group, start, n)   group 'holding' or 'input'
 *   slaveSimMex('writeRegisters', id, group, start, words)   16-bit words, 1-based
 *   R  = slaveSimMex('registers')           the register table from the AT addresses
 *   v  = slaveSimMex('get', id, path)       'ax.x', 'G.G_cfgScanTime', 'sim', ...
 *   slaveSimMex('set', id, path, value)
 *   E  = slaveSimMex('enums')               struct of the enumerations
 *   x  = slaveSimMex('chamberPosition', id, P)  axis mm where the chamber holds P MPa
 *   c  = slaveSimMex('names')               every readable path, sorted
 *   c  = slaveSimMex('values', id)          their values, in the same order
 *
 * A path that names a group ('G', 'ax', 's.fbHomeLimit') returns a struct
 * of everything under it. BOOL values come back as logical. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mex.h"
#include "slave_sim.h"

#define MAX_INSTANCES 64

static slave_sim_t *instances[MAX_INSTANCES];
static const st_name_t **index_rows;    /* every name row, sorted by name */
static int index_count;

static void free_all(void)
{
    for (int k = 0; k < MAX_INSTANCES; k++) {
        free(instances[k]);
        instances[k] = NULL;
    }
    free(index_rows);
    index_rows = NULL;
}

static int by_name(const void *a, const void *b)
{
    return strcmp((*(const st_name_t *const *)a)->name, (*(const st_name_t *const *)b)->name);
}

static void build_index(void)
{
    if (index_rows)
        return;
    index_count = st_names_count + sim_names_count;
    index_rows = malloc(sizeof *index_rows * index_count);
    for (int k = 0; k < st_names_count; k++)
        index_rows[k] = &st_names[k];
    for (int k = 0; k < sim_names_count; k++)
        index_rows[st_names_count + k] = &sim_names[k];
    qsort(index_rows, index_count, sizeof *index_rows, by_name);
    mexAtExit(free_all);
}

/* First row whose name is >= key. */
static int lower_bound(const char *key)
{
    int lo = 0, hi = index_count;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (strcmp(index_rows[mid]->name, key) < 0)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

static char *address(slave_sim_t *sl, const st_name_t *row)
{
    char *base;
    switch (row->root) {
    case ST_ROOT_G: base = (char *)&sl->G; break;
    case ST_ROOT_IO: base = (char *)&sl->IO; break;
    case ST_ROOT_S: base = (char *)&sl->s; break;
    default: base = (char *)sl; break;     /* hand-written rows: offsets in slave_sim_t */
    }
    return base + row->offset;
}

static double read_elem(const char *p, int type, int k)
{
    switch (type) {
    case ST_BOOL: return ((const bool *)p)[k];
    case ST_I8: return ((const int8_t *)p)[k];
    case ST_U8: return ((const uint8_t *)p)[k];
    case ST_I16: return ((const int16_t *)p)[k];
    case ST_U16: return ((const uint16_t *)p)[k];
    case ST_I32: return ((const int32_t *)p)[k];
    case ST_U32: return ((const uint32_t *)p)[k];
    case ST_F32: return ((const float *)p)[k];
    default: return ((const double *)p)[k];
    }
}

static void write_elem(char *p, int type, int k, double v)
{
    switch (type) {
    case ST_BOOL: ((bool *)p)[k] = v != 0; break;
    case ST_I8: ((int8_t *)p)[k] = (int8_t)llround(v); break;
    case ST_U8: ((uint8_t *)p)[k] = (uint8_t)llround(v); break;
    case ST_I16: ((int16_t *)p)[k] = (int16_t)llround(v); break;
    case ST_U16: ((uint16_t *)p)[k] = (uint16_t)llround(v); break;
    case ST_I32: ((int32_t *)p)[k] = (int32_t)llround(v); break;
    case ST_U32: ((uint32_t *)p)[k] = (uint32_t)llround(v); break;
    case ST_F32: ((float *)p)[k] = (float)v; break;
    default: ((double *)p)[k] = v; break;
    }
}

static mxArray *leaf_value(slave_sim_t *sl, const st_name_t *row)
{
    const char *p = address(sl, row);
    if (row->type == ST_BOOL && row->count == 1)
        return mxCreateLogicalScalar(*(const bool *)p);
    mxArray *a = mxCreateDoubleMatrix(1, row->count, mxREAL);
    double *d = mxGetPr(a);
    for (int k = 0; k < row->count; k++)
        d[k] = read_elem(p, row->type, k);
    return a;
}

/* Struct of the rows [lo, hi) whose names all start with prefix + ".". */
static mxArray *group_value(slave_sim_t *sl, const char *prefix, int lo, int hi)
{
    size_t plen = strlen(prefix) + 1;
    const char **names = mxCalloc(hi - lo, sizeof *names);
    size_t *lens = mxCalloc(hi - lo, sizeof *lens);
    int nf = 0;
    for (int k = lo; k < hi; k++) {
        const char *rest = index_rows[k]->name + plen;
        size_t len = strcspn(rest, ".");
        if (nf == 0 || lens[nf - 1] != len || strncmp(names[nf - 1], rest, len) != 0) {
            names[nf] = rest;
            lens[nf] = len;
            nf++;
        }
    }
    char **fields = mxCalloc(nf, sizeof *fields);
    for (int f = 0; f < nf; f++) {
        fields[f] = mxCalloc(lens[f] + 1, 1);
        memcpy(fields[f], names[f], lens[f]);
    }
    mxArray *s = mxCreateStructMatrix(1, 1, nf, (const char **)fields);
    int k = lo;
    for (int f = 0; f < nf; f++) {
        char *sub = mxCalloc(plen + lens[f] + 1, 1);
        memcpy(sub, index_rows[k]->name, plen + lens[f]);
        int end = k;
        while (end < hi && strncmp(index_rows[end]->name, sub, plen + lens[f]) == 0
               && (index_rows[end]->name[plen + lens[f]] == '\0'
                   || index_rows[end]->name[plen + lens[f]] == '.'))
            end++;
        if (end == k + 1 && index_rows[k]->name[plen + lens[f]] == '\0')
            mxSetFieldByNumber(s, 0, f, leaf_value(sl, index_rows[k]));
        else
            mxSetFieldByNumber(s, 0, f, group_value(sl, sub, k, end));
        k = end;
    }
    return s;
}

static slave_sim_t *instance(const mxArray *a)
{
    int id = (int)mxGetScalar(a);
    if (id < 1 || id > MAX_INSTANCES || !instances[id - 1])
        mexErrMsgIdAndTxt("slaveSimMex:id", "no simulated slave %d", id);
    return instances[id - 1];
}

static double field(const mxArray *u, const char *name)
{
    mxArray *f = mxGetField(u, 0, name);
    if (!f)
        mexErrMsgIdAndTxt("slaveSimMex:u", "u has no field %s", name);
    return mxGetScalar(f);
}

static const char *Y_NAMES[] = {
#define Y_NAME(name) #name,
    Y_FIELDS(Y_NAME)
};

static mxArray *y_struct(const sim_y_t *y)
{
    int n = sizeof Y_NAMES / sizeof Y_NAMES[0];
    mxArray *s = mxCreateStructMatrix(1, 1, n, Y_NAMES);
    const double *v = (const double *)y;
    for (int k = 0; k < n; k++)
        mxSetFieldByNumber(s, 0, k, mxCreateDoubleScalar(v[k]));
    return s;
}

/* 'holding' or 'input' */
static int group_arg(const mxArray *a)
{
    char g[16];
    if (mxGetString(a, g, sizeof g) || (strcmp(g, "holding") && strcmp(g, "input")))
        mexErrMsgIdAndTxt("slaveSimMex:group", "the register group is 'holding' or 'input'");
    return !strcmp(g, "holding");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char cmd[16];
    if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof cmd))
        mexErrMsgIdAndTxt("slaveSimMex:cmd", "first argument must be a command");
    build_index();

    if (!strcmp(cmd, "new")) {
        for (int k = 0; k < MAX_INSTANCES; k++) {
            if (!instances[k]) {
                instances[k] = calloc(1, sizeof(slave_sim_t));
                slave_sim_reset(instances[k]);
                plhs[0] = mxCreateDoubleScalar(k + 1);
                return;
            }
        }
        mexErrMsgIdAndTxt("slaveSimMex:full", "more than %d simulated slaves", MAX_INSTANCES);
    } else if (!strcmp(cmd, "free")) {
        int id = (int)mxGetScalar(prhs[1]);
        if (id >= 1 && id <= MAX_INSTANCES) {
            free(instances[id - 1]);
            instances[id - 1] = NULL;
        }
    } else if (!strcmp(cmd, "reset")) {
        slave_sim_reset(instance(prhs[1]));
    } else if (!strcmp(cmd, "step")) {
        slave_sim_t *sl = instance(prhs[1]);
        sim_u_t u = {field(prhs[2], "heartbeat"), field(prhs[2], "modeCode"),
                     field(prhs[2], "motionEnable"), field(prhs[2], "faultReset"),
                     field(prhs[2], "reference")};
        sim_y_t y;
        int scans = nrhs > 3 ? (int)mxGetScalar(prhs[3]) : 1;
        for (int k = 0; k < scans; k++)
            slave_sim_step(sl, &u, &y);
        plhs[0] = y_struct(&y);
    } else if (!strcmp(cmd, "scan")) {
        slave_sim_t *sl = instance(prhs[1]);
        sim_y_t y;
        int scans = nrhs > 2 ? (int)mxGetScalar(prhs[2]) : 1;
        for (int k = 0; k < scans; k++)
            slave_sim_scan(sl, &y);
        plhs[0] = y_struct(&y);
    } else if (!strcmp(cmd, "readRegisters") || !strcmp(cmd, "writeRegisters")) {
        slave_sim_t *sl = instance(prhs[1]);
        int holding = group_arg(prhs[2]);
        int start = (int)mxGetScalar(prhs[3]);
        if (!strcmp(cmd, "readRegisters")) {
            int n = (int)mxGetScalar(prhs[4]);
            uint16_t *words = mxCalloc(n > 0 ? n : 1, sizeof *words);
            slave_sim_read_registers(sl, holding, start, n, words);
            plhs[0] = mxCreateDoubleMatrix(1, n, mxREAL);
            for (int k = 0; k < n; k++)
                mxGetPr(plhs[0])[k] = words[k];
        } else {
            int n = (int)mxGetNumberOfElements(prhs[4]);
            uint16_t *words = mxCalloc(n > 0 ? n : 1, sizeof *words);
            for (int k = 0; k < n; k++) {
                double v = mxGetPr(prhs[4])[k];
                if (v < 0 || v > 65535 || v != floor(v))
                    mexErrMsgIdAndTxt("slaveSimMex:word", "register value %g is not a 16-bit word", v);
                words[k] = (uint16_t)v;
            }
            slave_sim_write_registers(sl, holding, start, n, words);
        }
    } else if (!strcmp(cmd, "registers")) {
        /* The register table: name, group, first, words, one row each */
        const char *f[] = {"name", "group", "first", "words"};
        int nh = st_holding_regs_count, ni = st_input_regs_count;
        plhs[0] = mxCreateStructMatrix(nh + ni, 1, 4, f);
        for (int k = 0; k < nh + ni; k++) {
            const st_reg_t *r = k < nh ? &st_holding_regs[k] : &st_input_regs[k - nh];
            mxSetFieldByNumber(plhs[0], k, 0, mxCreateString(r->name + 2));   /* without "G." */
            mxSetFieldByNumber(plhs[0], k, 1, mxCreateString(k < nh ? "holding" : "input"));
            mxSetFieldByNumber(plhs[0], k, 2, mxCreateDoubleScalar(r->first));
            mxSetFieldByNumber(plhs[0], k, 3, mxCreateDoubleScalar(r->words));
        }
    } else if (!strcmp(cmd, "get") || !strcmp(cmd, "set")) {
        slave_sim_t *sl = instance(prhs[1]);
        char path[256];
        if (nrhs < 3 || mxGetString(prhs[2], path, sizeof path))
            mexErrMsgIdAndTxt("slaveSimMex:path", "path must be a string");
        int k = lower_bound(path);
        bool exact = k < index_count && !strcmp(index_rows[k]->name, path);
        if (!strcmp(cmd, "set")) {
            if (!exact)
                mexErrMsgIdAndTxt("slaveSimMex:path", "cannot set %s", path);
            const st_name_t *row = index_rows[k];
            if ((int)mxGetNumberOfElements(prhs[3]) != row->count)
                mexErrMsgIdAndTxt("slaveSimMex:size", "%s takes %d values", path, row->count);
            for (int e = 0; e < row->count; e++) {
                double v = mxIsLogical(prhs[3]) ? ((mxLogical *)mxGetData(prhs[3]))[e]
                                                : mxGetPr(prhs[3])[e];
                write_elem(address(sl, row), row->type, e, v);
            }
            return;
        }
        if (exact) {
            plhs[0] = leaf_value(sl, index_rows[k]);
            return;
        }
        char key[260];
        snprintf(key, sizeof key, "%s.", path);
        int lo = lower_bound(key), hi = lo;
        while (hi < index_count && !strncmp(index_rows[hi]->name, key, strlen(key)))
            hi++;
        if (lo == hi)
            mexErrMsgIdAndTxt("slaveSimMex:path", "the simulated slave has no %s", path);
        plhs[0] = group_value(sl, path, lo, hi);
    } else if (!strcmp(cmd, "enums")) {
        /* E.<Enum>.<Member>, from "Enum.Member" rows */
        mxArray *E = mxCreateStructMatrix(1, 1, 0, NULL);
        for (int k = 0; k < st_enums_count; k++) {
            char name[128];
            snprintf(name, sizeof name, "%s", st_enums[k].name);
            char *dot = strchr(name, '.');
            *dot = '\0';
            mxArray *sub = mxGetField(E, 0, name);
            if (!sub) {
                mxAddField(E, name);
                sub = mxCreateStructMatrix(1, 1, 0, NULL);
                mxSetField(E, 0, name, sub);
            }
            mxAddField(sub, dot + 1);
            mxSetField(sub, 0, dot + 1, mxCreateDoubleScalar(st_enums[k].value));
        }
        plhs[0] = E;
    } else if (!strcmp(cmd, "chamberPosition")) {
        plhs[0] = mxCreateDoubleScalar(slave_sim_chamber_position(instance(prhs[1]),
                                                                  mxGetScalar(prhs[2])));
    } else if (!strcmp(cmd, "values")) {
        slave_sim_t *sl = instance(prhs[1]);
        plhs[0] = mxCreateCellMatrix(index_count, 1);
        for (int k = 0; k < index_count; k++)
            mxSetCell(plhs[0], k, leaf_value(sl, index_rows[k]));
    } else if (!strcmp(cmd, "names")) {
        plhs[0] = mxCreateCellMatrix(index_count, 1);
        for (int k = 0; k < index_count; k++)
            mxSetCell(plhs[0], k, mxCreateString(index_rows[k]->name));
    } else {
        mexErrMsgIdAndTxt("slaveSimMex:cmd", "unknown command %s", cmd);
    }
    (void)nlhs;
}
