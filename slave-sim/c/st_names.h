/* Name table of the generated slave (tools/st_target_c.py writes st_names.c):
 * every variable of G, IO and the PRG_Main instance by path, so the
 * simulator's gateway can read and write them by name. */
#ifndef ST_NAMES_H
#define ST_NAMES_H

#include <stddef.h>

enum { ST_ROOT_G, ST_ROOT_IO, ST_ROOT_S };
enum { ST_BOOL, ST_I8, ST_U8, ST_I16, ST_U16, ST_I32, ST_U32, ST_F32, ST_F64 };

typedef struct {
    const char *name;   /* "G.G_cmdPower.Enable", "s.fbHomeLimit.eState", ... */
    int root;           /* ST_ROOT_* */
    int type;           /* ST_BOOL ... ST_F64 */
    size_t offset;      /* from the start of the root struct */
    int count;          /* elements; 1 for scalars */
} st_name_t;

typedef struct {
    const char *name;   /* "E_SystemState.ST_IDLE" */
    int value;
} st_enum_t;

extern const st_name_t st_names[];
extern const int st_names_count;
extern const st_enum_t st_enums[];
extern const int st_enums_count;

#endif
