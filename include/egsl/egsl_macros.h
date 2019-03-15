#ifndef H_EGSL_MACROS
#define H_EGSL_MACROS

#include "egsl.h"

#define atv(v,i)         egsl_atv(v,i)
#define atm(v,i,j)       egsl_atm(v,i,j)
#define sub(v1,v2)       egsl_sub(v1,v2)
#define minus(v)         egsl_scale(-1.0,v)
#define sum(v1,v2)       egsl_sum(v1,v2)
#define sum3(v1,v2,v3)   egsl_sum(v1,egsl_sum(v2,v3))
#define tr(v)            egsl_transpose(v)
#define m(v1,v2)         egsl_mult(v1,v2)
#define m3(v1,v2,v3)     egsl_mult(v1,egsl_mult(v2,v3))
#define m4(v1,v2,v3,v4)  egsl_mult(v1,egsl_mult(v2,egsl_mult(v3,v4)))
#define comp_col(v1,v2)  egsl_compose_col(v1,v2)
#define comp_row(v1,v2)  egsl_compose_row(v1,v2)

#define zeros(rows,cols) egsl_zeros(rows,cols)
#define ones(rows,cols)  egsl_ones(rows,cols)
#define vers(th)         egsl_vers(th)
#define rot(theta)       egsl_rot(theta)

#define sc(d,v)         egsl_scale(d, v)
#define add_to(v1,v2)   egsl_add_to(v1, v2)
#define add_to_col(v1,j,v2)   egsl_add_to_col(v1, j, v2)
#define inv(v)   egsl_inverse(v)


#endif
