#ifndef H_LASER_DATA_INLINE
#define H_LASER_DATA_INLINE

//#include <gsl/gsl_math.h>
//#include <gsl/gsl_vector.h>
#include <limits>

//#include "csm.h"

/* Simple inline functions */

/*#warning Seen ld_valid_ray*/

INLINE int ld_valid_ray(LDP ld, int i) {
	return (i>=0) && (i<ld->nrays) && (ld->valid[i]);
}

INLINE int ld_valid_alpha(LDP ld, int i) {
	return ld->alpha_valid[i] != 0;
}

INLINE void ld_set_null_correspondence(LDP ld, int i) {
	ld->corr[i].valid = 0;
	ld->corr[i].j1 = -1;	
	ld->corr[i].j2 = -1;	
	ld->corr[i].dist2_j1 = std::numeric_limits<double>::quiet_NaN(); //GSL_NAN;
}

INLINE void ld_set_correspondence(LDP ld, int i, int j1, int j2) {
	ld->corr[i].valid = 1;
	ld->corr[i].j1 = j1;	
	ld->corr[i].j2 = j2;	
}

/** -1 if not found */

INLINE int ld_next_valid(LDP ld, int i, int dir) {
	int j;
	for(j=i+dir;(j<ld->nrays)&&(j>=0)&&!ld_valid_ray(ld,j);j+=dir);
	return ld_valid_ray(ld,j) ? j : -1;
}

INLINE int ld_next_valid_up(LDP ld, int i){
	return ld_next_valid(ld, i, +1);
}

INLINE int ld_next_valid_down(LDP ld, int i){
	return ld_next_valid(ld, i, -1);
}


INLINE int ld_valid_corr(LDP ld, int i) {
	return ld->corr[i].valid;
}

#endif

