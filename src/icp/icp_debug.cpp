#include <string.h>
#include "icp/icp.h"
#include <vector>

void debug_correspondences(struct sm_params * params) {
	LDP laser_sens = params->laser_sens;
	/** Do the test */
	find_correspondences_tricks(params);
	//struct correspondence c1[laser_sens->nrays];
	std::vector<correspondence> c1;
	c1.reserve(laser_sens->nrays);

	struct correspondence * c2 = laser_sens->corr;
	//memcpy(c1, c2, sizeof(struct correspondence) * laser_sens->nrays);
	c1.assign(c2, c2+laser_sens->nrays);
	long hash1 = ld_corr_hash(laser_sens);
	find_correspondences(params);
	long hash2 = ld_corr_hash(laser_sens);
	if(hash1 != hash2) {
		sm_error("find_correspondences_tricks might be buggy\n");
		int i = 0; for(i=0;i<laser_sens->nrays;i++) {
			if( (c1[i].valid != c2[i].valid) ||
				(c1[i].j1 != c2[i].j1) || (c1[i].j2 != c2[i].j2) ) {
					sm_error("\t   tricks: c1[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
						i, c1[i].valid, c1[i].j1, c1[i].j2, c1[i].dist2_j1);
					sm_error("\tno tricks: c2[%d].valid = %d j1 = %d  j2 = %d  dist2_j1 = %f\n",
						i, c2[i].valid, c2[i].j1, c2[i].j2, c2[i].dist2_j1);
				}
		}
		if(1) exit(-1);
	}
}
