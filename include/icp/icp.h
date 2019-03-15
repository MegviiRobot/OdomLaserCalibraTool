#ifndef _H_ICP_
#define _H_ICP_

//#include <gpc/gpc.h>

#include <csm/csm_all.h>

/** This sets the stage.  */
void sm_icp(struct sm_params*params, struct sm_result*res);

/** This is the meat */
int icp_loop(struct sm_params*params, const double*q0, double*x_new, 
 	double*total_error, int*nvalid, int*iterations);

/** This is the beef: computing in closed form the next estimate 
    given the correspondences. */
int compute_next_estimate(struct sm_params*params,
	const double x_old[3], double x_new[3]);

/** This termination criterium use epsilon_xy and epsilon_th.
   It is useless when using
   the point-to-line-distance; however, we put it here because
   one can choose to use the point-to-point distance. */
int termination_criterion(struct sm_params*params, const double*delta);

/** Naif algorithm */
void find_correspondences(struct sm_params*params);
/** Smart algorithm */
void find_correspondences_tricks(struct sm_params*params);
	/** Checks that find_correspondences_tricks and find_correspondences behave the same.
	 	Exit(-1) on error. */
	void debug_correspondences(struct sm_params * params);

void kill_outliers_trim(struct sm_params*params, double*total_error);
void kill_outliers_double(struct sm_params*params);
	
void compute_covariance_exact(
	LDP laser_ref, LDP laser_sens, const gsl_vector*x,
		val *cov0_x, val *dx_dy1, val *dx_dy2);

void visibilityTest(LDP ld, const gsl_vector*x_old);

/** Marks a ray invalid if reading is outside range [min_reading, max_reading]. */
void ld_invalid_if_outside(LDP ld, double min_reading, double max_reading);

void swap_double(double*a,double*b);

#endif
