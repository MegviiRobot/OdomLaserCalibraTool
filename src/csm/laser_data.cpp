#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "csm/csm_all.h"

double* alloc_double_array(int n, double def);
int* alloc_int_array(int n, int def);

/* -------------------------------------------------- */

LDP ld_alloc_new(int nrays) {
	LDP ld = (LDP) malloc(sizeof(struct laser_data));
	ld_alloc(ld, nrays);
	return ld;
}

double* alloc_double_array(int n, double def) {
	double *v = (double*) malloc(sizeof(double)*n);
	int i=0; for(i=0;i<n;i++) {
		v[i] = def;
	}
	return v;
}

int* alloc_int_array(int n, int def) {
	int *v = (int*) malloc(sizeof(int)*n);
	int i=0; for(i=0;i<n;i++) {
		v[i] = def;
	}
	return v;
}

void ld_alloc(LDP ld, int nrays) {
	ld->nrays = nrays;
	
	ld->valid        = alloc_int_array(nrays, 0);
	ld->readings     = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	ld->readings_sigma = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	ld->theta        = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	
	ld->min_theta = std::numeric_limits<double>::quiet_NaN();
	ld->max_theta = std::numeric_limits<double>::quiet_NaN();
	
	ld->cluster      = alloc_int_array(nrays, -1);
	ld->alpha        = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	ld->cov_alpha    = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	ld->alpha_valid  = alloc_int_array(nrays, 0);

	ld->true_alpha   = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
	
	ld->up_bigger    = alloc_int_array(nrays, 0);
	ld->up_smaller   = alloc_int_array(nrays, 0);
	ld->down_bigger  = alloc_int_array(nrays, 0);
	ld->down_smaller = alloc_int_array(nrays, 0);

	ld->corr = (struct correspondence*) 
		malloc(sizeof(struct correspondence)*nrays);

	int i;
	for(i=0;i<ld->nrays;i++) {
		ld->corr[i].valid = 0;
		ld->corr[i].j1 = -1;
		ld->corr[i].j2 = -1;
	}
	
	for(i=0;i<3;i++) {
		ld->odometry[i] = 
		ld->estimate[i] = 
		ld->true_pose[i] = std::numeric_limits<double>::quiet_NaN();
	}
	
	ld->points = (point2d*) malloc(nrays * sizeof(point2d));
	ld->points_w = (point2d*) malloc(nrays * sizeof(point2d));
	
	for(i=0;i<nrays;i++) {
		ld->points[i].p[0] = 
		ld->points[i].p[1] = 
		ld->points[i].rho = 
		ld->points[i].phi = std::numeric_limits<double>::quiet_NaN();
		ld->points_w[i] = ld->points[i];
	}
	
	strcpy(ld->hostname, "CSM");
}

void ld_free(LDP ld) {
	ld_dealloc(ld);
	free(ld);
}

void ld_dealloc(struct laser_data*ld){	
	free(ld->valid);
	free(ld->readings);
	free(ld->readings_sigma);
	free(ld->theta);
	free(ld->cluster);
	free(ld->alpha);
	free(ld->alpha_valid);
	free(ld->true_alpha);
	free(ld->cov_alpha);
	free(ld->up_bigger);
	free(ld->up_smaller);
	free(ld->down_bigger);
	free(ld->down_smaller);
	free(ld->corr);
	
/*	int i;
	for(i=0;i<ld->nrays;i++)
		gsl_vector_free(ld->p[i]);
	free(ld->p);*/

	free(ld->points);
	free(ld->points_w);
}


void ld_compute_cartesian(LDP ld) {
	int i;
	for(i=0;i<ld->nrays;i++) {
/*		if(!ld_valid_ray(ld,i)) continue;*/
		double x = cos(ld->theta[i])*ld->readings[i];
		double y = sin(ld->theta[i])*ld->readings[i];
		
		ld->points[i].p[0] = x, 
		ld->points[i].p[1] = y;
		ld->points[i].rho = std::numeric_limits<double>::quiet_NaN();
		ld->points[i].phi = std::numeric_limits<double>::quiet_NaN();
	}
}


void ld_compute_world_coords(LDP ld, const double *pose) {
	double pose_x = pose[0];
	double pose_y = pose[1];
	double pose_theta = pose[2];
	double cos_theta = cos(pose_theta); 
	double sin_theta = sin(pose_theta);
	const int nrays = ld->nrays ;

	point2d * points = ld->points;
	point2d * points_w = ld->points_w;
	int i; for(i=0;i<nrays;i++) {
		if(!ld_valid_ray(ld,i)) continue;
		double x0 = points[i].p[0], 
		       y0 = points[i].p[1]; 
		
		if(is_nan(x0) || is_nan(y0)) {
			sm_error("ld_compute_world_coords(): I expected that cartesian coords were already computed: ray #%d: %f %f.\n", i, x0, y0);
		}
		
		points_w[i].p[0] = cos_theta * x0 -sin_theta*y0 + pose_x;
		points_w[i].p[1] = sin_theta * x0 +cos_theta*y0 + pose_y;
		/* polar coordinates */
	}
	
	for(i=0;i<nrays;i++) {
		double x = points_w[i].p[0];
		double y = points_w[i].p[1];
		points_w[i].rho = sqrt( x*x+y*y);
		points_w[i].phi = atan2(y, x);
	}
	
}



int ld_num_valid_correspondences(LDP ld) {
	int i; 
	int num = 0;
	for(i=0;i<ld->nrays;i++) {
		if(ld->corr[i].valid)
			num++;
	}
	return num;
}


int ld_valid_fields(LDP ld)  {
	if(!ld) {
		sm_error("NULL pointer given as laser_data*.\n");	
		return 0;
	}
	
	int min_nrays = 10;
	int max_nrays = 10000;
	if(ld->nrays < min_nrays || ld->nrays > max_nrays) {
		sm_error("Invalid number of rays: %d\n", ld->nrays);
		return 0;
	}
	if(is_nan(ld->min_theta) || is_nan(ld->max_theta)) {
		sm_error("Invalid min / max theta: min_theta = %f max_theta = %f\n",
			ld->min_theta, ld->max_theta);
		return 0;
	}
	double min_fov = deg2rad(20.0); 
	double max_fov = 2.01 * M_PI;
	double fov = ld->max_theta - ld->min_theta;
	if( fov < min_fov || fov > max_fov) {
		sm_error("Strange FOV: %f rad = %f deg \n", fov, rad2deg(fov));
		return 0;
	}
	if(fabs(ld->min_theta - ld->theta[0]) > 1e-8) {
		sm_error("Min_theta (%f) should be theta[0] (%f)\n",
			ld->min_theta, ld->theta[0]);
		return 0;
	}
	if(fabs(ld->max_theta - ld->theta[ld->nrays-1]) > 1e-8) {
		sm_error("Min_theta (%f) should be theta[0] (%f)\n",
			ld->max_theta, ld->theta[ld->nrays-1]);
		return 0;
	}
	/* Check that there are valid rays */
	double min_reading = 0;
	double max_reading = 100;
	int i; for(i=0;i<ld->nrays;i++) {
		double th = ld->theta[i];
		if(ld->valid[i]) {
			double r = ld->readings[i];
			if(is_nan(r) || is_nan(th)) {
				sm_error("Ray #%d: r = %f  theta = %f but valid is %d\n",
					i, r, th, ld->valid[i]);
				return 0;
			}
			if( !( min_reading < r && r < max_reading ) ) {
				sm_error("Ray #%d: %f is not in interval (%f, %f) \n",
					i, r, min_reading, max_reading);
				return 0;
			}		
		} else {
			/* ray not valid, but checking theta anyway */
			if(is_nan(th)) {
				sm_error("Ray #%d: valid = %d  but theta = %f\n",
					i,  ld->valid[i], th);
				return 0;
			}

			if(ld->cluster[i] != -1 ) {
				sm_error("Invalid ray #%d has cluster %d\n.", i, ld->cluster[i]);
				return 0;
			}
		}
		if(ld->cluster[i] < -1 ) {
			sm_error("Ray #%d: Invalid cluster value %d\n.", i, ld->cluster[i]);
			return 0;
		}
		
		if(!is_nan(ld->readings_sigma[i]) && ld->readings_sigma[i] < 0) {
			sm_error("Ray #%d: has invalid readings_sigma %f \n", i, ld->readings_sigma[i]);
			return 0;
		}
		
	}
	/* Checks that there is at least 10% valid rays */
	int num_valid   = count_equal(ld->valid, ld->nrays, 1);
	int num_invalid = count_equal(ld->valid, ld->nrays, 0);
	if (num_valid < ld->nrays * 0.10) {
		sm_error("Valid: %d/%d invalid: %d.\n", num_valid, ld->nrays, num_invalid);
		return 0;
	}

	return 1;
}


/** Computes an hash of the correspondences */
unsigned int ld_corr_hash(LDP ld){
	unsigned int hash = 0;
	unsigned int i    = 0;

	for(i = 0; i < (unsigned)ld->nrays; i++) {
		int str = ld_valid_corr(ld, (int)i) ? (ld->corr[i].j1 + 1000*ld->corr[i].j2) : -1;
		hash ^= ((i & 1) == 0) ? (  (hash <<  7) ^ (str) ^ (hash >> 3)) :
		                         (~((hash << 11) ^ (str) ^ (hash >> 5)));
	}

	return (hash & 0x7FFFFFFF);
}



