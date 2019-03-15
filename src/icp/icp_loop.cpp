#include <math.h>
#include <string.h>

//#include <gsl/gsl_matrix.h>
#include <gsl_eigen/gsl_eigen.h>
#include <gpc/gpc.h>
#include <egsl/egsl_macros.h>

#include "csm/csm_all.h"

#include "icp/icp.h"
#include <vector>

int icp_loop(struct sm_params*params, const double*q0, double*x_new, 
	double*total_error, int*valid, int*iterations) {
	if(any_nan(q0,3)) {
		sm_error("icp_loop: Initial pose contains nan: %s\n", friendly_pose(q0));
		return 0;
	}

	LDP laser_sens = params->laser_sens;
	double x_old[3], delta[3], delta_old[3] = {0,0,0};
	copy_d(q0, 3, x_old);
	//unsigned int hashes[params->max_iterations];
	std::vector<unsigned int> hashes(params->max_iterations, 0);
	int iteration;
	
	sm_debug("icp: starting at  q0 =  %s  \n", friendly_pose(x_old));
	
	int all_is_okay = 1;
	
	for(iteration=0; iteration<params->max_iterations;iteration++) {
		egsl_push_named("icp_loop iteration");
		sm_debug("== icp_loop: starting iteration. %d  \n", iteration);

		/** Compute laser_sens's points in laser_ref's coordinates
		    by roto-translating by x_old */
		ld_compute_world_coords(laser_sens, x_old);

		/** Find correspondences (the naif or smart way) */
		if(params->use_corr_tricks)
			find_correspondences_tricks(params);
		else
			find_correspondences(params);

		/** If debug_verify_tricks, make sure that find_correspondences_tricks()
		    and find_correspondences() return the same answer */
			if(params->debug_verify_tricks)
				debug_correspondences(params);

		/* If not many correspondences, bail out */
		int num_corr = ld_num_valid_correspondences(laser_sens);
		double fail_perc = 0.05;
		if(num_corr < fail_perc * laser_sens->nrays) { /* TODO: arbitrary */
			sm_error("	: before trimming, only %d correspondences.\n",num_corr);
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		/* Kill some correspondences (using dubious algorithm) */
		if(params->outliers_remove_doubles)
			kill_outliers_double(params);
		
		int num_corr2 = ld_num_valid_correspondences(laser_sens);

		double error=0;
		/* Trim correspondences */
		kill_outliers_trim(params, &error);
		int num_corr_after = ld_num_valid_correspondences(laser_sens);
		
		*total_error = error; 
		*valid = num_corr_after;

		sm_debug("  icp_loop: total error: %f  valid %d   mean = %f\n", *total_error, *valid, *total_error/ *valid);
		
		/* If not many correspondences, bail out */
		if(num_corr_after < fail_perc * laser_sens->nrays){
			sm_error("  icp_loop: failed: after trimming, only %d correspondences.\n",num_corr_after);
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration"); /* loop context */
			break;
		}

		/* Compute next estimate based on the correspondences */
		if(!compute_next_estimate(params, x_old, x_new)) {
			sm_error("  icp_loop: Cannot compute next estimate.\n");
			all_is_okay = 0;
			egsl_pop_named("icp_loop iteration");
			break;			
		}

		pose_diff_d(x_new, x_old, delta);
		
		{
			sm_debug("  icp_loop: killing. laser_sens has %d/%d rays valid,  %d corr found -> %d after double cut -> %d after adaptive cut \n", count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays, num_corr, num_corr2, num_corr_after);
		}
		/** Checks for oscillations */
		hashes[iteration] = ld_corr_hash(laser_sens);
		
		{
			sm_debug("  icp_loop: it. %d  hash=%d nvalid=%d mean error = %f, x_new= %s\n", 
				iteration, hashes[iteration], *valid, *total_error/ *valid, 
				friendly_pose(x_new));
		}

		
		/** PLICP terminates in a finite number of steps! */
		if(params->use_point_to_line_distance) {
			int loop_detected = 0; /* TODO: make function */
			int a; for(a=iteration-1;a>=0;a--) {
				if(hashes[a]==hashes[iteration]) {
					sm_debug("icpc: oscillation detected (cycle length = %d)\n", iteration-a);
					loop_detected = 1;
					break;
				}
			}
			if(loop_detected) {
				egsl_pop_named("icp_loop iteration");
				break;
			} 
		}
	
		/* This termination criterium is useless when using
		   the point-to-line-distance; however, we put it here because
		   one can choose to use the point-to-point distance. */
		if(termination_criterion(params, delta)) {
			egsl_pop_named("icp_loop iteration");
			break;
		}
		
		copy_d(x_new, 3, x_old);
		copy_d(delta, 3, delta_old);
		
		
		egsl_pop_named("icp_loop iteration");
	}

	*iterations = iteration+1;
	
	return all_is_okay;
}

int termination_criterion(struct sm_params*params, const double*delta){
	double a = norm_d(delta);
	double b = fabs(delta[2]);
	return (a<params->epsilon_xy) && (b<params->epsilon_theta);
}

// 根据论文 Appendix I 计算两帧之间的运动量
int compute_next_estimate(struct sm_params*params, 
	const double x_old[3], double x_new[3]) 
{
	LDP laser_ref  = params->laser_ref;
	LDP laser_sens = params->laser_sens;
	
	//struct gpc_corr c[laser_sens->nrays];
	struct gpc_corr dummy;
	std::vector<gpc_corr> c(laser_sens->nrays, dummy);

	int i; int k=0;
	// 遍历每个激光点, 计算协方差 c 权重， 论文公式 16. 公式 16 定义了 plicp 的协方差为法向量，ppicp 的协方差为单位矩阵
	for(i=0;i<laser_sens->nrays;i++) {
		if(!laser_sens->valid[i])
			continue;
			
		if(!ld_valid_corr(laser_sens,i))
			continue;

		// 参考帧上找到的两个匹配点
		int j1 = laser_sens->corr[i].j1;
		int j2 = laser_sens->corr[i].j2;

		c[k].valid = 1;
		
		if(laser_sens->corr[i].type == correspondence::corr_pl) {

			c[k].p[0] = laser_sens->points[i].p[0];
			c[k].p[1] = laser_sens->points[i].p[1];
			c[k].q[0] = laser_ref->points[j1].p[0];
			c[k].q[1] = laser_ref->points[j1].p[1];

			/** TODO: here we could use the estimated alpha */
			// 根据参考帧上最近的两个点计算法向量 n
			double diff[2];
			diff[0] = laser_ref->points[j1].p[0]-laser_ref->points[j2].p[0];
			diff[1] = laser_ref->points[j1].p[1]-laser_ref->points[j2].p[1];
			double one_on_norm = 1 / sqrt(diff[0]*diff[0]+diff[1]*diff[1]);
			double normal[2];
			normal[0] = +diff[1] * one_on_norm;
			normal[1] = -diff[0] * one_on_norm;

			double cos_alpha = normal[0];
			double sin_alpha = normal[1];

			// 论文公式 16 中， C = n * n.transpose();
			c[k].C[0][0] = cos_alpha*cos_alpha;
			c[k].C[1][0] = 
			c[k].C[0][1] = cos_alpha*sin_alpha;
			c[k].C[1][1] = sin_alpha*sin_alpha;
			
/*			sm_debug("k=%d, i=%d sens_phi: %fdeg, j1=%d j2=%d,  alpha_seg=%f, cos=%f sin=%f \n", k,i,
				rad2deg(laser_sens->theta[i]), j1,j2, atan2(sin_alpha,cos_alpha), cos_alpha,sin_alpha);*/
			
#if 0
			/* Note: it seems that because of numerical errors this matrix might be
			   not semidef positive. */
			double det = c[k].C[0][0] * c[k].C[1][1] - c[k].C[0][1] * c[k].C[1][0];
			double trace = c[k].C[0][0] + c[k].C[1][1];
			
			int semidef = (det >= 0) && (trace>0);
			if(!semidef) {
	/*			printf("%d: Adjusting correspondence weights\n",i);*/
				double eps = -det;
				c[k].C[0][0] += 2*sqrt(eps);
				c[k].C[1][1] += 2*sqrt(eps);
			}
#endif			
		} else {
			c[k].p[0] = laser_sens->points[i].p[0];
			c[k].p[1] = laser_sens->points[i].p[1];
			
			projection_on_segment_d(
				laser_ref->points[j1].p,
				laser_ref->points[j2].p,
				laser_sens->points_w[i].p,
				c[k].q);

			/* Identity matrix */
			c[k].C[0][0] = 1;
			c[k].C[1][0] = 0;
			c[k].C[0][1] = 0;
			c[k].C[1][1] = 1;
		}
		
		
		double factor = 1;

		// 匹配的点云对应的角度 和 scan point 的角度之间的差越小，权重越大，表示越匹配。
		/* Scale the correspondence weight by a factor concerning the 
		   information in this reading. */
		if(params->use_ml_weights) {
			int have_alpha = 0;
			double alpha = 0;
			if(!is_nan(laser_ref->true_alpha[j1])) {
				alpha = laser_ref->true_alpha[j1];
				have_alpha = 1;
			} else if(laser_ref->alpha_valid[j1]) {
				alpha = laser_ref->alpha[j1];;
				have_alpha = 1;
			} else have_alpha = 0;
			
			if(have_alpha) {
				double pose_theta = x_old[2];
				/** Incidence of the ray 
					Note that alpha is relative to the first scan (not the world)
					and that pose_theta is the angle of the second scan with 
					respect to the first, hence it's ok. */
				double beta = alpha - (pose_theta + laser_sens->theta[i]);
				factor = 1 / square(cos(beta));
			} else {
				static int warned_before = 0;
				if(!warned_before) {
					sm_error("Param use_ml_weights was active, but not valid alpha[] or true_alpha[]." 
					          "Perhaps, if this is a single ray not having alpha, you should mark it as inactive.\n");						
					sm_error("Writing laser_ref: \n");						
					/*ld_write_as_json(laser_ref, stderr);*/
					warned_before = 1;
				}
			}
		} 
		
		/* Weight the points by the sigma in laser_sens */
		// sigma 表示激光数据的噪声
		if(params->use_sigma_weights) {
			if(!is_nan(laser_sens->readings_sigma[i])) {
				factor *= 1 / square(laser_sens->readings_sigma[i]);
			} else {
				static int warned_before = 0;
				if(!warned_before) {
					sm_error("Param use_sigma_weights was active, but the field readings_sigma[] was not filled in.\n");						
					sm_error("Writing laser_sens: \n");						
					/*ld_write_as_json(laser_sens, stderr);*/
				}
			}
		}
		
		c[k].C[0][0] *= factor;
		c[k].C[1][0] *= factor;
		c[k].C[0][1] *= factor;
		c[k].C[1][1] *= factor;
		
		k++;
	}
	
	/* TODO: use prior for odometry */
	double std = 0.11;
	const double inv_cov_x0[9] = 
		{1/(std*std), 0, 0,
		 0, 1/(std*std), 0,
		 0, 0, 0};
	
	
	int ok = gpc_solve(k, c, 0, inv_cov_x0, x_new);
	if(!ok) {
		sm_error("gpc_solve_valid failed\n");
		return 0;
	}

	double old_error = gpc_total_error(c, k, x_old);
	double new_error = gpc_total_error(c, k, x_new);

	sm_debug("\tcompute_next_estimate: old error: %f  x_old= %s \n", old_error, friendly_pose(x_old));
	sm_debug("\tcompute_next_estimate: new error: %f  x_new= %s \n", new_error, friendly_pose(x_new));
	sm_debug("\tcompute_next_estimate: new error - old_error: %g \n", new_error-old_error);

	double epsilon = 0.000001;
	if(new_error > old_error + epsilon) {
		sm_error("\tcompute_next_estimate: something's fishy here! Old error: %lf  new error: %lf  x_old %lf %lf %lf x_new %lf %lf %lf\n",old_error,new_error,x_old[0],x_old[1],x_old[2],x_new[0],x_new[1],x_new[2]);
	}
	
	return 1;
}
	


