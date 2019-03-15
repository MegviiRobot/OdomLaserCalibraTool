#include <stdio.h> 
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>

#include <gsl/gsl_statistics_double.h>
#include <options/options.h>
#include <csm/csm_all.h>

#include "aux_functions.h"
#include "solver_options.h"
#include "solver_utils.h"

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>


#include "calib_tuple.h"

#define vg(X,Y) 	( gsl_vector_get(X,Y) )
#define vs(X,Y,Z) 	( gsl_vector_set(X,Y,Z) )
#define mg(X,Y,Z) 	( gsl_matrix_get(X,Y,Z) )
#define ms(X,Y,Z,Q) 	( gsl_matrix_set(X,Y,Z,Q) )

using namespace std;



double f (const gsl_vector *z, void *params);


int solve(const vector <calib_tuple>& tuple, int mode, double max_cond_number, struct calib_result& res) {

/*!<!--####################		FIRST STEP: estimate J21 and J22  	#################-->*/
	double J21, J22;
	gsl_matrix * A 	= gsl_matrix_calloc(2,2);
 	gsl_vector * g 	= gsl_vector_calloc(2);
 	gsl_vector * L_i 	= gsl_vector_alloc(2);
 	double th_i;
	int n = (int) tuple.size();	

	for (int i = 0; i < n; i++)	{
		const calib_tuple &t = tuple[i];
		vs( L_i, 0,  t.T*t.phi_l); 
		vs( L_i, 1,  t.T*t.phi_r);
		gsl_blas_dger  (1, L_i, L_i, A); // A = A + L_i'*L_i;  A symmetric 
		gsl_blas_daxpy (t.sm[2], L_i, g);  // g = g + L_i'*y_i;  sm :{x , y, theta}
	}
	gsl_vector_free(L_i);
	
	// Verify that A isn't singular
	double cond = cond_number(A);
 	if ( cond > max_cond_number ) {
 		sm_error("Matrix A is almost singular (condition number: %f). Not very exciting data!\n", cond);
		return 0;
 	}
	
	// Ay = g --> y = inv(A)g; A square matrix;
	gsl_vector * y = gsl_vector_calloc(2);
	if (!solve_square_linear(A, g, y)) {
		sm_error("Cannot solve Ay=g. Invalid arguments\n");
		return 0;
	}
	J21 = vg(y, 0); J22 = vg(y, 1);
 	if ( gsl_isnan(J21) || gsl_isnan(J22) ) {
 		sm_error("Could not find first two parameters J21, J22 by solving linear equation.\n");
        sm_error("This is A: \n\n"); gsl_matrix_fwrite(stderr, A); fprintf(stderr, "\n");
        sm_error("This is g: \n\t"); gsl_vector_fwrite(stderr, g); fprintf(stderr, "\n");
        sm_error("This is y: \n\t"); gsl_vector_fwrite(stderr, y); fprintf(stderr, "\n");
		return 0;
 	}
//	cerr << "J21,J22: " << J21 << ", " << J22 << endl;
	gsl_vector_free(y);	

/*!<!--############## 		SECOND STEP: estimate the remaining parameters  		########-->*/	
	// Build M, M2
	gsl_matrix * M    	 = gsl_matrix_calloc(5,5);	
	gsl_matrix * M2   	 = gsl_matrix_calloc(6,6);
	gsl_matrix * L_k  	 = gsl_matrix_calloc(2,5);
	gsl_matrix * L_2k 	 = gsl_matrix_calloc(2,6);	
	double c, cx, cy, cx1, cx2, cy1, cy2, t1, t2;	
	double o_theta, T, phi_l, phi_r, w0;
	double sm[3];

    int nused = 0;
	for (int k = 0; k < n; k++) {
		const calib_tuple &t = tuple[k];
				
		o_theta  = t.T * (J21*t.phi_l + J22*t.phi_r);
		w0 = o_theta / t.T;
		
		/*if( fabs(o_theta) > deg2rad(1) ) {
            continue; 
		} else {
            nused ++;
		}*/
		
		if ( fabs(o_theta) > 1e-12 ) {
			t1 = (   sin(o_theta)   / (o_theta) );
			t2 = ( (1-cos(o_theta)) / (o_theta) );	
		}
		else {
			t1 = 1;
			t2 = 0;
		}		
		cx1 = 0.5 * t.T * (-J21 * t.phi_l) * t1;
		cx2 = 0.5 * t.T * (+J22 * t.phi_r) * t1;
		cy1 = 0.5 * t.T * (-J21 * t.phi_l) * t2;
		cy2 = 0.5 * t.T * (+J22 * t.phi_r) * t2;
		if ( (mode == 0)||(mode == 1) ) {
			cx = cx1 + cx2;
			cy = cy1 + cy2; 			
			double array[] = { 
				-cx, 1-cos(o_theta),   sin(o_theta), t.sm[0], -t.sm[1],
				-cy,  -sin(o_theta), 1-cos(o_theta), t.sm[1],  t.sm[0]
			};
			gsl_matrix_view tmp = gsl_matrix_view_array(array, 2, 5);
			L_k = &tmp.matrix;		
			// M = M + L_k' * L_k; M is symmetric
			gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_k, L_k, 1, M);
		}
		else {
			double array2[] = { 
				-cx1, -cx2, 1-cos(o_theta), 	sin(o_theta), t.sm[0], -t.sm[1],
				-cy1, -cy2,  -sin(o_theta), 1-cos(o_theta), t.sm[1],  t.sm[0]
			};
			gsl_matrix_view tmp = gsl_matrix_view_array(array2, 2, 6);
			L_2k = &tmp.matrix;		
			// M2 = M2 + L_2k' * L_2k; M2 is symmetric
			gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_2k, L_2k, 1, M2); 
		}
	}
	
//    sm_info("Andrea's hack: using %d / %d\n", nused, n);
	
	double est_b, est_d_l, est_d_r, laser_x, laser_y, laser_th;
	gsl_vector * x;
	switch (mode)
	{
	case 0:	/*!<!--######### mode 0: minimize in closed form, using M ########-->*/
		{
			x = full_calibration_min(M);
			
			est_b = vg(x, 0);
			est_d_l = 2*(-est_b * J21);
			est_d_r  = 2*(est_b * J22);
			laser_x = vg(x, 1);
			laser_y = vg(x, 2);
			laser_th = atan2(vg(x, 4), vg(x, 3));
			break;
		}
	case 1:	/*!<!--####  mode 1: start at the minimum eigenvalue, find numerical solution using M  ####-->*/
		{
			x = numeric_calibration(M);
				
			est_b = vg(x, 0);
			est_d_l = 2*(-est_b * J21);
			est_d_r  = 2*(est_b * J22);
			laser_x = vg(x, 1);
			laser_y = vg(x, 2);
			laser_th = atan2(vg(x, 4), vg(x, 3));
			break;
		}	
	case 2:	/*!<!--####  mode 2: start at the minimum eigenvalue, find numerical solution using M2 ####-->*/
		{
			x = numeric_calibration(M2);

			est_b = (vg(x, 0) + vg(x, 1))/2;
			est_d_l = 2*(-est_b * J21);
			est_d_r  = 2*(est_b * J22);
			laser_x = vg(x, 2);
			laser_y = vg(x, 3);
			laser_th = atan2(vg(x, 5), vg(x, 4));
			break;
		}
	default:
		break;
	}

/*!<!--###################		  TEST  		####################-->*/
// 	gsl_matrix * M3   	 = gsl_matrix_calloc(6,6);
// 	gsl_matrix * L_3k 	 = gsl_matrix_calloc(2,6);	
// 	double dx1, dx2, dy1, dy2;	
// 	double J11, J12;
// 	
// 	for (int k = 0; k < n; k++) {
// 		jo_read_double(tuple[k], "T", &T);
// 		jo_read_double(tuple[k], "phi_l", &phi_l);
// 		jo_read_double(tuple[k], "phi_r", &phi_r);
// 		jo_read_double_array (tuple[k], "sm", sm, 3, 0.0);
// 				
// 		o_theta  = sm[2];
// 		w0 = o_theta / T;
// 				
// 		if ( fabs(o_theta) > 1e-12 ) {
// 			t1 = (   sin(o_theta)   / (o_theta) );
// 			t2 = ( (1-cos(o_theta)) / (o_theta) );	
// 		}
// 		else {
// 			t1 = 1;
// 			t2 = 0;
// 		}		
// 		dx1 = T * ( phi_l) * t1;
// 		dx2 = T * ( phi_r) * t1;
// 		dy1 = T * ( phi_l) * t2;
// 		dy2 = T * ( phi_r) * t2;
// 		double array3[] = { 
// 			-dx1, -dx2, 1-cos(o_theta), 	sin(o_theta), sm[0], -sm[1],
// 			-dy1, -dy2,  -sin(o_theta), 1-cos(o_theta), sm[1],  sm[0]
// 		};
// 		gsl_matrix_view tmp = gsl_matrix_view_array(array3, 2, 6);
// 		L_3k = &tmp.matrix;		
// 		gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_3k, L_3k, 1, M3); 
// 		
// 	}
// 	gsl_vector * x3;	
// 	x3 = numeric_calibration(M3);
// 	J11 = vg(x3,0);
// 	J12 = vg(x3,1);
// 	cout << "J11,J12: " << J11 << ", " << J12 << endl;
// 	double axle2 = 2*((J11+J12)/(J22-J21));
// 	cout << " new d_l =  " << J11*4 << endl;
// 	cout << " new d_r =  " << J12*4 << endl;
// 	cout << "TEST\n";
// 	cout << " axle  "	<< axle2 << endl;
// 	cout << " l_diam  "  	<< 2*J11 - J21*axle2  << endl;
// 	cout << " r_diam  "  	<< 2*J12 + J22*axle2   << endl;
// 	cout << " l_x  "  	<< vg(x3,2) << endl;
// 	cout << " l_y  "  	<< vg(x3,3) << endl;
// 	cout << " l_th  "  	<< atan2(vg(x3, 5), vg(x3, 4)) << endl;

/*!<!--###################		  Output  		####################-->*/

	res.axle = est_b;
	res.radius_l = est_d_l/2;
	res.radius_r = est_d_r/2;
	res.l[0] = laser_x;
	res.l[1] = laser_y;
	res.l[2] = laser_th;
	
	return 1;

}


int solve_jacompute_disagreementckknife(const vector <calib_tuple>& tuples, int mode, double max_cond_number, struct calib_result& res) {
    
    union {
        double tmp_params[6];
        struct calib_result tmp_res;
    };
    
    // check that the union trick is working
    tmp_res.radius_l = 0;
    tmp_res.radius_r = 1;
    tmp_res.axle = 2;
    tmp_res.l[0] = 3;
    tmp_res.l[1] = 4;
    tmp_res.l[2] = 5;
    
    for(int i=0;i<6;i++)
        if(tmp_params[i] != i) {
            sm_error("Ooops! the union trick is not working");
            exit(-1);
        }
    
    int n = tuples.size();
    vector<calib_tuple> minusone;
    for(int i=1;i<n;i++)
        minusone.push_back(tuples[i]);
    
    
    double avg_minusone[6] = {0,0,0,0,0,0};
    
    for(int i=0;i<n;i++) {
        // let's put back the i-th and remove the (i-1)-nth
        minusone[i % (n-1) ] = tuples[i];
        
        if(!solve(minusone, mode, max_cond_number, tmp_res))
            return 0;
        
        for(int a=0;a<6;a++)
            avg_minusone[a] += tmp_params[a] / n;
            
    }

    // solve once again with original tuples
    if(!solve(tuples, mode, max_cond_number, tmp_res))
        return 0;

    double jack[6];
    for(int a=0;a<6;a++)
        jack[a] = n* tmp_params[a] - (n-1) * avg_minusone[a];
    
    
    const char*format = "%.10f %.10f %.10f %.10f %.10f %.10f\n";
    fprintf(stderr, "avg\t");
    fprintf(stderr, format, 
        avg_minusone[0],  avg_minusone[1],             avg_minusone[2],
        avg_minusone[3],  avg_minusone[4],             avg_minusone[5]);

    fprintf(stderr, "ori\t");
    fprintf(stderr, format, 
        tmp_params[0],  tmp_params[1],             tmp_params[2],
        tmp_params[3],  tmp_params[4],             tmp_params[5]);

    fprintf(stderr, "jack"); 

    fprintf(stderr, format, 
        jack[0],  jack[1],             jack[2],
        jack[3],  jack[4],             jack[5]);
    
    
}



void estimate_noise(std::vector<calib_tuple>&tuples, const calib_result&res, double&std_x,double&std_y,double&std_th) {
    int n = tuples.size();
    double err_sm[3][n];
    for(int i=0;i<tuples.size();i++) {
        tuples[i].compute_disagreement(res);
        err_sm[0][i] = tuples[i].err_sm[0];
        err_sm[1][i] = tuples[i].err_sm[1];
        err_sm[2][i] = tuples[i].err_sm[2];
    }

     std_x  = gsl_stats_sd(err_sm[0],1,n);
     std_y  = gsl_stats_sd(err_sm[1],1,n);
     std_th = gsl_stats_sd(err_sm[2],1,n);
    sm_info("Estimated sm errors (std dev):  %g mm, %g mm, %g deg\n", std_x * 1000, std_y * 1000, rad2deg(std_th));
}
