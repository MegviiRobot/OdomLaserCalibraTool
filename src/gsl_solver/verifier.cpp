/*! 
	\file   verifier.cpp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief  Implements methods to evaluate estimated parameters
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#include <json-c/json.h>
#include <options/options.h>
#include <csm/csm_all.h>

#include "verifier_utils.h"
#include "verifier_options.h"
#include "aux_functions.h"

#include "analyzer.h"

using namespace std;
using namespace CSM;

int main(int argc, const char * argv[]) {

	/*!<!-- Verify options -->*/
	verifier_params params;	
	struct option* ops = options_allocate(100);
	verifier_options(&params, ops);
	if(!options_parse_args(ops, argc, argv)) {
		options_print_help(ops, stderr);
		return -2;
	}		
	
	JO est_params;		
	FILE * input = open_file_for_reading(params.calib_params); 	
	if (!input) return 1;
 	est_params = json_read_stream(input); 
 	fclose(input);
 	double l_diam, r_diam, axle;
	double laser[3];
	jo_read_double(est_params, "l_diam", &l_diam);
	jo_read_double(est_params, "r_diam", &r_diam);
	jo_read_double(est_params, "axle", &axle);
	jo_read_double(est_params, "l_x", &laser[0]);
	jo_read_double(est_params, "l_y", &laser[1]);
	jo_read_double(est_params, "l_theta", &laser[2]);
	
	printf("\nparametri stimati: b = %f,  l_diam = %f,  r_diam = %f, l_x = %f, l_y = %f, l_theta = %f\n", axle, l_diam, r_diam, laser[0], laser[1], laser[2]);
	
	double J11 = l_diam/4;
	double J12 = r_diam/4;
	double J21 = -l_diam/(2*axle);
	double J22 = r_diam/(2*axle);
	
	vector <JO> odo;
	if (!read_jsonlist(params.odometry, &odo) ) {
		sm_error("Error opening input files.\n");
		return -1;
	}
	vector <JO> smStats;
	if (!read_jsonlist(params.smStats, &smStats) ) {
		sm_error("Error opening input files.\n");
		return -1;
	}	
	
	vector <JO> matchedOdo;
	vector <JO> matchedLas;
	matchOdoLas(odo, smStats, &matchedOdo, &matchedLas);
	int nOdo = (int)matchedOdo.size();
	int nLas = (int)matchedLas.size();
	vector <JO> integratedOdo;
	integratedOdo.reserve(nOdo);
	vector <JO> integratedLas;
	integratedLas.reserve(nLas);
/*!<!-- ######## STEP 1: INTEGRATES ODOMETRY AND CALCULATES ROTOTRANSLATION ########### -->*/
	double tmp_q[3];
	double q_int[3];
	double o[3];
	double T_i;
	double left_inc, right_inc;
	double ticks_to_rad;
	double w_l, w_r, v_i, w_i;
	double o_increment[3], t1, t2;	
	struct json_object *new_obj;
		
	//set initial position.
	jo_read_double_array (matchedOdo[0], "odometry", q_int, 3, 0.0);
	new_obj = json_object_new_object();
	jo_add_double_array (new_obj, "odometry", q_int, 3);
	integratedOdo.push_back(new_obj);

	for (int i = 0; i < nOdo-1; i++) {
		//calcolo v_k, w_k;
		T_i	  = gettime( matchedOdo[i+1], "timestamp") 	- gettime( matchedOdo[i], "timestamp");
		left_inc  = getticksD(matchedOdo[i+1], "left") 	- getticksD(matchedOdo[i], "left"); 
		right_inc = getticksD(matchedOdo[i+1], "right") 	- getticksD(matchedOdo[i], "right");
		ticks_to_rad =	2 * M_PI / ( 691.2 * 4 );
		w_l = (left_inc  * ticks_to_rad) / T_i;
		w_r = (right_inc * ticks_to_rad) / T_i;
		v_i = J11*w_l + J12*w_r;
		w_i = J21*w_l + J22*w_r;	
		o_increment[2] = w_i*T_i;

		if ( fabs(o_increment[2]) > 1e-7 ) {
			t1 = (   sin(o_increment[2]) / (o_increment[2]) );
			t2 = ( (1-cos(o_increment[2])) / (o_increment[2]) );
		}
		else {
			t1 = 1;
			t2 = 0;
		}
		//i-esimo passo di integrazione
		o_increment[0] = ( (v_i*T_i)*t1 );
		o_increment[1] = ( (v_i*T_i)*t2 );
		tmp_q[0] = q_int[0];
		tmp_q[1] = q_int[1];
		tmp_q[2] = q_int[2];
		planarComp(tmp_q, o_increment, q_int);
 		if (q_int[2] > M_PI) q_int[2] = q_int[2] - 2*M_PI;
 		if (q_int[2] < -M_PI) q_int[2] = q_int[2] + 2*M_PI;
		//store integration step
		new_obj = json_object_new_object();
		jo_add_double_array (new_obj, "odometry", q_int, 3);
		integratedOdo.push_back(new_obj);
	} // end for
	
	// calcolo della rototraslazione tra q0 e la qk ricavata dall'integrazione dell'odometria con i nuovi parametri
	jo_read_double_array (matchedOdo[0], "odometry", tmp_q, 3, 0.0);
	inv_planarComp(q_int, tmp_q, o);
	
	jo_read_double_array (matchedOdo[nOdo-1], "odometry", tmp_q, 3, 0.0);	
	printf("\nfinal odometry:\t%3.6f\t%3.6f\t%3.6f\n", tmp_q[0], tmp_q[1], tmp_q[2]);

/*!<!-- ########## STEP 2: CALCULATES ROTOTRANSLATION USING SCAN-MATCHER  ########### -->*/
	double tmp_s[3];
	double s[3];
	double s_int[3];

switch(params.use_gsl) {
	case 1: {
		cout << "Using GSL...\n";
		gsl_matrix * T = gsl_matrix_calloc(4,4);
		gsl_matrix * A = gsl_matrix_calloc(4,4);	
		gsl_matrix * R = gsl_matrix_calloc(4,4);		
			
		jo_read_double_array (matchedOdo[0], "odometry", tmp_s, 3, 0.0);
		new_obj = json_object_new_object();
		jo_add_double_array (new_obj, "odometrySm", tmp_s, 3);
		integratedLas.push_back(new_obj);
		rototras(A, tmp_s[0], tmp_s[1], tmp_s[2]);
		rototras(T, laser[0], laser[1], laser[2]);
		gsl_blas_dgemm (CblasNoTrans,CblasNoTrans, 1, A, T, 1, R);
		gsl_matrix_memcpy(A,R);
		gsl_matrix_free(R);

		for(int u = 0; u < nLas; u++) {
			R = gsl_matrix_calloc(4,4);
			jo_read_double_array (matchedLas[u], "x", tmp_s, 3, 0.0);
			rototras(T, tmp_s[0], tmp_s[1], tmp_s[2]);
			gsl_blas_dgemm (CblasNoTrans,CblasNoTrans, 1, A, T, 1, R);
			gsl_matrix_memcpy(A,R);
			gsl_matrix_free(R);			

			R = gsl_matrix_calloc(4,4);
			rototras(T, laser[0], laser[1], laser[2]);
			gsl_matrix * LU = gsl_matrix_alloc(4,4);
			gsl_permutation * p = gsl_permutation_alloc(4);
			gsl_matrix_memcpy(LU, T);
			int signum;
			gsl_linalg_LU_decomp (LU, p, &signum );
			gsl_linalg_LU_invert (LU, p, T );
			gsl_blas_dgemm (CblasNoTrans,CblasNoTrans, 1, A, T, 1, R);
			s[0] = gsl_matrix_get(R, 0, 3);
			s[1] = gsl_matrix_get(R, 1, 3);
			s[2] = atan2( gsl_matrix_get(R, 1, 0) , gsl_matrix_get(R, 0, 0) );
			new_obj = json_object_new_object();
			jo_add_double_array (new_obj, "odometrySm", s, 3);
			integratedLas.push_back(new_obj);
			gsl_matrix_free(R);
		}
		break;
	}
	case 0: {
		cout << "no GSL\n";
		jo_read_double_array (matchedOdo[0], "odometry", s_int, 3, 0.0);
		tmp_s[0] = s_int[0];
		tmp_s[1] = s_int[1];
		tmp_s[2] = s_int[2];
		new_obj = json_object_new_object();
		jo_add_double_array (new_obj, "odometrySm", s_int, 3);
		integratedLas.push_back(new_obj);
		
		planarComp(tmp_s, laser, s_int);
		if (s_int[2] > M_PI) s_int[2] = s_int[2] - 2*M_PI;
		if (s_int[2] < -M_PI) s_int[2] = s_int[2] + 2*M_PI;
		
		for (int u = 0; u < nLas; u++) {
			tmp_s[0] = s_int[0];
			tmp_s[1] = s_int[1];
			tmp_s[2] = s_int[2];
			jo_read_double_array (matchedLas[u], "x", s, 3, 0.0);	
			planarComp(tmp_s, s, s_int);
			if (s_int[2] > M_PI) s_int[2] = s_int[2] - 2*M_PI;
			if (s_int[2] < -M_PI) s_int[2] = s_int[2] + 2*M_PI;
			
			tmp_s[2] = s_int[2]- laser[2];
			tmp_s[0] = s_int[0] - laser[0]*cos(tmp_s[2]) + laser[1]*sin(tmp_s[2]);
			tmp_s[1] = s_int[1] - laser[0]*sin(tmp_s[2]) - laser[1]*cos(tmp_s[2]);
			if (tmp_s[2] > M_PI) tmp_s[2] = tmp_s[2] - 2*M_PI;
			if (tmp_s[2] < -M_PI) tmp_s[2] = tmp_s[2] + 2*M_PI;
			new_obj = json_object_new_object();
			jo_add_double_array (new_obj, "odometrySm", s_int, 3);
			integratedLas.push_back(new_obj);

		}
		tmp_s[0] = s_int[0];
		tmp_s[1] = s_int[1];
		tmp_s[2] = s_int[2];
	
		s_int[2] = tmp_s[2]- laser[2];
		s_int[0] = tmp_s[0] - laser[0]*cos(s_int[2]) + laser[1]*sin(s_int[2]);
		s_int[1] = tmp_s[1] - laser[0]*sin(s_int[2]) - laser[1]*cos(s_int[2]);		
		if (s_int[2] > M_PI) s_int[2] = s_int[2] - 2*M_PI;
		if (s_int[2] < -M_PI) s_int[2] = s_int[2] + 2*M_PI;
		
		new_obj = json_object_new_object();
		jo_add_double_array (new_obj, "odometrySm", s_int, 3);
		integratedLas.push_back(new_obj);

		jo_read_double_array (matchedOdo[0], "odometry", tmp_s, 3, 0.0);
		inv_planarComp(s_int, tmp_s, s);
	}
	default: break;
}

/*!<!-- ########## STEP 2: CALCULATES e = s (-) o  ########### -->*/
	double e[3];
	inv_planarComp(s, o, e);
	if (e[2] >  M_PI) e[2] = e[2] - 2*M_PI;
	if (e[2] < -M_PI) e[2] = e[2] + 2*M_PI;
	
/*!<!-- ####################		Output		#################### -->*/
	new_obj = json_object_new_object();
	jo_add_double_array (new_obj, "o", o, 3);
	jo_add_double_array (new_obj, "s", s, 3);
	jo_add_double_array (new_obj, "e", e, 3);
	// Write json object to file
	FILE * output;
	if(!strcmp(params.out, "stdout")) { output = stdout;}
	else {output = fopen(params.out, "w");}
	fprintf(output, "%s\n", json_object_to_json_string(new_obj) );
	fclose(output);

/*!<!-- ####################		Graph		#################### -->*/

	Analyzer * an;

//Creating Graph using integrated odometry
	Graph * ix_graph  = new Graph(); Graph * iy_graph  = new Graph(); Graph * ith_graph = new Graph();
	an->set_par(ix_graph, "x_int.vgr" , "X int coordinate"	 , "step", "x", "lines", "Integrated Odo");
	an->set_par(iy_graph, "y_int.vgr" , "Y int coordinate"	 , "step", "y", "lines", "Integrated Odo");
	an->set_par(ith_graph,"th_int.vgr","THETA int coordinate", "step", "th","lines", "Integrated Odo");
	double * int_x    = new double[nOdo];
	double * int_y    = new double[nOdo];
	double * int_th   = new double[nOdo];
	double * int_axis = new double[nOdo];
	for (int i = 0; i < nOdo; i++) {
		int_axis[i] = i;
		jo_read_double_array (integratedOdo[i], "odometry", tmp_s, 3, 0.0);
		int_x[i]  = tmp_s[0]; int_y[i]  = tmp_s[1]; int_th[i] = tmp_s[2];
	}
	an->add_data_graph(int_x, int_axis, nOdo, ix_graph);
	an->add_data_graph(int_y, int_axis, nOdo, iy_graph);
	an->add_data_graph(int_th,int_axis, nOdo, ith_graph);

//Creating Graph using Scan Matcher rototranslation
	Graph * lx_graph  = new Graph(); Graph * ly_graph  = new Graph(); Graph * lth_graph = new Graph();
	an->set_par(lx_graph, "x_las.vgr" , "X las coordinate"	 , "step", "x", "lines", "Scan Matcher");
	an->set_par(ly_graph, "y_las.vgr" , "Y las coordinate"	 , "step", "y", "lines", "Scan Matcher");
	an->set_par(lth_graph,"th_las.vgr","THETA las coordinate", "step", "th","lines", "Scan Matcher");

	nLas = nLas+1;
	double * las_x 	  = new double[nLas];
	double * las_y 	  = new double[nLas];
	double * las_th   = new double[nLas];
	double * las_axis = new double[nLas];
	for (int i = 0; i < nLas; i++) {
		las_axis[i] = i;
		jo_read_double_array (integratedLas[i], "odometrySm", tmp_s, 3, 0.0);
		las_x[i]  = tmp_s[0]; las_y[i]  = tmp_s[1]; las_th[i] = tmp_s[2];
	}
	an->add_data_graph(las_x, las_axis, nLas, lx_graph);
	an->add_data_graph(las_y, las_axis, nLas, ly_graph);
	an->add_data_graph(las_th,las_axis, nLas, lth_graph);

//Creating Graph using odometry
	Graph * ox_graph  = new Graph(); Graph * oy_graph  = new Graph(); Graph * oth_graph = new Graph();
	an->set_par(ox_graph, "x_odo.vgr" ,  "X odo coordinate"	  , "step", "x", "lines", "Odometry");
	an->set_par(oy_graph, "y_odo.vgr" ,  "Y odo coordinate"	  , "step", "y", "lines", "Odometry");
	an->set_par(oth_graph,"th_odo.vgr", "THETA odo coordinate", "step", "th","lines", "Odometry");
	double * odo_x 	  = new double[nOdo];
	double * odo_y 	  = new double[nOdo];
	double * odo_th   = new double[nOdo];
	double * odo_axis = new double[nOdo];
	for (int i = 0; i < nOdo; i++) {
		odo_axis[i] = i;
		jo_read_double_array (matchedOdo[i], "odometry", tmp_s, 3, 0.0);
		odo_x[i]  = tmp_s[0]; odo_y[i]  = tmp_s[1]; odo_th[i] = tmp_s[2];
	}
	an->add_data_graph(odo_x, odo_axis, nOdo, ox_graph);
	an->add_data_graph(odo_y, odo_axis, nOdo, oy_graph);
	an->add_data_graph(odo_th,odo_axis, nOdo, oth_graph);

// Creating Graph s_theta (as y) vs i_theta (as x)
	Graph * thesi_graph  = new Graph();
	an->set_par(thesi_graph, "th_s-i.vgr", "THETA sm versus integrated", "i_theta", "s_theta", "points", "");
	an->add_data_graph(las_th, int_th, nOdo, thesi_graph);

// Creating Graph trajectory
	double * spec_int_y    = new double[nOdo];
	double * spec_odo_y    = new double[nOdo];
	double * spec_las_y    = new double[nLas];
	for (int i = 0; i < nOdo; i++) {
		spec_int_y[i]  = -int_y[i];
		spec_odo_y[i]	= -odo_y[i];
	}
	for (int i = 0; i < nLas; i++) {
		spec_las_y[i]  = -las_y[i];
	}
	Graph traj_graph[3];
	an->set_par(&(traj_graph[0]), "i_traj.vgr", "Trajectory integrated"  , "", "", "lines", "integrated Odo");
	an->set_par(&(traj_graph[1]), "l_traj.vgr", "Trajectory scan matcher", "", "", "lines", "Scan matcher");
	an->set_par(&(traj_graph[2]), "o_traj.vgr", "Trajectory odometry"    , "", "", "lines", "odometry");
	
	an->add_data_graph(int_x, spec_int_y, nOdo, &traj_graph[0]);
	an->add_data_graph(las_x, spec_las_y, nOdo, &traj_graph[1]);
	an->add_data_graph(odo_x, spec_odo_y, nOdo, &traj_graph[2]);	

// Creating error plots
	Graph * eth_graph, * ethAbs_graph, * releth_graph;

	if (nLas == nOdo) {
		// Creating graph absolute theta error (int_th - las_th)
		ethAbs_graph = new Graph();
		an->set_par(ethAbs_graph, "err_th_abs.vgr", "Absolute THETA error", "step", "|E_th|", "lines", "");
		double * ethAbs = new double[nOdo];
		for (int i = 0; i < nOdo; i++) ethAbs[i] = fabs(int_th[i] - las_th[i]);
		an->add_data_graph(ethAbs, int_axis, nOdo, ethAbs_graph);

		// Creating graph theta error (int_th - las_th)
		eth_graph = new Graph();
		an->set_par(eth_graph, "err_th.vgr", "THETA error", "step", "E_th", "lines", "");
		double * eth = new double[nOdo];
		for (int i = 0; i < nOdo; i++) eth[i] = (int_th[i] - las_th[i]);
		an->add_data_graph(eth, int_axis, nOdo, eth_graph);

		// Creating graph error theta vs integrated theta
		releth_graph = new Graph();
		an->set_par(releth_graph, "rel_err_th.vgr", "relative THETA error", "i_th", "i_th - s_th", "points", "");
		an->add_data_graph(eth, int_th, nOdo, releth_graph);
	}
	else {
		cout << "Note: number of laser entries is: " << nLas 
		<< ", while number of odometry entries is: " << nOdo
		<< "\n Omitting error graphs!\n";
	}

// Creating plots
	int n_plots = 3;
	Graph * x_plots  = new Graph[n_plots];
	Graph * y_plots  = new Graph[n_plots];
	Graph * th_plots = new Graph[n_plots];
	
	 x_plots[0] =  ix_graph[0];  x_plots[1] =  lx_graph[0];  x_plots[2] = ox_graph[0];
	 y_plots[0] =  iy_graph[0];  y_plots[1] =  ly_graph[0];  y_plots[2] = oy_graph[0];
	th_plots[0] = ith_graph[0]; th_plots[1] = lth_graph[0]; th_plots[2] = oth_graph[0];

// Plotting
	cout << "Plotting odometry's and laser's graphs" << endl;
	an->create_same_plot("X coordinate", "step", "x", x_plots, n_plots, "x_plots.vpl", 1, 1);
	an->plot("x_plots.vpl");
	an->create_same_plot("Y coordinate", "step", "y", y_plots, n_plots, "y_plots.vpl", 1, 1);
	an->plot("y_plots.vpl");
	an->create_same_plot("Theta coordinate", "step", "th", th_plots, n_plots, "th_plots.vpl", 1, 1);
	an->plot("th_plots.vpl");
	an->create_same_plot("Trajectory", "world x", "world y", traj_graph, 3, "traj_plot.vpl", 1, 1);
	an->plot("traj_plot.vpl");

	if (nLas == nOdo) {
		cout << "Plotting errors' graphs" << endl;
		an->create_plot(thesi_graph, 1, "thesi_plot.vpl", 1, 0);
		an->plot("thesi_plot.vpl");
		an->create_plot(eth_graph  , 1, "eth_plot.vpl", 1, 0);
		an->plot("eth_plot.vpl");
		an->create_plot(ethAbs_graph,1, "ethAbs_plot.vpl", 1, 0);
		an->plot("ethAbs_plot.vpl");
		an->create_plot(releth_graph,1, "releth_plot.vpl", 1, 0);
		an->plot("releth_plot.vpl");
	}
	return 0;
}
