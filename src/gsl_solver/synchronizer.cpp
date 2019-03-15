/*!
		\file synchronizer.cpp
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief synchronizes an odometry log and a laser log
*/
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <iostream>

#include <json-c/json.h>
#include <options/options.h>
#include <csm/csm_all.h>

#include "synchronizer_options.h"
#include "aux_functions.h"

using namespace std;
using namespace CSM;

extern void synchro_options(synchro_params*p, struct option*ops);

int main(int argc, const char * argv[]) {
	sm_set_program_name("Synchronizer");

//!<!--###################    Verify options   #########################-->
	synchro_params params;	
	struct option* ops = options_allocate(100);
	synchro_options(&params, ops);
	if(!options_parse_args(ops, argc, argv)) {
		options_print_help(ops, stderr);
		return -1;
	}
	
//!<!--##################### 		Read logs			#######################-->
	vector <JO> laser; 
	vector <JO> odo;
	vector <JO> result;
	if (!read_jsonlist(params.las_file, &laser) ) {
		sm_error("Error reading laser_file '%s'.\n", params.las_file);
		return -1;
	}
	if (!read_jsonlist(params.odo_file, &odo) ) {
		sm_error("Error reading odometry file '%s'.\n", params.odo_file);
		return -1;
	}

	
//!<!--###################			 Start synchronization 			####################-->
	sm_info("Starting synchronization\n");	
	int n = (int) laser.size();
	
	int discarded_valid 	= 0;
	int discarded_error 	= 0;
	int discarded_nvalid 	= 0;
	int discarded_t_k 	= 0;
	int discarded_time_tol 	= 0;
	int discarded_no_odo 	= 0;
	int still_num		= 0;

//!<!--#########		calculate mean error, mean error threshold, nvalid threshold	########-->
	double mean_error[n];
	double mean_error_sorted[n];
	int nvalid_sorted[n];
	double perc = params.perc_threshold;
	
	for(int i = 0; i < n; i++) {
		double error;
		int nvalid;
		
		jo_read_double(laser[i], "error", &error);
		jo_read_int(laser[i], "nvalid", &nvalid);
		mean_error[i] = error/nvalid;
		mean_error_sorted[i] = mean_error[i];
		nvalid_sorted[i] = nvalid;
	}

	mysort(mean_error_sorted, n);
	mysort(nvalid_sorted, n);
	double mean_error_threshold = mean_error_sorted[ (int)round(perc*n)];
//	cout << "mean_error_threshold " << mean_error_threshold << endl;
	int nvalid_threshold = nvalid_sorted[ (int)round((1 - perc)*n)];

//!<!--########################			Analyze logs		################################-->
	int m = 0;	
	int last = 0;
	int jb_ref, jb_sens;
	double error, t_k, left_speed, right_speed;
	int nvalid, valid;
	double tr1, tr2, ts1, ts2;
	double Lr1, Lr2, Ls1, Ls2;
	double Rr1, Rr2, Rs1, Rs2;
	double laser_ref_ts, laser_sens_ts, t_delta;
	double times[4];
	double time_tol;
	double ref_alpha, sens_alpha, avg_left_start, avg_right_start,
		  avg_left_end, avg_right_end, left_inc, right_inc;

	double ticks_to_rad = 2 * M_PI / ( 691.2 * 4 );
	for (int k = 0; k < n; k++) {
		valid = jo_read_int(laser[k], "valid", &valid);
		if (valid == 0) {
			discarded_valid = discarded_valid + 1;
			continue;
		}
		
		/*
			for each entry in raw_sm we seek the two indices
			which are immediately before and after in raw_odo
		*/
		t_delta = params.laser_timestamp_correction;
		laser_ref_ts 	= t_delta + gettime(laser[k], "laser_ref_timestamp");
		laser_sens_ts 	= t_delta + gettime(laser[k], "laser_sens_timestamp");
		
		jb_ref  = find_closest_ref(laser_ref_ts, odo, last);
		jb_sens = find_closest_sens(laser_sens_ts, odo, last);
// 		jb_ref  = find_closest(laser_ref_ts, odo, last);
// 		jb_sens = find_closest(laser_sens_ts, odo, last);
		
		if ( (jb_ref == -1) || (jb_sens == -1) ) {
			discarded_no_odo = discarded_no_odo + 1;
			sm_info("merge_logs: Could not match timestamp %f %f\n", laser_ref_ts, laser_sens_ts);
			continue;
		} 
		last = jb_ref;

		tr1 = gettime(odo[jb_ref]	, "timestamp");
		tr2 = gettime(odo[jb_ref +1]	, "timestamp");
		ts1 = gettime(odo[jb_sens -1]	, "timestamp");
		ts2 = gettime(odo[jb_sens]	, "timestamp");

		times[0] = fabs( tr1 - laser_ref_ts ); 
		times[1] = fabs( tr2 - laser_ref_ts ); 
		times[2] = fabs( ts1 - laser_sens_ts ); 
		times[3] = fabs( ts2 - laser_sens_ts ); 
		mysort(times, 4);
		time_tol = times[3];
		
		ref_alpha  = (laser_ref_ts - tr1) / (tr2 - tr1);
		sens_alpha = (laser_sens_ts - ts1) /(ts2 - ts1);
		
		Lr1 = getticks(odo[jb_ref]	, "left");
		Lr2 = getticks(odo[jb_ref+1]	, "left");
		Ls1 = getticks(odo[jb_sens-1]	, "left");
		Ls2 = getticks(odo[jb_sens]	, "left");
		Rr1 = getticks(odo[jb_ref]	, "right");
		Rr2 = getticks(odo[jb_ref+1]	, "right");
		Rs1 = getticks(odo[jb_sens-1]	, "right");
		Rs2 = getticks(odo[jb_sens]	, "right");

		avg_left_start  = (1-ref_alpha) * Lr1 + ref_alpha * Lr2;
		avg_right_start = (1-ref_alpha) * Rr1 + ref_alpha * Rr2;
 		avg_left_end 	= (1-sens_alpha)* Ls1 + sens_alpha* Ls2;
		avg_right_end 	= (1-sens_alpha)* Rs1 + sens_alpha* Rs2;

		left_inc  = (avg_left_end - avg_left_start); 
		right_inc = (avg_right_end - avg_right_start);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// 		// Calculate time tolerance
// 		double times[4];
// 		times[0] = fabs( gettime(odo[jb_ref], "timestamp") - laser_ref_ts ); 
// 		times[1] = fabs( gettime(odo[jb_ref+1], "timestamp") - laser_ref_ts ); 
// 		times[2] = fabs( gettime(odo[jb_sens], "timestamp") - laser_sens_ts ); 
// 		times[3] = fabs( gettime(odo[jb_sens+1], "timestamp") - laser_sens_ts ); 
// 		mysort(times, 4);
// 		double time_tol = times[3];
// 		
// 		// Check if the robot is still
// 		double ref_alpha = (laser_ref_ts - gettime(odo[jb_ref],"timestamp")) / (gettime(odo[jb_ref+1],"timestamp") - gettime(odo[jb_ref],"timestamp") );
// 		double sens_alpha = ( laser_sens_ts - gettime(odo[jb_sens],"timestamp")) / (gettime(odo[jb_sens+1],"timestamp") - gettime(odo[jb_sens],"timestamp") );
// 		
// // 		double avg_left_start = ( getticks(odo[jb_ref], "left") + getticks(odo[jb_ref+1], "left") )/2;
// // 		double avg_right_start = ( getticks(odo[jb_ref], "right") + getticks(odo[jb_ref+1], "right") )/2;
// // 		double avg_left_end   = ( getticks(odo[jb_sens], "left") + getticks(odo[jb_sens+1], "left") )/2;		
// // 		double avg_right_end   = ( getticks(odo[jb_sens], "right") + getticks(odo[jb_sens+1], "right") )/2;
// 		double avg_left_start = ref_alpha*getticks(odo[jb_ref], "left")	+ (1-ref_alpha)*getticks(odo[jb_ref+1], "left");
// 		double avg_right_start = ref_alpha*getticks(odo[jb_ref],"right")+(1-ref_alpha)*getticks(odo[jb_ref+1],"right");	
// 
// 		double avg_left_end = sens_alpha*getticks(odo[jb_sens],"left") + (1-sens_alpha)*getticks(odo[jb_sens+1],"left");
// 		double avg_right_end = sens_alpha*getticks(odo[jb_sens],"right")+(1-sens_alpha)*getticks(odo[jb_sens+1],"right");
// 
// 		double left_inc  = (avg_left_end - avg_left_start); 
// 		double right_inc = (avg_right_end - avg_right_start);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>	
				
		if ( (left_inc == 0) && (right_inc == 0) ) {
			still_num = still_num + 1;
			continue;
		}
		
		/* Check if data is awful and eventually discard it */
		jo_read_double(laser[k], "error", &error);
		if( error == 0 ) {
			continue;
		}
		
		if(mean_error[k] > mean_error_threshold) {
			discarded_error = discarded_error + 1;
			continue;
		}

		jo_read_int(laser[k], "nvalid", &nvalid);
		if( nvalid <= nvalid_threshold ) {
			discarded_nvalid = discarded_nvalid + 1;
			continue;
		}

		if ( time_tol > params.time_tolerance ) {
			discarded_time_tol  = discarded_time_tol + 1;
			continue;
		}
		
		t_k = laser_sens_ts - laser_ref_ts;
		
		if( t_k > params.tk_threshold) {
			discarded_t_k  = discarded_t_k + 1;
			cout << "Interval between scans exceedes limit: " << t_k << " > " << params.tk_threshold << endl;
			continue;
		}

		left_speed   = (left_inc  * ticks_to_rad) / t_k;
		right_speed  = (right_inc * ticks_to_rad) / t_k;

		/* Build k-esim json object*/		
		struct json_object *new_obj;
		new_obj = json_object_new_object();
		json_object_object_add(new_obj, "T", json_object_new_double(t_k) );		
		json_object_object_add(new_obj, "phi_l", json_object_new_double(left_speed) );		
		json_object_object_add(new_obj, "phi_r", json_object_new_double(right_speed) );
		double sm[3];
		jo_read_double_array (laser[k], "x", sm, 3, 0.0);
		jo_add_double_array (new_obj, "sm", sm, 3);
//		double odometry[3];
// 		jo_read_double_array (odo[jb_ref], "odometry", odometry, 3, 0.0);
// 		jo_add_double_array (new_obj, "odometry", odometry, 3);
// 		json_object_object_add(new_obj, "time_tol", json_object_new_double(time_tol) );
// 		json_object_object_add(new_obj, "error", json_object_new_double(error) );
// 		json_object_object_add(new_obj, "mean_error", json_object_new_double(mean_error[k]) );
// 		json_object_object_add(new_obj, "nvalid", json_object_new_int(nvalid) );
// 		json_object_object_add(new_obj, "sens_alpha", json_object_new_double(sens_alpha) );
// 		json_object_object_add(new_obj, "ref_alpha", json_object_new_double(ref_alpha) );
		result.push_back(new_obj)	;
			
		m = m + 1;
	}

//!<!--######################				Output file				############################-->
	if ( !write_jsonlist(params.out_file, &result) )  {
		sm_error("Error writing on output file '%s'.\n", params.out_file);
		return (-1);	
	}	

	sm_info(" still number: %d\n", still_num);	
	sm_info(" Number of valid data: %d\n", m);
	sm_info(" discarded valid: %d\n", discarded_valid);
	sm_info(" discarded error: %d\n", discarded_error);
	sm_info(" discarded nvalid: %d\n", discarded_nvalid);
	sm_info(" discarded_t_k: %d\n", discarded_t_k);
	sm_info(" discarded_time_tol: %d\n", discarded_time_tol);
	sm_info(" discarded_no_odo: %d\n", discarded_no_odo);
	
	return 0;
}
