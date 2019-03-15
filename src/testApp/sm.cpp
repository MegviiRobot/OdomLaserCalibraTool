#include <time.h>
#include <string.h>
#include <libgen.h>

#include <options/options.h>
#include <csm/utils.h>
#include "csm/csm_all.h"
#include <csm/laser_data.h>

struct {
	const char * file_in;
	const char * file_out;
	const char * file_out_stats;
	const char * file_jj;
	int format;
	
	/* which algorithm to run */
	int algo;
	
	int recover_from_error;
	
	int debug;
} p;

extern void sm_options(struct sm_params*p, struct option*ops);

void spit(LDP ld, FILE * stream);

int main(int argc, const char*argv[]) {
	sm_set_program_name(argv[0]);
	
	struct sm_params params;
	struct sm_result result;
	
	struct option* ops = options_allocate(100);
	options_string(ops, "in", &p.file_in, "stdin", "Input file ");
	options_string(ops, "out", &p.file_out, "stdout", "Output file ");
	options_string(ops, "out_stats", &p.file_out_stats, "", "Output file (stats) ");
	options_string(ops, "file_jj", &p.file_jj, "",
		"File for journaling -- if left empty, journal not open.");
	options_int(ops, "algo", &p.algo, 0, "Which algorithm to use (0:(pl)ICP 1:gpm-stripped 2:HSM) ");
	
	options_int(ops, "debug", &p.debug, 0, "Shows debug information");
	options_int(ops, "recover_from_error", &p.recover_from_error, 0, "If true, tries to recover from an ICP matching error");
	
	
	p.format = 0;
/*	options_int(ops, "format", &p.format, 0,
		"Output format (0: log in JSON format, 1: log in Carmen format (not implemented))");*/
	
	sm_options(&params, ops);
	if(!options_parse_args(ops, argc, argv)) {
		fprintf(stderr, "\n\nUsage:\n");
		options_print_help(ops, stderr);
		return -1;
	}

	sm_debug_write(p.debug);

	/* Open input and output files */
	
	FILE * file_in = open_file_for_reading(p.file_in);
	if(!file_in) return -1;
	FILE * file_out = open_file_for_writing(p.file_out);
	if(!file_out) return -1;
	
	if(strcmp(p.file_jj, "")) {
		FILE * jj = open_file_for_writing(p.file_jj);
		if(!jj) return -1;
//		jj_set_stream(jj);
	}
	
	FILE * file_out_stats = 0;
	if(strcmp(p.file_out_stats, "")) {
		file_out_stats = open_file_for_writing(p.file_out_stats);
		if(!file_out_stats) return -1;
	}
	
	/* Read first scan */
	LDP laser_ref;
	if(!(laser_ref = ld_read_smart(file_in))) {
		sm_error("Could not read first scan.\n");
		return -1;
	}
	if(!ld_valid_fields(laser_ref))  {
		sm_error("Invalid laser data in first scan.\n");
		return -2;
	}
	
	
	/* For the first scan, set estimate = odometry */
	copy_d(laser_ref->odometry, 3, laser_ref->estimate);
	
	spit(laser_ref, file_out);
	int count=-1;
	LDP laser_sens;
	while( (laser_sens = ld_read_smart(file_in)) ) {
		
		count++;
		if(!ld_valid_fields(laser_sens))  {
			sm_error("Invalid laser data in (#%d in file).\n", count);
			return -(count+2);
		}
		
		params.laser_ref  = laser_ref;
		params.laser_sens = laser_sens;

		/* Set first guess as the difference in odometry */
		
		if(	any_nan(params.laser_ref->odometry,3) ||  
			any_nan(params.laser_sens->odometry,3) ) {
				sm_error("The 'odometry' field is set to NaN so I don't know how to get an initial guess. I usually use the difference in the odometry fields to obtain the initial guess.\n");
				sm_error("  laser_ref->odometry = %s \n",  friendly_pose(params.laser_ref->odometry) );
				sm_error("  laser_sens->odometry = %s \n", friendly_pose(params.laser_sens->odometry) );
				sm_error(" I will quit it here. \n");
				return -3;
		}
		
		double odometry[3];
		pose_diff_d(laser_sens->odometry, laser_ref->odometry, odometry);
		double ominus_laser[3], temp[3];
		ominus_d(params.laser, ominus_laser);
		oplus_d(ominus_laser, odometry, temp);
		oplus_d(temp, params.laser, params.first_guess);
		
		/* Do the actual work */
        sm_icp(&params, &result);
		
		if(!result.valid){
			if(p.recover_from_error) {
				sm_info("One ICP matching failed. Because you passed  -recover_from_error, I will try to recover."
				" Note, however, that this might not be good in some cases. \n");
				sm_info("The recover is that the displacement is set to 0. No result stats is output. \n");
				
				/* For the first scan, set estimate = odometry */
				copy_d(laser_ref->estimate, 3, laser_sens->estimate);
				
				ld_free(laser_ref); laser_ref = laser_sens;
				
			} else {
				sm_error("One ICP matching failed. Because I process recursively, I will stop here.\n");
				sm_error("Use the option -recover_from_error if you want to try to recover.\n");
				ld_free(laser_ref);
				return 2;
			}
		} else {
		
			/* Add the result to the previous estimate */
			oplus_d(laser_ref->estimate, result.x, laser_sens->estimate);

			/* Write the corrected log */
			spit(laser_sens, file_out);

//			/* Write the statistics (if required) */
//			if(file_out_stats) {
//				JO jo = result_to_json(&params, &result);
//				fputs(jo_to_string(jo), file_out_stats);
//				fputs("\n", file_out_stats);
//				jo_free(jo);
//			}

			ld_free(laser_ref); laser_ref = laser_sens;
		}
	}
	ld_free(laser_ref);
	
	return 0;
}


void spit(LDP ld, FILE * stream) {
	switch(p.format) {
		case(0): {
//			ld_write_as_json(ld, stream);
			break;
		}
		case(1): {
			/* XXX: to implement */
			break;
		}
	}
}

