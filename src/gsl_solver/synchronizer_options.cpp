/*!
		\file synchronizer_options.cpp
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Options for the syncronizer executable. (implementation file)
*/

#include <options/options.h>
#include "synchronizer_options.h"

void synchro_options(synchro_params *p, struct option*ops) {

	options_string(ops, "las_file",
		&(p->las_file), "", 
		"Input sm_stats file");

	options_string(ops, "odo_file", 
		(&p->odo_file), "", 
		"Input odometry file ");

	options_string(ops, "output", 
		&(p->out_file), "stdout", 
		"Output file ");

	options_double(ops, "laser_timestamp_correction", 
		&(p->laser_timestamp_correction), 0.0, 
		"Correction for laser timestamp");

	options_double(ops, "perc_threshold", 
		&(p->perc_threshold), 0.95,
		"Percentual threshold");

	options_double(ops, "time_tolerance", 
		&(p->time_tolerance), 0.05,
		"A threshold on time tolerance (sec)");

	options_double(ops, "tk_threshold",
		&(p->tk_threshold),  3.0, 
		"A threshold on the differerence between laser_sens_timestamp and laser_ref_timestamp (sec)");

}
