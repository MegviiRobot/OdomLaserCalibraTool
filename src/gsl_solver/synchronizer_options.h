/*!
		\file synchronizer_options.h
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Options for the synchronizer executable (header file)
*/

#ifndef _SYNCHRONIZER_OPTIONS_H
#define _SYNCHRONIZER_OPTIONS_H

/*! 
	\struct synchro_params
	\brief Synchronizer parameter instance structure
	@param las_file Input sm_stats file for synchronizer procedure.\n Default Null
	@param odo_file Input odometry file for synchronizer procedure.\n Default Null
	@param out_file Output file of synchronizer procedure.\n Default stdout
	@param laser_timestamp_correction Correction for laser timestamp.\n Default 0.0
	@param perc_threshold Value used to define a threshold on mean error and nvalid.\n Default 0.95, range is [0,1]
	@param time_tolerance A threshold on time tolerance.\n Default 0.05
	@param tk_threshold A threshold on the differerence between laser_sens_timestamp and laser_ref_timestamp.\n Default 5.0
*/
typedef struct {
	const char * las_file;
	const char * odo_file;
	const char * out_file;
	double laser_timestamp_correction;
	double perc_threshold;
	double time_tolerance;
	double tk_threshold;	
} synchro_params;

/*!
	\brief Initializes parameters for synchronization procedure
	\param p pointer to parameters' structure
	\param ops options' structure
*/
void synchro_options(synchro_params*p, struct option*ops);

#endif

