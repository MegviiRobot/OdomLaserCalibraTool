/*!	
	\file verifier_options.h
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Manages options for verifier (header file)
*/
#ifndef _VERIFIER_OPTIONS_H
#define _VERIFIER_OPTIONS_H

/*!
	\struct verifier_params
	@param sm_stats laser_stats input filename
	@param odometry odometry input filename
	@param parameters estimated parameters filename
	@param out output filename
*/
typedef struct {
	const char * smStats;
	const char * odometry;
	const char * calib_params;
	const char * out;
	int use_gsl;
} verifier_params;

/*!
	Initializes parameters for verifier procedure
	\param p pointer to parameters' structure
	\param ops options' structure
*/
void verifier_options(verifier_params*p, struct option*ops);

#endif

