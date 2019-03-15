/*!
		\file solver_options.h
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief options for the solver executable
*/

#ifndef _SOLVER_OPTIONS_H
#define _SOLVER_OPTIONS_H

/*! 	
	\struct solver_params
  	\brief Solver parameter instance structure
	@param input_file filename containing a list of tuples obtained from synchronizer
	@param output_file filename for output
	@param mode computation mode
*/
typedef struct {
	const char * input_file;	
	const char * output_file;
	int mode;
	
	double max_cond_number;
	
	int outliers_iterations;
	double outliers_percentage;
	
	int debug;
} solver_params;

/*!
	Initializes parameters for solver procedure
	\param p pointer to parameters' structure
	\param ops options' structure
*/
void solver_options(solver_params*p, struct option*ops);

#endif

