/*!
		\file olver_options.cpp
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Options for the solver executable. Initalization method
*/


#include <options/options.h>
#include "solver_options.h"

void solver_options(solver_params *p, struct option*ops) {

	options_string(ops, "input_file",
		&(p->input_file), "", 
		"Input file containing a list of tuples");

	options_string(ops, "output_file", 
		(&p->output_file), "", 
		"Output file ");
	
	options_int(ops, "mode", 
		(&p->mode), 0, 
		"Minimization method ");
		
	options_int(ops, "outliers_iterations", (&p->outliers_iterations), 4, 
		"Number of outliers removal iterations.");
	options_double(ops, "outliers_percentage", &(p->outliers_percentage), 0.01, 
		"Percentage of outliers to reject at each iteration. Double value: 0.01 == 1%%" );
		
	options_int(ops, "debug", &(p->debug), 0, "Shows debug information");

	options_double(ops, "max_cond_number", &(p->max_cond_number), 75, 
		"Max condition number allowed for the first linear estimation step." );
		
}
