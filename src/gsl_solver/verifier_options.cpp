/*!	
	\file verifier_options.ccp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Manages options for verifier (implementation file)
*/
#include <options/options.h>
#include "verifier_options.h"

void verifier_options(verifier_params *p, struct option*ops) {

 	options_string(ops, "smStats",
 		&(p->smStats), "", 
 		"Input laser file");

	options_string(ops, "odometry", 
		(&p->odometry), "", 
		"Input odometry file ");

	options_string(ops, "calib_params", 
		&(p->calib_params), "", 
		"Input file with estimated parameters");

	options_string(ops, "out", 
		&(p->out), "stdout", 
		"Output file");
	
	options_int(ops, "use_gsl", 
		&(p->use_gsl), 0, 
		"0: no GSL; 1 use GSL");
}
