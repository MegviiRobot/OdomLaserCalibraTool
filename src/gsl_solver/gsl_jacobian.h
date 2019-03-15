// Utilities for computing jacobian of a function

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

gsl_matrix * gsl_jacobian( gsl_vector * (*function)(const gsl_vector* x, void*params), const gsl_vector *x, void*params);
