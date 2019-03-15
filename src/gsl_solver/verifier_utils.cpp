/*! 
	\file   verifier_utils.cpp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief  Auxiliary methods for verifier.cpp
 */

#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_matrix.h>

using namespace std;

// a, b, result are pointers to array with three elements! 
// a(+)b
void planarComp(double * a, double * b, double * result) {
	result[0] = a[0] + b[0]*cos(a[2]) - b[1]*sin(a[2]);
	result[1] = a[1] + b[0]*sin(a[2]) + b[1]*cos(a[2]);
	result[2] = a[2] + b[2];
}

// a, b, result are pointers to array with three elements!
// a(-)b
void inv_planarComp(double * a, double * b, double * result) {
	result[2] = a[2] - b[2];
	result[0] = + a[0]*cos(b[2]) + a[1]*sin(b[2]) - b[0]*cos(b[2]) - b[1]*sin(b[2]);
	result[1] = - a[0]*sin(b[2]) + a[1]*cos(b[2]) + b[0]*sin(b[2]) - b[1]*cos(b[2]);
}

void rototras(gsl_matrix * M, double x, double y, double theta){
	/* M =	[	 cos(theta)	-sin(theta)	0	x
				 sin(theta)	 cos(theta)	0	y
					0			0		1	0
					0			0		0	1	] */
	gsl_matrix_set_zero(M);
	gsl_matrix_set(M , 0, 0, cos(theta) );
	gsl_matrix_set(M , 0, 1, -sin(theta) );
	gsl_matrix_set(M , 0, 2, 0 );
	gsl_matrix_set(M , 0, 3, x );
	gsl_matrix_set(M , 1, 0, sin(theta) );
	gsl_matrix_set(M , 1, 1, cos(theta) );
	gsl_matrix_set(M , 1, 2, 0 );
	gsl_matrix_set(M , 1, 3, y );
	gsl_matrix_set(M , 2, 0, 0 );
	gsl_matrix_set(M , 2, 1, 0 );
	gsl_matrix_set(M , 2, 2, 1.0 );
	gsl_matrix_set(M , 2, 3, 0 );
	gsl_matrix_set(M , 3, 0, 0 );
	gsl_matrix_set(M , 3, 1, 0 );
	gsl_matrix_set(M , 3, 2, 0 );
	gsl_matrix_set(M , 3, 3, 1.0 );
}

