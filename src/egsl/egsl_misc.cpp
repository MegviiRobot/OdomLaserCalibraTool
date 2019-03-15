#include <math.h>
#include "egsl/egsl.h"
#include <stdio.h>
using namespace std;

val egsl_rot(double theta) {
	double R[2*2] = {
		cos(theta), -sin(theta),
		sin(theta),  cos(theta)
	};
	return egsl_vFda(2,2,R);
}

val egsl_zeros(size_t rows, size_t columns) {
	val v = egsl_alloc(rows,columns);
	gsl_matrix * m = egsl_gslm(v);
	gsl_matrix_set_all(m,0.0);
	return v;
}

val egsl_ones(size_t rows, size_t columns) {
	val v = egsl_alloc(rows,columns);
	gsl_matrix * m = egsl_gslm(v);
	gsl_matrix_set_all(m,1.0);
	return v;
}

val egsl_vers(double theta){
	double v[2] = { cos(theta), sin(theta)};
	return egsl_vFa(2,v);
}

//void egsl_print_spectrum(const char*s, val v) {
//	gsl_matrix *m = egsl_gslm(v);
//	/* expect same size */
//	size_t n = m->rows();
//	double eval[n]; val evec[n];
//	egsl_symm_eig(v, eval, evec);
// 	size_t i,j;
//	for(j=0;j<n;j++) {
//		fprintf(stderr, "%s | eval[%d] = %+5.5f evec[%d]= ",
//			s, (int)j, eval[j],(int)j);
//		for(i=0;i<n;i++)
//			fprintf(stderr, "%+4.4f ", egsl_atv(evec[j],i));
//		fprintf(stderr, " sqrt(eval[%d])=%5.5f  \n", (int)j, sqrt(eval[j]));
//	}
//}
