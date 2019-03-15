#include "egsl/egsl.h"

val egsl_vFda(size_t rows, size_t cols, const double *a) {
	val v = egsl_alloc(rows, cols);

	size_t i; size_t j;
	for(i=0;i<rows;i++)
	for(j=0;j<cols;j++) {
		*egsl_atmp(v,i,j) = a[j+i*cols];
	}
	return v;
}

val egsl_vFa(size_t rows, const double*a) {
	val v = egsl_alloc(rows,1);
	size_t i;
	for(i=0;i<rows;i++)
		*egsl_atmp(v,i,0) =  a[i];
	return v;
}

//void egsl_v2a(val v, double*vec) {
//	gsl_matrix *m = egsl_gslm(v);
//	size_t i;
//	for(i=0; i < m->size1; i++)
//		vec[i] = gsl_matrix_get(m,i,0);
//}

//void egsl_v2da(val v, double*a){
//	gsl_matrix *m = egsl_gslm(v);
//	size_t i,j;
//	for(i=0;i<m->size1;i++)
//		for(j=0;j<m->size2;j++)
//			a[j*m->size1 +i] = gsl_matrix_get(m,i,j);
//}

//void egsl_v2vec(val v, gsl_vector*vec) {
//	size_t i;
//	for(i=0;i<vec->size;i++)
//		gsl_vector_set(vec,i, *egsl_atmp(v,i,0));
//}


//val egsl_vFgslv(const gsl_vector*vec){
//	val v = egsl_alloc(vec->size,1);
//	size_t i;
//	for(i=0;i<vec->size;i++)
//		*egsl_atmp(v,i,0) = gsl_vector_get(vec,i);
//	return v;
//}

//val egsl_vFgslm(const gsl_matrix*m){
//	val v = egsl_alloc(m->size1,m->size2);
//	gsl_matrix * m2 = egsl_gslm(v);
//	gsl_matrix_memcpy(m2,m);
//	return v;
//}

gsl_matrix* egsl_v2gslm(val v){
	gsl_matrix * m = egsl_gslm(v); 
	gsl_matrix * m2 = gsl_matrix_alloc(m->rows(),m->cols());
	gsl_matrix_memcpy(m2,m);
	return m2;
}
