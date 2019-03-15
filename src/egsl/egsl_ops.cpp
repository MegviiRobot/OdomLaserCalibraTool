//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_blas.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_eigen.h>

#include "egsl/egsl.h"



val egsl_sub(val v1,val v2){
	return egsl_sum(v1, egsl_scale(-1.0,v2));
}

val egsl_compose_col(val v1, val v2){
	gsl_matrix *m1 = egsl_gslm(v1);
	gsl_matrix *m2 = egsl_gslm(v2);
	egsl_expect_size(v2, 0, m1->cols());
	val v3 = egsl_alloc(m1->rows()+m2->rows(),m1->cols());
	gsl_matrix *m3 = egsl_gslm(v3);
	int i,j;
	for(j=0;j<m1->cols();j++) {
		for(i=0;i<m1->rows();i++)
			gsl_matrix_set(m3, i, j, gsl_matrix_get(m1,i,j));
		
		for(i=0;i<m2->rows();i++)
			gsl_matrix_set(m3, m1->rows()+i, j, gsl_matrix_get(m2,i,j));
	}
	return v3;
}

val egsl_compose_row(val v1, val v2){
	gsl_matrix *m1 = egsl_gslm(v1);
	gsl_matrix *m2 = egsl_gslm(v2);
	egsl_expect_size(v2, m1->rows(), 0);
	val v3 = egsl_alloc(m1->rows(), m1->cols() + m2->cols());
	gsl_matrix *m3 = egsl_gslm(v3);
	int i,j;
	for(i=0;i<m1->rows();i++) {
		for(j=0;j<m1->cols();j++)
			gsl_matrix_set(m3, i, j, gsl_matrix_get(m1,i,j));
		
		for(j=0;j<m2->cols();j++)
			gsl_matrix_set(m3, i, m1->cols()+j, gsl_matrix_get(m2,i,j));
	}
	return v3;
}

void egsl_add_to(val v1, val v2) {
	gsl_matrix * m1 = egsl_gslm(v1);
	gsl_matrix * m2 = egsl_gslm(v2);
	gsl_matrix_add(m1,m2);
}

void egsl_add_to_col(val v1, size_t j, val v2) {
/*	egsl_print("m1",v1);
	egsl_print("m2",v2); */
	gsl_matrix * m1 = egsl_gslm(v1);
	gsl_matrix * m2 = egsl_gslm(v2);

/*	printf("m1 size = %d,%d j = %d\n",m1->rows(),m1->cols(),j); */
	egsl_expect_size(v2, m1->rows(), 1);
	int i;
	for(i=0;i<m1->rows();i++) {
		*gsl_matrix_ptr(m1, i, j) += gsl_matrix_get(m2,i,0);
	}
}


val egsl_copy_val(val v1) {
	gsl_matrix * m1 = egsl_gslm(v1);
	val v2 = egsl_alloc(m1->rows(),m1->cols());
	gsl_matrix * m2 = egsl_gslm(v2);
	gsl_matrix_memcpy(m2,m1);
	return v2;
}

val egsl_scale(double s, val v1){
	val v2 = egsl_copy_val(v1);
	gsl_matrix * m2 = egsl_gslm(v2);
	gsl_matrix_scale(m2, s);
	return v2;
}

val egsl_sum(val v1, val v2){
	gsl_matrix * m1 = egsl_gslm(v1);
	gsl_matrix * m2 = egsl_gslm(v2);
	val v3 = egsl_alloc(m1->rows(),m1->cols());
	gsl_matrix * m3 = egsl_gslm(v3);
	gsl_matrix_memcpy(m3,m1);
	gsl_matrix_add(m3,m2);
	return v3;
}

//val egsl_sum3(val v1, val v2, val v3){
//	return egsl_sum(v1, egsl_sum(v2,v3));
//}

val egsl_mult(val v1, val v2){
	gsl_matrix * a = egsl_gslm(v1);
	gsl_matrix * b = egsl_gslm(v2);
	val v = egsl_alloc(a->rows(),b->cols());
	gsl_matrix * ab = egsl_gslm(v); 

	//gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,a,b,0.0,ab);
	*ab = *a * *b;

	return v;

}

val egsl_transpose(val v1){
	gsl_matrix * m1 = egsl_gslm(v1);
	val v2 = egsl_alloc(m1->cols(),m1->rows());
	gsl_matrix * m2 = egsl_gslm(v2);

	*m2 = m1->transpose();
	//gsl_matrix_transpose_memcpy(m2,m1);

	return v2;
}

val egsl_inverse(val v1){
	gsl_matrix*A = egsl_gslm(v1);
	val v2 = egsl_alloc(A->rows(),A->rows());
	gsl_matrix*invA = egsl_gslm(v2);


//	size_t n = A->rows();
//	gsl_matrix * m = gsl_matrix_alloc(n,n);
//
//	gsl_matrix_memcpy(m,A);
//	gsl_permutation * perm = gsl_permutation_alloc (n);
//	/* Make LU decomposition of matrix m */
//	int s;
//	gsl_linalg_LU_decomp (m, perm, &s);
//	/* Invert the matrix m */
//	gsl_linalg_LU_invert (m, perm, invA);
//	gsl_permutation_free(perm);
//	gsl_matrix_free(m);

	*invA = A->inverse();

	return v2;
}

//void egsl_symm_eig(val v, double* eigenvalues, val* eigenvectors) {
//	gsl_matrix *m = egsl_gslm(v);
//	size_t N = m->rows();
//	/* Check for v to be square */
//
//	gsl_matrix *A = gsl_matrix_alloc(N,N);
//	gsl_matrix_memcpy(A, m);
//
//	gsl_vector *eval = gsl_vector_alloc(N);
//	gsl_matrix *evec = gsl_matrix_alloc(N,N);
//
//	gsl_eigen_symmv_workspace * ws = gsl_eigen_symmv_alloc(N);
//	gsl_eigen_symmv(A, eval, evec, ws);
//	gsl_eigen_symmv_free(ws);
//
//
//	gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_VAL_DESC);
//
//	size_t j;
//	for(j=0;j<N;j++) {
//		eigenvalues[j] = gsl_vector_get(eval, j);
//		eigenvectors[j] = egsl_alloc(N,1);
//		size_t i;
//		for(i=0;i<N;i++)
//			*egsl_atmp(eigenvectors[j],i,0) = gsl_matrix_get(evec,i,j);
//	}
//
//
//	gsl_vector_free(eval);
//	gsl_matrix_free(evec);
//	gsl_matrix_free(A);
//}








