/*!	
	\file solver_utils.cpp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Implements methods for Solver_utils
*/
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>

#include <gsl/gsl_poly.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#include <options/options.h>
#include <json-c/json.h>
#include <csm/csm_all.h>
#include "aux_functions.h"
#include "solver_utils.h"

#define sp(X)		( pow(X,2) )
#define cp(X)		( pow(X,3) )
#define fp(X)		( pow(X,4) )
#define vg(X,Y) 	( gsl_vector_get(X,Y) )
#define vs(X,Y,Z) 	( gsl_vector_set(X,Y,Z) )
#define mg(X,Y,Z) 	( gsl_matrix_get(X,Y,Z) )
#define ms(X,Y,Z,Q) 	( gsl_matrix_set(X,Y,Z,Q) )

using namespace std;

double cond_number(gsl_matrix * M) 
{
	size_t row = M->size1;
	size_t col = M->size2;
	gsl_matrix * V 		= gsl_matrix_alloc(col,col);
	gsl_matrix * U 		= gsl_matrix_alloc(row,col);
	gsl_matrix_memcpy(U, M);

	//vettore per i valori singolari di A
	gsl_vector * S 	  = gsl_vector_alloc( min(row,col) ); 
	gsl_vector * work = gsl_vector_alloc( min(row,col) );
	gsl_linalg_SV_decomp(U, V, S, work );

	double cond = gsl_vector_max(S) / gsl_vector_min(S);

	gsl_matrix_free(U);
	gsl_matrix_free(V);
	gsl_vector_free(work);
	gsl_vector_free(S);
	return cond;
}

int solve_square_linear(gsl_matrix * A, gsl_vector * g, gsl_vector * x) 
{
	size_t sz = A->size1;
	if (A->size2 != sz) return 0;
	if (x->size  != sz) return 0;
	if (g->size  != sz) return 0;
	gsl_matrix * LU 		= gsl_matrix_alloc(sz,sz);
	gsl_permutation * p = gsl_permutation_alloc(sz);
	gsl_matrix_memcpy(LU, A);
	int signum;
	gsl_linalg_LU_decomp (LU, p, &signum );
	gsl_linalg_LU_solve  (LU, p, g, x );
	gsl_matrix_free(LU);
	gsl_permutation_free(p);

	return 1;
}

int eigenv(gsl_matrix * M, gsl_vector * eigenvalues, gsl_matrix * eigenvectors) 
{
	size_t sz2 = M->size2;
	size_t sz  = M->size1;
	if (sz != sz2) return 0;
	gsl_eigen_symmv_workspace * wk = gsl_eigen_symmv_alloc(sz);
	gsl_matrix *M_tmp = gsl_matrix_alloc(sz, sz);
	
	gsl_matrix_memcpy(M_tmp, M);
	gsl_eigen_symmv(M_tmp, eigenvalues, eigenvectors, wk);	
	gsl_eigen_gensymmv_sort (eigenvalues, eigenvectors, GSL_EIGEN_SORT_ABS_ASC);
	
	gsl_matrix_free(M_tmp);
	gsl_eigen_symmv_free(wk);
	return 1;
}

double calculate_error(gsl_vector * x, gsl_matrix * M) 
{
	double error;
	gsl_vector * tmp = gsl_vector_calloc( x->size );
	
	gsl_blas_dgemv(CblasNoTrans, 1, M, x, 0, tmp);
	gsl_blas_ddot(x ,tmp, &error);
	
	gsl_vector_free(tmp);	
	return error;
}

gsl_vector * full_calibration_min(gsl_matrix * M) 
{
	double 		m11 = mg(M,0,0);
	double		m13 = mg(M,0,2);
	double		m14 = mg(M,0,3);
	double		m15 = mg(M,0,4);
	double		m22 = mg(M,1,1);
	double		m25 = mg(M,1,4);
	double		m34 = mg(M,2,3);
	double		m35 = mg(M,2,4);
	double		m44 = mg(M,3,3); 
	double		m55 = mg(M,4,4);
	double a,b,c;

	/* 	Coefficienti del determinante M + lambda*W 	*/
	a = m11 * sp(m22) - m22 * sp(m13);

	b = 	  2 * m11 * sp(m22) * m44 - sp(m22) * sp(m14) 
		- 2 * m22 * sp(m13) * m44 - 2 * m11 * m22 * sp(m34) 
		- 2 * m11 * m22 * sp(m35) - sp(m22) * sp(m15) 
		+ 2 * m13 * m22 * m34 * m14 + sp(m13) * sp(m34) 
		+ 2 * m13 * m22 * m35 * m15 + sp(m13) * sp(m35);

	c = 	- 2 * m13 * cp(m35) * m15 - m22 * sp(m13) * sp(m44) + m11 * sp(m22) * sp(m44) 
		+ sp(m13) * sp(m35) * m44 + 2 * m13 * m22 * m34 * m14 * m44 + sp(m13) * sp(m34) * m44
		- 2 * m11 * m22 * sp(m34) * m44 - 2 * m13 * cp(m34) * m14 - 2 * m11 * m22 * sp(m35) * m44
		+ 2 * m11 * sp(m35) * sp(m34) + m22 * sp(m14) * sp(m35) - 2 * m13 * sp(m35) * m34 * m14
		- 2 * m13 * sp(m34) * m35 * m15 + m11 * fp(m34) + m22 * sp(m15) * sp(m34)
		+ m22 * sp(m35) * sp(m15) + m11 * fp(m35) - sp(m22) * sp(m14) * m44
		+ 2 * m13 * m22 * m35 * m15 * m44 + m22 * sp(m34) * sp(m14) - sp(m22) * sp(m15) * m44;

	/* 	Calcolo radice del polinomio 	*/
	gsl_complex * r0 = new gsl_complex;
	gsl_complex * r1 = new gsl_complex;

	gsl_poly_complex_solve_quadratic(a, b, c, r0, r1);	
	
	if ( (r0->dat[1] == 0) ) { //|| (GSL_IMAG(r1) == 0)) {	
		/* 	Costruzione matrice W 	*/
		gsl_matrix * W = gsl_matrix_calloc(5,5);
		gsl_matrix_set(W,3,3,1);
		gsl_matrix_set(W,4,4,1);
		gsl_vector * x0 = x_given_lambda(M, r0->dat[0], W);
		gsl_vector * x1 = x_given_lambda(M, r1->dat[0], W);
		gsl_matrix_free(W);
		
		double e0 = calculate_error(x0, M);
		double e1 = calculate_error(x1, M);
	
		return e0 < e1 ? x0 : x1;
	}
	else {
		sm_error("bad thing: imaginary solution\n");
		return NULL;
	}
}

gsl_vector * x_given_lambda(gsl_matrix * M, double lambda, gsl_matrix * W) 
{
	gsl_matrix * Z 	= gsl_matrix_calloc(5,5);
	gsl_matrix * ZZ = gsl_matrix_calloc(5,5);
	
	// Z = M + lamda*W
	gsl_matrix_memcpy(Z, W);
	gsl_matrix_scale (Z, lambda);
	gsl_matrix_add	 (Z, M);
	
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, Z, Z, 0, ZZ); /* 	ZZ = Z * Z' 	*/
	/** Calculate eigenvalues and eigenvectors */
	gsl_vector * eigenvalues  = gsl_vector_alloc(5);
	gsl_matrix * eigenvectors = gsl_matrix_alloc(5,5);
	eigenv(ZZ, eigenvalues, eigenvectors);
	gsl_vector * v0  = gsl_vector_alloc(5);
	gsl_matrix_get_col(v0, eigenvectors, gsl_vector_min_index(eigenvalues) );
		
	/** Conditions: x1 > 0; x4^2 + x5^2 = 1 */
	gsl_vector * tmp_v = gsl_vector_alloc(2);
	vs(tmp_v ,0, vg(v0, 3));
	vs(tmp_v ,1, vg(v0, 4));
	double norm  = gsl_blas_dnrm2(tmp_v);
	double coeff = ( GSL_SIGN( vg(v0,0) ) / norm );
	gsl_vector_scale(v0 ,coeff);
	
	gsl_matrix_free(Z);
	gsl_matrix_free(ZZ);
	gsl_matrix_free(eigenvectors);
	gsl_vector_free(eigenvalues);
	gsl_vector_free(tmp_v);
	return v0;
}

int fminimize_simplex(gsl_vector * init, gsl_vector * min_found, gsl_matrix * Q, double (*f) (const gsl_vector *z, void *params) ) 
{
	int sz = (int) init->size;
	if ( (Q->size1 != sz) || (Q->size2 != sz) ) return 0;
	const gsl_multimin_fminimizer_type * T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer * s = gsl_multimin_fminimizer_alloc (T, sz);
	
	void *par = Q->data;
 	gsl_multimin_function func;
 	func.n = sz;
 	func.f = f;
	func.params = par;
	gsl_vector *step = gsl_vector_alloc(sz); 
	gsl_vector_set_all(step, 1e-12);
	
	int iter = 0;
	double size;
	int status;
	gsl_multimin_fminimizer_set (s, &func, init, step);
	do {
		iter++;
		status = gsl_multimin_fminimizer_iterate (s);
		if (status) break;

		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size(size, 1e-8);
		
		if (status == GSL_SUCCESS) {
			sm_debug("Minimum found at:\n");
			for (int i = 0; i < sz; i++)
				sm_debug("%f \n", vg(s->x, i));
		}
	} while (status == GSL_CONTINUE && iter < 2000);
	
	if (status != GSL_SUCCESS) cout << "Symplex method: Maximum iteration reached before the minimum is found\n";
	gsl_vector_memcpy( min_found, gsl_multimin_fminimizer_x(s) );
	gsl_multimin_fminimizer_free (s);
	return 1;
}

gsl_vector * numeric_calibration(gsl_matrix * H) {
	int sz = (int)(H->size1);
	gsl_vector * eigenvalues  = gsl_vector_alloc(sz);
	gsl_matrix * eigenvectors = gsl_matrix_alloc(sz,sz);
	eigenv(H, eigenvalues, eigenvectors);
	
	gsl_vector * v0 = gsl_vector_alloc(sz);
	gsl_matrix_get_col(v0, eigenvectors, sz-5);	/*Starting point for numerical search*/
	
	gsl_vector * tmp_v = gsl_vector_alloc(2);
	vs(tmp_v , 0, vg(v0, sz-2));
	vs(tmp_v , 1, vg(v0, sz-1));
	double norm  = gsl_blas_dnrm2(tmp_v);
	double coeff = ( GSL_SIGN( vg(v0,0) ) / norm );
	gsl_vector_scale(v0 ,coeff);
	gsl_vector_free(tmp_v);

	gsl_vector * min = gsl_vector_alloc(sz);	/*vector where minimum will be stored*/
	fminimize_simplex(v0, min, H, &f);
		
	return min;
}

/**
	Function f(z) = [z(1) z(2) ... z(n)]' * M * [z(1) z(2) ... z(n)].
	Used for minimization in solver routine 
*/
double f (const gsl_vector *z, void *params)	{
	int sz = (int) z->size;
	gsl_vector * v = gsl_vector_alloc(sz);
	gsl_matrix * M = gsl_matrix_alloc(sz, sz);
	gsl_vector_memcpy(v, z);
	
	double *p = (double *)params;
	M->data = p;
	double * res = new double;
	gsl_vector * tmp = gsl_vector_calloc(sz);
	gsl_blas_dgemv(CblasNoTrans, 1, M, v, 0, tmp);
	gsl_blas_ddot(v ,tmp, res);
	gsl_vector_free(tmp);	
 
  	return( res[0] );
}


