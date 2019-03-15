#ifndef H_EASY_GSL
#define H_EASY_GSL

//#include <gsl/gsl_vector.h>
//#include <gsl/gsl_matrix.h>

#include <gsl_eigen/gsl_eigen.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

struct egsl_val {
	gsl_matrix * gslm;
	int cid : 16;
	int index : 16;
};

typedef struct egsl_val val;

/* Core functions */

/* Push a new context. */
void egsl_push();
void egsl_push_named(const char*name);
/* Pops a context */
void egsl_pop();
void egsl_pop_named(const char*name);
void egsl_free(void);

double* egsl_atmp(val v, size_t i, size_t j);
val egsl_alloc(size_t rows, size_t columns);
val egsl_alloc_in_context(int cid, size_t rows, size_t cols); //used by egsl_promote
gsl_matrix * egsl_gslm(val v);
/** Creates a copy of v in the previous context.*/
val egsl_promote(val v);

/** Operations among values */
val egsl_scale(double, val);
val egsl_sum(val, val);
//val egsl_sum3(val, val, val);
val egsl_mult(val, val);
val egsl_transpose(val);
val egsl_inverse(val);
val egsl_sub(val,val);
val egsl_sum(val v1,val v2);
val egsl_compose_col(val v1, val v2);
val egsl_compose_row(val v1, val v2);
void egsl_add_to(val v1, val v2);
void egsl_add_to_col(val v1, size_t j, val v2);

double egsl_norm(val);

//void egsl_symm_eig(val v, double* eigenvalues, val* eigenvectors);

double egsl_atv(val, size_t i);
//double egsl_atm(val, size_t i, size_t j);

/* File: egsl_conversions.c
  Conversions */

val egsl_vFa(size_t rows, const double*);
val egsl_vFda(size_t rows, size_t columns, const double*);

/** Copies a VECTOR value into array */
//void egsl_v2a(val, double*);
/** Copies a MATRIX value into array (row1 .. rown) */
//void egsl_v2da(val, double*);
/** Copies a vector value into a gsl_vector */
//void egsl_v2vec(val, gsl_vector*);

//val egsl_vFgslv(const gsl_vector*);
//val egsl_vFgslm(const gsl_matrix*);

gsl_matrix* egsl_v2gslm(val);

/*/ File: egsl_misc.c
    Miscellaneous useful matrixes. */
val egsl_zeros(size_t rows, size_t columns);
val egsl_ones(size_t rows, size_t columns);
val egsl_vers(double theta);
val egsl_rot(double theta);


/* Misc */
void egsl_print(const char*str, val);
/** Prints eigenvalues and eigenvectors of a symmetric matrix */
//void egsl_print_spectrum(const char*s, val v);
void egsl_print_stats(void);


	
/** Private implementations things */
void egsl_expect_size(val v, size_t rows, size_t cols);
void egsl_error(void);


void egsl_free_unused_memory();


//#ifdef __cplusplus
//}
//#endif

#endif
