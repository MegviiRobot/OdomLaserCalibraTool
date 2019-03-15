/*!	
	\file solver_utils.h
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Header of Solver_utils
*/

#include <vector>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_multimin.h>

#include "aux_functions.h"

using namespace std;

/*!
	\brief calculates condition number for a matrix
	@param M gsl_matrix pointer
	@return condition number of matrix M
*/
double cond_number(gsl_matrix * M);

/*!
	\brief Solves linear system \f$ Ax=g \f$, where A is a square matrix \f$ ( x = inv(A)g ) \f$
	@param A gsl_matrix pointer
	@param g gsl_vector pointer
	@param x gsl_vector pointer to store result
	@return 1 if succesfull, 0 otherwhise
*/
int solve_square_linear(gsl_matrix * A, gsl_vector * g, gsl_vector * x);
/*! 
	Compute eigenvalues and eigenvectors of a symmetric matrix M 
*/
/*!
	\brief Calculates eigenvalues and eigenvectors (sorted in ascending order) of a symmetric matrix
	@param M gsl_matrix pointer
	@param eigenvalues gsl_vector pointer
	@param eigenvectors gsl_vector pointer to store eigenvectors
	@return 1 if succesfull, 0 otherwise
*/
int eigenv(gsl_matrix * M, gsl_vector * eigenvalues, gsl_matrix * eigenvectors);

/*!
	\brief Calculates values of a quadratic function \f$ x'Mx \f$ for a given x
	@param x gsl_vector pointer
	@param M gsl_matrix pointer
	@return value of function
*/
double calculate_error(gsl_vector * x, gsl_matrix * M);

/*!
	\brief Calculate x solution of \f$ (M + \lambda W)x = 0 \f$; \lambda such that \f$ (M + \lambda W) \f$ is singular
	@param M gsl_matrix pointer
	@param lambda gsl_vector pointer
	@param W gsl_matrix pointer
	@return pointer to gsl_vector storing solution
	\note Caller must deallocate returned pointer
*/
gsl_vector * x_given_lambda(gsl_matrix * M, double lambda, gsl_matrix * W);

/*!
	\brief Solves \f$ (M + \lambda W)x = 0 \f$, M is a square matrix, \f$ W = \left[ \begin{array}{cc} 0_{3x3} & 0_{3x2} \\ 0_{2x3} & I_{2x2} \end{array} \right] \f$
	@param M gsl_matrix pointer
	@return pointer to gsl_vector storing solution
	\note Caller must deallocate returned pointer
*/
gsl_vector * full_calibration_min(gsl_matrix * M);

/*!
	\brief Numerically search minimum for a function \f$ x'Qx \f$, starting from a specified point, using symplex method
	@param init initial point for minimization
	@param min_found pointer to gsl_vector to store result
	@param Q pointer to gsl_matrix
	@param double(*f) is a pointer to function
	@return 1 if successfull, 0 otherwhise
*/
int fminimize_simplex(gsl_vector * init, gsl_vector * min_found, gsl_matrix * Q, double (*f) (const gsl_vector *z, void *params) );

/*!
	\brief Uses symplex method to estimate parameters
	@param H pointer to matrix of quadratic function (\f$ x'Hx \f$)
	@return pointer to gsl_vector storing minimum
*/
gsl_vector * numeric_calibration(gsl_matrix * H);

/*!
	\brief Implements a quadratic function (used in minimization functions)
	@param z pointer to point where the function has to be evaluated
	@param params pointer to parameters vector
*/
double f (const gsl_vector *z, void *params);


