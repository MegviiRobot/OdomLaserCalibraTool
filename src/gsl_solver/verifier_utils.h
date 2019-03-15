/*!	
	\file verifier_utils.h
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Header of verifier_utils
*/

#include <stdlib.h>
#include <gsl/gsl_matrix.h>

#include "aux_functions.h"

using namespace std;

/*!
	\brief Computate composition of planar transformations
	@param a \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
	@param b \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
	@param result \f$ a \oplus b  \f$ = \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
*/
void planarComp(double * a, double * b, double * result);

/*!
	\brief Computate inverse composition of planar transformations
	@param a \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
	@param b \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
	@param result \f$ a \ominus b  \f$ = \f$ \left[ \begin{array}{ccc} x & y & \theta \end{array} \right] \f$
*/
void inv_planarComp(double * a, double * b, double * result);

/*!
	\brief Computate homogeneus transformation
	@param M pointer to gsl_matrix to store result
	@param x translation along x axis
	@param y translation along y axis
	@param theta rotation (z axis)
*/
void rototras(gsl_matrix * M, double x, double y, double theta);

