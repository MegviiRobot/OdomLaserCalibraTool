/*
 * gsl_eigen.h
 *
 *  Created on: Oct 11, 2012
 *      Author: sprunkc
 */

#ifndef GSL_EIGEN_H_
#define GSL_EIGEN_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef  Eigen::VectorXd gsl_vector;
typedef  Eigen::MatrixXd gsl_matrix;

inline gsl_vector *gsl_vector_alloc (const size_t n){
  return new Eigen::VectorXd(n);
}

inline void gsl_vector_free (gsl_vector * v){
  delete v;
}

inline void gsl_vector_memcpy (gsl_vector * dest, const gsl_vector * src){
  *dest = *src;
}


inline double gsl_vector_get (const gsl_vector * v, const size_t i){
  return (*v)(i);
}

inline void gsl_vector_set (gsl_vector * v, const size_t i, double x){
  (*v)(i) = x;
}


inline gsl_matrix* gsl_matrix_alloc(const size_t rows, const size_t cols){
  return new Eigen::MatrixXd(rows, cols);
}

inline void gsl_matrix_free(gsl_matrix* m){
  delete m;
}

inline void gsl_matrix_memcpy(gsl_matrix* m2, const gsl_matrix* m){
  *m2 = *m;
}

inline void gsl_matrix_set(gsl_matrix * m, const size_t i, const size_t j, const double x){
  (*m)(i,j) = x;
}

inline void gsl_matrix_set_zero (gsl_matrix * m){
  m->setZero();
}


inline double gsl_matrix_get(const gsl_matrix * m, const size_t i, const size_t j){
  return (*m)(i,j);
}

inline double * gsl_matrix_ptr(gsl_matrix * m, const size_t i, const size_t j){
  return &(*m)(i,j);
}

inline void gsl_matrix_set_all (gsl_matrix * m, double x){
  m->setConstant(x);
}

inline void gsl_matrix_add (gsl_matrix * a, const gsl_matrix * b){
  *a += *b;
}

inline void gsl_matrix_scale (gsl_matrix * a, const double x){
  *a *=x;
}

#endif /* GSL_EIGEN_H_ */
