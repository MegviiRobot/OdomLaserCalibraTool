#ifndef H_MATH_UTILS_GSL
#define H_MATH_UTILS_GSL

//#include <gsl/gsl_math.h>
#include "egsl/egsl.h"

#include "laser_data.h"

#define gvg gsl_vector_get
#define gvs gsl_vector_set


/* GSL stuff */
	const char* gsl_friendly_pose(gsl_vector*v);
	gsl_vector * vector_from_array(unsigned int n, double *x);
	void vector_to_array(const gsl_vector*v, double*);
	//void copy_from_array(gsl_vector*v, double*);

	//void oplus(const gsl_vector*x1,const gsl_vector*x2, gsl_vector*res);
	void ominus(const gsl_vector*x, gsl_vector*res);
	//void pose_diff(const gsl_vector*pose2,const gsl_vector*pose1,gsl_vector*res);

	//void transform(const gsl_vector* point2d, const gsl_vector* pose, gsl_vector*result2d);
	//void gsl_vector_set_nan(gsl_vector*v);

	//double distance(const gsl_vector* a,const gsl_vector* b);
	//double distance_squared(const gsl_vector* a,const gsl_vector* b);

	/** Returns norm of 2D point p */
	//double norm(const gsl_vector*p);
	//const char* egsl_friendly_pose(val pose);
	//const char* egsl_friendly_cov(val cov);

/** Returns Fisher's information matrix. You still have to multiply
    it by (1/sigma^2). */
//val ld_fisher0(LDP ld);


#endif


