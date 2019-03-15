#ifndef H_CALIB_TUPLE
#define H_CALIB_TUPLE

#include <ostream>
#include "calib_tuple.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <json-c/json.h>


struct calib_result {
	double radius_l, radius_r;
	double axle;
	
	/** Laser pose */
	double l[3];
};

struct calib_tuple {
	
/** Input data */

	/** Period */
	double T;
	/** Left and right wheel velocities */
	double phi_l;
	double phi_r;
	/** Scan matching estimate */ 
	double sm[3];
	
/** Temp data, used for outliers computations */
	
	/** Estimated rototranslation based on odometry params. */
	double o[3];
	
	/** Estimated disagreement  sm - est_sm  */
	// double e_sm[3];
	
	double est_sm[3];
	double err_sm[3];
	
	/** Other way to estimate disagreement:   l (+) s  - o (+) l  */
	double err[3];
	
	int mark_as_outlier;
	
    
	/** Computes plenty of statistics, including estimated sm. */
	void compute_disagreement(const calib_result&);
	
	/** Computes fisher information matrix. inf_sm is the inverse of the covariance of sm */
    gsl_matrix* compute_fim(struct calib_result&, gsl_matrix * inf_sm);
	
	void write_as_long_line(std::ostream&os);

    calib_tuple();
    ~calib_tuple();
	
};


bool json2tuple(const JO jo, calib_tuple&tuple);
JO tuple2json(const calib_tuple&tuple);

JO calib_result2json(const calib_result&res) ;

#endif


