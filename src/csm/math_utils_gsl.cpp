#include "csm/csm_all.h"

//void transform(const gsl_vector* p, const gsl_vector* x, gsl_vector*res) {
//	double theta = gvg(x,2);
//	double c = cos(theta); double s = sin(theta);
//	gsl_vector_set(res, 0, c * gvg(p,0) -s*gvg(p,1) + gvg(x,0));
//	gsl_vector_set(res, 1, s * gvg(p,0) +c*gvg(p,1) + gvg(x,1));
//}
//
//void gsl_vector_set_nan(gsl_vector*v) {
//	gvs(v,0,GSL_NAN);
//	gvs(v,1,GSL_NAN);
//}
//
//double norm(const gsl_vector*a){
//	double x = gvg(a,0);
//	double y = gvg(a,1);
//	return sqrt(x*x+y*y);
//}

gsl_vector * vector_from_array(unsigned int n, double *x) {
	gsl_vector * v = gsl_vector_alloc(n);
	unsigned int i;
	for(i=0;i<n;i++)
		gvs(v,i,x[i]);

	return v;
}

//void copy_from_array(gsl_vector*v, double*x) {
//	size_t i;
//	for(i=0;i<v->size;i++)
//		gsl_vector_set(v,i, x[i]);
//}

void vector_to_array(const gsl_vector*v, double*x){
	int i;
	for(i=0;i<v->size();i++)
		x[i] = gvg(v,i);
}

//void oplus(const gsl_vector*x1,const gsl_vector*x2, gsl_vector*res) {
//	double c = cos(gvg(x1,2));
//	double s = sin(gvg(x1,2));
//	gvs(res,0,  gvg(x1,0)+c*gvg(x2,0)-s*gvg(x2,1));
//	gvs(res,1,  gvg(x1,1)+s*gvg(x2,0)+c*gvg(x2,1));
//	gvs(res,2,  gvg(x1,2)+gvg(x2,2));
//}

void ominus(const gsl_vector*x, gsl_vector*res) {
	double c = cos(gvg(x,2));
	double s = sin(gvg(x,2));
	gvs(res,0,  -c*gvg(x,0)-s*gvg(x,1));
	gvs(res,1,   s*gvg(x,0)-c*gvg(x,1));
	gvs(res,2,  -gvg(x,2));
}

//void pose_diff(const gsl_vector*pose2,const gsl_vector*pose1,gsl_vector*res) {
//	gsl_vector* temp = gsl_vector_alloc(3);
//	ominus(pose1, temp);
//	oplus(temp, pose2, res);
//	gsl_vector_free(temp);
//}

const char* gsl_friendly_pose(gsl_vector*v) {
	return friendly_pose(v->data());
}

//static char egsl_tmp_buf[1024];
//const char* egsl_friendly_pose(val v) {
//	sprintf(egsl_tmp_buf, "(%4.2f mm, %4.2f mm, %4.4f deg)",
//		1000*egsl_atv(v,0),
//		1000*egsl_atv(v,1),
//		rad2deg(egsl_atv(v,2)));
//	return egsl_tmp_buf;
//}
//
//const char* egsl_friendly_cov(val cov) {
//
//	double limit_x  = 2 * sqrt(egsl_atm(cov, 0, 0));
//	double limit_y  = 2 * sqrt(egsl_atm(cov, 1, 1));
//	double limit_th = 2 * sqrt(egsl_atm(cov, 2, 2));
//
//	sprintf(egsl_tmp_buf, "(+- %4.2f mm,+- %4.2f mm,+- %4.4f deg)",
//		1000*limit_x,
//		1000*limit_y,
//		rad2deg(limit_th));
//	return egsl_tmp_buf;
//}


/*double distance(const gsl_vector* a, const gsl_vector* b) {
	distance_counter++;
	double x = gvg(a,0)-gvg(b,0);
	double y = gvg(a,1)-gvg(b,1);
	return sqrt(x*x+y*y);
}

double distance_squared(const gsl_vector* a, const gsl_vector* b) {
	distance_counter++;
	double x = gvg(a,0)-gvg(b,0);
	double y = gvg(a,1)-gvg(b,1);
	return x*x+y*y;
}*/
