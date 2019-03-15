#include <csm/csm_all.h>

#include "calib_tuple.h"
#include "gsl_jacobian.h"

calib_tuple::calib_tuple() { 

}
/*calib_tuple::calib_tuple(const calib_tuple& x) {
//    fim = gsl_matrix_alloc (6,6);
//    gsl_matrix_memcpy(fim, x.fim);
}*/
calib_tuple::~calib_tuple() { 
//    gsl_matrix_free(fim); 
}

gsl_vector * jacobian_helper(const gsl_vector* calib, void*params) {
    calib_tuple * t = (calib_tuple*) params;
    calib_result r;
    r.radius_l = gsl_vector_get(calib, 0);
    r.radius_r = gsl_vector_get(calib, 1);
    r.axle = gsl_vector_get(calib, 2);
    r.l[0] = gsl_vector_get(calib, 3);
    r.l[1] = gsl_vector_get(calib, 4);
    r.l[2] = gsl_vector_get(calib, 5);
    t->compute_disagreement(r);
    
    gsl_vector*res = gsl_vector_alloc(3);
    gsl_vector_set(res, 0, t->est_sm[0]);
    gsl_vector_set(res, 1, t->est_sm[1]);
    gsl_vector_set(res, 2, t->est_sm[2]);
    return res;
}


gsl_matrix* calib_tuple::compute_fim(struct calib_result&r, gsl_matrix * inf_sm) {
    double state[6] = { r.radius_l, r.radius_r, r.axle, r.l[0], r.l[1], r.l[2]};
    gsl_vector_view v = gsl_vector_view_array(state, 6);
    gsl_matrix * J = gsl_jacobian(jacobian_helper, &(v.vector), this);
        
    // XXXXXXXXXX
    // FIM = J' * inf_sm * J
    // A = J' * inf_sm
    gsl_matrix * A = gsl_matrix_alloc(6,3);    
    gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1, J, inf_sm, 0, A);
    // FIM = A * J
    gsl_matrix * fim = gsl_matrix_alloc(6,6); 
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, A, J, 0, fim);
    
    gsl_matrix_free(J);
    gsl_matrix_free(A);
    return fim;
}

void calib_tuple::write_as_long_line(std::ostream&os) {
	os 
		<< T << "   "
		<< phi_l << "   "
		<< phi_r << "   "
		<< sm[0] << " "
		<< sm[1] << " "
		<< sm[2] << "   "
		<< o[0] << " "
		<< o[1] << " "
		<< o[2] << "   "
		<< err[0] << " "
		<< err[1] << " "
		<< err[2] << "   "
		<< est_sm[0] << " "
		<< est_sm[1] << " "
		<< est_sm[2] << "   "
		<< err_sm[0] << " "
		<< err_sm[1] << " "
		<< err_sm[2] << " "
		<< mark_as_outlier << 
		std::endl;
}

void calib_tuple::compute_disagreement(const struct calib_result&res) {
	double J11 = +res.radius_l/2;
	double J12 = +res.radius_r/2;
	double J21 = -res.radius_l/res.axle;
	double J22 = +res.radius_r/res.axle;
	
	static int once = 1;
//	if(once)
//	cout << "My J21,J22 "<< J21 << ", " << J22 << endl;
	once = 0;

	double speed = J11 * this->phi_l + J12 * this->phi_r;
	double omega = J21 * this->phi_l + J22 * this->phi_r;
	
	double o_theta  = T * omega;
		
	double t1,t2;	
	if ( fabs(o_theta) > 1e-12 ) {
		t1 = (   sin(o_theta)   / (o_theta) );
		t2 = ( (1-cos(o_theta)) / (o_theta) );	
	} else {
		t1 = 1;
		t2 = 0;
	}		
	
	this->o[0] = t1 * (speed*T);
	this->o[1] = t2 * (speed*T);
	this->o[2] = o_theta;
	
	double l_plus_s[3];
	double o_plus_l[3];
	oplus_d(res.l, this->sm, l_plus_s);
	oplus_d(o, res.l, o_plus_l);
	
	for(int i=0;i<3;i++)
		this->err[i] = l_plus_s[i] - o_plus_l[i];
	
	pose_diff_d(o_plus_l, res.l, this->est_sm);

	for(int i=0;i<3;i++)
		this->err_sm[i] = this->est_sm[i] - this->sm[i];

}

bool json2tuple(const JO jo, calib_tuple&tuple) {
	int res = 
		jo_read_double(jo, "T", &tuple.T) &&
		jo_read_double(jo, "phi_l", &tuple.phi_l) &&
		jo_read_double(jo, "phi_r", &tuple.phi_r) &&
		jo_read_double_array (jo, "sm", tuple.sm, 3, GSL_NAN);
		
	if(!res)
		sm_error("Cannot read tuple form JSON.\n");
	return res;
}

JO calib_result2json(const calib_result&res) {
	// Build json object
	struct json_object *new_obj;
	new_obj = json_object_new_object();		
	
	double est_E_d 	= res.radius_r / res.radius_l; 
	/** XXX ??? */
	double est_E_b 	= res.axle / 0.09;
	json_object_object_add(new_obj, "axle", json_object_new_double(res.axle) );
	json_object_object_add(new_obj, "l_diam", json_object_new_double(res.radius_l*2) );
	json_object_object_add(new_obj, "r_diam", json_object_new_double(res.radius_r*2) );
	json_object_object_add(new_obj, "l_x", json_object_new_double(res.l[0]) );
	json_object_object_add(new_obj, "l_y", json_object_new_double(res.l[1]) );
	json_object_object_add(new_obj, "l_theta", json_object_new_double(res.l[2]) );
	json_object_object_add(new_obj, "E_d", json_object_new_double(est_E_d) );
	json_object_object_add(new_obj, "E_b", json_object_new_double(est_E_b) );
	
	return new_obj;
}



