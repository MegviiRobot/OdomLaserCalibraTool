/* GPC: A library for the solution of General Point Correspondence problems.
  Copyright (C) 2006 Andrea Censi (andrea at censi dot org)
*/

#include <math.h>
//#include <gsl/gsl_math.h>
//#include <gsl/gsl_matrix.h>
#include "gpc/gpc.h"
#include "gpc/gpc_utils.h"
#include <gsl_eigen/gsl_eigen.h>
#include <stdio.h>
#include <vector>
using namespace std;


//original version
/* Note that we use static values here so we don't need to evaluate that all the time */
//#define M(matrix, rows, col) static gsl_matrix*matrix=0; if(!matrix) matrix = gsl_matrix_alloc(rows,col);
//#define MF(matrix) /*gsl_matrix_free(matrix)*/

//version to free the memory, slower
//#define M(matrix, rows, col) static gsl_matrix*matrix=0; if(!matrix) matrix = gsl_matrix_alloc(rows,col);
//#define MF(matrix) gsl_matrix_free(matrix); matrix=0;

//New version: have static Eigen variables and use pointers to these for compatibility
#define TAGME(name, tag) name##tag
#define M(matrix, rows, col) static Eigen::MatrixXd TAGME(matrix,_mem)(rows, col); static gsl_matrix* matrix=&TAGME(matrix,_mem);
#define MF(matrix) (void)(matrix)


int gpc_solve(int K, const std::vector<gpc_corr>& c,
	const double*x0, const double *cov_x0,
	double *x_out)
{
  (void) x0; (void) cov_x0;
  // 构建公式 19 开始的那些矩阵
	M(bigM,    4,4); M(g,  4,1); M(bigM_k,2,4);
	M(bigM_k_t,4,2); M(C_k,2,2); M(q_k,   2,1);
	M(temp41,  4,1); M(temp22,2,2);	M(temp22b,2,2);
	M(temp42,  4,2); M(temp44,4,4);	M(temp21, 2,1);
	M(temp22c, 2,2); M(temp12,1,2);

	gsl_matrix_set_zero(bigM);
	gsl_matrix_set_zero(g);
	gsl_matrix_set_zero(temp42);

	double d_bigM[4][4] = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
	double d_g[4] = {0, 0, 0, 0};
	int k;
	for(k=0;k<K;k++) {
		if(!c[k].valid) continue;

		double C00 = c[k].C[0][0];
		double C01 = c[k].C[0][1];
		double C10 = c[k].C[1][0];
		double C11 = c[k].C[1][1];

		if(C01 != C10) {
			fprintf(stderr, "k=%d; I expect C to be a symmetric matrix.\n", k);
			return 0;
		}
#if GPC_CHECK_SEMIDEF
		double det = C00*C11 - C01*C10;
		double trace = C00 + C11;
		int is_semidef_pos = (det >= 0) && (trace>0);
		if(!is_semidef_pos) {
			fprintf(stderr, "k=%d; I expect the matrix to be semidef positive (det>=0 and trace>0), det = %.20f trace= %.10f C = [%.15f,%.15f;%.15f %.15f]\n",
				k, det, trace,	C00,C01,C10,C11);
			return 0;
		}
#endif

		double qx = c[k].q[0];
		double qy = c[k].q[1];
		double px = c[k].p[0];
		double py = c[k].p[1];

		/* [ C00,  c01,  px C00 + c01 py , c01 px - py C00 ] */
		d_bigM[0][0] += C00;
		d_bigM[0][1] += C01;
		d_bigM[0][2] += +px * C00 + py * C01;
		d_bigM[0][3] += -py * C00 + px * C01;

		/*  [ C10 ,  C11 , py C11 + px C10 , px C11 - py C10 ] */
		d_bigM[1][0] += C10;
		d_bigM[1][1] += C11;
		d_bigM[1][2] += +px * C10 + py * C11;
		d_bigM[1][3] += +px * C11 - py * C10;

		/*Col 1 = [ py C10 + px C00 ]
		 Col 2 = [ py C11 + c01 px ]
		 Col 3 = [ py (py C11 + px C10) + px (px C00 + c01 py) ]
		 Col 4 = [ py (px C11 - py C10) + px (c01 px - py C00) ]
		*/
		d_bigM[2][0] += px * C00 + py * C10;
		d_bigM[2][1] += px * C01 + py * C11;
		d_bigM[2][2] += (px*px)*(+C00) + (px*py)*(+C10+C01) + (py*py)*(+C11);
		d_bigM[2][3] += (px*px)*(+C01) + (px*py)*(-C00+C11) + (py*py)*(-C10);

		/*Col 1 = [ px C10 - py C00 ]
		  Col 2 = [ px C11 - c01 py ]
		 Col 3 = [ px (py C11 + px C10) - py (px C00 + c01 py) ]
		 Col 4 = [ px (px C11 - py C10) - py (c01 px - py C00) ]*/
		d_bigM[3][0] += -py * C00 + px * C10;
		d_bigM[3][1] += -py * C01 + px * C11;
		d_bigM[3][2] += (px*px)*(+C10) + (px*py)*(-C00+C11) + (py*py)*(-C01);
		d_bigM[3][3] += (px*px)*(+C11) + (px*py)*(-C10-C01) + (py*py)*(+C00);

		d_g[0] += C00*qx+C10*qy;
		d_g[1] += C01*qx+C11*qy;
		d_g[2] += qx * (C00*px+C01*py) + qy * (C10*px+C11*py);
		d_g[3] += qx * (C00*(-py)+C01*px) + qy * (C10*(-py)+C11*px);

	}

	{
	unsigned int a,b;
	 for(a=0;a<4;a++)
			*gsl_matrix_ptr(g, a, 0) = -2 * d_g[a];
	 for(a=0;a<4;a++)
		for(b=0;b<4;b++)
		gsl_matrix_set(bigM, a, b,  2 * d_bigM[a][b]);
	}

	// 公式 26 开始的那些矩阵
	M(mA,2,2); gms(mA,0,0, gmg(bigM,0,0)); gms(mA,0,1, gmg(bigM,0,1));
	           gms(mA,1,0, gmg(bigM,1,0)); gms(mA,1,1, gmg(bigM,1,1));
	M(mB,2,2); gms(mB,0,0, gmg(bigM,0,2)); gms(mB,0,1, gmg(bigM,0,3));
	           gms(mB,1,0, gmg(bigM,1,2)); gms(mB,1,1, gmg(bigM,1,3));
	M(mD,2,2); gms(mD,0,0, gmg(bigM,2,2)); gms(mD,0,1, gmg(bigM,2,3));
	           gms(mD,1,0, gmg(bigM,3,2)); gms(mD,1,1, gmg(bigM,3,3));

	M(mS,2,2); M(mSa,2,2);


	/*	mS = mD - mB.trans * mA.inv * mB;
	  temp22b = inv(A) */
	m_inv(mA, temp22b);
	/* temp22c = inv(A) * mB           */
	m_mult(temp22b, mB, temp22c);
	/* temp22 = mB'               */
	m_trans(mB, temp22);
	m_mult(temp22, temp22c, temp22b);
	m_scale(-1.0,temp22b);
	m_add(mD,temp22b,mS);

	/* mSa = mS.inv * mS.det; */
	m_inv(mS, mSa);
	m_scale(m_det(mS), mSa);

	if(TRACE_ALGO) {
		m_display("mA",mA);
		m_display("mB",mB);
		m_display("mD",mD);
		m_display("mS",mS);
		m_display("mSa",mSa);
	}

	M(g1,2,1);	M(g2,2,1);
	M(g1t,1,2);	M(g2t,1,2);
	M(mAi,2,2);	M(mBt,2,2);

	gms(g1,0,0,gmg(g,0,0));
	gms(g1,1,0,gmg(g,1,0));
	gms(g2,0,0,gmg(g,2,0));
	gms(g2,1,0,gmg(g,3,0));
	m_trans(g1, g1t);
	m_trans(g2, g2t);
	m_trans(mB, mBt);
	m_inv(mA, mAi);

	M(m1t,1,2);	M(m1,2,1);
	M(m2t,1,2);	M(m2,2,1);
	M(m3t,1,2);	M(m3,2,1);

	/* m1t = g1t*mAi*mB */
	m_mult(g1t,mAi,temp12);
	m_mult(temp12,mB,m1t);

	m_trans(m1t,m1);
	/*     m2t = m1t*mSa    */
	m_mult(m1t,mSa,m2t);
	m_trans(m2t,m2);
	/* m3t = g2t*mSa     */
	m_mult(g2t,mSa,m3t);
	m_trans(m3t,m3);

	double p[3] = {
		  m_dot(m2t,m2) - 2*m_dot(m2t,m3) +   m_dot(m3t,m3),
		4*m_dot(m2t,m1) - 8*m_dot(m2t,g2) + 4*m_dot(g2t,m3),
		4*m_dot(m1t,m1) - 8*m_dot(m1t,g2) + 4*m_dot(g2t,g2)};

	double l[3] = {m_det(mS), 2*gmg(mS,0,0)+2*gmg(mS,1,1), 4};

	/* q = p - l^2        */
	double q[5] = {p[0]-(l[0]*l[0]), p[1]-(2*l[1]*l[0]),
		p[2]-(l[1]*l[1]+2*l[0]*l[2]), -(2*l[2]*l[1]), -(l[2]*l[2])};

	if(TRACE_ALGO) {
		fprintf(stderr, "p = %f %f %f \n", p[2], p[1], p[0]);
		fprintf(stderr, "l = %f %f %f \n", l[2], l[1], l[0]);
		fprintf(stderr, "q = %f %f %f %f %f \n", q[4],  q[3],  q[2], q[1], q[0]);
	}

	/*
	double lambdas[4];
	if(!poly_real_roots(5, q, lambdas)) {
		fprintf(stderr, "Cannot solve polynomial.\n");
		return 0;
	}

	double lambdas_error[4];
	double lambdas_pose[4][3];

	for(int i=0;i<4;i++) {
		double lambda = lambdas[i];

		if(TRACE_ALGO) {
			fprintf(stderr, "lambda = %f \n", lambda);
		}

		M(W,4,4); gsl_matrix_set_zero(W); gms(W,2,2,1.0); gms(W,3,3,1.0);
		M(x,4,1);

		m_scale(2*lambda, W);
		gsl_matrix_add(bigM,W);
		m_inv(bigM, temp44);
		m_mult(temp44, g, x);
		m_scale(-1.0, x);

		lambdas_pose[i][0] = gmg(x,0,0);
		lambdas_pose[i][1] = gmg(x,1,0);
		lambdas_pose[i][2] = atan2(gmg(x,3,0),gmg(x,2,0));

		lambdas_error[i] = gpc_total_error(c, K, lambdas_pose[i]);
	}

	int lowest_error = 0;
	for(int i=0;i<4;i++) {
		printf("#%d lambda=%lf error=%lf\n",i,lambdas[i],lambdas_error[i]);
		if(lambdas_error[i] < lambdas_error[lowest_error])
			lowest_error = i;
	}

	double lr;
	poly_greatest_real_root(5,q,&lr);
	printf("Choose %d: lambda = %lf   bigger real root = %lf \n",lowest_error,lambdas[lowest_error],lr);

	x_out[0]=lambdas_pose[lowest_error][0];
	x_out[1]=lambdas_pose[lowest_error][1];
	x_out[2]=lambdas_pose[lowest_error][2];
	*/

	double lambda;
	if(!poly_greatest_real_root(5, q, &lambda)) return 0;

	M(W,4,4); gsl_matrix_set_zero(W); gms(W,2,2,1.0); gms(W,3,3,1.0);
	M(x,4,1);

	m_scale(2*lambda, W);
	gsl_matrix_add(bigM,W);
	m_inv(bigM, temp44);
	m_mult(temp44, g, x);
	m_scale(-1.0, x);

	x_out[0] = gmg(x,0,0);
	x_out[1] = gmg(x,1,0);
	x_out[2] = atan2(gmg(x,3,0),gmg(x,2,0));

	if(TRACE_ALGO) {
		fprintf(stderr, "x =  %f  %f %f deg\n", x_out[0], x_out[1],x_out[2]*180/M_PI);
	}

	MF(mA); MF(mB); MF(mD); MF(mS); MF(mSa);
	MF(m1t);	MF(m1);	MF(m2t);	MF(m2);
	MF(m3t);	MF(m3);	MF(W); 	MF(x);
	MF(bigM); MF(g); MF(bigM_k);
	MF(bigM_k_t); MF(C_k); MF(q_k);
	MF(temp42); MF(temp44);	MF(temp21);
	MF(temp41); MF(temp22);	MF(temp22b);
	MF(temp22c); MF(temp12);
	MF(g1);	MF(g2);
	MF(g1t);	MF(g2t);
	MF(mAi);	MF(mBt);
	return 1;
}


double gpc_error(const struct gpc_corr*co, const double*x) {
	double c = cos(x[2]);
	double s = sin(x[2]);
	double e[2];
	e[0] = c*(co->p[0]) -s*(co->p[1]) + x[0] - co->q[0];
	e[1] = s*(co->p[0]) +c*(co->p[1]) + x[1] - co->q[1];
	double this_error = e[0]*e[0]*co->C[0][0]+2*e[0]*e[1]*co->C[0][1]+e[1]*e[1]*co->C[1][1];

	if(0) /* due to limited numerical precision, error might be negative */
	if(this_error < 0) {
		fprintf(stderr, "Something fishy: error = %lf e = [%lf %lf]  C = [%lf,%lf;%lf,%lf]\n", this_error,
			e[0],e[1],co->C[0][0],co->C[0][1],co->C[1][0],co->C[1][1]);
	}
	return this_error;
}

double gpc_total_error(const std::vector<gpc_corr>& co, int n, const double*x){
  int i;
	double error = 0;
	for(i=0;i<n;i++) {
		if(!co[i].valid) continue;
		error += gpc_error(&(co.at(i)),x);
	}
	if(0) /* due to limited numerical precision, error might be negative */
	if(error<0) {
		fprintf(stderr, "Something fishy!\n");
	}
	return error;
}
	
/*
               [       C00       ]         [       c01       ]
                [                 ]         [                 ]
                [       C10       ]         [       C11       ]
(%o10)  Col 1 = [                 ] Col 2 = [                 ]
                [ py C10 + px C00 ]         [ py C11 + c01 px ]
                [                 ]         [                 ]
                [ px C10 - py C00 ]         [ px C11 - c01 py ]
         [              px C00 + c01 py              ]
         [                                           ]
         [              py C11 + px C10              ]
         [                                           ]
 Col 3 = [   2                     2                 ]
         [ py  C11 + px py C10 + px  C00 + c01 px py ]
         [                                           ]
         [               2                         2 ]
         [ px py C11 + px  C10 - px py C00 - c01 py  ]
         [              c01 px - py C00              ]
         [                                           ]
         [              px C11 - py C10              ]
         [                                           ]
 Col 4 = [               2                         2 ]
         [ px py C11 - py  C10 - px py C00 + c01 px  ]
         [                                           ]
         [   2                     2                 ]
         [ px  C11 - px py C10 + py  C00 - c01 px py ]*/


