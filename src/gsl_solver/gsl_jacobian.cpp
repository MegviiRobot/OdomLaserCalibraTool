#include <gsl/gsl_deriv.h>
#include "gsl_jacobian.h"

struct my_params_struct {
    int i, j;
    gsl_vector * (*function)(const gsl_vector* x, void*params);
    const gsl_vector * x0;
    void * params;
};

double my_gsl_function(double xj, void * my_params) {
    struct my_params_struct * s = (struct my_params_struct*) my_params;   

    // create x by copying x0 and setting xj
    gsl_vector * x = gsl_vector_alloc(s->x0->size);
    gsl_vector_memcpy(x, s->x0);
    gsl_vector_set(x, s->j, xj);
    
    gsl_vector * fx = s->function(x, s->params);
    
    double fi = gsl_vector_get(fx, s->i);
    
    gsl_vector_free(fx);
    gsl_vector_free(x);
    
    return fi;
}


gsl_matrix * gsl_jacobian( gsl_vector * (*function)(const gsl_vector* x, void*params), const gsl_vector *x, void*params) {
  
    gsl_vector * fx =  function(x,params);
    
    int rows = fx->size;
    int columns = x->size;
        
    gsl_matrix * m = gsl_matrix_alloc(rows,columns);
    
    
    for(int i=0;i<rows;i++)
        for(int j=0;j<columns;j++) {

            struct my_params_struct s;
            s.function = function;
            s.x0 = x;
            s.i = i;
            s.j = j;
            s.params = params;
            
            gsl_function F;
            F.function = &my_gsl_function;
            F.params = &s;
        
            double result;
            double abserr;
            double h = 0.001;

            int res = gsl_deriv_central( &F, gsl_vector_get(x,j), h, &result, &abserr);

            
            gsl_matrix_set(m,i,j,result);
            
        }
        
    gsl_vector_free(fx);
    
    
    return m;
}

