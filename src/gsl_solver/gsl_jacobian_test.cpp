#include "gsl_jacobian.h"

gsl_vector* test1(const gsl_vector*x, void*dummy) {
    gsl_vector*res = gsl_vector_alloc(x->size);
    for(int i=0;i<x->size;i++) {
        gsl_vector_set(res, i, gsl_vector_get(x, i) *  gsl_vector_get(x, i));
    }
    
    return res;
}

int main() {
    
    gsl_vector * v = gsl_vector_alloc(4);
    gsl_vector_set(v,0,1);
    gsl_vector_set(v,1,2);
    gsl_vector_set(v,2,3);
    gsl_vector_set(v,3,4);
    
    gsl_matrix * J = gsl_jacobian(test1, v, 0);
    
    gsl_matrix_fprintf(stderr, J, "%g");
    
    fprintf(stderr, "\n");
    
}
