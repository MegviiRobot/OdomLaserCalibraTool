//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_blas.h>
//#include <gsl/gsl_linalg.h>

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "egsl/egsl.h"
#include "egsl/egsl_imp.h"
#define MAX_VALS 1024
#define MAX_CONTEXTS 1024

/*#define INVALID (val_from_context_and_index(1000,1000))*/


struct egsl_variable {
	gsl_matrix * gsl_m;
};

struct egsl_context {
	char name[256];
	int nallocated;
	int nvars;
	struct egsl_variable vars[MAX_VALS]; 
};

/* Current context */
int cid=0;
/* Maximum number of contexts */
int max_cid = 0; 
static struct egsl_context egsl_contexts[MAX_CONTEXTS];


int egsl_first_time = 1;

int egsl_total_allocations = 0;
int egsl_cache_hits = 0;

void egsl_error(void) {
	/* TODO: better handling of errors */

	egsl_print_stats();

	assert(0);
}

val assemble_val(int cid, int index, gsl_matrix*m) {
	val v; 
		v.cid=cid; 
		v.index=index; 
		v.gslm = m; 
	return v;
}

int its_context(val v) {
	return v.cid;
}

int its_var_index(val v) {
	return v.index;
}


#if 0
inline void check_valid_val(val v) { int i = v.cid; v.cid=i;}	
#else
void check_valid_val(val v) {
	int context = its_context(v);
	if(context>cid) {
		fprintf(stderr, "Val is from invalid context (%d>%d)\n",context,cid);
		egsl_error();
	}
	int var_index = its_var_index(v);
	if(var_index >= egsl_contexts[context].nvars) {
		fprintf(stderr, "Val is invalid  (%d>%d)\n",var_index, 
			egsl_contexts[context].nvars);		
		egsl_error();
	}
}
#endif

gsl_matrix * egsl_gslm(val v) {
	check_valid_val(v);
	return v.gslm;
}

void egsl_push() { egsl_push_named("unnamed context"); }
void egsl_pop() { egsl_pop_named("unnamed context"); }

void egsl_push_named(const char*name) {
	if(egsl_first_time) {
		int c;
		for(c=0;c<MAX_CONTEXTS;c++) {
			egsl_contexts[c].nallocated = 0;
			egsl_contexts[c].nvars = 0;
			sprintf(egsl_contexts[c].name, "not yet used");
		}
		egsl_first_time  = 0;
	}
	cid++;

	if(cid >= MAX_CONTEXTS) {
		fprintf(stderr, "egsl: maximum number of contexts reached \n");
		egsl_print_stats();
		assert(0);
	}

	if(max_cid < cid) max_cid = cid;
	
	if(name != 0) 
		sprintf(egsl_contexts[cid].name, "%s", name);
	else
		sprintf(egsl_contexts[cid].name, "Unnamed context");		
}

void egsl_pop_named(const char*name) {
	assert(cid>=0);/*, "No egsl_push before?"); */
	if(name != 0) {
		if(strcmp(name, egsl_contexts[cid].name)) {
			fprintf(stderr, "egsl: context mismatch. You want to pop '%s', you are still at '%s'\n",
				name, egsl_contexts[cid].name);
			egsl_print_stats();
			assert(0);
		}
	}
	
	egsl_contexts[cid].nvars = 0;
	sprintf(egsl_contexts[cid].name, "Popped");
	cid--;
}
/*
void egsl_pop_check(int assumed) {
	if(assumed != cid) {
		fprintf(stderr, "egsl: You think you are in context %d while you are %d. \n", assumed, cid);
		if(assumed < cid) 
			fprintf(stderr, "     It seems you miss %d egsl_pop() somewhere.\n", - assumed + cid);			
		else
			fprintf(stderr, "     It seems you did %d egsl_pop() more.\n",  + assumed - cid);			
		egsl_print_stats();
	}
	assert(cid>=0);
	egsl_contexts[cid].nvars = 0;
	cid--;
}*/

void egsl_print_stats() {
	fprintf(stderr, "egsl: total allocations: %d   cache hits: %d\n",
		egsl_total_allocations, egsl_cache_hits);
/*	printf("egsl: sizeof(val) = %d\n",(int)sizeof(val)); */
	int c; for(c=0;c<=max_cid&&c<MAX_CONTEXTS;c++) {
	/*	printf("egsl: context #%d\n ",c); */
/*	 	if(0==egsl_contexts[c].nallocated) break; */
		fprintf(stderr, "egsl: context #%d allocations: %d active: %d name: '%s' \n",
			c,	egsl_contexts[c].nallocated, egsl_contexts[c].nvars, egsl_contexts[c].name);
	}
}

val egsl_alloc(size_t rows, size_t columns) {
	struct egsl_context*c = egsl_contexts+cid;
	
/*	if(cid<3)
	printf("Alloc cid=%d nvars=%d nalloc=%d\n",cid,c->nvars,c->nallocated); */
	
	if(c->nvars>=MAX_VALS) {
		fprintf(stderr,"Limit reached, in context %d, nvars is %d\n",cid,c->nvars);
		egsl_error();
	}
	int index = c->nvars;
	if(index<c->nallocated) {
		gsl_matrix*m = c->vars[index].gsl_m;
		if((size_t) m->rows() == rows && (size_t) m->cols() == columns) {
			egsl_cache_hits++;
			c->nvars++;
			return assemble_val(cid,index,c->vars[index].gsl_m);
		} else {
			gsl_matrix_free(m);
			egsl_total_allocations++;			
			c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
			c->nvars++;
			return assemble_val(cid,index,c->vars[index].gsl_m);
		}
	} else {
		egsl_total_allocations++;
		c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
		c->nvars++;
		c->nallocated++;
		return assemble_val(cid,index,c->vars[index].gsl_m);
	}
}

val egsl_alloc_in_context(int context, size_t rows, size_t columns) {
    struct egsl_context*c = egsl_contexts+context;

/*  if(cid<3)
    printf("Alloc cid=%d nvars=%d nalloc=%d\n",cid,c->nvars,c->nallocated); */

    if(c->nvars>=MAX_VALS) {
        fprintf(stderr,"Limit reached, in context %d, nvars is %d\n",context,c->nvars);
        egsl_error();
    }
    int index = c->nvars;
    if(index<c->nallocated) {
        gsl_matrix*m = c->vars[index].gsl_m;
        if((size_t) m->rows() == rows && (size_t) m->cols() == columns) {
            egsl_cache_hits++;
            c->nvars++;
            return assemble_val(context,index,c->vars[index].gsl_m);
        } else {
            gsl_matrix_free(m);
            egsl_total_allocations++;
            c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
            c->nvars++;
            return assemble_val(context,index,c->vars[index].gsl_m);
        }
    } else {
        egsl_total_allocations++;
        c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
        c->nvars++;
        c->nallocated++;
        return assemble_val(context,index,c->vars[index].gsl_m);
    }
}

/*
val egsl_alloc_in_context(int context, size_t rows, size_t columns) {
	egsl_total_allocations++;
	struct egsl_context *c = egsl_contexts+context;
	int index = c->nvars;
	c->vars[index].gsl_m = gsl_matrix_alloc((size_t)rows,(size_t)columns);
	c->nvars++;
	c->nallocated++;
	return assemble_val(context,index,c->vars[index].gsl_m);
}
*/


/** Creates a copy of v in the previous context. */
val egsl_promote(val v) {
	if(cid==0) {
		egsl_error();
	}

	gsl_matrix * m = egsl_gslm(v);
	val v2 = egsl_alloc_in_context(cid-1, m->rows(), m->cols());
	gsl_matrix * m2 = egsl_gslm(v2);
	gsl_matrix_memcpy(m2, m);
	return v2;
}




void egsl_expect_size(val v, size_t rows, size_t cols) {
	gsl_matrix * m = egsl_gslm(v);

	int bad = (rows && ((size_t) m->rows()!=rows)) || (cols && ((size_t) m->cols()!=cols));
	if(bad) {
		fprintf(stderr, "Matrix size is %d,%d while I expect %d,%d",
			(int)m->rows(),(int)m->cols(),(int)rows,(int)cols);

		egsl_error();
	}
}


void egsl_print(const char*str, val v) {
	gsl_matrix * m = egsl_gslm(v);
	int i,j;
	int context = its_context(v);
	int var_index = its_var_index(v);
	fprintf(stderr, "%s =  (%d x %d)  context=%d index=%d\n",
		str,(int)m->rows(),(int)m->cols(),  context, var_index);

	for(i=0;i<m->rows();i++) {
		if(i==0)
			fprintf(stderr, " [ ");
		else
			fprintf(stderr, "   ");
		
		for(j=0;j<m->cols();j++)
			fprintf(stderr, "%f ", gsl_matrix_get(m,i,j));
		
		
		if(i==m->rows()-1)
			fprintf(stderr, "] \n");
		else
			fprintf(stderr, "; \n");
	}	
}

double* egsl_atmp(val v, size_t i, size_t j) {
	gsl_matrix * m = egsl_gslm(v);
	return gsl_matrix_ptr(m,(size_t)i,(size_t)j);
}


double egsl_norm(val v1){
	egsl_expect_size(v1, 0, 1);
	double n=0;
	int i;
	gsl_matrix * m = egsl_gslm(v1);
	for(i=0;i<m->rows();i++) {
		double v = gsl_matrix_get(m,i,0);
		n += v * v;
	}
	return sqrt(n);
}

double egsl_atv(val v1,  size_t i){
	return *egsl_atmp(v1, i, 0);
}

//double egsl_atm(val v1, size_t i, size_t j){
//	return *egsl_atmp(v1, i, j);
//}


void egsl_free_unused_memory(){
  int c;
  for(c=0;c<=max_cid;c++) {
    for(int i=egsl_contexts[c].nvars; i<egsl_contexts[c].nallocated; i++){
      gsl_matrix_free(egsl_contexts[c].vars[i].gsl_m);
    }
    egsl_contexts[c].nallocated = egsl_contexts[c].nvars;
  }
}
