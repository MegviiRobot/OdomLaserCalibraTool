/*! 
	\file   analyzer.cpp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief  Implements methods of class Analyzer
*/

#include "analyzer.h"
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sstream>

// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
using namespace std;

void Analyzer::set_par(Graph * g, const char * name, const char * title,
	const char * x_axis, const char * y_axis, const char * type,
	const char * legend, double *mx, double *Mx, double *my, double *My) {
	g->name   = name;
	g->title  = title;
	g->x_axis = x_axis;
	g->y_axis = y_axis;
	g->type   = type;
	g->legend = legend;
	g->mx = mx;
	g->Mx = Mx;
	g->my = my;
	g->My = My;
}


void Analyzer::add_data_graph(double * x, double * y, int N, Graph * g) {
 
	// scrive su un file nome_graph.gp i vettori x,y
	ofstream gr_os(g->name);

	for (int k = 0; k < N; k++) {
		gr_os << y[k];
		gr_os << "\t";
		gr_os << x[k];
		gr_os << "\n";
	}
	gr_os.close();
}

void Analyzer::create_plot(Graph * plots, int num_g, const char * file_plot, bool grid, bool legend,
	double *mx, double *Mx, double *my, double *My) {

	// creazione del file
	ofstream pl_os(file_plot);
	int num_row, num_col;
	double x_delay, y_delay, x_size, y_size;
	// impostaione multiplot
	bool multi = (num_g > 1);
	if (multi) { 
		pl_os << "set multiplot\n";
	
		num_row =  (int) floor((double)num_g / 3.0) 	+ 1;
		num_col =  (int) ceil((double)num_g / (double)num_row);
	
		x_delay = 1.0 / num_col;
		y_delay = 1.0 / num_row;
		x_size  = x_delay - x_delay * 0.1;
		y_size  = y_delay - y_delay * 0.1;
	}

	if (grid) pl_os   << "set grid\n";
	if (legend) pl_os << "set key outside\n\n";

	int i = 0;

	double x_origin, y_origin;
	for (int r = 0; r < num_row; r++) {
		for (int c = 0; c < num_col; c++) {

			i = r * num_col + c;
			if ( i < num_g ) {
				// setting title for graph[i]
				pl_os << "set title \""; pl_os << plots[i].title; pl_os << "\" \n";
				// setting origin of graph[i]
				x_origin = x_delay * c;
				y_origin = y_delay * (num_row - r - 1);
				if (multi) {
					pl_os << "set origin "; pl_os << x_origin; pl_os << ","; pl_os << y_origin; pl_os << "\n";
					// setting size   of graph[i]
					pl_os << "set size "; pl_os << x_size; pl_os << ","; pl_os << y_size; pl_os << "\n";
				}
				// setting labels
				pl_os << "set xlabel \""; pl_os << plots[i].x_axis; pl_os << "\" 0,0\n";
				pl_os << "set ylabel \""; pl_os << plots[i].y_axis; pl_os << "\" 0,0\n";
				// plotting graph[i]
				if (plots[i].mx != NULL) {
					pl_os << "plot [" << mx[0] << " to " << Mx[0] << "] [" << my[0] << " to " << My[0] << "] '";
				}
				else pl_os << "plot '";
				pl_os << plots[i].name;
				pl_os << "' using \"%lf%lf\" ";
				if (plots[i].type != NULL) {pl_os << "w "; pl_os << plots[i].type; pl_os << " ";}
				if (legend) pl_os << "title '" << plots[i].legend << "'\n\n";
				else pl_os << "notitle \n\n";
				if(!multi) {
					pl_os.close();
					return;
				}
			}
		}
	}

	pl_os.close();
}

void Analyzer::create_same_plot(const char * title, const char * x_axis, const char * y_axis, Graph * plots, int num_g, const char * file_plot, bool grid, bool legend, double *mx, double *Mx, double *my, double *My) {

	// creazione del file
	ofstream pl_os(file_plot);
	
	// impostaione multiplot
	if (grid) pl_os << "set grid\n";
	if (legend) pl_os << "set key outside right top nobox\n\n";

	int i = 0;	

	// setting title for graph[i]
	pl_os << "set title \""; pl_os << title; pl_os << "\" \n";
	// setting labels
	pl_os << "set xlabel \""; pl_os << x_axis; pl_os << "\" 0,0\n";
	pl_os << "set ylabel \""; pl_os << y_axis; pl_os << "\" 0,0\n";
	// plotting graph[i]
	pl_os << "plot ";
	if (mx != NULL) {
		pl_os << "[" << mx[0] << " to " << Mx[0] << "] [" << my[0] << " to " << My[0] << "] ";
	}
	for (int i = 0; i < num_g; i++ ) {
		pl_os << "'" << plots[i].name << "'"
		      << " using \"%lf%lf\" ";
		if (plots[i].type != NULL) {pl_os << "w "; pl_os << plots[i].type; pl_os << " ";}
		if (legend) pl_os << "title '" << plots[i].legend << "'";
		else pl_os << "notitle \n\n";
		if (i != num_g-1) pl_os << ", \\\n";
	}
	pl_os << "\n" << endl;

	pl_os.close();
}

// execute Gnuplot with file "file_plot"
void Analyzer::plot(const char * file_plot) {
	// usato nella chiamta a popen
	stringstream cmd;
	cmd << "gnuplot -persist " << file_plot << "\0";
	system(cmd.str().c_str());
}

