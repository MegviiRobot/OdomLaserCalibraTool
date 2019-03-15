/*!	
	\file analyzer.h
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Header of class Analyzer 	
*/


#ifndef __ANALYZER_H__
#define __ANALYZER_H__

#include <math.h>
#include <stdio.h>
//#include <fstream>
//#include <string.h>

/*! 
	used to handle information about plot (not data)
	@param name name of data file for graph
	@param title title of graph
	@param x_axis x-axis label
	@param y_axis y-axis label
	@param type "impulses" "points" "lines"
	@param legend string for legend
	@param mx,Mx,my,My pointers to values for bounds
*/
typedef struct{
	const char * name;
	const char * title;
	const char * x_axis;
	const char * y_axis;
	const char * type;
	const char * legend;
	double *mx, *Mx, *my, *My;
} Graph;


/*!
	\class Analyzer 
	\brief class for printing graphs, using Gnuplot

	Create Gnuplot data-file from vectors,
	create Gnuplot script to plot data.
	Possibility of multiplot.
*/
class Analyzer {

public:
/*!
	Assign parameters to Graph object
	@param g Graph object
	@note For bounds (mx,Mx,my,My) default value is null. You can call this function omitting these parameters.
*/
void set_par(Graph * g, const char * name, const char * title,
		const char * x_axis, const char * y_axis,
		const char * type, const char * legend,
		double *mx=NULL, double *Mx=NULL, double *my=NULL, double *My=NULL);

/*!
	Create Gnuplot data-file from vectors x[] and y[]
	@param x pointer to a vector of double (x data)
	@param y pointer to a vector of double (y data)
	@param N number of elements in x[] and y[] (plot y to x)
	@param g pointer to Graph object
*/
void add_data_graph(double * x, double * y, int N, Graph * g);


/*! 
	Create Gnuplot script to plot (multiplot) data from files
	@param file_plot name of Gnuplot script 
	@param plots     vector of Graph objects
	@param num_g 	 number of subplots
	@param grid      grid on/off
	@param legend    legend on/off
	@param mx,Mx,my,My pointers to values for bounds (default is null)
*/
void create_plot(Graph * plots, int num_g, const char * file_plot, bool grid, bool legend,
	double *mx=NULL, double *Mx=NULL, double *my=NULL, double *My=NULL);

/*! 
	Create Gnuplot script to plot multiple data on the same plot
	@param file_plot name of Gnuplot script
	@param plots     vector of Graph objects
	@param num_g 	 number of subplots
	@param grid      grid on/off
	@param legend    legend on/off
	@param mx,Mx,my,My pointers to values for bounds (default is null)
*/
void create_same_plot(const char * title, const char * x_axis, const char * y_axis,
	Graph * plots, int num_g, const char * file_plot, bool grid, bool legend,
	double *mx=NULL, double *Mx=NULL, double *my=NULL, double *My=NULL);

/*! 
	Execute Gnuplot with file "file_plot" 
	@param file_plot name of Gnuplot script
*/
void plot(const char * file_plot);

};

#endif
