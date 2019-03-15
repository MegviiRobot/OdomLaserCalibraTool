/*!
		\file aux_functions.h
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Collection of auxiliary functions (header file)
*/
#include <stdlib.h>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <json-c/json.h>

#ifndef M_PI
#define M_PI	3.14159265
#endif

using namespace std;
using namespace CSM;

/*!
	\brief Reads a list of JSON objects from filename
	@param filename name of the file to be read
	@param list pointer to a vector of JsonObject (JO) used to store data
	@return 1 if successfull, 0 otherwhise
*/
int read_jsonlist(const char * filename, vector <JO> *list);

/*!
	\brief Writes a list of JSON objects to filename
	@param filename name of the file to be written
	@param list pointer to a vector of JsonObject (JO)
	@returns 1 if successfull, 0 otherwhise
*/
int write_jsonlist(const char * filename, vector <JO> *list);

/*!
	\brief Searches the object (odometry object) closest to a given timestamp
	@param ts timestamp to be matched
	@param odo vector of odometry objects (JO)
	@param start_at starting index for search
	@return 0 if no interval matching has been found, otherwhise the index of the closest before
*/
int find_closest(double ts, vector <JO> odo, int start_at);

/*!
	\brief Searches the object closest (before) to a given timestamp
	@param ts timestamp to be matched
	@param odo vector of odometry objects (JO)
	@param bound starting index for search
	@return 0 if no interval matching has been found, otherwhise the index of the closest before
*/
int find_closest_ref(double ts, vector <JO> odo, int bound);

/*!
	\brief Searches the object closest (after) to a given timestamp
	@param ts timestamp to be matched
	@param odo vector of odometry objects (JO)
	@param bound starting index (from the end of the list) for search
	@return 0 if no interval matching has been found, otherwhise the index of the closest before
*/
int find_closest_sens(double ts, vector <JO> odo, int bound);

/*!
	\brief Sorts in ascending order the elements of an array of doubles
	@param sort array to be sorted
	@param length length of array
*/
void mysort(double *sort, int length);

/*!
	\brief Sorts in ascending order the elements of an array of ints
	@param sort array to be sorted
	@param length length of array
*/
void mysort(int *sort, int length);

/*!
	\brief Gets the ticks from an odometry object (JO)
	@param obj odometry object
	@param name left/right, selects left or right ticks
	@return number of ticks (int)
*/
int getticks(JO obj, const char* name);

/*!
	\brief Gets the ticks from an odometry object (JO)
	@param obj odometry object
	@param name left/right, selects left or right ticks
	@return number of ticks (double)
*/
double getticksD(JO obj, const char* name);

/*!
	\brief Gets the timestamp from an odometry/laser object (JO).
	Time is supposed to be represented by a two values array, an int for seconds and another for microseconds
	@param obj odometry object
	@param name field name (es: "Timestamp")
	@return timestamp
*/
double gettime(JO obj, const char* name);

/*!
	\brief Given two vectors (laser and odometry) computates two subvectors with matching timestamps
	@param odo vector of JO for odometry
	@param las_stats vector of JO for laser (laser statistics)
	@param linit starting index of laser subvector
	@param lend ending index of laser subvector
	@param oinit starting index of odometry subvector
	@param oend ending index of odometry subvector
	@return 1 if successfull, 0 otherwhise

*/
int logSync(vector <JO> odo, vector <JO> las_stats, int* linit, int* lend, int* oinit, int* oend);

/*!
	\brief Calculates matching between laser_stats and odometry objects
	@param odo vector of JO for odometry
	@param smStats vector of JO for laser (laser statistics)
	@param matchedOdo vector of JO to store matched odometry
	@param matchedLas vector of JO to store matched laser (statistics)
	@return 1 if successfull, 0 otherwhise
*/
int matchOdoLas(vector <JO> odo, vector <JO> smStats, vector <JO> * matchedOdo, vector <JO> * matchedLas);
