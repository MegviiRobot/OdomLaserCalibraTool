/*!
		\file aux_functions.cpp
		\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
		\brief Collection of auxiliary functions (implementation file)
*/
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <json-c/json.h>
#include <csm/csm_all.h>

#include "aux_functions.h"

using namespace std;
using namespace CSM;

int read_jsonlist(const char * filename, vector <JO> *list) {
	FILE * input = open_file_for_reading(filename); 
	if(!input) return 0;
	
	JO tmp;
	while (tmp = json_read_stream(input)) 
		list->push_back(tmp);
	
	if(!feof(input)) {
		sm_error("Error reading JSON file, after %d objects.\n", list->size());
		return 0;
	}
	
	fclose(input);
	return (1);
}

int write_jsonlist(const char * filename, vector <JO> *list) {
	FILE * output = open_file_for_writing(filename);
	if(!output) return 0;
	
	for (int i = 0; i < (int)list->size(); i++ ) {
 		if (i > 0) fprintf(output,"\n");
		
		fprintf(output, "%s", json_object_to_json_string(list[0][i]) );
	}
	
	fclose(output);	
 	return (1);
}

int find_closest(double ts, vector <JO> odo, int start_at) {
	for ( int i = start_at; i <  (int)odo.size() - 1; i++ ) {
		double ts_i = gettime(odo[i], "timestamp");
		double ts_ip = gettime(odo[i+1], "timestamp");
		if ( (ts_i < ts) && ( ts_ip > ts) ) {
			return(i);
		}
		if (ts_i > ts) {
			sm_error("find_closest: Sorry, could not find match for ts = %f\n", ts);
			return(-1);
		}
	}
	return(-1);
}
int find_closest_ref(double ts, vector <JO> odo, int bound) {
	double ts_i, ts_ip;
	if (bound==0) bound = -1;
	for ( int i = bound + 1; i <  (int)odo.size() - 1; i++ ) {
		ts_i = gettime(odo[i], "timestamp");
		ts_ip = gettime(odo[i+1], "timestamp");
		if ( (ts_i <= ts) && ( ts_ip >= ts) ) return (i);
	}
	sm_error("find_closest_ref: Sorry, could not find match for ts = %f\n", ts);
	return(-1);
}
int find_closest_sens(double ts, vector <JO> odo, int bound) {
	double ts_i, ts_ip;
	for ( int i = bound+1; i <  (int)odo.size(); i++ ) {
		ts_i = gettime(odo[i], "timestamp");
		ts_ip = gettime(odo[i-1], "timestamp");
		if ( (ts_i >= ts) && ( ts_ip <= ts) ) return (i);
	}
	sm_error("find_closest_sens: Sorry, could not find match for ts = %f\n", ts);
	return(-1);
}

void mysort(double *sort, int length) {
	int i, j;
	double tmp;
		
	for( i = 0; i < length - 1; i++) {
		for(j = i; j < length; j++) {
			if(sort[i] > sort[j]) {
				tmp = sort[i];
				sort[i] = sort[j];
				sort[j] = tmp;
			}
		}
  }
}

void mysort(int *sort, int length) {
	int i, j;
	int tmp;
		
	for( i = 0; i < length - 1; i++) {
		for(j = i; j < length; j++) {
			if(sort[i] > sort[j]) {
				tmp = sort[i];
				sort[i] = sort[j];
				sort[j] = tmp;
			}
		}
  	}
}

int getticks(JO obj, const char* name) {
	int tick;
	jo_read_int(obj, name, &tick);
	return tick;	
}

double getticksD(JO obj, const char* name) {
	double tick;
	jo_read_double(obj, name, &tick);
	return tick;	
}

double gettime(JO obj, const char* name) {
	double read_time[2];
	jo_read_double_array (obj, name, read_time, 2, 0.0);
	return read_time[0] + 0.000001 * read_time[1];	
}

int logSync(vector <JO> odo, vector <JO> las_stats, int* linit, int* lend, int* oinit, int* oend) {
	int nLas = (int)las_stats.size();
		
	bool found = false;
	int ref;
	for(int l = 0; (l < nLas) && !found; l++ ) {
		double lT = gettime(las_stats[l], "laser_ref_timestamp");
		ref = find_closest_ref(lT, odo, 0); 
		if (ref != -1) {
			found = true;
			linit[0] = l;
			oinit[0] = ref;
		}
	}
	if (!found) {
	 	sm_error("Could not find intersection of logs.\n");
		return 0;
	}

	found = false;
	int sens;
	for(int l = (nLas - 1); (l >= 0) && !found; l-- ) {
		double lT = gettime(las_stats[l], "laser_sens_timestamp");
		sens = find_closest_sens(lT, odo, 0); 
		if (sens != -1) {
			found = true;
			lend[0] = l;
			oend[0] = sens;
		}	
	}
	if (!found) {
	 	sm_error("Could not find intersection of logs.\n");
		return 0;
	}
	return 1;
}

int matchOdoLas(vector <JO> odo, vector <JO> smStats, vector <JO> * matchedOdo, vector <JO> * matchedLas){
	cout << "Checking correspondances between laser and odometry files..."<<endl;
	int linit, lend, oinit, oend;
	if ( !logSync(odo, smStats, &linit, &lend, &oinit, &oend) ) {
		sm_error("Error while looking for intersection\n");
		return 1;
	}
	
	double t1,t2,ts;
	double lticks, rticks;
	double o1[3], o2[3], os[3];
	int timestamp[2];

	ts = gettime(smStats[linit],"laser_ref_timestamp");
	t1 = gettime(odo[oinit], "timestamp");
	t2 = gettime(odo[oinit+1], "timestamp");
	double ref_alpha  = (ts-t1)/(t2-t1);
	lticks = (1-ref_alpha)*getticks(odo[oinit],"left") + ref_alpha*getticks(odo[oinit+1], "left");
	rticks = (1-ref_alpha)*getticks(odo[oinit],"right") + ref_alpha*getticks(odo[oinit+1], "right");
	jo_read_double_array (odo[oinit], "odometry", o1, 3, 0.0);
	jo_read_double_array (odo[oinit+1], "odometry", o2, 3, 0.0);	
	os[0] = (1-ref_alpha)*o1[0] + ref_alpha*o2[0];
	os[1] = (1-ref_alpha)*o1[1] + ref_alpha*o2[1];
	os[2] = (1-ref_alpha)*o1[2] + ref_alpha*o2[2];

	struct json_object *new_obj;
	new_obj = json_object_new_object();
	timestamp[0] = int(ts); 
	timestamp[1] = int(ts*1000000 - floor(ts)*1000000);
	jo_add_int_array (new_obj, "timestamp", timestamp, 2); 
	json_object_object_add(new_obj, "left", json_object_new_double(lticks) );
	json_object_object_add(new_obj, "right", json_object_new_double(rticks) );
	jo_add_double_array (new_obj, "odometry", os, 3);
	matchedOdo->push_back(new_obj);

	for( int i = oinit + 1; i < oend; i++ ) {
		new_obj = json_object_new_object();
		ts = gettime(odo[i], "timestamp");
		timestamp[0] = int(ts); 
		timestamp[1] = int(ts*1000000 - floor(ts)*1000000);
		jo_add_int_array (new_obj, "timestamp", timestamp, 2); 
		lticks = getticks(odo[i],"left");
		rticks = getticks(odo[i],"right");
		json_object_object_add(new_obj, "left", json_object_new_double(lticks) );
		json_object_object_add(new_obj, "right", json_object_new_double(rticks) );
		jo_read_double_array (odo[i], "odometry", os, 3, 0.0);
		jo_add_double_array (new_obj, "odometry", os, 3);
		matchedOdo->push_back(new_obj);
	}	

	ts = gettime(smStats[lend],"laser_sens_timestamp");
	t1 = gettime(odo[oend-1], "timestamp");
	t2 = gettime(odo[oend], "timestamp");
	double sens_alpha = (t2-ts)/(t2-t1);
	lticks = (1-sens_alpha)*getticks(odo[oend-1],"left") + sens_alpha*getticks(odo[oend], "left");
	rticks = (1-sens_alpha)*getticks(odo[oend-1],"right") + sens_alpha*getticks(odo[oend], "right");
	jo_read_double_array (odo[oend-1], "odometry", o1, 3, 0.0);
	jo_read_double_array (odo[oend], "odometry", o2, 3, 0.0);	
	os[0] = (1-sens_alpha)*o1[0] + sens_alpha*o2[0];
	os[1] = (1-sens_alpha)*o1[1] + sens_alpha*o2[1];
	os[2] = (1-sens_alpha)*o1[2] + sens_alpha*o2[2];	

	new_obj = json_object_new_object();
	timestamp[0] = int(ts); 
	timestamp[1] = int(ts*1000000 - floor(ts)*1000000);
	jo_add_int_array (new_obj, "timestamp", timestamp, 2); 
	json_object_object_add(new_obj, "left", json_object_new_double(lticks) );
	json_object_object_add(new_obj, "right", json_object_new_double(rticks) );
	jo_add_double_array (new_obj, "odometry", os, 3);
	matchedOdo->push_back(new_obj);
	
	double error;
	int valid;
	int nvalid;
	int nLas = lend - linit;
	double mean_error[nLas];
	double mean_error_sorted[nLas];
	int nvalid_sorted[nLas];
	double perc = 0.99;	
	for(int i = linit; i <= lend; i++) {
		int nvalid;
		
		jo_read_double(smStats[i], "error", &error);
		jo_read_int(smStats[i], "nvalid", &nvalid);
		mean_error[i] = error/nvalid;
		mean_error_sorted[i] = mean_error[i];
		nvalid_sorted[i] = nvalid;
	}	
	mysort(mean_error_sorted, nLas);
	mysort(nvalid_sorted, nLas);
	double mean_error_threshold = mean_error_sorted[ (int)round(perc*nLas)];
	cout << "mean_error_threshold " << mean_error_threshold << endl;
	int nvalid_threshold = nvalid_sorted[ (int)round((1 - perc)*nLas)];
	cout << "nvalid_threshold " << nvalid_threshold << endl;

	for (int i = linit; i <= lend; i++) {
		jo_read_int (smStats[i], "valid", &valid);
		if (valid == 0) { 
			cout << "VALID N" << i << endl; 
			continue;
		}
// 		if (mean_error[i] > mean_error_threshold) {
// 			cout << "MEAN ERROR N" << i << "\t" << mean_error[i] << endl; 
// 			continue;
// 		}
// 		jo_read_int (smStats[i], "nvalid", &nvalid);
// 		if (nvalid < nvalid_threshold) {
// 			cout << "NVALID N" << i << endl; 
// 			continue;
// 		}
		jo_read_double (smStats[i], "error", &error);
		if (error > 5.0) {
			cout << "ERROR N" << i << "\t" << error << endl; 
			continue;
		}
		matchedLas->push_back(smStats[i]);
	}
	return 0;
}

