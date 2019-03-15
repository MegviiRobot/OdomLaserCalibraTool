#include <string.h>
#include <errno.h>

#include "csm/utils.h"
#include "csm/csm_all.h"


/** Wraps around fopen and provides error message. */
FILE * open_file(const char *filename, const char*mode);


FILE * open_file(const char *filename, const char*mode) {
	FILE*file = fopen(filename, mode);
	if(file==NULL) {
		sm_error("Could not open file '%s': %s.\n", filename, strerror(errno)); 
		return 0;
	}
	return file;
}

FILE * open_file_for_reading(const char*filename) {
	if(!strcmp(filename, "-"    )) return stdin;
	if(!strcmp(filename, "stdin")) return stdin;
	return open_file(filename, "r");
}

FILE * open_file_for_writing(const char*filename) {
	if(!strcmp(filename, "-"     )) return stdout;
	if(!strcmp(filename, "stdout")) return stdout;
	if(!strcmp(filename, "stderr")) return stderr;
	return open_file(filename, "w");
}

void my_basename(const char *file, char*dest) {
  const char *start = strrchr(file, DIR_SEPARATOR);
  if(!start)
	strcpy(dest, file);
  else
    strcpy(dest, start+1);
}

void my_basename_no_suffix(const char *file, char*dest) {
  const char *start = strrchr(file, DIR_SEPARATOR);
  if(!start) start = file; else start += 1;
  const char *end = strrchr(file, '.');
  if(!end || end<start) end = start + strlen(start);

  strncpy(dest, start, (size_t) (end-start));
  dest[end-start] = '\0';

/*  printf("start: '%s' end: '%s' base_no_suffix: '%s'\n", start, end, dest);*/
}

void my_no_suffix(const char *file, char*dest) {
  const char *end = strrchr(file, '.');
  if(!end) end = file + strlen(file);
  strncpy(dest, file, (size_t) (end-file) );
  dest[end-file] = '\0';
}


/** Our version of strdup. */
char * my_strdup(const char *s) {
	size_t len = strlen(s) + 1; /* null byte */
	char * t = (char*) malloc(len);
	memcpy(t,s,len);
	return t;
}

