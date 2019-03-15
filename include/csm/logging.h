#ifndef SM_LOGGING_H
#define SM_LOGGING_H

extern const char * sm_program_name;

void sm_set_program_name(const char*);

void sm_debug(const char *msg, ...);
void sm_error(const char *msg, ...);
void sm_info(const char *msg, ...);

/* Optional context handling for hyerarchical display */
void sm_log_push(const char*);
void sm_log_pop();

/* Enable/disable writing of debug information */
void sm_debug_write(int enabled);



/* Private interface */
void sm_write_context();
void check_for_xterm_color();
	
#endif
