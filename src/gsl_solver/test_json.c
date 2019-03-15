#include <csm/csm_all.h>
#include <options/options.h>

int main(int argc, const char * argv[]) {
	sm_set_program_name(argv[0]);
	
	int n;
	
	/* Clean treatment of parameters using the "options" library */
	struct option* ops = options_allocate(3);
	options_int(ops, "n", &n, 1, "Number of copies");
	
	if(!options_parse_args(ops, argc, argv)) {
		sm_info("%s : reads a JSON stream and copies it multiplied by n."
			"\n\nOptions:\n", (char*)argv[0]);
		options_print_help(ops, stderr);
		return -1;
	}
	
	JO jo; 
	
	while((jo = json_read_stream(stdin))) {
		const char * s = json_object_to_json_string(jo);
		int i; for(i=0;i<n;i++) {
			puts(s); puts("\n");
		}
		jo_free(jo);
	}
	
	return 0;
}
