#ifndef H_MATH_UTILS
#define H_MATH_UTILS


/* Sometimes I really don't understand compilers.. */ 
#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#ifndef NAN
#define NAN GSL_NAN
#endif
     
/** Returns norm of 2D point p */
double norm_d(const double p[2]);

double distance_d(const double a[2], const double b[2]);
double distance_squared_d(const double a[2], const double b[2]);

/** Returns an angle difference in the [-pi, pi] range */
double angleDiff(double a, double b);
double square(double x);

/** Degrees to radians */
double deg2rad(double deg);

/** Radians to degrees */
double rad2deg(double rad);


int minmax(int from,int to,int x);

/** Copies n doubles from from to to */
void copy_d(const double*from, int n, double*to);

/** These are the operators defined in Smith & Cheeseman  */
void ominus_d(const double x[3], double res[3]);
void oplus_d(const double x1[3], const double x2[3], double res[3]);
void pose_diff_d(const double second[3], const double first[3], double res[3]);
	
	
void transform_d(const double point2d[2], const double pose[3], double result2d[2]);
	
/** Projects (p[0],p[1]) on the LINE passing through (ax,ay)-(bx,by). If distance!=0, distance is set
to the distance from the point to the segment */
void projection_on_line_d(
	const double a[2],
	const double b[2],
	const double p[2],
	double res[2],
	double *distance);
	
/** Projection of P on the SEGMENT A-B */
void projection_on_segment_d(
	const double a[2],
	const double b[2],
	const double P[2],
   double proj[2]);
	
/** Distance of x from its projection on segment a-b */
double dist_to_segment_d(const double a[2], const double b[2], const double x[2]);

/** Same thing as dist_to_segment_d(), but squared */
double dist_to_segment_squared_d(const double a[2], const double b[2], const double x[2]);

/* Executes ray tracing for a segment. p0 and p1 are the segments extrema, eye is the position
of the eye, and direction is the direction of the ray coming out of the eye. Returns true
if the ray intersects the segment, and in that case *range contains the length of the ray. */
int segment_ray_tracing(const double p0[2], const double p1[2], const double eye[2], double direction, double*range);

/** Returns the orientation of the normal for the line passing through p0-p1 */
double segment_alpha(const double p0[2], const double p1[2]);

/** A function to print poses and covariances in a friendly way */
const char* friendly_pose(const double*pose);

/** Returns true v is NAN */
int is_nan(double v);

/** Returns true if any value in d is NAN */
int any_nan(const double *d, int n);

/** Count numbers of items in array v equal to value */
int count_equal(const int*v, int n, int value);

/** Normalizes an angle in the 0-2PI range */
double normalize_0_2PI(double angle);

/** Maximum value in the array */
double max_in_array(const double*v, int n);

#endif

