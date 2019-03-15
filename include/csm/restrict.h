/* Some preprocessor magic for having fast inline functions. */
#ifndef INLINE
#define INLINE static inline
#endif
#ifndef INLINE_DECL
#define INLINE_DECL static inline
#endif

/* Some preprocessor magic for the "restrict" keyword:
	http://www.cellperformance.com/mike_acton/2006/05/demystifying_the_restrict_keyw.html */
#if __STDC_VERSION__ >= 199901
#elif defined (__GNUC__) && __GNUC__ >= 2 && __GNUC_MINOR__ >= 91
	#define restrict __restrict__
#else
	#define restrict
#endif

/* Some preprocessor magic for calling this library from C++ */

#ifdef __cplusplus
	#define restrict /* nothing */
#endif

