#ifndef MOTION_PLANNING_DEBUG_HH
#define MOTION_PLANNING_DEBUG_HH

#ifdef MOTION_PLANNING_DEBUG
#define mp_debug_print(...) fprintf(stderr, __VA_ARGS__)
#else
#define mp_debug_print(...)
#endif
#endif // MOTION_PLANNING_DEBUG_HH
