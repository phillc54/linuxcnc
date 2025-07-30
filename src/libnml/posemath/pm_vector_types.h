#ifndef PM_VECTOR_TYPES_H
#define PM_VECTOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

enum {PM_VECTOR_SIZE=9};

// WARNING this is an array under the hood!
typedef struct {
    double ax[PM_VECTOR_SIZE];
} PmVector;

#ifdef __cplusplus
}				/* matches extern "C" for C++ */
#endif

#endif // PM_VECTOR_TYPES_H
