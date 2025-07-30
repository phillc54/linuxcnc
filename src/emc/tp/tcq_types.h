#ifndef TCQ_TYPES_H
#define TCQ_TYPES_H

#include "tc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* space for trajectory planner queues, plus 10 more for safety */
    struct tc_struct_t queue[DEFAULT_TC_QUEUE_SIZE + 10];
    int size;			/* size of queue */
    int start;      // Index of front of queue (for RT side to access / pop)
    int end;		// Index of back of queue (for userspace side to access / push)
    int rend;
} TC_QUEUE_STRUCT;

// Reserves a portion of the queue to keep old motions around for reverse run
// This space cannot be used to queue new motions (so the effective worst-case
// queue size is smaller than nominal by at least this much).
#define TCQ_REVERSE_MARGIN 100

// Threshold to declare the queue "full" in motion planning and wait for TP to
// drain some motions.  This much be larger than the number of motions added by
// the worst-case motion command. For example, adding a single new line / arc
// can cause two additional motions to be inserted (for biarc blending). It
// doesn't really hurt for this to be larger (except that it slightly wastes
// memory in the queue). 
#define QUEUE_FULL_MARGIN 20

static inline int atomicIncrementIndex(int *idx, int queue_size)
{
    int new_idx = ((__atomic_load_n(idx, __ATOMIC_ACQUIRE)) + 1) % queue_size;
    __atomic_exchange_n(idx, new_idx, __ATOMIC_ACQ_REL);
    return new_idx;
}

static inline int atomicDecrementIndex(int *idx, int queue_size)
{
    int new_idx = ((__atomic_load_n(idx, __ATOMIC_ACQUIRE)) - 1 + queue_size) % queue_size;
    __atomic_exchange_n(idx, new_idx, __ATOMIC_ACQ_REL);
    return new_idx;
}

#ifdef __cplusplus
}				/* matches extern "C" for C++ */
#endif
#endif // TCQ_TYPES_H
