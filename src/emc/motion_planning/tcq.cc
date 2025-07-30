/*!
 ********************************************************************
 * Description: tcq.c
 *\brief queue handling functions for trajectory planner
 * These following functions implement the motion queue that
 * is fed by tpAddLine/tpAddCircle and consumed by tpRunCycle.
 * They have been fully working for a long time and a wise programmer
 * won't mess with them.
 *
 *\author Derived from a work by Fred Proctor & Will Shackleford
 *\author rewritten by Chris Radek
 *
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2004 All rights reserved.
 *
 ********************************************************************/

#include "tcq.hh"
#include <stddef.h>
#include "stdio.h"

/** Return 0 if queue is valid, -1 if not */
static inline int tcqCheck(TC_QUEUE_STRUCT const * const tcq)
{
    if ((0 == tcq))
    {
        return -1;
    }
    return 0;
}

static inline int tcqSize(TC_QUEUE_STRUCT const *tcq)
{
    return tcq->size - TCQ_REVERSE_MARGIN;
}

/*! tcqPut() function
 *
 * \brief puts a TC element at the end of the queue
 *
 * This function adds a tc element at the end of the queue.
 * It gets called by tpAddLine() and tpAddCircle()
 *
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 tc        the new TC element to be added
 *
 * @return	 int	   returns success or failure
 */
int tcqPut(TC_QUEUE_STRUCT * const tcq, TC_STRUCT const * const tc)
{
    /* check for initialized */
    if (tcqCheck(tcq)) return -1;

    /* add it */
    int current_end = __atomic_load_n(&tcq->end, __ATOMIC_ACQUIRE);
    int current_start = __atomic_load_n(&tcq->start, __ATOMIC_ACQUIRE);

    int current_len = (current_end - current_start + tcq->size) % tcq->size;

    // The actual available space in the queue
    int forward_queue_size = tcq->size - TCQ_REVERSE_MARGIN;
    int actual_available_space = forward_queue_size - current_len;
    if (actual_available_space < 1) {
        return -2;
    }

    tcq->queue[current_end] = *tc;
    // Only increment the index if we have the space (so tcqPut can be
    // repeatedly called without overflowing the queue)
    atomicIncrementIndex(&tcq->end, tcq->size);

    return 0;
}


/*! tcqPopBack() function
 *
 * \brief removes the newest TC element (converse of tcqRemove)
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success or failure
 */
int tcqPopBack(TC_QUEUE_STRUCT * const tcq)
{
    /* check for initialized */
    if (tcqCheck(tcq)) return -1;

    /* Too short to pop! */
    if (tcqLen(tcq) < 1) {
        return -2;
    }
    

    atomicDecrementIndex(&tcq->end, tcq->size);

    return 0;
}

#define TCQ_REVERSE_MARGIN 100

/*! tcqLen() function
 *
 * \brief returns the number of elements in the queue
 *
 * Function gets called by tpSetVScale(), tpAddLine(), tpAddCircle()
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns number of elements
 */
int tcqLen(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    // Called from userspace size, we control end
    int end = __atomic_load_n(&tcq->end, __ATOMIC_ACQUIRE);
    // Read from RT at last possible time
    int start = __atomic_load_n(&tcq->start, __ATOMIC_ACQUIRE);

    // Return the positive length
    return (end - start + tcq->size) % tcq->size;
}

/*!
 * Gets the n-th TC element from the back of the queue (0 = end, -1 = one before the end, etc).
 */
TC_STRUCT * tcqBack(TC_QUEUE_STRUCT * tcq, int n)
{

    if (tcqCheck(tcq)) {
        // Allow retrieval of items from the beginning (0+) or from the end (-1 and lower), but don't allow more than 1 pass)
        return NULL;
    }
    // Called from userspace size, we control end
    const int end = __atomic_load_n(&tcq->end, __ATOMIC_ACQUIRE);
    // Read from RT at last possible time
    const int start = __atomic_load_n(&tcq->start, __ATOMIC_ACQUIRE);
    const int len = (end - start + tcq->size) % tcq->size;

    if (0 == len) {
        return NULL;
    } else if (n > 0) {
        return NULL;
    } else if (n > -len) {
        //Fix for negative modulus error
        int k = end + n - 1 + tcq->size;
        int idx = k % tcq->size;
        return &(tcq->queue[idx]);
    } else {
        return NULL;
    }
}

TC_STRUCT const * tcqBack(TC_QUEUE_STRUCT const * tcq, int n) {
    // Scott Meyer's trick to implement a const overload
    return static_cast<TC_STRUCT const *>(tcqBack(const_cast<TC_QUEUE_STRUCT*>(tcq), n));
}

/*! tcqFull() function
 *
 * \brief get the full status of the queue
 * Function returns full if the count is closer to the end of the queue than TC_QUEUE_MARGIN
 *
 * Function called by update_status() in control.c
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int       returns status (0==not full, 1==full)
 */
int tcqFull(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) {
	   return 1;		/* null queue is full, for safety */
    }

    /* call the queue full if the length is into the margin, so reduce the
       effect of a race condition where the appending process may not see the
       full status immediately and send another motion */

    int size_with_margin = tcqSize(tcq) - QUEUE_FULL_MARGIN;
    if (tcqLen(tcq) > size_with_margin) {
        /* we're into the margin, so call it full */
        return 1;
    }

    /* we're not into the margin */
    return 0;
}
