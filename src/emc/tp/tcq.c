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

#include "tcq.h"
#include <stddef.h>
#include "assert.h"

/** Return 0 if queue is valid, -1 if not */
static inline int tcqCheck(TC_QUEUE_STRUCT const * const tcq)
{
    if ((0 == tcq))
    {
        return -1;
    }
    return 0;
}

/*! tcqCreate() function
 *
 * \brief Creates a new queue for TC elements.
 *
 * This function creates a new queue for TC elements.
 * It gets called by tpCreate()
 *
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 _size	   size of the new queue
 * @param	 tcSpace   holds the space allocated for the new queue, allocated in motion.c
 *
 * @return	 int	   returns success or failure
 */
int tcqCreate(TC_QUEUE_STRUCT * const tcq, int _size)
{
    if (0 == tcq ) {
        return -1;
    }
    if (_size <= 0 || _size > DEFAULT_TC_QUEUE_SIZE) {
        return -2;
    }

    tcq->size = _size;

    // Initialize all indices to zero
    // This is the one time that RT is allowed to write to end since it's at
    // init time when userspace is not doing anything
    __atomic_store_n(&tcq->end, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tcq->start, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tcq->rend, 0, __ATOMIC_RELEASE);
    tcqReset(tcq);

    return 0;
}

/*! tcqDelete() function
 *
 * \brief Deletes a queue holding TC elements.
 *
 * This function creates deletes a queue. It doesn't free the space
 * only throws the pointer away.
 * It gets called by tpDelete()
 * \todo FIXME, it seems tpDelete() is gone, and this function isn't used.
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success
 */
int tcqDelete(TC_QUEUE_STRUCT * const tcq)
{
    // No-op since the struct is part of the tcq struct
    return 0;
}

/*! tcqInit() function
 *
 * \brief Initializes a queue with TC elements.
 *
 * This function initializes a queue with TC elements.
 * It gets called by tpClear() and
 * 	  	   		  by tpRunCycle() when we are aborting
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success or failure (if no tcq found)
 */
int tcqReset(TC_QUEUE_STRUCT * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    // Force the queue to be empty by advancing the start / rend to match end
    // This method ensures that RT doesn't alter end, which would violate
    // assumptions in userspace NOTE: this ensures correct ordering but is NOT
    // synchronizing by itself.  To end up with a truely empty queue, userspace
    // must not write anything while this is being executed.
    int end = __atomic_load_n(&tcq->end, __ATOMIC_ACQUIRE);
    __atomic_store_n(&tcq->start, end, __ATOMIC_RELEASE);
    __atomic_store_n(&tcq->rend, end, __ATOMIC_RELEASE);

    return 0;
}

void tcqDiscardReverseQueue(TC_QUEUE_STRUCT * const tcq)
{
    if (0 == tcqCheck(tcq)) {
        tcq->rend = 0;
    }
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
 * @deprecated
 */
int tcqPut(TC_QUEUE_STRUCT * const tcq, TC_STRUCT const * const tc)
{
    // Deprecated
    assert(false);
}


/*! tcqPopBack() function
 *
 * \brief removes the newest TC element (converse of tcqRemove)
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 int	   returns success or failure
 * @deprecated in realtime
 */
int tcqPopBack(TC_QUEUE_STRUCT * const tcq)
{
    /* check for initialized */
    if (tcqCheck(tcq)) return -1;

    /* Too short to pop! */
    if (tcqLen(tcq) < 1) {
        return -1;
    }

    int n = tcq->end - 1 + tcq->size;
    tcq->end = n % tcq->size;

    return 0;
}

int tcqPop(TC_QUEUE_STRUCT * const tcq)
{

    if (tcqCheck(tcq)) {
        return -1;
    }

    // Don't care if new segments are added after we start, because we can't safely pop them
    int len = tcqLen(tcq);
    if (len < 1) {
        return -1;
    }

    /* update start ptr and reset allFull flag and len */
    atomicIncrementIndex(&tcq->start, tcq->size);

    if (tcqReverseLen(tcq) >= TCQ_REVERSE_MARGIN) {
        //If we're run out of spare reverse history, then advance rend
        atomicIncrementIndex(&tcq->rend, tcq->size);
    }

    return 0;
}

/*! tcqRemove() function
 *
 * \brief removes n items from the queue
 *
 * This function removes the first n items from the queue,
 * after checking that they can be removed
 * (queue initialized, queue not empty, enough elements in it)
 * Function gets called by tpRunCycle() with n=1
 * \todo FIXME: Optimize the code to remove only 1 element, might speed it up
 *
 * @param    tcq       pointer to the new TC_QUEUE_STRUCT
 * @param	 n         the number of TC elements to be removed
 *
 * @return	 int	   returns success or failure
 */
int tcqRemove(TC_QUEUE_STRUCT * const tcq, int n)
{

    if (n <= 0) {
	    return 0;		/* okay to remove 0 or fewer */
    }

    if (tcqCheck(tcq) || ((tcq->start == tcq->end)) ||
            (n > tcqLen(tcq))) {	/* too many requested */
	    return -1;
    }

    /* update start ptr and reset allFull flag and len */
    tcq->start = (tcq->start + n) % tcq->size;

    return 0;
}

/**
 * Step backward into the reverse history.
 */
int tcqBackStep(TC_QUEUE_STRUCT * const tcq)
{
    if (tcqCheck(tcq)) {
        return -1;
    }

    // start == end means that queue is empty
    
    int rempty = (tcq->start == tcq->rend);
    if ( rempty ) {
        return -1;
    }
    /* update start ptr and reset allFull flag and len */
    atomicDecrementIndex(&tcq->start, tcq->size);

    return 0;
}

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

int tcqReverseLen(TC_QUEUE_STRUCT const * const tcq)
{
    if (tcqCheck(tcq)) return -1;

    int rend = __atomic_load_n(&tcq->rend, __ATOMIC_ACQUIRE);
    int start = __atomic_load_n(&tcq->start, __ATOMIC_ACQUIRE);

    // Return the positive length of the reverse portion of the queue
    return (start - rend + tcq->size) % tcq->size;
}

/*! tcqItem() function
 *
 * \brief gets the n-th TC element in the queue, without removing it
 *
 * Function gets called by tpSetVScale(), tpRunCycle(), tpIsPaused()
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 TC_STRUCT returns the TC elements
 */
TC_STRUCT * tcqItem(TC_QUEUE_STRUCT * tcq, int n)
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
    }
    if (n >= len) {
        return NULL;
    } else if (n >= 0) {
        int idx =  (start + n) % tcq->size;
        return &(tcq->queue[idx]);
    } else if (n >= -len) {
        //Fix for negative modulus error
        int k = end + n + tcq->size;
        int idx = k % tcq->size;
        return &(tcq->queue[idx]);
    } else {
        return NULL;
    }
}

/*! tcqLast() function
 *
 * \brief gets the last TC element in the queue, without removing it
 *
 *
 * @param    tcq       pointer to the TC_QUEUE_STRUCT
 *
 * @return	 TC_STRUCT returns the TC element
 */
TC_STRUCT *tcqLast(TC_QUEUE_STRUCT * tcq)
{
    return tcqItem(tcq, -1);
}

