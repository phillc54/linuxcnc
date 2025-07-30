/********************************************************************
 * Description: tcq.c
 *\brief queue handling functions for trajectory planner
 * These following functions implement the motion queue that
 * is fed by tpAddLine/tpAddCircle and consumed by tpRunCycle.
 * They have been fully working for a long time and a wise programmer
 * won't mess with them.
 *
 *   Derived from a work by Fred Proctor & Will Shackleford
 *
 * Author:
 * License: GPL Version 2
 * System: Linux
 *    
 * Copyright (c) 2004 All rights reserved.
 *
 * Last change:
 ********************************************************************/

/* queue of TC_STRUCT elements*/
#ifndef TCQ_H
#define TCQ_H

#include "tcq_types.h"
#include "tc_types.h"

/* TC_QUEUE_STRUCT functions */

/* create queue of _size */
extern int tcqCreate(TC_QUEUE_STRUCT * const tcq, int _size);

/* free up queue */
extern int tcqDelete(TC_QUEUE_STRUCT * const tcq);

/* reset queue to empty */
extern int tcqReset(TC_QUEUE_STRUCT * const tcq);

extern void tcqDiscardReverseQueue(TC_QUEUE_STRUCT * const tcq);

/* put tc on end */
extern int tcqPut(TC_QUEUE_STRUCT * const tcq, TC_STRUCT const * const tc);

/* remove a single tc from the back of the queue */
extern int tcqPopBack(TC_QUEUE_STRUCT * const tcq);

extern int tcqPop(TC_QUEUE_STRUCT * const tcq);

/* remove n tcs from front */
extern int tcqRemove(TC_QUEUE_STRUCT * const tcq, int n);

extern int tcqBackStep(TC_QUEUE_STRUCT * const tcq);

/* how many tcs on queue */
extern int tcqLen(TC_QUEUE_STRUCT const * const tcq);
extern int tcqReverseLen(TC_QUEUE_STRUCT const * const tcq);

/* look at nth item, first is 0 */
extern TC_STRUCT * tcqItem(TC_QUEUE_STRUCT * tcq, int n);

/**
 * Get the "end" of the queue, the most recently added item.
 */
extern TC_STRUCT * tcqLast(TC_QUEUE_STRUCT * tcq);

#endif
