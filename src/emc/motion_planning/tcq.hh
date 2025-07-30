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
#ifndef TCQ_HH
#define TCQ_HH

#include "tcq_types.h"
#include "tc_types.h"

/* TC_QUEUE_STRUCT functions */

// NOTE: don't do init / delete in userspace

/* put tc on end */
extern int tcqPut(TC_QUEUE_STRUCT * const tcq, TC_STRUCT const * const tc);

/* remove a single tc from the back of the queue */
extern int tcqPopBack(TC_QUEUE_STRUCT * const tcq);

// NOTE: can't pop in userspace side
// NOTE: can't remove segments from front from userspace side
// NOTE: can't backstep in userspace (no reason to)

/* how many tcs on queue */
extern int tcqLen(TC_QUEUE_STRUCT const * const tcq);

/* look at nth item, first is 0 */
extern TC_STRUCT * tcqBack(TC_QUEUE_STRUCT * tcq, int n);
extern TC_STRUCT const * tcqBack(TC_QUEUE_STRUCT const * tcq, int n);

/* get full status */
extern int tcqFull(TC_QUEUE_STRUCT const * const tcq);

#endif // TCQ_HH
