/********************************************************************
* Description: tc.h
*   Discriminate-based trajectory planning
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
#ifndef TC_H
#define TC_H

#include "posemath.h"
#include "emcpos.h"
#include "emcmotcfg.h"
#include "tc_types.h"
#include "tp_types.h"
#include "pm_vector.h"

double tcGetMaxVelFromLength(TC_STRUCT const * const tc);

double tcGetAccelScale(TC_STRUCT const * tc);
double tcGetOverallMaxAccel(TC_STRUCT const * tc);
double tcGetTangentialMaxAccel(TC_STRUCT const * const tc);

double tcGetFinalVelInternal(const TC_STRUCT * tc);
double tcGetFinalVelLimitInternal(const TC_STRUCT * tc);
double tcGetPlanFinalVel(const TC_STRUCT * tc);

PmVector tcGetPos(TC_STRUCT const * const tc);
PmVector tcGetPosReal(TC_STRUCT const * const tc, double progress);
double tcGetVLimit(TC_STRUCT const * const tc, double v_target, double v_limit_linear, double v_limit_angular);

void tcSetOptimizationState(TC_STRUCT *tc, TCPlanningState state);
TCPlanningState tcGetOptimizationState(TC_STRUCT const *tc);

double tcGetDistanceToGo(TC_STRUCT const * const tc, int direction);
double tcGetTarget(TC_STRUCT const * const tc, int direction);

int tcSetTermCond(TC_STRUCT * tc, tc_term_cond_t term_cond);

const char *tcTermCondAsString(tc_term_cond_t c);
const char *tcCanonMotionTypeAsString(EMCMotionTypes c);
const char *tcMotionTypeAsString(tc_motion_type_t c);
const char *tcSyncModeAsString(tc_spindle_sync_t c);

#endif				/* TC_H */
