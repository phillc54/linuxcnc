/********************************************************************
* Description: tp.h
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef TP_H
#define TP_H

#include "posemath.h"
#include "tc_types.h"
#include "tp_types.h"
#include "tp_enums.h"

#define DEFAULT_TP_SUPERSAMPLE_RATE 1

#ifdef __cplusplus
extern "C" {
#endif

int tpCreate(TP_STRUCT * const tp, int _queueSize);
int tpResetAtModeChange(TP_STRUCT * const tp);
int tpInit(TP_STRUCT * const tp);
int tpClearDIOs(TP_STRUCT * const tp);
int tpSetCycleTime(TP_STRUCT * const tp, double secs, int supersamples);
double tpGetCycleTime(TP_STRUCT * const tp);
int tpSetVlimit(TP_STRUCT * const tp, double vLimit, double vLimitAng);
int tpGetExecId(TP_STRUCT const * const tp);
int tpGetCompletedId(TP_STRUCT const * const tp);
// NOTE: removed tpGetQueuedId() since it's unused in RT side
struct state_tag_t tpGetExecTag(const TP_STRUCT * const tp);
int tpGetExecSrcLine(const TP_STRUCT * const tp);
int tpGetNextExecId(TP_STRUCT const * const tp);
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos);
int tpAddCurrentPos(TP_STRUCT * const tp, const PmVector * const disp);
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos);

tp_err_t tpRunCycle(TP_STRUCT * const tp);
int tpPause(TP_STRUCT * const tp);
int tpResume(TP_STRUCT * const tp);
int tpAbort(TP_STRUCT * const tp);
int tpGetPos(TP_STRUCT const  * const tp, EmcPose * const pos);
EmcPose tpGetCurrentPos(TP_STRUCT const  * const tp);
void tpSetFilterStatus(TP_STRUCT * const tp, bool at_rest);
int tpIsDone(TP_STRUCT * const tp);
int tpQueueDepth(TP_STRUCT * const tp);
int tpActiveDepth(TP_STRUCT * const tp);
int tpGetMotionType(TP_STRUCT * const tp);
void tpToggleDIOs(TC_STRUCT * const tc); //gets called when a new tc is taken from the queue. it checks and toggles all needed DIO's

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end);
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end); //gets called to place DIO toggles on the TC queue

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir);
bool tpIsMoving(TP_STRUCT const * const tp);
bool tpIsStopping(TP_STRUCT const * const tp);

TC_STRUCT const *tpGetCurrentSegment(TP_STRUCT * tp);

void cancel_probing();
void tpStopWithError(TP_STRUCT * tp, const char *fmt, ...)
    __attribute__((format(printf,2,3)));

const char *wait_type_as_str(WaitFlagIndex idx);

bool waiting_for_probe_power();

#ifdef __cplusplus
}				/* matches extern "C" for C++ */
#endif
#endif				/* TP_H */
