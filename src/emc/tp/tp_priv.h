#ifndef TP_PRIV_H
#define TP_PRIV_H

#include "tp_enums.h"
#include "tc_types.h"
#include "blendmath_types.h"
#include "tp_types.h"
#include "stdbool.h"
#include "error_util.h"

int tpCheckEndCondition(TC_STRUCT * const tc, double nominal_cycle_time, double distance_to_end, double v_next);

double tpGetCurrentVel(TP_STRUCT const * const tp, PmVector const * const v_current, int * pure_angular);

double tpGetRealAbsFeedScale(TP_STRUCT const * const tp,
                      TC_STRUCT const * const tc);

double tpGetRealAbsTargetVel(TP_STRUCT const * const tp,
                          TC_STRUCT const * const tc);

double tpGetRealMaxTargetVel(TP_STRUCT const * const tp, TC_STRUCT const * const tc);

double tpGetRealFinalVel(TP_STRUCT const * const tp,
                         TC_STRUCT const * const tc, const TC_STRUCT * nexttc);

void setSpindleOrigin(spindle_origin_t *origin, double position);

int tpCalculateRampAccel(
    TP_STRUCT const * const tp,
    TC_STRUCT * const tc,
    double * const acc,
    double * const vel_desired,
    double v_final);

double estimate_rigidtap_decel_distance(double vel, double uu_per_rev);

void tpUpdateRigidTapState(
    TP_STRUCT * const tp,
    TC_STRUCT * const tc,
    TC_STRUCT * const nexttc);

WaitFlagMask tpIsWaiting(TP_STRUCT const * const tp);
WaitFlagMask tpIsWaitingOnSegment(TP_STRUCT const * const tp, TC_STRUCT const *tc);

int tpUpdateMovementStatus(TP_STRUCT * const tp,
                           TC_STRUCT const * const tc,
                           TC_STRUCT const * const nexttc);

int tpHandleStopConditions(TP_STRUCT * const tp, TC_STRUCT * const tc, TC_STRUCT * const nexttc);

void tpSetRotaryUnlock(IndexRotaryAxis axis, int unlock);

int tpGetRotaryIsUnlocked(IndexRotaryAxis axis);

int tpCompleteSegment(TP_STRUCT * const tp,
                      TC_STRUCT * const tc);

tp_err_t tpCheckWaitConditions(TP_STRUCT * const tp, TC_STRUCT * const tc);

tp_err_t tpActivateSegment(TP_STRUCT * const tp, TC_STRUCT * const tc);

void tpSyncVelocityMode(TC_STRUCT * const tc);

void checkPositionSyncError(TP_STRUCT const *tp, TC_STRUCT const *tc);

void clearPositionSyncErrors();
void clearPosTrackingStatus();
void clearSpindleSyncStatus();

EndCondition checkEndCondition(double dt_plan,
    double distance_to_go,
    double v_current, double v_final_plan,
    double a_max);

double findSpindleDisplacement(
        double new_pos,
        spindle_origin_t origin
    );

double findSpindleVelocity(
        double spindle_velocity,
        spindle_origin_t origin
    );

bool spindleReversed(spindle_origin_t origin, double prev_pos, double current_pos);

void cmdSpindleScale(double scale);
void cmdSpindlePauseTimeout(int timeout_counts);

void reportTPAxisError(TP_STRUCT const *tp, unsigned failed_axes, const char *msg_prefix);


void tpSyncPositionMode(TP_STRUCT * const tp,
                        TC_STRUCT * const tc);

int tpUpdateStatusCommon(TP_STRUCT const * const tp);

int tpFindDisplacementForSegment(TP_STRUCT * const tp,
        TC_STRUCT * const tc, TC_STRUCT * const nexttc,
        UpdateCycleMode cycle_mode);

int tpCleanupAtEmptyQueue(TP_STRUCT * const tp);
int tpCleanupAfterAbort(TP_STRUCT * const tp);
#endif // TP_PRIV_H
