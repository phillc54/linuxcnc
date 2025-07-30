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
#ifndef MOTION_PLANNING_HH
#define MOTION_PLANNING_HH

#include "posemath.h"
#include "tc_types.h"
#include "tp_types.h"
#include "tp_enums.h"

#include "error_util.h"
#include "blendmath_types.h"

#define DEFAULT_TP_SUPERSAMPLE_RATE 1

#include <vector>

using SmoothingVector = std::vector<double>;

struct SmoothingData {
    // Start with a dummy endpoint to simplify some of the algorithms
    SmoothingData() :
#ifdef MOTION_PLANNING_DEBUG
        motion_line{0},
        unique_id{0},
        planning_state{TC_PLAN_UNTOUCHED},
        limiting_id{0},
        smoothed{false},
        touched{false},
        v_opt{0.0},
        s{0.0},
#endif
        ignore{false},
        ds{0.0},
        v_smooth{0.0},
        t{0.0},
        t_orig{0.0}
        {}
#ifdef MOTION_PLANNING_DEBUG
    std::vector<int> motion_line;
    std::vector<int> unique_id;
    std::vector<TCPlanningState> planning_state;
    std::vector<int> limiting_id;
    std::vector<bool> smoothed;
    std::vector<bool> touched;
    SmoothingVector v_opt;
    SmoothingVector s; // Mostly for debugging
#endif
    std::vector<bool> ignore;
    SmoothingVector ds;
    SmoothingVector v_smooth;
    SmoothingVector t;
    SmoothingVector t_orig;
};


extern "C" {

int tpUpdateConfig(TP_STRUCT * const tp);
int tpClearPlanning(TP_STRUCT * const tp);
int tpClearDIOs(TP_STRUCT * const tp);
double tpGetCycleTime(TP_STRUCT * const tp);

int tpSetId(TP_STRUCT * const tp, int id);
tc_unique_id_t tpGetNextUniqueId(TP_STRUCT * const tp);
void tcSetId(TP_STRUCT * const tp, TC_STRUCT * const tc, struct state_tag_t tag);
int tpGetQueuedId(TP_STRUCT const *tp);

// Functions to add new motions or modify settings for subsequent motions / command output

int tpSetTermCond(TP_STRUCT * const tp, tc_term_cond_t cond, double tolerance);
int tpSetBlendMode(TP_STRUCT * const tp, blend_mode_t blend_mode);

void tpRestoreWaitConditions(tp_planning_t *planning, TC_STRUCT *tc_to_skip);

bool tpCheckNeedsAtSpeed(tp_planning_t *planning, EMCMotionTypes motion_type);
bool tpCheckNeedsProbeReady(tp_planning_t *planning, EMCMotionTypes motion_type);

// Cache spindle state in TP for lookahead purposes
int tpSetSpindleOn(tp_planning_t &planner, spindle_cmd_t &cmd, bool wait_for_atspeed);
int tpSetSpindleOff(tp_planning_t &planner);
int tpSetSpindleSpeed(tp_planning_t planner, spindle_cmd_t & cmd);

int tpSetEnableFeedhold(tp_planning_t * planner, bool allow_feedhold);
int tpSetEnableFeedScale(tp_planning_t * planner, bool allow_feed_scale);
int tpSetEnableSpindleScale(tp_planning_t * planner, bool allow_spindle_scale);
int tpSetEnableAdaptiveFeed(tp_planning_t * planner, bool allow_adaptive_feed);

int tpSetNextWaitForProbeReady(tp_planning_t &planner);

int tpAddRigidTap(
    TP_STRUCT * const tp,
    const rigid_tap_cmd_t & rigid_tap,
    struct state_tag_t const &tag);

int tpAddLine(
    TP_STRUCT * const tp,
    EmcPose end,
    EMCMotionTypes canon_motion_type,
    double vel,
    double ini_maxvel,
    double acc,
    probe_mode_t probe_mode,
    IndexRotaryAxis indexrotary,
    struct state_tag_t const &tag);

int tpAddCircle(TP_STRUCT * const tp,
        EmcPose end,
        PmCartesian center,
        PmCartesian normal,
        int turn, double expected_angle_rad,
        EMCMotionTypes canon_motion_type,
        double vel,
        double ini_maxvel,
        double acc,
        double acc_normal,
        struct state_tag_t const &tag);

int tpAddDwell(TP_STRUCT * const tp,
    double time_sec, double delta_rpm,
    const state_tag_t &tag);

tp_err_t tpForceFinalizeQueue(TP_STRUCT * const tp);

EmcPose getMotionPlanningGoalPos(TP_STRUCT const  * const tp);
EmcPose tpGetCurrentPos(TP_STRUCT const  * const tp);

int tpSetSpindleSync(TP_STRUCT * const tp, double sync, int velocity_mode);

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end);
int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end); //gets called to place DIO toggles on the TC queue

// Internal functions (moved from tp_priv.h)

tp_err_t tpOptimizePlannedMotions(TP_STRUCT * const tp, int optimization_depth);

int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc);

int tpSetupSegmentBlend(TP_STRUCT * const tp, TC_STRUCT * const prev2_tc, TC_STRUCT * const prev_tc, TC_STRUCT * const tc);

tp_err_t tpCreateBiarcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const this_tc);

tp_err_t applyProfileCorrection(TP_STRUCT * tp, const TC_STRUCT * prev2_tc, TC_STRUCT * prev_tc, TC_STRUCT * this_tc);

double tpGetTangentKinkRatio(void);

bool isArcBlendFaster(TC_STRUCT const * const prev_tc, double expected_v_max);

void tpFallbackToTangent(TC_STRUCT * const prev_tc);

tp_err_t tpCreateLineLineBlend(TP_STRUCT * const tp,
                               TC_STRUCT * const prev_tc,
                               TC_STRUCT * const tc,
                               TC_STRUCT * const blend_tc);

int handlePrevTermCondition(TC_STRUCT *prev_tc, TC_STRUCT *tc);
int handleModeChange(TC_STRUCT *prev_tc, TC_STRUCT *tc);
int tpSetupSyncedIO(TP_STRUCT * const tp, TC_STRUCT * const tc);

tp_err_t tpFinalizeAndEnqueue(TP_STRUCT * const tp, TC_STRUCT * const tc, const PmVector * nominal_goal);

tp_err_t tpDoPeakSmoothing(SmoothingData &path_data, int optimization_depth, double t_window);

double tpComputeOptimalVelocity(TC_STRUCT const * const tc, TC_STRUCT const * const prev1_tc, int opt_step);

tp_err_t tpCheckBlendPrerequisites(
    TP_STRUCT const * const tp,
    TC_STRUCT * const prev_tc,
    TC_STRUCT * const tc,
    const char ** result);

TCIntersectType tpSetupTangent(
    TP_STRUCT const * const tp,
    TC_STRUCT * const prev_tc,
    TC_STRUCT * const tc);

int tpInitBlendArcFromAdjacent(
    TC_STRUCT const * const adjacent_tc,
    TC_STRUCT * const blend_tc,
    const BlendParameters * param,
    double vel,
    double ini_maxvel,
    double acc,
    tc_motion_type_t motion_type);

int find_max_element(double arr[], int sz);

}

#endif // MOTION_PLANNING_HH
