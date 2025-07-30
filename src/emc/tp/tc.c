/*!
********************************************************************
* Description: tc.c
*\brief Discriminate-based trajectory planning
*
*\author Derived from a work by Fred Proctor & Will Shackleford
*\author rewritten by Chris Radek
*
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include "rtapi.h"		/* rtapi_print_msg */
#include "rtapi_math.h"
#include "posemath.h"
#include "blendmath_types.h"
#include "emcpose.h"
#include "tc.h"
#include "tp_types.h"
#include "spherical_arc9.h"
#include "motion_types.h"

//Debug output
#include "tp_debug.h"
#include "tp_enums.h"
#include "tp_call_wrappers.h"


double tcGetMaxVelFromLength(TC_STRUCT const * const tc)
{
    double sample_maxvel = tc->target / (tc->cycle_time * TP_MIN_SEGMENT_CYCLES);
    return fmin(tc->maxvel_geom, sample_maxvel);
}

double tcGetAccelScale(const TC_STRUCT *tc)
{
    // Handle any acceleration reduction due to an approximate-tangent "blend" with the previous or next segment
    double a_scale = (1.0 - fmax(tc->kink_accel_reduce, tc->kink_accel_reduce_prev));
    return a_scale;
}

double tcGetOverallMaxAccel(const TC_STRUCT *tc)
{
    return tc->maxaccel * tcGetAccelScale(tc);
}

/**
 * Get acceleration for a tc based on the trajectory planner state.
 */
double tcGetTangentialMaxAccel(TC_STRUCT const * const tc)
{
    double a_scale = tcGetAccelScale(tc);

    // Reduce allowed tangential acceleration in circular motions to stay
    // within overall limits (accounts for centripetal acceleration while
    // moving along the circular path).
    if (tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_SPHERICAL) {
        //Limit acceleration for cirular arcs to allow for normal acceleration
        a_scale *= tc->acc_ratio_tan;
    }
    return tc->maxaccel * a_scale;
}

double tcGetFinalVelInternal(const TC_STRUCT * tc)
{
    double fv=0;
    __atomic_load(&tc->shared.final_vel, &fv, __ATOMIC_ACQUIRE);
    return fv;
}

double tcGetFinalVelLimitInternal(const TC_STRUCT * tc)
{
    double fv=0;
    __atomic_load(&tc->shared.final_vel_limit, &fv, __ATOMIC_ACQUIRE);
    return fv;
}

double tcGetPlanFinalVel(TC_STRUCT const * tc)
{
    TCPlanningState plan_level = __atomic_load_n(&tc->shared.optimization_state, __ATOMIC_ACQUIRE);
    switch (plan_level) {
    case TC_PLAN_UNTOUCHED:
        break;
    case TC_PLAN_GEOMETRY_PLANNED: // Geometry is final shape, optimizer will start to increase velocity limits
        return fmin(tcGetFinalVelInternal(tc), tcGetFinalVelLimitInternal(tc));
    }
    return 0.0;
}

double tcGetVLimit(TC_STRUCT const * const tc, double v_target, double v_limit_linear, double v_limit_angular)
{
    if ((tc->synchronized == TC_SYNC_POSITION)){
        // No limits applied during position sync
        return v_target;
    }
    switch (tc->motion_type) {
    case TC_LINEAR:
        return pmLine9VLimit(&tc->coords.line, v_target, v_limit_linear, v_limit_angular);
    case TC_CIRCULAR:
        return pmCircle9VLimit(&tc->coords.circle, v_target, v_limit_linear, v_limit_angular);
    case TC_SPHERICAL:
        return arc9VLimit(&tc->coords.arc, v_target, v_limit_linear, v_limit_angular);
    case TC_RIGIDTAP:
    case TC_DWELL:
        break;
        // for the non-synched portion of rigid tapping
    }
    return fmin(v_target, v_limit_linear);
}

/**
 * Calculate the distance left in the trajectory segment in the indicated
 * direction.
 */
double tcGetDistanceToGo(TC_STRUCT const * const tc, int direction)
{
    double distance;
    if (direction == TC_DIR_FORWARD) {
        // Return standard distance to go
        distance = tc->target - tc->progress;
    } else {
        // Reverse direction, distance from zero instead of target
        distance = tc->progress;
    }

    return distance;
}

double tcGetTarget(TC_STRUCT const * const tc, int direction)
{
    return (direction == TC_DIR_REVERSE) ? 0.0 : tc->target;
}

/*! tcGetPos() function
 *
 * \brief This function calculates the machine position along the motion's path.
 *
 * As we move along a TC, from zero to its length, we call this function repeatedly,
 * with an increasing tc->progress.
 * This function calculates the machine position along the motion's path
 * corresponding to the current progress.
 * It gets called at the end of tpRunCycle()
 *
 * @param    tc    the current TC that is being planned
 *
 * @return	 PmVector   returns a position (\ref PmVector = datatype carrying XYZABC information
 */

PmVector tcGetPos(TC_STRUCT const * const tc) {
    return tcGetPosReal(tc, tc->progress);
}

PmVector tcGetPosReal(TC_STRUCT const * const tc, double progress)
{
    PmCartesian xyz;
    PmCartesian abc;
    PmCartesian uvw;
    PmVector pos = {};

    switch (tc->motion_type){
        case TC_RIGIDTAP:
            pmCartLinePoint(&tc->coords.rigidtap.actual_xyz, progress, &xyz);
            // no rotary move allowed while tapping
            abc = tc->coords.rigidtap.abc;
            uvw = tc->coords.rigidtap.uvw;
            CartToVec(&xyz, &abc, &uvw, &pos);
            break;
        case TC_LINEAR:
            pmLine9Point(&tc->coords.line, progress, &pos);
            break;
        case TC_CIRCULAR:
            pmCircle9Point(&tc->coords.circle, progress, &pos);
            break;
        case TC_SPHERICAL:
            arc9Point(&tc->coords.arc,
                    progress,
                    &pos);
            break;
        case TC_DWELL:
            pos = tc->coords.dwell.dwell_pos;
            break;
    }
    return pos;
}


/**
 * Set the terminal condition (i.e. blend or stop) for the given motion segment.
 * Also sets flags on the next segment relevant to blending (e.g. parabolic blend sets the blend_prev flag).
 */
int tcSetTermCond(TC_STRUCT *tc, tc_term_cond_t term_cond)
{
    if (!tc) {
        return -1;
    }

    tc->blend_mode.mode = term_cond;
    return 0;
}

// Helper functions to convert enums to pretty-print for debug output

const char *tcTermCondAsString(tc_term_cond_t c)
{
    switch (c)
    {
        case TC_TERM_COND_STOP:
            return "EXACT_STOP";
        case TC_TERM_COND_EXACT:
            return "EXACT_PATH";
        case TC_TERM_COND_PARABOLIC:
            return "PARABOLIC";
        case TC_TERM_COND_TANGENT:
            return "TANGENT";
    }
    return "NONE";
}

const char *tcCanonMotionTypeAsString(EMCMotionTypes c)
{
    switch (c)
    {
    case EMC_MOTION_TYPE_NONE:
        return "None";
    case EMC_MOTION_TYPE_RIGIDTAP:
        return "RigidTap";
    case EMC_MOTION_TYPE_TRAVERSE:
        return "Rapid";
    case EMC_MOTION_TYPE_FEED:
        return "Feed";
    case EMC_MOTION_TYPE_ARC:
        return "Arc";
    case EMC_MOTION_TYPE_TOOLCHANGE:
        return "ToolChange";
    case EMC_MOTION_TYPE_PROBING:
        return "Probing";
    case EMC_MOTION_TYPE_INDEXROTARY:
        return "IndexRotary";
    }
    return "NONE";
}

const char *tcMotionTypeAsString(tc_motion_type_t c)
{
    switch (c)
    {
        case TC_LINEAR:
            return "Linear";
        case TC_CIRCULAR:
            return "Circular";
        case TC_RIGIDTAP:
            return "RigidTap";
        case TC_SPHERICAL:
            return "SphericalArc";
        case TC_DWELL:
            return "Dwell";
    }
    return "NONE";
}

const char *tcSyncModeAsString(tc_spindle_sync_t c)
{
    switch (c)
    {
        case TC_SYNC_NONE:
            return "sync_none";
        case TC_SYNC_VELOCITY:
            return "sync_velocity";
        case TC_SYNC_POSITION:
            return "sync_position";
    }
    return "NONE";
}

void tcSetOptimizationState(TC_STRUCT *tc, TCPlanningState state)
{
    //tc_debug_print("Setting optimization state unique_id %lld to %d\n", tc->unique_id, state);
    __atomic_store_n(&tc->shared.optimization_state, state, __ATOMIC_RELEASE);
}

TCPlanningState tcGetOptimizationState(TC_STRUCT const *tc)
{
    return  __atomic_load_n(&tc->shared.optimization_state, __ATOMIC_ACQUIRE);
}

