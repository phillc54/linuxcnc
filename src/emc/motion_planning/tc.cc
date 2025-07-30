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

#ifdef TP_LOG_PARSING
#undef TP_DEBUG
#endif

#include "rtapi.h"		/* rtapi_print_msg */
#include "rtapi_math.h"
#include "posemath.h"
#include "blendmath.hh"
#include "emcpose.h"
#include "tc.hh"
#include "tp_types.h"
#include "spherical_arc9.hh"
#include "motion_types.h"

#include "motion_planning_config.hh"
#include "emc_messages.hh"

//Debug output
#include "tp_debug.h"
#include "tp_enums.h"
#include "tp_call_wrappers.hh"

#include <algorithm>

double tcGetMaxVelFromLength(TC_STRUCT const * const tc)
{
    double sample_maxvel = tc->target / (tc->cycle_time * TP_MIN_SEGMENT_CYCLES);
    return fmin(tc->maxvel_geom, sample_maxvel);
}

double tcGetPlanTargetVel(
    TC_STRUCT const * const tc)
{
    // NOTE: emccanon pre-computes a nominal velocity for synched moves based on the planned spindle velocity
    // This function used to be more involved before the redesign to use canon velocity value...
    return tc->reqvel;
}

/**
 * Planning-time maximum velocity for the given segment.
 * This function ignores runtime conditions like if a segment is on final deceleration, actively blending, etc.
 */
double tcGetPlanMaxTargetVel(
    TC_STRUCT const * const tc,
    double max_feed_scale)
{
    double v_max = tcGetMaxVelFromLength(tc);
    double v_target = tcGetPlanTargetVel(tc);
    double v_max_target = v_target * max_feed_scale;
    // Clip maximum velocity by the segment's own maximum velocity
    return fmin(v_max_target, v_max);
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


int tcSetKinkProperties(TC_STRUCT *prev_tc, TC_STRUCT *tc, double kink_vel, double accel_reduction)
{
  // NOTE: use_kink field is not set until later, if we choose tangent blending without an arc connector
  prev_tc->kink_vel = kink_vel;
  prev_tc->kink_accel_reduce = fmax(accel_reduction, prev_tc->kink_accel_reduce);
  tc->kink_accel_reduce_prev = fmax(accel_reduction, tc->kink_accel_reduce_prev);

  return 0;
}

/**
 * Calulate the unit tangent vector at the start of a move for any segment.
 */
int tcGetStartTangentUnitVector(TC_STRUCT const * const tc, PmVector * const out)
{
    
    return tcGetTangentUnitVector(tc, 0, out);
}

/**
 * Calculate the unit tangent vector at the end of a move for any segment.
 */
int tcGetEndTangentUnitVector(
    TC_STRUCT const * const tc,
    PmVector * const out) {

    
    return tcGetTangentUnitVector(tc, tc->target, out);
}

int tcGetTangentUnitVector(TC_STRUCT const * const tc, double progress, PmVector * const out) {
    static const PmCartesian zero={0,0,0};
    switch (tc->motion_type) {
        case TC_LINEAR:
            *out = tc->coords.line.uVec;
            return 0;
        case TC_RIGIDTAP:
            // Augment with zero vector, no re-scaling necessary
            return CartToVec(&tc->coords.rigidtap.nominal_xyz.uVec, &zero, &zero, out);
        case TC_DWELL:
            return -1;
        case TC_CIRCULAR:
        {
            return pmCircle9TangentVector(
                &tc->coords.circle,
                progress,
                out);
        }
        case TC_SPHERICAL:
        {
            double t = progress / tc->target;
            return arc9Tangent(&tc->coords.arc, t, out);
        }
    }
    emcOperatorError("Invalid motion type %d!\n",tc->motion_type);
    return TP_ERR_FAIL;
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

int tcGetPos(TC_STRUCT const * const tc, PmVector * const out) {
    tcGetPosReal(tc, tc->progress, out);
    return 0;
}

int tcGetStartpoint(TC_STRUCT const * const tc, PmVector * const pos)
{
    PmCartesian xyz;
    PmCartesian abc;
    PmCartesian uvw;

    switch (tc->motion_type){
        case TC_RIGIDTAP:
            xyz = tc->coords.rigidtap.nominal_xyz.start;
            abc = tc->coords.rigidtap.abc;
            uvw = tc->coords.rigidtap.uvw;
            return CartToVec(&xyz, &abc, &uvw, pos);
        case TC_LINEAR:
            *pos = tc->coords.line.start;
            return 0;
        case TC_CIRCULAR:
            pmCircleStartPoint(&tc->coords.circle.xyz, &xyz);
            abc = tc->coords.circle.abc.start;
            uvw = tc->coords.circle.uvw.start;
            return CartToVec(&xyz, &abc, &uvw, pos);
        case TC_SPHERICAL:
            return arc9Point(&tc->coords.arc,
                    0,
                    pos);
        case TC_DWELL:
            *pos = tc->coords.dwell.dwell_pos;
            return TP_ERR_OK;
    }

    return TP_ERR_FAIL;
}

int tcGetEndpoint(TC_STRUCT const * const tc, PmVector * const out) {
    tcGetPosReal(tc, tc->target, out);
    return 0;
}

int tcGetPosReal(TC_STRUCT const * const tc, double progress, PmVector * const pos)
{
    PmCartesian xyz;
    PmCartesian abc;
    PmCartesian uvw;

    switch (tc->motion_type){
        case TC_RIGIDTAP:
            pmCartLinePoint(&tc->coords.rigidtap.actual_xyz, progress, &xyz);
            // no rotary move allowed while tapping
            abc = tc->coords.rigidtap.abc;
            uvw = tc->coords.rigidtap.uvw;
            return CartToVec(&xyz, &abc, &uvw, pos);
        case TC_LINEAR:
            return pmLine9Point(&tc->coords.line, progress, pos);
        case TC_CIRCULAR:
            return pmCircle9Point(&tc->coords.circle, progress, pos);
        case TC_SPHERICAL:
            return arc9Point(&tc->coords.arc,
                    progress,
                    pos);
        case TC_DWELL:
            *pos = tc->coords.dwell.dwell_pos;
            return TP_ERR_OK;
    }
    return -1;
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

int tcFindBlendTolerance(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    double * const T_blend)
{
    const double tolerance_ratio = 0.5;
    // NOTE: the next segment's correction distance expands the allowed tolerance (since we have to be able to blend by that much or it will be worse than nothing).
    double T1 = fmax(tc->blend_mode.explicit_tolerance + tc->correction_dist, 0.0);
    double nominal_tolerance = T1; // Tolerance for blending between segment n-1 and segment n is controlled by the nth segment's tolerance
    // This means that the following example code will work as expected:
    // N1 G1 X1 Y0
    // N2 G64 P100.0
    // N3 G1 Y1
    // N4 G64 P0.01
    // N5 G1 X2
    //
    // The blend between N1 and N3 will have a large tolerance, and the blend between N3 and N5 will have a small tolerance

    // Additionally limit the tolerance to prevent over-consuming long segments with sharp corners
    // so that we always consuming half a segment or less (parabolic equivalent)
    double blend_tolerance = fmin(fmin(nominal_tolerance,
                prev_tc->nominal_length * tolerance_ratio),
            tc->nominal_length * tolerance_ratio);
    *T_blend = blend_tolerance;

    return 0;
}

/**
 * Initialize a new trajectory segment with common parameters.
 *
 * NOTE: this function only sets default values that are non-zero. Make sure
 * the struct is properly initialized BEFORE calling this function.
 */
int tcInit(TC_STRUCT * const tc,
        tc_motion_type_t motion_type,
        int canon_motion_type,
        double cycle_time,
        unsigned char enables)
{

    /** Motion type setup */
    tc->motion_type = motion_type;
    tc->canon_motion_type = canon_motion_type;

    /** Segment settings passed down from interpreter*/
    tc->enables = enables;
    tc->cycle_time = cycle_time;

    /** Segment settings (given values later during setup / optimization) */
    tc->indexrotary = INDEX_NONE;
    tc->needs_probe_ready = false;
    tc->needs_spindle_atspeed = false;

    tc->active_depth = 1;

    tc->acc_ratio_tan = BLEND_ACC_RATIO_TANGENTIAL;
    tc->accel_scale = emcmotConfig->joint_filter_cfg.acceleration_scale;

    return TP_ERR_OK;
}

/**
 * Set kinematic properties for a trajectory segment.
 */
int tcSetupMotion(TC_STRUCT * const tc,
        double vel,
        double ini_maxvel,
        double acc)
{
    

    tc->maxaccel = acc;

    tc->maxvel_geom = ini_maxvel;

    tc->reqvel = vel;

    return TP_ERR_OK;
}

int tcSetupState(TC_STRUCT * const tc, TP_STRUCT const * const tp)
{
    tc->blend_mode = tp->planner.blend_mode;
    tc->synchronized = tp->planner.synchronized;
    tc->uu_per_rev = tp->planner.uu_per_rev;
    return TP_ERR_OK;
}

int tcUpdateCircleAccRatio(TC_STRUCT * tc, double v_max_path)
{
    if (tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_OK;
    }
    if (tc->coords.circle.xyz_ratio <= 0.0) {
        return TP_ERR_FAIL;
    }

    double v_max3 = v_max_path * tc->coords.circle.xyz_ratio;
    double eff_radius = pmCircleEffectiveMinRadius(&tc->coords.circle.xyz);
    double v_max_filter = std::min(findMaxVelocityWithFilteringActive(eff_radius), v_max3);

    CircleAccLimits limits = pmCircleActualMaxVel(eff_radius,
                                                 v_max_filter,
                                                 tc->acc_normal_max * tcGetAccelScale(tc));
    tc->maxvel_geom = limits.v_max / tc->coords.circle.xyz_ratio;
    tc->acc_ratio_tan = limits.acc_ratio;
    return TP_ERR_OK;
}

/**
 * "Finalizes" a segment so that its length can't change.
 * By setting the finalized flag, we tell the optimizer that this segment's
 * length won't change anymore. Since any blends are already set up, we can
 * trust that the length will be the same, and so can use the length in the
 * velocity optimization.
 */
int tcFinalizeLength(TC_STRUCT * const tc, double max_feed_override)
{
    //Apply velocity corrections
    if (!tc) {
        return TP_ERR_OK;
    }

    tp_debug_json5_log_start(tcFinalizeLength);
#ifdef TP_DEBUG
    print_json5_long_long_("unique_id", tc->unique_id);
#endif
    if (tcGetOptimizationState(tc) > TC_PLAN_UNTOUCHED) {
        tp_debug_json5_log_end("planning: tc unique_id %lld already finalized", tc->unique_id);
        return TP_ERR_OK;
    }

#ifdef TP_DEBUG
    double maxvel_old = tc->maxvel_geom;
#endif

    CHP(tcUpdateCircleAccRatio(tc, tcGetPlanMaxTargetVel(tc, max_feed_override)));

    switch (tc->blend_mode.mode)
    {
    case TC_TERM_COND_PARABOLIC:
        tp_debug_print("Finalizing an unblended segment means forcing exact stop mode");
        // Intentional fallthrough
    case TC_TERM_COND_STOP:
         // Confusing but "exact path" condition is replaced with tangent if
         // it's fast enough. Getting exact path mode here means tangency check
         // failed.
    case TC_TERM_COND_EXACT:
        tcSetTermCond(tc, TC_TERM_COND_STOP);
        break;
    case TC_TERM_COND_TANGENT:
        break;
    }
    tcSetOptimizationState(tc, TC_PLAN_GEOMETRY_PLANNED);

#ifdef TP_DEBUG
    print_json5_tc_id_data_(tc);
    print_json5_string_("motion_type", tcMotionTypeAsString((tc_motion_type_t)tc->motion_type));
    print_json5_string_("term_cond", tcTermCondAsString((tc_term_cond_t)tc->blend_mode.mode));
    print_json5_double(maxvel_old);
    print_json5_double_("maxvel", tc->maxvel_geom);
    print_json5_double_("acc_ratio_tan", tc->acc_ratio_tan);
#endif

    tp_debug_json5_log_end("tc %llu finalized", tc->unique_id);
    return TP_ERR_OK;
}

int pmRigidTapInit(PmRigidTap * const tap,
        PmVector const * const start,
        PmVector const * const end,
        double reversal_scale)
{
    PmCartesian start_xyz, end_xyz;
    PmCartesian abc, uvw;

    //Slightly more allocation this way, but much easier to read
    VecToCart(start, &start_xyz, &tap->abc, &tap->uvw);
    VecToCart(end, &end_xyz, &abc, &uvw);

    // Setup XYZ motion
    pmCartLineInit(&tap->nominal_xyz, &start_xyz, &end_xyz);
    tap->actual_xyz = tap->nominal_xyz;

    // Setup initial tap state
    tap->reversal_target = tap->nominal_xyz.tmag;
    tap->reversal_scale = reversal_scale;
    tap->state = RIGIDTAP_TAPPING;

    if (!pmCartCartCompare(&tap->abc, &abc) || !pmCartCartCompare(&tap->uvw, &uvw)) {
        return TP_ERR_RANGE;
    }
    return TP_ERR_OK;

}

#if 0

/**
 * Given a PmCircle and a circular segment, copy the circle in as the XYZ portion of the segment, then update the motion parameters.
 * NOTE: does not yet support ABC or UVW motion!
 */
int tcSetCircleXYZ(TC_STRUCT * const tc, PmCircle const * const circ)
{

    //Update targets with new arc length
    if (!circ || tc->motion_type != TC_CIRCULAR) {
        return TP_ERR_FAIL;
    }

    // Store the new circular segment (or use the current one)

    if (!circ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "SetCircleXYZ missing new circle definition\n");
        return TP_ERR_FAIL;
    }

    tc->coords.circle.xyz = *circ;
    // Update the arc length fit to this new segment
    findSpiralArcLengthFit(&tc->coords.circle.xyz, &tc->coords.circle.fit, TP_ANGLE_EPSILON);

    // compute the new total arc length using the fit and store as new
    // target distance
    tc->target = pmCircle9Length(&tc->coords.circle);

    return TP_ERR_OK;
}
#endif

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

int tcSetLine9(TC_STRUCT * const tc, PmLine9 const * const line9)
{
    if (tc->motion_type != TC_LINEAR) { return -1; }
    tc->coords.line = *line9;
    tc->target = pmLine9Length(line9);
    return 0;
}

int tcSetCircle9(TC_STRUCT * const tc, PmCircle9 const * const circle9)
{
    if (tc->motion_type != TC_CIRCULAR) { return -1; }
    tc->coords.circle = *circle9;
    tc->target = circle9->total_length;
    return 0;
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

double tcGetFinalVelLimit(const TC_STRUCT * tc)
{
    double fv_out=0;
    __atomic_load(&tc->shared.final_vel_limit, &fv_out, __ATOMIC_ACQUIRE);
    return fv_out;
}

void tcSetFinalVelLimit(TC_STRUCT * tc, double v_final_limit_new)
{
    double v_final_limit_old=0;
    __atomic_exchange(&tc->shared.final_vel_limit, &v_final_limit_new, &v_final_limit_old, __ATOMIC_ACQUIRE);

    if (v_final_limit_new > 0 && fabs(v_final_limit_old - v_final_limit_new) < 1e-6) {
        tc->internal.found_final_vel_limit = true;
    }
}

double tcGetFinalVel(const TC_STRUCT * tc)
{
    double fv_out=0;
    __atomic_load(&tc->shared.final_vel, &fv_out, __ATOMIC_ACQUIRE);
    return fv_out;
}

void tcSetFinalVel(TC_STRUCT * tc, double final_vel)
{
    __atomic_store(&tc->shared.final_vel, &final_vel, __ATOMIC_RELEASE);
}
