/********************************************************************
* Description: tp.c
*   Trajectory planner based on TC elements
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/
#include "rtapi.h"              /* rtapi_print_msg */
#include "posemath.h"           /* Geometry types & functions */
#include "tc.h"
#include "tp.h"
#include "tcq.h"
#include "emcpose.h"
#include "rtapi_math.h"
#include "mot_priv.h"
#include "motion_debug.h"
#include "motion_types.h"
#include "motion_type_defaults.h"
#include "spherical_arc9.h"
#include "blendmath_types.h"
#include "math_util.h"
#include "joint_util.h"
#include "string.h"
#include "tp_priv.h"
#include "motion_shared.h"
#include "tp_call_wrappers.h"
#include "error_util.h"
#ifdef UNIT_TEST
#include "stdio.h"
#endif

// Mark strings for translation, but defer translation to userspace
#define _(s) (s)


// Absolute maximum distance (in revolutions) to add to the end of a rigidtap move to account for spindle reversal
static const double RIGIDTAP_MAX_OVERSHOOT_REVS = 10.0;

/**
 * @section tpdebugflags TP debugging flags
 * Enable / disable various debugging functions here.
 * These flags control debug printing from RTAPI. These functions are
 * admittedly kludged on top of the existing rtapi_print framework. As written,
 * though, it's an easy way to selectively compile functions as static or not,
 * and selectively compile in assertions and debug printing.
 */

#include "tp_debug.h"

#ifdef TP_DEBUG
static const bool _tp_debug = true;
#else
static const bool _tp_debug = false;
#endif

#ifdef TC_DEBUG
static const bool _tc_debug = true;
#else
static const bool _tc_debug = false;
#endif

/** static function primitives (ugly but less of a pain than moving code around)*/
static inline LineDescriptor formatLinePrefix(struct state_tag_t const *tag);

void reportTPAxisError(TP_STRUCT const *tp, unsigned failed_axes, const char *msg_prefix)
{
    if (failed_axes)
    {
        AxisMaskString failed_axes_str = axisBitMaskToString(failed_axes);
        rtapi_print_msg(RTAPI_MSG_ERR, "%s, ax%cs [%s], line %d, %g sec\n",
                        msg_prefix ?: "unknown error",
                        failed_axes_str.len > 1 ? 'e' : 'i', // Ugly workaround for english grammar
                        failed_axes_str.axes,
                        tp->exec.execTag.fields[GM_FIELD_LINE_NUMBER],
                        tp->exec.time_elapsed_sec);
    }
}

static bool needConsistencyCheck(ConsistencyCheckMask mask)
{
    return emcmotConfig->consistencyCheckConfig.extraConsistencyChecks & mask;
}


/**
 * Get a segment's feed scale based on the current planner state and emcmotStatus.
 * @note depends on emcmotStatus for system information.
 */
double tpGetRealAbsFeedScale(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {
    if (!tc) {
        return 0.0;
    }

    double net_feed_scale = fabs(emcmotStatus->net_feed_scale);

    //All reasons to disable feed override go here
    bool pausing = tp->exec.pausing && (tc->synchronized == TC_SYNC_NONE || tc->synchronized == TC_SYNC_VELOCITY);
    bool aborting = tp->exec.aborting;
    if (pausing)  {
        return 0.0;
    } else if (aborting) {
        return 0.0;
    } else if (tc->synchronized == TC_SYNC_POSITION ) {
        return 1.0;
    } else {
        return net_feed_scale;
    }
}

/**
 * Get target velocity for a tc based on the trajectory planner state.
 * This gives the requested velocity, capped by the segments maximum velocity.
 * @note returns the magnitude of velocity (reverse run is handled at a higher level)
 */
double tpGetRealAbsTargetVel(TP_STRUCT const * const tp,
        TC_STRUCT const * const tc) {

    if (!tc) {
        return 0.0;
    }

    // Get the maximum allowed target velocity, and make sure we're below it
    return fmin(tc->target_vel * tpGetRealAbsFeedScale(tp,tc), tpGetRealMaxTargetVel(tp, tc));
}

/**
 * Get the worst-case target velocity for a segment based on the trajectory planner state.
 * Note that this factors in the user-specified velocity limit.
 */
double tpGetRealMaxTargetVel(TP_STRUCT const * const tp, TC_STRUCT const * const tc)
{
    if (!tc) {
        return 0.0;
    }

    double max_scale = (tc->synchronized == TC_SYNC_POSITION) ? 1.0 : emcmotConfig->maxFeedScale;

    double v_max = tcGetMaxVelFromLength(tc);

    // Get maximum reachable velocity from max feed override
    double v_max_target = tc->target_vel * max_scale;

    /* Check if the cartesian velocity limit applies and clip the maximum
     * velocity. The vLimit is from the max velocity slider, and should
     * restrict the maximum velocity during non-synced moves and velocity
     * synchronization. However, position-synced moves have the target velocity
     * computed in the TP, so it would disrupt position tracking to apply this
     * limit here.
     */
    double v_limited = tcGetVLimit(tc, v_max_target, tp->exec.vLimit, tp->exec.vLimitAng);
    return fmin(v_limited, v_max);
}


/**
 * Get final velocity for a tc based on the trajectory planner state.
 * This function factors in the feed override and TC limits. It clamps the
 * final velocity to the maximum velocity and the next segment's target velocity
 */
double tpGetRealFinalVel(
    TP_STRUCT const * const tp,
    TC_STRUCT const * const tc,
    TC_STRUCT const *nexttc) {
    /* If we're stepping, then it doesn't matter what the optimization says, we want to end at a stop.
     * If the term_cond gets changed out from under us, detect this and force final velocity to zero
     */
    if (emcmotStatus->stepping || tc->blend_mode.mode != TC_TERM_COND_TANGENT || tp->exec.reverse_run) {
        return 0.0;
    }
    
    // NOTE: during optimization, the final velocity is restricted to be reachable by the next segment
    double v_plan = tcGetPlanFinalVel(tc);
    double v_target_this = tpGetRealAbsTargetVel(tp, tc);
    double v_limited = fmin(v_plan, v_target_this);

    if (nexttc) {
        double v_target_next = tpGetRealAbsTargetVel(tp, nexttc);
        return fmin(v_limited, v_target_next);
    } else {
        return v_limited;
    }
}

/**
 * Set up a spindle origin based on the current spindle COMMANDED direction and the given position.
 *
 * The origin is used to calculate displacements used in spindle position tracking.
 * The direction is stored as part of the origin to prevent discontinuous
 * changes in displacement due to sign flips
 */
void setSpindleOrigin(spindle_origin_t *origin, double position)
{
    if (!origin) {
        return;
    }
    origin->position = position;
    origin->direction = get_spindle_command_direction(emcmotStatus);
}

void updateSpindlePositionFromProgress(spindle_origin_t *origin, TC_STRUCT const * const tc)
{
    if (!origin || !tc) {
        return;
    }
    origin->position += tc->progress * origin->direction / tc->uu_per_rev;
    origin->direction = get_spindle_command_direction(emcmotStatus);
}

/**
 * @section tpaccess tp class-like API
 */

/**
 * Create the trajectory planner structure with an empty queue.
 */
int tpCreate(TP_STRUCT * const tp, int _queueSize)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (_queueSize <= 0) {
        tp->planner.queue_size_config = TP_DEFAULT_QUEUE_SIZE;
    } else {
        tp->planner.queue_size_config = _queueSize;
    }

    /* create the queue */
    if (-1 == tcqCreate(&tp->queue, tp->planner.queue_size_config)) {
        return TP_ERR_FAIL;
    }

    tp->planner.nextUniqueId = 0;
    /* init the rest of our data */
    return tpInit(tp);
}

/**
 * Clears any potential DIO toggles and anychanged.
 * If any DIOs need to be changed: dios[i] = 1, DIO needs to get turned on, -1
 * = off
 */
int tpClearDIOs(TP_STRUCT * const tp) {
    //XXX: All IO's will be flushed on next synced aio/dio! Is it ok?
    int i;
    tp->planner.syncdio.anychanged = 0;
    tp->planner.syncdio.dio_mask = 0;
    tp->planner.syncdio.aio_mask = 0;
    for (i = 0; i < num_dio; i++) {
        tp->planner.syncdio.dios[i] = 0;
    }
    for (i = 0; i < num_aio; i++) {
        tp->planner.syncdio.aios[i] = 0;
    }

    return TP_ERR_OK;
}

void clearPosTrackingStatus()
{
    emcmotStatus->pos_tracking_error = 0;
    emcmotStatus->pos_tracking_velocity = 0;
}

/**
 * "Soft initialize" the trajectory planner tp.
 * This is a "soft" initialization in that TP_STRUCT configuration to
 * reflect the fact that the TP for the moment is done processing, and ready
 * for new motions. The TP is currently idle, so the execution state is updated
 * to reflect this (zero velocity, no active state tag, etc.). However, the
 * TP's goal pose and other planner-side settings are left unchanged. In
 * theory, motion planning in userspace could be doing a big expensive planning
 * operation and trickling out new motions in bursts. As long as each motion
 * executes to completion, no changes to the planner state are required here.
 *
 * @note Originally, this function would have forced tp->goalPos =
 * tp->currentPos. However, this is no longer necessary now that motion drops
 * into free mode when program execution is completed or aborted. Whenever
 * motion mode changes between FREE, TELEOP, or COORD, TP goal position is
 * updated, so any future planning in userspace motion planning library will
 * have the correct position.
 *
 *
 */
int tpCleanupAtEmptyQueue(TP_STRUCT * const tp)
{
    if (!tp) {
        return 0;
    }

    emcmotStatus->tcqlen = 0; // By definition at an empty queue

    // NOTE: no longer need to set goal position to current position here because of the new abort handshake
    // An empty queue with no other context should be able to act like a long pause
    // tp->goalPos = tp->exec.currentPos;
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->excess_vel = 0.0;
    emcmotStatus->current_vel = 0.0;
    tp->exec.currentVel = PmVector_zero;
    emcmotStatus->distance_to_go = 0.0;
    ZERO_EMC_POSE(emcmotStatus->dtg);

    tp->exec.execId = 0;
    tp->exec.nextexecId = 0;
    static const struct state_tag_t empty_tag = {};
    tp->exec.execTag = empty_tag;
    tp->exec.motionType = 0;
    tp->exec.activeDepth = 0;

    // Note: canonically motion can only pause if it's not empty, so an empty queue must clear this
    tp->exec.pausing = 0;

    emcmotStatus->enables_queued = emcmotStatus->enables_new;

    emcmotStatus->dwell_time_remaining = 0.0;
    emcmotStatus->cutter_comp_phase = 0;
    // Clear reported spindle synchronization and related status
    clearSpindleSyncStatus();
    clearPosTrackingStatus();

    emcmotStatus->rigid_tap_state = RIGIDTAP_INACTIVE;

    // Clear any waits for spindle index / at-speed (since the motions that would be waiting no longer exist)
    for (int k=0; k < MAX_WAIT_INDICES; ++k) {
        tp->exec.waiting[k] = MOTION_INVALID_ID;
    }
    emcmotStatus->tp_waiting = 0;

    return 0;
}

int tpResetAtModeChange(TP_STRUCT * const tp)
{
    // Force full cleanup of motion queue and reset of goal position.
    tcqReset(&tp->queue);
    tp->goalPos = tp->exec.currentPos;

    tpCleanupAtEmptyQueue(tp);

    // Any time the mode changes, the TP is no longer in control of the motion being "in position", so it's safe to clear the drain counter.
    tp->exec.joint_filter_drain_counter = 0;

    // Finally, do any abort cleanup and force the machine out of coordinated mode so it enables in a consistent state.
    return tpCleanupAfterAbort(tp);
}

/**
 * With asynchronous planning, this is safe to call only in the following conditions:
 * 1) motion is not in coord mode (so TP state has no effect on motion output).
 * 2) motion is about to leave coord mode (so it won't execute another update with the current queue)
 *
 * @warning without these precautions, userspace may be in the process of
 * queueing additional motions, and assumptions about goal / start position
 * will be violated.
 */
int tpCleanupAfterAbort(TP_STRUCT * const tp)
{
    if (!tp) {
        return 0;
    }

    tp->exec.aborting = 0;
    cancel_probing();

    // Clear out internal trackers for elapsed time
    tp->exec.time_elapsed_sec = 0.0;
    tp->exec.time_elapsed_ticks = 0;

    // Force reset to forward run direction (all reverse history is discarded at abort / stop)
    tp->exec.reverse_run = TC_DIR_FORWARD;

#ifdef TP_DEBUG
    PmCartesian axis_vel_limit = getXYZVelBounds();
    PmCartesian axis_accel_limit = getXYZAccelBounds();
    print_json5_log_start(tpCleanupAfterAbort);
    print_json5_PmCartesian(axis_vel_limit);
    print_json5_PmCartesian(axis_accel_limit);
    print_json5_log_end();
#endif

    // Kick us out of coordinated mode in the next control update since we
    // can't safely continue without a handshake from userspace
    if (GET_MOTION_COORD_FLAG()) {
        emcmotStatus->request_mode = EMCMOT_MOTION_FREE;
    }
    // NOTE: synchdio flags are cleared if needed when TP is stopped / aborted
    return 0;
}

/**
 * Fully initialize the tp structure.
 * Sets tp configuration to default values and calls tpClear to create a fresh,
 * empty queue.
 */
int tpInit(TP_STRUCT * const tp)
{
    tp->cycleTime = 0.0;
    tp->superSampleRate = 1; // No super-sampling by default
    //Velocity limits
    tp->exec.vLimit = 0.0;
    tp->exec.vLimitAng = 0.0;

    tp->exec.joint_filter_drain_counter = 0;

    setSpindleOrigin(&tp->exec.spindle_cmd.origin, 0.0);
    tp->exec.spindle_cmd.trigger_revs = 0;

    tp->exec.reverse_run = TC_DIR_FORWARD;

    // Initialize the current state (used during tpClear to initialize other state)
    tp->exec.currentPos = PmVector_zero;
    tp->exec.currentVel = PmVector_zero;

    // Only set up TP defaults for tolerance once (after that, it must match what interp / task specifies)
    tp->planner.blend_mode.mode = TC_TERM_COND_PARABOLIC;
    tp->planner.blend_mode.explicit_tolerance = 1e99;
    tp->planner.blend_mode.req_min_radius = 0.;
    tp->planner.blend_mode.contour_tolerance = 0.;

    tp->planner.enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;

    // Clear / init the queue at startup
    return tpCleanupAfterAbort(tp); // Safe to call at init time since nothing is moving
}

/**
 * Set the cycle time for the trajectory planner.
 */
int tpSetCycleTime(TP_STRUCT * const tp, double secs, int supersamples)
{
    if (0 == tp || secs <= 0.0) {
        return TP_ERR_FAIL;
    }

    tp->cycleTime = secs / supersamples;
    tp->superSampleRate = supersamples;

    return TP_ERR_OK;
}

double tpGetCycleTime(TP_STRUCT * const tp)
{
    // NOTE: Overall cycle time for the servo loop is the base cycle time * number of loop iterations the TP runs
    return tp->cycleTime * tp->superSampleRate;
}

/**
 * Set the maximum velocity for linear and rotary-only moves.
 * I think this is the [TRAJ] max velocity. This should be the max velocity of
 * const the TOOL TIP, not necessarily any particular axis. This applies to
 * subsequent moves until changed.
 */
int tpSetVlimit(TP_STRUCT * const tp, double vLimit, double vLimitAng)
{
    if (!tp) return TP_ERR_FAIL;

    tp->exec.vLimit = fmax(vLimit, 0.0);
    tp->exec.vLimitAng = fmax(vLimitAng, 0.0);
    tc_pdebug_print("Setting Vlimit %f %f\n", tp->exec.vLimit, tp->exec.vLimitAng);

    return TP_ERR_OK;
}

/** Returns the id of the last motion that is currently
  executing.*/
int tpGetExecId(const TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    return tp->exec.execId;
}

int tpGetCompletedId(const TP_STRUCT * const tp)
{
    if (!tp) {
        return 0;
    }

    //Ugly but direct approach
    //Alternative is to store ID locally
    return tp->exec.tc_completed_id;
}

struct state_tag_t tpGetExecTag(TP_STRUCT const * const tp)
{
    if (0 == tp) {
        struct state_tag_t empty = {};
        return empty;
    }

    return tp->exec.execTag;
}

int tpGetExecSrcLine(const TP_STRUCT * const tp)
{
    if (tp) {
        return tp->exec.execTag.fields[GM_FIELD_LINE_NUMBER];
    }
    return 0;
}

int tpGetNextExecId(const TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    return tp->exec.nextexecId;
}

/**
 * Used to tell the tp the initial position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    int res_invalid = tpSetCurrentPos(tp, pos);
    if (res_invalid) {
        return TP_ERR_FAIL;
    }

    emcPoseToPmVector(pos, &tp->goalPos);
    return TP_ERR_OK;
}


/**
 * Set current position.
 * It sets the current position AND the goal position to be the same.  Used
 * only at TP initialization and when switching modes.
 */
int tpSetCurrentPos(TP_STRUCT * const tp, EmcPose const * const pos)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (emcPoseValid(pos)) {
        emcPoseToPmVector(pos, &tp->exec.currentPos);
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpSetCurrentPos on id %d!"
                "pos is %.12g, %.12g, %.12g\n",
                tp->exec.execId,
                pos->tran.x,
                pos->tran.y,
                pos->tran.z);
        return TP_ERR_INVALID;
    }
}

int tpAddCurrentPos(TP_STRUCT * const tp, PmVector const * const disp)
{
    if (!tp || !disp) {
        return TP_ERR_MISSING_INPUT;
    }

    if (!VecHasNAN(disp)) {
        VecVecAddEq(&tp->exec.currentPos, disp);
        return TP_ERR_OK;
    } else {
        rtapi_print_msg(RTAPI_MSG_ERR, "Tried to set invalid pose in tpAddCurrentPos on id %d!"
                "disp is %.12g, %.12g, %.12g\n",
                tp->exec.execId,
                disp->ax[0],
                disp->ax[1],
                disp->ax[2]);
        return TP_ERR_INVALID;
    }
}

/**
 * Check for valid tp before queueing additional moves.
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->exec.aborting) {
        rtapi_print_msg(RTAPI_MSG_ERR, "TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

const char *blendTypeAsString(tc_blend_type_t c)
{
    switch(c) {
    case NO_BLEND:
        return "NO_BLEND";
    case PARABOLIC_BLEND:
        return "PARABOLIC_BLEND";
    case TANGENT_SEGMENTS_BLEND:
        return "TANGENT_SEGMENTS";
    case ARC_BLEND:
        return "ARC_BLEND";
    }
    return "";
}

static void addRigidTapOverrun(TC_STRUCT * const tc, double revolutions, double uu_per_rev)
{
    PmCartLine *actual_xyz = &tc->coords.rigidtap.actual_xyz;
    pmCartLineStretch(actual_xyz, actual_xyz->tmag + revolutions * uu_per_rev, 0);
    tc->target = actual_xyz->tmag;
}

const char *cycleModeToString(UpdateCycleMode mode)
{
    switch (mode) {
    case UPDATE_NORMAL:
        return "normal";
    case UPDATE_PARABOLIC_BLEND:
        return "parabolic_blend";
    case UPDATE_SPLIT:
        return "split_cycle";
    }
    return "unknown";
}

double findTrapezoidalDesiredVel(double a_max,
                                 double dx,
                                 double v_final,
                                 double v_current,
                                 double cycle_time)
{
    double dt = fmax(cycle_time, TP_TIME_EPSILON);
    // Discriminant is 3 terms (when final velocity is non-zero)
    double discr_term1 = pmSq(v_final);
    double discr_term2 = a_max * (2.0 * dx - v_current * dt);
    double tmp_adt = a_max * dt * 0.5;
    double discr_term3 = pmSq(tmp_adt);
    double discr = discr_term1 + discr_term2 + discr_term3;

    //Start with -B/2 portion of quadratic formula
    double maxnewvel = -tmp_adt;

    // Negative discriminant here is possible in rare cases due to numerical errors
    maxnewvel += pmSqrt(discr);
#ifdef TP_DEBUG
    double trapezoidal_pos_root =  -tmp_adt + pmSqrt(discr);
    double trapezoidal_neg_root = -tmp_adt - pmSqrt(discr);
    tp_debug_json5_double(trapezoidal_pos_root);
    tp_debug_json5_double(trapezoidal_neg_root);
#endif

    return maxnewvel;
}

/**
 * If necessary, interpolate to find the true end time of a segment (if it's less than 1 timestep away).
 * @param time_step Nominal timestep for the servo loop (MUST BE > 0)
 * @param v_current
 * @param v_next
 * @param distance_to_go
 * @return
 */
EndCondition checkEndCondition(double dt_plan, //!< time delta to the new trajectory sample. Usually the nominal servo loop time, but may be shorter (e.g. after a split cycle).
                               double dtg, //!< Overall distance remaining in the segment
                               double currentvel, //!< Current trajectory segment velocity before any updates in this cycle
                               double v_final_plan, //!< Planned final velocity
                               double a_max)
{
#ifdef TP_DEBUG
    tp_debug_json5_double(dt_plan);
    tp_debug_json5_double(dtg);
    tp_debug_json5_double(currentvel);
    tp_debug_json5_double(v_final_plan);
    tp_debug_json5_double(a_max);
#endif

    if (dtg < TP_POS_EPSILON) {
        // Check that we're planning for a non-trivial amount of distance or time
        tp_debug_json5_verdict("end_condition_check", "close to end, declare it done");
        EndCondition tooshort = {
            currentvel,
            0,
            0,
            END_CONDITION_COMPLETE,
        };
        return tooshort;
    }
    if (dt_plan < TP_TIME_EPSILON) {
        // Check that we're planning for a non-trivial amount of distance or time
        tp_debug_json5_verdict("end_condition_check","time is too short, unchanged");
        EndCondition tooshort = {
            currentvel,
            0,
            0,
            END_CONDITION_NOOP,
        };
        return tooshort;
    }

    // Try to reach the final velocity at the end of this cycle.
    // If we can't reach it due to accel limits then get as close as possible
    // Given these constraints, are we moving fast enough to reach the end of the motion within 1 cycle?
    double v_final_reachable = CLAMP(v_final_plan, currentvel - a_max * dt_plan, currentvel + a_max * dt_plan);
    double v_avg = (currentvel + v_final_reachable) / 2.0;

    double v_avg_to_finish = dtg / dt_plan;
    if (v_avg_to_finish > v_avg) {
        // We're not moving fast enough, won't close the distance
        tp_debug_json5_verdict("end_condition_check","too far away");
        EndCondition too_far_away = {
            0,
            0,
            0,
            END_CONDITION_NORMAL,
            };
        return too_far_away;
    }

    // Conditions are just right so we can reach the end of the motion within one cycle
    double dv = v_final_reachable - currentvel;
    double dt_min = dv / a_max; // NOTE: this is signed based on acceleration direction
    double dt_guess = dtg / v_avg;
    if (dt_guess < fabs(dt_min)) {
        // We can finish the segment within one cycle, but not reach the exact final velocity
        // Accelerate at the maximum rate until we run out of distance, and declare victory with whatever velocity we reach

        double a = SIGN(dt_min) * a_max;
        double disc = pmSq(currentvel) + 2.0 * a * dtg;

        double completion_time = 2.0 * dtg / (currentvel + pmSqrt(disc));

        //Update final velocity with actual result
        double v_at_endpt = currentvel + completion_time * a;

        tp_debug_print("\"end_condition_check\": \"velocity not reached with a = %0.17f, displacement = %0.17f, dt_guess %0.17f, dt_min %0.17f\"", a, (v_at_endpt + currentvel) / 2.0 * completion_time, dt_guess, dt_min);
        EndCondition velocity_not_reached = {
            v_at_endpt,
            completion_time,
            a,
            END_CONDITION_COMPLETE};
        return velocity_not_reached;
    } else {
        // We can reach the exact final velocity within one cycle time
        tp_debug_json5_verdict("end_condition_check","exact end");
        EndCondition exact_end = {
            v_final_reachable,
            dt_guess,
            dv / dt_guess,
            END_CONDITION_COMPLETE,
        };
        return exact_end;
    }
}

/**
 * Compute updated position and velocity for a timestep based on a trapezoidal
 * motion profile.
 * @param acc actual acceleration to use for this update (respects machine limits)
 * @param vel_desired acceleration-boounded, time-optimal target velocity . Actual velocity may be lower (e.g. due to segment max velocity / requested feed rate).
 *
 * Creates the trapezoidal velocity profile based on the segment's velocity and
 * acceleration limits. The formula has been tweaked slightly to allow a
 * non-zero velocity at the instant the target is reached.
 */
void tpCalculateTrapezoidalAccel(
    TP_STRUCT const * const tp,
    TC_STRUCT * const tc,
    double * const acc,
    double * const vel_desired,
    double v_final)
{
    // Find maximum allowed velocity from feed and machine limits
    double tc_target_vel = tpGetRealAbsTargetVel(tp, tc);

    /* Calculations for desired velocity based on trapezoidal profile */
    double dx = tcGetDistanceToGo(tc, tp->exec.reverse_run);
    double maxaccel = tcGetTangentialMaxAccel(tc);

    double maxnewvel = findTrapezoidalDesiredVel(
        maxaccel, dx, v_final, tc->currentvel, tc->cycle_time);

    // Find bounded new velocity based on target velocity
    // Note that we use a separate variable later to check if we're on final decel
    double newvel = LIMIT(maxnewvel, tc_target_vel);

    // Calculate acceleration needed to reach newvel, bounded by machine maximum
    double dt = fmax(tc->cycle_time, TP_TIME_EPSILON);
    double maxnewaccel = (newvel - tc->currentvel) / dt;

    *acc = LIMIT(maxnewaccel, maxaccel);
    *vel_desired = maxnewvel;
}

static inline double interp1(double t0, double t1, double v0, double v1, double t)
{
    if (fabs(t1-t0) < TP_POS_EPSILON) {
        // KLUDGE not ideal, but if the time interval is so short then it shouln't matter which side we pick
        return v1;
    }
    return (v1-v0)/(t1-t0) * (t-t0) + v0;
}

/**
 * Calculate "ramp" acceleration for a cycle.
 */
int tpCalculateRampAccel(
    TP_STRUCT const * const tp,
    TC_STRUCT * const tc,
    double * const acc,
    double * const vel_desired,
    double v_final)
{
    // displacement remaining in this segment
    double dx = fmax(tcGetDistanceToGo(tc, tp->exec.reverse_run), TP_POS_EPSILON);

    const double v_current = tc->currentvel;

    /* Check if the final velocity is too low to properly ramp up.*/
    if (v_final < TP_VEL_EPSILON) {
#ifdef TC_DEBUG
        print_json5_double_("ramp_final_vel", v_final);
#endif
        return TP_ERR_FAIL;
    }

    // Estimate constant acceleration required
    double acc_final = (pmSq(v_final)-pmSq(v_current))/(2.0*fmax(dx, TP_POS_EPSILON));

    // Saturate estimated acceleration against maximum allowed by segment
    double acc_max = tcGetTangentialMaxAccel(tc);

    // Output acceleration and velocity for position update
    *acc = LIMIT(acc_final, acc_max);
    *vel_desired = v_final;

    return TP_ERR_OK;
}

void tpToggleDIOs(TC_STRUCT * const tc) {

    int i=0;
    if (tc->syncdio.anychanged != 0) { // we have DIO's to turn on or off
        for (i=0; i < num_dio; i++) {
            if (!(tc->syncdio.dio_mask & (1 << i))) continue;
            if (tc->syncdio.dios[i] > 0) emcmotDigitalOutWrite(i, 1); // turn DIO[i] on
            if (tc->syncdio.dios[i] < 0) emcmotDigitalOutWrite(i, 0); // turn DIO[i] off
        }
        for (i=0; i < num_aio; i++) {
            if (!(tc->syncdio.aio_mask & (1 << i))) continue;
            emcmotAioWrite(i, tc->syncdio.aios[i]); // set AIO[i]
        }
        tc->syncdio.anychanged = 0; //we have turned them all on/off, nothing else to do for this TC the next time
    }
}
/**
 * The displacement is always computed with respect to a specie
 */
double findSpindleDisplacement(
        double new_pos,
        spindle_origin_t origin
        )
{
    return origin.direction * (new_pos - origin.position);
}

double findSpindleVelocity(
        double spindle_velocity,
        spindle_origin_t origin
        )
{
    return origin.direction * spindle_velocity;
}

/**
 * Helper function to compare commanded and actual spindle velocity.
 * If the signs of velocity don't match, then the spindle is reversing direction.
 * NOTE: not re-entrant due to spindle reversal timeout
 */
bool spindleReversed(spindle_origin_t origin, double prev_pos, double current_pos)
{
    bool timeout_done = --emcmotStatus->spindle_cmd.reversal_timeout < 0;
    return timeout_done && origin.direction * (current_pos - prev_pos) <= 0;
}

/**
 * Temporarily override the spindle command during rigid tapping
 */
void cmdSpindleScale(double scale)
{
    emcmotStatus->spindle_cmd.reversal_timeout = 0;
    emcmotStatus->spindle_cmd.velocity_rpm_out = emcmotStatus->spindle_cmd.velocity_rpm_nominal * scale;
}

void cmdSpindlePauseTimeout(int timeout_counts)
{
    emcmotStatus->spindle_cmd.reversal_timeout = timeout_counts;
    emcmotStatus->spindle_cmd.velocity_rpm_out = 0.0;
}

/**
 * Safely reverses rigid tap motion towards the starting point, preserving the existing tracking error.
 * @note extra motion distance for overrun must be added separately
 */
static inline void reverseRigidTapMotion(TC_STRUCT * const tc,
                                  spindle_origin_t * const spindle_origin)
{
    // we've stopped, so set a new target at the original position
    PmCartesian start, end;
    PmCartLine *actual_xyz = &tc->coords.rigidtap.actual_xyz;

    // Set a new spindle origin at the approximate reversal point, and keep the current tracking error as the new offset
    updateSpindlePositionFromProgress(spindle_origin, tc);

    pmCartLinePoint(&tc->coords.rigidtap.actual_xyz, tc->progress, &start);
    end = tc->coords.rigidtap.nominal_xyz.start;
    pmCartLineInit(actual_xyz, &start, &end);
    tc->coords.rigidtap.reversal_target = tc->target = actual_xyz->tmag;
    // NOTE: reset both progress and sync location:
    // At the point of reversal, the spindle is already synchronized, so
    // store the current position tracking error (in user units) as the sync offset
    // This way any accumulated error is not forgotten during the retraction
    tc->progress = 0.0;
}

double estimate_rigidtap_decel_distance(double vel, double uu_per_rev)
{
    double cmd_latency_dist = fmax(vel * emcmotStatus->spindle_cmd_latency_sec, 0.);
    double decel_distance = emcmotStatus->spindle_max_acceleration_rps2 > 0.0 ? pmSq(vel) / (2.0 * fabs(uu_per_rev) * emcmotStatus->spindle_max_acceleration_rps2) : 0.0;
    return cmd_latency_dist + decel_distance;
}

/**
 * Handle special cases for rigid tapping.
 * This function deals with updating the goal position and spindle position
 * during a rigid tap cycle. In particular, the target and spindle goal need to
 * be carefully handled since we're reversing direction.
 */
void tpUpdateRigidTapState(
    TP_STRUCT * const tp,
    TC_STRUCT * const tc,
    TC_STRUCT * const nexttc)
{
    static double old_spindle_pos = 0.0;
    double spindle_pos = emcmotStatus->spindle_fb.position_rev;
    static double retract_allowance = 0.0;

    rigid_tap_state_t const initial_state = tc->coords.rigidtap.state;
    switch (initial_state) {
        case RIGIDTAP_INACTIVE:
            return;
        case RIGIDTAP_TAPPING:
        {
            // HACK just hard-code this to a fixed number of counts
            tc->uu_per_rev = tc->coords.rigidtap.tap_uu_per_rev;
            double decel_distance = estimate_rigidtap_decel_distance(tc->currentvel, tc->uu_per_rev);
            double reversal_target = tc->coords.rigidtap.reversal_target - decel_distance;
            if (tc->progress >= reversal_target) {
                if (tc->coords.rigidtap.dwell_counts) {
                    // Stop at the bottom and wait (implies a bit of extra dwell if there is spindle command latency)
                    cmdSpindlePauseTimeout(tc->coords.rigidtap.dwell_counts);
                } else {
                    // Preserve legacy behavior by default (no delay in spindle commands)
                    cmdSpindleScale(-1.0*tc->coords.rigidtap.reversal_scale);
                }
                // command reversal to stop / reverse at the bottom of the hole
                emcmotStatus->rigid_tap_reversal_vel_rps = emcmotStatus->spindle_fb.velocity_rps;
                tc->coords.rigidtap.state = RIGIDTAP_REVERSING;
                retract_allowance = estimate_rigidtap_decel_distance(tc->currentvel, tc->coords.rigidtap.retract_uu_per_rev);
            }
            break;
        }
        case RIGIDTAP_REVERSING:
            if (spindleReversed(tp->exec.spindle_cmd.origin, old_spindle_pos, spindle_pos) && tc->currentvel <= 0.0) {
                cmdSpindleScale(-1.0*tc->coords.rigidtap.reversal_scale); // No-op if dwell is zero
                emcmotStatus->rigid_tap_overshoot = tc->progress - tc->coords.rigidtap.reversal_target;
                reverseRigidTapMotion(tc, &tp->exec.spindle_cmd.origin);
                double expected_reversal_overshoot = fmax(retract_allowance / tc->coords.rigidtap.retract_uu_per_rev, 0);
                // Anticipate the overshoot at the top and add a safety factor for spindle uncertainty
                addRigidTapOverrun(tc, RIGIDTAP_MAX_OVERSHOOT_REVS + expected_reversal_overshoot, tc->coords.rigidtap.retract_uu_per_rev);
                tc->coords.rigidtap.state = RIGIDTAP_RETRACTION;
                tc->uu_per_rev = tc->coords.rigidtap.retract_uu_per_rev;
            }
            break;
        case RIGIDTAP_RETRACTION:
            if (tc->progress >= tc->coords.rigidtap.reversal_target) {
                // Flip spindle direction again to start final reversal
                cmdSpindleScale(1.0);
                tc->coords.rigidtap.state = RIGIDTAP_FINAL_REVERSAL;
                // Once we've cleared the hole, there's no reason to keep synched with the spindle unless we're peck tapping
                if (emcmotConfig->joint_filter_cfg.acceleration_scale == 1 && (
                        !nexttc ||
                        nexttc->motion_type != TC_RIGIDTAP ||
                        tc->blend_mode.mode != TC_TERM_COND_TANGENT)) {
                    // Stop synchronized motion if not peck tapping
                    tc->synchronized = 0;
                    tc->target_vel = 0; // Stop as fast as possible during final reversal
                    tc->canon_motion_type = EMC_MOTION_TYPE_TRAVERSE;
                    // Scale the acceleration down to match non-synched motion settings
                    // This isn't quite perfect because we don't have jerk limiting active until the end of the motion but
                }
            }
            break;
        case RIGIDTAP_FINAL_REVERSAL:
            if (spindleReversed(tp->exec.spindle_cmd.origin, old_spindle_pos, spindle_pos) && tc->currentvel <= 0.0) {
                reverseRigidTapMotion(tc, &tp->exec.spindle_cmd.origin);
                tc->uu_per_rev = tc->coords.rigidtap.tap_uu_per_rev;
                tc->coords.rigidtap.state = RIGIDTAP_FINAL_PLACEMENT;
                if (!nexttc
                        || nexttc->motion_type != TC_RIGIDTAP
                        || tc->blend_mode.mode != TC_TERM_COND_TANGENT) {
                    // Stop synchronized motion if not peck tapping
                    tc->synchronized = 0;
                    tc->target_vel = tc->maxvel_geom; // Move as fast as possible to the final position
                    tc->canon_motion_type = EMC_MOTION_TYPE_TRAVERSE;

                    // If we need to run at lower acceleration after leaving
                    // synch, then we stay synched until the tap stops after
                    // retraction, then switch acceleration values once the
                    // machine has come to rest.
                    tc->maxaccel *= emcmotConfig->joint_filter_cfg.acceleration_scale;
                }
            }
            break;
        case RIGIDTAP_FINAL_PLACEMENT:
            // this is a regular move now, it'll stop at target above.
            old_spindle_pos = 0.0;
            retract_allowance = 0.0;
            break;
    }
    old_spindle_pos = spindle_pos;
#ifdef TC_DEBUG
    rigid_tap_state_t current_state = tc->coords.rigidtap.state;
    print_json5_log_start(tpUpdateRigidTapState);
    print_json5_unsigned(current_state);
    print_json5_log_end();
#endif
}

/**
 * Update emcMotStatus with information about trajectory motion.
 * Based on the specified trajectory segment tc, read its progress and status
 * flags. Then, update the emcmotStatus structure with this information.
 */
int tpUpdateMovementStatus(TP_STRUCT * const tp,
        TC_STRUCT const * const tc,
        TC_STRUCT const * const nexttc)
{
    if (!tp) {
        return TP_ERR_FAIL;
    }

    emcmotStatus->cutter_comp_phase = 0;
    tp->exec.motionType = tc->canon_motion_type;
    tp->exec.activeDepth = tc->active_depth;
    tp->exec.execId = tc->id;

#ifdef MOTION_HAL_SEGMENT_ID_DEBUGGING
    // Store the next ID as well to show what will execute next to the GUI
    
    int nextexecId = tp->exec.nextexecId = tc->tag.fields[GM_FIELD_NEXT_LINE];
    if (0 == nextexecId && nexttc) {
        tp->exec.nextexecId = nexttc->tag.fields[GM_FIELD_LINE_NUMBER];
    } else {
        tp->exec.nextexecId = nextexecId;
    }
#endif

    emcmotStatus->distance_to_go = tc->target - tc->progress;
    emcmotStatus->cutter_comp_phase = tc->tag.fields[GM_FIELD_COMP_PHASE];
    emcmotStatus->enables_queued = tc->enables;
    emcmotStatus->requested_vel = tpGetRealAbsTargetVel(tp, tc);
    // emcmotStatus->current_vel updated at end of cycle
    emcmotStatus->dwell_time_remaining = (tc->motion_type == TC_DWELL) ? tc->coords.dwell.remaining_time : 0.0;
    {
        PmVector tc_pos = tcGetPosReal(tc, tcGetTarget(tc, tp->exec.reverse_run));
        PmVector dtg = VecVecSub(tc_pos, &tp->exec.currentPos);
        pmVectorToEmcPose(&dtg, &emcmotStatus->dtg);
    }

    
    if (tc->synchronized != TC_SYNC_POSITION) {
        clearPosTrackingStatus();
    }
    emcmotStatus->rigid_tap_state = tc->motion_type == TC_RIGIDTAP ? tc->coords.rigidtap.state : RIGIDTAP_INACTIVE;
    return TP_ERR_OK;
}

/**
 * Keeps track of time required to drain motion smoothing filters after TP has reached zero velocity.
 * Commanded motion is not actually stopped until the TP and any time-delayed smoothing is done.
 */
static bool checkJointFiltersEmpty(TP_STRUCT * const tp)
{
    return tp->exec.filters_at_rest;
}

/**
 * @return true if the TP is moving (i.e. non-zero velocity, acceleration, etc.)
 * @note: Need to check acceleration in future limited-jerk planning
 */
static bool checkTPCommandingMovement(TP_STRUCT * const tp)
{
    return emcmotStatus->current_vel > 0;
}

/**
 * Track the active "depth" of joint filters so that the control loop knows if
 * coordinated joint motion is actually complete.
 *
 * If the TP is currently commanding zero velocity and acceleration, then the
 * joint filters will start to drain. For example, a moving average window of
 * 5ms means that all joints will stop moving no more than 5ms after the TP
 * reaches zero velocity.
 */
void updateJointFilterActiveDepth(TP_STRUCT *tp)
{
    if (checkTPCommandingMovement(tp)) {
        tp->exec.joint_filter_drain_counter = MAX(emcmotStatus->joint_filter_window_size_counts, 1);
    } else if (!checkJointFiltersEmpty(tp)){
        --tp->exec.joint_filter_drain_counter;
    }
}


/**
 * Cleanup if tc is not valid (empty queue).
 * If the program ends, or we hit QUEUE STARVATION, do a soft reset on the trajectory planner.
 * TODO merge with tpClear?
 */
int tpHandleStopConditions(TP_STRUCT * const tp, TC_STRUCT * const tc, TC_STRUCT * const nexttc)
{
    bool queue_empty = (!tc && !nexttc);
    // Note: empty queue implies movement stopped
    bool movement_stopped = (!tc || tc->currentvel == 0.0) && (!nexttc || nexttc->currentvel == 0.0);

    // 1) Abort means a controlled slow to a stop as fast as possible, then flushing of the queue / motion status
    
    // moving, and if it is somehow moving then we're not ready to finish
    // aborting?)

    if (tp->exec.aborting && (movement_stopped || tpIsWaiting(tp))) {
        // Clean queue right away once we reach the planned stop
        tcqReset(&tp->queue);
        tpCleanupAtEmptyQueue(tp);
        if (checkJointFiltersEmpty(tp)) {
            // Now finish the cleanup since smoothed motion is also done
            tpCleanupAfterAbort(tp); // Safe to call at abort-time since we're about to drop out of coord mode
            // Every abort (expected or otherwise) now ends with dropping out of
            // coord mode because of the order of operations:
            // 1) get_pos_cmds calls tpRunCycle etc. which leads to here (and the
            //    request to leave coord mode)
            // 2) this control loop update finishes
            // 3) next control loop update handles inputs
            // 4) set_operating_mode() is called and see the coordinating flag,
            //    drops out of coord mode BEFORE the next position update. Even if
            //    userspace manages to sneak a command in, it will be ignored due to this.
        }
        return TP_ERR_STOPPED;
    } else if (queue_empty) {
        tpCleanupAtEmptyQueue(tp);
        return TP_ERR_STOPPED;
    } else {
        // Normal operation, filters are still being fed by TP
    }

    // Safe to continue with the rest of planning (no abort / reset required)
    return TP_ERR_OK;
}

/** Wrapper function to unlock rotary axes */
void tpSetRotaryUnlock(IndexRotaryAxis axis, int unlock) {
    emcmotSetRotaryUnlock(axis, unlock);
}

/** Wrapper function to check rotary axis lock */
int tpGetRotaryIsUnlocked(IndexRotaryAxis axis) {
    return emcmotGetRotaryIsUnlocked(axis);
}

/**
 * Cleanup after a trajectory segment is complete.
 * If the current move is complete and we're not waiting on the spindle for
 * const this move, then pop if off the queue and perform cleanup operations.
 * Finally, get the next move in the queue.
 */
int tpCompleteSegment(TP_STRUCT * const tp,
        TC_STRUCT * const tc) {

    if (tpIsWaitingOnSegment(tp, tc)) {
        return TP_ERR_FAIL;
    }

    // if we're synced, and this move is ending, save the
    // spindle position so the next synced move can be in
    // the right place.
    if(tc->synchronized != TC_SYNC_NONE) {
        updateSpindlePositionFromProgress(&tp->exec.spindle_cmd.origin, tc);
    } else {
        setSpindleOrigin(&tp->exec.spindle_cmd.origin, 0.0);
    }

    if(tc->indexrotary != INDEX_NONE) {
        // this was an indexing move, so before we remove it we must
        // relock the axis
        tpSetRotaryUnlock(tc->indexrotary, 0);
        // if it is now locked, fall through and remove the finished move.
        // otherwise, just come back later and check again
        if(tpGetRotaryIsUnlocked(tc->indexrotary)) {
            return TP_ERR_FAIL;
        }
    }

    //Clear status flags associated since segment is done
    
    tc->activation_state = SEGMENT_NEW;
    tc->complete = false;
    tc->cycle_time = tp->cycleTime;
    //Velocities are by definition zero for a non-active segment
    tc->currentvel = 0.0;
    tp->exec.tc_completed_id = tc->id;
    
    // done with this move
    if (tp->exec.reverse_run) {
        tcqBackStep(&tp->queue);
    } else {
        int res_pop = tcqPop(&tp->queue);
        if (res_pop) rtapi_print_msg(RTAPI_MSG_ERR,"Got error %d from tcqPop!\n", res_pop);
    }

    return TP_ERR_OK;
}

void setSpindleTrigger(TP_STRUCT * const tp, TC_STRUCT * const tc)
{
    const double spindle_vel_rps_raw = get_spindle_speed_out_rpm(emcmotStatus) / 60.0;

    // WARNING: this assumes that max acceleration is constant over the segment to estimate the position error
    // This won't be true in the future if we switch to s-curve planning
    double a_max = tcGetTangentialMaxAccel(tc);
    double accel_revs_est = fabs(tc->uu_per_rev * pmSq(spindle_vel_rps_raw) / (2.0 * a_max));
    // Advance the spindle origin by at least enough
    double spindle_offset_turns = ceil(accel_revs_est + 0.25);
    setSpindleOrigin(&tp->exec.spindle_cmd.origin, spindle_offset_turns * get_spindle_command_direction(emcmotStatus));

    // Setup spindle trigger conditions
    tp->exec.spindle_cmd.trigger_revs = -accel_revs_est;
}


bool waiting_for_probe_power()
{
    bool probe_waiting = emcmotStatus->probe_enable_request && !emcmotStatus->probe_ready;
    bool setter_waiting = emcmotStatus->setter_enable_request && !emcmotStatus->setter_ready;
    return probe_waiting || setter_waiting;
}

bool joint_filters_match_config_size()
{
    return emcmotStatus->joint_filter_window_size_counts == (int)(emcmotConfig->joint_filter_cfg.window_size_sec / emcmotConfig->servoCycleTime);
}

/**
 * Check if the spindle has reached the required speed for a move.
 * Returns a "wait" code if the spindle needs to spin up before a move and it
 * has not reached the requested speed, or the spindle index has not been
 * detected.
 */
tp_err_t tpCheckWaitConditions(TP_STRUCT * const tp, TC_STRUCT * const tc)
{
    const double waited_time = (double)(tp->exec.time_elapsed_ticks - tp->exec.time_at_wait)*tp->cycleTime;
    // this is no longer the segment we were waiting_for_index for
    for (WaitFlagIndex k = WAIT_FOR_SPINDLE_INDEX; k < MAX_WAIT_INDICES; ++k) {
        int waiting_id = tp->exec.waiting[k];
        if (MOTION_ID_VALID(waiting_id) && waiting_id != tc->id)
        {
            rtapi_print_msg(
                RTAPI_MSG_ERR,
                "Was waiting for %s on motion id %d, but reached id %d\n",
                wait_type_as_str(k),
                waiting_id, tc->id);
            tp->exec.waiting[k] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_SPINDLE_ATSPEED])) {
        if(!emcmotStatus->spindle_is_atspeed) {
            // spindle is still not at the right speed, so wait another cycle
            if ( waited_time > emcmotConfig->timeout_cfg.atspeed_wait_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sSpindle did not reach desired speed %0.2f RPM within %0.2f seconds (spindle-at-speed-timeout)",
                    linedesc.buf,
                    emcmotStatus->spindle_cmd.velocity_rpm_out,
                    emcmotConfig->timeout_cfg.atspeed_wait_timeout_sec);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            tp->exec.waiting[WAIT_FOR_SPINDLE_ATSPEED] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_SPINDLE_INDEX])) {
        if (emcmotStatus->spindle_fb.index_enable) {
            if (waited_time > emcmotConfig->timeout_cfg.index_wait_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sSpindle index signal was not detected within %.2f seconds (spindle-index-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                /* haven't passed index yet */
                return TP_ERR_WAITING;
            }
        } else {
            /* passed index, start the move */
            emcmotStatus->spindle_fb.synced = 1;
            tp->exec.waiting[WAIT_FOR_SPINDLE_INDEX] = MOTION_INVALID_ID;
            setSpindleTrigger(tp, tc);
            emcmotStatus->spindle_sync_state = SYNC_TRIGGER_WAIT;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_PROBE_READY])) {
        // Indicates that hardware is turned on and probe input will be valid
        if (waiting_for_probe_power()) {
            
            if (waited_time > emcmotConfig->timeout_cfg.probe_wait_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sProbe not ready within %.2f seconds (probe-ready-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            // Acknowledge the wait is over since probe is active
            tp->exec.waiting[WAIT_FOR_PROBE_READY] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_PROBING])) {
        // Indicates that hardware is turned on and probe input will be valid
        if (!emcmotStatus->probing) {
            
            if (waited_time > emcmotConfig->timeout_cfg.probe_wait_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sProbing move did not start within %.2f seconds (probing-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            // Acknowledge the wait is over since probe is active
            tp->exec.waiting[WAIT_FOR_PROBING] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_OPTIMIZATION])) {
        // Indicates that hardware is turned on and probe input will be valid
        if (tcGetOptimizationState(tc) == TC_PLAN_UNTOUCHED) {
            
            if (waited_time > emcmotConfig->timeout_cfg.optimization_wait_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sLookahead optimization not complete within %.2f seconds (lookahead-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            // Acknowledge the wait is over since probe is active
            tp->exec.waiting[WAIT_FOR_OPTIMIZATION] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_INDEXER_UNLOCK])) {
        // Wait for indexer to unlock on the required axis
        if (!tpGetRotaryIsUnlocked(tc->indexrotary)) {
            if (waited_time > emcmotConfig->timeout_cfg.indexer_unlock_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sIndexer axis failed to unlock within %.2f seconds (indexer-unlock-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            // Should not reach here because out-of-range check fails locally
            tp->exec.waiting[WAIT_FOR_INDEXER_UNLOCK] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_FILTER_DISABLE])) {
        // Wait for control to actually disable joint filters (only happens
        // once all joints have been at rest long enough for them to "drain").
        if (emcmotStatus->joint_filter_window_size_counts != MIN_FILTER_SIZE) {
            if (waited_time > emcmotConfig->timeout_cfg.filter_change_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sMotion filters not disabled for synched motion within %.2f seconds (filter-disable-timeout)",
                    linedesc.buf,
                    waited_time);
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            tp->exec.waiting[WAIT_FOR_FILTER_DISABLE] = MOTION_INVALID_ID;
        }
    }

    if (MOTION_ID_VALID(tp->exec.waiting[WAIT_FOR_FILTER_ENABLE])) {
        // Wait for indexer to unlock on the required axis

        if (!joint_filters_match_config_size()) {
            if (waited_time > emcmotConfig->timeout_cfg.filter_change_timeout_sec) {
                LineDescriptor linedesc = formatLinePrefix(&tc->tag);
                tpStopWithError(tp, "%sMotion filter not renabled within %.2f seconds (filter-enable-timeout), requested size is %d but current size is %d",
                                linedesc.buf,
                                waited_time,
                                emcmotStatus->joint_filter_window_size_counts,
                                (int)(emcmotConfig->joint_filter_cfg.window_size_sec / emcmotConfig->servoCycleTime)
                                       );
            } else {
                return TP_ERR_WAITING;
            }
        } else {
            tp->exec.waiting[WAIT_FOR_FILTER_ENABLE] = MOTION_INVALID_ID;
        }
    }

    return TP_ERR_OK;
}

void checkPositionMatch(TP_STRUCT *tp, TC_STRUCT const *tc)
{
    unsigned has_position_mismatch = 0;
    PmVector tp_position_error={};
    if (needConsistencyCheck(CCHECK_C0_CONTINUITY)){
        tp_position_error = tcGetPos(tc);
        VecVecSubEq(&tp_position_error, &tp->exec.currentPos);

        has_position_mismatch = findAbsThresholdViolations(tp_position_error, emcmotConfig->consistencyCheckConfig.maxPositionDriftError);
    }

    // Log a bunch of TP internal state if required by debug level or position error
    if (has_position_mismatch || _tp_debug) {
        print_json5_log_start(tpActivateSegment);
        if (has_position_mismatch && !tp->exec.aborting) {
            AxisMaskString failed_axes_str = axisBitMaskToString(has_position_mismatch);
            print_json5_string_("mismatched_axes", failed_axes_str.axes);
            tpStopWithError(tp, "Motion aborted due to command axis position mismatch, details have been logged\n");
        }
        print_json5_long_long_("time_ticks", tp->exec.time_elapsed_ticks);
        print_json5_tc_id_data_(tc);
        print_json5_string_("motion_type_name", tcMotionTypeAsString(tc->motion_type));
        print_json5_int_("motion_type", tc->motion_type);
        print_json5_int_field(tc, canon_motion_type);
        print_json5_string_("canon_type_name", tcCanonMotionTypeAsString(tc->canon_motion_type));
        print_json5_int_("active_axes", tc->tag.fields[GM_FIELD_FEED_AXES]);

        // Position settings

        print_json5_PmVector(tp_position_error);
        print_json5_int_("pos_err_threshold", emcmotConfig->consistencyCheckConfig.maxPositionDriftError);
        print_json5_double_field(tc, target);
        print_json5_double_field(tc, progress);
        // Velocity settings
        print_json5_double_field(tc, reqvel);
        print_json5_double_field(tc, target_vel);
        print_json5_double_("finalvel", tcGetPlanFinalVel(tc));
        print_json5_double_("finalvel_plan", tcGetFinalVelInternal(tc));
        print_json5_double_("finalvel_raw", tc->shared.final_vel_limit);
        print_json5_double_("optimization_state", tc->shared.optimization_state);
        print_json5_double_field(tc, kink_vel);
        print_json5_int_field(tc, use_kink);
        int blend_mode = 0;
        if (tc->blend_mode.mode == TC_TERM_COND_PARABOLIC) {
            blend_mode = 2;
        } else if (tc->blend_mode.mode == TC_TERM_COND_TANGENT) {
            blend_mode = tc->use_kink ? 3 : 4;
        }
        print_json5_int(blend_mode);
        // Acceleration settings
        print_json5_double_("accel_scale", tcGetAccelScale(tc));
        print_json5_double_("acc_overall", tcGetOverallMaxAccel(tc));
        print_json5_double_("acc_tangential", tcGetTangentialMaxAccel(tc));
        print_json5_bool_("accel_ramp", tc->accel_mode);
        print_json5_string_("sync_mode", tcSyncModeAsString(tc->synchronized));

        print_json5_int_("term_cond", tc->blend_mode.mode);
        print_json5_int_field(tc, id);
        print_json5_int_field(tc, needs_spindle_atspeed);
        print_json5_double_field(tc, cycle_time);

        print_json5_log_end();
    }
}

static inline bool enteringPositionSync(TC_STRUCT *tc)
{
    return (tc->synchronized == TC_SYNC_POSITION && !(emcmotStatus->spindle_fb.synced));
}

/**
 * Checks if the next segment to be activated will leave spindle position sync.
 * If so, we need to wait if there's a non-default jerk filter active
 */
static inline bool exitingPositionSync(TC_STRUCT *tc)
{
    return (tc->synchronized != TC_SYNC_POSITION && (emcmotStatus->spindle_fb.synced));
}

static void tcCheckAccelMode(TP_STRUCT *tp, TC_STRUCT *tc)
{
    // Acceleration ramping is not possible in the following conditions:
    // 1) Nearly-zero initial / final velocity (ramp would take too long)
    // 2) Rapid moves (would cost cycle time)
    // 3) Moves with exact stop conditions or wait conditions (forcing initial velocity to be zero)
    if (
        tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE ||
        tc->blend_mode.mode != TC_TERM_COND_TANGENT ||
        tc->synchronized)
    {
        tc->accel_mode = TC_ACCEL_TRAPZ;
        return;
    }

    double cutoff_time = 1.0 / (fmax(emcmotConfig->arc_blend_cfg.ramp_frequency, TP_TIME_EPSILON));
    double length = tcGetDistanceToGo(tc, tp->exec.reverse_run);
    // Given what velocities we can actually reach, estimate the total time for the segment under ramp conditions
    double avg_ramp_vel = (tc->currentvel + fmin(tcGetPlanFinalVel(tc), tpGetRealAbsTargetVel(tp,tc))) / 2.0;
    double cutoff_vel = length / fmax(fmax(cutoff_time, tp->cycleTime), TP_TIME_EPSILON);

    if (avg_ramp_vel > cutoff_vel)
    {
        tc->accel_mode = TC_ACCEL_RAMP;
    }
    // NOTE: optimization is allowed to directly request ramp acceleration
}


/**
 * "Activate" a segment being read for the first time.
 * This function handles initial setup of a new segment read off of the queue
 * for the first time.
 *
 * @note: Wait conditions are handled here by returning the "WAITING" enum
 * without activating the segment. This prevents further execution until
 * tpCheckWaitConditions sees that the wait condition is met. tpRunCycle calls
 * the wait check before getting here, so a new segment can only be activated
 * (and executed) once all wait conditions are satisfied.
 */
tp_err_t tpActivateSegment(TP_STRUCT * const tp, TC_STRUCT * const tc) {
    if (!tc) {
        return TP_ERR_OK;
    }

#ifdef TP_PEDANTIC
    if (!tp) {
        return TP_ERR_MISSING_INPUT;
    }
#endif

    switch (tc->activation_state) {
    case SEGMENT_NEW:
    {
        // Do not change initial velocity here, since tangent blending already sets this up
        tp->exec.motionType = tc->canon_motion_type;
        tc->on_final_decel = 0;
        tc->cycle_time = tp->cycleTime;

        // Based on the INI setting for "cutoff frequency", this calculation finds
        // short segments that can have their acceleration be simple ramps, instead
        // of a trapezoidal motion. This leads to fewer jerk spikes, at a slight
        // performance cost.
        tcCheckAccelMode(tp, tc);

        if (tc->motion_type == TC_DWELL) {
            // Dwell the requested amount of time, plus any extra time required by the spindle rpm delta
            double dwell_time = tc->coords.dwell.dwell_time_req;
            if (tc->coords.dwell.delta_rpm_req > 0
                && emcmotStatus->spindle_max_acceleration_rps2 > 0) {
                // Get the current spindle acceleration (changes based on belt position, for example) and compute a dwell time to wait for the spindle to reach the speed delta
                // This avoids the at-speed handshake which may not be consistent enough for soft-tapping
                double delta_rps = tc->coords.dwell.delta_rpm_req / 60.0;
                dwell_time += delta_rps / emcmotStatus->spindle_max_acceleration_rps2 + emcmotStatus->spindle_cmd_latency_sec;
            }
            tc->coords.dwell.dwell_time = tc->coords.dwell.remaining_time = dwell_time;
        }

        // Update the modal state displayed by the TP
        tp->exec.execTag = tc->tag;
        clearPositionSyncErrors();
        checkPositionMatch(tp, tc);

        if (tp->exec.reverse_run &&
                (tc->motion_type == TC_RIGIDTAP ||
                 tc->synchronized != TC_SYNC_NONE ||
                 tc->needs_spindle_atspeed ||
                 tc->probe.active ||
                 tc->needs_probe_ready ||
                 tc->indexrotary != INDEX_NONE)) {
            // Can't handle segments with wait conditions in reverse run
            tc->activation_state = SEGMENT_BLOCKED;
            return TP_ERR_REVERSE_EMPTY;
        }
        tc->activation_state = SEGMENT_CHECK_PRIMARY_WAIT;
    } // Fallthrough
    case SEGMENT_CHECK_PRIMARY_WAIT:
    {
        if (tcGetOptimizationState(tc) == TC_PLAN_UNTOUCHED) {
            // IMPORTANT: this is the safeguard that prevents unfinished segments
            // from being executed (e.g. segments that might be blended once
            // additional ones are enqueued). This works because MDI and auto mode
            // now send "TP_FLUSH" commands whenever they're done queueing motions,
            // so the TP knows when it can finalize what's in the queue.
            tp_debug_print("Can't start motion id %d unique id %llu because it's not finalized\n", tc->id, tc->unique_id);
            tp->exec.waiting[WAIT_FOR_OPTIMIZATION] = tc->id;
        }

        if (tc->needs_spindle_atspeed || enteringPositionSync(tc)) {
            tp->exec.waiting[WAIT_FOR_SPINDLE_ATSPEED] = tc->id;
        }

        if (emcmotConfig->joint_filter_cfg.window_size_sec > 0) {
            if (enteringPositionSync(tc)) {
                // Override the active filter for position synch and wait for it to be fully disabled
                // NOTE: consider adding a custom config just for tapping (in case we want to detune acceleration)
                emcmotStatus->joint_filter_cfg = default_joint_filter_cfg;
                tp->exec.waiting[WAIT_FOR_FILTER_DISABLE] = tc->id;
            } else if (exitingPositionSync(tc)) {
                // Restore the original filter config and wait for it to be enabled
                emcmotStatus->joint_filter_cfg = emcmotConfig->joint_filter_cfg;
                tp->exec.waiting[WAIT_FOR_FILTER_ENABLE] = tc->id;
            }
        }

        if(tc->needs_probe_ready) {
            tp->exec.waiting[WAIT_FOR_PROBE_READY] = tc->id;
        }

        tc->activation_state = SEGMENT_CHECK_SECONDARY_WAIT;

        // If we need any primary wait conditions, then we have to drop out here.
        // Otherwise, continue to secondary waits check
        if (tpIsWaiting(tp)) {
            return TP_ERR_WAITING;
        }
    } // Fallthrough
    case SEGMENT_CHECK_SECONDARY_WAIT:
    {
        if(tc->probe.active) {
            
            // Start a wait for the probe to become ready
            emcmotStatus->probe_mode = tc->probe;
            tp->exec.waiting[WAIT_FOR_PROBING] = tc->id;
        }

        if (enteringPositionSync(tc)) {
            tp_debug_print("Setting up position sync\n");
            // if we aren't already synced, wait
            // ask for an index reset
            emcmotStatus->spindle_fb.index_enable = 1;
            setSpindleOrigin(&tp->exec.spindle_cmd.origin, 0.0);
            emcmotStatus->spindle_sync_state = SYNC_SEEK_INDEX;
            tp->exec.waiting[WAIT_FOR_SPINDLE_INDEX] = tc->id;
        }

        if (tc->indexrotary != INDEX_NONE) {
            // request that the axis unlock
            tpSetRotaryUnlock(tc->indexrotary, 1);
            tp->exec.waiting[WAIT_FOR_INDEXER_UNLOCK] = tc->id;
        }

        tc->activation_state = SEGMENT_ACTIVE;

        // If we need any secondary wait conditions, then we have to drop out here.
        if (tpIsWaiting(tp)) {
            return TP_ERR_WAITING;
        }
    } // Fallthrough
    case SEGMENT_ACTIVE:
        return TP_ERR_OK;
    case SEGMENT_BLOCKED:
        // Special case for reverse run (any moves requiring wait conditions,
        // or where moving in reverse makes no sense, like probing
        return TP_ERR_REVERSE_EMPTY;
    }

    return TP_ERR_OK;
}

/**
 * Run velocity mode synchronization.
 * Update requested velocity to follow the spindle's velocity (scaled by feed rate).
 */
void tpSyncVelocityMode(TC_STRUCT * const tc) {
    double speed = fabs(emcmotStatus->spindle_fb.velocity_rps);
    double pos_error = speed * tc->uu_per_rev;
    tc->target_vel = pos_error;
    emcmotStatus->spindle_sync_state = SYNC_VELOCITY;
}

#if 0
/**
 * A function that looks like sqrt but is flatter, and does not have infinite slope at x = 0 for c > 0
 * @pre c >= 0, x >= 0
 * @return
 */
static inline double pseudo_sqrt_cexpr(double x, double c)
{
    const double den = (pmSqrt(c + 1) - pmSqrt(c));
    const double b0 = -pmSqrt(c
    const double b1 = 1;
    return (b1 * pmSqrt(x+c) + b0) / den;
}
#endif

static int pos_sync_error_reported = 0;
void checkPositionSyncError(TP_STRUCT const *tp, TC_STRUCT const *tc)
{
    const double max_allowed_error = emcmotConfig->maxPositionTrackingError;
    if (!pos_sync_error_reported
            && emcmotStatus->spindle_sync_state == SYNC_POSITION
            && fabs(emcmotStatus->pos_tracking_error) > max_allowed_error) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Spindle position tracking error exceeds limit of %f on line %d\n",
                        max_allowed_error,
                        tc->id);
        print_json5_log_start(tpSyncPosition);
        print_json5_long_long_("time_ticks", tp->exec.time_elapsed_ticks);
        print_json5_double_("current_vel", emcmotStatus->current_vel);
        print_json5_double_("time", tp->exec.time_elapsed_sec);
        print_json5_double_("spindle_revs", emcmotStatus->spindle_fb.position_rev);
        print_json5_double_("spindle_speed_rps", get_spindle_speed_out_rpm(emcmotStatus) / 60.);
        print_json5_double_("spindle_speed_out", get_spindle_speed_out_rpm(emcmotStatus));
        print_json5_double_("spindle_speed_cmd_rpm", emcmotStatus->spindle_cmd.velocity_rpm_out);
        print_json5_double_("spindle_tracking_lookahead_steps", emcmotStatus->spindle_tracking_lookahead_steps);
        print_json5_long_("pos_tracking_mode", emcmotStatus->pos_tracking_mode);
        print_json5_double_("pos_tracking_velocity", emcmotStatus->pos_tracking_velocity);
        print_json5_double_("pos_tracking_error", emcmotStatus->pos_tracking_error);
        print_json5_end_();
        pos_sync_error_reported = 1;
    }
}

void clearPositionSyncErrors()
{
    pos_sync_error_reported = 0;
}

#define PSEUDO_SQRT_EPSILON 0.001
double pseudo_sqrt(double x)
{
    const double sqrt_c = sqrt(PSEUDO_SQRT_EPSILON);
    const double den = (sqrt(PSEUDO_SQRT_EPSILON + 1) - sqrt_c);
    const double b0 = -sqrt_c;
    const double b1 = 1;
    return (b1 * pmSqrt(x + PSEUDO_SQRT_EPSILON) + b0) / den;
}

/**
 * Run position mode synchronization.
 * Updates requested velocity for a trajectory segment to track the spindle's position.
 */
void tpSyncPositionMode(
    TP_STRUCT * const tp,
    TC_STRUCT * const tc)
{
    
    // For position synced next moves, they haven't actually moved yet, so there's no "error" to correct yet, so the best guess of the target we'll want at the start is assuming perfect tracking (which will be updated when the segment is actually activated)
    // The real choice in the main update function is non-synched (target is just reqvel * scale), or synched either v/p which uses velocity-mode-style target vel
    // the ONLY use of target vel in nexttc (while it's not active) is to be a limit on final vel so we don't overshoot the next segments velocity

    // Start with raw spindle position and our saved offset
    double spindle_pos_raw = emcmotStatus->spindle_fb.position_rev;

    const double spindle_vel_rps = findSpindleVelocity(emcmotStatus->spindle_fb.velocity_rps, tp->exec.spindle_cmd.origin);
    // Estimate spindle position delta (2 steps seems to minimize physical position error
    const double lookahead_steps = CLAMP(emcmotStatus->spindle_tracking_lookahead_steps, 0, 100);
    const double spindle_lookahead_delta = spindle_vel_rps * tp->cycleTime * lookahead_steps;

    // Note that this quantity should be non-negative under normal conditions.
    double spindle_displacement_measured = findSpindleDisplacement(spindle_pos_raw,
                                                          tp->exec.spindle_cmd.origin);
    double spindle_displacement = spindle_displacement_measured + spindle_lookahead_delta;

    double v_final = spindle_vel_rps * tc->uu_per_rev;

    // Multiply by user feed rate to get equivalent desired position
    const double pos_desired = (spindle_displacement) * tc->uu_per_rev;
    double net_progress = tc->progress;

    const double pos_error = pos_desired - net_progress;

    double a_max = tcGetTangentialMaxAccel(tc);

#ifdef TC_DEBUG
    print_json5_log_start(tpSyncPositionMode);
    const double vel_error_before = v_final - tc->target_vel;
    print_json5_double(vel_error_before);
    print_json5_double(pos_error);
    print_json5_double(spindle_displacement);
    print_json5_double(spindle_lookahead_delta);
    print_json5_double(spindle_pos_raw);
    print_json5_double(pos_desired);
    print_json5_double(net_progress);
#endif

    switch (emcmotStatus->spindle_sync_state) {
    case SYNC_INACTIVE:
    case SYNC_VELOCITY:
    case SYNC_SEEK_INDEX:
        // not valid here
        break;

    case SYNC_TRIGGER_WAIT:
        tc->target_vel = 0.0;
        if (spindle_displacement < tp->exec.spindle_cmd.trigger_revs) {
            break;
        }
        emcmotStatus->spindle_sync_state = SYNC_ACCEL_RAMP;
    case SYNC_ACCEL_RAMP:
        // Axis has caught up to spindle once position error goes positive
        // note that it may not be perfectly synced at this point, but it should be pretty close
        if (pos_error >= 0) {
            emcmotStatus->spindle_sync_state = SYNC_POSITION;
        }
    case SYNC_POSITION:
        if (tc->on_final_decel) {
            emcmotStatus->spindle_sync_state = SYNC_FINAL_DECEL;
        }
    case SYNC_FINAL_DECEL:
        // we have synced the beginning of the move as best we can -
        // track position (minimize pos_error).
        // This is the velocity we should be at when the position error is c0

        /*
         * In general, position tracking works by perturbing the
         * velocity-synced feed up or down to correct for transient position
         * errors.  If position error is 0 (e.g. a perfect spindle and
         * encoder), then the behavior is identical to velocity-synced motion.
         *
         * velocity
         * |          v_p
         * |         /\
         * |        /..\         v_0
         * |--------....-----------
         * |        ....
         * |        ....
         * |_________________________
         *         |----| t      time
         *
         * To correct a position error x_err (shaded area above), we need to
         * momentarily increase the velocity to catch up, then drop back to the
         * sync velocity.
         *
         * In effect, this is the trapezoidal velocity planning problem, if:
         * 1) remaining distance dx = x_err
         * 2) "final" velocity = v_0
         * 3) max velocity / acceleration from motion segment
         */

        switch(emcmotStatus->pos_tracking_mode) {
        case POS_TRACK_MODE_LEGACY:
        {
            // LinuxCNC 2.6 approach to spindle tracking (high jitter when
            // position error is very small due to slope of sqrt for small
            // values)
            double v_sq = a_max * pos_error;
            double v_target_stock = SIGN(v_sq) * pmSqrt(fabs(v_sq)) + v_final;
            tc->target_vel = v_target_stock;
            break;
        }
        case POS_TRACK_MODE_FLATTENED:
        {
            // Experimental spindle tracking that adds correction terms using a pythagorean sum
            double v_sq = a_max * pos_error;
            double v_target_flat = SIGN(v_sq) * pseudo_sqrt(fabs(v_sq)) + v_final;
            tc->target_vel = v_target_flat;
            break;
        }
        case POS_TRACK_MODE_TRAPEZOIDAL:
        {
            double v_max = tc->maxvel_geom;
            // Use trapezoidal velocity calculation to find target velocity
            // NOTE: this tracking method is smoother but has larger average tracking errors; it's here mostly for compatibility
            double v_target_trapz = fmin(findTrapezoidalDesiredVel(a_max, pos_error, v_final, tc->currentvel, tc->cycle_time), v_max);
            tc->target_vel = v_target_trapz;
            break;
        }
        }
    }
    emcmotStatus->pos_tracking_velocity = tc->target_vel;
    emcmotStatus->pos_tracking_error = pos_error;

    //Finally, clip requested velocity at zero
    if (tc->target_vel < 0.0) {
        tc->target_vel = 0.0;
    }

#ifdef TC_DEBUG
    print_json5_double_("target_vel", tc->target_vel);
    tp_debug_json5_log_end("sync position updated");
#endif
}

void reportTPFatalAddCurrentPosError(TP_STRUCT const *tp, TC_STRUCT const *tc)
{
    // Any failure here is fatal since assumptions are violated about the displacement output
    // As such we should disable motion as fast as possible to prevent unexpected motion.
    emcmotStatus->request_enable = 0;

    // Now, print a dump of everything relevant to the console to diagnose what caused it
    print_json5_start_();
    print_json5_string_("log_entry","tpAddCurrentPosError");
    print_json5_array_start_("joint_data");
    for (int j=0; j<num_joints; ++j) {
        emcmot_joint_t const *jtmp = joints + j;
        if ( !GET_JOINT_ACTIVE_FLAG(jtmp) || !GET_JOINT_ENABLE_FLAG(jtmp) ) {
            continue;
        }

        print_json5_object_start_(NULL);
        print_json5_int_("joint_num", j);
        print_json5_double_("ferror", jtmp->ferror);
        print_json5_double_("ferror_limit", jtmp->ferror_limit);
        print_json5_double_("min_ferror", jtmp->min_ferror);
        print_json5_double_("pos_cmd", jtmp->pos_cmd);
        print_json5_double_("pos_fb", jtmp->pos_fb);
        print_json5_double_("vel_cmd", jtmp->vel_cmd);
        print_json5_double_("vel_cmd_inst", jtmp->vel_cmd_inst);
        print_json5_double_("ferr_high_mark", jtmp->ferror_high_mark);
        print_json5_object_end_();
    }
    print_json5_array_end_();

    print_json5_long_long_("time_ticks", tp->exec.time_elapsed_ticks);
    print_json5_tc_id_data_(tc);
    print_json5_string_("motion_type_name", tcMotionTypeAsString(tc->motion_type));
    print_json5_int_("motion_type", tc->motion_type);
    print_json5_int_field(tc, canon_motion_type);
    print_json5_string_("canon_type_name", tcCanonMotionTypeAsString(tc->canon_motion_type));
    print_json5_int_("active_axes", tc->tag.fields[GM_FIELD_FEED_AXES]);
    print_json5_double_field(tc, target);
    print_json5_double_field(tc, progress);
    // Velocity settings
    print_json5_double_field(tc, reqvel);
    print_json5_double_field(tc, target_vel);
    print_json5_double_("finalvel", tcGetPlanFinalVel(tc));
    print_json5_double_("finalvel_plan", tcGetFinalVelInternal(tc));
    print_json5_double_("finalvel_raw", tc->shared.final_vel_limit);
    print_json5_double_("optimization_state", tc->shared.optimization_state);
    print_json5_double_field(tc, kink_vel);
    print_json5_int_field(tc, use_kink);
    int blend_mode = 0;
    if (tc->blend_mode.mode == TC_TERM_COND_PARABOLIC) {
        blend_mode = 2;
    } else if (tc->blend_mode.mode == TC_TERM_COND_TANGENT) {
        blend_mode = tc->use_kink ? 3 : 4;
    }
    print_json5_int(blend_mode);
    // Acceleration settings
    print_json5_double_("accel_scale", tcGetAccelScale(tc));
    print_json5_double_("acc_overall", tcGetOverallMaxAccel(tc));
    print_json5_double_("acc_tangential", tcGetTangentialMaxAccel(tc));
    print_json5_bool_("accel_ramp", tc->accel_mode);
    print_json5_string_("sync_mode", tcSyncModeAsString(tc->synchronized));

    print_json5_int_("term_cond", tc->blend_mode.mode);
    print_json5_int_field(tc, id);
    print_json5_int_field(tc, needs_spindle_atspeed);

    print_json5_end_();
}

/**
        TC_STRUCT *next2tc = tcqItem(&tp->queue, 2);
 * Do a complete update on one segment.
 * Handles the majority of updates on a single segment for the current cycle.
 */
static bool checkRigidTapOverrun(TP_STRUCT *tp, TC_STRUCT const *tc)
{
    if (   tc->motion_type == TC_RIGIDTAP
        && tc->coords.rigidtap.state == RIGIDTAP_REVERSING
        && !tp->exec.aborting
        && (tc->on_final_decel || tcGetDistanceToGo(tc, tp->exec.reverse_run) < tc->coords.rigidtap.tap_uu_per_rev)) {
        // We're going to hit the bottom of the hole (spindle has overshot) and are losing synchronization
        LineDescriptor linedesc = formatLinePrefix(&tc->tag);
        tpStopWithError(tp, "%sSpindle reversal exceeded maximum overshoot in rigid tapping cycle. Increase cycle depth or reduce spindle speed.",
                        linedesc.buf);
        return true;
    }
    return false;
}

int tpFindDisplacementForSegment(TP_STRUCT * const tp,
    TC_STRUCT * const tc,
    TC_STRUCT * const nexttc,
    UpdateCycleMode cycle_mode)
{
    if (tc->motion_type == TC_DWELL) {
        // Knock one cycle off the remaining dwell time
        tc->coords.dwell.remaining_time = fmax(tc->coords.dwell.remaining_time - tc->cycle_time, 0.0);
        tc->complete = (tc->coords.dwell.remaining_time == 0.0);
        tc->currentvel = 0.0;
        tc->progress = 0;
        tc->on_final_decel = false;
        return TP_ERR_OK;
    }

    //placeholders for position for this update
    PmVector before = tcGetPos(tc);

    // Run cycle update with stored cycle time
    double acc=0, vel_desired=0;

    double const distance_to_go = tcGetDistanceToGo(tc, tp->exec.reverse_run);
    double const v_final = tpGetRealFinalVel(tp, tc, nexttc);
    double const a_max_tangential = tcGetTangentialMaxAccel(tc);
    // Find maximum allowed velocity from feed and machine limits
    // Store a copy of final velocity

    tp_debug_json5_log_start(tpFindDisplacementForSegment);

#ifdef TC_DEBUG
    const double v_max = tpGetRealMaxTargetVel(tp, tc);
    const double v_target = tpGetRealAbsTargetVel(tp, tc);
    const double v_target_next = tpGetRealAbsTargetVel(tp, nexttc);
    print_json5_double_("v_current", tc->currentvel);
    print_json5_double_("dt_plan", tc->cycle_time);
#endif

    // First, check if we can reach the end of this motion segment within this cycle (and handle splitting here)
    EndCondition ec = checkEndCondition(
        tc->cycle_time,
        distance_to_go,
        tc->currentvel,
        v_final,
        a_max_tangential
        );

    switch (ec.end_condition) {
    case END_CONDITION_NOOP:
        tp_debug_json5_log_end("noop update");
        return TP_ERR_OK;

    case END_CONDITION_COMPLETE:
        // Reached the end within one timestep, so wrap up the segment based on final conditions
        tc->complete = true;
        tc->currentvel = ec.v_at_endpt;
        tc->cycle_time = ec.completion_time;
        vel_desired = v_final; // We want to be at the planned final velocity but might not be
        acc = ec.acc;
        tc->progress = tcGetTarget(tc, tp->exec.reverse_run);
        tc->vel_error = v_final- tc->currentvel;
#ifdef TC_DEBUG
        print_json5_string_("end_condition_check", "complete");
        print_json5_double_("dt_limited", ec.completion_time);
        print_json5_double_("v_next_limited", ec.v_at_endpt);
        print_json5_bool_("seg_is_complete", ec.end_condition);
        print_json5_string_("accel_mode", "final_ramp");
#endif
        break;

    case END_CONDITION_NORMAL:
    {
#ifdef TC_DEBUG
        print_json5_string_("end_condition_check", "normal");
#endif
        int res_accel = 1;
        if (tc->accel_mode && tc->blend_mode.mode == TC_TERM_COND_TANGENT) {
            // If the slowdown is not too great, use velocity ramping instead of trapezoidal velocity
            // Also, don't ramp up for parabolic blends
            res_accel = tpCalculateRampAccel(tp, tc, &acc, &vel_desired, v_final);
        }

        // Check the return in case the ramp calculation failed, fall back to trapezoidal
        if (res_accel != TP_ERR_OK) {
            tpCalculateTrapezoidalAccel(tp, tc, &acc, &vel_desired, v_final);
        }

        // If the resulting velocity is less than zero, than we're done. This
        // causes a small overshoot, but in practice it is very small.
        double v_next = tc->currentvel + acc * tc->cycle_time;
        tc->vel_error = fmax(v_next - vel_desired, 0.0);

        double displacement = (tc->currentvel + v_next) / 2.0 * tc->cycle_time;
        tc->currentvel = v_next;
        double disp_sign = tp->exec.reverse_run ? -1. : 1.;
        tc->progress += (disp_sign * displacement);

        //Progress has to be within the allowable range
        tc->progress = CLAMP(tc->progress, 0.0, tc->target);

        tc->complete = false;

#ifdef TC_DEBUG
        int accel_mode_ramp = (res_accel == TP_ERR_OK);
        print_json5_double_("v_next", v_next);
        print_json5_double_("displacement", displacement);
        print_json5_string_("accel_mode", accel_mode_ramp ? "ramp" : "trapezoidal");
#endif
        break;
    }
    }

#ifdef TC_DEBUG
        print_json5_string_("cycle", cycleModeToString(cycle_mode));
        print_json5_double_("acc", acc);
        print_json5_double_("vel_desired", vel_desired);
#endif

    tc->on_final_decel = (fabs(vel_desired - tc->currentvel) < TP_VEL_EPSILON) && (acc < 0.0);

#ifdef TC_DEBUG
    {
        /* Debug Output (inserted into caller's output)*/
        print_json5_long_long_("time_ticks", tp->exec.time_elapsed_ticks);
        print_json5_tc_id_data_(tc);
        print_json5_string_("motion_type", tcMotionTypeAsString(tc->motion_type));
        if (tc->motion_type == TC_RIGIDTAP) {
            print_json5_int_("rigid_tap_phase", tc->coords.rigidtap.state);
        }
        print_json5_double(v_target);
        print_json5_double(v_target_next);
        print_json5_double(v_final);
        print_json5_double_("v_final_limit",tc->shared.final_vel_limit);
        print_json5_double(v_max);
        print_json5_double_("v_error", tc->vel_error);
        print_json5_double_("a_max_path", tc->maxaccel);
        print_json5_double_("a_max_tangential", tcGetTangentialMaxAccel(tc));
        print_json5_double_("kink_accel_reduce", tc->kink_accel_reduce);
        print_json5_double_("kink_vel", tc->kink_vel);
        print_json5_double_("use_kink", tc->use_kink);
        print_json5_double_("target", tcGetTarget(tc, tp->exec.reverse_run));
        print_json5_double_("distance_to_go", tcGetDistanceToGo(tc, tp->exec.reverse_run));
        print_json5_double_("v_current", tc->currentvel);
        print_json5_double_("a_current", acc);
        print_json5_double_("feed_scale", tpGetRealAbsFeedScale(tp, tc));
        print_json5_double_("dt", tc->cycle_time);
        print_json5_bool_("reverse_run", tp->exec.reverse_run);
        print_json5_string_("term_cond", tcTermCondAsString((tc_term_cond_t)tc->blend_mode.mode));
        print_json5_bool_("final_decel", tc->on_final_decel);
        print_json5_bool_("complete", tc->complete);
        print_json5_long_("canon_type", tc->canon_motion_type);
    }
#endif

    PmVector displacement = tcGetPos(tc);
    VecVecSubEq(&displacement, &before);
    tp_debug_json5_PmVector(displacement);
    tp_debug_json5_log_end("done");

    //Store displacement (checking for valid pose)
    int res_set = tpAddCurrentPos(tp, &displacement);
    if (res_set) {
        reportTPFatalAddCurrentPosError(tp, tc);
    }

    return res_set;
}

/**
 * Send default values to status structure.
 */
int tpUpdateStatusCommon(TP_STRUCT const * const tp) {
    // Update queue length
    emcmotStatus->tcqlen = tcqLen(&tp->queue);
    // Set default value for requested speed
    emcmotStatus->requested_vel = 0.0;
    emcmotStatus->excess_vel = 0.0;
    emcmotStatus->current_vel = 0.0;
    emcmotStatus->dwell_time_remaining = 0.0;
    emcmotStatus->tp_waiting = 0;
    return TP_ERR_OK;
}

void clearSpindleSyncStatus()
{
    emcmotStatus->spindle_sync_state = SYNC_INACTIVE;
    emcmotStatus->spindle_fb.synced = 0;
}

/**
 * Calculate an updated goal position for the next timestep.
 * This is the brains of the operation. It's called every TRAJ period and is
 * expected to set tp->currentPos to the new machine position. Lots of other
 * const tp fields (depth, done, etc) have to be twiddled to communicate the
 * status; I think those are spelled out here correctly and I can't clean it up
 * without breaking the API that the TP presents to motion.
 */
tp_err_t updateSyncTargets(TP_STRUCT *tp, TC_STRUCT *tc, TC_STRUCT *nexttc)
{
    // Clear old tracking status (will be updated in the appopriate handler)
    clearPosTrackingStatus();

    /
    /// Need to really separte this so we can in one shot compute the current target velocity
    switch (tc->synchronized) {
        case TC_SYNC_NONE:
            clearSpindleSyncStatus();
            if (tc->motion_type != TC_RIGIDTAP) {
                // Get the maximum allowed target velocity, and make sure we're below it
                // Note that rigid tapping moves during retract / final placement set target velocity directly
                tc->target_vel = tc->reqvel;
            }
            break;
        case TC_SYNC_VELOCITY:
            // Update target velocities
            tpSyncVelocityMode(tc);
            break;
        case TC_SYNC_POSITION:
            tpSyncPositionMode(tp, tc);
            checkPositionSyncError(tp, tc);
            break;
    }
    if (nexttc) {
        switch (nexttc->synchronized) {
        case TC_SYNC_NONE:
            // Get the maximum allowed target velocity, and make sure we're below it
            nexttc->target_vel = nexttc->reqvel;
            break;
        case TC_SYNC_VELOCITY:
        case TC_SYNC_POSITION:
            // Until next segment is actually active, assume we'll get to the
            // endpoint with no position error (i.e. nominal velocity as
            // determined by spindle speed and feed rate)
            {
                double speed = fabs(emcmotStatus->spindle_fb.velocity_rps);
                double pos_error = speed * nexttc->uu_per_rev;
                nexttc->target_vel = pos_error;
            }
            break;
        }
    }
    return TP_ERR_OK;
}

static bool checkWaitForExactStop(TP_STRUCT const *tp, TC_STRUCT const *tc)
{
    bool needs_full_stop = (tc->blend_mode.mode == TC_TERM_COND_EXACT && tcGetFinalVelLimitInternal(tc) == 0.0) ||
        tc->blend_mode.mode == TC_TERM_COND_STOP;
    // Wait for filters to be 1 step away from drained (meaning they'll be empty at the end of the cycle).
    // The count gets updated at the end of the cycle, hence the explicit check here.
    // This check preserves backwards compatibility when filters are disabled so we don't add extra delays.
    return (needs_full_stop && tp->exec.joint_filter_drain_counter > 1);
}

int tpRunCycleInternal(TP_STRUCT * const tp)
{
    /* Get pointers to current and relevant future segments. It's ok here if
     * future segments don't exist (NULL pointers) as we check for this later).
     */
    TC_STRUCT *tc = tcqItem(&tp->queue, 0); //!< Pointer to current motion segment or NULL
    
    TC_STRUCT *nexttc = tp->exec.reverse_run ? NULL : tcqItem(&tp->queue, 1); //!< Pointer to "next" motion segment or NULL

    // Check for any conditions that would cause TP to stop motion (abort, pause, empty queue, etc.)
    // Also checks joint filters and waits to declare motion done until they're drained
    if (TP_ERR_OK != tpHandleStopConditions(tp, tc, nexttc)) {
        return TP_ERR_STOPPED;
    }

    tpUpdateStatusCommon(tp);

    //Return early if we have a reason to wait (i.e. not ready for motion)
    if (tpCheckWaitConditions(tp, tc) != TP_ERR_OK){
        emcmotStatus->tp_waiting = tpIsWaiting(tp);
        return TP_ERR_WAITING;
    }

    tp_err_t res_activate = tpActivateSegment(tp, tc);
    if (res_activate < TP_ERR_OK) {
        tpStopWithError(tp, "Aborting motion due to planning error at line %d", tp->exec.execTag.fields[GM_FIELD_LINE_NUMBER]);
        return res_activate;
    } else if (res_activate > TP_ERR_OK) {
        tp->exec.time_at_wait = tp->exec.time_elapsed_ticks;
        emcmotStatus->tp_waiting = tpIsWaiting(tp);
        return res_activate;
    }

    // End of preparation, running beyond this point means the TP is generating new positions

    // Preprocess rigid tap move (handles threading direction reversals)
    if (tc->motion_type == TC_RIGIDTAP) {
        tpUpdateRigidTapState(tp, tc, nexttc);
    }

    // If synchronized with spindle, calculate requested velocity to track spindle motion
    // NOTE: only need to do this once per cycle
    updateSyncTargets(tp, tc, nexttc);

    tpFindDisplacementForSegment(tp, tc, nexttc, UPDATE_NORMAL);

    //Update status for a normal step
    tpToggleDIOs(tc);
    tpUpdateMovementStatus(tp, tc, nexttc); // tc exists

    // Done movement for current segment, now check if there are any failure / wait conditions

    if (checkRigidTapOverrun(tp, tc)) {
        return TP_ERR_FAIL;
    }
    if (checkWaitForExactStop(tp, tc)) {
        return TP_ERR_WAITING;
    }

    // Nothing to wait on if we get here, so check if there's any remaining
    // cycle time and start moving the next segment if possible.
    if (nexttc && tc->blend_mode.mode == TC_TERM_COND_TANGENT && tc->complete) {
        // Splitting with next segment assuming a non-zero
        double leftover_time = tp->cycleTime - tc->cycle_time;
        double transition_velocity = tc->currentvel;
        nexttc->currentvel = transition_velocity;

        TC_STRUCT *next2tc = tp->exec.reverse_run ? NULL : tcqItem(&tp->queue, 2);
        if (leftover_time > TP_POS_EPSILON) {
            nexttc->cycle_time = leftover_time;

            tp->exec.motionType = nexttc->canon_motion_type;
            tcCheckAccelMode(tp, nexttc);

            // NOTE: in spindle-sync, target velocity is copied over to nexttc already

            tpFindDisplacementForSegment(tp, nexttc, NULL, UPDATE_SPLIT);

            // We're in the next motion segment, so set DIOs and report
            tpToggleDIOs(nexttc);
            tpUpdateMovementStatus(tp, nexttc, next2tc);
        }
    }

    // If TC is complete, remove it from the queue.
    if (tc->complete) {
        tpCompleteSegment(tp, tc);
    }
    return TP_ERR_OK;
}

double tpGetCurrentVel(TP_STRUCT const * const tp, PmVector const * const v_current, int *pure_angular)
{
    PmCartesian xyz, abc, uvw;
    VecToCart(v_current, &xyz, &abc, &uvw);
    double v_out = 0.0;
    *pure_angular = 0;
    switch (tpGetExecTag(tp).fields[GM_FIELD_FEED_AXES]) {
    case 0:
        break;
    case 1:
        pmCartMag(&xyz, &v_out);
        break;
    case 2:
        pmCartMag(&uvw, &v_out);
        break;
    case 3:
        pmCartMag(&abc, &v_out);
        *pure_angular = 1;
        break;
    }
    return v_out;
}

int tpRunCycle(TP_STRUCT *tp)
{
    // Before every TP update, ensure that elapsed time and
    // TP measurements are stored for error checks
    tp->exec.time_elapsed_sec+=tp->cycleTime;
    ++tp->exec.time_elapsed_ticks;
    PmVector const axis_pos_old = tp->exec.currentPos;

    tp_err_t res = TP_ERR_OK;
    for (int i=0; i < MAX(tp->superSampleRate, 1); ++i) {
        res = tpRunCycleInternal(tp);
        if (res == TP_ERR_WAITING || res == TP_ERR_STOPPED) {
            break;
        }
    }

    // After update (even a no-op), update pos / vel / accel
    PmVector const axis_vel_old = tp->exec.currentVel;
    PmVector const axis_pos = tp->exec.currentPos;

    PmVector axis_vel = VecVecSub(axis_pos, &axis_pos_old);
    VecScalMultEq(&axis_vel, 1.0 / tpGetCycleTime(tp));
    tp->exec.currentVel = axis_vel;

    emcmotStatus->current_vel = tpGetCurrentVel(tp, &tp->exec.currentVel, &emcmotStatus->pure_angular_move);
    emcmotStatus->excess_vel = emcmotStatus->current_vel - emcmotStatus->requested_vel;

    if (needConsistencyCheck(CCHECK_AXIS_LIMITS)) {
        PmVector axis_accel = VecVecSub(axis_vel, &axis_vel_old);
        VecScalMultEq(&axis_accel, 1.0 / tpGetCycleTime(tp));

        unsigned accel_error_mask = findAccelViolations(axis_accel);
        unsigned vel_error_mask = findVelocityViolations(axis_vel);
        //unsigned pos_limit_error_mask = findMaxPositionViolations(axis_pos);

        reportTPAxisError(tp, accel_error_mask, "Acceleration limit exceeded");
        reportTPAxisError(tp, vel_error_mask, "Velocity limit exceeded");
        //reportTPAxisError(tp, pos_limit_error_mask, "Position limits exceeded");

        if ((_tc_debug && !tpIsDone(tp)) || (
                    accel_error_mask | vel_error_mask)
                ) {
            print_json5_log_start(tpRunCycle);
            print_json5_long_long_("time_ticks", tp->exec.time_elapsed_ticks);
            print_json5_PmVector(axis_pos);
            print_json5_PmVector(axis_vel);
            print_json5_PmVector(axis_accel);
            print_json5_unsigned(accel_error_mask);
            print_json5_unsigned(vel_error_mask);
            double current_vel = emcmotStatus->current_vel;
            print_json5_double(current_vel);
            TC_STRUCT const *tc = tpGetCurrentSegment(tp);
            if (tc) {
                print_json5_tc_id_data_(tc);
                print_json5_TC_STRUCT_kinematics("tc_kins", tc);
                print_json5_TC_STRUCT_geometry("tc_geom", tc);
            }
            print_json5_double_("time", tp->exec.time_elapsed_sec);
            print_json5_end_();

        }

        if (res == TP_ERR_OK && accel_error_mask | vel_error_mask) {
            return TP_ERR_GEOM;
        }
    }
    return res;
}

int tpPause(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->exec.pausing = 1;
    return TP_ERR_OK;
}

int tpResume(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->exec.pausing = 0;
    return TP_ERR_OK;
}

int tpAbort(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    if (!tp->exec.aborting) {
        /* const to abort, signal a pause and set our abort flag */
        tpPause(tp);
        tp->exec.aborting = 1;
    }

    
    return tpClearDIOs(tp); //clears out any already cached DIOs
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->exec.motionType;
}

int tpGetPos(TP_STRUCT const * const tp, EmcPose * const pos)
{

    if (0 == tp) {
        ZERO_EMC_POSE((*pos));
        return TP_ERR_FAIL;
    } else {
        pmVectorToEmcPose(&tp->exec.currentPos, pos);
    }

    return TP_ERR_OK;
}

EmcPose tpGetCurrentPos(const TP_STRUCT * const tp)
{
    EmcPose out={};
    if (tp) {
        pmVectorToEmcPose(&tp->exec.currentPos, &out);
    }

    return out;
}

void tpSetFilterStatus(TP_STRUCT * const tp, bool at_rest)
{
    tp->exec.filters_at_rest = at_rest;
}

int tpIsDone(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return checkJointFiltersEmpty(tp) && !tcqLen(&tp->queue);
}

int tpQueueDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tcqLen(&tp->queue);
}

int tpActiveDepth(TP_STRUCT * const tp)
{
    if (0 == tp) {
        return TP_ERR_OK;
    }

    return tp->exec.activeDepth;
}

int tpSetAout(TP_STRUCT * const tp, unsigned char index, double start, double end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->planner.syncdio.anychanged = 1; //something has changed
    tp->planner.syncdio.aio_mask |= (1 << index);
    tp->planner.syncdio.aios[index] = start;
    return TP_ERR_OK;
}

int tpSetDout(TP_STRUCT * const tp, int index, unsigned char start, unsigned char end) {
    if (0 == tp) {
        return TP_ERR_FAIL;
    }
    tp->planner.syncdio.anychanged = 1; //something has changed
    tp->planner.syncdio.dio_mask |= (1 << index);
    if (start > 0)
        tp->planner.syncdio.dios[index] = 1; // the end value can't be set from canon currently, and has the same value as start
    else
        tp->planner.syncdio.dios[index] = -1;
    return TP_ERR_OK;
}

int tpSetRunDir(TP_STRUCT * const tp, tc_direction_t dir)
{
    if (tpIsMoving(tp)) {
        return TP_ERR_FAIL;
    }

    switch (dir) {
        case TC_DIR_FORWARD:
        case TC_DIR_REVERSE:
            tp->exec.reverse_run = dir;
            return TP_ERR_OK;
    }
    rtapi_print_msg(RTAPI_MSG_ERR,"Invalid direction flag in SetRunDir");
    return TP_ERR_FAIL;
}

WaitFlagMask tpIsWaiting(TP_STRUCT const * const tp)
{
    WaitFlagMask mask = 0;
    for (int k=0; k < MAX_WAIT_INDICES; ++k) {
        if (tp->exec.waiting[k] != MOTION_INVALID_ID) {
            mask |= 1 << k;
        }
    }
    return mask;
}

WaitFlagMask tpIsWaitingOnSegment(TP_STRUCT const * const tp, TC_STRUCT const *tc)
{
    WaitFlagMask mask = 0;
    if (!tp || !tc) {
        return mask;
    }

    for (int k=0; k < MAX_WAIT_INDICES; ++k) {
        if (tp->exec.waiting[k] == tc->id) {
            mask |= 1 << k;
        }
    }
    return mask;
}

bool tpIsMoving(TP_STRUCT const * const tp)
{

    
    if (emcmotStatus->current_vel >= TP_VEL_EPSILON ) {
        tp_debug_print("TP moving, current_vel = %.16g\n", emcmotStatus->current_vel);
        return true;
    } else if (tpIsWaiting(tp)) {
        tp_debug_print("TP is waiting on an external state\n");
        return true;
    }
    return false;
}

bool tpIsStopping(TP_STRUCT const * const tp)
{
    return tp->exec.aborting;
}


TC_STRUCT const *tpGetCurrentSegment(TP_STRUCT *tp)
{
    return tcqItem(&tp->queue, 0);
}

static inline LineDescriptor formatLinePrefix(struct state_tag_t const *tag)
{
    LineDescriptor linebuf;
    linebuf.buf[0]='\0';
    int line = tag->fields[GM_FIELD_LINE_NUMBER];
    int local_line = tag->fields[GM_FIELD_LOCAL_LINE_NUMBER];
    const int len = sizeof(linebuf.buf);
    if (line > 0) {
        if (local_line > 0 && local_line != line) {
            rtapi_snprintf(linebuf.buf, len, "Line %d, local line %d: ", line, local_line);
        } else {
            rtapi_snprintf(linebuf.buf, len, "Line %d: ", line);
        }
    }
    linebuf.buf[len-1]='\0';
    return linebuf;
}

/**
 * Abort the trajectory planner and raise a motion error so that the program is forced to stop.
 *
 * FIXME-USPACE handle userspace motion planning with some explicit synchronization (e.g. drop out of coord mode)
 */
void tpStopWithError(TP_STRUCT *tp, const char *fmt, ...)
{
    if (fmt) {
        va_list args;
        va_start(args, fmt);
#ifdef UNIT_TEST
        fprintf(stderr, fmt, args);
#else
        enqueueError(fmt, args);
#endif
        va_end(args);
    }
    tpAbort(tp);
    SET_MOTION_ERROR_FLAG(1);
}

void cancel_probing()
{
    static const probe_mode_t cancelled={};
    // Clear all probe config
    emcmotStatus->probe_mode = cancelled;
    emcmotStatus->probing = false;
}


const char *wait_type_as_str(WaitFlagIndex idx)
{
    switch (idx)
    {
    case WAIT_FOR_SPINDLE_INDEX:
        return "spindle at-speed";
    case WAIT_FOR_SPINDLE_ATSPEED:
        return "spindle index";
    case WAIT_FOR_PROBE_READY:
        return "probe ready";
    case WAIT_FOR_PROBING:
        return "probing";
    case WAIT_FOR_OPTIMIZATION:
        return "optimization";
    case WAIT_FOR_INDEXER_UNLOCK:
        return "indexer unlock";
    case WAIT_FOR_FILTER_DISABLE:
        return "joint filter disable";
    case WAIT_FOR_FILTER_ENABLE:
        return "joint filter enable";

    // Add new entries here
    case MAX_WAIT_INDICES:
        break;
    }
    return "unknown";
}


// vim:sw=4:sts=4:et:
