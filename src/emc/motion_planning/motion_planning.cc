/**
 * @file tp.cc
 * Userspace motion planning
 *
 * Copyright (c) 2022 All rights reserved.
 */

#ifdef TP_LOG_PARSING
#undef TP_DEBUG
#endif

//#define MOTION_PLANNING_DEBUG
#include "motion_planning_debug.hh"

#include "math.h"

#include "posemath.h"           /* Geometry types & functions */
#include "tc.hh"
#include "tcq.hh"
#include "motion_planning.hh"
#include "emcpose.h"
#include "motion.h"
#include "spherical_arc9.hh"
#include "blendmath.hh"
#include "math_util.h"
#include "joint_util.hh"
#include "string.h"
#include "tp_call_wrappers.hh"
#include "error_util.h"

#include "emc_messages.hh"
#include <algorithm>
#include <vector>
#include "stdio.h"

#include "motion_planning_config.hh"

#include "motion_type_defaults.h"

#include <iostream>
#include <iomanip>
#include <ctime>

#ifdef UNIT_TEST
#include "stdio.h"
#endif



// Absolute maximum distance (in revolutions) to add to the end of a rigidtap move to account for spindle reversal
static const double RIGIDTAP_MAX_OVERSHOOT_REVS = 10.0;

#ifdef TP_OPTIMIZER_LOGGING
static clock_t start_time=clock();
#endif

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

// NOTE: turned off this feature, which causes blends between rapids to
// use the feed override instead of the rapid override
#undef TP_SHOW_BLENDS

#define TP_OPTIMIZATION_LAZY

static TC_STRUCT blend_tc1 = {};
static TC_STRUCT blend_tc2 = {};
static tc_unique_id_t cached_blend_id_1 = -1;
static tc_unique_id_t cached_blend_id_2 = -1;

/** static function primitives (ugly but less of a pain than moving code around)*/

/**
 * Get scaled-down acceleration bounds for planning new moves.
 * These may be as large as actual axis limits.
 */
static inline PmVector getPlanningAccelBounds(TC_STRUCT const *prev_tc, TC_STRUCT const *tc) {
    // NOTE: range checking is done at config level for accel scale
    return VecScalMult(getAccelBounds(), std::min(prev_tc->accel_scale, tc->accel_scale));
}

static int is_feed_type(EMCMotionTypes motion_type)
{
    switch(motion_type) {
    case EMC_MOTION_TYPE_RIGIDTAP:
    case EMC_MOTION_TYPE_ARC:
    case EMC_MOTION_TYPE_FEED:
    case EMC_MOTION_TYPE_PROBING:
        return 1;
    case EMC_MOTION_TYPE_NONE:
    case EMC_MOTION_TYPE_TOOLCHANGE:
    case EMC_MOTION_TYPE_TRAVERSE:
    case EMC_MOTION_TYPE_INDEXROTARY:
        return 0;
    }
    return 1;
}

int tpSetSpindleOn(tp_planning_t &planner, spindle_cmd_t &cmd, bool wait_for_atspeed)
{
    planner.spindle_cmd =  cmd;
    planner.atspeed_next_feed = wait_for_atspeed;
    planner.wait_for_speed_change = wait_for_atspeed;
    return 0;
}

int tpSetSpindleOff(tp_planning_t &planner)
{
    planner.spindle_cmd = spindle_cmd_t{};
    planner.atspeed_next_feed = false;
    // NOTE: assume a speed change means a new wait by default
    planner.wait_for_speed_change = true;
    return 0;
}

int tpSetSpindleSpeed(tp_planning_t planner, spindle_cmd_t &cmd)
{
    planner.spindle_cmd = cmd;
    // Check if we need to wait at the next feed move based on the last spindle-on command
    planner.atspeed_next_feed = planner.wait_for_speed_change;
    return 0;
}

/**
 * If for whatever reason we don't end up using a motion segment, then the wait
 * conditions assigned to it may have to be reused for the next segment (like
 * for a zero-length move).
 */
void tpRestoreWaitConditions(tp_planning_t *planning, TC_STRUCT *tc_to_skip)
{
    planning->atspeed_next_feed |= tc_to_skip->needs_spindle_atspeed;
    planning->wait_for[WAIT_FOR_PROBE_READY] |= tc_to_skip->needs_probe_ready;
}

bool tpCheckNeedsAtSpeed(tp_planning_t *planning, EMCMotionTypes motion_type)
{
    char issue_atspeed = 0;
    if(planning->atspeed_next_feed && is_feed_type(motion_type) ) {
        issue_atspeed = 1;
        planning->atspeed_next_feed = 0;
    }
    if(!is_feed_type(motion_type) && planning->spindle_cmd.css_factor) {
        planning->atspeed_next_feed = 1;
    }
    return issue_atspeed;
}

bool tpCheckNeedsProbeReady(tp_planning_t *planning)
{
    if (planning->wait_for[WAIT_FOR_PROBE_READY]) {
        planning->wait_for[WAIT_FOR_PROBE_READY] = false;
        return true;
    }
    return false;
}

int tpSetEnableFeedhold(tp_planning_t *planner, bool allow_feedhold)
{
    if (allow_feedhold) {
        planner->enables_new |= FH_ENABLED;
    } else {
        planner->enables_new &= ~FH_ENABLED;
    }
    return 0;
}

int tpSetEnableFeedScale(tp_planning_t *planner, bool allow_feed_scale)
{
    if (allow_feed_scale) {
        planner->enables_new |= FS_ENABLED;
    } else {
        planner->enables_new &= ~FS_ENABLED;
    }
    return 0;
}

int tpSetEnableSpindleScale(tp_planning_t *planner, bool allow_spindle_scale)
{
    if (allow_spindle_scale) {
        planner->enables_new |= SS_ENABLED;
    } else {
        planner->enables_new &= ~SS_ENABLED;
    }
    return 0;
}

int tpSetEnableAdaptiveFeed(tp_planning_t *planner, bool allow_adaptive_feed)
{
    if (allow_adaptive_feed) {
        planner->enables_new |= AF_ENABLED;
    } else {
        planner->enables_new &= ~AF_ENABLED;
    }
    return 0;
}

/**
 * @section tpgetset Internal Get/Set functions
 * @brief Calculation / status functions for commonly used values.
 * These functions return the "actual" values of things like a trajectory
 * segment's feed override, while taking into account the status of tp itself.
 */

/**
 * Wrapper to bounds-check the tangent kink ratio from HAL.
 */
double tpGetTangentKinkRatio(void) {
    const double max_ratio = 0.7071;
    const double min_ratio = 0.001;

    return fmax(fmin(emcmotConfig->arc_blend_cfg.tangent_kink_ratio,max_ratio),min_ratio);
}

/**
 * @section tpaccess tp class-like API
 */

static constexpr const syncdio_t empty_syncdio{};
/**
 * Clears any potential DIO toggles and anychanged.
 * If any DIOs need to be changed: dios[i] = 1, DIO needs to get turned on, -1
 * = off
 */
int tpClearDIOs(TP_STRUCT * const tp) {
    tp->planner.syncdio = syncdio_t{};
    return TP_ERR_OK;
}

int tpUpdateConfig(TP_STRUCT * const tp)
{
    return updateTPConfig();
}

/**
 *    "Soft initialize" the trajectory planner tp.
 *    This is a "soft" initialization in that TP_STRUCT configuration
 *    parameters (cycleTime, vMax, and aMax) are left alone, but the queue is
 *    cleared, and the flags are set to an empty, ready queue. The currentPos
 *    is left alone, and goalPos is set to this position.  This function is
 *    intended to put the motion queue in the state it would be if all queued
 *    motions finished at the current position.
 */
int tpClearPlanning(TP_STRUCT * const tp)
{
    if (!tp) {
        return 0;
    }

    tp->planner.atspeed_next_feed = false;
    tp->planner.spindle_cmd = spindle_cmd_t{};
    tp->planner.nextId = 0;

    updateTPConfig();

    return tpClearDIOs(tp);
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
    return tp->cycleTime * tp->superSampleRate;
}

/**
 * Sets the id that will be used for the next appended motions.
 * nextId is incremented so that the next time a motion is appended its id will
 * be one more than the previous one, modulo a signed int. If you want your own
 * ids for each motion, call this before each motion you append and stick what
 * you want in here.
 */
int tpSetId(TP_STRUCT * const tp, int id)
{

    if (!MOTION_ID_VALID(id)) {
        return TP_ERR_FAIL;
    }

    if (0 == tp) {
        return TP_ERR_FAIL;
    }

    tp->planner.nextId = id;

    return TP_ERR_OK;
}

tc_unique_id_t tpGetNextUniqueId(TP_STRUCT * const tp)
{
    return tp->planner.nextUniqueId++;
}

void tcSetId(TP_STRUCT * const tp, TC_STRUCT * const tc, struct state_tag_t tag)
{
    
    // delays blend creation by one segment, so we need to keep the old unique
    // ID's around for them to be monotonic
    blend_tc1.unique_id = cached_blend_id_1;
    blend_tc2.unique_id = cached_blend_id_2;
    cached_blend_id_1 = tpGetNextUniqueId(tp);
    cached_blend_id_2 = tpGetNextUniqueId(tp);
    tc->unique_id = tpGetNextUniqueId(tp);
    tc->id = tp->planner.nextId;
    // Invalidate the blend arc ID and tag (populated during blending)
    blend_tc1.id = blend_tc2.id = MOTION_INVALID_ID;
    tc->tag = tag;
    static const struct state_tag_t empty_tag = {};
    blend_tc1.tag = empty_tag;
    blend_tc2.tag = empty_tag;
}

int tpGetQueuedId(const TP_STRUCT * tp)
{
    if (!tp) {
        return 0;
    }

    TC_STRUCT const * const tc_last = tcqBack(&tp->queue, 0);
    if (!tc_last) {
        return 0;
    }
    return tc_last->id;
}


/**
 * Sets the termination condition for all subsequent queued moves.
 * If cond is TC_TERM_COND_STOP, motion comes to a stop before a subsequent move
 * begins. If cond is TC_TERM_COND_PARABOLIC, the following move is begun when the
 * current move slows below a calculated blend velocity.
 *
 * @deprecated
 */
int tpSetTermCond(TP_STRUCT * const tp, tc_term_cond_t cond, double tolerance)
{
    switch (cond) {
        //Purposeful waterfall for now
        case TC_TERM_COND_PARABOLIC:
        case TC_TERM_COND_TANGENT:
        case TC_TERM_COND_EXACT:
        case TC_TERM_COND_STOP:
        if (tp) {
            tp->planner.blend_mode.mode = cond;
            tp->planner.blend_mode.explicit_tolerance = tolerance;
            tp->planner.blend_mode.req_min_radius = 0;
            tp->planner.blend_mode.contour_tolerance = 0;
            return TP_ERR_OK;
        }
            break;
    }

    return TP_ERR_FAIL;
}

int tpSetBlendMode(TP_STRUCT * const tp, blend_mode_t blend_mode)
{
    tp->planner.blend_mode = blend_mode;
    return TP_ERR_OK;
}

/**
 * Check for valid tp before queueing additional moves.
 */
int tpErrorCheck(TP_STRUCT const * const tp) {

    if (!tp) {
        emcOperatorError("TP is null\n");
        return TP_ERR_FAIL;
    }
    if (tp->exec.aborting) {
        emcOperatorError("TP is aborting\n");
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

/**
 * Initialize a blend arc from its parent lines.
 * This copies and initializes properties from the previous and next lines to
 * initialize a blend arc. This function does not handle connecting the
 * segments together, however.
 */
int tpInitBlendArcFromAdjacent(
    TC_STRUCT const * const adjacent_tc,
    TC_STRUCT* const blend_tc,
    BlendParameters const *param,
    double vel,
    double ini_maxvel,
    double acc,
    tc_motion_type_t motion_type) {

#ifdef TP_SHOW_BLENDS
    int canon_motion_type = EMC_MOTION_TYPE_ARC;
#else
    int canon_motion_type = adjacent_tc->canon_motion_type;
#endif

    tcInit(blend_tc,
            motion_type,
            canon_motion_type,
            adjacent_tc->cycle_time,
            adjacent_tc->enables); // NOTE: blend arc never needs the atspeed flag, since the previous line will have it (and cannot be consumed).

    blend_tc->tag = adjacent_tc->tag;
    // KLUDGE tell an observer that the current segment is generated by the TP as part of a blend
    blend_tc->tag.packed_flags |= (0x1 << GM_FLAG_TP_BLEND);

    // Copy over sync state data from segment directly (Note that TP planning state may have changes since blend calculation lags behind)
    blend_tc->blend_mode = adjacent_tc->blend_mode;
    blend_tc->synchronized = adjacent_tc->synchronized;
    blend_tc->uu_per_rev = adjacent_tc->uu_per_rev;

    blend_tc->blend_mode.mode = TC_TERM_COND_TANGENT;

    // find "helix" length for target
    double length;
    double v_max = ini_maxvel;
    switch (motion_type) {
    case TC_LINEAR:
    {
        length = blend_tc->coords.line.tmag;
        break;
    }
    case TC_SPHERICAL:
    {
        length = arc9Length(&blend_tc->coords.arc);

        auto const &arc = blend_tc->coords.arc;
        double eff_radius = spiralEffectiveRadius(arc.radius, arc.angle, arc.spiral);
        double v_max_filter = std::min(findMaxVelocityWithFilteringActive(eff_radius), ini_maxvel);
        CircleAccLimits limits = arc9AccLimit(eff_radius, v_max_filter, param->a_max_planar);
        v_max = limits.v_max;
        blend_tc->acc_ratio_tan = limits.acc_ratio;
        break;
    }
    default:
        return TP_ERR_FAIL;
    }

    // Set kinematics parameters from blend calculations
    tcSetupMotion(blend_tc,
            vel,
            v_max,
            acc);


    blend_tc->target = length;
    blend_tc->nominal_length = length;
    blend_tc->syncdio = adjacent_tc->syncdio;

    //NOTE: blend arc radius and everything else is finalized, so set this to 1.
    tcSetOptimizationState(blend_tc, TC_PLAN_GEOMETRY_PLANNED);

    return TP_ERR_OK;
}

#if 0

int tcSetLineXYZ(TC_STRUCT * const tc, PmCartLine const * const line)
{

    //Update targets with new arc length
    if (!line || tc->motion_type != TC_LINEAR) {
        return TP_ERR_FAIL;
    }
    if (!tc->coords.line.abc.tmag_zero || !tc->coords.line.uvw.tmag_zero) {
        emcOperatorError("SetLineXYZ does not supportABC or UVW motion\n");
        return TP_ERR_FAIL;
    }

    tc->coords.line = *line;
    tc->target = line->tmag;
    return TP_ERR_OK;
}
#endif

int find_max_element(double arr[], int sz)
{
    if (sz < 1) {
        return -1;
    }
    // Assumes at least one element
    int max_idx = 0;
    for (int idx = 0; idx < sz; ++idx) {
        if (arr[idx] > arr[max_idx]) {
            max_idx = idx;
        }
    }
    return max_idx;
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

bool isArcBlendFaster(TC_STRUCT const * const prev_tc, double expected_v_max)
{
    return prev_tc->kink_vel < expected_v_max;
}

/**
 * Last step in setting up blend conditions between segments.
 */
void tpFallbackToTangent(
        TC_STRUCT * const tc)
{
    if (!tc) {
        return;
    }

    tcSetTermCond(tc, TC_TERM_COND_TANGENT);
    tc->use_kink = 1;
    tcFinalizeLength(tc, emcmotConfig->maxFeedScale);
}

/**
 * Verify continuity after all the slicing and dicing is done stitching the trajectory segments back together.
 * If segments are NOT continuous and tangent, change the blend mode to exact
 * stop to prevent overruns. This should not happen under normal operation and is
 * a sign of a bug in the planner.
 *
 * This function will stop the TP if the result is discontinuous (should never happen), and sets the appropriate blend mode
 * for each segment if all segments are properly C1 continuous.
 */
tp_err_t tpVerifyBiarcContinuity(
    TP_STRUCT * tp,
    TC_STRUCT * const prev_tc,
    TC_STRUCT * const first_blend_tc,
    TC_STRUCT * const second_blend_tc,
    TC_STRUCT const * const this_tc,
    bool consume)
{
    double const c0_pos_limit = emcmotConfig->consistencyCheckConfig.continuityPositionLimit_uu;
    double const c1_angle_limit = emcmotConfig->consistencyCheckConfig.continuityAngleLimit_rad;
    ContinuityCheck c1 = checkContinuity(prev_tc, first_blend_tc, c0_pos_limit, c1_angle_limit, consume);
    ContinuityCheck cmid = checkContinuity(first_blend_tc, second_blend_tc, c0_pos_limit, c1_angle_limit, false);
    ContinuityCheck c2 = checkContinuity(second_blend_tc, this_tc, c0_pos_limit, c1_angle_limit, false);

    bool continuity_error = !(c1.is_C0_continuous && cmid.is_C0_continuous && c2.is_C0_continuous);
    bool tangency_error = !(c1.is_C1_continuous && cmid.is_C1_continuous && c2.is_C1_continuous);

    tcSetTermCond(prev_tc, c1.is_C1_continuous ? TC_TERM_COND_TANGENT : TC_TERM_COND_STOP);
    tcSetTermCond(first_blend_tc, cmid.is_C1_continuous ? TC_TERM_COND_TANGENT : TC_TERM_COND_STOP);
    tcSetTermCond(second_blend_tc, c2.is_C1_continuous ? TC_TERM_COND_TANGENT : TC_TERM_COND_STOP);

    if (_tp_debug || tangency_error || continuity_error)
    {
        print_json5_log_start(tpVerifyBiarcContinuity);
        print_json5_long_long_("prev_line", prev_tc->tag.fields[GM_FIELD_LINE_NUMBER]);
        print_json5_long_long_("this_line", prev_tc->tag.fields[GM_FIELD_LINE_NUMBER]);
        print_json5_long_long_("prev_unique_id", prev_tc->unique_id);
        print_json5_long_long_("this_unique_id", this_tc->unique_id);
        print_json5_string_("prev_motion_type", tcMotionTypeAsString(prev_tc->motion_type));
        print_json5_string_("b1_motion_type", tcMotionTypeAsString(first_blend_tc->motion_type));
        print_json5_string_("b2_motion_type", tcMotionTypeAsString(second_blend_tc->motion_type));
        print_json5_string_("this_motion_type", tcMotionTypeAsString(this_tc->motion_type));
        print_json5_PmVector_("u1_diff", c1.u_diff);
        print_json5_PmVector_("p1_diff", c1.p_diff);
        print_json5_double_("u1_diff_dot", c1.dot);
        print_json5_PmVector_("u_mid_diff", cmid.u_diff);
        print_json5_PmVector_("p_mid_diff", cmid.p_diff);
        print_json5_double_("u_mid_diff_dot", cmid.dot);
        print_json5_PmVector_("u2_diff", c2.u_diff);
        print_json5_double_("u2_diff_dot", c2.dot);
        print_json5_PmVector_("p2_diff", c2.p_diff);
        print_json5_bool_("prev_C0", c1.is_C0_continuous);
        print_json5_bool_("mid_C0", cmid.is_C0_continuous);
        print_json5_bool_("next_C0", c2.is_C0_continuous);
        print_json5_bool_("prev_C1", c1.is_C1_continuous);
        print_json5_bool_("mid_C1", cmid.is_C1_continuous);
        print_json5_bool_("next_C1", c2.is_C1_continuous);
        print_json5_log_end();
    }
    return (tangency_error || continuity_error) ? TP_ERR_UNRECOVERABLE : TP_ERR_OK;
}

tp_err_t tpCreateBiarcBlend(TP_STRUCT * const tp, TC_STRUCT * const prev_tc, TC_STRUCT * const this_tc)
{
#ifdef TP_DEBUG
    double prev_len = prev_tc->target;
    double this_len = this_tc->target;

    double maxvel = fmin(
        tcGetPlanMaxTargetVel(prev_tc, emcmotConfig->biarc_solver_cfg.feed_override_allowance),
        tcGetPlanMaxTargetVel(this_tc, emcmotConfig->biarc_solver_cfg.feed_override_allowance));
    double reqvel = fmin(prev_tc->reqvel, this_tc->reqvel);
    tc_motion_type_t prev_motion_type = prev_tc->motion_type;
    tc_motion_type_t this_motion_type = this_tc->motion_type;
    PmVector u1_stretch={}, u2_stretch={};
    PmVector p1_stretch={}, p2_stretch={};
#endif

    PmVector prev_end;
    tcGetEndpoint(prev_tc, &prev_end);
    PmVector this_start;
    tcGetStartpoint(this_tc, &this_start);
    double dev = VecVecDisp(&prev_end, &this_start);
    if (fabs(dev) > TP_POS_EPSILON) {
        tp_debug_json5_log_start(tpCreateBiarcBlend);
        tp_debug_json5_PmVector(prev_end);
        tp_debug_json5_PmVector(this_start);
        tp_debug_json5_log_end("End points don't match up");
        return TP_ERR_FAIL;
    }

    PmVector vel_bound = getVelBounds();
    PmVector acc_bound = getPlanningAccelBounds(prev_tc, this_tc);

    static biarc_solver_results_t biarc_results = {};
    memset(&biarc_results, 0, sizeof(biarc_solver_results_t));

    BlendControls controls = {};
    CHP(optimize_biarc_blend_size(
        prev_tc,
        this_tc,
        &emcmotConfig->biarc_solver_cfg,
        &vel_bound,
        &acc_bound,
        &biarc_results,
        &controls,
        tp->cycleTime));

    biarc_control_points_t const * const control_pts = &biarc_results.solution.control_pts;
    blend_boundary_t const * const boundary = &biarc_results.solution.boundary;

    BlendPoints points1 = {}, points2 = {};
    BlendParameters const *param1 = &biarc_results.solution.param1;
    BlendParameters const *param2 = &biarc_results.solution.param2;

    CHP(find_arc_points_from_solution(
        &boundary->u1,
        &control_pts->u_mid,
        &boundary->P1,
        &control_pts->P_mid,
        &control_pts->Pb1,
        control_pts->d1,
        &controls,
        param1,
        &points1));

    // Check if previous segment can be consumed
    if (param1->consume) {
        // Force the start point to be the previous line's starting point since it's going to be consumed
        // This fixes any miniscule position mismatches at the previous segment
        // endpoint. The resulting arc may be slightly less circular, but this
        // is factored in when creating the final arc geometry.
        tcGetStartpoint(prev_tc, &points1.arc_start) ;
    }

    // Initial work is complete, now populate the blend segments

    CHP(init_blend_segment_geometry(&blend_tc1, &points1));


    CHP(find_arc_points_from_solution(
        &control_pts->u_mid,
        &boundary->u2,
        &control_pts->P_mid,
        &boundary->P2,
        &control_pts->Pb2,
        control_pts->d2,
        &controls,
        param2,
        &points2));
    CHP(init_blend_segment_geometry(&blend_tc2, &points2));

    // Don't bother if another solution is better
    double expected_v_max = fmin(param1->v_plan, param2->v_plan);
    tp_debug_json5_double(expected_v_max);
    if (emcmotConfig->arc_blend_cfg.allow_fallback && !isArcBlendFaster(prev_tc, expected_v_max)) {
        tp_debug_json5_log_end("Aborting arc blend creation, other methods are faster");
        return TP_ERR_NO_ACTION;
    }

    // Copy deviation estimate for circumscribe / inscribe correction for planning only
    if (blend_tc1.motion_type == TC_SPHERICAL) {
        blend_tc1.coords.arc.estimated_dev = biarc_results.solution.T_plan;
    }

    union {PmLine9 line9; PmCircle9 circle9;} prev_geom={}, this_geom={};

    if (!param1->consume) {
        switch (prev_tc->motion_type) {
        case TC_LINEAR:
        {
            prev_geom.line9 = prev_tc->coords.line;
            double new_len = fmax(boundary->s1, 0.);
            pmLine9Cut(&prev_geom.line9, new_len, KEEP_START_PT);
            break;
        }
        case TC_CIRCULAR:
        {
            prev_geom.circle9 = prev_tc->coords.circle;
            pmCircle9Cut(&prev_geom.circle9, boundary->s1, KEEP_START_PT);
            break;
        }
        default:
            return TP_ERR_FAIL;
        }
    }

    switch (this_tc->motion_type) {
    case TC_LINEAR:
    {
        this_geom.line9 = this_tc->coords.line;
        double new_len = fmax(boundary->s2, 0.);
        pmLine9Cut(&this_geom.line9, new_len, KEEP_END_PT);
        break;
    }
    case TC_CIRCULAR:
    {
        this_geom.circle9 = this_tc->coords.circle;
        pmCircle9Cut(&this_geom.circle9, boundary->s2, KEEP_END_PT);
        break;
    }
    default:
        return TP_ERR_FAIL;
    }

    CHP(tpInitBlendArcFromAdjacent(
        prev_tc,
        &blend_tc1,
        param1,
        controls.v_req,
        param1->v_plan,
        param1->a_max_planar,
        blend_tc1.motion_type));
    CHP(tpInitBlendArcFromAdjacent(
        this_tc,
        &blend_tc2,
        param2,
        controls.v_req,
        param2->v_plan,
        param2->a_max_planar,
        blend_tc2.motion_type));

    // Passed all the pre-checks, ready to commit to changing the line segments
    if (!param1->consume) {
        if (prev_tc->motion_type == TC_LINEAR) {
            tcSetLine9(prev_tc, &prev_geom.line9);
        } else if (prev_tc->motion_type == TC_CIRCULAR) {
            tcSetCircle9(prev_tc, &prev_geom.circle9);
        }
    }

    if (this_tc->motion_type == TC_LINEAR) {
        tcSetLine9(this_tc, &this_geom.line9);
    } else if (this_tc->motion_type == TC_CIRCULAR) {
        tcSetCircle9(this_tc, &this_geom.circle9);
    }

    // Update ID and tag of blend arcs to match the previous and next segments
    blend_tc1.id = prev_tc->id;
    blend_tc1.tag = prev_tc->tag;
    blend_tc2.id = this_tc->id;
    blend_tc2.tag = this_tc->tag;

    // The DIO's only change if the previous segment is consumed
    blend_tc1.syncdio.anychanged &= param1->consume;
    // Second arc should follow DIO's of the following segment (since the midpoint is the closest to the ideal endpoint)
    // DIOs have already changed by the time the next segment starts, so ignore changes here
    this_tc->syncdio.anychanged = 0;

    // Copy this over for easy debugging later
    blend_tc1.original_end_pt = prev_tc->original_end_pt;

#ifdef TP_DEBUG
    tp_debug_json5_log_start(tpCreateBiarcBlend);
    if (param1->consume) {
        tcGetStartTangentUnitVector(prev_tc, &u1_stretch);
        tcGetStartpoint(prev_tc, &p1_stretch);
    } else {
        tcGetEndTangentUnitVector(prev_tc, &u1_stretch);
        tcGetEndpoint(prev_tc, &p1_stretch);
    }
    tcGetStartTangentUnitVector(this_tc, &u2_stretch);
    tcGetStartpoint(this_tc, &p2_stretch);
    print_json5_double_field(&controls, correction_dist);
    print_json5_double(prev_len);
    print_json5_double(this_len);
    print_json5_double_("prev_len_new", prev_tc->target);
    print_json5_double_("this_len_new", this_tc->target);
    print_json5_double_("blend_arc_length", biarc_results.solution.arc_len_est);
    print_json5_int(prev_motion_type);
    print_json5_int(this_motion_type);
    print_json5_long_long_("src_line", this_tc->tag.fields[GM_FIELD_LINE_NUMBER]);
    print_json5_long_long_("prev_unique_id", prev_tc->unique_id);
    print_json5_long_long_("unique_id1", blend_tc1.unique_id);
    print_json5_long_long_("unique_id2", blend_tc2.unique_id);
    print_json5_long_long_("this_unique_id", this_tc->unique_id);
    print_json5_biarc_solution_t("solution", &biarc_results.solution);
    print_json5_PmVector(u1_stretch);
    print_json5_PmVector(u2_stretch);
    print_json5_PmVector_("p1_stretch", p1_stretch);
    PmVector p11_blend;
    tcGetStartpoint(&blend_tc1, &p11_blend);
    PmVector p12_blend;
    tcGetEndpoint(&blend_tc1, &p12_blend);
    PmVector p21_blend;
    tcGetStartpoint(&blend_tc2, &p21_blend);
    PmVector p22_blend;
    tcGetEndpoint(&blend_tc2, &p22_blend);
    print_json5_PmVector_("p11_blend", p11_blend);
    print_json5_PmVector_("p12_blend", p12_blend);
    print_json5_PmVector_("orig_endpoint", prev_tc->original_end_pt);
    print_json5_PmVector_("p21_blend", p21_blend);
    print_json5_PmVector_("p22_blend", p22_blend);
    print_json5_PmVector_("p2_stretch", p2_stretch);
    print_json5_biarc_control_points_t("control_pts", &biarc_results.solution.control_pts);
    print_json5_blend_boundary_t("boundary", &biarc_results.solution.boundary);
    print_json5_int_("iterations", biarc_results.iterations);
    print_json5_string_("result", biarc_result_to_str(biarc_results.result));
    print_json5_double_("deviation", biarc_results.solution.err.deviation_margin);
    print_json5_double_("tolerance", controls.path_tolerance);
    PmVector disp_final;
    tcGetEndpoint(&blend_tc1, &disp_final);
    VecVecSubEq(&disp_final, &prev_tc->original_end_pt);
    double final_dev = VecMag(&disp_final);
    print_json5_double_("endpt_deviation", final_dev);
    print_json5_TC_STRUCT_kinematics("prev_perf", prev_tc);
    print_json5_TC_STRUCT_kinematics("blend1_perf", &blend_tc1);
    print_json5_TC_STRUCT_kinematics("blend2_perf", &blend_tc2);
    print_json5_TC_STRUCT_kinematics("this_performance", this_tc);
    print_json5_double(maxvel);
    print_json5_double(reqvel);
    print_json5_bool_("consume", param1->consume);
    print_json5_SphericalArc9("arc1", &blend_tc1.coords.arc);
    print_json5_SphericalArc9("arc2", &blend_tc2.coords.arc);
    print_json5_end_();
#endif

    // NOTE: C1 continuity check is now required as it is used to decide if blend was successful
    CHP(tpVerifyBiarcContinuity(tp, prev_tc, &blend_tc1, &blend_tc2, this_tc, param1->consume));

    if (param1->consume) {
        if (tcqPopBack(&tp->queue)) {
            //This is unrecoverable since we've already changed the line. Something is wrong if we get here...
            emcOperatorError("Motion internal error: previous motion segment at line %d (id %d) is missing, trajectory is invalid\n", this_tc->tag.fields[GM_FIELD_LINE_NUMBER], this_tc->id);
            
            return TP_ERR_UNRECOVERABLE;
        }
    }

    return TP_ERR_OK;
}

/**
 * Assumes that the segment is linear (or behavior is undefined)
 */
static void computeLineLengthAndTarget(TC_STRUCT * tc, PmVector const *start, PmVector const *end)
{
    pmLine9Init(&tc->coords.line, start, end);
    tc->target = tc->nominal_length = pmLine9Length(&tc->coords.line);
}

/**
 * Adjusts the end point / start point of a pair of lines based on the estimated blend deviation.
 */
tp_err_t applyProfileCorrection(TP_STRUCT *tp, TC_STRUCT const * prev2_tc, TC_STRUCT *prev_tc, TC_STRUCT *this_tc)
{
    if (!prev_tc ||
        prev_tc->motion_type != TC_LINEAR ||
        !this_tc ||
        this_tc->motion_type != TC_LINEAR ||
        this_tc->blend_mode.contour_tolerance <= 0.0 ||
        prev_tc->blend_mode.mode != TC_TERM_COND_PARABOLIC)
    {
        // No correction is possible for non-linear or missing segments (or if profile correction is disabled)
        return TP_ERR_NO_ACTION;
    }

    // Do a throwaway calculation just to figure out what the normal direction / deviation is (for the nominal shape)
    // KLUDGE back up the old stuff
    PmVector prev_start_pt = prev_tc->coords.line.start;
    computeLineLengthAndTarget(prev_tc, prev2_tc ? &prev2_tc->original_end_pt : NULL, &prev_tc->original_end_pt);
    computeLineLengthAndTarget(this_tc, &prev_tc->original_end_pt, NULL); // NOTE: tc endpoint is still the nominal end point

    PmVector new_endpt = prev_tc->original_end_pt;

    this_tc->correction_dist = do_line_profile_correction(
        &prev_tc->coords.line,
        &this_tc->coords.line,
        this_tc->blend_mode.contour_tolerance,
        &new_endpt,
        prev_tc->id,
        this_tc->id);

    // After end-point adjustment, we need to recompute the line segment and lengths
    computeLineLengthAndTarget(prev_tc, &prev_start_pt, &new_endpt);
    // NOTE: the incoming segment's endpoint is unchanged
    computeLineLengthAndTarget(this_tc, &new_endpt, NULL);
    return TP_ERR_OK;
}

void reportPlanningErrorContext(tp_planning_t *planner, TC_STRUCT *tc, const char *error_type)
{
    // Now, print a dump of everything relevant to the console
    print_json5_start_();
    print_json5_string_("log_entry",error_type ?: "unknown");
    print_json5_tc_id_data_(tc);
    print_json5_string_("motion_type", tcMotionTypeAsString(tc->motion_type));
    print_json5_double_("target", tc->target);
    print_json5_string_("term_cond", tcTermCondAsString(tc->blend_mode.mode));
    print_json5_end_();
}

/**
 * Add a newly created motion segment to the tp queue.
 * Returns an error code if the queue operation fails, otherwise adds a new
 * segment to the queue and updates the end point of the trajectory planner.
 */
int tpAddSegmentToQueue(TP_STRUCT * const tp, TC_STRUCT * const tc) {

    int retval = tcqPut(&tp->queue, tc);
    if (0 != retval) {
        emcOperatorError("Motion internal error: tcqPut() failed with error code %d", retval);
        reportPlanningErrorContext(&tp->planner, tc, "failed to queue");
        
        return TP_ERR_FAIL;
    }

    // Store end of current move as new final goal of TP
    // KLUDGE: endpoint is garbage for rigid tap since it's supposed to retract past the start point.
    if (tc->motion_type != TC_RIGIDTAP) {
        tcGetEndpoint(tc, &tp->goalPos);
    }

    
    // Move this to control.c?
    // tp->exec.filter_flush_timeout = max(emcmotStatus->mov_avg_window_size, 1);
    //Fixing issue with duplicate id's?
#ifdef TP_DEBUG
    print_json5_log_start(Enqueue);
    print_json5_tc_id_data_(tc);
    print_json5_string_("motion_type", tcMotionTypeAsString(tc->motion_type));
    print_json5_double_("target", tc->target);
    print_json5_string_("term_cond", tcTermCondAsString(tc->blend_mode.mode));
    print_json5_PmVector_("tp_goal_pos", tp->goalPos);
    print_json5_end_();
#endif

    return TP_ERR_OK;
}

int handlePrevTermCondition(TC_STRUCT *prev_tc, TC_STRUCT *tc)
{
    if (!tc) {
        return TP_ERR_NO_ACTION;
    }

    switch (tc->motion_type) {
    case TC_RIGIDTAP:
        if (prev_tc && prev_tc->motion_type != TC_RIGIDTAP) {
            tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
            tcFinalizeLength(prev_tc, emcmotConfig->maxFeedScale);
        }
        // Rigidtap motions are always exact-path to allow pecking
        tcSetTermCond(tc, TC_TERM_COND_EXACT);
        break;
    case TC_LINEAR:
    case TC_CIRCULAR:
    case TC_SPHERICAL:
    {
        tc_term_cond_t prev_term = prev_tc ? prev_tc->blend_mode.mode : TC_TERM_COND_STOP;
        tcSetTermCond(prev_tc, prev_term);
    }
        break;
    case TC_DWELL:
        // Previous motion must do an exact stop at the dwell point since that's the whole point
        tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
        tcFinalizeLength(prev_tc, emcmotConfig->maxFeedScale);
        tcSetTermCond(tc, TC_TERM_COND_STOP);
        return TP_ERR_OK;
    }
    return TP_ERR_OK;
}

int handleModeChange(TC_STRUCT *prev_tc, TC_STRUCT *tc)
{
    if (!tc || !prev_tc) {
        return TP_ERR_FAIL;
    }

    // Can't blend across feed / rapid transitions
    if ((prev_tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE) ^
            (tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE)) {
        tp_debug_print("Blending disabled: rapid / feed transition must follow exact path\n");
        tcSetTermCond(prev_tc, TC_TERM_COND_EXACT);
        if (tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE) {
            // Force retraction moves to follow exact path (to avoid gouging after a hole retraction)
            tcSetTermCond(tc, TC_TERM_COND_EXACT);
        }
    }

    if (prev_tc->synchronized != TC_SYNC_POSITION &&
            tc->synchronized == TC_SYNC_POSITION) {
        tp_debug_print("Blending disabled: entering position-sync mode\n");
        // NOTE: blending IS allowed when exiting position sync mode, so be
        // careful of tolerances here to avoid making thicker threads
        // (particularly if ;the lead-out is perpendicular)
        return tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
    }

    if(tc->needs_spindle_atspeed) {
        // Need to wait for spindle before starting tc, so it's not safe to
        // allow any blending from the previous to current segment
        tp_debug_print("Blending disabled: waiting on spindle atspeed for tc %d, unique_id %llu\n",
                       tc->id, tc->unique_id);
        return tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
    }

    return TP_ERR_OK;
}

int tpSetupSyncedIO(TP_STRUCT * const tp, TC_STRUCT * const tc) {
    if (tp->planner.syncdio.anychanged != 0) {
        tc->syncdio = tp->planner.syncdio; //enqueue the list of DIOs that need toggling
        tpClearDIOs(tp); // clear out the list, in order to prepare for the next time we need to use it
        return TP_ERR_OK;
    } else {
        tc->syncdio.anychanged = 0;
        return TP_ERR_NO_ACTION;
    }
}

tp_err_t tpFinalizeAndEnqueue(TP_STRUCT * const tp, TC_STRUCT * const tc, PmVector const *nominal_goal)
{
    tcGetPosReal(tc, tc->target, &tc->original_end_pt);
    TC_STRUCT *prev2_tc = tcqBack(&tp->queue, -1);
    {
        // NOTE: prev_tc pointer is invalidated after this because of the segment juggling to insert a blend
        TC_STRUCT *prev_tc = tcqBack(&tp->queue, 0);

        // Make sure the blending flags are consistent w/ previous segment
        handlePrevTermCondition(prev_tc, tc);

        // Prevent blends for specific mode changes (where blending isn't possible anyway)
        handleModeChange(prev_tc, tc);

        // Do two-phase blending for correct inscribed arc error
        // NOTE:
        CHF(tpSetupSegmentBlend(tp, prev2_tc, prev_tc, tc));
    }

    tcFinalizeLength(prev2_tc, emcmotConfig->maxFeedScale);

    CHF(tpAddSegmentToQueue(tp, tc));
    //Run speed optimization (will abort safely if there are no tangent segments)
    tp_err_t res_opt = tpOptimizePlannedMotions(tp, emcmotConfig->arc_blend_cfg.optimization_depth);
    if (nominal_goal) {
        // Update the goal position so that future motions know what their planned start point is
        tp->goalPos = *nominal_goal;
    }

    return res_opt;
}

tp_err_t tpForceFinalizeQueue(TP_STRUCT * const tp)
{
    // No more path correction is possible so force any oustanding blends to happen
    TC_STRUCT *prev2_tc = tcqBack(&tp->queue, -1);
    {
        // NOTE: prev_tc pointer is invalidated after this because of the segment juggling to insert a blend
        TC_STRUCT *prev_tc = tcqBack(&tp->queue, 0);

        if (!prev_tc) {
            // Queue is empty so there's nothing to do
            return TP_ERR_OK;
        }
        // Last segment will stop because no more are coming
        prev_tc->blend_mode.mode = TC_TERM_COND_STOP;

        // Force any outstanding blend calculations to be done
        // NOTE: the following functions handle the case where prev2_tc is NULL (i.e. only one segment in the queue)
        CHF(tpSetupSegmentBlend(tp, prev2_tc, prev_tc, NULL));
    }

    // Finalize all segment lengths so optimization is possible
    tcFinalizeLength(prev2_tc, emcmotConfig->maxFeedScale);
    TC_STRUCT *prev_tc_afterblend = tcqBack(&tp->queue, 0);
    tcFinalizeLength(prev_tc_afterblend, emcmotConfig->maxFeedScale);

    // Run the last optimization for all remaining segments
    tp_err_t res_opt = tpOptimizePlannedMotions(tp, emcmotConfig->arc_blend_cfg.optimization_depth);

    return res_opt;
}

/**
 * Tells motion planning that we just requested a power on / off of probe or ETS.
 * This means that the next move has to wait until we've reached that desired state.
 * Examples:
 *  operator loaded the probe in the spindle, machine shouldn't move until probe is powered on
 *  operator stowed the probe, don't move the machine until it's powered off to avoid spurious faults
 */
int tpSetNextWaitForProbeReady(tp_planning_t &planner)
{
    
    planner.wait_for[WAIT_FOR_PROBE_READY] = true;
    return 0;
}

static void clearBlendStructs()
{
    memset(&blend_tc1, 0, sizeof(TC_STRUCT));
    memset(&blend_tc2, 0, sizeof(TC_STRUCT));
}

static void addRigidTapOverrun(TC_STRUCT * const tc, double revolutions, double uu_per_rev)
{
    PmCartLine *actual_xyz = &tc->coords.rigidtap.actual_xyz;
    pmCartLineStretch(actual_xyz, actual_xyz->tmag + revolutions * uu_per_rev, 0);
    tc->target = actual_xyz->tmag;
}

/**
 * Adds a rigid tap cycle to the motion queue.
 */
int tpAddRigidTap(TP_STRUCT * const tp,
                  rigid_tap_cmd_t const &rigid_tap,
    const state_tag_t & tag) {
    if (tpErrorCheck(tp)) {
        return TP_ERR_FAIL;
    }

    PmVector end;
    emcPoseToPmVector(&rigid_tap.pos, &end);

    if(!tp->planner.synchronized) {
        emcOperatorError("Cannot add unsynchronized rigid tap move.\n");
        return TP_ERR_FAIL;
    }

    TC_STRUCT tc = {};
    clearBlendStructs();

    tcSetId(tp, &tc, tag);

    tcInit(&tc,
            TC_RIGIDTAP,
            EMC_MOTION_TYPE_RIGIDTAP,
            tp->cycleTime,
            tp->planner.enables_new);

    // Wait conditions
    tc.needs_spindle_atspeed = tpCheckNeedsAtSpeed(&tp->planner, EMC_MOTION_TYPE_RIGIDTAP);
    tc.needs_probe_ready = tpCheckNeedsProbeReady(&tp->planner);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            rigid_tap.ini_maxvel,
            rigid_tap.ini_maxvel,
            rigid_tap.acc);

    tc.coords.rigidtap.tap_uu_per_rev = rigid_tap.tap_uu_per_rev;
    tc.coords.rigidtap.retract_uu_per_rev = rigid_tap.retract_uu_per_rev;

    // Setup rigid tap geometry
    CHP(pmRigidTapInit(
        &tc.coords.rigidtap,
        &tp->goalPos,
        &end,
        rigid_tap.scale));

    tc.coords.rigidtap.dwell_counts = std::max(0.0, rigid_tap.dwell_time / tp->cycleTime);

    addRigidTapOverrun(&tc, RIGIDTAP_MAX_OVERSHOOT_REVS, tc.coords.rigidtap.tap_uu_per_rev);

    // Do NOT update the goal position with a rigid tap move
    int res_add = tpFinalizeAndEnqueue(tp, &tc, NULL);
    if (TP_ERR_OK != res_add) {
        // Restore the at-speed flag if needed
        tpRestoreWaitConditions(&tp->planner, &tc);
    }
    return res_add;
}

int tpAddDwell(TP_STRUCT * const tp, double time_sec, double delta_rpm, struct state_tag_t const &tag)
{
    // NOTE: it's not possible to dwell for less than 1 servo timestep due to how dwells are implemented
    TC_STRUCT tc={};
    clearBlendStructs();
    tcSetId(tp, &tc, tag);
    tcInit(&tc,
            TC_DWELL,
            0,
            tp->cycleTime,
            0);
    tc.motion_type = TC_DWELL;
    tc.coords.dwell.dwell_time_req = fmax(time_sec, tp->cycleTime);
    tc.coords.dwell.delta_rpm_req = fabs(delta_rpm);
    tc.coords.dwell.dwell_pos = tp->goalPos;
    // Initialize with invalid values (dwell time computed at segment activation)
    tc.coords.dwell.dwell_time = -1;
    tc.coords.dwell.remaining_time = -1;
    tc.blend_mode.mode = TC_TERM_COND_STOP;
    tc.tag = tag;

    return tpFinalizeAndEnqueue(tp, &tc, NULL); // NOTE: no at-speed handling in dwells
}


static double applyKinkVelLimit(TC_STRUCT const * const tc, double vel_in)
{
    if (tc->use_kink && tc->kink_vel >= 0  && tc->blend_mode.mode == TC_TERM_COND_TANGENT) {
        // Only care about kink_vel with tangent segments that have not been arc blended
        return fmin(vel_in, tc->kink_vel);
    }
    return vel_in;
}




static inline void debug_print_tc_opt(TC_STRUCT const *tc, int opt_step, const char *comment,  double acc_this=-1, double vs_back=-1, double vf_limit_prev=-1, double vf_limit_this=-1)
{
    if (!tc) {
        tp_optimizer_print("%d,-1,-1,-1,-1,-1,-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,\"%s\"\n", opt_step, comment ?: "");
    } else {
        tp_optimizer_print("%d,%lld,%d,%d,%d,%d,%d,%0.8f,%0.6f,%0.8f,%0.6f,%0.8f,%0.6f,%0.8f,%0.8f,%0.8f,\"%s\"\n",
                      opt_step,
                      tc->unique_id,
                      tc->tag.fields[GM_FIELD_LINE_NUMBER],
                      tc->motion_type,
                      tc->canon_motion_type,
                      tc->blend_mode.mode,
                      tc->shared.optimization_state,
                      tcGetPlanTargetVel(tc),
                      tcGetFinalVel(tc),
                      tcGetFinalVelLimit(tc),
                      tc->maxaccel,
                      tc->target,
                      acc_this,
                      vs_back,
                           vf_limit_prev,
                           vf_limit_this,
                      comment ?: ""
                      );
    }
}

static inline void debug_print_optimization_state(TC_STRUCT const *tc, TC_STRUCT const *prev1_tc, int opt_step, const char *comment, double acc_this=-1, double vs_back=-1, double vf_limit_prev=-1, double vf_limit_this=-1)
{
    // Print a row for each active segment in the optimization
    debug_print_tc_opt(tc, opt_step, comment, acc_this, vs_back, vf_limit_prev, vf_limit_this);
    debug_print_tc_opt(prev1_tc, opt_step, "");
}

static size_t getOptimizationSearchEnd(SmoothingData const &path_data)
{
    // We pad the WIP data with an extra dummy entry at the beginning and end for the peak-finding algorithm
    return std::min(path_data.v_smooth.size()-1, (size_t)emcmotConfig->arc_blend_cfg.optimization_depth);
}

/**
 * Based on the nth and (n-1)th segment, find a safe final velocity for the (n-1)th segment.
 * This function also caps the target velocity if velocity ramping is enabled. If we
 * don't do this, then the linear segments (with higher tangential
 * acceleration) will speed up and slow down to reach their target velocity,
 * creating "humps" in the velocity profile.
 */
double tpComputeOptimalVelocity(TC_STRUCT const * const tc, TC_STRUCT const * const prev1_tc, double v_f_this, int opt_step) {
    //Calculate the maximum starting velocity vs_back of segment tc, given the
    //trajectory parameters
    double acc_this = tcGetTangentialMaxAccel(tc);

    // Find the reachable velocity of tc, moving backwards in time
    double vs_back = pmSqrt(pmSq(v_f_this) + 2.0 * acc_this * tc->target);
    // Find the reachable velocity of prev1_tc, moving forwards in time

    double vf_limit_this = tcGetPlanMaxTargetVel(tc, emcmotConfig->maxFeedScale);
    double v_max_prev = tcGetPlanMaxTargetVel(prev1_tc, emcmotConfig->maxFeedScale);
    double vf_limit_prev = applyKinkVelLimit(prev1_tc, v_max_prev);
    //Limit the PREVIOUS velocity by how much we can overshoot into
    double vf_reachable = std::min(std::min(vf_limit_this, vf_limit_prev), vs_back);

    const char *comment = "optimizing (before final vel)";
    debug_print_optimization_state(tc, prev1_tc, opt_step, comment, acc_this, vs_back, vf_limit_prev, vf_limit_this);

    return vf_reachable;
}

double interp1(double t0, double t1, double v0, double v1, double t)
{
    if (std::abs(t1-t0) < TP_POS_EPSILON) {
        // KLUDGE not ideal, but if the time interval is so short then it shouln't matter which side we pick
        return v1;
    }
    return (v1-v0)/(t1-t0) * (t-t0) + v0;
}

tp_err_t tpDoPeakSmoothing(SmoothingData &path_data, int optimization_depth, double t_window)
{
    // Start from the optimal solution and smooth it
    SmoothingVector &v_smooth = path_data.v_smooth;
    SmoothingVector const &t = path_data.t;

    if (v_smooth.size() != t.size()) {
        emcOperatorError("Motion Planning assumptions violated: mismatched vector sizes v(%lu) vs t(%lu)", v_smooth.size(), t.size());
        return TP_ERR_FAIL;
    }

    if (v_smooth.empty()) {
        // We should know about this because the constructor starts with 1 element populated
        emcOperatorError("Motion planning assumptions violated: smoothing velocity vector is empty");
        return TP_ERR_FAIL;
    }

    size_t search_depth = getOptimizationSearchEnd(path_data);
    // How much ripple we're willing to tolerate before declaring we've found a trough
    static const constexpr double tol = 1e-6;
    // Search the vectors
    for (size_t k = 1; k < search_depth; ++k) {
        if (!path_data.ignore.at(k) && (v_smooth.at(k) >= (v_smooth.at(k-1))) && (v_smooth.at(k) >= (v_smooth.at(k+1)))) {
            // Found a new local peak at k
            //mp_debug_print("Found a local peak at ID %d, v = %0.17g\n", path_data.unique_id.at(k), v_smooth.at(k));

            // Search each side for the peak size
            double min_v_bwd = v_smooth.at(k);

            size_t k_bwd = k-1;
            for (; k_bwd >= 1; --k_bwd) {
                double v_bwd = v_smooth.at(k_bwd);
                if (v_bwd > (min_v_bwd + tol) || path_data.ignore.at(k_bwd)) {
                    break;
                }
                double t_cutoff = t.at(k) - t_window / 2.;
                if (t.at(k_bwd) < t_cutoff) {
#ifdef INTERP_SMOOTHING_AT_T_WINDOW
                    size_t k0 = k_bwd+1;
                    size_t k1 = k_bwd;
                    double v_bwd = interp1(t.at(k0), t.at(k1), v_smooth.at(k0), v_smooth.at(k1), t_cutoff);
                    min_v_bwd = std::min(min_v_bwd, v_bwd);
#endif
                    break;
                } else {
                    min_v_bwd = std::min(min_v_bwd, v_bwd);
                }
            }
            size_t peak_idx_bwd = k_bwd+1;

            double min_v_fwd = v_smooth.at(k);
            size_t k_fwd = k+1;
            for (; k_fwd < search_depth; ++k_fwd) {
                double v_fwd = v_smooth.at(k_fwd);
                if (v_fwd > (min_v_fwd + tol) || path_data.ignore.at(k_fwd)) {
                    break;
                }
                double t_cutoff = t.at(k) + t_window / 2.;
                if (t.at(k_fwd) > t_cutoff) {
#ifdef INTERP_SMOOTHING_AT_T_WINDOW
                    size_t k0 = k_fwd-1;
                    size_t k1 = k_fwd;
                    double v_fwd = interp1(t.at(k0), t.at(k1), v_smooth.at(k0), v_smooth.at(k1), t_cutoff);
                    min_v_fwd = std::min(min_v_fwd, v_fwd);
#endif
                    break;
                } else {
                    min_v_fwd = std::min(min_v_fwd, v_fwd);
                }
            }
            size_t peak_idx_fwd = k_fwd-1;

#ifdef MOTION_PLANNING_DEBUG
            size_t fwd_id = path_data.unique_id.at(peak_idx_fwd);
            size_t bwd_id = path_data.unique_id.at(peak_idx_bwd);
            size_t peak_id = path_data.unique_id.at(k);

            size_t limiting_id = min_v_bwd > min_v_fwd ? path_data.unique_id.at(peak_idx_bwd) : path_data.unique_id.at(peak_idx_fwd);
#endif
            double edge_val = std::max(min_v_bwd, min_v_fwd);
            mp_debug_print("Smoothing peak at %ld (from %ld to %ld) to max of (%0.17g, %0.17g), v_smooth at edges (%0.17g, %0.17g)\n",
                           peak_id,
                           bwd_id,
                           fwd_id,
                           min_v_bwd,
                           min_v_fwd,
                           path_data.v_smooth.at(peak_idx_bwd),
                           path_data.v_smooth.at(peak_idx_fwd));
            for (size_t j = peak_idx_bwd; j <= peak_idx_fwd; ++j) {
                if (path_data.ignore.at(j)) {
                    emcOperatorError("Motion planning assumptions violated: Cannot smooth an ignored segment at idx %ld", j);
                    return TP_ERR_FAIL;
                }
                // Flatten the peak by forcing intermediate values to the larger of the edge values
                if (v_smooth.at(j)>edge_val) {
#ifdef MOTION_PLANNING_DEBUG
                    path_data.limiting_id.at(j) = limiting_id;
#endif
                    v_smooth.at(j) =  edge_val;
                }
#ifdef MOTION_PLANNING_DEBUG
                path_data.touched.at(j) = true;
#endif
            }
            // Update K value based on forward search
            k = std::max(peak_idx_fwd, k);
        }
    }

    // Update timebase for next iteration
    for (size_t k=1; k < v_smooth.size(); ++k) {
        double v_avg_clamped = std::max((v_smooth.at(k) + v_smooth.at(k-1))/2., TP_VEL_EPSILON);
        double ds_new = path_data.ds.at(k);
        double dt_new = ds_new / v_avg_clamped;
        path_data.t.at(k) = path_data.t.at(k-1) + dt_new;
    }

    return TP_ERR_OK;
}

bool enteringSpindleSync(TC_STRUCT const *prev_tc, TC_STRUCT const *tc)
{
    return (!prev_tc || (TC_SYNC_POSITION != prev_tc->synchronized)) && (tc && TC_SYNC_POSITION == tc->synchronized);
}

bool exitingSpindleSync(TC_STRUCT const *prev_tc, TC_STRUCT const *tc)
{
    return (prev_tc && (TC_SYNC_POSITION == prev_tc->synchronized)) && (tc && TC_SYNC_POSITION != tc->synchronized);
}

bool filter_in_use()
{
    auto const &active = emcmotConfig->joint_filter_cfg;
    return active.window_size_sec != default_joint_filter_cfg.window_size_sec || active.acceleration_scale != default_joint_filter_cfg.acceleration_scale;
}

/**
 * Return true if any of the following wait conditions are met for a pair of segments
 * 1) the next segment requires spindle-at-speed
 * 2) the next segment requires position synchronization with the spindle to start (e.g. beginning of threading or rigid tapping cycle)
 * 3) the next segment is a probe move and needs the probe / toolsetter to be ready
 */
bool tcNeedsWaitAtStart(TC_STRUCT const *prev_tc, TC_STRUCT const * tc)
{
    if (!tc) {
        // Can't wait for a segment that doesn't exist
        return false;
    }
    // Need to wait if we're starting position synchronization because an index might take a while to find.
    // Usually this requires at-speed too but it might not in the future.
    bool needs_synch_wait = enteringSpindleSync(prev_tc, tc) || (exitingSpindleSync(prev_tc, tc) && filter_in_use());
    return tc->needs_spindle_atspeed || needs_synch_wait || !!(tc->probe.active);
}


SmoothingData computeLimitingVelocities(TC_QUEUE_STRUCT *queue,int optimization_depth, double t_window)
{
    SmoothingData path_data{};

    double t_smoothed = 0.0;

    // By definition start from zero final velocity
    // 1) Find maximum reachable velocity at the end of each segment

#ifdef TP_OPTIMIZER_LOGGING
    auto t_now = clock();
    double delta_sec = static_cast<double>( clock () - start_time ) /  CLOCKS_PER_SEC;
    start_time = t_now;
    tp_optimizer_print("Optimization results (compute time %0.3f sec):\nstep,id,src_line,type,canon_type,blend_type,optim_state,max_vel,final_vel,final_vel_limit,maxaccel,dist,acc_this,vs_back,info\n", delta_sec);
#endif
    for (int k = 0; k < optimization_depth; ++k) {
        // Update the pointers to the trajectory segments in use
        TC_STRUCT const *tc = tcqBack(queue, -k);
        TC_STRUCT *prev1_tc = tcqBack(queue, -1 - k);

        if ( !prev1_tc || !tc) {
            debug_print_optimization_state(tc, prev1_tc, k, "Reached end of queue in optimization");
            break;
        }

        
        // be out of date anyway in a planning context. Instead, make it so
        // that any modification to a running segment is always safe via atomic
        // optimization state flag. Any change to final velocity will be an increase, so we won't leave the the segment in an invalid state.
        // NOTE: need to make sure that future feed smoothing work doesn't trip this up either.

        //TCPlanningState const prev_opt_state = tcGetOptimizationState(prev1_tc);
        TCPlanningState const this_opt_state = tcGetOptimizationState(tc);
        double v_f_prev = 0.0;
        double ds = tc->target;

        if (this_opt_state == TC_PLAN_UNTOUCHED) {
            // Segment geometry is not finalized, cannot draw any conclusions about final velocity yet.
            // NOTE: this is somewhat redundant
            ds = 0; // Ignore unfinished segments
            debug_print_optimization_state(tc, prev1_tc, k, "Geometry is not finalized, skipping this segment");
        } else if (prev1_tc->blend_mode.mode != TC_TERM_COND_TANGENT || tc->motion_type == TC_DWELL) {
            prev1_tc->internal.found_final_vel_limit = true; // And that limit is zero...
            tcSetFinalVelLimit(prev1_tc, 0.0); // Likely redundant but be explicit here in case
            debug_print_optimization_state(tc, prev1_tc, k, "Hit non-tangent segment");
            if (k != 0) {
                // Could still be unfinished segments behind this one
                
                break;
            }
        } else if (tcNeedsWaitAtStart(prev1_tc, tc)) {
            //Assume worst case that we have a stop at this point since it must be able to wait for the spindle
            prev1_tc->internal.found_final_vel_limit = true;
            tcSetFinalVelLimit(prev1_tc, 0.0); // Likely redundant but be explicit here in case
            debug_print_optimization_state(tc, prev1_tc, k, "spindle-at-speed required");
            break;
        } else if (prev1_tc->internal.found_final_vel_limit){
            v_f_prev = tcGetFinalVelLimit(prev1_tc);
            debug_print_optimization_state(tc, prev1_tc, k, "already at optimal limit");
        } else {
            // Haven't found the upper limit yet so compute it now
            v_f_prev = tpComputeOptimalVelocity(tc, prev1_tc, tcGetFinalVelLimit(tc), k);
            tcSetFinalVelLimit(prev1_tc, v_f_prev);
            debug_print_optimization_state(tc, prev1_tc, k, "(duplicate row, after update final vel)");
        }

        // Fill the cache vectors
        double v_avg_upper_bound = 0;
        if (prev1_tc->internal.found_final_vel_limit) {
            // Once we converge on a final velocity for each segment, we know the highest average velocity the segment can have.
            v_avg_upper_bound = (tcGetFinalVelLimit(prev1_tc) + v_f_prev)/2.;
        } else {
            // Don't know how fast the segment is allowed to go yet, so assume
            // the worst.  This ensures that early iterations of path smoothing
            // are more aggressive. As more segments are queued, the reachable
            // velocity of segments at the end of the queue increases (in
            // general). This in turn reduces the time each segment takes, so
            // the next smoothing iteration could potentially "see" more
            // segments around a given peak, and paradoxically cause the
            // planned velocity to be lower (if the window now includes a slow
            // spot that was previously farther away).
            v_avg_upper_bound = tcGetPlanMaxTargetVel(tc, emcmotConfig->biarc_solver_cfg.feed_override_allowance);
        }
        double v_avg_clamped = std::max(v_avg_upper_bound, TP_VEL_EPSILON);
        path_data.v_smooth.push_back(v_f_prev);
        double dt = ds / v_avg_clamped;
        double t_prev = path_data.t.back();
        bool ignore_smoothing = tc->canon_motion_type == EMC_MOTION_TYPE_TRAVERSE || tc->synchronized;
        path_data.ignore.push_back(ignore_smoothing);
        path_data.t.push_back(t_prev + dt);
        path_data.ds.push_back(ds);
#ifdef MOTION_PLANNING_DEBUG
        path_data.s.push_back(ds + path_data.s.back());
        path_data.motion_line.push_back(prev1_tc->tag.fields[GM_FIELD_LINE_NUMBER]);
        path_data.unique_id.push_back(prev1_tc->unique_id);
        path_data.planning_state.push_back(tcGetOptimizationState(prev1_tc));
        path_data.smoothed.push_back(prev1_tc->internal.smoothed);
        path_data.touched.push_back(false);
#endif

        // Don't need to continue optimizing past segments that have already been smoothed provided we leave enough horizon past any possible peaks
        if (prev1_tc->internal.smoothed) {
            if (t_smoothed == 0.0) {
                t_smoothed = t_prev;
            }
            // One back from the most recently smoothed
            if (t_prev > (t_smoothed + t_window*1.02)) {
                break;
            }
        }
    }

    {
    // KLUDGE shove a tail end on to provide a dummy value that will never be used?
        path_data.v_smooth.push_back(0);
        path_data.ignore.push_back(true);
        // KLUDGE not sure if necessary but just in case
        path_data.t.push_back(path_data.t.back()+10.0*t_window);
        path_data.ds.push_back(0);
#ifdef MOTION_PLANNING_DEBUG
        path_data.s.push_back(path_data.s.back());
        path_data.motion_line.push_back(-1);
        path_data.unique_id.push_back(-1);
        path_data.planning_state.push_back(TC_PLAN_UNTOUCHED);
        path_data.smoothed.push_back(false);
        path_data.touched.push_back(false);
#endif
    }


    // Finally, initialize the smoothed velocity data for subsequent passes
#ifdef MOTION_PLANNING_DEBUG
    path_data.v_opt = path_data.v_smooth;
    path_data.limiting_id = std::vector<int>(path_data.v_smooth.size(), 0);
#endif
    path_data.t_orig = path_data.t;
    return path_data;
}

#ifdef MOTION_PLANNING_DEBUG
static int call_id = 0;
#endif

/**
 * Do "rising tide" optimization to find allowable final velocities for each queued segment.
 * Walk along the queue from the back to the front. Based on the "current"
 * segment's final velocity, calculate the previous segment's maximum allowable
 * final velocity. The depth we walk along the queue is controlled by the
 * TP_LOOKAHEAD_DEPTH constant for now. The process safetly aborts early due to
 * a short queue or other conflicts.
 */

double findSmoothingSaturationTime(TC_QUEUE_STRUCT const *queue, SmoothingData const &path_data)
{
    SmoothingVector const &t_search = path_data.t_orig;
    // Assumes that all older segments are saturated if we walk backwards from the end and see a saturated one
    for (size_t k = 0; k < t_search.size()-1; ++k) {
        TC_STRUCT const *tc = tcqBack(queue, -k);
        if (!tc) {
            break;
        }
        if (tc->internal.found_final_vel_limit) {
            return t_search.at(k);
        }
    }
    return t_search.back();
}

// How much we're allowed to reduce the final velocity of a given segment below its previous amount (which breaks TP assumptions about what final velocity it can reach).

// If the TP can handle a sudden drop in final velocity, then we can be less fussy about this.
// For example, if the TP is executing a segment expecting a final velocity of
// 2.0, and we suddenly drop that to 1.0, it should do its best to reach it,
// but will probably arrive at the end of the segment at a higher velocity. As
// long as this is below the maximum theoretical final velocity of this
// segment, it will have the necessary space to continue planning (and won't
// overrun off the end).

// DO THIS ^^^^ it will make life a lot easier because then we can use the
// current optimizer to find the absolute max final velocity, and then do
// smoothing to find the target that may be slower, then we don't care what we
// set that final velocity to in between because the TP will deal with it.
static const constexpr double V_FINAL_REDUCTION_TOLERANCE_ABS = 1e-6;

tp_err_t applyLimitingVelocities(TC_QUEUE_STRUCT *queue, SmoothingData const &path_data, double t_window)
{
    SmoothingVector const &v_smooth = path_data.v_smooth;
    size_t search_depth = getOptimizationSearchEnd(path_data);

    double time_at_saturation = findSmoothingSaturationTime(queue, path_data);
    for (size_t k = 1; k < search_depth; ++k) {
        TC_STRUCT const *tc = tcqBack(queue, -k + 1);
        TC_STRUCT *prev1_tc = tcqBack(queue, -k);
        if ( !prev1_tc || !tc) {
            break;
        }
        if (!prev1_tc->internal.smoothed) {
            double new_v_final_prev = v_smooth.at(k);
            double v_final_prev = tcGetFinalVel(prev1_tc);
            double v_final_prev_limit = tcGetFinalVelLimit(prev1_tc);
            double v_final_this =  tcGetFinalVel(tc);

#ifdef MOTION_PLANNING_DEBUG
            //What matters here is what's actually being applies
            double v_reachable = tpComputeOptimalVelocity(tc, prev1_tc, v_final_this, k);

            // Don't allow a new velocity that's unreachable, so clamp whatever we plan to fit
            double v_new_restricted = std::min(v_reachable, new_v_final_prev);

            if ((v_final_prev-V_FINAL_REDUCTION_TOLERANCE_ABS) > v_new_restricted) {
                mp_debug_print("Motion planning warning: {warning_msg: \"smoothed / reachable final velocity has decreased\", idx: %ld, prev_final_vel: %0.17g, prev_smooth_vel: %0.17g, this_final_vel: %0.17g, this_smooth_vel: %0.17g, unique_id: %lld, motion_line: %d, distance: %0.17g, size: %ld,}",
                                 k,
                                 v_final_prev,
                                 v_smooth.at(k),
                                 v_final_this,
                                 v_smooth.at(k-1),
                                 tc->unique_id,
                                 tc->tag.fields[GM_FIELD_LINE_NUMBER],
                                 tc->target,
                                 v_smooth.size());
            }
#endif
            if (new_v_final_prev > v_final_prev_limit) {
                emcOperatorError("Motion planning assumptions violated: {error_msg: \"smoothed final velocity %0.17g exceeds limit %0.17g\", idx: %ld, prev_final_vel: %0.17g, prev_smooth_vel: %0.17g, this_final_vel: %0.17g, this_smooth_vel: %0.17g, unique_id: %lld, motion_line: %d, distance: %0.17g, size: %ld,}",
                                 new_v_final_prev,
                                 v_final_prev_limit,
                                 k,
                                 v_final_prev,
                                 v_smooth.at(k),
                                 v_final_this,
                                 v_smooth.at(k-1),
                                 tc->unique_id,
                                 tc->tag.fields[GM_FIELD_LINE_NUMBER],
                                 tc->target,
                                 v_smooth.size());
                return TP_ERR_FAIL;
            }

            // KLUDGE shrink our planned maximum velocities by a small fraction in case of numerical noise (leaves a bit of buffer to tweak them up and down)
            tcSetFinalVel(prev1_tc, new_v_final_prev);
            // When we flush the queue (e.g. at program end), all segment velocities are final
            // Make sure to compare against the original velocity / time values to ensure that all reachable values are stable
            if ((path_data.t_orig.at(k) > (time_at_saturation + t_window))) {
                prev1_tc->internal.smoothed = true;
            }
        }
    }
    return TP_ERR_OK;
}

tp_err_t tpOptimizePlannedMotions(TP_STRUCT * const tp, int optimization_depth) {
    int smoothing_passes = std::max(emcmotConfig->arc_blend_cfg.smoothing_passes, 0);
    double t_window = 1.0 / std::max(emcmotConfig->arc_blend_cfg.ramp_frequency, TP_TIME_EPSILON);

    mp_debug_print("motion planning: running optimization pass %d\n", call_id);

    // Pointers to the "current", previous, and 2nd previous trajectory
    // components. Current in this context means the segment being optimized,
    // NOT the currently excecuting segment.
    SmoothingData path_data = computeLimitingVelocities(&tp->queue, optimization_depth, t_window);

    tp_err_t res_smoothing = TP_ERR_OK;
    // 2) Find any small local velocity peaks and flatten them
//    mp_debug_print("Got t_window %f\n", t_window);
    for (int j = 0; j < smoothing_passes; ++j) {
        // Start with a small window and work up as a crude way to avoid over-smoothing with large windows
        // This is not ideal and is rather brute-force
        double t_window_current = (j+1)/std::max(smoothing_passes, 1) * t_window;
        res_smoothing = tpDoPeakSmoothing(path_data, optimization_depth, t_window_current);
        if (TP_ERR_OK != res_smoothing) {
            mp_debug_print("Peak smoothing failed after %d iterations\n", j);
            return res_smoothing;
        }
    }

#ifdef MOTION_PLANNING_DEBUG
//    mp_debug_print("ID, k, line, uid, v_opt             , v_smooth             , s                    , t\n");
    for (size_t k=0; k< path_data.v_opt.size(); ++k) {
        mp_debug_print("%d,%lu,%d,%d,%0.17g,%0.17g,%0.17g,%0.17g,%d,%d,%c,%c\n",
                call_id,
                k,
                path_data.motion_line[k],
                path_data.unique_id[k],
                path_data.v_opt[k],
                path_data.v_smooth[k],
                path_data.s[k],
                path_data.t[k],
                path_data.planning_state[k],
                path_data.limiting_id[k],
                path_data.smoothed[k] ? 'y' : 'n',
                path_data.touched[k] ? 'y' : 'n'
                );
    }
    call_id++;
#endif
    if (TP_ERR_OK != res_smoothing) {
        return res_smoothing;
    }

    CHP(applyLimitingVelocities(&tp->queue, path_data, 1.01*t_window));
    return TP_ERR_OK;
}


tp_err_t tpCheckBlendPrerequisites(
    TP_STRUCT const * const tp,
    TC_STRUCT * const prev_tc,
    TC_STRUCT * const tc,
    const char **result) {
    if (!tc || !prev_tc) {
        *result ="missing tc or prev tc in tangent check";
         return TP_ERR_FAIL;
    }
    if (emcmotConfig->arc_blend_cfg.optimization_depth < 2) {
        *result ="Optimization depth too low for tangent optimization";
         return TP_ERR_FAIL;
    }

    if (prev_tc->blend_mode.mode == TC_TERM_COND_STOP) {
        *result ="Found exact stop condition";
         return TP_ERR_FAIL;
    }

    if (prev_tc->indexrotary != INDEX_NONE || tc->indexrotary != INDEX_NONE) {
        *result ="rotary axis move requires indexing";
         return TP_ERR_FAIL;
    }

    // NOTE: this check is deprecated since the TP is now explicitly prevented from executing un-finalized segments
    if (prev_tc->progress > prev_tc->target / 2.0) {
        *result ="prev_tc progress is too large, aborting blend arc";
         return TP_ERR_FAIL;
    }
    *result ="blendable";
    return TP_ERR_OK;
}

/**
 * Check for tangency between the current segment and previous segment.
 * If the current and previous segment are tangent, then flag the previous
 * segment as tangent, and limit the current segment's velocity by the sampling
 * rate.
 */
TCIntersectType tpSetupTangent(TP_STRUCT const * const tp,
        TC_STRUCT * const prev_tc, TC_STRUCT * const tc) {
    tp_debug_json5_log_start(tpSetupTangent);
    if (!tc || !prev_tc) {
        tp_debug_json5_log_end("missing tc or prev tc in tangent check");
        return TC_INTERSECT_INCOMPATIBLE;
    }

#ifdef TP_DEBUG
    print_json5_tc_id_data_(tc);
#endif

    const char *result="";
    if (TP_ERR_OK != tpCheckBlendPrerequisites(tp, prev_tc, tc, &result))
    {
        tp_debug_json5_string(result);
        tp_debug_json5_log_end("unable to blend");
        return TC_INTERSECT_INCOMPATIBLE;
    }

    PmVector prev_tan, this_tan;
    if (
        TP_ERR_OK != tcGetEndTangentUnitVector(prev_tc, &prev_tan) ||
        TP_ERR_OK != tcGetStartTangentUnitVector(tc, &this_tan))
    {
        tp_debug_json5_log_end("missing tangents");
        return TC_INTERSECT_INCOMPATIBLE;
    }

    tp_debug_json5_PmVector(prev_tan);
    tp_debug_json5_PmVector(this_tan);

    // Assume small angle approximation here
    const double SHARP_CORNER_DEG = 2.0;
    const double SHARP_CORNER_EPSILON = pmSq(PM_PI * ( SHARP_CORNER_DEG / 180.0));

    if (VecVecUnitAntiParallel(&prev_tan, &this_tan, SHARP_CORNER_EPSILON))
    {
        tp_debug_json5_log_end("Found sharp corner");
        return TC_INTERSECT_INCOMPATIBLE;
    }

    // Calculate instantaneous acceleration required for change in direction
    // from v1 to v2, assuming constant speed
    double v_max1 = tcGetPlanMaxTargetVel(prev_tc, emcmotConfig->maxFeedScale);
    double v_max2 = tcGetPlanMaxTargetVel(tc, emcmotConfig->maxFeedScale);
    // Note that this is a minimum since the velocity at the intersection must
    // be the slower of the two segments not to violate constraints.
    double v_max_tangent = fmin(v_max1, v_max2);
    tp_debug_json5_double(v_max_tangent);

    // Account for acceleration past final velocity during a split cycle
    // (e.g. next segment starts accelerating again so the average velocity is higher at the end of the split cycle)
    double a_inst = v_max_tangent / tp->cycleTime + tc->maxaccel;

    // Set up worst-case final velocity
    // Compute the actual magnitude of acceleration required given the tangent directions
    // Do this by assuming that we decelerate to a stop on the previous segment,
    // and simultaneously accelerate up to the maximum speed on the next one.
    PmVector acc1 = VecScalMult(prev_tan, a_inst);
    PmVector acc2 = VecScalMult(this_tan, a_inst);
    PmVector acc_diff = VecVecSub(acc2, &acc1);

    tp_debug_json5_PmVector(acc1);
    tp_debug_json5_PmVector(acc2);
    tp_debug_json5_PmVector(acc_diff);

    PmVector acc_bound = getPlanningAccelBounds(prev_tc, tc);

    double acc_scale_max = findAbsMaxScale(acc_diff, &acc_bound);

    if (prev_tc->motion_type == TC_CIRCULAR || tc->motion_type == TC_CIRCULAR) {
        acc_scale_max /= BLEND_ACC_RATIO_TANGENTIAL;
    }
    tp_debug_json5_double(acc_scale_max);

    // Controls the tradeoff between reduction of final velocity, and reduction of allowed segment acceleration
    
    const double kink_ratio = tpGetTangentKinkRatio();

    tp_debug_json5_double(kink_ratio);

    if (acc_scale_max < kink_ratio) {
        tp_debug_json5_log_end("Segments considered tangent with acceleration scale %g within %g, kink_vel %g", acc_scale_max, kink_ratio, v_max_tangent);
        tcSetTermCond(prev_tc, TC_TERM_COND_TANGENT);
        tcSetKinkProperties(prev_tc, tc, v_max_tangent, acc_scale_max);
        return TC_INTERSECT_TANGENT;
    } else {
        switch (prev_tc->blend_mode.mode) {
        case TC_TERM_COND_STOP:
        case TC_TERM_COND_EXACT:
            // Shouldn't happen normally since we detect sharpness
            tp_debug_json5_log_end("Corner too sharp for exact-path, forcing exact stop");
            return TC_INTERSECT_INCOMPATIBLE;
        case TC_TERM_COND_PARABOLIC:
        case TC_TERM_COND_TANGENT:
            break;
        }
    }
    if (prev_tc->blend_mode.mode == TC_TERM_COND_EXACT) {
        tp_debug_json5_log_end("Found exact path condition with non-tangent intersection");
        return  TC_INTERSECT_INCOMPATIBLE;
    }

    tcSetKinkProperties(prev_tc, tc, v_max_tangent * kink_ratio / acc_scale_max, kink_ratio);

#ifdef TP_DEBUG
    print_json5_double_field(prev_tc, kink_vel);
#endif

    if (tcGetOptimizationState(tc) > TC_PLAN_UNTOUCHED && tcGetOptimizationState(prev_tc) > TC_PLAN_UNTOUCHED) {
        tp_debug_json5_log_end("Can't create blend when segment lengths are finalized");
        return TC_INTERSECT_INCOMPATIBLE;
    }

    tp_debug_json5_log_end("Nontangent intersection needs blend, acceleration scale is %g greater than cutoff %g, kink_vel is %g", acc_scale_max, kink_ratio, prev_tc->kink_vel);

    // NOTE: acceleration will be reduced later if tangent blend is used
    return TC_INTERSECT_NONTANGENT;
}

/**
 * Handle creating a blend arc when a new line segment is about to enter the queue.
 * This function handles the checks, setup, and calculations for creating a new
 * blend arc. Essentially all of the blend arc functions are called through
 * here to isolate the process.
 */
int tpSetupSegmentBlend(
    TP_STRUCT * const tp,
    TC_STRUCT * const prev2_tc,
    TC_STRUCT * const prev_tc,
    TC_STRUCT * const tc)
{
    const char *result;
    if (TP_ERR_OK != tpCheckBlendPrerequisites(tp, prev_tc, tc, &result)) {
        // Can't do any blending on these segments due to prerequisite
        tcSetTermCond(prev_tc, TC_TERM_COND_STOP);
    } else {
        // NOTE this is a no-op if various conditions aren't met (both segments are lines, etc.)
        applyProfileCorrection(tp, prev2_tc, prev_tc, tc);
    }

    // First phase, figure out what blend
    // Check the intersection type and handle tangency if the two segments are very nearly tangent
    TCIntersectType res_intersect = tpSetupTangent(tp, prev2_tc, prev_tc);
    switch (res_intersect) {
    case TC_INTERSECT_TANGENT:
        // All done, segments are tangent so no further analysis is needed
        break;
    case TC_INTERSECT_NONTANGENT:
    {
        TC_STRUCT prev_tc_copy = *prev_tc;
        // Workaround for "consuming" prev2_tc, start by popping prev_tc so it can be re-added later
        tcqPopBack(&tp->queue); //pop prev_tc
        int res = tpCreateBiarcBlend(tp, prev2_tc, &prev_tc_copy);
        if (res == TP_ERR_OK) {
            // Need to fully copy out the previous segment since it will get overwritten by the incoming blend segments
            // This better not fail or we're in trouble
            CHF(tpAddSegmentToQueue(tp, &blend_tc1));
            CHF(tpAddSegmentToQueue(tp, &blend_tc2));
        } else if (res == TP_ERR_UNRECOVERABLE){
            return TP_ERR_UNRECOVERABLE;
        } else {
            // Blend arc creation failed, but we can get some blending by using the kink properties from the tangent check
            tpFallbackToTangent(prev2_tc);
        }
        CHF(tpAddSegmentToQueue(tp, &prev_tc_copy));
        break;
    }
    case TC_INTERSECT_INCOMPATIBLE:
        // Force exact-stop if we can't do any blending
        tcSetTermCond(prev2_tc, TC_TERM_COND_STOP);
        break;
    }

    return TP_ERR_OK;

}

/**
 * Add a straight line to the tc queue.
 * end of the previous move to the new end specified here at the
 * currently-active accel and vel settings from the tp struct.
 */
int tpAddLine(
    TP_STRUCT * const tp,
    EmcPose end_p,
    EMCMotionTypes canon_motion_type,
    double vel,
    double ini_maxvel,
    double acc,
    probe_mode_t probe_mode,
    IndexRotaryAxis indexrotary,
    const state_tag_t & tag)
{
    PmVector end;
    emcPoseToPmVector(&end_p, &end);
    if (tpErrorCheck(tp) < 0) {
        return TP_ERR_FAIL;
    }
    // Initialize new tc struct for the line segment
    TC_STRUCT tc = {};
    tc.probe = probe_mode;

    clearBlendStructs();

    tcSetId(tp, &tc, tag);

    tcInit(&tc,
            TC_LINEAR,
            canon_motion_type,
            tp->cycleTime,
            tp->planner.enables_new);

    tc.needs_spindle_atspeed = tpCheckNeedsAtSpeed(&tp->planner, canon_motion_type);
    tc.needs_probe_ready = tpCheckNeedsProbeReady(&tp->planner);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);

    // Setup line geometry
    computeLineLengthAndTarget(&tc, &tp->goalPos, &end);
    // Cache nominal geometry for profile correction
    tc.indexrotary = indexrotary;

#ifdef TP_DEBUG
    {
        // macros use the variable name, need a plain name to please the JSON5 parser
        print_json5_log_start(tpAddLine);
        print_json5_tc_id_data_(&tc);
        print_json5_PmVector_("start", tp->goalPos);
        print_json5_PmVector(end);
        print_json5_double(vel);
        print_json5_double(ini_maxvel);
        print_json5_double(acc);
        print_json5_double_("nominal_length", tc.nominal_length);
        print_json5_unsigned_("enables", tp->planner.enables_new);
        print_json5_long(indexrotary);
        print_json5_bool_("needs_atspeed", tc.needs_spindle_atspeed);
        print_json5_bool_("needs_probe_ready", tc.needs_probe_ready);
        print_json5_long(canon_motion_type);
        PmVector delta = VecVecSub(end, &tp->goalPos);
        print_json5_PmVector(delta);
        print_json5_PmLine9_("line", &tc.coords.line);
        print_json5_log_end();
    }
#endif

    if (tc.target < TP_POS_EPSILON) {
        emcOperatorError("failed to create line id %d, zero-length segment\n",tp->planner.nextId);
        return TP_ERR_ZERO_LENGTH;
    } else {
        int res_add = tpFinalizeAndEnqueue(tp, &tc, NULL);
        if (TP_ERR_OK != res_add) {
            tpRestoreWaitConditions(&tp->planner, &tc);
        }
        return res_add;
    }
}


/**
 * Adds a circular (circle, arc, helix) move from the end of the
 * last move to this new position.
 *
 * @param end is the xyz/abc point of the destination.
 *
 * see pmCircleInit for further details on how arcs are specified. Note that
 * degenerate arcs/circles are not allowed. We are guaranteed to have a move in
 * xyz so the target is always the circle/arc/helical length.
 */
int tpAddCircle(TP_STRUCT * const tp,
    EmcPose end_p,
    PmCartesian center,
    PmCartesian normal,
    int turn,
    double expected_angle_rad,
    EMCMotionTypes canon_motion_type,
    double vel,
    double ini_maxvel,
    double acc,
    double acc_normal,
    const state_tag_t & tag)
{
    if (tpErrorCheck(tp)<0) {
        return TP_ERR_FAIL;
    }
    PmVector end;
    emcPoseToPmVector(&end_p, &end);

    TC_STRUCT tc = {};

    clearBlendStructs();

    tcSetId(tp, &tc, tag);

    tcInit(&tc,
            TC_CIRCULAR,
            canon_motion_type,
            tp->cycleTime,
            tp->planner.enables_new);

    tc.needs_spindle_atspeed = tpCheckNeedsAtSpeed(&tp->planner, canon_motion_type);
    tc.needs_probe_ready = tpCheckNeedsProbeReady(&tp->planner);

    // Setup any synced IO for this move
    tpSetupSyncedIO(tp, &tc);

    // Copy over state data from the trajectory planner
    tcSetupState(&tc, tp);

    // Setup circle geometry
    int res_init = pmCircle9Init(&tc.coords.circle,
            &tp->goalPos,
            &end,
            &center,
            &normal,
            turn,
            expected_angle_rad);

    if (res_init) return res_init;

    tc.acc_normal_max = acc_normal;

    // Copy in motion parameters
    tcSetupMotion(&tc,
            vel,
            ini_maxvel,
            acc);

    pmCircle9LengthAndRatios(&tc.coords.circle);
    // Update tc target with existing circular segment
    tc.target = tc.coords.circle.total_length;
    tc.nominal_length = tc.target;

#ifdef TP_DEBUG
    {
        // macros use the variable name, need a plain name to please the JSON5 parser
        print_json5_log_start(tpAddCircle);
        print_json5_tc_id_data_(&tc);
        print_json5_PmVector_("start", tp->goalPos);
        print_json5_PmVector(end);
        print_json5_PmCartesian(center);
        print_json5_PmCartesian(normal);
        print_json5_long(turn);
        print_json5_double(expected_angle_rad);
        print_json5_double(vel);
        print_json5_double(ini_maxvel);
        print_json5_double(acc);
        print_json5_double(acc_normal);
        print_json5_unsigned_("enables", tp->planner.enables_new);
        print_json5_bool_("needs_atspeed", tc.needs_spindle_atspeed);
        print_json5_bool_("needs_probe_ready", tc.needs_probe_ready);
        print_json5_long(canon_motion_type);
        PmVector delta = VecVecSub(end, &tp->goalPos);
        print_json5_PmVector(delta);
        print_json5_PmCircle_("xyz_circle", tc.coords.circle.xyz);
        print_json5_log_end();
    }
#endif

    if (tc.target < TP_POS_EPSILON) {
        return TP_ERR_ZERO_LENGTH;
    } else {
        int res_add = tpFinalizeAndEnqueue(tp, &tc, NULL);
        if (TP_ERR_OK != res_add) {
            // Restore the at-speed flag if needed
            tpRestoreWaitConditions(&tp->planner, &tc);
        }
        return res_add;
    }
}

int tpSetSpindleSync(TP_STRUCT * const tp, double sync, int velocity_mode) {
    // WARNING assumes positive sync
    if(sync > 0) {
        if (velocity_mode) {
            tp->planner.synchronized = TC_SYNC_VELOCITY;
        } else {
            tp->planner.synchronized = TC_SYNC_POSITION;
        }
        tp->planner.uu_per_rev = sync;
    } else {
        tp->planner.synchronized = TC_SYNC_NONE;
        tp->planner.uu_per_rev = sync;
    }

    return TP_ERR_OK;
}

int tpGetMotionType(TP_STRUCT * const tp)
{
    return tp->exec.motionType;
}

EmcPose getMotionPlanningGoalPos(TP_STRUCT const  * const tp)
{
    EmcPose out={};
    if (tp) {
        pmVectorToEmcPose(&tp->goalPos, &out);
    }

    return out;
}

EmcPose tpGetCurrentPos(const TP_STRUCT * const tp)
{
    EmcPose out={};
    if (tp) {
        pmVectorToEmcPose(&tp->exec.currentPos, &out);
    }

    return out;
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

