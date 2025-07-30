/********************************************************************
* Description: blendmath.c
*   Circular arc blend math functions
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*   
* Copyright (c) 2014 All rights reserved.
*
* Last change:
********************************************************************/

#ifdef TP_LOG_PARSING
#undef TP_DEBUG
#endif

#include "posemath.h"
#include "tc_types.h"
#include "tc.hh"
#include "tp_types.h"
#include "motion_types.h"
#include "rtapi_math.h"
#include "spherical_arc9.hh"
#include "blendmath.hh"
#include "tp_debug.h"
#include "tp_enums.h"
#include "stdlib.h"
#include "math_util.h"
#include "emcpose.h"
#include "tp_json5_print.h"

#include "tp_call_wrappers.hh"

static const double NEAR_INFINITY=1e300;

/**
 * @section geomfuncs Geometry check functions
 */

/**
 * Somewhat redundant function to calculate the segment intersection angle.
 * The intersection angle is half of the supplement of the "divergence" angle
 * between unit vectors. If two unit vectors are pointing in the same
 * direction, then the intersection angle is PI/2. This is based on the
 * simple_tp formulation for tolerances.
 */
double findIntersectionAngle(PmVector const * const u1,
        PmVector const * const u2)
{
    double dot = LIMIT(VecVecDot(u1, u2), 1.0);

    return acos(-dot)/2.0;
}

/**
 * Given a vector of "bounds" and a direction vector, normalize the direction vector components in terms of the bound values.
 * In effect, this measures how much of the total allowed range is used in each axis of the direction vector.
 */
PmVector normalizeToBounds(PmVector vec,
        PmVector const *bounds)
{
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        double b = bounds->ax[i];
        vec.ax[i] = b ? vec.ax[i] / b : 0.0;
    }

    return vec;
}

double findAbsMaxScale(PmVector vec, const PmVector * bounds)
{
    vec = normalizeToBounds(vec, bounds);
    return VecAbsMax(&vec);
}

// KLUDGE this is a messy workaround for named fields in PmVector
static inline void minBoundRatioIfNonzero(double s, double b, double *m2)
{
    if (s > 0.0 && b > 0.0) {
        //Have to square b here since the scale is also squared
        *m2 = fmin(*m2, pmSq(b) / s);
    }
}

/**
 * Finds the maximum allowed value of the bounded quantity on the given plane.
 *
 * @param plane_envelope_sq is the square of the maximal value of the ith component of a unit
 * vector rotated through the plane. This basically specifies the largest
 * possible contribution of each axis to a direction in the plane. We use squared values here
 * because they are easier to compute, and require only one square root at the end.
 * @param bounds Specifies the per-axis maximum value (e.g. acceleration, velocity)
 * @param max_planar_value maximum length of a vector in the plane that is always within limits
 * @return -1 on error, 0 if completed successfully
 */
int findMaxValueOnPlane(PmVector const * plane_envelope_sq,
        PmVector const * bounds,
        double * max_planar_value)
{
    if (!plane_envelope_sq || !bounds || !max_planar_value) {
        return -1;
    }
    double m2 = NEAR_INFINITY;

    for (int i = 0; i < PM_VECTOR_SIZE; ++i) {
        minBoundRatioIfNonzero(plane_envelope_sq->ax[i], bounds->ax[i], &m2);
    }

    // One square root at the end to get the actual max accel
    *max_planar_value = pmSqrt(m2);
    return 0;
}

/** Find real roots of a quadratic equation in standard form. */
int quadraticFormula(double A, double B, double C, double * const root0,
        double * const root1)
{
    if (A < 0.0) {
        A *=-1;
        B *=-1;
        C *=-1;
    }
    double disc = pmSq(B) - 4.0 * A * C;
    if (disc < 0) {
        tp_debug_print("discriminant %.12g < 0, A=%.12g, B=%.12g,C=%.12g\n", disc, A, B, C);
        return TP_ERR_FAIL;
    }
    double t1 = pmSqrt(disc);
    if (root0) {
        *root0 = ( -B + t1) / (2.0 * A);
    }
    if (root1) {
        *root1 = ( -B - t1) / (2.0 * A);
    }
    return TP_ERR_OK;
}

/**
 * @section blending blend math functions
 */

double findMaxByAltitude(
    PmVector const * const u1,
    PmVector const * const u2,
    double max1,
    double max2,
    double included_angle)
{
    if (fabs(included_angle) < TP_ANGLE_EPSILON) {
        return fmin(max1, max2);
    }
    // Clip the angle at a reasonable value (less than 90 deg), to prevent div by zero
    double phi_effective = fmin(included_angle, PM_PI * 0.49);
    //tp_debug_json5_double(phi_effective);

    // Copy over maximum values, restricted to place altitude within the base of the triangle
    double Cp = cos(phi_effective);
    double max_effective1 = fmin(max1, max2 / Cp);
    double max_effective2 = fmin(max2, max1 / Cp);
    //tp_debug_json5_double(max_effective1);
    //tp_debug_json5_double(max_effective2);

    // Get "altitude"
    double max_area = max_effective1 * max_effective2 / 2.0 * sin(phi_effective);
    //tp_debug_json5_double(max_area);

    // Get "base" of triangle
    PmVector tmp1 = VecScalMult(*u1, max_effective1);
    PmVector diff = VecVecSub(VecScalMult(*u2, max_effective2), &tmp1);
    double base = VecMag(&diff);
    //tp_debug_print("v_base = %f\n", base);

    return 2.0 * max_area / base;
}

static inline double normalAccelFromMax(double a_max)
{
    return a_max * BLEND_ACC_RATIO_NORMAL;
}

/**
 * Find basic parameters for a single arc wthin a biarc blend
 */
int find_blend_parameters(
    PmVector const * const u1,
    PmVector const * const u2,
    PmVector const * const acc_bound,
    PmVector const * const vel_bound,
    BlendControls const *controls,
    double v_max1,
    double v_max2,
    BlendParameters * const param)
{
    param->arc_intersection_halfangle = findIntersectionAngle(
        u1,
        u2);
    double arc_included_angle = M_PI - 2.0 * param->arc_intersection_halfangle;

    param->v_max_altitude = findMaxByAltitude(
        u1,
        u2,
        v_max1,
        v_max2,
        arc_included_angle);

    int res_limits = find_blend_vel_accel_planar_limits(
        u1,
        u2,
        acc_bound,
        vel_bound,
        &param->a_max_planar,
        &param->v_max_planar);

    // Store max normal acceleration
    param->a_n_max = normalAccelFromMax(param->a_max_planar);

    double v_max = fmax(param->v_max_planar, param->v_max_altitude);
    param->v_plan = fmin(controls->v_goal, v_max);

    double R_goal_minimal = pmSq(controls->v_goal) / param->a_n_max;

    const double CT = cos(param->arc_intersection_halfangle);
    const double h_to_R = CT / fmax((1.-CT), TP_POS_EPSILON);
    double R_goal_correction = controls->correction_dist * h_to_R;
    param->R_goal = fmax(fmax(R_goal_minimal, controls->req_min_radius), R_goal_correction);

    return res_limits;
}

tp_err_t init_blend_segment_geometry(
    TC_STRUCT * const blend_tc,
    BlendPoints const *points)
{
    blend_tc->motion_type = points->motion_type;

    switch (points->motion_type) {
    case TC_LINEAR:
        pmLine9Init(&blend_tc->coords.line, &points->arc_start, &points->arc_end);
        break;
    case TC_SPHERICAL:
    {
        CHP(arc9InitFromPoints(
            &blend_tc->coords.arc,
            &points->arc_start,
            &points->arc_end,
            &points->arc_center,
            &points->u_tan));
    }
        break;
    default:
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

int init_blend_controls_from_segments(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    double override_allowance,
    BlendControls * const controls)
{
    // Copy over geometric maximum velocities
    controls->v_max_geom1 = prev_tc->maxvel_geom;
    controls->v_max_geom2 = tc->maxvel_geom;

    // Find the nominal velocity for the blend segment with no overrides
    double v_req_prev = tcGetPlanMaxTargetVel(prev_tc, 1.0);
    double v_req_this = tcGetPlanMaxTargetVel(tc, 1.0);
    controls->v_req = fmax(v_req_prev, v_req_this);

    // Find the worst-case velocity we should reach for either segment
    controls->v_goal = fmax(tcGetPlanMaxTargetVel(prev_tc, override_allowance),
            tcGetPlanMaxTargetVel(tc, override_allowance));

    // Find net tolerance that's actually usable (so that we're
    // not passing 0.0 around if tolerance is not specified)
    tcFindBlendTolerance(prev_tc, tc, &controls->path_tolerance);

    // Use same assumptions as tolerance (where G64 called before a motion line
    // should take effect with the next motion line (so it applies to the blend
    // with the previous segment)
    controls->req_min_radius = tc->blend_mode.req_min_radius;

    // For TP-based profile correction, we store the correction distance in the
    // current segment for consistency This is unlike the emccanon version
    // which had to store it in prev_tc (due to the way motions are queued w/
    // NCD in canon)
    controls->correction_dist = tc->correction_dist;

    if (controls->path_tolerance <= 0.0) {
        tp_debug_print("tolerance %f is too small to blend effectively\n", controls->path_tolerance);
        return TP_ERR_FAIL;
    }

    return 0;
}

/**
 * Check if the previous line segment will be consumed based on the blend arc parameters.
 * @pre assumes non-null arguments
 */
int blendCheckConsume(
    BlendParameters * const param,
    double L_prev)
{
    param->consume = L_prev <= TP_POS_EPSILON;
    return 0;
}

/** @section spiralfuncs Functions to approximate spiral arc length */

/**
 * Force a value to be within the range specified, with that range shrunk by the relative margin.
 * Example: If rel_margin is 0.1, upper = 2, lower = 1, value must be within [1.05,1.95]
 * @pre result is undefined if rel_margin exceeds [0,1], or lower > upper
 */
double clamp(double upper, double lower, double rel_margin, double value)
{
    double m = (upper - lower) * rel_margin / 2.0;
    double u = upper - m;
    double l = lower + m;
    return fmin(fmax(value, l), u);
}

static inline double find_prev_reachable_blend_position(TC_STRUCT const *prev_tc, double Rb)
{
    return fmax(prev_tc->target - Rb, 0);
}

static inline double find_next_reachable_blend_position(TC_STRUCT const *tc, double Rb)
{
    return fmin(fmin(Rb, tc->target), tc->nominal_length / 2.0);
}

/**
 * Populate the parameter structure based on the specified blend region size and the geometry of segments to be blended.
 *
 * The biarc-blend optimizer finds candidate end points / tangent directions
 * for a blend. The blend region is nominally a circle around the intersection
 * point (for a line-line blend), but really this is an approximation. "Rb" is
 * the distance along the path, away from the intersection point, where the
 * blend starts / ends.
 * @param Rb blend region size
 * @param prev_tc previous segment
 * @param tc next segment (assumed to have start point = prev_tc's end point)
 * @param out
 * @return
 */
tp_err_t find_blend_points_and_tangents(
    double Rb,
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    blend_boundary_t * const out)
{
    out->s1 = find_prev_reachable_blend_position(prev_tc, Rb);
    out->s2 = find_next_reachable_blend_position(tc, Rb);

    out->prev_len = prev_tc->target - out->s1;

    // For blend-sizing heuristics
    tcGetStartpoint(tc, &out->P);

    tcGetPosReal(prev_tc, out->s1, &out->P1);
    tcGetPosReal(tc, out->s2, &out->P2);

    tcGetTangentUnitVector(prev_tc, out->s1, &out->u1);
    tcGetTangentUnitVector(tc, out->s2, &out->u2);

    return TP_ERR_OK;
}

/**
 * Based on the requested blend size (from parameters), find the intermediate line segments that will be used to generate each of the arcs for the blend.
 *
 * Constructs 3 intermediate line segments as a scaffold for the bi-arc blend:
 * The first line is tangent to the previous motion, with length d1
 * The third line is tangent to the next motion, with length d2
 * The second line connectes the first and third lines, with length d1+d2
 *
 * These line segments are later used by the blend optimizer to construct two
 * circular arcs for the blend. Therefore, these intermediate segments don't
 * need to become full TP motion segments, we just need to know the basic
 * geometry (end points, lengths and unit vectors).
 *
 * @param blend_params specifies end points and tangent directions of the previous and next segments
 * @param control_pts output containing the endpoints and tangents of the intermediate segments
 * @return 0 if successful, -1 otherwise
 */
int find_blend_intermediate_segments(
    biarc_solver_config_t const *config,
    blend_boundary_t const * const blend_params,
    biarc_control_points_t * const control_pts,
    bool symmetrical)
{
    double k = symmetrical ? 1. : blend_params->s2 / blend_params->prev_len;
    if (k <= TP_POS_EPSILON) {
        return -1;
    }

    k = CLAMP(k, fmax(1./config->max_arc_size_ratio, TP_POS_EPSILON), config->max_arc_size_ratio);

    // compute u_bar and P_bar vectors used for intermediate segment solution
    // u_bar = u1+u2*k
    // P_bar = P2-P1
    PmVector du12 = VecScalMult(blend_params->u2, k);
    VecVecAddEq(&du12, &blend_params->u1);

    PmVector dP12 = VecVecSub(blend_params->P1, &blend_params->P2);

    // Find B / C first since A could be zero if u1 / u2 are colinear
    double B = 2. * VecVecDot(&dP12, &du12);

    double C =VecMagSq(&dP12);

    double A = VecMagSq(&du12) - pmSq(1.+k);

    // true iff u1 / u2 are nearly parallel or anti-parallel
    if (fabs(A) < 1e-12)
    {
        if (fabs(B) < TP_POS_EPSILON) {
            // u vectors are perpendicular to displacement vector between P1/P2
            double dot = VecVecDot(&blend_params->u1, &blend_params->u2);
            if (dot > 0) {
                // u1 and -u2 are in opposite directions, no biarc solution exists
                return -2;
            } else {
                // u1 and -u2 are in the same direction, assume "rectangular" solution
                control_pts->d1 = pmSqrt(C)/2.0;
            }
        } else {
            control_pts->d1 = -C/B;
        }
    } else {
        double other;
        int res_qf = quadraticFormula(A, B, C, &control_pts->d1, &other);
        if (res_qf || control_pts->d1 < 0) {
            return -1;
        }
    }

    control_pts->d2 = k*control_pts->d1;

    control_pts->Pb1 = VecScalMult(blend_params->u1, control_pts->d1);
    VecVecAddEq(&control_pts->Pb1, &blend_params->P1);

    control_pts->Pb2 = VecScalMult(blend_params->u2, -control_pts->d2);
    VecVecAddEq(&control_pts->Pb2, &blend_params->P2);

    if (0 != VecVecDirection(&control_pts->Pb2, &control_pts->Pb1, &control_pts->u_mid)) {
        // If unit vector is degenerate
        return -1;
    }
    control_pts->P_mid = VecScalMult(control_pts->u_mid, control_pts->d1);
    VecVecAddEq(&control_pts->P_mid, &control_pts->Pb1);

    if (control_pts->d1 > 0) {
        return 0;
    } else {
        return -3;
    }
}

/**
 * Calculate how much of a segment can be consumed for a blend based on internal limits (total length, nominal length, etc.).
 * The actual blend may be smaller than the max found here (e.g. due to tolerance, max velocity limits, etc.)
 */
static inline double find_max_blend_length(TC_STRUCT const * const tc)
{
    double usable_length = fmin(tc->target, tc->nominal_length / 2.0);

    if (tc->motion_type == TC_CIRCULAR) {
        
        double phi_max = PM_PI / 3.0;
        double total_arc_length = pmSqrt(pmSq(tc->coords.circle.xyz.height) + pmSq(tc->coords.circle.fit.total_planar_length));
        double l_from_angle = phi_max / tc->coords.circle.xyz.angle * total_arc_length;
        return fmin(l_from_angle, usable_length);
    } else {
        return usable_length;
    }
}

double find_max_blend_region(TC_STRUCT const * const prev_tc, TC_STRUCT const * const tc, bool symmetrical)
{
    double l_prev = find_max_blend_length(prev_tc);
    double l_this = find_max_blend_length(tc);
    // NOTE: this region can be larger than a segment's total length, so need to check for this when sampling)
    return symmetrical ? fmin(l_prev, l_this) : fmax(l_prev, l_this);
}

static void find_blend_size_internal(
    PmVector const *u1,
    PmVector const *u2,
    double d, //!< the length of a single leg of the pair of line segments bounding the arc
    double *R_geom,
    double *arc_len_est)
{
    double dot = VecVecDot(u1, u2);

    double cos_2theta = fmax(-dot, -1.0 + 1.0e-15);

    double sin_theta_approx = pmSqrt((1. - cos_2theta) / 2);
    double cos_theta_approx = pmSqrt((1. + cos_2theta) / 2);
    double tan_theta_approx = sin_theta_approx / fmax(cos_theta_approx, TP_ANGLE_EPSILON);

    
    // Find radius from the limiting length (assume length is actually symmetrical
    *R_geom = tan_theta_approx * d;
    *arc_len_est = 2.0 * d * sin_theta_approx;
}

tp_err_t find_blend_size_from_intermediates(
    blend_boundary_t const * const blend_boundary,
    biarc_control_points_t const * const intermediates,
    double *R_geom,
    double *arc_len_est)
{
    double R_geom1 = 0;
    double arc_len1 = 0;
    double R_geom2 = 0;
    double arc_len2 = 0;

    find_blend_size_internal(&blend_boundary->u1, &intermediates->u_mid, intermediates->d1, &R_geom1, &arc_len1);
    find_blend_size_internal(&intermediates->u_mid, &blend_boundary->u2, intermediates->d2, &R_geom2, &arc_len2);

    *R_geom = fmin(R_geom1, R_geom2);
    *arc_len_est = fmin(arc_len1, arc_len2);

    return TP_ERR_OK;
}

static const biarc_solution_t zero_region_result = {};

static inline double get_effective_range(blend_solver_constraints_t const * const constraints)
{
    return constraints->upper.Rb - constraints->lower.Rb;
}

static inline double guess_interpolated_region_size(
    blend_solver_constraints_t const * const constraints,
    double R_goal,
    double T_goal)
{
    double dRb = constraints->upper.Rb - constraints->lower.Rb;
    double dR_plan = constraints->upper.R_plan - constraints->lower.R_plan;
    double dR_goal = R_goal - constraints->lower.R_plan;
    double Rb_interp_R = constraints->lower.Rb + dRb * (dR_goal / fmax(dR_plan, TP_POS_EPSILON));

    double dT_plan = constraints->upper.T_plan - constraints->lower.T_plan;
    double dT_goal = T_goal - constraints->lower.T_plan;
    double Rb_interp_T = constraints->lower.Rb + dRb * (dT_goal / fmax(dT_plan, TP_POS_EPSILON));

    if (abs(constraints->bias_count) > 3 ) {
        return (constraints->upper.Rb + constraints->lower.Rb) / 2.;
    } else {
        return clamp(constraints->upper.Rb, constraints->lower.Rb, 0.1, fmin(Rb_interp_R, Rb_interp_T));
    }
}

/**
 * Update the solution guess based on solver conditions and constraints.
 *
 * The current solution is a lower bound iff:
 *  1) the planned radius is smaller than the goal radius
 *  2) deviation from the end point is less than the allowed tolerance
 *  3) effective speed of the previous segment is greater than the blend speed (due to normal acceleration limits)
 */
static inline BiarcSolverStatusType check_solution(
    biarc_solver_config_t const *config,
    double a_n_max,
    double R_goal,
    blend_solver_constraints_t *constraints,
    biarc_solution_t const *solution,
    double radius_rel_tol,
    double radius_abs_tol,
    int iterations)
{
    // We either need to fully consume the previous segment (so the effective endpoint becomes the midpoint of the blend), or otherwise leave enough there so that the minimum feed is similar to the blend.
    bool consume_prev = solution->boundary.s1 < TP_POS_EPSILON;

    double R_size_limit_equiv = pmSq(solution->err.v_size_limit*(1.+ config->velocity_rel_tolerance)) / a_n_max;

    if (solution->R_plan <= R_goal && solution->err.deviation_margin >= -config->deviation_abs_tolerance && (consume_prev || solution->R_plan <= R_size_limit_equiv)) {
        constraints->lower = *solution;
        constraints->bias_count = constraints->bias_count > 0 ? -1 : constraints->bias_count - 1;

        // Check if we're close to the goal "radius", i.e. we've met the blend target velocity limit
        if ((fabs(solution->err.radius_rel) < radius_rel_tol
             || fabs(solution->err.radius_abs) < radius_abs_tol))
        {
            return BIARC_REACHED_GOAL_SIZE;
        } else if (solution->err.radius_rel < 0.0 && solution->err.deviation_margin < config->deviation_abs_tolerance) {
            return BIARC_REACHED_TOLERANCE;
        } else if (get_effective_range(constraints) < config->convergence_tolerance) {
            return BIARC_CONVERGED;
        }
    } else {
        constraints->bias_count = constraints->bias_count < 0 ? 1 : constraints->bias_count + 1;
        constraints->upper = *solution;
        double R_bound = fmax(constraints->upper.R_plan, constraints->lower.R_plan);
        double v_bound = pmSqrt(R_bound * a_n_max);
        if (iterations > 1 && v_bound < config->velocity_cutoff) {
            return BIARC_MIN_LENGTH_LIMITED;
        }
    }

    return BIARC_NOT_SOLVED;
}

int find_blend_vel_accel_planar_limits(
    PmVector const * const u_tan1,
    PmVector const * const u_tan2,
    PmVector const * const acc_bound,
    PmVector const * const vel_bound,
    double *a_max_planar,
    double *v_max_planar)
{
    // Find an orthonormal basis and "envelope" vector for the plane that represents
    // the worst-case contribution of each axis to the overall velocity / acceleration
    PmVector u, v;
    switch(VecVecOrthonormal(u_tan1, u_tan2, &u, &v)) {
    case 0:
    {
        // Have full orthonormal basis, continue with planar calculation
        PmVector uv_plane_envelope_sq = VecVecHarmonicAddSq(u, &v);

        // Reuse the envelope vector (this is why accel and vel are calculated together)
        int res_dia = findMaxValueOnPlane(&uv_plane_envelope_sq, acc_bound, a_max_planar);
        res_dia |= findMaxValueOnPlane(&uv_plane_envelope_sq, vel_bound, v_max_planar);

        return res_dia;
    }
    case 1:
    {
        *v_max_planar = 1.0 / findAbsMaxScale(u, vel_bound);
        *a_max_planar = 1.0 / findAbsMaxScale(u, acc_bound);
        return 0;
    }
    }
    // Shouldn't get here but if we do it's an unknown failure
    return -1;
}

void find_biarc_solution_error(
    blend_boundary_t const *boundary, //!< Boundary of the blend solution corresponding to Rb
    double blend_tolerance,
    double R_goal,
    double R_plan,
    double T_plan,
    double cycle_time,
    biarc_solution_errors_t * const err)
{
    err->deviation_margin = blend_tolerance - T_plan; // Positive means solution is good, negative means it's violating the tolerance

    // Check for solution convergence based on specified tolerances
    err->radius_rel = (R_plan - R_goal + TP_POS_EPSILON) / (R_goal + TP_POS_EPSILON);
#ifdef BIARC_DEBUG
    print_json5_double(R_goal);
    print_json5_double(R_plan);
    print_json5_double(blend_tolerance);
    print_json5_double(T_plan);
    print_json5_double(err->radius_rel);
#endif
    err->radius_abs = R_plan - R_goal;

    err->v_size_limit = (boundary->s1) / (cycle_time * TP_MIN_SEGMENT_CYCLES);
}

static tp_err_t check_start_end_intersection(TC_STRUCT const *prev_tc, TC_STRUCT const *this_tc)
{
    if (!prev_tc || !this_tc) {
        return TP_ERR_FAIL;
    }
    PmVector endpt;
    tcGetEndpoint(prev_tc, &endpt);
    PmVector startpt;
    tcGetStartpoint(this_tc, &startpt);
    if (VecVecDisp(&endpt, &startpt) > TP_POS_EPSILON) {
        return TP_ERR_FAIL;
    }
    return TP_ERR_OK;
}

/**
 * Find the optimal Rb to get the smallest blend that can meet the feed rate requirements.
 * Choose initial Rb = Rb_max
 * Find blend start / end points and tangent vectors, within Rb of intersection P with procedure find_blend_points_and_tangents
 * Find blend segment lengths d, L and intermediate points with procedure find_blend_intermediate_segments
 * Calculate blend kinematics for the two blends.
 * Find blend radii and arc lengths given P1, P2, Pb1, Pb2
 * Compute max velocity on blend arcs from geometry
 * Check blend performance:
 * Verify distance from blend midpoint to original intersection P < tolerance
 * Check if requested velocity is reached (if not, need bigger arc)
 * Check if blend radii are larger than they need to be (e.g. worst-case normal acceleration < a_t_max)
 * Refine solution as necessary:
 * Guess a new Rb based on heuristic function of Rb, Rb_max, v_plan with procedure guess_blend_region_size
 * Stop after N iterations or until Rb converges
 */
tp_err_t optimize_biarc_blend_size(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    biarc_solver_config_t const * const config,
    PmVector const * const vel_bound,
    PmVector const * const acc_bound,
    biarc_solver_results_t * const biarc_results,
    BlendControls * const controls,
    double cycle_time)
{
    if (!prev_tc || !tc || !biarc_results) {
        return TP_ERR_MISSING_INPUT;
    }
    // For convenience
    biarc_solution_t *solution = &biarc_results->solution;
    CHP(init_blend_controls_from_segments(prev_tc, tc, config->feed_override_allowance, controls));

    bool force_symmetrical_search = (tc->correction_dist == 0) && controls->req_min_radius == 0;
    double Rb_init = find_max_blend_region(prev_tc, tc, force_symmetrical_search);

    // Initial solver values for Rb range using somewhat dumb (but cheap) values
    solution->Rb = biarc_results->constraints.upper.Rb = Rb_init;
    biarc_results->constraints.upper.R_plan = 0; // To be filled in at first pass through solver
    biarc_results->constraints.lower = zero_region_result;

    const unsigned MAX_ITERATIONS = config->max_iterations;

    CHP(check_start_end_intersection(prev_tc, tc));

    CHP(find_blend_points_and_tangents(solution->Rb, prev_tc, tc, &solution->boundary));

    // Intersection is at start of next segment, so use startpoint method since it's much faster
    PmVector P;
    CHP(tcGetStartpoint(tc, &P));

    for (biarc_results->iterations = 1; biarc_results->iterations < MAX_ITERATIONS; ++biarc_results->iterations)
    {
        // Find blend segment lengths d, L and intermediate points with procedure find_blend_intermediate_segments
        if (TP_ERR_OK != find_blend_intermediate_segments(config, &solution->boundary, &solution->control_pts, force_symmetrical_search))
        {
            biarc_results->result = BIARC_DEGENERATE_SEGMENTS;
            return TP_ERR_FAIL;
        }

        // Next, find the velocity / acceleration limits at the intermediate segment
        double v_max_mid = 1. / findAbsMaxScale(solution->control_pts.u_mid, vel_bound);
        //double a_n_max_mid = 1. / findAbsMaxScale(solution->control_pts.u_mid, acc_bound) * BLEND_ACC_RATIO_NORMAL;

        // Find blend parameters for each arc in the biarc blend:
        CHP(find_blend_parameters(
            &solution->boundary.u1,
            &solution->control_pts.u_mid,
            acc_bound,
            vel_bound,
            controls,
            controls->v_max_geom1,
            v_max_mid,
            &solution->param1
            ));
        blendCheckConsume(&solution->param1, solution->boundary.s1);

        CHP(find_blend_parameters(
            &solution->control_pts.u_mid,
            &solution->boundary.u2,
            acc_bound,
            vel_bound,
            controls,
            v_max_mid,
            controls->v_max_geom2,
            &solution->param2
            ));

        // Plan for the higest reachable velocity, and lowest limiting normal acceleration (bias the solver towards slightly larger blends)
        double v_plan_guess = fmax(solution->param1.v_plan, solution->param2.v_plan);
        double a_n_max = fmin(solution->param1.a_n_max, solution->param2.a_n_max);
        double R_goal = fmax(solution->param1.R_goal, solution->param2.R_goal);

        // Scale up solver tolerance to have the same net effect on velocity
        // Because radius is proportional to the square of velocity, we need to rescale it.
        //
        // (1) (v_limit - v_plan) / v_plan = v_rel_tol
        // (2) (R_limit - R_plan) / R_plan = radius_rel_tol
        // (3) R = v^2 / a_n
        // Solve for radius_rel_tol by substituting (3) into (2) to convert to velocity terms, then solve (1) for v_limit and also substitute in.
        // Result is radius_rel_tol = v_rel_tol^2 + 2*v_rel_tol;
        //
        // For absolute tolerance:
        // (4) (v_limit - v_plan)  = v_abs_tol
        // (5) (R_limit - R_plan) = radius_abs_tol
        // Substitute (3) into (5), solve (4) for v_limit, then substitute in
        // radius_abs_tol = (v_abs_tol^2 + 2*v_plan*v_abs_tol)/a_n_max;
        const double solver_radius_rel_tol =  (config->velocity_rel_tolerance + 2) * config->velocity_rel_tolerance;
        const double solver_radius_abs_tol = (config->velocity_abs_tolerance + 2. * v_plan_guess) * config->velocity_abs_tolerance / a_n_max;

        // Verify distance from blend midpoint to original intersection P < tolerance
        solution->T_plan = VecVecDisp(&solution->control_pts.P_mid, &P);

        // Look at both arcs and compute solution parameters (nominal geometry / arc_length)
        CHP(find_blend_size_from_intermediates(
            &solution->boundary,
            &solution->control_pts,
            &solution->R_geom_est,
            &solution->arc_len_est
            ));

        // Cheat by expressing max radius in terms of v_max even though it's not strictly the limiting factor
        double guess_v_max_length = solution->arc_len_est / (cycle_time * TP_MIN_SEGMENT_CYCLES);
        double guess_R_max_from_vel = pmSq(guess_v_max_length) / a_n_max;
        double guess_R_max = fmax(guess_R_max_from_vel, controls->req_min_radius);

        // This clamping throws off the slope-based update. Need to do slopes based on R_geom, or somehow compensate for it
        solution->R_plan = fmin(solution->R_geom_est, guess_R_max);

        // Check for solution convergence based on specified tolerances
        find_biarc_solution_error(
            &solution->boundary,
            controls->path_tolerance,
            R_goal,
            solution->R_plan,
            solution->T_plan,
            cycle_time,
            &solution->err);

        // Refine Rb if solution has not converged
        biarc_results->result = check_solution(
            config,
            a_n_max,
            R_goal,
            &biarc_results->constraints,
            &biarc_results->solution,
            solver_radius_rel_tol,
            solver_radius_abs_tol,
            biarc_results->iterations);

#ifdef BIARC_DEBUG
        print_json5_log_start("biarc_iteration");
        print_json5_long_("iteration", biarc_results->iterations);
        print_json5_long_("check_result", biarc_results->result);
        print_json5_biarc_solution_t("current_solution", &biarc_results->solution);
        print_json5_log_end();
#endif

        switch(biarc_results->result) {
        case BIARC_NOT_SOLVED:
            // Ok, keep going
            break;
        case BIARC_MIN_LENGTH_LIMITED:
        case BIARC_DEGENERATE_SEGMENTS:
        case BIARC_FAILED_TO_CONVERGE:
        case BIARC_EXCEEDED_MAX_ITERATIONS:
            // Lower bound of solution either doesn't work at all or doesn't meet our standards, so blend fails
            return TP_ERR_FAIL;
        case BIARC_REACHED_GOAL_SIZE:
        case BIARC_REACHED_TOLERANCE:
        case BIARC_CONVERGED:
            // Blend successful!
            return TP_ERR_OK;
        }

        // Refine Rb if solution has not converged yet
        solution->Rb = guess_interpolated_region_size(&biarc_results->constraints, R_goal, controls->path_tolerance);

        // Find blend start / end points and tangent vectors, within Rb of intersection P with procedure find_blend_points_and_tangents
        CHP(find_blend_points_and_tangents(solution->Rb, prev_tc, tc, &solution->boundary));
    }

    // Ran out of iterations, try the best solution we have

    *solution = biarc_results->constraints.lower;

    if (solution->err.deviation_margin >= -config->deviation_abs_tolerance) {
        biarc_results->result = BIARC_EXCEEDED_MAX_ITERATIONS;
        return TP_ERR_OK;
    } else {
        biarc_results->result = BIARC_FAILED_TO_CONVERGE;
        return TP_ERR_FAIL;
    }
}

#ifdef UNIT_TEST
#include <stdio.h>
#endif
#ifdef DO_BIARC_SCAN
tp_err_t scan_blend_properties(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    biarc_solver_config_t const * const config,
    PmVector const * const vel_bound,
    PmVector const * const acc_bound,
    biarc_solver_results_t * const biarc_results,
    BlendParameters * const param,
    double cycle_time,
    double resolution
    )
{
    if (!prev_tc || !tc || !biarc_results) {
        return TP_ERR_MISSING_INPUT;
    }

    // Find net tolerance that's actually usable (so that we're
    // not passing 0.0 around if tolerance is not specified)
    tcFindBlendTolerance(prev_tc, tc, &param->tolerance);

    blend_solver_constraints_t constraints;

    // Initial solver values for Rb range using somewhat dumb (but cheap) values
    constraints.upper = find_max_blend_region(prev_tc, tc);
    constraints.tolerance_upper = constraints.upper;

    blendParamInitVelocities(prev_tc, tc, config->feed_override_allowance, param);

    double s1_max = prev_tc->target - constraints.tolerance_upper;
    double s2_max = constraints.tolerance_upper;
    PmVector u1, u2;
    tcGetTangentUnitVector(prev_tc, s1_max, &u1);
    tcGetTangentUnitVector(tc, s2_max, &u2);

    findVMaxByAltitude(
        &u1,
        &u2,
        param);
    
    // Approximate the blend kinematics using the initial guess
    // Actual limits will be enforced later
    CHP(blendParamKinematics(
        &u1,
        &u2,
        param,
        acc_bound,
        vel_bound));

    // Intersection is at start of next segment, so use startpoint method since it's much faster
    EmcPose P;
    CHP(tcGetStartpoint(tc, &P));

#ifdef UNIT_TEST
    printf(">a_n_max %f\n", param->a_n_max);
    printf("|f,%12s,%12s,%12s,%12s,%12s,%12s\n",
           "Rb",
           "tol_dev",
           "R_geom",
           "R_plan",
           "v_plan",
           "d");
#endif
    const unsigned MAX_ITERATIONS = (unsigned)((double)constraints.upper.Rb / resolution);
    for (biarc_results->iterations = 1; biarc_results->iterations < MAX_ITERATIONS; ++biarc_results->iterations)
    {
        biarc_results->Rb = (double)biarc_results->iterations * resolution;
        // Find blend start / end points and tangent vectors, within Rb of intersection P with procedure find_blend_points_and_tangents
        find_blend_points_and_tangents(biarc_results->Rb, prev_tc, tc, &biarc_results->boundary);
        // Find blend segment lengths d, L and intermediate points with procedure find_blend_intermediate_segments
        int failed = 0;
        double T;
        if (TP_ERR_OK != find_blend_intermediate_segments(&biarc_results->boundary, &biarc_results->control_pts))
        {
            failed = 1;
            T=-1;
        } else {

        // Verify distance from blend midpoint to original intersection P < tolerance
        VecVecDisp(&biarc_results->control_pts.P_mid, &P.tran, &T);

        CHP(find_blend_size_from_intermediates(
            &biarc_results->boundary,
            &biarc_results->control_pts,
            &biarc_results->R_geom,
            &biarc_results->arc_len_est));

        // Cheat by expressing max radius in terms of v_max even though it's not strictly the limiting factor
        double guess_v_max_length = biarc_results->arc_len_est / cycle_time;
        double guess_R_effective_max = pmSq(guess_v_max_length) / param->a_n_max;

        biarc_results->R_plan = fmin(biarc_results->R_geom, guess_R_effective_max);
        }
#ifdef UNIT_TEST
            printf("|%d,%12f,%12f,%12f,%12f,%12f,%12f\n",
               failed,
               biarc_results->Rb,
               T,
               biarc_results->R_geom,
               biarc_results->R_plan,
               pmSqrt(param->a_n_max * biarc_results->R_plan),
               biarc_results->control_pts.d);
#endif
    }
    biarc_results->result = BIARC_EXCEEDED_MAX_ITERATIONS;
    return TP_ERR_FAIL;
}
#endif

const char * biarc_result_to_str(BiarcSolverStatusType r)
{
    switch (r) {
    case BIARC_DEGENERATE_SEGMENTS:
        return "degenerate";
    case BIARC_FAILED_TO_CONVERGE:
        return "diverged";
    case BIARC_EXCEEDED_MAX_ITERATIONS:
        return "max_iterations";
    case BIARC_NOT_SOLVED:
        return "not_solved";
    case BIARC_REACHED_GOAL_SIZE:
        return "reached_goal_size";
    case BIARC_REACHED_TOLERANCE:
        return "reached_tol";
    case BIARC_CONVERGED:
        return "converged";
    case BIARC_MIN_LENGTH_LIMITED:
        return "hit_min_len";
    }
    return "";
}

int find_arc_points_from_solution(
    PmVector const * const u1,
    PmVector const * const u2,
    PmVector const * const P1,
    PmVector const * const P2,
    PmVector const * const P,
    double d,
    BlendControls const * const controls,
    BlendParameters const *param,
    BlendPoints * const points
    )
{
    double straightness_margin = M_PI_2 - param->arc_intersection_halfangle;

    if (fabs(straightness_margin) < 1e-6) {
        points->motion_type = TC_LINEAR;
    } else {
        points->motion_type = TC_SPHERICAL;
        PmVector u_normal = VecVecSub(*u2, u1);
        CHP(VecUnitEq(&u_normal));

        double center_dist = d / cos(param->arc_intersection_halfangle);
        points->arc_center = VecVecAdd(VecScalMult(u_normal, center_dist), P);
    }

    points->arc_start = *P1;
    points->arc_end = *P2;
    points->u_tan = *u1;

    return 0;
}

/**
 * Given allowed limits, check if two sequential trajectory segments have C1 continuity.
 */
ContinuityCheck checkContinuity(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const next_tc,
    double c0_coincident_limit, //!< maximum displacement (in path units) between two points that are considered coincident
    double c1_angle_limit, //!< maximum angle between unit vectors to be considered tangent
    bool consume
    )
{
    const double dot_prod_min_val = 1.0 - c1_angle_limit;
    ContinuityCheck out = {};
    PmVector u_1, u_2;
    if (consume) {
        tcGetStartTangentUnitVector(prev_tc, &u_1);
    }else {
        tcGetEndTangentUnitVector(prev_tc, &u_1);
    }
    tcGetStartTangentUnitVector(next_tc, &u_2);
    out.u_diff = VecVecSub(u_2, &u_1);
    out.dot = VecVecDot(&u_1, &u_2);
    out.is_C1_continuous = out.dot >= dot_prod_min_val;
    PmVector p_1, p_2;

    if (consume) {
        tcGetStartpoint(prev_tc, &p_1);
    } else {
        tcGetEndpoint(prev_tc, &p_1);
    }
    tcGetStartpoint(next_tc, &p_2);

    out.p_diff = VecVecSub(p_2, &p_1);
    out.is_C0_continuous = VecMagSq(&out.p_diff) < pmSq(c0_coincident_limit);

    return out;
}

CircleAccLimits pmCircleActualMaxVel(
    double eff_radius,
    double v_max,
    double a_max)
{
    double a_n_max_cutoff = BLEND_ACC_RATIO_NORMAL * a_max;

    // Find the acceleration necessary to reach the maximum velocity
    double a_n_vmax = pmSq(v_max) / fmax(eff_radius, DOUBLE_FUZZ);
    // Find the maximum velocity that still obeys our desired tangential / total acceleration ratio
    double v_max_cutoff = sqrt(a_n_max_cutoff * eff_radius);

    double v_max_actual = v_max;
    double acc_ratio_tan = BLEND_ACC_RATIO_TANGENTIAL;

    if (a_n_vmax > a_n_max_cutoff) {
        v_max_actual = v_max_cutoff;
    } else {
        acc_ratio_tan = sqrt(1.0 - pmSq(a_n_vmax / a_max));
    }

    CircleAccLimits limits = {
        v_max_actual,
        acc_ratio_tan
    };

    return limits;
}

double do_line_profile_correction(const PmLine9 * L1, const PmLine9 * L2, double contour_tolerance, PmVector * P_intersect, int prev_id, int next_id)
{
#ifdef TP_DEBUG
    tp_debug_json5_log_start(do_line_profile_correction);
#endif
    tp_debug_json5_double(prev_id);
    tp_debug_json5_double(next_id);

    double L1_tmag = L1->tmag;
    double L2_tmag = L2->tmag;
    tp_debug_json5_double(L1_tmag);
    tp_debug_json5_double(L2_tmag);
    double min_length = fmin(L1_tmag, L2_tmag);
    tp_debug_json5_double(min_length);
    PmVector const *u1 = &L1->uVec;
    PmVector const *u2 = &L2->uVec;
    // "Normal" direction for the basic blend is u2 - u1, and the direction to shift the point is opposite, so u1-u2
    PmVector growth_dir_vec = VecVecSub(*u1, u2);
    double n_mag = VecMag(&growth_dir_vec);
    if (n_mag < CART_FUZZ) {
        // Don't bother correcting position for nearly straight segments
        tp_debug_json5_log_end("segments are nearly tangent");
        return 0;
    }
    const double SHARP_CORNER_DEG = 2.0;
    const double SHARP_CORNER_EPSILON = pmSq(PM_PI * ( SHARP_CORNER_DEG / 180.0));
    if (VecVecUnitAntiParallel(u1, u2, SHARP_CORNER_EPSILON)) {
        // Don't bother correcting position for nearly straight segments
        tp_debug_json5_log_end("corner is too sharp");
        return 0;
    }
    PmVector u_growth = VecScalMult(growth_dir_vec, 1./n_mag);

    // Compute the largest allowed blend arc based on tolerances
    // This ends up being very simple for the line-line blend if we don't worry about sizing for a given max velocity
    double intersection_angle = acos(VecVecDot(u1, u2));
    tp_debug_json5_double(intersection_angle);
    double d_max_geom = min_length / 2.0;
    tp_debug_json5_double(d_max_geom);
    double theta = intersection_angle / 2.0;
    tp_debug_json5_double(theta);
    const double CT = cos(theta);
    const double TT = tan(theta);
    const double h_to_d = CT / (1.-CT) * TT;
    double d = d_max_geom;
    tp_debug_json5_double(d);
    double h_ideal = d / h_to_d;
    tp_debug_json5_double(h_ideal);
    double h_limited_est = fmin(h_ideal, contour_tolerance);
    tp_debug_json5_double(h_limited_est);
    double d_limited = h_limited_est * h_to_d;


    // Penalize any unblended length past the (somewhat arbitrary) cutoff distance
    // This is kind of an ugly way to let users be lazy and use slightly larger blend tolerances without screwing up large exact-path segments
    const double UNBLENDED_LENGTH_MIN_CUTOFF = min_length * 0.1;
    const double UNBLENDED_LENGTH_MAX_CUTOFF = min_length * 0.5;
    tp_debug_json5_double(UNBLENDED_LENGTH_MIN_CUTOFF);
    tp_debug_json5_double(UNBLENDED_LENGTH_MAX_CUTOFF);
    double unblended_length_est = min_length - 2.*d_limited;
    tp_debug_json5_double(unblended_length_est);
    double punishment_num = (min_length-unblended_length_est)/min_length - UNBLENDED_LENGTH_MIN_CUTOFF;
    double punishment_den = UNBLENDED_LENGTH_MAX_CUTOFF - UNBLENDED_LENGTH_MIN_CUTOFF;
    double unblended_length_ratio = punishment_num / punishment_den;
    double unblended_punishment = sqrt(fmin(fmax(unblended_length_ratio, 0.0), 1.0));
    tp_debug_json5_double(unblended_punishment);

    double correction_dist_blendarc = h_limited_est * unblended_punishment / CT;
    tp_debug_json5_double(correction_dist_blendarc);

// Disable internal smoothing correction for now because there's too many ways
// for it to fail, like if the velocity doesn't reach the nominal value,
// due to feed override, being near the end of the queue, etc.
#ifdef DO_SMOOTHING_CORRECTION
    double d_final = correction_dist_blendarc * h_to_d;
    double R_final = d_final / TT;
    double s_arc_max = R_final * 2.0 * theta;
    
    // Need to do this more carefully (better characterization of geometry, handling of unblended portions, better calculation of blend velocity)
    double correction_dist_smoothing_raw = arc_centroid_approx(1./R_final, fmin(smooth_dist_at_vel, s_arc_max));
    tp_debug_json5_double(correction_dist_smoothing_raw);
    double correction_dist_smoothing = correction_dist_smoothing_raw * unblended_punishment;
    tp_debug_json5_double(correction_dist_smoothing);
#else
    double correction_dist_smoothing = 0.0;
#endif

    double correction_dist = correction_dist_blendarc + correction_dist_smoothing;
    if (correction_dist > 0) {
        PmVector correction_disp = VecScalMult(u_growth, correction_dist);
        VecVecAddEq(P_intersect, &correction_disp);
#ifdef TP_DEBUG
        print_json5_PmVector(correction_disp);
        print_json5_PmVector_("old_endpoint", L1->end);
        print_json5_PmVector_("new_endpoint", *P_intersect);
#endif
    }

#ifdef TP_DEBUG
    print_json5_end_();
#endif
    return correction_dist;
}

double arc_centroid_approx(double curvature, double s)
{
    //6      120     5040   362880
    double x = s * curvature/2;
    const double xsq = pmSq(x);
    double series_val = ((((xsq/39916800. - 1./362880.)*xsq + 1./5040)*xsq - 1./120)*xsq + 1./6)*x;
    return s/2. * series_val;
}

