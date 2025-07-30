/********************************************************************
* Description: blendmath.h
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
#ifndef BLENDMATH_HH
#define BLENDMATH_HH

#include "posemath.h"
#include "tc_types.h"
#include "tp_enums.h"
#include "blendmath_types.h"
#include "pm_vector.h"

double findIntersectionAngle(PmVector const * const u1,
        PmVector const * const u2);

PmVector normalizeToBounds(PmVector vec,
        PmVector const *bounds);

double findAbsMaxScale(PmVector vec,
        PmVector const *bounds);

int findMaxValueOnPlane(PmVector const * plane_envelope_sq,
        PmVector const * bounds,
        double * max_planar_value);

int quadraticFormula(
    double A,
    double B,
    double C,
    double * const root0,
    double * const root1);

int blendCheckConsume(BlendParameters * const param,
        double L_prev);

int find_blend_parameters(const PmVector * const u1,
    const PmVector * const u2,
    PmVector const * const acc_bound,
    PmVector const * const vel_bound, const BlendControls * controls,
    double v_max1, double v_max2,
    BlendParameters * const param);

int find_blend_vel_accel_planar_limits(
    PmVector const * const u_tan1,
    PmVector const * const u_tan2,
    PmVector const * const acc_bound,
    PmVector const * const vel_bound,
    double *a_max_planar,
    double *v_max_planar);

double findMaxByAltitude(PmVector const * const u1,
    PmVector const * const u2, double max1, double max2,
    double included_angle);

int init_blend_controls_from_segments(TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    double override_allowance,
    BlendControls * const controls);

tp_err_t init_blend_segment_geometry(TC_STRUCT * const blend_tc,
    BlendPoints const *points);

// API for biarc blends

const char * biarc_result_to_str(BiarcSolverStatusType r);

tp_err_t find_blend_points_and_tangents(
    double Rb,
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    blend_boundary_t * const out);

int find_blend_intermediate_segments(biarc_solver_config_t const *config,
    blend_boundary_t const * const boundary,
    biarc_control_points_t * const control_pts,
    bool symmetrical);

double find_max_blend_region(TC_STRUCT const * const prev_tc, TC_STRUCT const * const tc, bool symmetrical);

tp_err_t find_blend_size_from_intermediates(blend_boundary_t const * const blend_boundary,
    biarc_control_points_t const * const intermediates,
    double *R_geom, double * arc_len_est);

tp_err_t optimize_biarc_blend_size(TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    const biarc_solver_config_t * const config,
    PmVector const * const vel_bound,
    PmVector const * const acc_bound,
    biarc_solver_results_t * const biarc_results, BlendControls * const controls,
    double cycle_time);

// For testing only
tp_err_t scan_blend_properties(
    TC_STRUCT const * const prev_tc,
    TC_STRUCT const * const tc,
    biarc_solver_config_t const * const config,
    PmVector const * const vel_bound,
    PmVector const * const acc_bound,
    biarc_solver_results_t * const biarc_results,
    BlendParameters * const param,
    double cycle_time,
    double resolution);

int blend_find_arcpoints3(
    BlendPoints * const points,
    biarc_solver_results_t const * const);

int find_arc_points_from_solution(PmVector const * const u1,
    PmVector const * const u2,
    PmVector const * const P1,
    PmVector const * const P2,
    PmVector const * const P,
    double d, const BlendControls * const controls,
    const BlendParameters * param,
    BlendPoints * const points);

ContinuityCheck checkContinuity(const TC_STRUCT * const prev_tc,
    TC_STRUCT const * const next_tc, double c0_coincident_limit, double c1_angle_limit, bool consume);

CircleAccLimits pmCircleActualMaxVel(
    double eff_radius,
    double v_max3,
    double a_max3);

double do_line_profile_correction(const PmLine9 * L1, const PmLine9 * L2, double contour_tolerance, PmVector * P_intersect, int prev_id, int next_id);

double arc_centroid_approx(double curvature, double arc_len);

#endif
