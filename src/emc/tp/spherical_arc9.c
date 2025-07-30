/**
 * @file spherical_arc9.c
 * 
 * API for 9D version of "spherical arc" (an arc on the surface of a 9D sphere)
 *
 * @author Robert W. Ellenberg <rwe24g@gmail.com>
 *
 * @copyright Copyright 2019, Robert W. Ellenberg
 *
 * This source code is released for free distribution under the terms of the
 * GNU General Public License (V2) as published by the Free Software Foundation.
 */

#include "spherical_arc9.h"
#include "posemath.h"
#include "pm_vector.h"
#include "rtapi_math.h"
#include "tp_enums.h"
#include "blendmath_types.h"
#include "pm_circle9.h" // For spiral radius calculation

static inline double sq(double x) {return x*x;}


int arc9Point(SphericalArc9 const * const arc, double progress, PmVector * const out)
{
#ifdef ARC_PEDANTIC
    if (!arc) {return TP_ERR_MISSING_INPUT;}
    if (!out) {return TP_ERR_MISSING_OUTPUT;}
#endif

    //Convert progress to actual progress around the arc
    double angle_in = progress / arc->radius;
    double scale0 = sin(arc->angle - angle_in) / arc->Sangle;
    double scale1 = sin(angle_in) / arc->Sangle;

    PmVector interp0 = VecScalMult(arc->rStart, scale0);
    PmVector interp1 = VecScalMult(arc->rEnd, scale1);

    *out = VecVecAdd(interp0, &interp1);
    VecVecAddEq(out, &arc->center);
    return TP_ERR_OK;
}

double arc9Length(SphericalArc9 const * const arc)
{
    return arc->radius * arc->angle;
}

int arc9Tangent(SphericalArc9 const * const arc, double const t, PmVector * const out)
{
    if (!arc || !out) {
        return TP_ERR_MISSING_INPUT;
    }

    // This section implements the derivative of the SLERP formula to get a
    // local tangent vector.
    const double theta = arc->angle;
    const double k = theta / arc->Sangle;
    const double k0 = -cos( (1.0 - t) * theta);
    const double k1 = cos( t * theta);


    // Ugly sequence to build up tangent vector from components of the derivative
    PmVector dp0 = VecScalMult(arc->rStart, k * k0);
    PmVector dp1 = VecScalMult(arc->rEnd, k * k1);
    *out = VecVecAdd(dp0, &dp1);

    // tangential vector complete, now normalize
    VecUnitEq(out);

    return TP_ERR_OK;
}

double arc9VLimit(const SphericalArc9 * const arc, double v_target, double v_limit_linear, double v_limit_angular)
{
    return VecVLimit(&arc->uTan, v_target, v_limit_linear, v_limit_angular);
}

CircleAccLimits arc9AccLimit(SphericalArc9 const *arc,
    double v_max,
    double a_max)
{
    double a_n_max_cutoff = BLEND_ACC_RATIO_NORMAL * a_max;
    double eff_radius = spiralEffectiveRadius(arc->radius, arc->angle, arc->spiral);

    // Find the acceleration necessary to reach the maximum velocity
    double a_n_vmax = sq(v_max) / fmax(eff_radius, DOUBLE_FUZZ);
    // Find the maximum velocity that still obeys our desired tangential / total acceleration ratio
    double v_max_cutoff = sqrt(a_n_max_cutoff * eff_radius);

    double v_max_actual = v_max;
    double acc_ratio_tan = BLEND_ACC_RATIO_TANGENTIAL;

    if (a_n_vmax > a_n_max_cutoff) {
        v_max_actual = v_max_cutoff;
    } else {
        acc_ratio_tan = sqrt(1.0 - sq(a_n_vmax / a_max));
    }

    CircleAccLimits limits = {
        v_max_actual,
        acc_ratio_tan
    };

    return limits;
}
