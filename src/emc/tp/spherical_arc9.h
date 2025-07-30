#ifndef SPHERICAL_ARC9_H
#define SPHERICAL_ARC9_H

#include "pm_vector.h"
#include "spherical_arc9_types.h"

#define ARC_POS_EPSILON 1e-12
#define ARC_MIN_RADIUS 1e-12
#define ARC_MIN_ANGLE 1e-6

int arc9InitFromPoints(
    SphericalArc9 * const arc,
    PmVector const * const start,
    PmVector const * const end,
    PmVector const * const center,
    const PmVector * const uTan);

int arc9Point(SphericalArc9 const * const arc, double angle_in, PmVector * const out);

double arc9Length(SphericalArc9 const * const arc);

int arc9Tangent(SphericalArc9 const * const arc, const double t, PmVector * const out);

double arc9VLimit(SphericalArc9 const * const arc, double v_target, double v_limit_linear, double v_limit_angular);

CircleAccLimits arc9AccLimit(
    SphericalArc9 const *arc,
    double v_max,
    double a_max);
#endif // SPHERICAL_ARC9_H
