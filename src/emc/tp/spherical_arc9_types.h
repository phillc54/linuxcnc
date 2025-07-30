#ifndef SPHERICAL_ARC9_TYPES_H
#define SPHERICAL_ARC9_TYPES_H

#include "pm_vector.h"
typedef struct spherical_arc_9_t {
    // Three defining points for the arc
    PmVector start;
    PmVector end;
    PmVector center;
    // Relative vectors from center to start and center to end
    // These are cached here since they'll be reused during SLERP
    PmVector rStart;
    PmVector rEnd;
    PmVector uTan;   /* Tangent vector at start of arc (copied from
                           prev. tangent line)*/
    double estimated_dev; // Estimated deviation from nominal end point (tolerance for the blend)
    double radius;
    double spiral;
    // Angle that the arc encloses
    double angle;
    double Sangle;
} SphericalArc9;

#endif // SPHERICAL_ARC9_TYPES_H
