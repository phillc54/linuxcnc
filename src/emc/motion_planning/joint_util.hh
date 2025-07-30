#ifndef JOINT_UTIL_HH
#define JOINT_UTIL_HH

#include <posemath.h>
#include "pm_vector.h"

PmCartesian getXYZAccelBounds();
PmCartesian getXYZVelBounds();

PmVector getAccelBounds();
PmVector getVelBounds();

unsigned jointAccelViolation(int joint_idx, double acc);
unsigned jointVelocityViolation(int joint_idx, double v_actual);
unsigned jointMaxPositionViolation(int joint_idx, double position);
unsigned jointMinPositionViolation(int joint_idx, double position);

unsigned findAccelViolations(PmVector axis_accel);
unsigned findVelocityViolations(PmVector axis_vel);
unsigned findMaxPositionViolations(PmVector axis_pos);
unsigned findMinPositionViolations(PmVector axis_pos);

/**
 * Finds the smallest non-zero component in a non-negative "bounds" vector.
 * Used to identify the "slowest" axis, but ignore axes
 */
double findMinNonZero(PmCartesian const * const bounds);

#endif // JOINT_UTIL_HH
