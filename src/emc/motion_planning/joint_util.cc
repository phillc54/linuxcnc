#include "joint_util.hh"
#include "pm_vector.h"
#include "emcglb.h"
#include "limit_checks.hh"

PmCartesian getXYZAccelBounds() {
    PmCartesian acc_bound = {
        axis_max_acceleration[0],
        axis_max_acceleration[1],
        axis_max_acceleration[2],
    };
    return acc_bound;
}

PmCartesian getXYZVelBounds() {
    PmCartesian vel_bound = {
        axis_max_velocity[0],
        axis_max_velocity[1],
        axis_max_velocity[2],
    };
    return vel_bound;
}

PmVector getAccelBounds()
{
    PmVector acc_bound={};
    for (int i = 0; i < PM_VECTOR_SIZE; ++i) {
        acc_bound.ax[i] = axis_max_acceleration[i];
    }
    return acc_bound;
}

PmVector getVelBounds()
{
    PmVector vel_bound={};
    for (int i = 0; i < PM_VECTOR_SIZE; ++i) {
        vel_bound.ax[i] = axis_max_velocity[i];
    }
    return vel_bound;
}

unsigned jointMaxPositionViolation(int joint_idx, double position)
{
    static const double ABS_TOL = 1e-6;
    return (unsigned)(position > axis_max_position_limit[joint_idx] + ABS_TOL) << joint_idx;
}

unsigned jointMinPositionViolation(int joint_idx, double position)
{
    static const double ABS_TOL = 1e-6;
    return (unsigned)(position < axis_min_position_limit[joint_idx] - ABS_TOL) << joint_idx;
}

/**
 * Checks for any acceleration violations based on axis limits.
 *
 * @return 0 if all acceleration bounds are respected, or a bit-set of failed axes (in XYZABCUVW bit order).
 */
unsigned findAccelViolations(PmVector axis_accel)
{

    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0);
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        fail_bits |= jointAccelViolation(i, axis_accel.ax[i]);
    }
    return fail_bits;
}

unsigned findVelocityViolations(PmVector axis_vel)
{

    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0);
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        fail_bits |= jointVelocityViolation(i, axis_vel.ax[i]);
    }
    return fail_bits;
}

unsigned findMaxPositionViolations(PmVector axis_pos)
{

    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0);
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        fail_bits |= jointMaxPositionViolation(i, axis_pos.ax[i]);
    }
    return fail_bits;
}

unsigned findMinPositionViolations(PmVector axis_pos)
{

    // Bit-mask each failure so we can report all failed axes
    unsigned fail_bits = (unsigned)(0x0);
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        fail_bits |= jointMaxPositionViolation(i, axis_pos.ax[i]);
    }
    return fail_bits;
}

double findMinNonZero(const PmCartesian * const bounds) {
    //Start with max accel value
    double act_limit = fmax(fmax(bounds->x, bounds->y), bounds->z);

    // Compare only with active axes
    if (bounds->x > 0) {
        act_limit = fmin(act_limit, bounds->x);
    }
    if (bounds->y > 0) {
        act_limit = fmin(act_limit, bounds->y);
    }
    if (bounds->z > 0) {
        act_limit = fmin(act_limit, bounds->z);
    }
    return act_limit;
}

/**
 * Checks all axis values in an EmcPose to see if they exceed a magnitude threshold.
 * @return a bitmask that is 0 if all axes are within the threshold. Any
 * out-of-limit axes set their corresponding bit to 1 in the returned value (X
 * is 0th bit, Y is 1st, etc.).
 */
unsigned int findAbsThresholdViolations(PmVector vec, double threshold)
{
    threshold = fabs(threshold);
    unsigned fail_bits = (unsigned)(0x0);
    for (int i=0; i < PM_VECTOR_SIZE; ++i) {
        fail_bits |= ((fabs(vec.ax[i]) > threshold) << i);
    }
    return fail_bits;
}
