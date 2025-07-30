/********************************************************************
* Description: tp_types.h
*   Trajectory planner types and constants
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef TP_TYPES_H
#define TP_TYPES_H

#include "tc_types.h"
#include "tcq_types.h"

#ifndef __cplusplus
#include <rtapi_bool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TP_DEFAULT_QUEUE_SIZE 32
/* Minimum length of a segment in cycles (must be greater than 1 to ensure each
 * segment is hit at least once.) */
#define TP_MIN_SEGMENT_CYCLES 1.02
/* Values chosen for accel ratio to match parabolic blend acceleration
 * limits. */
#define TP_OPTIMIZATION_CUTOFF 4
/* If the queue is shorter than the threshold, assume that we're approaching
 * the end of the program */
#define TP_QUEUE_THRESHOLD 3

/* closeness to zero, for determining if a move is pure rotation */
#define TP_PURE_ROTATION_EPSILON 1e-6

/* "neighborhood" size (if two values differ by less than the epsilon,
 * then they are effectively equal.)*/
#define TP_ACCEL_EPSILON 1e-4
#define TP_VEL_EPSILON   DOUBLE_FUZZ
#define TP_POS_EPSILON   1e-12
#define TP_TIME_EPSILON  1e-12
#define TP_ANGLE_EPSILON 1e-6
#define TP_ANGLE_EPSILON_SQ (TP_ANGLE_EPSILON * TP_ANGLE_EPSILON)
#define TP_MIN_ARC_ANGLE 1e-3
#define TP_MIN_ARC_LENGTH 1e-6
#define TP_BIG_NUM 1e10

/**
 * Persistant data for spindle status within tpRunCycle.
 * This structure encapsulates some static variables to simplify refactoring of
 * synchronized motion code.
 */
typedef struct {
    spindle_origin_t origin; //!< initial position of spindle during synchronization (direction-aware)

    double trigger_revs;
} tp_spindle_t;

// Bitmask for wait conditions (for query functions)
typedef unsigned WaitFlagMask;

typedef enum {
    WAIT_FOR_SPINDLE_ATSPEED,
    WAIT_FOR_SPINDLE_INDEX,
    WAIT_FOR_PROBE_READY,
    WAIT_FOR_PROBING,
    WAIT_FOR_OPTIMIZATION,
    WAIT_FOR_INDEXER_UNLOCK,
    WAIT_FOR_FILTER_DISABLE,
    WAIT_FOR_FILTER_ENABLE,
    MAX_WAIT_INDICES
} WaitFlagIndex;

typedef struct {
    tp_spindle_t spindle_cmd; //Spindle data
    int tc_completed_id; /* ID of most recent completed segment, i.e. "-1" in the queue"*/

    PmVector currentPos;
    PmVector currentVel;

    double vLimit;		/* absolute upper limit on all linear vels */
    double vLimitAng;		/* absolute upper limit on all angular vels */

    int execId;
    struct state_tag_t execTag; /* state tag corresponding to running motion */
    int nextexecId;
    int joint_filter_drain_counter;
    bool filters_at_rest;
    int activeDepth;		/* number of motions blending */
    int aborting; // Abort is in progress (TP resets this flag to zero after stopping)
    int pausing; // Pause is requested, indicates TP should slow down and stop until flag is cleared.
    int reverse_run;      /* Indicates that TP is running in reverse */
    int motionType;

    double time_elapsed_sec; // Total elapsed TP run time in seconds
    long long time_elapsed_ticks; // Total elapsed TP run time in cycles (ticks)
    long long time_at_wait; // Time when TP started to wait for spindle

    int waiting[MAX_WAIT_INDICES];
} tp_execution_t;


typedef struct {
    int queue_size_config;
    double cycleTime;
    int superSampleRate;

    int nextId;
    tc_unique_id_t nextUniqueId;

    blend_mode_t blend_mode;
    tc_spindle_sync_t synchronized;       // spindle sync required for this move
    double uu_per_rev;          /* user units per spindle revolution */

    syncdio_t syncdio; //record tpSetDout's here

    // Fields moved from emcmotStatus since this needs to be managed synchronously in task
    spindle_cmd_t spindle_cmd;
    unsigned char enables_new;

    bool atspeed_next_feed; // Indicates next feed move must wait for spindle at-speed
    bool wait_for_speed_change; // If true, trigger a new at-speed wait any time the spindle speed changes

    unsigned char wait_for[MAX_WAIT_INDICES];

} tp_planning_t;


/**
 * Trajectory planner state structure.
 * Stores persistant data for the trajectory planner that should be accessible
 * by outside functions.
 */
typedef struct tp_data_t {
    TC_QUEUE_STRUCT queue; // Shared queue structure used by both the planner and execution parts

    tp_execution_t exec; // trajectory flags and fields used during execution of trajectories

    tp_planning_t planner; // flags and states used for planning / lookahead

    // These are likely planner state but we need to figure out how this would
    // work safely with init / clears initiated from the RT side.
    PmVector goalPos;
    double cycleTime;
    int superSampleRate;

} TP_STRUCT;

typedef struct {
    char buf[100];
} LineDescriptor;

#ifdef __cplusplus
}
#endif
#endif				/* TP_TYPES_H */
