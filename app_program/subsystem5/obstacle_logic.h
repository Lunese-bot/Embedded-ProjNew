/*
 *      obstacle_logic.h
 *      Subsystem 5 -- pure algorithms: obstacle profiling and bypass
 *      planning.  No kernel calls and no hardware, so every function here
 *      can be unit-tested with made-up scan data (subsystem5_test.c).
 *
 *      All tuning values (A6, "needs validation" in design review 8.3) are
 *      collected here.  Change them after measuring, and record why.
 */

#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include "subsystem5.h"

/* ---- Detection -------------------------------------------------- */
#define S5_DETECT_MM      250 /* obstacle closer than this ...   */
#define S5_DETECT_COUNT   3   /* ... this many pings in a row     */
#define S5_HIT_RANGE_MM   500 /* during a scan: "part of it"      */
#define S5_CLUSTER_GAP_MM 150 /* jump that separates two objects  */

/* ---- Robot geometry (measure the real car) ---------------------- */
#define S5_ROBOT_HALF_WIDTH_MM 80
#define S5_ROBOT_LENGTH_MM     200
#define S5_SIDE_MARGIN_MM      60  /* extra gap beside the obstacle    */
#define S5_OBSTACLE_DEPTH_MM   150 /* assumed: ultrasonic can't see it */
#define S5_MIN_CLEAR_MM        300 /* free range needed on bypass side */

/* ---- Bypass manoeuvre ------------------------------------------- */
#define S5_BYPASS_TURN_DEG   45
#define S5_BYPASS_SPEED_MM_S 150
#define S5_SEARCH_SPEED_MM_S 100
#define S5_SEARCH_EXTRA_MM   100 /* drive this far past expected line */
#define S5_REVERSE_MM        150 /* when no side is safe              */

/* ---- Line recovery (bounded, NFR5) ------------------------------ */
#define S5_MAX_RECOVERY      3
#define S5_RECOVERY_TURN_DEG 20
#define S5_RECOVERY_BASE_MM  100
#define S5_RECOVERY_STEP_MM  50

/* One ultrasonic sample taken during a scan */
typedef struct
{
    int8_t   deg; /* servo angle, + = left               */
    uint16_t mm;  /* distance, 0 = no echo (free space)  */
} s5_sample_t;

/* One step of a manoeuvre */
typedef struct
{
    s5_motion_req_t m;
    uint8_t         search_line; /* 1 = stop early when line seen */
} s5_step_t;

#define S5_PLAN_MAX 10

/* sin(deg) * 1000, deg in -90..+90 */
extern int32_t s5_sin1000(int32_t deg);

/* Build a profile from samples sorted by angle (most negative first).
   Returns TRUE and fills *p when an obstacle is found. */
extern bool s5_build_profile(const s5_sample_t * p_samples, int32_t count,
                             s5_profile_t * p_profile);

/* Pick the bypass side for a profile (also stored in p->bypass_side). */
extern s5_side_t s5_choose_side(s5_profile_t * p_profile);

/* Lateral offset (mm) the car must move sideways to clear the obstacle
   on the given side. */
extern int32_t s5_lateral_needed_mm(const s5_profile_t * p_profile,
                                    s5_side_t            side);

/* Steps of the bypass for p->bypass_side.  Returns the step count. */
extern int32_t s5_plan_bypass(const s5_profile_t * p_profile,
                              s5_step_t * p_plan, int32_t max);

/* Steps of recovery attempt (0-based) after the line was not found. */
extern int32_t s5_plan_recovery(s5_side_t side, int32_t attempt,
                                s5_step_t * p_plan, int32_t max);

/* Angles for the coarse scan, and the fine scan around a profile */
extern int32_t s5_coarse_angles(int8_t * p_deg, int32_t max);
extern int32_t s5_fine_angles(const s5_profile_t * p_profile, int8_t * p_deg,
                              int32_t max);

#endif /* OBSTACLE_LOGIC_H */

/*** end of file ***/
