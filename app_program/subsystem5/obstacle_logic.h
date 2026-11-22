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
#define S5_DETECT_MM      225 /* obstacle closer than this ...   */
#define S5_DETECT_COUNT   3   /* ... this many pings in a row     */
#define S5_HIT_RANGE_MM   500 /* during a scan: "part of it"      */
#define S5_SCAN_MAX_DEG   45  /* scan no wider than this (= SERVO_MAX_DEG) */
#define S5_CLUSTER_GAP_MM 150 /* jump that separates two objects  */

/* ---- Stopping distance (Tribunal 1, Version C) -------------------
 *  Required detect range >= reaction distance + braking + safety margin
 *  reaction distance = top speed x detection latency
 *  The robot STOPS before it sweeps the servo, so sweep time is not part
 *  of the reaction distance (the report's first formula included it).
 *
 *  latency = S5_DETECT_COUNT pings x 20 ms      (3 x 20 = 60 ms)
 *          + one echo at the 2 m range limit     (~12 ms)
 *          + one Motion Control period           (10 ms)        = 82 ms
 *  reaction at 400 mm/s (S2_MAX_SPEED_MM_S) = 400 x 0.082 ~= 33 mm
 *
 *  S5_BRAKING_MM is an ESTIMATE: measure it (S5-U22) and update it.
 *  The build fails if the detect distance stops covering the total. */
#define S5_TOP_SPEED_MM_S      400 /* = S2_MAX_SPEED_MM_S (subsystem2.h) */
#define S5_LATENCY_MS          ((S5_DETECT_COUNT * 20) + 12 + 10)
#define S5_REACTION_MM         ((S5_TOP_SPEED_MM_S * S5_LATENCY_MS + 999) / 1000)
#define S5_BRAKING_MM          100 /* TODO measure at top speed (S5-U22) */
#define S5_STOP_MARGIN_MM      50
#define S5_STOP_NEEDED_MM \
    (S5_REACTION_MM + S5_BRAKING_MM + S5_STOP_MARGIN_MM)

_Static_assert(S5_DETECT_MM >= S5_STOP_NEEDED_MM,
               "S5_DETECT_MM too short to stop: reaction + braking + margin");

/* ---- Robot geometry (measure the real car) ---------------------- */
#define S5_ROBOT_HALF_WIDTH_MM 80
#define S5_ROBOT_LENGTH_MM     200
#define S5_SIDE_MARGIN_MM      20  /* extra gap beside the obstacle    */
#define S5_OBSTACLE_DEPTH_MM   150 /* assumed: ultrasonic can't see it */
#define S5_MIN_CLEAR_MM        300 /* free range needed on bypass side */
#define S5_SIDE_BIAS_DEG        10 /* obstacle centre clearly to one side */

/* ---- Bypass manoeuvre ------------------------------------------- */
#define S5_BYPASS_TURN_DEG   35
#define S5_BYPASS_FIRST_MM   80 /* short first diagonal before moving farther sideways */
#define S5_BYPASS_CLEARANCE_DEG 45 /* heading for the rest of the side clearance */
#define S5_RETURN_TURN_DEG   40
#define S5_BYPASS_SPEED_MM_S 60
#define S5_SEARCH_SPEED_MM_S 60
#define S5_SEARCH_MARGIN_MM   40 /* extra sideways reach on the return search */
#define S5_SEARCH_RETRY_MM    85 /* straight retry after a return with no sighting */
#define S5_REJOIN_PROBE_DEG   15 /* check to either side after alignment */
#define S5_REJOIN_SEARCH_MM  100 /* short drive toward the last sighting */
#define S5_REJOIN_CHECK_MM    25 /* creep before confirming the line */
#define S5_REVERSE_MM        150 /* when no side is safe              */

/* ---- Line recovery (bounded, NFR5) ------------------------------ */
#define S5_RECOVERY_STRAIGHT_MM 175 /* search along the original heading */
#define S5_FINAL_SEARCH_MM       30 /* first short creep on that heading */
#define S5_EXTRA_SEARCH_MM       35 /* one more creep if tape is unconfirmed */

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

/* Ordered coarse + fine samples; the fine value wins at equal angles. */
extern int32_t s5_merge_samples(const s5_sample_t *a, int32_t na,
                                const s5_sample_t *b, int32_t nb,
                                s5_sample_t *out, int32_t max);

/* Pick the bypass side for a profile (also stored in p->bypass_side). */
extern s5_side_t s5_choose_side(s5_profile_t * p_profile);

/* Lateral offset (mm) the car must move sideways to clear the obstacle
   on the given side. */
extern int32_t s5_lateral_needed_mm(const s5_profile_t * p_profile,
                                    s5_side_t            side);

/* Steps of the bypass for p->bypass_side.  Returns the step count. */
extern int32_t s5_plan_bypass(const s5_profile_t * p_profile,
                              s5_step_t * p_plan, int32_t max);

/* Face the approach heading, then search along it. heading_deg is relative
   to the heading before the bypass. */
extern int32_t s5_plan_straight_recovery(int32_t heading_deg,
                                        s5_step_t * p_plan, int32_t max);

/* Angles for the coarse scan, and the fine scan around a profile */
extern int32_t s5_coarse_angles(int8_t * p_deg, int32_t max);
extern int32_t s5_fine_angles(const s5_profile_t * p_profile, int8_t * p_deg,
                              int32_t max);

#endif /* OBSTACLE_LOGIC_H */

/*** end of file ***/
