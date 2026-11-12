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
/* ... that agree.  A real obstacle only gets CLOSER as the robot drives at it
   (up to about 40 mm per 60 ms ping at 0.6 m/s) and is steady when the robot
   is stopped.  Noise jumps both ways, so a reading that moves AWAY must stay
   within a much smaller step than one that moves closer. */
#define S5_DETECT_TOL_CLOSER_MM 40
#define S5_DETECT_TOL_AWAY_MM   15
#define S5_MONITOR_PERIOD_MS 60 /* time between monitoring pings.  The HC-SR04
                                   data sheet advises >= 60 ms: at 20 ms the
                                   late echo of one ping looks like a close
                                   object after the next trigger. */
#define S5_HIT_RANGE_MM   500 /* during a scan: "part of it"      */
#define S5_SCAN_MAX_DEG   60  /* scan no wider than this (= SERVO_MAX_DEG) */
#define S5_CLUSTER_GAP_MM 150 /* jump that separates two objects  */
/* Fine scan: degrees past each coarse edge.  Was 20; 15 keeps a step
   beyond the true edge (coarse edges can be 30 deg off) while cutting
   the scan time.  Line-follow run 2026-11-12: the scan took ~8 s and hit
   S5_SCAN_TMO_MS (fault 0x5003). */
#define S5_FINE_MARGIN_DEG 15

/* ---- Stopping distance (Tribunal 1, Version C) -------------------
 *  Required detect range >= reaction distance + braking + safety margin
 *  reaction distance = top speed x detection latency
 *  The robot STOPS before it sweeps the servo, so sweep time is not part
 *  of the reaction distance (the report's first formula included it).
 *
 *  latency = S5_DETECT_COUNT pings x 60 ms      (3 x 60 = 180 ms)
 *          + one echo at the 2 m range limit     (~12 ms)
 *          + one Motion Control period           (10 ms)        = 202 ms
 *  reaction at 400 mm/s (S2_MAX_SPEED_MM_S) = 400 x 0.202 ~= 80 mm
 *
 *  S5_BRAKING_MM is an ESTIMATE: measure it (S5-U22) and update it.
 *  The build fails if the detect distance stops covering the total. */
#define S5_TOP_SPEED_MM_S      400 /* = S2_MAX_SPEED_MM_S (subsystem2.h) */
#define S5_LATENCY_MS          ((S5_DETECT_COUNT * S5_MONITOR_PERIOD_MS) + 12 + 10)
#define S5_REACTION_MM         ((S5_TOP_SPEED_MM_S * S5_LATENCY_MS) / 1000)
#define S5_BRAKING_MM          100 /* TODO measure at top speed (S5-U22) */
#define S5_STOP_MARGIN_MM      50
#define S5_STOP_NEEDED_MM \
    (S5_REACTION_MM + S5_BRAKING_MM + S5_STOP_MARGIN_MM)

_Static_assert(S5_DETECT_MM >= S5_STOP_NEEDED_MM,
               "S5_DETECT_MM too short to stop: reaction + braking + margin");

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

/* Merge the coarse scan (a) and the fine scan (b) into one list sorted by
   angle (most negative first).  On equal angles the fine sample wins.
   The fine scan only covers the obstacle and 20 deg past each edge; the
   coarse samples outside that window keep the clearance on each side. */
extern int32_t s5_merge_samples(const s5_sample_t * p_a, int32_t count_a,
                                const s5_sample_t * p_b, int32_t count_b,
                                s5_sample_t * p_out, int32_t max);

/* Monitoring filter: next count of consecutive near readings.
   cur_mm = 0 (no echo) or >= S5_DETECT_MM -> 0.  A near reading that
   is more than S5_DETECT_TOL_CLOSER_MM closer, or more than
   S5_DETECT_TOL_AWAY_MM further, than the previous near one starts the count
   again at 1 (noise jumps around, a real object does not). */
extern int32_t s5_near_count_next(int32_t count, uint16_t last_mm,
                                  uint16_t cur_mm);

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
