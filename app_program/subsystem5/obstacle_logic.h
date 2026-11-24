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

/* ---- Measured pass (side look) -----------------------------------
 *  While passing, the ultrasonic looks sideways at the obstacle and the
 *  odometer gives forward travel.  Three phases: seeking the side ->
 *  alongside -> past the rear corner.  The rear corner position is the
 *  travel at the last echo + sensor-to-axle + the forward reach of the
 *  beam (range x cos(look angle)); the robot then drives on until its
 *  rear (axle - rear overhang) and the swept footprint of the inward
 *  pivot are past it.
 *
 *  OFF by default.  It needs robot dimensions that only the owner can
 *  measure, so there are NO default values: to enable it define
 *  S5_SIDE_PASS 1 together with S5_SERVO_AHEAD_MM (axle to servo shaft),
 *  S5_SERVO_ARM_MM (servo shaft to the sensor midpoint), S5_REAR_OVERHANG_MM,
 *  S5_FRONT_OVERHANG_MM (mm, each with its
 *  measurement date) and S5_SIDE_LOOK_DEG (<= the validated servo limit;
 *  checked at compile time in task_obstacle_scan.c). */
#ifndef S5_SIDE_PASS
#define S5_SIDE_PASS 0 /* 0 = fixed-depth pass as before */
#endif

/* Extra gap (mm) kept between the robot and the obstacle's rear corner at
   turn-in.  Approved by the owner 2026-11-24 at 30 mm.  This is the ONE
   place to change it if the robot clips the corner (larger = turns in
   later) or drives on too far (smaller).  The build fails outside 0..100. */
#ifndef S5_PIVOT_MARGIN_MM
#define S5_PIVOT_MARGIN_MM 30
#endif
_Static_assert((S5_PIVOT_MARGIN_MM >= 0) && (S5_PIVOT_MARGIN_MM <= 100),
               "S5_PIVOT_MARGIN_MM must be 0..100 mm");
/* Robot dimensions for the measured pass, in mm, from the owner.  Measure
   again and edit here if the robot is rebuilt.
   S5_SERVO_AHEAD_MM   110  axle centre to servo shaft centre, owner 2026-11-24
   S5_SERVO_ARM_MM      30  servo shaft to sensor midpoint, owner 2026-11-24
   S5_REAR_OVERHANG_MM  50  axle centre to frame rear edge, measured 2026-11-24
                            (ruler re-check to +-5 mm still requested)
   S5_FRONT_OVERHANG_MM 140 axle centre to frontmost point with bracket:
                            PHOTO ESTIMATE 2026-11-24, ruler check pending
   S5_SIDE_LOOK_DEG     60  the servo's validated limit; angle still to be
                            confirmed by the probe test (S5M-01)
   Half-width stays S5_ROBOT_HALF_WIDTH_MM (80); 65 mm (tyres) only if the
   body and bracket are not wider (owner to check). */
#ifndef S5_SERVO_AHEAD_MM
#define S5_SERVO_AHEAD_MM 110
#endif
#ifndef S5_SERVO_ARM_MM
#define S5_SERVO_ARM_MM 30
#endif
#ifndef S5_REAR_OVERHANG_MM
#define S5_REAR_OVERHANG_MM 50
#endif
#ifndef S5_FRONT_OVERHANG_MM
#define S5_FRONT_OVERHANG_MM 140
#endif
#ifndef S5_SIDE_LOOK_DEG
#define S5_SIDE_LOOK_DEG 60
#endif
_Static_assert((S5_SERVO_AHEAD_MM > 0) && (S5_SERVO_AHEAD_MM <= 250) &&
               (S5_SERVO_ARM_MM >= 0) && (S5_SERVO_ARM_MM <= 100) &&
               (S5_REAR_OVERHANG_MM > 0) && (S5_REAR_OVERHANG_MM <= 250) &&
               (S5_FRONT_OVERHANG_MM > 0) && (S5_FRONT_OVERHANG_MM <= 300) &&
               (S5_SIDE_LOOK_DEG > 0) && (S5_SIDE_LOOK_DEG <= 90),
               "robot dimension outside its plausible range (mm)");

#if S5_SIDE_PASS
#if !defined(S5_SERVO_AHEAD_MM) || !defined(S5_SERVO_ARM_MM) || \
    !defined(S5_REAR_OVERHANG_MM) || \
    !defined(S5_FRONT_OVERHANG_MM) || \
    !defined(S5_SIDE_LOOK_DEG)
#error "S5_SIDE_PASS needs the owner's measured robot dimensions (see obstacle_logic.h)"
#endif
#endif
#define S5_SIDE_WINDOW_PCT   250  /* accept echoes up to this % of the planned gap */
#define S5_SIDE_BEAM_HALF_DEG 15  /* (inferred) ultrasonic half beam angle */
#define S5_SIDE_GUARD_MM     10   /* stop when closer than half-width + this       */
#define S5_SIDE_FOUND_COUNT  3    /* distinct readings in a row that start "alongside" */
#define S5_SIDE_LOST_COUNT   3    /* distinct missing readings in a row ... */
#define S5_SIDE_LOST_MM      20   /* ... spanning this much travel end it   */
#define S5_SIDE_FAULT_COUNT  5    /* sensor failures in a row = sensor fault */
#define S5_SIDE_PARTIAL_MM   30   /* echo this early: depth is only a lower bound */
#define S5_PASS_CAP_MM       1000 /* default cap; the owner confirms (open question) */
#define S5_SIDE_ARM_TMO_MS   2000 /* wait for the servo to point and ping    */

/* Robot dimensions the tracker needs (mm).  Built from the owner's
   constants in firmware; host tests pass their own fixture values. */
typedef struct
{
    int32_t servo_ahead_mm;  /* axle to the servo shaft, along the robot */
    int32_t arm_mm;          /* servo shaft to the sensor midpoint       */
    int32_t rear_mm;         /* rear of the body behind the axle   */
    int32_t front_mm;        /* front of the body ahead of the axle */
    int32_t half_width_mm;
    int32_t margin_mm;       /* extra clearance at turn-in         */
    int32_t return_turn_deg; /* the inward pivot angle             */
} s5_geom_t;

/* ---- Bypass manoeuvre ------------------------------------------- */
#define S5_BYPASS_TURN_DEG   35
#define S5_BYPASS_FIRST_MM   80 /* short first diagonal before moving farther sideways */
#define S5_BYPASS_CLEARANCE_DEG 45 /* heading for the rest of the side clearance */
#define S5_RETURN_TURN_DEG   45
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
    /* Measured pass (S5_SIDE_PASS).  m.distance_mm stays the fixed-depth
       distance: it is the fallback when the side echo never appears. */
    uint8_t         side_track;  /* 1 = look sideways and end by measurement */
    int8_t          look_deg;    /* servo angle, + = left (towards the obstacle) */
    uint16_t        cap_mm;      /* rear corner must be seen before this travel */
    uint16_t        request_mm;  /* distance actually requested from motion */
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

/* ---- Measured pass: pure tracker (host tested) ---- */
typedef enum
{
    S5_PASS_SEEKING = 0, /* no side echo yet                           */
    S5_PASS_ALONGSIDE,   /* the obstacle's side face is in view        */
    S5_PASS_PASSED,      /* rear corner seen, driving on to clear it   */
    S5_PASS_CLEARED,     /* DONE: body and pivot are past the rear corner */
    S5_PASS_FALLBACK,    /* DONE: no echo ever, fixed depth used (clearance NOT certified) */
    S5_PASS_CAP,         /* FAIL: still beside it at the travel cap    */
    S5_PASS_TOO_CLOSE,   /* FAIL: gap to the obstacle below the guard  */
    S5_PASS_SENSOR_FAULT /* FAIL: repeated sensor failures             */
} s5_pass_phase_t;

/* One side reading as the scan task publishes it */
typedef enum
{
    S5_SIDE_ST_ECHO = 1, /* valid echo, mm is the range              */
    S5_SIDE_ST_NO_ECHO,  /* valid ping, nothing came back            */
    S5_SIDE_ST_FAULT     /* the sensor could not make the measurement */
} s5_side_status_t;

typedef struct
{
    uint32_t seq;    /* grows by one per published reading; 0 = none yet */
    uint8_t  status; /* s5_side_status_t                                  */
    uint16_t mm;
} s5_side_reading_t;

typedef struct
{
    s5_geom_t geom;
    uint8_t   phase;       /* s5_pass_phase_t                         */
    uint8_t   partial;     /* 1 = echo at the very start: depth is a lower bound */
    int32_t   look_deg;    /* angle magnitude, 1..90                  */
    int32_t   fallback_mm; /* fixed-depth pass distance (travel since the pass began) */
    int32_t   cap_mm;
    uint32_t  last_seq;
    int32_t   hits;        /* consecutive accepted echoes             */
    int32_t   misses;      /* consecutive missing echoes while alongside */
    int32_t   faults;
    int32_t   run_start_mm;
    int32_t   run_range_mm;
    int32_t   first_mm;    /* travel at the first confirmed echo (start of the debounce run) */
    int32_t   first_range_mm;
    int32_t   last_mm;     /* travel at the last confirmed echo (NOT at the later decision) */
    int32_t   last_range_mm;
    int32_t   min_gap_mm;  /* smallest sensor-to-face gap seen (inferred: sensor on the centre line) */
    int32_t   rear_mm;     /* rear corner, along the line, axle frame */
    int32_t   target_mm;   /* axle travel at which the turn-in starts */
    int32_t   depth_mm;    /* observed span along the line            */
    int32_t   decided_mm;  /* travel when the rear was declared        */
} s5_pass_t;

#if S5_SIDE_PASS
extern void s5_geom_from_config(s5_geom_t * p_geom); /* owner's constants */
#endif

/* Sensor position for a look angle: forward of the axle, and sideways of
   the centre line towards the obstacle (sensor on the centre line, inferred). */
extern int32_t s5_sensor_along_mm(const s5_geom_t * p_geom, int32_t look_deg);
extern int32_t s5_sensor_lateral_mm(const s5_geom_t * p_geom, int32_t look_deg);

/* Largest range accepted as "the obstacle side" for this look angle. */
extern int32_t s5_side_window_mm(int32_t look_deg);

/* Most travel after the last echo that the clearing drive can need. */
extern int32_t s5_pass_tail_max_mm(const s5_geom_t * p_geom, int32_t look_deg);

/* Smallest (axle - rear corner) distance at which pivoting inward by
   turn_deg cannot swing any body corner into an obstacle face that is
   gap_mm from the centre line.  Pure geometry. */
extern int32_t s5_pivot_min_axle_mm(const s5_geom_t * p_geom, int32_t gap_mm);

extern void s5_pass_init(s5_pass_t * p_pass, const s5_geom_t * p_geom,
                         int32_t look_deg, int32_t fallback_mm, int32_t cap_mm);

/* Feed one poll.  travel_mm = forward travel since the pass began.
   p_reading may be NULL or repeat an earlier seq (not counted twice);
   an older seq is ignored.  Returns the phase; SEEKING, ALONGSIDE and
   PASSED are running, the others are final. */
extern s5_pass_phase_t s5_pass_update(s5_pass_t * p_pass, int32_t travel_mm,
                                      const s5_side_reading_t * p_reading);

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
