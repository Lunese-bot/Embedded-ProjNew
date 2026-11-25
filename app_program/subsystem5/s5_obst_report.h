/*
 *      s5_obst_report.h
 *      Subsystem 5 -- obstacle size report: one record per obstacle
 *      encounter, and its MQTT wire frame (topic robot/g10/obstacle/profile).
 *
 *      Plain C with no kernel calls (host-tested in testing/subsystem5 and
 *      decoded the same way by tools/dashboard.html).
 *
 *      What the numbers mean
 *        front_mm   nearest point of the obstacle in the front scan (the
 *                   profile's distance_mm).  NOT a size.
 *        width_mm   the profile's width_mm: lateral offset of the leftmost
 *                   hit minus that of the rightmost hit, each = range x
 *                   sin(servo angle) (obstacle_logic.c s5_build_profile).
 *                   It is never max range - min range.
 *        length_mm  extent along the robot's ORIGINAL direction of travel,
 *                   observed with the side-looking ultrasonic during the
 *                   straight pass leg only (s5_pass_t.depth_mm): forward
 *                   travel between the first and the last confirmed side
 *                   echo, each shifted by the beam reach range x cos(look
 *                   angle).  Turns, reversing and the tail drive after the
 *                   rear edge are never counted.  The fixed planning depth
 *                   (S5_OBSTACLE_DEPTH_MM, "assumed") is never put here.
 *
 *      One encounter = one START_AVOID.  It gets a new profile_id (1..65535,
 *      0 is never used) and the system time t0_ms; (t0_ms, profile_id)
 *      tells two encounters apart even across a robot restart (the id
 *      restarts at 1, t0 restarts near 0).  rev grows by one at every
 *      change of the record, so a receiver drops repeats (same id, same
 *      rev) and late copies (lower rev).
 *
 *      Wire frame, S5_OR_BYTES = 20 bytes, multi-byte fields big-endian:
 *        byte  0      version            S5_OR_VERSION (1)
 *        bytes 1-2    profile_id         1..65535
 *        byte  3      rev                0..255 (wraps; compare with id)
 *        byte  4      stage              s5_or_stage_t        0..3
 *        byte  5      width_status       s5_or_width_t        0..4
 *        byte  6      length_status      s5_or_length_t       0..4
 *        byte  7      length_reason      s5_or_reason_t       0..9
 *        bytes 8-9    front_mm           0..2000, 0 = not known
 *        bytes 10-11  width_mm           0..4000, 0 unless width_status
 *                                        is MEASURED or LOWER_BOUND
 *        bytes 12-13  length_mm          0..2000, 0 unless length_status
 *                                        is MEASURED or PARTIAL
 *        bytes 14-17  t0_ms              system time of START_AVOID
 *        byte  18     bypass_side        s5_side_t 0..2
 *        byte  19     reserved, 0
 *      A frame of another length or version, or with a field outside its
 *      range, is rejected by s5_or_decode (the dashboard does the same).
 */

#ifndef S5_OBST_REPORT_H
#define S5_OBST_REPORT_H

#include <stdint.h>
#include <stdbool.h>

#define S5_OR_TOPIC   "obstacle/profile" /* after S1_TOPIC_ROOT "robot/g10/" */
#define S5_OR_VERSION 1U
#define S5_OR_BYTES   20

#define S5_OR_FRONT_MAX_MM  2000
#define S5_OR_WIDTH_MAX_MM  4000
#define S5_OR_LENGTH_MAX_MM 2000

typedef enum
{
    S5_OR_STAGE_SCANNING = 0, /* encounter started, no profile yet      */
    S5_OR_STAGE_FRONT,        /* front profile ready (width known)      */
    S5_OR_STAGE_LENGTH,       /* length resolved, bypass still running  */
    S5_OR_STAGE_FINAL         /* bypass ended (success, failure, abort) */
} s5_or_stage_t;
#define S5_OR_STAGE_MAX S5_OR_STAGE_FINAL

typedef enum
{
    S5_OR_W_PENDING = 0,  /* scan not finished                          */
    S5_OR_W_MEASURED,     /* both edges inside the scan                 */
    S5_OR_W_LOWER_BOUND,  /* an edge is at the scan limit: wider than this */
    S5_OR_W_ONE_BEAM,     /* only one angle hit it: edges not located   */
    S5_OR_W_NONE          /* no obstacle in the scan, or no profile     */
} s5_or_width_t;
#define S5_OR_W_MAX S5_OR_W_NONE

typedef enum
{
    S5_OR_L_PENDING = 0,  /* not started (scanning)                     */
    S5_OR_L_MEASURING,    /* side look running / waiting for the pass   */
    S5_OR_L_MEASURED,     /* front and rear edge both observed          */
    S5_OR_L_PARTIAL,      /* already beside it when observation began:
                             length_mm is a lower bound                 */
    S5_OR_L_UNAVAILABLE   /* see length_reason                          */
} s5_or_length_t;
#define S5_OR_L_MAX S5_OR_L_UNAVAILABLE

typedef enum
{
    S5_OR_R_NONE = 0,
    S5_OR_R_OBSERVE_OFF,  /* build without the side look (S5_LENGTH_OBSERVE 0) */
    S5_OR_R_NO_BYPASS,    /* no profile, no obstacle, or no safe side   */
    S5_OR_R_NO_SIDE_ECHO, /* the pass leg ended without a side echo     */
    S5_OR_R_REAR_NOT_SEEN,/* still beside it when the pass leg ended    */
    S5_OR_R_SENSOR_FAULT, /* repeated failed side readings              */
    S5_OR_R_LOOK_FAILED,  /* the side look never produced a reading     */
    S5_OR_R_ABORTED,      /* ABORT / DISABLE / stop before it resolved  */
    S5_OR_R_BYPASS_FAILED,/* a motion step failed or timed out first    */
    S5_OR_R_GEOMETRY      /* look angle unsuitable for a side echo      */
} s5_or_reason_t;
#define S5_OR_R_MAX S5_OR_R_GEOMETRY

typedef struct
{
    uint16_t profile_id;    /* 0 = no encounter since boot */
    uint8_t  rev;
    uint8_t  stage;         /* s5_or_stage_t  */
    uint8_t  width_status;  /* s5_or_width_t  */
    uint8_t  length_status; /* s5_or_length_t */
    uint8_t  length_reason; /* s5_or_reason_t */
    uint8_t  bypass_side;   /* s5_side_t      */
    uint16_t front_mm;
    uint16_t width_mm;
    uint16_t length_mm;
    uint32_t t0_ms;
} s5_obst_report_t;

/* Next profile id after prev (1..65535, skips 0). */
extern uint16_t s5_or_next_id(uint16_t prev);

/* New encounter: every size field cleared, stage SCANNING, rev 0.  The
   previous encounter's numbers can never survive into the new one. */
extern void s5_or_begin(s5_obst_report_t * p_rec, uint16_t id, uint32_t t0_ms);

/* Front profile ready.  valid = 0: no obstacle (width NONE).  The scan
   covered servo angles scan_min_deg..scan_max_deg (+ = left); an edge at
   either limit makes the width a lower bound.  b_length_follows = a bypass
   will run with the side look, so length becomes MEASURING; otherwise
   UNAVAILABLE with reason no_length_reason. */
extern void s5_or_front(s5_obst_report_t * p_rec, uint8_t valid,
                        uint16_t distance_mm, int32_t right_edge_deg,
                        int32_t left_edge_deg, uint16_t width_mm,
                        uint8_t bypass_side, int32_t scan_min_deg,
                        int32_t scan_max_deg, bool b_length_follows,
                        s5_or_reason_t no_length_reason);

/* Length resolved while the bypass continues (stage LENGTH).  status is
   MEASURED, PARTIAL or UNAVAILABLE; length_mm is kept only for the first two. */
extern void s5_or_length(s5_obst_report_t * p_rec, s5_or_length_t status,
                         int32_t length_mm, s5_or_reason_t reason);

/* Bypass over (stage FINAL).  A length still PENDING / MEASURING becomes
   UNAVAILABLE with reason why_unresolved. */
extern void s5_or_final(s5_obst_report_t * p_rec, s5_or_reason_t why_unresolved);

extern void s5_or_encode(const s5_obst_report_t * p_rec, uint8_t out[S5_OR_BYTES]);

/* Returns true and fills *p_rec only for a well-formed frame. */
extern bool s5_or_decode(const uint8_t * p_in, int32_t len, s5_obst_report_t * p_rec);

/* Short names for the USB line and the tests. */
extern const char * s5_or_stage_name(uint8_t stage);
extern const char * s5_or_width_name(uint8_t status);
extern const char * s5_or_length_name(uint8_t status);
extern const char * s5_or_reason_name(uint8_t reason);

#endif /* S5_OBST_REPORT_H */

/*** end of file ***/
