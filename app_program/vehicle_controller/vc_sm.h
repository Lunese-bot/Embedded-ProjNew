/*
 *      vc_sm.h
 *      Subsystem 6 -- Vehicle Controller mission state machine.
 *
 *      Pure logic: vc_sm_handle() takes one event and returns a list of
 *      ACTIONS (move, request, report).  It never calls the kernel or a
 *      driver, so the same code runs in the controller task, in the
 *      on-target tests (on a private vc_sm_t, motors untouched) and on a PC.
 *      The task (task_vehicle_ctrl.c) carries the actions out through
 *      vc_ports.c.
 */

#ifndef VC_SM_H
#define VC_SM_H

#include "vc_types.h"

/* ---------------------------------------------------------------------
 *  Tuning.  Preliminary values: validate on the track (NFR2, NFR5,
 *  design review 8.3 "recovery limits ... need validation").
 * --------------------------------------------------------------------- */
#define VC_SPEED_CRUISE_MMS 120 /* normal line following (tuned 2026-11-04: was 200 at 60%) */
#define VC_SPEED_SLOW_MMS   60  /* barcode reading, humps (scaled with cruise) */
/* Slow down in curves: forward speed is cut by up to this much (permille)
   at full steering, in proportion to how hard S3 steers.  400 = at a full
   turn the robot drives at 60 % of cruise; a small nudge barely slows it.
   Never below VC_CURVE_MIN_MMS.  TUNE (live on the dashboard). */
#define VC_CURVE_SLOW_PERMILLE 600 /* tuned 2026-11-04: U-turn works at 60 % */
#define VC_CURVE_MIN_MMS       60
#define VC_TURN_DEG         90  /* barcode LEFT / RIGHT */
#define VC_SEARCH_TURN_DEG  30  /* LOST_LINE attempt 1 (attempt 2: x2) */
#define VC_SEARCH_BACK_MM   50  /* LOST_LINE attempt 3: reverse */

#define VC_SELF_CHECK_MS     2000  /* MISSION_START: wait for READY */
#define VC_LINE_ATTEMPT_MS   1500  /* LOST_LINE: one search attempt */
#define VC_LINE_MAX_ATTEMPTS 3     /* then FAULT (bounded recovery) */
/* DECODE_BARCODE: one read.  Was 1500, but a whole read takes far longer:
   big-print *A* is ~291 mm long, so at 60 mm/s it takes ~4.9 s to cross,
   plus the end-of-barcode quiet zone (5 x widest bar = ~1.6 s).  After
   1500 ms the retry re-armed barcode_task, which threw away every bar read
   so far (2026-11-08: cause of "too few elements").  8000 covers big *A*
   at 60 mm/s with margin; S3 still ends a read itself on its quiet zone. */
#define VC_DECODE_MS         12000 /* big print *A* is 291 mm = 8.1 s at 36 mm/s (60 % speed); was 8000 */
/* Was 2.  A retry cannot work while driving forward: after a failed read the
   barcode is already behind the robot, so each "retry" only held the robot
   in DECODE_BARCODE (slow, junctions ignored) for another VC_DECODE_MS.
   0 = one read, then report it and keep following (review 2026-11-10). */
#define VC_BARCODE_MAX_RETRY 0     /* then give up, keep following */
#define VC_EXECUTE_MS        6000  /* EXECUTE_ACTION: advance + turn (was 4000) */
#define VC_JUNCTION_STOP_MS  2000  /* wait for S2 to confirm zero speed */
/* Stand still this long at a junction before the turn / drive-on, so the
   stop is a clear, visible pause (2026-11-11).  0 = act as soon as S2
   confirms zero speed. */
#ifndef VC_JUNCTION_HOLD_MS
#define VC_JUNCTION_HOLD_MS  1000
#endif
#define VC_JUNCTION_CLEAR_MM 60    /* straight distance to clear the crossing; TUNE */
/* LEFT / RIGHT at a junction: drive this far first, so the AXLE (not the
   sensors) is over the crossing when the robot pivots.  See
   ROBOT_IR_TO_AXLE_MM in robot_types.h.  0 = turn at once. */
#ifndef VC_JUNCTION_ADVANCE_MM
#define VC_JUNCTION_ADVANCE_MM ROBOT_IR_TO_AXLE_MM
#endif
#define VC_TERRAIN_MS        5000  /* TERRAIN_CHECK: hump must clear */
/* Was 15000, shorter than a normal bypass: scan (up to ~5 s) + 4 turns +
   3 legs (~9 s) + up to 3 line recoveries (~9 s).  The controller faulted
   while S5 was still searching for the line (review 2026-11-10). */
#define VC_AVOID_MS          45000 /* AVOID_OBSTACLE: whole bypass */

/* ---------------------------------------------------------------------
 *  Actions produced by the state machine
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_ACT_MOVE = 1, /* p1 = vc_move_t, p2 = value          -> S2 */
    VC_ACT_REQUEST,  /* target = vc_source_t (0 = all), p1 = vc_request_t, p2 =
                        arg */
    VC_ACT_REPORT,   /* target = source of fault, p1 = severity, p2 = code -> S1
                      */
    VC_ACT_STATE     /* p1 = new vc_state_t, p2 = old vc_state_t */
} vc_action_kind_t;

typedef struct
{
    uint8_t kind;   /* vc_action_kind_t */
    uint8_t target; /* vc_source_t, 0 = broadcast */
    int16_t rsv;
    int32_t p1;
    int32_t p2;
} vc_action_t;

#define VC_MAX_ACTIONS 12

typedef struct
{
    int32_t     n;
    bool        overflow; /* more than VC_MAX_ACTIONS: a bug */
    vc_action_t a[VC_MAX_ACTIONS];
} vc_actions_t;

/* The whole controller state.  One instance lives in the task. */
typedef struct
{
    vc_status_t          st;
    uint32_t             ready_mask;       /* VC_READY_BIT()s seen */
    uint32_t             attempt_since_ms; /* LOST_LINE / DECODE timer */
    uint8_t              attempts;         /* LOST_LINE search attempts */
    uint8_t              retries;          /* barcode read retries */
    uint32_t             junction_stop_ms; /* JUNCTION_STOP: when S2 confirmed the stop */
    uint8_t              junction_stopped; /* JUNCTION_STOP: 1 = standing, holding */
    uint8_t              exec_phase;       /* EXECUTE_ACTION: 1 = advancing before the turn */
    int8_t               lost_side;        /* -1 left, +1 right, 0 unknown */
    navigation_command_t pending_nav;      /* decoded action for next junction */
    char                 pending_barcode[16]; /* full decoded barcode text */
} vc_sm_t;

extern void vc_sm_init(vc_sm_t * p_sm, uint32_t now_ms);
extern void vc_sm_handle(vc_sm_t * p_sm, const vc_event_t * p_event,
                         uint32_t now_ms, vc_actions_t * p_out);

/* Which subsystem may move the robot in a state (VC_SRC_NONE = only the
   controller's own scripted moves) */
extern vc_source_t vc_sm_motion_owner(vc_state_t state);
extern bool        vc_sm_is_active(vc_state_t state);

extern const char * vc_state_name(vc_state_t state);
extern const char * vc_event_name(vc_event_type_t type);

#endif /* VC_SM_H */

/*** end of file ***/
