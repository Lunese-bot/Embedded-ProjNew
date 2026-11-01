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
#define VC_SPEED_CRUISE_MMS 200 /* normal line following */
#define VC_SPEED_SLOW_MMS   100 /* barcode reading, humps */
#define VC_TURN_DEG         90  /* barcode LEFT / RIGHT */
#define VC_SEARCH_TURN_DEG  30  /* LOST_LINE attempt 1 (attempt 2: x2) */
#define VC_SEARCH_BACK_MM   50  /* LOST_LINE attempt 3: reverse */

#define VC_SELF_CHECK_MS     2000  /* MISSION_START: wait for READY */
#define VC_LINE_ATTEMPT_MS   1500  /* LOST_LINE: one search attempt */
#define VC_LINE_MAX_ATTEMPTS 3     /* then FAULT (bounded recovery) */
#define VC_DECODE_MS         1500  /* DECODE_BARCODE: one read */
#define VC_BARCODE_MAX_RETRY 2     /* then give up, keep following */
#define VC_EXECUTE_MS        4000  /* EXECUTE_ACTION: manoeuvre */
#define VC_TERRAIN_MS        5000  /* TERRAIN_CHECK: hump must clear */
#define VC_AVOID_MS          15000 /* AVOID_OBSTACLE: whole bypass */

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
    int8_t               lost_side;        /* -1 left, +1 right, 0 unknown */
    navigation_command_t pending_nav;      /* for EXECUTE_ACTION */
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
