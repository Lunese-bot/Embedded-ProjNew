/*
 *	vc_sm.h
 *	Subsystem 6 -- Vehicle Controller mission state machine.
 *
 *	Pure logic: vc_sm_handle() takes one event and returns a list of
 *	ACTIONS (move, request, report).  It never calls the kernel or a
 *	driver, so the same code runs in the controller task, in the
 *	on-target tests (on a private VC_SM, motors untouched) and on a PC.
 *	The task (task_vehicle_ctrl.c) carries the actions out through
 *	vc_ports.c.
 */

#ifndef VC_SM_H
#define VC_SM_H

#include "vc_types.h"

/* ---------------------------------------------------------------------
 *  Tuning.  Preliminary values: validate on the track (NFR2, NFR5,
 *  design review 8.3 "recovery limits ... need validation").
 * --------------------------------------------------------------------- */
#define VC_SPEED_CRUISE_MMS	200	/* normal line following */
#define VC_SPEED_SLOW_MMS	100	/* barcode reading, humps */
#define VC_TURN_DEG		90	/* barcode LEFT / RIGHT */
#define VC_SEARCH_TURN_DEG	30	/* LOST_LINE attempt 1 (attempt 2: x2) */
#define VC_SEARCH_BACK_MM	50	/* LOST_LINE attempt 3: reverse */

#define VC_SELF_CHECK_MS	2000	/* MISSION_START: wait for READY */
#define VC_LINE_ATTEMPT_MS	1500	/* LOST_LINE: one search attempt */
#define VC_LINE_MAX_ATTEMPTS	3	/* then FAULT (bounded recovery) */
#define VC_DECODE_MS		1500	/* DECODE_BARCODE: one read */
#define VC_BARCODE_MAX_RETRY	2	/* then give up, keep following */
#define VC_EXECUTE_MS		4000	/* EXECUTE_ACTION: manoeuvre */
#define VC_TERRAIN_MS		5000	/* TERRAIN_CHECK: hump must clear */
#define VC_AVOID_MS		15000	/* AVOID_OBSTACLE: whole bypass */

/* ---------------------------------------------------------------------
 *  Actions produced by the state machine
 * --------------------------------------------------------------------- */
typedef enum {
	VC_ACT_MOVE = 1,	/* p1 = VC_MOVE, p2 = value          -> S2 */
	VC_ACT_REQUEST,		/* target = VC_SOURCE (0 = all), p1 = VC_REQUEST, p2 = arg */
	VC_ACT_REPORT,		/* target = source of fault, p1 = severity, p2 = code -> S1 */
	VC_ACT_STATE		/* p1 = new VC_STATE, p2 = old VC_STATE */
} VC_ACTION_KIND;

typedef struct {
	UB	kind;		/* VC_ACTION_KIND */
	UB	target;		/* VC_SOURCE, 0 = broadcast */
	H	rsv;
	W	p1;
	W	p2;
} VC_ACTION;

#define VC_MAX_ACTIONS	12

typedef struct {
	INT		n;
	BOOL		overflow;	/* more than VC_MAX_ACTIONS: a bug */
	VC_ACTION	a[VC_MAX_ACTIONS];
} VC_ACTIONS;

/* The whole controller state.  One instance lives in the task. */
typedef struct {
	VC_STATUS		st;
	UW			ready_mask;	/* VC_READY_BIT()s seen */
	UW			attempt_since_ms; /* LOST_LINE / DECODE timer */
	UB			attempts;	/* LOST_LINE search attempts */
	UB			retries;	/* barcode read retries */
	B			lost_side;	/* -1 left, +1 right, 0 unknown */
	NavigationCommand	pending_nav;	/* for EXECUTE_ACTION */
} VC_SM;

IMPORT void vc_sm_init(VC_SM *sm, UW now_ms);
IMPORT void vc_sm_handle(VC_SM *sm, const VC_EVENT *ev, UW now_ms,
			 VC_ACTIONS *out);

/* Which subsystem may move the robot in a state (VC_SRC_NONE = only the
   controller's own scripted moves) */
IMPORT VC_SOURCE vc_sm_motion_owner(VC_STATE state);
IMPORT BOOL	 vc_sm_is_active(VC_STATE state);

IMPORT const char *vc_state_name(VC_STATE state);
IMPORT const char *vc_event_name(VC_EVENT_TYPE type);

#endif /* VC_SM_H */
