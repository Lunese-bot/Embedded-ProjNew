/*
 *	subsystem6.h
 *	Subsystem 6 -- Vehicle Controller / Mission Control.
 *	Student ID: <student ID>.
 *
 *	Public interface of subsystem 6.  Other subsystems include it as
 *	"../subsystem6/subsystem6.h".
 *
 *	The Vehicle Controller owns the mission state (design review 5, 6.1)
 *	and is the ONLY source of movement commands.  It drives no hardware.
 *
 *	How other subsystems use it
 *	---------------------------
 *	  IF-06  any -> S6  report an event:
 *	           vc_post_event(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0, TMO_POL);
 *	         ask to move (forwarded only if you own motion right now):
 *	           vc_post_event(VC_SRC_LINE, VC_EV_MOVE_REQUEST,
 *	                         VC_MOVE_STEER, -250, TMO_POL);
 *	         report a fault (FR8):
 *	           vc_post_event(VC_SRC_IMU, VC_EV_FAULT,
 *	                         VC_FAULT_CRITICAL, 0x4001, TMO_POL);
 *	  IF-07  S6 -> any  read the mission state:
 *	           VC_STATUS s; vc_get_status(&s);
 *	         or wait for a state:
 *	           tk_wai_flg(flg_vc_state, VC_FLG_RUNNING, TWF_ORW, &p, TMO_FEVR);
 *
 *	Event and command formats are in vc_types.h.
 */

#ifndef SUBSYSTEM6_H
#define SUBSYSTEM6_H

#include "../app.h"
#include "vc_types.h"

/* Priority band 10-11: ABOVE the five sensing/actuating subsystems
   (design review section 3: Vehicle Controller = High, event-driven).
   PROPOSED -- agree it in TEAM.md section 4. */
#define S6_PRI_VC		10

#define S6_TICK_MS		10	/* loop period when idle: S3 steering rate (NFR2) */
#define S6_TELEMETRY_MS		200	/* status to S1 (NFR2 MQTT period) */
#define S6_EVENT_QUEUE_LEN	16	/* events buffered in mbf_vc_events */

/* IF-06: events in (owned by S6).  Use vc_post_event(). */
IMPORT ID	mbf_vc_events;

/* IF-07: status out (owned by S6).  Use vc_get_status(). */
IMPORT ID	mtx_vc_status;

/* IF-07: one bit per mission state, exactly one set at a time, plus
   VC_FLG_RUNNING while a mission is in progress. */
IMPORT ID	flg_vc_state;
#define VC_FLG_STATE(st)	(1U << (UINT)(st))
#define VC_FLG_RUNNING		(1U << 16)

/* Send one event to the controller.  Call from TASKS only, not from
   interrupt or cyclic handlers (post from your task instead).  Use
   TMO_POL in time-critical tasks: returns E_TMOUT if the queue is full. */
IMPORT ER vc_post_event(VC_SOURCE src, VC_EVENT_TYPE type, W p1, W p2, TMO tmo);

/* Copy the current controller status.  Never blocks for long. */
IMPORT ER vc_get_status(VC_STATUS *out);

/* Private to subsystem 6 */
IMPORT ID	tid_vc;
IMPORT void	vehicle_ctrl_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM6_H */
