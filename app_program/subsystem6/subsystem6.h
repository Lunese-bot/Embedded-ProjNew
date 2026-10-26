/*
 *      subsystem6.h
 *      Subsystem 6 -- Vehicle Controller / Mission Control.
 *      Student ID: <student ID>.
 *
 *      Public interface of subsystem 6.  Other subsystems include it as
 *      "../subsystem6/subsystem6.h".
 *
 *      The Vehicle Controller owns the mission state (design review 5, 6.1)
 *      and is the ONLY source of movement commands.  It drives no hardware.
 *
 *      How other subsystems use it
 *      ---------------------------
 *        IF-06  any -> S6  report an event:
 *                 vc_post_event(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0, TMO_POL);
 *               ask to move (forwarded only if you own motion right now):
 *                 vc_post_event(VC_SRC_LINE, VC_EV_MOVE_REQUEST,
 *                               VC_MOVE_STEER, -250, TMO_POL);
 *               report a fault (FR8):
 *                 vc_post_event(VC_SRC_IMU, VC_EV_FAULT,
 *                               VC_FAULT_CRITICAL, 0x4001, TMO_POL);
 *        IF-07  S6 -> any  read the mission state:
 *                 vc_status_t s; vc_get_status(&s);
 *               or wait for a state:
 *                 tk_wai_flg(g_flg_vc_state, VC_FLG_RUNNING, TWF_ORW, &p,
 * TMO_FEVR);
 *
 *      Event and command formats are in vc_types.h.
 */

#ifndef SUBSYSTEM6_H
#define SUBSYSTEM6_H

#include "../app.h"
#include "vc_types.h"

/* Priority band 10-11: ABOVE the five sensing/actuating subsystems
   (design review section 3: Vehicle Controller = High, event-driven).
   PROPOSED -- agree it in TEAM.md section 4. */
#define S6_PRI_VC 10

#define S6_TICK_MS         10 /* loop period when idle: S3 steering rate (NFR2) */
#define S6_TELEMETRY_MS    200 /* status to S1 (NFR2 MQTT period) */
#define S6_EVENT_QUEUE_LEN 16  /* events buffered in g_mbf_vc_events */

/* IF-06: events in (owned by S6).  Use vc_post_event(). */
extern ID g_mbf_vc_events;

/* IF-07: status out (owned by S6).  Use vc_get_status(). */
extern ID g_mtx_vc_status;

/* IF-07: one bit per mission state, exactly one set at a time, plus
   VC_FLG_RUNNING while a mission is in progress. */
extern ID g_flg_vc_state;
#define VC_FLG_STATE(st) (1U << (uint32_t)(st))
#define VC_FLG_RUNNING   (1U << 16)
/* VC_FLG_STATE stays a macro (BARR-C 6.3 deviation): other subsystems
   use it to build constant wait patterns for tk_wai_flg. */

/* Send one event to the controller.  Call from TASKS only, not from
   interrupt or cyclic handlers (post from your task instead).  Use
   TMO_POL in time-critical tasks: returns E_TMOUT if the queue is full. */
extern ER vc_post_event(vc_source_t src, vc_event_type_t type, int32_t param1,
                        int32_t param2, TMO tmo);

/* Copy the current controller status.  Never blocks for long. */
extern ER vc_get_status(vc_status_t * p_out);

/* Private to subsystem 6 */
extern ID          g_tid_vc;
extern vc_status_t g_vc_status_shared; /* guarded by g_mtx_vc_status */
extern void        vehicle_ctrl_task(int32_t stacd, void * p_exinf);

#endif /* SUBSYSTEM6_H */

/*** end of file ***/
