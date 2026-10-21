/*
 *	subsystem6.c
 *	Subsystem 6 -- Vehicle Controller: creates its objects, starts its
 *	task, and provides the public calls of subsystem6.h.
 *
 *	Kernel objects (TEAM.md section 6):
 *	  1 task            vehicle_ctrl_task
 *	  1 message buffer  mbf_vc_events   IF-06 events in
 *	  1 mutex           mtx_vc_status   IF-07 status snapshot
 *	  1 event flag      flg_vc_state    IF-07 current state bits
 */

#include "subsystem6.h"

EXPORT ID	mbf_vc_events;
EXPORT ID	mtx_vc_status;
EXPORT ID	flg_vc_state;
EXPORT ID	tid_vc;

/* Written only by vehicle_ctrl_task, under mtx_vc_status */
EXPORT VC_STATUS vc_status_shared;

EXPORT ER subsystem6_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};

	/* Message buffer: events are COPIED in, so senders need no memory
	   pool and can send from any task on either core. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S6_EVENT_QUEUE_LEN * (sizeof(VC_EVENT) + sizeof(INT));
	cmbf.maxmsz = sizeof(VC_EVENT);
	mbf_vc_events = tk_cre_mbf(&cmbf);
	if (mbf_vc_events < E_OK) {
		return mbf_vc_events;
	}

	/* Mutex with priority inheritance: a low-priority reader (S1)
	   cannot hold up the controller. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_vc_status = tk_cre_mtx(&cmtx);
	if (mtx_vc_status < E_OK) {
		return mtx_vc_status;
	}

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = VC_FLG_STATE(VC_ST_IDLE);
	flg_vc_state = tk_cre_flg(&cflg);
	if (flg_vc_state < E_OK) {
		return flg_vc_state;
	}

	vc_status_shared.state    = VC_ST_IDLE;
	vc_status_shared.last_nav = NAV_NONE;
	return E_OK;
}

EXPORT ER subsystem6_start(void)
{
	/* No affinity: event-driven, short, either core is fine */
	tid_vc = app_start_task(vehicle_ctrl_task, S6_PRI_VC, 0, "vehicle_ctrl");
	return (tid_vc < E_OK) ? tid_vc : E_OK;
}

EXPORT ER vc_post_event(VC_SOURCE src, VC_EVENT_TYPE type, W p1, W p2, TMO tmo)
{
	VC_EVENT ev;
	SYSTIM	 now;

	tk_get_tim(&now);
	ev.src     = (UB)src;
	ev.type    = (UB)type;
	ev.rsv     = 0;
	ev.p1      = p1;
	ev.p2      = p2;
	ev.time_ms = now.lo;
	return tk_snd_mbf(mbf_vc_events, &ev, sizeof(ev), tmo);
}

EXPORT ER vc_get_status(VC_STATUS *out)
{
	ER er = tk_loc_mtx(mtx_vc_status, 50);	/* held for a copy only */
	if (er < E_OK) {
		return er;
	}
	*out = vc_status_shared;
	tk_unl_mtx(mtx_vc_status);
	return E_OK;
}
