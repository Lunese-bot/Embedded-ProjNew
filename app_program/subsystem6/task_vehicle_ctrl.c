/*
 *	task_vehicle_ctrl.c
 *	Subsystem 6 -- the Vehicle Controller task.
 *
 *	Loop:
 *	  1. wait for an event on mbf_vc_events, at most S6_TICK_MS, then
 *	     read the channels S2..S5 own (vc_bridge_s2..s5.c), so their
 *	     events wait at most one tick, and S3's steering is forwarded
 *	     every tick (10 ms, NFR2)
 *	  2. no event (or events arriving non-stop) -> make a TICK event,
 *	     so state time limits are still checked
 *	  3. vc_sm_handle() decides; this task carries out its actions
 *	     through vc_ports.c
 *	  4. publish the new status (mutex snapshot + state flag)
 *	  5. every S6_TELEMETRY_MS, send the status to S1
 *
 *	APIs used: tk_rcv_mbf, tk_get_tim, tk_loc_mtx/tk_unl_mtx,
 *	           tk_clr_flg/tk_set_flg
 */

#include "subsystem6.h"
#include "vc_sm.h"
#include "vc_ports.h"
#include "vc_bridge.h"

IMPORT VC_STATUS vc_status_shared;	/* subsystem6.c */

LOCAL VC_SM	 vc_sm;			/* the live controller */
LOCAL VC_ACTIONS vc_actions;

LOCAL UW now_ms(void)
{
	SYSTIM t;
	tk_get_tim(&t);
	return t.lo;
}

LOCAL void run_actions(const VC_ACTIONS *acts)
{
	INT i;

	for (i = 0; i < acts->n; i++) {
		const VC_ACTION *a = &acts->a[i];

		switch ((VC_ACTION_KIND)a->kind) {
		case VC_ACT_MOVE:
			CHECK(vc_port_move((VC_MOVE)a->p1, a->p2), "vc_port_move");
			break;
		case VC_ACT_REQUEST:
			CHECK(vc_port_request((VC_SOURCE)a->target,
					      (VC_REQUEST)a->p1, a->p2),
			      "vc_port_request");
			break;
		case VC_ACT_REPORT:
			CHECK(vc_port_report((VC_SOURCE)a->target,
					     (VC_FAULT_SEVERITY)a->p1, (UW)a->p2),
			      "vc_port_report");
			break;
		case VC_ACT_STATE:
			tm_printf((UB*)"vc: %s -> %s\n",
				  vc_state_name((VC_STATE)a->p2),
				  vc_state_name((VC_STATE)a->p1));
			break;
		default:
			break;
		}
	}
	if (acts->overflow) {
		/* More actions than fit: a bug in vc_sm.c.  Fail safe. */
		tm_printf((UB*)"vc: ERROR action overflow, stopping\n");
		vc_port_move(VC_MOVE_STOP, 0);
	}
}

LOCAL void publish_status(BOOL state_changed)
{
	UINT ptn;

	if (tk_loc_mtx(mtx_vc_status, TMO_FEVR) == E_OK) {
		vc_status_shared = vc_sm.st;
		tk_unl_mtx(mtx_vc_status);
	}

	if (state_changed) {
		ptn = VC_FLG_STATE(vc_sm.st.state);
		if (vc_sm_is_active(vc_sm.st.state)) {
			ptn |= VC_FLG_RUNNING;
		}
		tk_clr_flg(flg_vc_state, 0);		/* clear all bits */
		tk_set_flg(flg_vc_state, ptn);
	}
}

/* One event through the state machine, then out to the ports */
LOCAL void handle(const VC_EVENT *ev, UW t, BOOL from_s5)
{
	VC_STATE before   = vc_sm.st.state;
	UW	 rejected = vc_sm.st.moves_rejected;

	vc_sm_handle(&vc_sm, ev, t, &vc_actions);
	run_actions(&vc_actions);

	if (from_s5) {
		vc_bridge_s5_after(ev, vc_sm.st.state,
				   vc_sm.st.moves_rejected != rejected);
	}
	if (ev->type == VC_EV_MANOEUVRE_DONE && before == VC_ST_AVOID_OBSTACLE) {
		vc_bridge_s5_manoeuvre_done(ev->p1 != 0);
	}
	publish_status(before != vc_sm.st.state);
}

EXPORT void vehicle_ctrl_task(INT stacd, void *exinf)
{
	VC_EVENT ev;
	VC_EVENT tick = {0};
	INT	 len;
	UW	 t, last_tick, last_telem;

	(void)stacd; (void)exinf;

	t = now_ms();
	vc_sm_init(&vc_sm, t);
	last_tick = last_telem = t;
	tick.src  = VC_SRC_VC;
	tick.type = VC_EV_TICK;
	publish_status(TRUE);
	tm_printf((UB*)"vc: task %d ready, state %s (send VC_EV_CMD_START)\n",
		  tk_get_tid(), vc_state_name(vc_sm.st.state));

	while (1) {
		/* 1. next event, or time out after one tick */
		len = tk_rcv_mbf(mbf_vc_events, &ev, S6_TICK_MS);
		t = now_ms();

		if (len == (INT)sizeof(ev)) {
			handle(&ev, t, FALSE);
		} else if (len >= 0) {
			tm_printf((UB*)"vc: bad event size %d\n", len);
		} else if (len != E_TMOUT) {
			CHECK(len, "vc tk_rcv_mbf");
		}

		while (vc_bridge_s2_next(&ev)) {
			handle(&ev, t, FALSE);
		}
		while (vc_bridge_s3_next(&ev, vc_sm.st.state)) {
			handle(&ev, t, FALSE);
		}
		while (vc_bridge_s4_next(&ev)) {
			handle(&ev, t, FALSE);
		}
		while (vc_bridge_s5_next(&ev)) {
			handle(&ev, t, TRUE);
		}

		/* 2. tick: also when events keep arriving faster than S6_TICK_MS */
		if (t - last_tick >= S6_TICK_MS) {
			last_tick = t;
			tick.time_ms = t;
			handle(&tick, t, FALSE);
		}

		/* 5. telemetry (low rate, S1 decides what goes over MQTT) */
		if (t - last_telem >= S6_TELEMETRY_MS) {
			last_telem = t;
			vc_port_telemetry(&vc_sm.st);
		}
	}
}
