/*
 *	vc_sm.c
 *	Subsystem 6 -- Vehicle Controller mission state machine.
 *
 *	Implements the "Vehicle controller" statechart of the design review
 *	(sections 5 and 6.1) and the fault rules of FR8 / NFR5 / NFR8:
 *
 *	  IDLE --start--> MISSION_START --all READY--> FOLLOW_LINE (hub)
 *	  FOLLOW_LINE --line_lost-------> LOST_LINE      --found--> FOLLOW_LINE
 *	              --obstacle--------> AVOID_OBSTACLE --done---> FOLLOW_LINE
 *	              --barcode_start---> DECODE_BARCODE --decoded-> EXECUTE_ACTION
 *	                                                 --manoeuvre_done--> FOLLOW_LINE
 *	              --hump_start------> TERRAIN_CHECK  --cleared-> FOLLOW_LINE
 *	              --end_of_course---> MISSION_COMPLETE
 *	  any active state --critical fault / impact / stop--> FAULT --reset--> IDLE
 *
 *	Line-loss is ignored in DECODE_BARCODE, EXECUTE_ACTION, TERRAIN_CHECK
 *	(design review table) and AVOID_OBSTACLE (the robot leaves the line
 *	on purpose).  Every non-hub state has a time limit, so no recovery
 *	can run forever (NFR5).
 *
 *	No kernel calls in this file.
 */

#include "vc_sm.h"

/* ------------------------------------------------------------------ */
/*  Names, for console and telemetry                                   */
/* ------------------------------------------------------------------ */

LOCAL const char * const state_names[VC_ST_COUNT] = {
	"IDLE", "MISSION_START", "FOLLOW_LINE", "LOST_LINE",
	"AVOID_OBSTACLE", "DECODE_BARCODE", "EXECUTE_ACTION",
	"TERRAIN_CHECK", "FAULT", "MISSION_COMPLETE"
};

LOCAL const char * const event_names[VC_EV_COUNT] = {
	"NONE", "READY", "MOVE_REQUEST", "FAULT",
	"CMD_START", "CMD_STOP", "CMD_RESET", "CMD_PAUSE", "CMD_RESUME",
	"WIFI_DOWN", "WIFI_UP",
	"MANOEUVRE_DONE",
	"LINE_LOST", "LINE_FOUND", "JUNCTION", "BARCODE_START",
	"BARCODE_DECODED", "BARCODE_FAILED", "END_OF_COURSE",
	"HUMP_START", "HUMP_CLEARED", "IMPACT",
	"OBSTACLE", "AVOID_DONE", "AVOID_FAILED",
	"TICK"
};

EXPORT const char *vc_state_name(VC_STATE state)
{
	return ((UINT)state < (UINT)VC_ST_COUNT) ? state_names[state] : "?";
}

EXPORT const char *vc_event_name(VC_EVENT_TYPE type)
{
	return ((UINT)type < (UINT)VC_EV_COUNT) ? event_names[type] : "?";
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

EXPORT BOOL vc_sm_is_active(VC_STATE state)
{
	return (state >= VC_ST_MISSION_START) && (state <= VC_ST_TERRAIN_CHECK);
}

EXPORT VC_SOURCE vc_sm_motion_owner(VC_STATE state)
{
	switch (state) {
	case VC_ST_FOLLOW_LINE:
	case VC_ST_DECODE_BARCODE:	/* S3 keeps steering, speed capped */
	case VC_ST_TERRAIN_CHECK:	/* S3 keeps steering, speed capped */
		return VC_SRC_LINE;
	case VC_ST_AVOID_OBSTACLE:
		return VC_SRC_OBST;
	default:
		/* LOST_LINE: the controller runs the search itself */
		return VC_SRC_NONE;	/* controller's own moves only */
	}
}

LOCAL void emit(VC_ACTIONS *out, VC_ACTION_KIND kind, VC_SOURCE target,
		W p1, W p2)
{
	VC_ACTION *a;

	if (out->n >= VC_MAX_ACTIONS) {
		out->overflow = TRUE;
		return;
	}
	a = &out->a[out->n++];
	a->kind   = (UB)kind;
	a->target = (UB)target;
	a->rsv    = 0;
	a->p1     = p1;
	a->p2     = p2;
}

LOCAL void move(VC_ACTIONS *out, VC_MOVE m, W value)
{
	emit(out, VC_ACT_MOVE, VC_SRC_MOTION, (W)m, value);
}

LOCAL void request(VC_ACTIONS *out, VC_SOURCE target, VC_REQUEST r, W arg)
{
	emit(out, VC_ACT_REQUEST, target, (W)r, arg);
}

LOCAL void report(VC_SM *sm, VC_ACTIONS *out, VC_SOURCE src,
		  VC_FAULT_SEVERITY sev, UW code)
{
	sm->st.last_fault_code = code;
	sm->st.last_fault_sev  = sev;
	sm->st.last_fault_src  = (UB)src;
	emit(out, VC_ACT_REPORT, src, (W)sev, (W)code);
}

LOCAL UW elapsed(UW now_ms, UW since_ms)
{
	return now_ms - since_ms;	/* unsigned: wraps correctly */
}

/* ------------------------------------------------------------------ */
/*  State entry actions                                                */
/* ------------------------------------------------------------------ */

LOCAL void enter(VC_SM *sm, VC_STATE next, UW now_ms, VC_ACTIONS *out);

/* LOST_LINE search pattern, one step per attempt (bounded, NFR5):
     1. turn towards the side the line was last seen
     2. turn twice as far the other way
     3. reverse a little, back over where the line was   */
LOCAL void search_step(VC_SM *sm, VC_ACTIONS *out)
{
	BOOL left_first = (sm->lost_side <= 0);	/* unknown: try left */

	switch (sm->attempts) {
	case 1:
		move(out, left_first ? VC_MOVE_TURN_LEFT : VC_MOVE_TURN_RIGHT,
		     VC_SEARCH_TURN_DEG);
		break;
	case 2:
		move(out, left_first ? VC_MOVE_TURN_RIGHT : VC_MOVE_TURN_LEFT,
		     2 * VC_SEARCH_TURN_DEG);
		break;
	default:
		move(out, VC_MOVE_REVERSE, VC_SEARCH_BACK_MM);
		break;
	}
	request(out, VC_SRC_LINE, VC_REQ_LINE_SEARCH, (W)sm->attempts);
}

LOCAL void enter_execute(VC_SM *sm, UW now_ms, VC_ACTIONS *out)
{
	switch (sm->pending_nav) {
	case NAV_LEFT:
		move(out, VC_MOVE_TURN_LEFT, VC_TURN_DEG);
		break;
	case NAV_RIGHT:
		move(out, VC_MOVE_TURN_RIGHT, VC_TURN_DEG);
		break;
	case NAV_U_TURN:
		move(out, VC_MOVE_U_TURN, 0);
		break;
	case NAV_STRAIGHT:
	case NAV_NONE:
	default:
		/* nothing to do: carry straight on along the line */
		enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		break;
	}
}

LOCAL void enter(VC_SM *sm, VC_STATE next, UW now_ms, VC_ACTIONS *out)
{
	VC_STATE prev = sm->st.state;

	sm->st.prev_state     = prev;
	sm->st.state          = next;
	sm->st.paused         = FALSE;
	sm->st.state_since_ms = now_ms;
	sm->attempt_since_ms  = now_ms;
	emit(out, VC_ACT_STATE, VC_SRC_VC, (W)next, (W)prev);

	switch (next) {
	case VC_ST_IDLE:
		if (prev == VC_ST_FAULT) {
			move(out, VC_MOVE_RESET, 0);	/* clear S2's latch */
		}
		move(out, VC_MOVE_STOP, 0);
		break;

	case VC_ST_MISSION_START:
		move(out, VC_MOVE_STOP, 0);
		sm->ready_mask          = 0;
		sm->st.mission_start_ms = now_ms;
		sm->st.max_hump_um      = 0;
		sm->st.barcodes_read    = 0;
		sm->st.last_nav         = NAV_NONE;
		request(out, VC_SRC_NONE, VC_REQ_SELF_CHECK, 0);
		break;

	case VC_ST_FOLLOW_LINE:
		sm->retries = 0;
		request(out, VC_SRC_OBST, VC_REQ_OBST_MONITOR, 0);
		request(out, VC_SRC_LINE, VC_REQ_LINE_FOLLOW, 0);
		move(out, VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS);
		move(out, VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS);
		break;

	case VC_ST_LOST_LINE:
		move(out, VC_MOVE_STOP, 0);
		move(out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
		sm->attempts = 1;
		search_step(sm, out);
		break;

	case VC_ST_AVOID_OBSTACLE:
		move(out, VC_MOVE_STOP, 0);
		request(out, VC_SRC_LINE, VC_REQ_BARCODE_OFF, 0);
		request(out, VC_SRC_OBST, VC_REQ_OBST_AVOID, 0);
		break;

	case VC_ST_DECODE_BARCODE:
		move(out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
		request(out, VC_SRC_LINE, VC_REQ_BARCODE_READ, 0);
		break;

	case VC_ST_EXECUTE_ACTION:
		enter_execute(sm, now_ms, out);
		break;

	case VC_ST_TERRAIN_CHECK:
		move(out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
		request(out, VC_SRC_IMU, VC_REQ_TERRAIN_MEASURE, 0);
		break;

	case VC_ST_FAULT:
		move(out, VC_MOVE_SAFE_STOP, 0);	/* safe state first (NFR8) */
		request(out, VC_SRC_NONE, VC_REQ_STOP_ALL, 0);
		break;

	case VC_ST_MISSION_COMPLETE:
		move(out, VC_MOVE_STOP, 0);
		request(out, VC_SRC_NONE, VC_REQ_STOP_ALL, 0);
		break;

	default:
		break;
	}
}

LOCAL void go_fault(VC_SM *sm, VC_SOURCE src, UW code, UW now_ms,
		    VC_ACTIONS *out)
{
	report(sm, out, src, VC_FAULT_CRITICAL, code);
	if (sm->st.state != VC_ST_FAULT) {
		enter(sm, VC_ST_FAULT, now_ms, out);
	}
}

/* ------------------------------------------------------------------ */
/*  Events handled the same way in every state                         */
/*  Returns TRUE if the event is fully handled.                        */
/* ------------------------------------------------------------------ */

LOCAL W clamp_slow(VC_MOVE m, W value)
{
	if ((m == VC_MOVE_FORWARD || m == VC_MOVE_SET_SPEED)
	    && value > VC_SPEED_SLOW_MMS) {
		return VC_SPEED_SLOW_MMS;
	}
	return value;
}

LOCAL BOOL handle_common(VC_SM *sm, const VC_EVENT *ev, UW now_ms,
			 VC_ACTIONS *out)
{
	VC_STATE  state  = sm->st.state;
	BOOL      active = vc_sm_is_active(state);
	VC_SOURCE src    = (VC_SOURCE)ev->src;

	switch ((VC_EVENT_TYPE)ev->type) {
	case VC_EV_MOVE_REQUEST:
		/* Command arbitration: only the current owner may move us */
		if (src != VC_SRC_NONE && src == vc_sm_motion_owner(state)
		    && ev->p1 >= 0 && ev->p1 < (W)VC_MOVE_COUNT
		    && ev->p1 != (W)VC_MOVE_RESET) {
			VC_MOVE m = (VC_MOVE)ev->p1;
			W value = ev->p2;
			if (state == VC_ST_DECODE_BARCODE
			    || state == VC_ST_TERRAIN_CHECK) {
				value = clamp_slow(m, value);
			}
			move(out, m, value);
		} else {
			sm->st.moves_rejected++;
		}
		return TRUE;

	case VC_EV_FAULT:
		if (ev->p1 == (W)VC_FAULT_CRITICAL) {
			if (active) {
				go_fault(sm, src, (UW)ev->p2, now_ms, out);
			} else {
				report(sm, out, src, VC_FAULT_CRITICAL, (UW)ev->p2);
			}
		} else if (ev->p1 == (W)VC_FAULT_RECOVERABLE) {
			report(sm, out, src, VC_FAULT_RECOVERABLE, (UW)ev->p2);
			/* A recoverable line fault starts bounded line recovery;
			   every other subsystem runs its own bounded recovery and
			   escalates with a CRITICAL fault if that fails. */
			if (src == VC_SRC_LINE && state == VC_ST_FOLLOW_LINE) {
				sm->lost_side = 0;
				enter(sm, VC_ST_LOST_LINE, now_ms, out);
			}
		} else {
			report(sm, out, src, VC_FAULT_NON_CRITICAL, (UW)ev->p2);
		}
		return TRUE;

	case VC_EV_IMPACT:
		if (active) {
			go_fault(sm, src, VC_FC_IMPACT, now_ms, out);
		}
		return TRUE;

	case VC_EV_CMD_STOP:
		if (active) {
			go_fault(sm, src, VC_FC_OPERATOR_STOP, now_ms, out);
		}
		return TRUE;

	case VC_EV_WIFI_DOWN:
		/* Non-critical (FR8): report, keep navigating */
		sm->st.wifi_up = FALSE;
		report(sm, out, VC_SRC_COMMS, VC_FAULT_NON_CRITICAL, VC_FC_WIFI_DOWN);
		return TRUE;

	case VC_EV_WIFI_UP:
		sm->st.wifi_up = TRUE;
		return TRUE;

	default:
		return FALSE;
	}
}

/* ------------------------------------------------------------------ */
/*  Per-state behaviour                                                */
/* ------------------------------------------------------------------ */

EXPORT void vc_sm_init(VC_SM *sm, UW now_ms)
{
	VC_SM zero = {0};

	*sm = zero;
	sm->st.state          = VC_ST_IDLE;
	sm->st.prev_state     = VC_ST_IDLE;
	sm->st.state_since_ms = now_ms;
	sm->st.last_nav       = NAV_NONE;
	sm->pending_nav       = NAV_NONE;
}

EXPORT void vc_sm_handle(VC_SM *sm, const VC_EVENT *ev, UW now_ms,
			 VC_ACTIONS *out)
{
	VC_EVENT_TYPE type = (VC_EVENT_TYPE)ev->type;
	UW in_state = elapsed(now_ms, sm->st.state_since_ms);

	out->n = 0;
	out->overflow = FALSE;
	if (type != VC_EV_TICK) {
		sm->st.events_handled++;
	}

	/* a hump may also be crossed while reading a barcode or turning:
	   always keep the highest one (FR5) */
	if (type == VC_EV_HUMP_CLEARED && ev->p1 > sm->st.max_hump_um) {
		sm->st.max_hump_um = ev->p1;
	}

	if (type == VC_EV_CMD_PAUSE) {
		if (vc_sm_is_active(sm->st.state) && !sm->st.paused) {
			sm->st.paused = TRUE;
			move(out, VC_MOVE_STOP, 0);
		}
		return;
	}
	if (type == VC_EV_CMD_RESUME) {
		if (sm->st.paused) {
			/* re-enter: reissues the state's requests, restarts timers */
			enter(sm, sm->st.state, now_ms, out);
		} else if (sm->st.state == VC_ST_FAULT
			   || sm->st.state == VC_ST_MISSION_COMPLETE) {
			enter(sm, VC_ST_IDLE, now_ms, out);	/* remote reset */
		}
		return;
	}

	/* paused: no motion, no timeouts; safety events below still act */
	if (sm->st.paused && (type == VC_EV_MOVE_REQUEST || type == VC_EV_TICK)) {
		return;
	}

	if (handle_common(sm, ev, now_ms, out)) {
		return;
	}
	if (sm->st.paused) {
		return;
	}

	switch (sm->st.state) {
	case VC_ST_IDLE:
		if (type == VC_EV_CMD_START) {
			enter(sm, VC_ST_MISSION_START, now_ms, out);
		}
		break;

	case VC_ST_MISSION_START:
		if (type == VC_EV_READY) {
			sm->ready_mask |= VC_READY_BIT(ev->src);
			if ((sm->ready_mask & VC_READY_ALL) == VC_READY_ALL) {
				enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
			}
		} else if (type == VC_EV_TICK && in_state >= VC_SELF_CHECK_MS) {
			if ((sm->ready_mask & VC_READY_REQUIRED) == VC_READY_REQUIRED) {
				/* optional subsystem missing: run degraded */
				report(sm, out, VC_SRC_VC, VC_FAULT_NON_CRITICAL,
				       VC_FC_SELF_CHECK);
				enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
			} else {
				go_fault(sm, VC_SRC_VC, VC_FC_SELF_CHECK, now_ms, out);
			}
		}
		break;

	case VC_ST_FOLLOW_LINE:
		switch (type) {
		case VC_EV_OBSTACLE:	/* safety first */
			enter(sm, VC_ST_AVOID_OBSTACLE, now_ms, out);
			break;
		case VC_EV_LINE_LOST:
			sm->lost_side = (B)ev->p1;
			enter(sm, VC_ST_LOST_LINE, now_ms, out);
			break;
		case VC_EV_BARCODE_START:
			enter(sm, VC_ST_DECODE_BARCODE, now_ms, out);
			break;
		case VC_EV_BARCODE_DECODED:	/* S3 decoded without a START */
			sm->pending_nav = (NavigationCommand)ev->p1;
			sm->st.last_nav = sm->pending_nav;
			sm->st.barcodes_read++;
			enter(sm, VC_ST_EXECUTE_ACTION, now_ms, out);
			break;
		case VC_EV_HUMP_START:
			enter(sm, VC_ST_TERRAIN_CHECK, now_ms, out);
			break;
		case VC_EV_END_OF_COURSE:
			enter(sm, VC_ST_MISSION_COMPLETE, now_ms, out);
			break;
		default:
			break;	/* JUNCTION without a barcode: keep following */
		}
		break;

	case VC_ST_LOST_LINE:
		if (type == VC_EV_LINE_FOUND) {
			enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		} else if (type == VC_EV_OBSTACLE) {
			enter(sm, VC_ST_AVOID_OBSTACLE, now_ms, out);
		} else if (type == VC_EV_TICK
			   && elapsed(now_ms, sm->attempt_since_ms) >= VC_LINE_ATTEMPT_MS) {
			sm->attempts++;
			if (sm->attempts > VC_LINE_MAX_ATTEMPTS) {
				go_fault(sm, VC_SRC_VC, VC_FC_LINE_RECOVERY, now_ms, out);
			} else {
				sm->attempt_since_ms = now_ms;
				search_step(sm, out);
			}
		}
		break;

	case VC_ST_AVOID_OBSTACLE:
		if (type == VC_EV_AVOID_DONE) {
			enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		} else if (type == VC_EV_AVOID_FAILED) {
			/* recoverable (FR8): one more bounded search for the line
			   with S3; LOST_LINE ends in FAULT if that fails too */
			report(sm, out, VC_SRC_OBST, VC_FAULT_RECOVERABLE,
			       VC_FC_AVOID_FAILED);
			enter(sm, VC_ST_LOST_LINE, now_ms, out);
		} else if (type == VC_EV_TICK && in_state >= VC_AVOID_MS) {
			go_fault(sm, VC_SRC_VC, VC_FC_STATE_TIMEOUT, now_ms, out);
		}
		break;

	case VC_ST_DECODE_BARCODE:
		switch (type) {
		case VC_EV_OBSTACLE:
			enter(sm, VC_ST_AVOID_OBSTACLE, now_ms, out);
			break;
		case VC_EV_BARCODE_DECODED:
			sm->pending_nav = (NavigationCommand)ev->p1;
			sm->st.last_nav = sm->pending_nav;
			sm->st.barcodes_read++;
			enter(sm, VC_ST_EXECUTE_ACTION, now_ms, out);
			break;
		case VC_EV_END_OF_COURSE:
			enter(sm, VC_ST_MISSION_COMPLETE, now_ms, out);
			break;
		case VC_EV_BARCODE_FAILED:
		case VC_EV_TICK:
			if (type == VC_EV_TICK
			    && elapsed(now_ms, sm->attempt_since_ms) < VC_DECODE_MS) {
				break;
			}
			sm->retries++;
			if (sm->retries > VC_BARCODE_MAX_RETRY) {
				/* give up on this barcode, keep the mission going */
				report(sm, out, VC_SRC_LINE, VC_FAULT_NON_CRITICAL,
				       VC_FC_BARCODE);
				enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
			} else {
				sm->attempt_since_ms = now_ms;
				request(out, VC_SRC_LINE, VC_REQ_BARCODE_READ,
					(W)sm->retries);
			}
			break;
		default:
			break;	/* LINE_LOST ignored while reading */
		}
		break;

	case VC_ST_EXECUTE_ACTION:
		if (type == VC_EV_MANOEUVRE_DONE) {
			enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		} else if (type == VC_EV_OBSTACLE) {
			enter(sm, VC_ST_AVOID_OBSTACLE, now_ms, out);
		} else if (type == VC_EV_TICK && in_state >= VC_EXECUTE_MS) {
			/* manoeuvre never finished: look for the line */
			report(sm, out, VC_SRC_MOTION, VC_FAULT_RECOVERABLE,
			       VC_FC_STATE_TIMEOUT);
			enter(sm, VC_ST_LOST_LINE, now_ms, out);
		}
		break;

	case VC_ST_TERRAIN_CHECK:
		if (type == VC_EV_HUMP_CLEARED) {
			enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		} else if (type == VC_EV_OBSTACLE) {
			enter(sm, VC_ST_AVOID_OBSTACLE, now_ms, out);
		} else if (type == VC_EV_TICK && in_state >= VC_TERRAIN_MS) {
			report(sm, out, VC_SRC_IMU, VC_FAULT_RECOVERABLE,
			       VC_FC_STATE_TIMEOUT);
			enter(sm, VC_ST_FOLLOW_LINE, now_ms, out);
		}
		break;

	case VC_ST_FAULT:
	case VC_ST_MISSION_COMPLETE:
		/* stay stopped until acknowledged */
		if (type == VC_EV_CMD_RESET) {
			enter(sm, VC_ST_IDLE, now_ms, out);
		}
		break;

	default:
		/* corrupt state: fail safe */
		go_fault(sm, VC_SRC_VC, VC_FC_STATE_TIMEOUT, now_ms, out);
		break;
	}
}
