/*
 *	vc_bridge_s5.c
 *	Subsystem 6 -- adapter to subsystem 5's mbf_s5_cmd / mbf_s5_evt.
 *
 *	S5 -> controller
 *	  OBSTACLE_DETECTED  -> VC_EV_OBSTACLE (p1 = distance mm)
 *	  MOTION_REQ         -> VC_EV_MOVE_REQUEST(s); S5 waits for
 *	                        MOTION_DONE with the same seq:
 *	                          STOP          answered at once
 *	                          TURN +/-deg   answered on MANOEUVRE_DONE
 *	                          FORWARD       SET_SPEED + DRIVE_DIST, ditto
 *	                          REVERSE       SET_SPEED + REVERSE, ditto
 *	                        a refused request is answered with ok = 0
 *	  LINE_REACQUIRED    -> VC_EV_AVOID_DONE
 *	  AVOID_FAILED       -> VC_EV_AVOID_FAILED (bounded line search),
 *	                        or a CRITICAL fault if no side was clear
 *	  FAULT              -> VC_EV_FAULT CRITICAL, code 0x50nn: without
 *	                        the sensor the robot cannot see obstacles
 *	  PROFILE_READY      -> printed only (S1 can read s5_get_status)
 *
 *	controller -> S5
 *	  SELF_CHECK  -> READY back to the controller if S5 answers status
 *	  OBST_MONITOR-> S5_CMD_ENABLE
 *	  OBST_AVOID  -> S5_CMD_START_AVOID
 *	  STOP_ALL    -> S5_CMD_ABORT, then S5_CMD_DISABLE
 *	  an OBSTACLE the controller does not act on -> S5_CMD_ABORT
 */

#include "vc_bridge.h"
#include "../subsystem5/subsystem5.h"

#ifdef S5_PRI_AVOID	/* S5's real interface is present */

#define PENDING_MAX	3

LOCAL VC_EVENT	pending[PENDING_MAX];	/* converted, not yet handed out */
LOCAL INT	pending_n;
LOCAL BOOL	wait_done;		/* an S5 motion is in progress */
LOCAL UH	wait_seq;

LOCAL void push(VC_EVENT_TYPE type, W p1, W p2, UW time_ms)
{
	VC_EVENT *e;

	if (pending_n >= PENDING_MAX) {
		return;
	}
	e = &pending[pending_n++];
	e->src     = VC_SRC_OBST;
	e->type    = (UB)type;
	e->rsv     = 0;
	e->p1      = p1;
	e->p2      = p2;
	e->time_ms = time_ms;
}

LOCAL void send_cmd(S5_CMD_TYPE type, UB ok, UH seq)
{
	S5_CMD c;

	c.type = (UB)type;
	c.ok   = ok;
	c.seq  = seq;
	CHECK(tk_snd_mbf(mbf_s5_cmd, &c, sizeof(c), TMO_POL), "vc->s5 cmd");
}

LOCAL void convert_motion(const S5_MOTION_REQ *m, UW t)
{
	switch (m->kind) {
	case S5_MOVE_STOP:
		push(VC_EV_MOVE_REQUEST, VC_MOVE_STOP, 0, t);
		break;
	case S5_MOVE_TURN:
		if (m->angle_deg >= 0) {
			push(VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, m->angle_deg, t);
		} else {
			push(VC_EV_MOVE_REQUEST, VC_MOVE_TURN_RIGHT, -m->angle_deg, t);
		}
		break;
	case S5_MOVE_FORWARD:
		push(VC_EV_MOVE_REQUEST, VC_MOVE_SET_SPEED, m->speed_mm_s, t);
		push(VC_EV_MOVE_REQUEST, VC_MOVE_DRIVE_DIST, m->distance_mm, t);
		break;
	case S5_MOVE_REVERSE:
		push(VC_EV_MOVE_REQUEST, VC_MOVE_SET_SPEED, m->speed_mm_s, t);
		push(VC_EV_MOVE_REQUEST, VC_MOVE_REVERSE, m->distance_mm, t);
		break;
	default:
		send_cmd(S5_CMD_MOTION_DONE, 0, m->seq);	/* unknown: refuse */
		return;
	}
	wait_done = TRUE;
	wait_seq  = m->seq;
}

EXPORT BOOL vc_bridge_s5_next(VC_EVENT *ev)
{
	S5_EVT	e;
	INT	len;
	INT	i;

	while (pending_n == 0) {
		len = tk_rcv_mbf(mbf_s5_evt, &e, TMO_POL);
		if (len < (INT)sizeof(e)) {
			return FALSE;	/* empty (E_TMOUT) or bad size */
		}
		switch (e.type) {
		case S5_EVT_OBSTACLE_DETECTED:
			push(VC_EV_OBSTACLE, e.distance_mm, 0, e.time_ms);
			break;
		case S5_EVT_MOTION_REQ:
			convert_motion(&e.u.motion, e.time_ms);
			break;
		case S5_EVT_LINE_REACQUIRED:
			push(VC_EV_AVOID_DONE, 0, 0, e.time_ms);
			break;
		case S5_EVT_AVOID_FAILED:
			if (e.code == S5_FAULT_BLOCKED) {
				push(VC_EV_FAULT, VC_FAULT_CRITICAL, 0x5000 | e.code,
				     e.time_ms);
			} else {
				push(VC_EV_AVOID_FAILED, e.code, 0, e.time_ms);
			}
			break;
		case S5_EVT_FAULT:
			push(VC_EV_FAULT, VC_FAULT_CRITICAL, 0x5000 | e.code, e.time_ms);
			break;
		case S5_EVT_PROFILE_READY:
			tm_printf((UB*)"vc: s5 profile d=%d w=%d side=%d\n",
				  e.u.profile.distance_mm, e.u.profile.width_mm,
				  e.u.profile.bypass_side);
			break;
		default:
			break;
		}
	}

	*ev = pending[0];
	for (i = 1; i < pending_n; i++) {
		pending[i - 1] = pending[i];
	}
	pending_n--;
	return TRUE;
}

EXPORT void vc_bridge_s5_after(const VC_EVENT *ev, VC_STATE state_after,
			       BOOL rejected)
{
	if (ev->type == VC_EV_OBSTACLE && state_after != VC_ST_AVOID_OBSTACLE) {
		send_cmd(S5_CMD_ABORT, 0, 0);	/* we will not act on it */
		return;
	}
	if (ev->type != VC_EV_MOVE_REQUEST || !wait_done) {
		return;
	}
	if (rejected) {
		pending_n = 0;			/* drop the rest of this request */
		wait_done = FALSE;
		send_cmd(S5_CMD_MOTION_DONE, 0, wait_seq);
	} else if (ev->p1 == (W)VC_MOVE_STOP) {
		wait_done = FALSE;		/* a stop is done at once */
		send_cmd(S5_CMD_MOTION_DONE, 1, wait_seq);
	}
}

EXPORT void vc_bridge_s5_manoeuvre_done(BOOL ok)
{
	if (wait_done) {
		wait_done = FALSE;
		send_cmd(S5_CMD_MOTION_DONE, ok ? 1 : 0, wait_seq);
	}
}

EXPORT ER vc_bridge_s5_request(VC_REQUEST req, W arg)
{
	S5_STATUS st;

	(void)arg;
	switch (req) {
	case VC_REQ_SELF_CHECK:
		if (s5_get_status(&st) == E_OK && st.state != S5_ST_ERROR) {
			push(VC_EV_READY, 0, 0, 0);
		}
		break;
	case VC_REQ_OBST_MONITOR:
		send_cmd(S5_CMD_ENABLE, 0, 0);
		break;
	case VC_REQ_OBST_AVOID:
		wait_done = FALSE;
		send_cmd(S5_CMD_START_AVOID, 0, 0);
		break;
	case VC_REQ_STOP_ALL:
		pending_n = 0;
		wait_done = FALSE;
		send_cmd(S5_CMD_ABORT, 0, 0);
		send_cmd(S5_CMD_DISABLE, 0, 0);
		break;
	default:
		break;
	}
	return E_OK;
}

#else	/* only the S5 template: S5 uses vc_post_event() directly */

EXPORT BOOL vc_bridge_s5_next(VC_EVENT *ev)
{
	(void)ev;
	return FALSE;
}

EXPORT void vc_bridge_s5_after(const VC_EVENT *ev, VC_STATE state_after,
			       BOOL rejected)
{
	(void)ev; (void)state_after; (void)rejected;
}

EXPORT void vc_bridge_s5_manoeuvre_done(BOOL ok)
{
	(void)ok;
}

EXPORT ER vc_bridge_s5_request(VC_REQUEST req, W arg)
{
	(void)req; (void)arg;
	return E_OK;
}

#endif /* S5_PRI_AVOID */
