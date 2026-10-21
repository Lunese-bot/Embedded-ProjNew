/*
 *	vc_bridge_s2.c
 *	Subsystem 6 -- adapter to subsystem 2 (motion control).
 *	S2's interface: ../subsystem2/subsystem2.h (IF-M1, IF-M2, IF-M3).
 *
 *	controller -> S2 (mbf_motion_cmd through S2's s2_cmd_xxx helpers)
 *	  STOP          s2_cmd_stop()  (falls back to s2_emergency_stop()
 *	                if the queue is full: a stop is never dropped)
 *	  FORWARD v     s2_cmd_drive(v, 0)
 *	  STEER s       s2_cmd_drive(speed, steer): S3's -1000..+1000
 *	                (+ = right) becomes S2's wheel-speed difference
 *	                (+ = left), full scale = the current speed
 *	  SET_SPEED v   remembered; re-sent at once while driving
 *	  TURN_LEFT a   s2_cmd_turn(turn speed, +a)   (+ = left in S2)
 *	  TURN_RIGHT a  s2_cmd_turn(turn speed, -a)
 *	  U_TURN        s2_cmd_turn(turn speed, 180)
 *	  DRIVE_DIST d  s2_cmd_drive_dist(speed, +d)
 *	  REVERSE d     s2_cmd_drive_dist(speed, -d)
 *	  SAFE_STOP     s2_emergency_stop()  (latched in S2)
 *	  RESET         s2_cmd_reset()
 *	S2 stops by itself if DRIVE is not refreshed within S2_CMD_TIMEOUT_MS
 *	(500 ms).  In FOLLOW_LINE the steering from S3 refreshes it every
 *	10 ms; if that stops, S2 stops and reports it.
 *
 *	S2 -> controller (flg_motion, polled with TWF_BITCLR, never blocks)
 *	  MFLG_CMD_DONE     -> VC_EV_MANOEUVRE_DONE (p1 = 1)
 *	  MFLG_FAULT        -> CRITICAL fault 0x20nn (nn = MOTION_FAULT)
 *	  MFLG_CMD_TIMEOUT  -> CRITICAL fault 0x2010 (steering stopped)
 *	  SELF_CHECK        -> VC_EV_READY if S2 is not latched in a fault
 */

#include "vc_bridge.h"
#include "../subsystem2/subsystem2.h"

#ifdef S2_PRI_MOTION	/* S2's real interface is present */

#define VC_TURN_SPEED_MMS	120	/* wheel speed for pivot turns */
#define S2_FC_TIMEOUT		0x2010

#define S2_EVENT_BITS	(MFLG_CMD_DONE | MFLG_FAULT | MFLG_CMD_TIMEOUT)

LOCAL INT	speed = 0;		/* current forward speed, mm/s */
LOCAL INT	steer = 0;		/* last steering, S2 units */
LOCAL BOOL	driving = FALSE;	/* last command was a DRIVE */
LOCAL UINT	got;			/* flag bits not yet handed out */
LOCAL BOOL	ready_pending;

EXPORT ER vc_bridge_s2_move(VC_MOVE m, W value)
{
	ER er;

	switch (m) {
	case VC_MOVE_STOP:
		driving = FALSE;
		er = s2_cmd_stop();
		if (er < E_OK) {
			er = s2_emergency_stop();	/* never drop a stop */
		}
		return er;
	case VC_MOVE_FORWARD:
		speed = (INT)value;
		steer = 0;
		driving = TRUE;
		return s2_cmd_drive(speed, 0);
	case VC_MOVE_STEER:
		steer = -(INT)((value * (W)speed) / 1000);
		driving = TRUE;
		return s2_cmd_drive(speed, steer);
	case VC_MOVE_SET_SPEED:
		speed = (INT)value;
		return driving ? s2_cmd_drive(speed, steer) : E_OK;
	case VC_MOVE_TURN_LEFT:
		driving = FALSE;
		return s2_cmd_turn(VC_TURN_SPEED_MMS, (INT)value);
	case VC_MOVE_TURN_RIGHT:
		driving = FALSE;
		return s2_cmd_turn(VC_TURN_SPEED_MMS, -(INT)value);
	case VC_MOVE_U_TURN:
		driving = FALSE;
		return s2_cmd_turn(VC_TURN_SPEED_MMS, 180);
	case VC_MOVE_DRIVE_DIST:
		driving = FALSE;
		return s2_cmd_drive_dist(speed, (INT)value);
	case VC_MOVE_REVERSE:
		driving = FALSE;
		return s2_cmd_drive_dist(speed, -(INT)value);
	case VC_MOVE_SAFE_STOP:
		driving = FALSE;
		return s2_emergency_stop();
	case VC_MOVE_RESET:
		driving = FALSE;
		return s2_cmd_reset();
	default:
		return E_PAR;
	}
}

LOCAL void make(VC_EVENT *ev, VC_EVENT_TYPE type, W p1, W p2)
{
	ev->src     = VC_SRC_MOTION;
	ev->type    = (UB)type;
	ev->rsv     = 0;
	ev->p1      = p1;
	ev->p2      = p2;
	ev->time_ms = 0;
}

EXPORT BOOL vc_bridge_s2_next(VC_EVENT *ev)
{
	UINT		ptn;
	MOTION_STATUS	st;

	if (ready_pending) {
		ready_pending = FALSE;
		make(ev, VC_EV_READY, 0, 0);
		return TRUE;
	}
	if (got == 0
	    && tk_wai_flg(flg_motion, S2_EVENT_BITS, TWF_ORW | TWF_BITCLR,
			  &ptn, TMO_POL) == E_OK) {
		got = ptn & S2_EVENT_BITS;
	}

	if (got & MFLG_FAULT) {
		got &= ~MFLG_FAULT;
		st.fault = 0;
		s2_get_status(&st);
		make(ev, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x2000 | (st.fault & 0xFF));
		return TRUE;
	}
	if (got & MFLG_CMD_TIMEOUT) {
		got &= ~MFLG_CMD_TIMEOUT;
		make(ev, VC_EV_FAULT, VC_FAULT_CRITICAL, S2_FC_TIMEOUT);
		return TRUE;
	}
	if (got & MFLG_CMD_DONE) {
		got &= ~MFLG_CMD_DONE;
		make(ev, VC_EV_MANOEUVRE_DONE, 1, 0);
		return TRUE;
	}
	return FALSE;
}

EXPORT ER vc_bridge_s2_request(VC_REQUEST req, W arg)
{
	MOTION_STATUS st;

	(void)arg;
	if (req == VC_REQ_SELF_CHECK && s2_get_status(&st) == E_OK
	    && st.state != MSTATE_FAULT) {
		ready_pending = TRUE;
	}
	return E_OK;
}

#else	/* only the S2 example: log the command, S2 not bound yet */

EXPORT ER vc_bridge_s2_move(VC_MOVE m, W value)
{
	(void)m; (void)value;
	return E_OK;
}

EXPORT BOOL vc_bridge_s2_next(VC_EVENT *ev)
{
	(void)ev;
	return FALSE;
}

EXPORT ER vc_bridge_s2_request(VC_REQUEST req, W arg)
{
	(void)req; (void)arg;
	return E_OK;
}

#endif /* S2_PRI_MOTION */
