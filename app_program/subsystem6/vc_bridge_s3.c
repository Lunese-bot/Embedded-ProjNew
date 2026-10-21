/*
 *	vc_bridge_s3.c
 *	Subsystem 6 -- adapter to subsystem 3 (IR line following + barcode).
 *	S3's interface: ../subsystem3/subsystem3.h (IF-31, IF-32, IF-33).
 *
 *	S3 -> controller
 *	  mbf_s3_events (IF-32):
 *	    LINE_LOST        -> VC_EV_LINE_LOST, p1 = last side
 *	    LINE_FOUND       -> VC_EV_LINE_FOUND
 *	    JUNCTION         -> VC_EV_JUNCTION
 *	    BARCODE_START    -> VC_EV_BARCODE_START
 *	    BARCODE_DECODED  -> VC_EV_BARCODE_DECODED, p1 = nav, p2 = 1 if read backwards
 *	    BARCODE_FAILED   -> VC_EV_BARCODE_FAILED, p1 = reason, p2 = S3 retries
 *	    CAL_FAILED       -> non-critical fault 0x3002 (old calibration kept)
 *	  s3_get_line_status() (IF-31): while S3 owns motion (FOLLOW_LINE,
 *	    DECODE_BARCODE, TERRAIN_CHECK), each NEW sample's PD steering
 *	    becomes a VC_EV_MOVE_REQUEST STEER.  S3 publishes it, it does not
 *	    send it, so the controller reads it every tick (10 ms, NFR2).
 *
 *	controller -> S3 (s3_request, IF-33)
 *	  SELF_CHECK     -> VC_EV_READY if S3 answers (and is calibrated in
 *	                    ADC mode)
 *	  LINE_FOLLOW    -> S3_REQ_BC_ENABLE
 *	  BARCODE_READ n -> S3_REQ_BC_REARM for a retry (n > 0)
 *	  BARCODE_OFF    -> S3_REQ_BC_DISABLE (while avoiding an obstacle)
 *	  LINE_SEARCH    -> nothing: the controller turns the robot, S3 keeps
 *	                    tracking and reports LINE_FOUND
 */

#include "vc_bridge.h"
#include "vc_sm.h"
#include "../subsystem3/subsystem3.h"

#ifdef S3_PRI_LINE	/* S3's real interface is present */

#define S3_FC_CAL_FAILED	0x3002

LOCAL UW	last_seq;
LOCAL BOOL	ready_pending;

LOCAL void make(VC_EVENT *ev, VC_EVENT_TYPE type, W p1, W p2, UW t)
{
	ev->src     = VC_SRC_LINE;
	ev->type    = (UB)type;
	ev->rsv     = 0;
	ev->p1      = p1;
	ev->p2      = p2;
	ev->time_ms = t;
}

EXPORT BOOL vc_bridge_s3_next(VC_EVENT *ev, VC_STATE state)
{
	S3_EVENT	e;
	S3_LINE_STATUS	st;
	INT		len;

	if (ready_pending) {
		ready_pending = FALSE;
		make(ev, VC_EV_READY, 0, 0, 0);
		return TRUE;
	}

	/* 1. events first: a LINE_LOST must beat the next steering value */
	while ((len = tk_rcv_mbf(mbf_s3_events, &e, TMO_POL)) == (INT)sizeof(e)) {
		switch (e.type) {
		case S3_EVT_LINE_LOST:
			make(ev, VC_EV_LINE_LOST, e.side, 0, e.time_ms);
			return TRUE;
		case S3_EVT_LINE_FOUND:
			make(ev, VC_EV_LINE_FOUND, 0, 0, e.time_ms);
			return TRUE;
		case S3_EVT_JUNCTION:
			make(ev, VC_EV_JUNCTION, 0, 0, e.time_ms);
			return TRUE;
		case S3_EVT_BARCODE_START:
			make(ev, VC_EV_BARCODE_START, 0, 0, e.time_ms);
			return TRUE;
		case S3_EVT_BARCODE_DECODED:
			tm_printf((UB*)"vc: barcode \"%s\" nav %d\n", e.text, e.nav);
			make(ev, VC_EV_BARCODE_DECODED, e.nav, (W)e.arg, e.time_ms);
			return TRUE;
		case S3_EVT_BARCODE_FAILED:
			make(ev, VC_EV_BARCODE_FAILED, e.code, (W)e.arg, e.time_ms);
			return TRUE;
		case S3_EVT_CAL_FAILED:
			make(ev, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, S3_FC_CAL_FAILED,
			     e.time_ms);
			return TRUE;
		default:
			break;	/* CAL_DONE: nothing for the controller */
		}
	}

	/* 2. steering, once per new S3 sample, only while S3 owns motion */
	if (vc_sm_motion_owner(state) == VC_SRC_LINE
	    && s3_get_line_status(&st) == E_OK && st.seq != last_seq
	    && st.line_state == LINE_STATE_FOLLOW) {
		last_seq = st.seq;
		make(ev, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, st.steering, st.time_ms);
		return TRUE;
	}
	return FALSE;
}

EXPORT ER vc_bridge_s3_request(VC_REQUEST req, W arg)
{
	S3_LINE_STATUS st;

	switch (req) {
	case VC_REQ_SELF_CHECK:
		if (s3_get_line_status(&st) == E_OK
		    && (st.calibrated || !S3_IR_USE_ADC)) {
			ready_pending = TRUE;
		}
		return E_OK;
	case VC_REQ_LINE_FOLLOW:
		return s3_request(S3_REQ_BC_ENABLE);
	case VC_REQ_BARCODE_READ:
		return (arg > 0) ? s3_request(S3_REQ_BC_REARM) : E_OK;
	case VC_REQ_BARCODE_OFF:
		return s3_request(S3_REQ_BC_DISABLE);
	default:
		return E_OK;
	}
}

#else	/* only the S3 example: S3 uses vc_post_event() directly */

EXPORT BOOL vc_bridge_s3_next(VC_EVENT *ev, VC_STATE state)
{
	(void)ev; (void)state;
	return FALSE;
}

EXPORT ER vc_bridge_s3_request(VC_REQUEST req, W arg)
{
	(void)req; (void)arg;
	return E_OK;
}

#endif /* S3_PRI_LINE */
