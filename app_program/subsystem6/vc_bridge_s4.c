/*
 *	vc_bridge_s4.c
 *	Subsystem 6 -- adapter to subsystem 4 (IMU terrain monitoring).
 *	S4's interface: ../subsystem4/subsystem4.h (IF-S4-1, IF-S4-4).
 *
 *	S4 -> controller (mbf_imu_evt, IMU_EVENT_MSG)
 *	  CAL_DONE      -> VC_EV_READY            (self check passed)
 *	  CAL_FAILED    -> non-critical fault 0x4004 (S4 retries on its own)
 *	  HUMP_START    -> VC_EV_HUMP_START
 *	  HUMP_END      -> VC_EV_HUMP_CLEARED, p1 = peak height (um)
 *	  HUMP_ABORT    -> VC_EV_HUMP_CLEARED, p1 = 0
 *	  IMPACT        -> VC_EV_IMPACT           (critical: safe stop)
 *	  TILT_LIMIT    -> fault 0x4003 at S4's own severity
 *	  FAULT         -> recoverable fault 0x4001 (S4 retrying, bounded)
 *	  FAULT_FATAL   -> NON-CRITICAL fault 0x4002: the IMU is not needed to
 *	                   follow the line, so the run continues without
 *	                   hump data or impact detection.  TEAM DECISION:
 *	                   change to VC_FAULT_CRITICAL for a safe stop.
 *	  HUMP_PEAK, MOTION, RECOVERED -> not needed by the controller
 *
 *	controller -> S4
 *	  SELF_CHECK -> s4_request_peak_reset() + s4_request_calibration()
 *	                (the robot is standing still in MISSION_START)
 */

#include "vc_bridge.h"
#include "../subsystem4/subsystem4.h"

#ifdef S4_PRI_IMU	/* S4's real interface is present */

#define S4_FC_FAULT		0x4001
#define S4_FC_FATAL		0x4002
#define S4_FC_TILT		0x4003
#define S4_FC_CAL_FAILED	0x4004

LOCAL W severity(UB imu_sev)
{
	switch (imu_sev) {
	case IMU_SEV_CRITICAL:		return VC_FAULT_CRITICAL;
	case IMU_SEV_RECOVERABLE:	return VC_FAULT_RECOVERABLE;
	default:			return VC_FAULT_NON_CRITICAL;
	}
}

LOCAL void make(VC_EVENT *ev, VC_EVENT_TYPE type, W p1, W p2, UW t)
{
	ev->src     = VC_SRC_IMU;
	ev->type    = (UB)type;
	ev->rsv     = 0;
	ev->p1      = p1;
	ev->p2      = p2;
	ev->time_ms = t;
}

EXPORT BOOL vc_bridge_s4_next(VC_EVENT *ev)
{
	IMU_EVENT_MSG m;
	INT len;

	while (1) {
		len = tk_rcv_mbf(mbf_imu_evt, &m, TMO_POL);
		if (len < (INT)sizeof(m)) {
			return FALSE;	/* empty (E_TMOUT) or bad size */
		}
		switch (m.type) {
		case IMU_EVT_CAL_DONE:
			make(ev, VC_EV_READY, 0, 0, m.time_ms);
			return TRUE;
		case IMU_EVT_CAL_FAILED:
			make(ev, VC_EV_FAULT, VC_FAULT_NON_CRITICAL,
			     S4_FC_CAL_FAILED, m.time_ms);
			return TRUE;
		case IMU_EVT_HUMP_START:
			make(ev, VC_EV_HUMP_START, 0, 0, m.time_ms);
			return TRUE;
		case IMU_EVT_HUMP_END:
			make(ev, VC_EV_HUMP_CLEARED, m.value, 0, m.time_ms);
			return TRUE;
		case IMU_EVT_HUMP_ABORT:
			make(ev, VC_EV_HUMP_CLEARED, 0, 0, m.time_ms);
			return TRUE;
		case IMU_EVT_IMPACT:
			make(ev, VC_EV_IMPACT, m.value, 0, m.time_ms);
			return TRUE;
		case IMU_EVT_TILT_LIMIT:
			make(ev, VC_EV_FAULT, severity(m.severity), S4_FC_TILT,
			     m.time_ms);
			return TRUE;
		case IMU_EVT_FAULT:
			make(ev, VC_EV_FAULT, VC_FAULT_RECOVERABLE, S4_FC_FAULT,
			     m.time_ms);
			return TRUE;
		case IMU_EVT_FAULT_FATAL:
			make(ev, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, S4_FC_FATAL,
			     m.time_ms);
			return TRUE;
		default:
			break;	/* HUMP_PEAK, MOTION, RECOVERED: skip */
		}
	}
}

EXPORT ER vc_bridge_s4_request(VC_REQUEST req, W arg)
{
	(void)arg;
	if (req == VC_REQ_SELF_CHECK) {
		CHECK(s4_request_peak_reset(), "s4_request_peak_reset");
		return s4_request_calibration();	/* answers CAL_DONE */
	}
	return E_OK;	/* S4 measures continuously: nothing else to ask */
}

#else	/* only the S4 example: S4 uses vc_post_event() directly */

EXPORT BOOL vc_bridge_s4_next(VC_EVENT *ev)
{
	(void)ev;
	return FALSE;
}

EXPORT ER vc_bridge_s4_request(VC_REQUEST req, W arg)
{
	(void)req; (void)arg;
	return E_OK;
}

#endif /* S4_PRI_IMU */
