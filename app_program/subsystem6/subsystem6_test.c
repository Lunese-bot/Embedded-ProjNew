/*
 *	subsystem6_test.c
 *	On-target tests for subsystem 6.  Built only with: make TEST=1
 *	Record results in testing/subsystem6/TEST_LOG.md.  IDs: S6-U01, ...
 *
 *	The state-machine tests drive a PRIVATE VC_SM, so the live
 *	controller and the motors are not touched.  The same scenarios run
 *	on a PC with host_test/ (no Pico needed).
 */

#include "subsystem6.h"
#include "vc_sm.h"
#include "../test_report.h"

#if APP_TEST

LOCAL VC_SM	 t_sm;
LOCAL VC_ACTIONS t_out;
LOCAL UW	 t_now;

LOCAL void feed(VC_SOURCE src, VC_EVENT_TYPE type, W p1, W p2)
{
	VC_EVENT ev = {0};

	ev.src  = (UB)src;
	ev.type = (UB)type;
	ev.p1   = p1;
	ev.p2   = p2;
	vc_sm_handle(&t_sm, &ev, t_now, &t_out);
}

LOCAL void advance(UW ms)		/* ticks every 20 ms for ms */
{
	UW done;

	for (done = 0; done < ms; done += 20) {	/* wrap-safe */
		t_now += 20;
		feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
	}
}

LOCAL BOOL has_move(VC_MOVE m)
{
	INT i;

	for (i = 0; i < t_out.n; i++) {
		if (t_out.a[i].kind == VC_ACT_MOVE && t_out.a[i].p1 == (W)m) {
			return TRUE;
		}
	}
	return FALSE;
}

LOCAL void start_mission(void)
{
	t_now = 1000;
	vc_sm_init(&t_sm, t_now);
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
	feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
	feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
	feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
}

EXPORT void subsystem6_test(void)
{
	VC_STATUS st;
	TEST_INIT();

	/* --- live objects --- */
	TEST_CHECK("S6-U01", tid_vc > 0 && mbf_vc_events > 0
		   && mtx_vc_status > 0 && flg_vc_state > 0,
		   "kernel objects and task created");
	TEST_CHECK("S6-U02", vc_get_status(&st) == E_OK && st.state == VC_ST_IDLE,
		   "live controller waits in IDLE");
	TEST_CHECK("S6-U03", vc_post_event(VC_SRC_COMMS, VC_EV_WIFI_UP, 0, 0,
					   TMO_POL) == E_OK,
		   "vc_post_event accepted");

	/* --- state machine (private instance) --- */
	start_mission();
	TEST_CHECK("S6-U04", t_sm.st.state == VC_ST_FOLLOW_LINE
		   && has_move(VC_MOVE_FORWARD),
		   "start + all READY -> FOLLOW_LINE, drive forward");

	t_now = 1000; vc_sm_init(&t_sm, t_now);
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
	advance(VC_SELF_CHECK_MS + 40);
	TEST_CHECK("S6-U05", t_sm.st.state == VC_ST_FAULT,
		   "self check: motion/line missing -> FAULT");

	start_mission();
	feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
	feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, NAV_LEFT, 0);
	TEST_CHECK("S6-U06", t_sm.st.state == VC_ST_EXECUTE_ACTION
		   && has_move(VC_MOVE_TURN_LEFT),
		   "barcode LEFT -> EXECUTE_ACTION, turn left");
	feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 0, 0);
	TEST_CHECK("S6-U07", t_sm.st.state == VC_ST_FOLLOW_LINE,
		   "manoeuvre done -> FOLLOW_LINE");

	start_mission();
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	advance(VC_LINE_ATTEMPT_MS * (VC_LINE_MAX_ATTEMPTS + 1));
	TEST_CHECK("S6-U08", t_sm.st.state == VC_ST_FAULT
		   && t_sm.st.last_fault_code == VC_FC_LINE_RECOVERY,
		   "line not found within limits -> FAULT");

	start_mission();
	feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 300);
	TEST_CHECK("S6-U09", t_sm.st.state == VC_ST_AVOID_OBSTACLE
		   && t_out.n == 0 && t_sm.st.moves_rejected == 1,
		   "AVOID: move request from S3 rejected");
	feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_RIGHT, 45);
	TEST_CHECK("S6-U10", has_move(VC_MOVE_TURN_RIGHT),
		   "AVOID: move request from S5 forwarded");

	start_mission();
	feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
	TEST_CHECK("S6-U11", t_sm.st.state == VC_ST_FOLLOW_LINE,
		   "WiFi down is non-critical: keep following");

	feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4001);
	TEST_CHECK("S6-U12", t_sm.st.state == VC_ST_FAULT
		   && has_move(VC_MOVE_SAFE_STOP),
		   "critical fault -> FAULT, motors stopped");
	feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
	TEST_CHECK("S6-U13", t_sm.st.state == VC_ST_IDLE,
		   "reset from FAULT -> IDLE");

	start_mission();
	feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 42, 0);
	TEST_CHECK("S6-U14", t_sm.st.state == VC_ST_FOLLOW_LINE
		   && t_sm.st.max_hump_um == 42,
		   "hump: line loss ignored, peak recorded");

	TEST_SUMMARY("subsystem6");
}

#endif /* APP_TEST */
