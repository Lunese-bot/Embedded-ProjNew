/*
 *	test_vc_sm.c
 *	PC unit tests for the Vehicle Controller state machine (vc_sm.c).
 *	No Pico, no kernel.  From this folder:
 *
 *	    make            # builds and runs, prints PASS/FAIL per check
 *
 *	IDs S6-H01 ... are recorded in testing/subsystem6/TEST_LOG.md.
 */

#include <stdio.h>
#include "../vc_sm.h"

static VC_SM		sm;
static VC_ACTIONS	out;
static UW		now;
static int		passed, failed;

#define CHECK(id, cond, desc)						\
	do {								\
		if (cond) { passed++; printf("[TEST] %s PASS  %s\n", id, desc); } \
		else { failed++; printf("[TEST] %s FAIL  %s   (%s:%d)\n",	\
					id, desc, __FILE__, __LINE__); }	\
	} while (0)

static void feed(VC_SOURCE src, VC_EVENT_TYPE type, W p1, W p2)
{
	VC_EVENT ev = {0};
	ev.src = (UB)src; ev.type = (UB)type; ev.p1 = p1; ev.p2 = p2;
	vc_sm_handle(&sm, &ev, now, &out);
}

static void advance(UW ms)
{
	for (UW done = 0; done < ms; done += 20) {	/* wrap-safe */
		now += 20;
		feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
	}
}

static int has_move(VC_MOVE m, W value, int check_value)
{
	for (int i = 0; i < out.n; i++) {
		if (out.a[i].kind == VC_ACT_MOVE && out.a[i].p1 == (W)m
		    && (!check_value || out.a[i].p2 == value)) {
			return 1;
		}
	}
	return 0;
}

static int has_request(VC_SOURCE target, VC_REQUEST r)
{
	for (int i = 0; i < out.n; i++) {
		if (out.a[i].kind == VC_ACT_REQUEST && out.a[i].target == target
		    && out.a[i].p1 == (W)r) {
			return 1;
		}
	}
	return 0;
}

static void start_mission(void)
{
	now = 0xFFFFF000u;	/* close to wrap-around on purpose */
	vc_sm_init(&sm, now);
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
	feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
	feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
	feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
}

int main(void)
{
	/* start-up */
	now = 0; vc_sm_init(&sm, now);
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 200);
	CHECK("S6-H01", sm.st.state == VC_ST_IDLE && out.n == 0,
	      "IDLE ignores move requests");
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	CHECK("S6-H02", sm.st.state == VC_ST_MISSION_START
	      && has_request(VC_SRC_NONE, VC_REQ_SELF_CHECK),
	      "start -> MISSION_START, self check requested");

	start_mission();
	CHECK("S6-H03", sm.st.state == VC_ST_FOLLOW_LINE
	      && has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1)
	      && has_request(VC_SRC_LINE, VC_REQ_LINE_FOLLOW),
	      "all READY -> FOLLOW_LINE at cruise speed");

	now = 0; vc_sm_init(&sm, now);
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
	feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
	advance(VC_SELF_CHECK_MS + 20);
	CHECK("S6-H04", sm.st.state == VC_ST_FOLLOW_LINE
	      && sm.st.last_fault_code == VC_FC_SELF_CHECK,
	      "optional IMU/obstacle missing -> run degraded, reported");

	now = 0; vc_sm_init(&sm, now);
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
	advance(VC_SELF_CHECK_MS + 20);
	CHECK("S6-H05", sm.st.state == VC_ST_FAULT, "motion/line missing -> FAULT");

	/* arbitration */
	start_mission();
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, -300);
	CHECK("S6-H06", has_move(VC_MOVE_STEER, -300, 1),
	      "FOLLOW_LINE: S3 steering forwarded");
	feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_REVERSE, 100);
	CHECK("S6-H07", out.n == 0 && sm.st.moves_rejected == 1,
	      "FOLLOW_LINE: S5 move rejected");
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, 99, 0);
	CHECK("S6-H08", out.n == 0, "invalid move code rejected");

	/* barcode */
	start_mission();
	feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
	CHECK("S6-H09", sm.st.state == VC_ST_DECODE_BARCODE
	      && has_move(VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS, 1),
	      "barcode start -> DECODE_BARCODE, slow");
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 300);
	CHECK("S6-H10", has_move(VC_MOVE_FORWARD, VC_SPEED_SLOW_MMS, 1),
	      "DECODE: S3 speed capped to slow");
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	CHECK("S6-H11", sm.st.state == VC_ST_DECODE_BARCODE,
	      "DECODE: line loss ignored");
	feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, NAV_RIGHT, 0x2A);
	CHECK("S6-H12", sm.st.state == VC_ST_EXECUTE_ACTION
	      && has_move(VC_MOVE_TURN_RIGHT, VC_TURN_DEG, 1)
	      && sm.st.barcodes_read == 1 && sm.st.last_nav == NAV_RIGHT,
	      "RIGHT -> turn right 90");
	feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 0, 0);
	CHECK("S6-H13", sm.st.state == VC_ST_FOLLOW_LINE, "manoeuvre done -> FOLLOW_LINE");

	feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, NAV_STRAIGHT, 0);
	CHECK("S6-H14", sm.st.state == VC_ST_FOLLOW_LINE && !out.overflow,
	      "STRAIGHT: straight back to FOLLOW_LINE");

	feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
	feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
	feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
	CHECK("S6-H15", sm.st.state == VC_ST_DECODE_BARCODE
	      && has_request(VC_SRC_LINE, VC_REQ_BARCODE_READ),
	      "barcode retry within limit");
	feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
	CHECK("S6-H16", sm.st.state == VC_ST_FOLLOW_LINE
	      && sm.st.last_fault_code == VC_FC_BARCODE
	      && sm.st.last_fault_sev == VC_FAULT_NON_CRITICAL,
	      "retries used up -> keep following, non-critical report");

	feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
	advance(VC_DECODE_MS * (VC_BARCODE_MAX_RETRY + 1) + 40);
	CHECK("S6-H17", sm.st.state == VC_ST_FOLLOW_LINE,
	      "barcode read timeouts count as failures");

	feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, NAV_U_TURN, 0);
	advance(VC_EXECUTE_MS + 20);
	CHECK("S6-H18", sm.st.state == VC_ST_LOST_LINE,
	      "manoeuvre timeout -> LOST_LINE recovery");

	/* line loss */
	start_mission();
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	CHECK("S6-H19", sm.st.state == VC_ST_LOST_LINE && has_move(VC_MOVE_STOP, 0, 0)
	      && has_request(VC_SRC_LINE, VC_REQ_LINE_SEARCH),
	      "line lost -> stop, search");
	advance(VC_LINE_ATTEMPT_MS);
	feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
	CHECK("S6-H20", sm.st.state == VC_ST_FOLLOW_LINE, "line found -> FOLLOW_LINE");
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	advance(VC_LINE_ATTEMPT_MS * VC_LINE_MAX_ATTEMPTS + 40);
	CHECK("S6-H21", sm.st.state == VC_ST_FAULT
	      && sm.st.last_fault_code == VC_FC_LINE_RECOVERY,
	      "line recovery bounded -> FAULT");

	/* obstacle */
	start_mission();
	feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
	CHECK("S6-H22", sm.st.state == VC_ST_AVOID_OBSTACLE
	      && has_request(VC_SRC_OBST, VC_REQ_OBST_AVOID), "obstacle -> AVOID");
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
	feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, 30);
	CHECK("S6-H23", sm.st.state == VC_ST_AVOID_OBSTACLE
	      && has_move(VC_MOVE_TURN_LEFT, 30, 1),
	      "AVOID: line loss ignored, S5 moves forwarded");
	feed(VC_SRC_OBST, VC_EV_AVOID_DONE, 0, 0);
	CHECK("S6-H24", sm.st.state == VC_ST_FOLLOW_LINE, "avoid done -> FOLLOW_LINE");
	feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
	feed(VC_SRC_OBST, VC_EV_AVOID_FAILED, 0, 0);
	CHECK("S6-H35", sm.st.state == VC_ST_LOST_LINE
	      && sm.st.last_fault_code == VC_FC_AVOID_FAILED
	      && sm.st.last_fault_sev == VC_FAULT_RECOVERABLE,
	      "avoid failed -> bounded line search (recoverable)");
	feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
	feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
	advance(VC_AVOID_MS + 20);
	CHECK("S6-H25", sm.st.state == VC_ST_FAULT, "avoid timeout -> FAULT");

	/* terrain */
	start_mission();
	feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
	feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 35, 80);
	feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
	feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 20, 50);
	CHECK("S6-H26", sm.st.state == VC_ST_FOLLOW_LINE && sm.st.max_hump_um == 35,
	      "highest hump kept");
	feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
	feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 51, 0);
	CHECK("S6-H36", sm.st.state == VC_ST_DECODE_BARCODE && sm.st.max_hump_um == 51,
	      "hump crossed while reading a barcode is still recorded");

	/* faults */
	start_mission();
	feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
	CHECK("S6-H27", sm.st.state == VC_ST_FOLLOW_LINE && !sm.st.wifi_up,
	      "WiFi down: keep navigating");
	feed(VC_SRC_LINE, VC_EV_FAULT, VC_FAULT_RECOVERABLE, 0x3001);
	CHECK("S6-H28", sm.st.state == VC_ST_LOST_LINE, "recoverable line fault -> LOST_LINE");
	feed(VC_SRC_IMU, VC_EV_IMPACT, 0, 0);
	CHECK("S6-H29", sm.st.state == VC_ST_FAULT && has_move(VC_MOVE_SAFE_STOP, 0, 0)
	      && has_request(VC_SRC_NONE, VC_REQ_STOP_ALL), "impact -> FAULT, stop all");
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 100);
	CHECK("S6-H30", out.n == 0, "FAULT: no motion accepted");
	feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
	CHECK("S6-H31", sm.st.state == VC_ST_FAULT, "FAULT: start ignored until reset");
	feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
	CHECK("S6-H32", sm.st.state == VC_ST_IDLE && has_move(VC_MOVE_RESET, 0, 0),
	      "reset -> IDLE, motion latch cleared");

	start_mission();
	feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
	CHECK("S6-H33", sm.st.state == VC_ST_FAULT
	      && sm.st.last_fault_code == VC_FC_OPERATOR_STOP, "remote stop -> FAULT");

	start_mission();
	feed(VC_SRC_LINE, VC_EV_END_OF_COURSE, 0, 0);
	CHECK("S6-H34", sm.st.state == VC_ST_MISSION_COMPLETE
	      && has_move(VC_MOVE_STOP, 0, 0), "end of course -> MISSION_COMPLETE");

	/* line search pattern and ownership */
	start_mission();
	feed(VC_SRC_LINE, VC_EV_LINE_LOST, +1, 0);	/* last seen on the right */
	CHECK("S6-H37", has_move(VC_MOVE_TURN_RIGHT, VC_SEARCH_TURN_DEG, 1),
	      "search turns towards the side the line was last seen");
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 500);
	CHECK("S6-H38", out.n == 0, "LOST_LINE: S3 steering rejected while searching");
	advance(VC_LINE_ATTEMPT_MS);
	CHECK("S6-H39", has_move(VC_MOVE_TURN_LEFT, 2 * VC_SEARCH_TURN_DEG, 1),
	      "second attempt turns twice as far the other way");
	feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_RESET, 0);
	CHECK("S6-H40", out.n == 0, "subsystems may not request RESET");
	feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
	CHECK("S6-H41", has_request(VC_SRC_LINE, VC_REQ_BARCODE_OFF),
	      "AVOID: barcode reading switched off");

	/* pause / resume */
	start_mission();
	feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
	CHECK("S6-H42", sm.st.paused && has_move(VC_MOVE_STOP, 0, 0)
	      && sm.st.state == VC_ST_FOLLOW_LINE, "pause: stop motors, keep state");
	feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 500);
	CHECK("S6-H43", out.n == 0, "paused: no motion accepted");
	advance(VC_AVOID_MS + VC_TERRAIN_MS + 1000);
	CHECK("S6-H44", sm.st.state == VC_ST_FOLLOW_LINE && sm.st.paused,
	      "paused: no timeout faults");
	feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
	CHECK("S6-H45", !sm.st.paused && has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1),
	      "resume: state re-entered, driving again");
	feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
	feed(VC_SRC_IMU, VC_EV_IMPACT, 0, 0);
	CHECK("S6-H46", sm.st.state == VC_ST_FAULT && !sm.st.paused,
	      "impact while paused -> FAULT");
	feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
	CHECK("S6-H47", sm.st.state == VC_ST_IDLE, "resume in FAULT = remote reset");
	feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
	CHECK("S6-H48", !sm.st.paused, "pause ignored when not running");

	printf("[TEST] subsystem6 host SUMMARY %d passed, %d failed\n", passed, failed);
	return failed ? 1 : 0;
}
