/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	U01-U09 test the pure algorithms with made-up scans, so they pass
 *	without any hardware connected.  U10-U11 need the sensor wired.
 */

#include "s5_local.h"
#include "drv_ultrasonic.h"
#include "../test_report.h"

#if APP_TEST

/* A box straight ahead at 200 mm, about 120 mm wide, open both sides */
LOCAL const S5_SAMPLE scan_centre[] = {
	{ -60, 0 }, { -40, 0 }, { -20, 210 }, { -10, 200 }, { 0, 200 },
	{ 10, 200 }, { 20, 215 }, { 40, 0 }, { 60, 0 }
};

/* Same box, but a wall close on the left */
LOCAL const S5_SAMPLE scan_wall_left[] = {
	{ -60, 0 }, { -40, 0 }, { -20, 210 }, { -10, 200 }, { 0, 200 },
	{ 10, 200 }, { 20, 215 }, { 40, 650 }, { 60, 220 }
};

/* Blocked everywhere */
LOCAL const S5_SAMPLE scan_blocked[] = {
	{ -60, 230 }, { -30, 210 }, { 0, 200 }, { 30, 210 }, { 60, 230 }
};

#define N(a)	((INT)(sizeof(a) / sizeof((a)[0])))

EXPORT void subsystem5_test(void)
{
	S5_PROFILE	p;
	S5_STEP		plan[S5_PLAN_MAX];
	S5_STATUS	st;
	S5_SAMPLE	empty[3] = { { -30, 0 }, { 0, 0 }, { 30, 0 } };
	INT		n;
	UH		mm;
	BOOL		ok;

	TEST_INIT();

	TEST_CHECK("S5-U01", s5_sin1000(30) == 500 && s5_sin1000(-90) == -1000
		   && s5_sin1000(0) == 0, "sine table");

	ok = s5_build_profile(scan_centre, N(scan_centre), &p);
	TEST_CHECK("S5-U02", ok && p.distance_mm == 200 &&
		   p.right_edge_deg == -20 && p.left_edge_deg == 20,
		   "profile: distance and edges of a centred box");
	TEST_CHECK("S5-U03", p.width_mm > 100 && p.width_mm < 200,
		   "profile: width of a centred box ~145 mm");

	TEST_CHECK("S5-U04", !s5_build_profile(empty, 3, &p) && !p.valid,
		   "profile: no echo anywhere = no obstacle");

	(void)s5_build_profile(scan_wall_left, N(scan_wall_left), &p);
	TEST_CHECK("S5-U05", p.bypass_side == S5_SIDE_RIGHT,
		   "side: wall on the left -> bypass right");

	(void)s5_build_profile(scan_blocked, N(scan_blocked), &p);
	TEST_CHECK("S5-U06", p.bypass_side == S5_SIDE_NONE,
		   "side: blocked both sides -> none");
	n = s5_plan_bypass(&p, plan, S5_PLAN_MAX);
	TEST_CHECK("S5-U07", n == 2 && plan[1].m.kind == S5_MOVE_REVERSE,
		   "plan: blocked -> stop and reverse only");

	(void)s5_build_profile(scan_wall_left, N(scan_wall_left), &p);
	n = s5_plan_bypass(&p, plan, S5_PLAN_MAX);
	TEST_CHECK("S5-U08", n == 7 && plan[0].m.kind == S5_MOVE_STOP &&
		   plan[1].m.angle_deg == -S5_BYPASS_TURN_DEG &&
		   plan[5].m.angle_deg ==  S5_BYPASS_TURN_DEG &&
		   plan[6].search_line == 1,
		   "plan: right bypass = stop, R45, fwd, L45, fwd, L45, search");

	n = s5_plan_recovery(S5_SIDE_RIGHT, 2, plan, S5_PLAN_MAX);
	TEST_CHECK("S5-U09", n == 2 && plan[1].search_line &&
		   plan[1].m.distance_mm ==
		   S5_RECOVERY_BASE_MM + 2 * S5_RECOVERY_STEP_MM,
		   "recovery: attempt 3 searches further");

	tk_dly_tsk(300);	/* let the scan task run a few periods */
	TEST_CHECK("S5-U10", s5_get_status(&st) == E_OK && st.pings > 0,
		   "scan task is pinging (status counts pings)");

	/* Needs the sensor: hold a flat object 20-30 cm in front */
	mm = st.last_distance_mm;
	tm_printf((UB*)"[TEST] S5-U11 info: last distance %d mm, %d pings\n",
		  (INT)mm, (INT)st.pings);
	TEST_CHECK("S5-U11", mm == US_NO_ECHO || (mm > 20 && mm < 2000),
		   "last distance is 'no echo' or in sensor range");

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
