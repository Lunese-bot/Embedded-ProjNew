/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	These tests never make the wheels turn: they check the maths and
 *	the kernel interfaces, and only send STOP / SAFE_STOP / RESET.
 *	Driving tests (speed, distance, turn accuracy) are MANUAL tests,
 *	listed in TEST_LOG.md.
 */

#include "subsystem2.h"
#include "s2_config.h"
#include "motion_pid.h"
#include "../test_report.h"

#if APP_TEST

LOCAL INT absdiff(INT a, INT b)
{
	return (a > b) ? a - b : b - a;
}

EXPORT void subsystem2_test(void)
{
	PID_CTRL	pid;
	MOTION_STATUS	st;
	float		out, prev;
	INT		i;
	BOOL		ok;
	UINT		ptn;

	TEST_INIT();

	/* S2-U01: one wheel turn of ticks = one wheel circumference */
	TEST_CHECK("S2-U01",
		   absdiff((INT)motion_ticks_to_mm(S2_TICKS_PER_REV),
			   (INT)(S2_PI * S2_WHEEL_DIAM_MM)) <= 1,
		   "ticks_to_mm: one revolution = pi * wheel diameter");

	/* S2-U02: a 360 deg pivot moves each wheel pi * track */
	TEST_CHECK("S2-U02",
		   absdiff((INT)motion_turn_arc_mm(360),
			   (INT)(S2_PI * S2_TRACK_MM)) <= 1 &&
		   (INT)motion_turn_arc_mm(-90) == (INT)motion_turn_arc_mm(90),
		   "turn arc: 360 deg = pi * track, left = right");

	/* S2-U03: ramp moves by at most one step and never overshoots */
	TEST_CHECK("S2-U03",
		   motion_ramp(0, 100, 8) == 8 &&
		   motion_ramp(95, 100, 8) == 100 &&
		   motion_ramp(0, -100, 8) == -8,
		   "set-point ramp limited to one step");

	/* S2-U04: output is clamped and the integral cannot wind up */
	pid_init(&pid, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD,
		 S2_PID_I_LIMIT, (float)S2_DUTY_MAX);
	ok = TRUE;
	for (i = 0; i < 1000; i++) {	/* 10 s of a wheel held still */
		out = pid_update(&pid, 400.0f, 0.0f, 0.01f);
		if (out > (float)S2_DUTY_MAX) ok = FALSE;
	}
	TEST_CHECK("S2-U04",
		   ok && pid.integral <= S2_PID_I_LIMIT,
		   "PID output and integral stay within limits");

	/* S2-U05: with the wheel too slow, output rises over time (I term);
	   at the target, output equals the feed-forward plus integral */
	pid_init(&pid, S2_PID_KFF, S2_PID_KP, S2_PID_KI, 0.0f,
		 S2_PID_I_LIMIT, (float)S2_DUTY_MAX);
	prev = pid_update(&pid, 100.0f, 50.0f, 0.01f);
	out  = pid_update(&pid, 100.0f, 50.0f, 0.01f);
	TEST_CHECK("S2-U05", out > prev,
		   "PID integral increases output while below target");

	/* S2-U06: the control loop is running at its period.  Count only
	   this 500 ms window: overruns from boot (USB connect, the burst of
	   test output before this point) are not the loop's fault. */
	{
		UW	loops0, ovr0;

		tk_dly_tsk(300);	/* let the console drain the test output */
		ok = (s2_get_status(&st) == E_OK);
		loops0 = st.loop_count;
		ovr0   = st.overruns;
		tk_dly_tsk(500);
		ok = ok && (s2_get_status(&st) == E_OK);
		tm_printf((UB*)"[TEST] S2-U06 info: %d loops in 500 ms, %d overruns"
			  " (%d since boot)\n", (INT)(st.loop_count - loops0),
			  (INT)(st.overruns - ovr0), (INT)st.overruns);
		TEST_CHECK("S2-U06",
			   ok && (st.loop_count - loops0) >= 45 &&
			   (st.loop_count - loops0) <= 55 &&
			   st.overruns == ovr0,
			   "motion_task runs every 10 ms, no overruns");
	}

	/* S2-U07: STOP command is accepted and the robot is stopped */
	TEST_CHECK("S2-U07",
		   s2_cmd_stop() == E_OK,
		   "IF-M1 message buffer accepts a command");
	tk_dly_tsk(50);
	s2_get_status(&st);
	TEST_CHECK("S2-U08",
		   st.state == MSTATE_IDLE && st.duty_l == 0 &&
		   st.duty_r == 0,
		   "after STOP: state IDLE, both duties 0");

	/* S2-U09: emergency stop latches; DRIVE is refused until RESET */
	tk_clr_flg(flg_motion, ~MFLG_SAFE_STOPPED);
	s2_emergency_stop();
	ok = (tk_wai_flg(flg_motion, MFLG_SAFE_STOPPED,
			 TWF_ORW | TWF_BITCLR, &ptn, 100) == E_OK);
	s2_cmd_drive(100, 0);		/* must be ignored */
	tk_dly_tsk(50);
	s2_get_status(&st);
	TEST_CHECK("S2-U09",
		   ok && st.state == MSTATE_SAFE_STOP &&
		   st.duty_l == 0 && st.duty_r == 0,
		   "emergency stop within 100 ms, latched, DRIVE ignored");

	/* S2-U10: RESET leaves Safe Stop */
	s2_cmd_reset();
	tk_dly_tsk(50);
	s2_get_status(&st);
	TEST_CHECK("S2-U10", st.state == MSTATE_IDLE,
		   "RESET returns to IDLE");

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
