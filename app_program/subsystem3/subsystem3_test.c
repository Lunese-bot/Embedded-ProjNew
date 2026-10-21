/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md.
 *
 *	S3-U01..U05 need no IR hardware: they feed synthetic data to the
 *	algorithms and to barcode_task.  S3-U06 needs the robot on the
 *	track (manual).  The same algorithm checks also run on a PC:
 *	testing/subsystem3/host/.
 *
 *	NOTE: S3-U05 reads mbf_s3_events itself.  If the Vehicle Controller
 *	is also reading it in the test build, it may take the event first;
 *	run this test before the controller starts, or disable it.
 */

#include "subsystem3.h"
#include "../subsystem6/subsystem6.h"	/* tid_vc: paused during S3-U05 */
#include "code39.h"
#include "line_algo.h"
#include "../test_report.h"

#if APP_TEST

LOCAL void drain_events(void)
{
	S3_EVENT	evt;

	while (tk_rcv_mbf(mbf_s3_events, &evt, TMO_POL) > 0) {
		/* discard */
	}
}

/* Wait up to tmo_ms for an event of the given type */
LOCAL BOOL wait_event(UB type, S3_EVENT *p_evt, INT tmo_ms)
{
	while (tmo_ms > 0) {
		if (tk_rcv_mbf(mbf_s3_events, p_evt, 50) == (INT)sizeof(S3_EVENT)) {
			if (p_evt->type == type) {
				return TRUE;
			}
		} else {
			tmo_ms -= 50;
		}
	}
	return FALSE;
}

EXPORT void subsystem3_test(void)
{
	uint32_t	w[S3_BC_MAX_ELEMENTS];
	char		text[S3_BC_TEXT_MAX];
	bool		rev = false;
	code39_params_t	prm = { S3_BC_MIN_RATIO_X10 };
	S3_LINE_STATUS	st1, st2;
	S3_EVENT	evt;
	uint32_t		n;
	uint32_t		i;

	TEST_INIT();

	/* S3-U01: Code 39 "*L*" decodes, forwards and backwards */
	n = code39_encode("L", 20, 50, w, S3_BC_MAX_ELEMENTS);
	TEST_CHECK("S3-U01",
		   n == 29 &&
		   code39_decode(w, n, &prm, text, sizeof(text), &rev) == CODE39_OK &&
		   text[0] == 'L' && text[1] == '\0' && !rev,
		   "Code 39 *L* decodes forwards");
	for (i = 0; i < n / 2; i++) {
		uint32_t t = w[i]; w[i] = w[n - 1 - i]; w[n - 1 - i] = t;
	}
	TEST_CHECK("S3-U01b",
		   code39_decode(w, n, &prm, text, sizeof(text), &rev) == CODE39_OK &&
		   text[0] == 'L' && rev,
		   "Code 39 *L* decodes backwards");

	/* S3-U02: a corrupted barcode is rejected */
	n = code39_encode("R", 20, 50, w, S3_BC_MAX_ELEMENTS);
	w[12] = 50;	/* a 4th wide element in the data character */
	TEST_CHECK("S3-U02",
		   code39_decode(w, n, &prm, text, sizeof(text), &rev) != CODE39_OK,
		   "corrupted barcode rejected");

	/* S3-U03: line position and steering sign */
	{
		line_params_t	p;
		line_tracker_t	t;
		uint16_t	right_only[S3_IR_COUNT] = { 0, 0, 1000 };

		line_params_default(&p);
		p.line_mask = S3_MASK_L | S3_MASK_C | S3_MASK_R;
		line_tracker_reset(&t);
		(void)line_tracker_step(&t, &p, right_only, false);
		TEST_CHECK("S3-U03", t.position == 1000 && t.steering > 0 &&
			   t.last_side == 1,
			   "line on the right -> position +1000, steer right");
	}

	/* S3-U04: the line task runs every 10 ms (seq advances ~50 in 500 ms) */
	s3_get_line_status(&st1);
	tk_dly_tsk(500);
	s3_get_line_status(&st2);
	{
		IMPORT UW	s3_line_overruns, s3_line_read_errors, s3_line_wakeups;
		T_RTSK		rt;
		T_RCYC		rc;
		ER		e1, e2;

		e1 = tk_ref_tsk(tid_s3_line, &rt);
		e2 = tk_ref_cyc(cyc_s3_line, &rc);
		tm_printf((UB*)"[TEST] S3-U04 info: %d samples in 500 ms, loops %d,"
			  " overruns %d, read errors %d, task stat 0x%x (er %d),"
			  " cyc stat 0x%x (er %d)\n",
			  (INT)(st2.seq - st1.seq), (INT)s3_line_wakeups,
			  (INT)s3_line_overruns, (INT)s3_line_read_errors,
			  (INT)rt.tskstat, (INT)e1, (INT)rc.cycstat, (INT)e2);
	}
	TEST_CHECK("S3-U04", (st2.seq - st1.seq) >= 45 && (st2.seq - st1.seq) <= 55,
		   "line task samples every 10 ms");

	/* S3-U05: barcode_task decodes runs fed into mbf_s3_bc_runs and
	   reports BARCODE_START then BARCODE_DECODED(NAV_LEFT) on IF-32 */
	{
		S3_BC_RUN	run;

		/* The Vehicle Controller also reads mbf_s3_events (vc_bridge_s3.c)
		   and would take these events first: pause it for this test. */
		(void)tk_sus_tsk(tid_vc);
		drain_events();
		n = code39_encode("L", 20000, 50000, w, S3_BC_MAX_ELEMENTS);
		run.black = 0; run.width_us = 500000;	/* leading quiet zone */
		tk_snd_mbf(mbf_s3_bc_runs, &run, sizeof(run), TMO_FEVR);
		for (i = 0; i < n; i++) {
			run.black = (i % 2 == 0) ? 1 : 0;
			run.width_us = w[i];
			tk_snd_mbf(mbf_s3_bc_runs, &run, sizeof(run), TMO_FEVR);
		}
		TEST_CHECK("S3-U05a", wait_event(S3_EVT_BARCODE_START, &evt, 500),
			   "BARCODE_START event sent");
		TEST_CHECK("S3-U05b",
			   wait_event(S3_EVT_BARCODE_DECODED, &evt, 1000) &&
			   evt.nav == NAV_LEFT && evt.text[0] == 'L',
			   "BARCODE_DECODED event with NAV_LEFT");
		drain_events();
		(void)tk_rsm_tsk(tid_vc);
	}

	/* S3-U06 (manual, on the track): see TEST_LOG.md.  Print the live
	   status for 2 s so a person can move the robot over the line. */
	for (i = 0; i < 10; i++) {
		s3_get_line_status(&st1);
		tm_printf((UB*)"[S3] state=%d pos=%d steer=%d mask=%x raw=%d,%d,%d\n",
			  st1.line_state, st1.position, st1.steering,
			  st1.black_mask, st1.raw[0], st1.raw[1], st1.raw[2]);
		tk_dly_tsk(200);
	}

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
