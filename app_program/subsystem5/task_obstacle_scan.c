/*
 *	task_obstacle_scan.c
 *	Subsystem 5 -- Obstacle Detection & Scanning task.
 *
 *	Owns: ultrasonic sensor (GP16/GP17) and scan servo (GP12, PWM 6).
 *
 *	Modes (Statechart 6.3):
 *	  MONITOR  servo straight ahead, one ping every 20 ms.  S5_DETECT_COUNT
 *	           pings in a row under S5_DETECT_MM -> OBSTACLE_DETECTED
 *	           to the controller, then HOLD.
 *	  HOLD     wait for the avoid task: SCAN_REQ (scan) or RESUME.
 *	  SCAN     coarse scan, profile, fine scan around the edges,
 *	           profile again, hand it over (S5_FLG_PROFILE), HOLD.
 *	Monitoring pauses while S5_FLG_ENABLED is clear.
 */

#include "s5_local.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#define SCAN_MAX_SAMPLES	40
#define SCAN_READS		3	/* median of 3 per angle */
#define SCAN_READ_GAP_MS	15

typedef enum { MODE_MONITOR, MODE_HOLD } SCAN_MODE;

LOCAL UW	silent_in_scan;		/* consecutive no-echo during scans */

LOCAL void point_and_wait(INT deg)
{
	INT from = servo_get_deg();

	servo_set_deg(deg);
	tk_dly_tsk(servo_settle_ms(from, deg));
}

/* Sweep the given angles, one median reading each. */
LOCAL INT sweep(const B *deg, INT n, S5_SAMPLE *out)
{
	INT i;
	UH  mm;

	for (i = 0; i < n; i++) {
		point_and_wait(deg[i]);
		mm = us_read_median_mm(SCAN_READS, SCAN_READ_GAP_MS);
		s5_note_ping(mm);
		silent_in_scan = (mm == US_NO_ECHO) ? silent_in_scan + 1 : 0;
		out[i].deg = deg[i];
		out[i].mm  = mm;
	}
	return n;
}

LOCAL void print_profile(const char *tag, const S5_PROFILE *p)
{
	tm_printf((UB*)"s5: %s d=%dmm edges R%d..L%d deg w=%dmm "
		  "clrL=%d clrR=%d side=%d\n", tag,
		  p->distance_mm, p->right_edge_deg, p->left_edge_deg,
		  p->width_mm, p->clear_left_mm, p->clear_right_mm,
		  p->bypass_side);
}

/* Coarse then fine scan.  Leaves the profile in the status snapshot
   (valid = 0 if nothing usable was found). */
LOCAL void do_scan(void)
{
	B		ang[SCAN_MAX_SAMPLES];
	S5_SAMPLE	smp[SCAN_MAX_SAMPLES];
	S5_PROFILE	prof;
	S5_PROFILE	coarse;
	INT		n;
	S5_EVT		e = {0};

	silent_in_scan = 0;

	/* 1. coarse */
	s5_set_state(S5_ST_COARSE_SCAN);
	n = s5_coarse_angles(ang, SCAN_MAX_SAMPLES);
	n = sweep(ang, n, smp);
	if (!s5_build_profile(smp, n, &prof)) {
		tm_printf((UB*)"s5: coarse scan found nothing\n");
	} else {
		print_profile("coarse", &prof);
		coarse = prof;

		/* 2. fine, around the edges seen by the coarse scan */
		s5_set_state(S5_ST_FINE_SCAN);
		n = s5_fine_angles(&prof, ang, SCAN_MAX_SAMPLES);
		n = sweep(ang, n, smp);
		if (s5_build_profile(smp, n, &prof)) {
			print_profile("fine", &prof);
		} else {
			/* moved away or a bad read: keep the coarse profile */
			tm_printf((UB*)"s5: fine scan lost it, using coarse\n");
			prof = coarse;
		}
	}

	if (silent_in_scan >= S5_ECHO_FAULT_COUNT) {
		s5_set_fault(S5_FAULT_ECHO_TIMEOUT);
		e.type = S5_EVT_FAULT;
		e.code = S5_FAULT_ECHO_TIMEOUT;
		s5_send_evt(&e);
	}

	point_and_wait(0);
	s5_store_profile(&prof);
}

EXPORT void s5_scan_task(INT stacd, void *exinf)
{
	SCAN_MODE	mode = MODE_MONITOR;
	INT		near_count = 0;
	UINT		flg;
	SYSTIM		t0;
	SYSTIM		t1;
	INT		used;
	UH		mm;
	S5_EVT		e;

	(void)stacd; (void)exinf;

	CHECK(us_init(), "s5 us_init");
	CHECK(servo_init(), "s5 servo_init");
	tm_printf((UB*)"s5: scan task %d up (bench mode %d)\n",
		  tk_get_tid(), S5_BENCH_MODE);

	while (1) {
		/* Requests from the avoid task come first, in any mode */
		if (tk_wai_flg(flg_s5, S5_FLG_SCAN_REQ, TWF_ORW | TWF_BITCLR,
			       &flg, TMO_POL) == E_OK) {
			do_scan();
			tk_set_flg(flg_s5, S5_FLG_PROFILE);
			mode = MODE_HOLD;
			continue;
		}
		if (tk_wai_flg(flg_s5, S5_FLG_RESUME, TWF_ORW | TWF_BITCLR,
			       &flg, TMO_POL) == E_OK) {
			point_and_wait(0);
			near_count = 0;
			mode = MODE_MONITOR;
		}

		if (mode == MODE_HOLD) {
			/* sleep until the avoid task wants something */
			tk_wai_flg(flg_s5, S5_FLG_SCAN_REQ | S5_FLG_RESUME,
				   TWF_ORW, &flg, TMO_FEVR);
			continue;
		}

		/* MONITOR, but only while the controller allows it */
		if (tk_wai_flg(flg_s5, S5_FLG_ENABLED, TWF_ORW, &flg,
			       TMO_POL) != E_OK) {
			near_count = 0;
			servo_off();
			tk_wai_flg(flg_s5, S5_FLG_ENABLED | S5_FLG_SCAN_REQ,
				   TWF_ORW, &flg, TMO_FEVR);
			servo_set_deg(0);
			continue;
		}

		tk_get_tim(&t0);
		mm = us_read_mm();
		s5_note_ping(mm);

		if (mm != US_NO_ECHO && mm < S5_DETECT_MM) {
			near_count++;
		} else {
			near_count = 0;
		}

		if (near_count >= S5_DETECT_COUNT) {
			near_count = 0;
			mode = MODE_HOLD;
			s5_set_state(S5_ST_DETECTED);
			s5_count_obstacle();
			tm_printf((UB*)"s5: obstacle at %d mm\n", mm);

			e = (S5_EVT){0};
			e.type = S5_EVT_OBSTACLE_DETECTED;
			e.distance_mm = mm;
			s5_send_evt(&e);
#if S5_BENCH_MODE
			{	/* play the controller: start avoiding at once */
				S5_CMD c = { S5_CMD_START_AVOID, 0, 0 };
				tk_snd_mbf(mbf_s5_cmd, &c, sizeof(c), TMO_POL);
			}
#endif
			continue;
		}

		/* keep the 20 ms period (NFR2) */
		tk_get_tim(&t1);
		used = (INT)(t1.lo - t0.lo);
		if (used < S5_PERIOD_MS) {
			tk_dly_tsk(S5_PERIOD_MS - used);
		}
	}
}
