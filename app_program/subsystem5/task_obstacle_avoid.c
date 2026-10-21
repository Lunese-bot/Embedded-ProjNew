/*
 *	task_obstacle_avoid.c
 *	Subsystem 5 -- Obstacle Avoidance task.
 *
 *	Receives S5_CMD from the Vehicle Controller on mbf_s5_cmd.  On
 *	START_AVOID (the controller has stopped the car):
 *	  1. ask the scan task for a profile        (S5_FLG_SCAN_REQ)
 *	  2. choose a side and plan the bypass      (obstacle_logic.c)
 *	  3. send each step as S5_EVT_MOTION_REQ and wait for its
 *	     S5_CMD_MOTION_DONE; "search" steps stop early when the line
 *	     is seen (s5_port_line_detected)
 *	  4. line found  -> turn back to the original heading,
 *	                    S5_EVT_LINE_REACQUIRED
 *	     not found   -> up to S5_MAX_RECOVERY wider searches, then
 *	                    S5_EVT_AVOID_FAILED (the controller decides:
 *	                    retry, or Safe Stop, FR8)
 *	  5. tell the scan task to go back to monitoring (S5_FLG_RESUME)
 *	ABORT or DISABLE at any point ends the manoeuvre at once.
 */

#include "s5_local.h"

typedef enum {
	STEP_DONE,		/* motion finished                 */
	STEP_LINE,		/* search step stopped on the line */
	STEP_FAILED,		/* controller reported failure     */
	STEP_TIMEOUT,		/* no MOTION_DONE in time          */
	STEP_ABORT		/* ABORT / DISABLE received        */
} STEP_RESULT;

LOCAL UH	seq_next = 1;
LOCAL INT	heading_deg;	/* sum of completed turns since START */

LOCAL TMO step_timeout_ms(const S5_MOTION_REQ *m)
{
	if (m->kind == S5_MOVE_FORWARD || m->kind == S5_MOVE_REVERSE) {
		UW spd = (m->speed_mm_s > 0) ? m->speed_mm_s : 100;
		return (TMO)((m->distance_mm * 1000UL) / spd) + S5_MOVE_SLACK_MS;
	}
	return S5_TURN_TMO_MS;
}

/* Handle a command that arrives while we are busy.  Returns TRUE if it
   means "stop the manoeuvre". */
LOCAL BOOL busy_command(const S5_CMD *c)
{
	switch (c->type) {
	case S5_CMD_ABORT:
		return TRUE;
	case S5_CMD_DISABLE:
		tk_clr_flg(flg_s5, ~(UINT)S5_FLG_ENABLED);
		return TRUE;
	case S5_CMD_ENABLE:
		tk_set_flg(flg_s5, S5_FLG_ENABLED);
		return FALSE;
	default:		/* stale MOTION_DONE, repeated START_AVOID */
		return FALSE;
	}
}

LOCAL UH send_motion(const S5_MOTION_REQ *m)
{
	S5_EVT e = {0};

	e.type     = S5_EVT_MOTION_REQ;
	e.u.motion = *m;
	e.u.motion.seq = seq_next++;
	s5_send_evt(&e);
	return e.u.motion.seq;
}

/* Send one step and wait for it to finish. */
LOCAL STEP_RESULT run_step(const S5_STEP *st)
{
	S5_CMD	c;
	UH	seq;
	TMO	left = step_timeout_ms(&st->m);
	TMO	slice;
	INT	len;

	seq = send_motion(&st->m);

#if S5_BENCH_MODE
	tm_printf((UB*)"s5: [bench] move kind=%d ang=%d dist=%d spd=%d\n",
		  st->m.kind, st->m.angle_deg, st->m.distance_mm,
		  st->m.speed_mm_s);
	{	/* pretend the motion takes a while, then finishes */
		S5_CMD done = { S5_CMD_MOTION_DONE, 1, 0 };
		done.seq = seq;
		tk_dly_tsk(S5_BENCH_MOVE_MS);
		tk_snd_mbf(mbf_s5_cmd, &done, sizeof(done), TMO_POL);
	}
#endif

	while (left > 0) {
		/* search steps poll the line sensors every period */
		slice = st->search_line ? S5_PERIOD_MS : left;
		if (slice > left) {
			slice = left;
		}
		len = tk_rcv_mbf(mbf_s5_cmd, &c, slice);
		left -= slice;

		if (len > 0) {
			if (c.type == S5_CMD_MOTION_DONE && c.seq == seq) {
				if (!c.ok) {
					return STEP_FAILED;
				}
				if (st->m.kind == S5_MOVE_TURN) {
					heading_deg += st->m.angle_deg;
				}
				/* a search that ran its full length may still
				   have ended on the line */
				if (st->search_line && s5_port_line_detected()) {
					return STEP_LINE;
				}
				return STEP_DONE;
			}
			if (busy_command(&c)) {
				return STEP_ABORT;
			}
			continue;
		}
		if (st->search_line && s5_port_line_detected()) {
			S5_STEP stop = {0};
			stop.m.kind = S5_MOVE_STOP;
			(void)send_motion(&stop.m);
			return STEP_LINE;
		}
	}
	return STEP_TIMEOUT;
}

/* Run a plan.  Stops at the first result other than STEP_DONE. */
LOCAL STEP_RESULT run_plan(const S5_STEP *plan, INT n)
{
	INT		i;
	STEP_RESULT	r = STEP_DONE;

	for (i = 0; i < n; i++) {
		if (plan[i].search_line) {
			s5_set_state(S5_ST_SEARCH_LINE);
		}
		r = run_step(&plan[i]);
		if (r != STEP_DONE) {
			break;
		}
	}
	return r;
}

LOCAL void report(S5_EVT_TYPE type, S5_FAULT_CODE code)
{
	S5_EVT e = {0};

	e.type = (UB)type;
	e.code = (UB)code;
	s5_send_evt(&e);
	if (code != S5_FAULT_NONE) {
		s5_set_fault(code);
	}
}

/* Turn back to the heading the car had before the bypass. */
LOCAL STEP_RESULT realign(void)
{
	S5_STEP st = {0};

	if (heading_deg == 0) {
		return STEP_DONE;
	}
	st.m.kind = S5_MOVE_TURN;
	st.m.angle_deg = (B)(-heading_deg);
	return run_step(&st);
}

LOCAL void avoid_obstacle(void)
{
	S5_STEP		plan[S5_PLAN_MAX];
	S5_PROFILE	prof;
	S5_EVT		e = {0};
	UINT		flg;
	INT		n;
	INT		k;
	STEP_RESULT	r;

	heading_deg = 0;

	/* 1. profile */
	tk_clr_flg(flg_s5, ~(UINT)S5_FLG_PROFILE);
	tk_set_flg(flg_s5, S5_FLG_SCAN_REQ);
	if (tk_wai_flg(flg_s5, S5_FLG_PROFILE, TWF_ORW | TWF_BITCLR, &flg,
		       S5_SCAN_TMO_MS) != E_OK) {
		report(S5_EVT_FAULT, S5_FAULT_NO_PROFILE);
		goto done;
	}
	s5_load_profile(&prof);
	e.type = S5_EVT_PROFILE_READY;
	e.u.profile = prof;
	s5_send_evt(&e);
	if (!prof.valid) {
		/* obstacle gone (moved, or a false alarm): carry on */
		tm_printf((UB*)"s5: no obstacle in scan, resuming\n");
		report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
		goto done;
	}

	/* 2. plan */
	s5_set_state(S5_ST_PLANNING);
	n = s5_plan_bypass(&prof, plan, S5_PLAN_MAX);
	if (prof.bypass_side == S5_SIDE_NONE) {
		(void)run_plan(plan, n);	/* stop + back off */
		report(S5_EVT_AVOID_FAILED, S5_FAULT_BLOCKED);
		goto done;
	}

	/* 3. bypass */
	s5_set_state(S5_ST_BYPASSING);
	r = run_plan(plan, n);

	/* 4. bounded line recovery */
	for (k = 0; r == STEP_DONE && k < S5_MAX_RECOVERY; k++) {
		s5_set_state(S5_ST_LINE_RECOVERY);
		tm_printf((UB*)"s5: line not found, recovery %d/%d\n",
			  k + 1, S5_MAX_RECOVERY);
		n = s5_plan_recovery((S5_SIDE)prof.bypass_side, k, plan,
				     S5_PLAN_MAX);
		r = run_plan(plan, n);
	}

	switch (r) {
	case STEP_LINE:
		if (realign() == STEP_ABORT) {
			break;
		}
		s5_count_avoid_ok();
		tm_printf((UB*)"s5: line reacquired\n");
		report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
		break;
	case STEP_DONE:			/* every recovery ran out */
		report(S5_EVT_AVOID_FAILED, S5_FAULT_NONE);
		break;
	case STEP_TIMEOUT:
		report(S5_EVT_FAULT, S5_FAULT_MOTION_TIMEOUT);
		break;
	case STEP_FAILED:
		report(S5_EVT_AVOID_FAILED, S5_FAULT_NONE);
		break;
	case STEP_ABORT:
	default:
		tm_printf((UB*)"s5: avoidance aborted\n");
		break;
	}

done:
	s5_set_state((tk_wai_flg(flg_s5, S5_FLG_ENABLED, TWF_ORW, &flg,
				 TMO_POL) == E_OK)
		     ? S5_ST_MONITORING : S5_ST_IDLE);
	tk_set_flg(flg_s5, S5_FLG_RESUME);
}

EXPORT void s5_avoid_task(INT stacd, void *exinf)
{
	S5_CMD	c;
	INT	len;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"s5: avoid task %d up\n", tk_get_tid());

	while (1) {
		len = tk_rcv_mbf(mbf_s5_cmd, &c, TMO_FEVR);
		if (len < (INT)sizeof(c)) {
			CHECK(len, "s5 rcv cmd");
			continue;
		}
		switch (c.type) {
		case S5_CMD_ENABLE:
			tk_set_flg(flg_s5, S5_FLG_ENABLED);
			s5_set_state(S5_ST_MONITORING);
			break;
		case S5_CMD_DISABLE:
			tk_clr_flg(flg_s5, ~(UINT)S5_FLG_ENABLED);
			s5_set_state(S5_ST_IDLE);
			break;
		case S5_CMD_START_AVOID:
			avoid_obstacle();
			break;
		case S5_CMD_ABORT:
			/* controller cancels a detection it will not act on:
			   let the scan task monitor again */
			tk_set_flg(flg_s5, S5_FLG_RESUME);
			break;
		default:		/* stale MOTION_DONE: ignore */
			break;
		}
	}
}
