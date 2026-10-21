/*
 *	subsystem5.c
 *	Subsystem 5 -- Ultrasonic Obstacle Detection and Avoidance.
 *	Creates the kernel objects, starts the two tasks, and keeps the
 *	status snapshot other subsystems read with s5_get_status().
 *
 *	Tasks (design review section 3):
 *	  s5_scan_task   Obstacle Detection & Scanning.  Owns the ultrasonic
 *	                 sensor and the servo.  Pings every 20 ms, and runs
 *	                 the coarse + fine scan when asked.
 *	  s5_avoid_task  Obstacle Avoidance.  Receives the controller's
 *	                 commands, plans the bypass, sends motion requests,
 *	                 and reacquires the line (bounded retries).
 *
 *	Kernel objects:
 *	  mbf_s5_cmd   message buffer  VC -> S5        (public)
 *	  mbf_s5_evt   message buffer  S5 -> VC        (public)
 *	  flg_s5       event flag      scan <-> avoid  (private)
 *	  mtx_s5       mutex           guards s5_status (private)
 */

#include "s5_local.h"

EXPORT ID	mbf_s5_cmd;
EXPORT ID	mbf_s5_evt;
EXPORT ID	flg_s5;
EXPORT ID	mtx_s5;
EXPORT ID	tid_s5_scan;
EXPORT ID	tid_s5_avoid;
EXPORT UW	s5_evt_dropped;

LOCAL S5_STATUS	s5_status;

EXPORT ER subsystem5_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};

	/* Each message in a message buffer also costs a small header, so
	   allow a few extra bytes per message. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.maxmsz = sizeof(S5_CMD);
	cmbf.bufsz  = S5_CMD_BUF_MSGS * (sizeof(S5_CMD) + 4);
	mbf_s5_cmd = tk_cre_mbf(&cmbf);
	if (mbf_s5_cmd < E_OK) {
		return mbf_s5_cmd;
	}

	cmbf.maxmsz = sizeof(S5_EVT);
	cmbf.bufsz  = S5_EVT_BUF_MSGS * (sizeof(S5_EVT) + 4);
	mbf_s5_evt = tk_cre_mbf(&cmbf);
	if (mbf_s5_evt < E_OK) {
		return mbf_s5_evt;
	}

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = S5_ENABLE_AT_START ? S5_FLG_ENABLED : 0;
	flg_s5 = tk_cre_flg(&cflg);
	if (flg_s5 < E_OK) {
		return flg_s5;
	}

	/* Priority inheritance: a low-priority reader (telemetry) holding
	   the lock cannot hold up the S5 tasks for long. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_s5 = tk_cre_mtx(&cmtx);
	if (mtx_s5 < E_OK) {
		return mtx_s5;
	}

	s5_status = (S5_STATUS){0};
	s5_status.state = S5_ENABLE_AT_START ? S5_ST_MONITORING : S5_ST_IDLE;
	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
	/* No core affinity yet.  Pin the scan task (busy-waits on the echo)
	   to one core with TP_PRC1/TP_PRC2 once the team agrees core use. */
	tid_s5_avoid = app_start_task(s5_avoid_task, S5_PRI_AVOID, 0,
				      "s5_avoid_task");
	if (tid_s5_avoid < E_OK) {
		return tid_s5_avoid;
	}
	tid_s5_scan = app_start_task(s5_scan_task, S5_PRI_SCAN, 0,
				     "s5_scan_task");
	return (tid_s5_scan < E_OK) ? tid_s5_scan : E_OK;
}

/* ------------------------------------------------------------------ */
/*  Status snapshot                                                    */
/* ------------------------------------------------------------------ */

EXPORT ER s5_get_status(S5_STATUS *out)
{
	ER er;

	if (out == NULL) {
		return E_PAR;
	}
	er = tk_loc_mtx(mtx_s5, 100);
	if (er < E_OK) {
		return er;
	}
	*out = s5_status;
	tk_unl_mtx(mtx_s5);
	return E_OK;
}

#define S5_LOCKED(stmt)						\
	do {							\
		if (tk_loc_mtx(mtx_s5, TMO_FEVR) == E_OK) {	\
			stmt;					\
			tk_unl_mtx(mtx_s5);			\
		}						\
	} while (0)

EXPORT void s5_set_state(S5_STATE st)
{
	S5_LOCKED(s5_status.state = (UB)st);
}

EXPORT void s5_set_fault(S5_FAULT_CODE f)
{
	S5_LOCKED(s5_status.last_fault = (UB)f);
}

EXPORT void s5_note_ping(UH mm)
{
	S5_LOCKED(
		s5_status.pings++;
		s5_status.last_distance_mm = mm;
		if (mm == 0) {
			s5_status.echo_timeouts++;
		}
	);
}

EXPORT void s5_count_obstacle(void)
{
	S5_LOCKED(s5_status.obstacles++);
}

EXPORT void s5_count_avoid_ok(void)
{
	S5_LOCKED(s5_status.avoid_ok++);
}

EXPORT void s5_store_profile(const S5_PROFILE *p)
{
	S5_LOCKED(s5_status.profile = *p);
}

EXPORT void s5_load_profile(S5_PROFILE *p)
{
	S5_LOCKED(*p = s5_status.profile);
}

/* ------------------------------------------------------------------ */
/*  Events to the Vehicle Controller                                   */
/* ------------------------------------------------------------------ */

EXPORT ER s5_send_evt(S5_EVT *e)
{
	SYSTIM	now;
	ER	er;

	tk_get_tim(&now);
	e->time_ms = now.lo;

	/* Never block: if the controller is not reading, drop and count */
	er = tk_snd_mbf(mbf_s5_evt, e, sizeof(*e), TMO_POL);
	if (er < E_OK) {
		s5_evt_dropped++;
	}
	return er;
}

EXPORT const char *s5_state_name(S5_STATE st)
{
	switch (st) {
	case S5_ST_IDLE:		return "IDLE";
	case S5_ST_MONITORING:		return "MONITORING";
	case S5_ST_DETECTED:		return "DETECTED";
	case S5_ST_COARSE_SCAN:		return "COARSE_SCAN";
	case S5_ST_FINE_SCAN:		return "FINE_SCAN";
	case S5_ST_PLANNING:		return "PLANNING";
	case S5_ST_BYPASSING:		return "BYPASSING";
	case S5_ST_SEARCH_LINE:		return "SEARCH_LINE";
	case S5_ST_LINE_RECOVERY:	return "LINE_RECOVERY";
	case S5_ST_ERROR:		return "ERROR";
	default:			return "?";
	}
}
