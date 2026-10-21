/*
 *	subsystem3.c
 *	Subsystem 3 -- IR Line Following and Barcode Navigation: creates its
 *	objects, starts its tasks, and implements the public calls of
 *	subsystem3.h (IF-31, IF-32, IF-33).
 *
 *	Kernel objects (TEAM.md section 6):
 *	  2 tasks        line_task, barcode_task
 *	  1 cyclic       cyc_s3_line      wakes line_task every 10 ms
 *	  1 mutex        mtx_s3_status    guards s3_status (IF-31)
 *	  2 msg buffers  mbf_s3_events    events to the controller (IF-32)
 *	                 mbf_s3_bc_runs   barcode runs, line -> barcode task
 *	  1 event flag   flg_s3_req       requests (IF-33)
 */

#include "subsystem3.h"

EXPORT ID		mbf_s3_events = E_NOEXS;	/* IF-32 */
EXPORT ID		mtx_s3_status = E_NOEXS;	/* IF-31 */
EXPORT ID		mbf_s3_bc_runs = E_NOEXS;
EXPORT ID		flg_s3_req = E_NOEXS;		/* IF-33 */
EXPORT ID		tid_s3_line = E_NOEXS;
EXPORT ID		tid_s3_barcode = E_NOEXS;
EXPORT ID		cyc_s3_line = E_NOEXS;
EXPORT S3_LINE_STATUS	s3_status;
EXPORT volatile BOOL	s3_bc_scanning = FALSE;
EXPORT volatile UB	s3_black_mask_live = 0;

#define S3_BC_RUN_QUEUE_LEN	(S3_BC_MAX_ELEMENTS + 4)

EXPORT ER subsystem3_init(void)
{
	T_CMTX	cmtx = {0};
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};

	s3_status.line_state = LINE_STATE_INIT;
	s3_status.bc_state   = BARCODE_STATE_IDLE;
	s3_status.bc_enabled = TRUE;

	/* Mutex with priority inheritance: the 10 ms line task must not be
	   held up by a low-priority reader (e.g. telemetry) */
	cmtx.mtxatr = TA_INHERIT;
	mtx_s3_status = tk_cre_mtx(&cmtx);
	if (mtx_s3_status < E_OK) {
		return mtx_s3_status;
	}

	/* IF-32: fixed-size events, copied into the buffer */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S3_EVT_QUEUE_LEN * (sizeof(S3_EVENT) + sizeof(INT));
	cmbf.maxmsz = sizeof(S3_EVENT);
	mbf_s3_events = tk_cre_mbf(&cmbf);
	if (mbf_s3_events < E_OK) {
		return mbf_s3_events;
	}

	/* Barcode runs: enough for one whole barcode plus margin */
	cmbf.bufsz  = S3_BC_RUN_QUEUE_LEN * (sizeof(S3_BC_RUN) + sizeof(INT));
	cmbf.maxmsz = sizeof(S3_BC_RUN);
	mbf_s3_bc_runs = tk_cre_mbf(&cmbf);
	if (mbf_s3_bc_runs < E_OK) {
		return mbf_s3_bc_runs;
	}

	/* IF-33: TA_WMUL so both tasks may wait on their own bits;
	   each task clears only the bits it handles */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_s3_req = tk_cre_flg(&cflg);
	if (flg_s3_req < E_OK) {
		return flg_s3_req;
	}

	/* Cyclic handler: fixed 10 ms sampling, created stopped */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)s3_line_cyclic;
	ccyc.cyctim = S3_LINE_PERIOD_MS;
	ccyc.cycphs = S3_LINE_PERIOD_MS;
	cyc_s3_line = tk_cre_cyc(&ccyc);
	return (cyc_s3_line < E_OK) ? cyc_s3_line : E_OK;
}

EXPORT ER subsystem3_start(void)
{
	/* Both on core 1 (TP_PRC2): line_task produces barcode runs that
	   barcode_task consumes; keeping them on one core keeps their
	   order simple.  Not core 0: the USB console (pri 3) and CYW43
	   (pri 4) service tasks are pinned there, and on the bench they
	   starved line_task (READY, 90 loops in ~15 s).  Core 1 holds only
	   motion_task (14) and imu_task (18).  TEAM.md section 4. */
	tid_s3_barcode = app_start_task(barcode_task, S3_PRI_BARCODE, TP_PRC2,
					"barcode_task");
	if (tid_s3_barcode < E_OK) {
		return tid_s3_barcode;
	}

	tid_s3_line = app_start_task(line_task, S3_PRI_LINE, TP_PRC2,
				     "line_task");
	if (tid_s3_line < E_OK) {
		return tid_s3_line;
	}

	return tk_sta_cyc(cyc_s3_line);
}

/* IF-31 */
EXPORT ER s3_get_line_status(S3_LINE_STATUS *p_out)
{
	ER	er;

	if (p_out == NULL) {
		return E_PAR;
	}
	er = tk_loc_mtx(mtx_s3_status, TMO_FEVR);
	if (er < E_OK) {
		return er;
	}
	*p_out = s3_status;
	return tk_unl_mtx(mtx_s3_status);
}

/* IF-33 */
EXPORT ER s3_request(UINT req)
{
	if ((req & ~(UINT)(S3_FLG_LINE_REQ | S3_FLG_BC_REQ)) != 0) {
		return E_PAR;
	}
	return tk_set_flg(flg_s3_req, req);
}

/* IF-32: never blocks.  A full buffer drops the event and counts it. */
EXPORT void s3_send_event(S3_EVENT *p_evt)
{
	SYSTIM	now;
	ER	er;

	tk_get_tim(&now);
	p_evt->time_ms = now.lo;

	er = tk_snd_mbf(mbf_s3_events, p_evt, sizeof(S3_EVENT), TMO_POL);
	if (er != E_OK) {
		tk_loc_mtx(mtx_s3_status, TMO_FEVR);
		s3_status.evt_dropped++;
		tk_unl_mtx(mtx_s3_status);
	}
}

/* Barcode text -> navigation command.  PLACEHOLDER mapping in
   s3_config.h; only the first character is used. */
EXPORT NavigationCommand s3_text_to_nav(const UB *p_text)
{
	switch (p_text[0]) {
	case S3_BC_CHAR_LEFT:		return NAV_LEFT;
	case S3_BC_CHAR_RIGHT:		return NAV_RIGHT;
	case S3_BC_CHAR_STRAIGHT:	return NAV_STRAIGHT;
	case S3_BC_CHAR_UTURN:		return NAV_U_TURN;
	default:			return NAV_NONE;
	}
}
